// SDK boundary: startup, bounded I2C, PWM gate, storage, UART; flight decisions
// live in Rust.
#include "board.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <math.h>
#include <string.h>
static i2c_master_dev_handle_t sensor;
static nvs_handle_t storage;
static bool storage_open = false, outputs = false;
static portMUX_TYPE output_lock = portMUX_INITIALIZER_UNLOCKED;
static uint64_t deadline = 0;
static TaskHandle_t owner_task;
static bool rc_ready = false;
static uint8_t sbus[25];
static unsigned sbus_length = 0;
uint64_t flix_hal_now(void) { return (uint64_t)esp_timer_get_time(); }
void flix_hal_stop(void) {
  portENTER_CRITICAL(&output_lock);
  deadline = 0;
  if (outputs)
    gpio_set_level(CONFIG_FLIX_GATE_PIN, 0);
  portEXIT_CRITICAL(&output_lock);
}
static void expiry(void *unused) {
  (void)unused;
  portENTER_CRITICAL(&output_lock);
  if (outputs && (!deadline || flix_hal_now() >= deadline))
    gpio_set_level(CONFIG_FLIX_GATE_PIN, 0);
  portEXIT_CRITICAL(&output_lock);
}
static bool pin_ok(int pin) {
  return GPIO_IS_VALID_OUTPUT_GPIO(pin) && pin != 19 && pin != 20 &&
         !(pin >= 26 && pin <= 37) && pin != 0 && pin != 43 && pin != 44 &&
         pin != 45 && pin != 46;
}
static void init_outputs(void) {
#ifdef CONFIG_FLIX_PWM_ENABLE
  const int pins[7] = {CONFIG_FLIX_MOTOR_RL, CONFIG_FLIX_MOTOR_RR,
                       CONFIG_FLIX_MOTOR_FR, CONFIG_FLIX_MOTOR_FL,
                       CONFIG_FLIX_GATE_PIN, CONFIG_FLIX_I2C_SDA,
                       CONFIG_FLIX_I2C_SCL};
  for (int i = 0; i < 7; i++) {
    if (!pin_ok(pins[i]) || pins[i] == CONFIG_FLIX_RC_PIN)
      return;
    for (int j = 0; j < i; j++)
      if (pins[i] == pins[j])
        return;
  }
  gpio_config_t gate = {.pin_bit_mask = 1ULL << CONFIG_FLIX_GATE_PIN,
                        .mode = GPIO_MODE_OUTPUT,
                        .pull_down_en = GPIO_PULLDOWN_ENABLE};
  if (gpio_config(&gate) != ESP_OK)
    return;
  gpio_set_level(CONFIG_FLIX_GATE_PIN, 0);
  ledc_timer_config_t timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                               .duty_resolution = LEDC_TIMER_10_BIT,
                               .timer_num = LEDC_TIMER_0,
                               .freq_hz = 38000,
                               .clk_cfg = LEDC_AUTO_CLK};
  if (ledc_timer_config(&timer) != ESP_OK)
    return;
  for (int i = 0; i < 4; i++) {
    ledc_channel_config_t channel = {.gpio_num = pins[i],
                                     .speed_mode = LEDC_LOW_SPEED_MODE,
                                     .channel = (ledc_channel_t)i,
                                     .timer_sel = LEDC_TIMER_0,
                                     .duty = 0,
                                     .hpoint = 0};
    if (ledc_channel_config(&channel) != ESP_OK)
      return;
  }
  outputs = true;
#endif
}
static void commit(const float motors[4], uint64_t produced) {
  flix_hal_stop(); // gate closed while updating PWM registers
  if (!outputs || flix_hal_now() - produced >= 1000)
    return;
  bool active = false;
  for (int i = 0; i < 4; i++) {
    if (!isfinite(motors[i]) || motors[i] < 0 || motors[i] > 1)
      return;
    active |= motors[i] > 0;
  }
  for (int i = 0; i < 4; i++) {
    if (ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i,
                      (uint32_t)(motors[i] * 1023.0f)) != ESP_OK)
      return;
    if (ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i) != ESP_OK)
      return;
  }
  portENTER_CRITICAL(&output_lock);
  deadline = produced + 2000;
  if (active && flix_hal_now() < deadline)
    gpio_set_level(CONFIG_FLIX_GATE_PIN, 1);
  portEXIT_CRITICAL(&output_lock);
}
static void init_sensor(void) {
  if (!pin_ok(CONFIG_FLIX_I2C_SDA) || !pin_ok(CONFIG_FLIX_I2C_SCL) ||
      CONFIG_FLIX_I2C_SDA == CONFIG_FLIX_I2C_SCL)
    return;
  i2c_master_bus_config_t config = {.i2c_port = I2C_NUM_0,
                                    .sda_io_num = CONFIG_FLIX_I2C_SDA,
                                    .scl_io_num = CONFIG_FLIX_I2C_SCL,
                                    .clk_source = I2C_CLK_SRC_DEFAULT,
                                    .glitch_ignore_cnt = 7,
                                    .flags.enable_internal_pullup = true};
  i2c_master_bus_handle_t bus;
  if (i2c_new_master_bus(&config, &bus) != ESP_OK)
    return;
  i2c_device_config_t device = {.dev_addr_length = I2C_ADDR_BIT_LEN_7,
                                .device_address = 0x68,
                                .scl_speed_hz = 400000};
  if (i2c_master_bus_add_device(bus, &device, &sensor) != ESP_OK)
    sensor = NULL;
}
int32_t flix_hal_read(uint8_t reg, uint8_t *out, uint32_t len) {
  if (!sensor || !out || len > 14 || len == 0)
    return -1;
  // 1 ms transaction timeout. Driver/RTOS rounding must be measured on target.
  return i2c_master_transmit_receive(sensor, &reg, 1, out, len, 1);
}
int32_t flix_hal_write(uint8_t reg, uint8_t value) {
  if (!sensor)
    return -1;
  uint8_t data[2] = {reg, value};
  return i2c_master_transmit(sensor, data, 2, 1);
}
int32_t flix_hal_storage_read(uint32_t slot, uint8_t *out, uint32_t len) {
  if (!storage_open || slot > 1 || len != 404)
    return -1;
  size_t size = len;
  esp_err_t result =
      nvs_get_blob(storage, slot == 0 ? "config0" : "config1", out, &size);
  if (result == ESP_ERR_NVS_NOT_FOUND)
    return 1;
  return result == ESP_OK && size == len ? 0 : -1;
}
int32_t flix_hal_storage_write(uint32_t slot, const uint8_t *data,
                               uint32_t len) {
  if (!storage_open || slot > 1 || len != 404)
    return -1;
  flix_hal_stop();
  esp_err_t result =
      nvs_set_blob(storage, slot == 0 ? "config0" : "config1", data, len);
  return result == ESP_OK ? nvs_commit(storage) : result;
}
static void wake(void *unused) {
  (void)unused;
  if (owner_task)
    xTaskNotifyGive(owner_task);
}
static void flight_task(void *unused) {
  (void)unused;
  esp_task_wdt_add(NULL);
  uint8_t input[1120], reply[280];
  float motors[4];
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    const uint64_t start = flix_hal_now();
    int len = uart_read_bytes(UART_NUM_0, input, sizeof(input), 0);
    if (len > 0)
      flix_board_usb(input, (uint32_t)len, start);
    flix_radio_poll(start);
    flix_espnow_poll(start);
    if (rc_ready) {
      uint8_t bytes[100];
      int count = uart_read_bytes(UART_NUM_1, bytes, sizeof(bytes), 0);
      for (int i = 0; i < count; i++) {
        if (sbus_length == 0 && bytes[i] != 0x0f)
          continue;
        sbus[sbus_length++] = bytes[i];
        if (sbus_length == 25) {
          flix_board_rc(sbus, start);
          sbus_length = 0;
        }
      }
    }
    uint32_t state = flix_board_tick(motors);
    if (state == 3 || state == 2)
      commit(motors, start);
    else
      flix_hal_stop();
    for (int i = 0; i < 4; i++) {
      uint32_t count = flix_board_reply(reply, sizeof(reply));
      if (!count)
        break;
      // TX is best effort. Never wait for UART capacity in the control task.
      uart_tx_chars(UART_NUM_0, (const char *)reply, count);
      uint32_t signed_count = flix_board_sign_reply(reply, count, sizeof(reply), start);
      if (signed_count) {
        flix_radio_send(reply, signed_count);
        flix_espnow_send(reply, signed_count);
      }
    }
    if (flix_hal_now() - start > 1000)
      flix_hal_stop();
    esp_task_wdt_reset();
  }
}
void app_main(void) {
  init_outputs();
  flix_hal_stop();
  if (nvs_flash_init() == ESP_OK &&
      nvs_open("flixrust", NVS_READWRITE, &storage) == ESP_OK)
    storage_open = true;
  init_sensor();
  uart_config_t uart = {.baud_rate = 115200,
                        .data_bits = UART_DATA_8_BITS,
                        .parity = UART_PARITY_DISABLE,
                        .stop_bits = UART_STOP_BITS_1,
                        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                        .source_clk = UART_SCLK_DEFAULT};
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart));
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 2240, 2240, 0, NULL, 0));
  const int32_t pins[7] = {CONFIG_FLIX_I2C_SDA,  CONFIG_FLIX_I2C_SCL,
                           CONFIG_FLIX_MOTOR_RL, CONFIG_FLIX_MOTOR_RR,
                           CONFIG_FLIX_MOTOR_FR, CONFIG_FLIX_MOTOR_FL,
                           CONFIG_FLIX_RC_PIN};
  flix_board_init(pins);
  flix_radio_init();
  if (GPIO_IS_VALID_GPIO(CONFIG_FLIX_RC_PIN)) {
    uart_config_t rc = {.baud_rate = 100000,
                        .data_bits = UART_DATA_8_BITS,
                        .parity = UART_PARITY_EVEN,
                        .stop_bits = UART_STOP_BITS_2,
                        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                        .source_clk = UART_SCLK_DEFAULT};
    if (uart_param_config(UART_NUM_1, &rc) == ESP_OK &&
        uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, CONFIG_FLIX_RC_PIN,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_OK &&
        uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV) == ESP_OK &&
        uart_driver_install(UART_NUM_1, 200, 0, 0, NULL, 0) == ESP_OK)
      rc_ready = true;
  }
  esp_timer_handle_t expiry_timer, tick_timer;
  esp_timer_create_args_t e = {.callback = expiry, .name = "motor_expiry"},
                          t = {.callback = wake, .name = "flight_tick"};
  ESP_ERROR_CHECK(esp_timer_create(&e, &expiry_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(expiry_timer, 250));
  ESP_ERROR_CHECK(xTaskCreatePinnedToCore(flight_task, "flight", 8192, NULL, 20,
                                          &owner_task, 1) == pdPASS
                      ? ESP_OK
                      : ESP_FAIL);
  ESP_ERROR_CHECK(esp_timer_create(&t, &tick_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, 1000));
}
