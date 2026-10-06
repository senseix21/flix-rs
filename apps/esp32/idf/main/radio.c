#include "board.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"
#include <fcntl.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
static int socket_fd = -1;
static struct sockaddr_in accepted_endpoint;
static bool have_endpoint = false;
static uint64_t session_epoch;
#ifdef CONFIG_FLIX_WIFI_ENABLE
static void connected(void *ctx, esp_event_base_t base, int32_t id,
                      void *data) {
  (void)ctx;
  (void)data;
  if (base == WIFI_EVENT &&
      (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED))
    esp_wifi_connect();
}
#endif
void flix_radio_init(void) {
#ifdef CONFIG_FLIX_ESPNOW_ENABLE
  if (esp_netif_init() != ESP_OK || esp_event_loop_create_default() != ESP_OK) return;
  wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
  if (esp_wifi_init(&init) != ESP_OK ||
      esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK ||
      esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK ||
      esp_wifi_start() != ESP_OK || esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) return;
#endif
#ifdef CONFIG_FLIX_WIFI_ENABLE
  if (esp_netif_init() != ESP_OK || esp_event_loop_create_default() != ESP_OK)
    return;
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
  if (esp_wifi_init(&init) != ESP_OK)
    return;
  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, connected, NULL);
  wifi_config_t config = {0};
  strlcpy((char *)config.sta.ssid, CONFIG_FLIX_WIFI_SSID,
          sizeof(config.sta.ssid));
  strlcpy((char *)config.sta.password, CONFIG_FLIX_WIFI_PASSWORD,
          sizeof(config.sta.password));
  config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  if (esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK ||
      esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK ||
      esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK ||
      esp_wifi_set_config(WIFI_IF_STA, &config) != ESP_OK ||
      esp_wifi_start() != ESP_OK)
    return;
  socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (socket_fd < 0)
    return;
  int flags = fcntl(socket_fd, F_GETFL, 0);
  if (flags < 0 || fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    close(socket_fd);
    socket_fd = -1;
    return;
  }
  struct sockaddr_in local = {.sin_family = AF_INET,
                              .sin_port = htons(14550),
                              .sin_addr.s_addr = htonl(INADDR_ANY)};
  if (bind(socket_fd, (struct sockaddr *)&local, sizeof(local)) < 0) {
    close(socket_fd);
    socket_fd = -1;
  }
#endif
}
void flix_radio_poll(uint64_t now) {
  const uint64_t epoch = flix_board_radio_epoch();
  if (epoch != session_epoch) {
    session_epoch = epoch;
    have_endpoint = false;
  }
  if (socket_fd < 0)
    return;
  for (int i = 0; i < 4; i++) {
    uint8_t bytes[281];
    struct sockaddr_in source;
    socklen_t size = sizeof(source);
    int count = recvfrom(socket_fd, bytes, sizeof(bytes), MSG_DONTWAIT,
                         (struct sockaddr *)&source, &size);
    if (count <= 0)
      break;
    if (count <= 280 && flix_board_radio(bytes, (uint32_t)count, now) == 0) {
      accepted_endpoint = source;
      have_endpoint = true;
    }
  }
}
void flix_radio_send(const uint8_t *bytes, uint32_t count) {
  if (socket_fd >= 0 && have_endpoint && count <= 280)
    sendto(socket_fd, bytes, count, MSG_DONTWAIT,
           (struct sockaddr *)&accepted_endpoint, sizeof(accepted_endpoint));
}
