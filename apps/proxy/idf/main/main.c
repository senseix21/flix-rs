// SDK-only IO handoffs. The Rust bridge is called exclusively by app_main.
#include "driver/usb_serial_jtag.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
extern int32_t flix_proxy_init(const uint8_t *);
extern int32_t flix_proxy_input(const uint8_t *, uint32_t, uint64_t);
extern int32_t flix_proxy_received(const uint8_t *, const uint8_t *, uint32_t, uint64_t);
extern uint32_t flix_proxy_output(uint8_t *, bool, uint64_t);
extern void flix_proxy_reset(void);
typedef struct { uint16_t length; uint8_t bytes[250]; } Packet;
static StaticQueue_t queue_state;
static uint8_t queue_storage[4 * sizeof(Packet)];
static QueueHandle_t ingress;
static uint8_t peer_mac[6]; // Immutable after callbacks are registered.
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static bool overflow, send_done;
static esp_now_send_status_t send_status;
static void received(const esp_now_recv_info_t *info, const uint8_t *bytes, int length) {
  if (!info || !info->src_addr || !bytes || length <= 0 || length > 250 ||
      memcmp(info->src_addr, peer_mac, 6)) return;
  Packet packet = {.length = length};
  memcpy(packet.bytes, bytes, length);
  if (xQueueSend(ingress, &packet, 0) != pdTRUE) {
    portENTER_CRITICAL(&lock); overflow = true; portEXIT_CRITICAL(&lock);
  }
}
static void sent(const uint8_t *mac, esp_now_send_status_t status) {
  if (!mac || memcmp(mac, peer_mac, 6)) return;
  portENTER_CRITICAL(&lock); send_status = status; send_done = true; portEXIT_CRITICAL(&lock);
}
static bool blob(nvs_handle_t nvs, const char *key, uint8_t *out, size_t size) {
  size_t length = size;
  return nvs_get_blob(nvs, key, out, &length) == ESP_OK && length == size;
}
static bool nonzero(const uint8_t *bytes, size_t n) {
  for (size_t i=0; i<n; i++) if (bytes[i]) return true;
  return false;
}
void app_main(void) {
  // Absent/invalid provisioning leaves the bridge inactive, never discovers peers.
  nvs_handle_t nvs;
  if (nvs_flash_init() != ESP_OK || nvs_open("flixproxy", NVS_READONLY, &nvs) != ESP_OK) return;
  uint8_t pmk[16], lmk[16], channel;
  bool valid = blob(nvs,"peer",peer_mac,6) && blob(nvs,"pmk",pmk,16) &&
      blob(nvs,"lmk",lmk,16) && nvs_get_u8(nvs,"channel",&channel)==ESP_OK;
  nvs_close(nvs);
  if (!valid || !nonzero(pmk,16) || !nonzero(lmk,16) || channel < 1 || channel > 14 ||
      flix_proxy_init(peer_mac) != 0) return;
  ESP_ERROR_CHECK(esp_netif_init()); ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t wifi = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi)); ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_channel(channel,WIFI_SECOND_CHAN_NONE));
  ESP_ERROR_CHECK(esp_now_init()); ESP_ERROR_CHECK(esp_now_set_pmk(pmk));
  esp_now_peer_info_t peer = {0}; memcpy(peer.peer_addr,peer_mac,6); memcpy(peer.lmk,lmk,16);
  peer.channel=channel; peer.ifidx=WIFI_IF_STA; peer.encrypt=true;
  ESP_ERROR_CHECK(esp_now_add_peer(&peer)); memset(&peer,0,sizeof(peer)); memset(pmk,0,sizeof(pmk)); memset(lmk,0,sizeof(lmk));
  ingress=xQueueCreateStatic(4,sizeof(Packet),queue_storage,&queue_state);
  ESP_ERROR_CHECK(esp_now_register_recv_cb(received)); ESP_ERROR_CHECK(esp_now_register_send_cb(sent));
  usb_serial_jtag_driver_config_t usb={.rx_buffer_size=2240,.tx_buffer_size=512};
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
  bool connected=false, busy=false; uint64_t sent_at=0, pending_at=0;
  uint8_t input[1120], output[250]; uint32_t length=0, offset=0;
  for (;;) {
    uint64_t now=(uint64_t)esp_timer_get_time();
    bool online=usb_serial_jtag_is_connected();
    portENTER_CRITICAL(&lock);
    bool lost=overflow, done=send_done; esp_now_send_status_t status=send_status;
    overflow=false;send_done=false;
    portEXIT_CRITICAL(&lock);
    if (busy && done) {busy=false;if (status!=ESP_NOW_SEND_SUCCESS) lost=true;}
    // If send completion is lost, reboot rather than reuse an ambiguous callback.
    if (busy && now-sent_at>100000) {flix_proxy_reset();esp_restart();}
    if (online!=connected || lost) {
      flix_proxy_reset();xQueueReset(ingress);length=offset=0;connected=online;
      // Clear buffered USB data on cable transitions/overflow; never replay it.
      ESP_ERROR_CHECK(usb_serial_jtag_driver_uninstall());
      ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    }
    if (online) {
      int n=usb_serial_jtag_read_bytes(input,sizeof(input),0);
      if (n>0) flix_proxy_input(input,n,now);
      Packet packet;
      for (int i=0;i<4 && xQueueReceive(ingress,&packet,0)==pdTRUE;i++)
        flix_proxy_received(peer_mac,packet.bytes,packet.length,now);
      if (!busy) {
        uint8_t packet[250];uint32_t count=flix_proxy_output(packet,true,now);
        if (count) {
          if (esp_now_send(peer_mac,packet,count)==ESP_OK) {busy=true;sent_at=now;}
          else {flix_proxy_reset();}
        }
      }
      if (!length) {length=flix_proxy_output(output,false,now);offset=0;pending_at=now;}
      if (length) {
        if (now-pending_at>100000) {
          flix_proxy_reset();length=offset=0;
          ESP_ERROR_CHECK(usb_serial_jtag_driver_uninstall());
          ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
        } else {
          int count=usb_serial_jtag_write_bytes(output+offset,length-offset,0);
          if (count>0) offset+=count;
          if (offset==length) length=offset=0;
        }
      }
    } else {xQueueReset(ingress);}
    vTaskDelay(1);
  }
}
