// Optional unqualified profile. Provisioning runs only in the owner during
// maintenance.
#include "board.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "sdkconfig.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdbool.h>
#include <string.h>
typedef struct {
  uint8_t mac[6];
  uint16_t length;
  uint8_t bytes[250];
} Packet;
static StaticQueue_t queue_state;
static uint8_t queue_storage[4 * sizeof(Packet)];
static QueueHandle_t ingress;
static uint8_t peers[4][6];
static unsigned peer_count;
static portMUX_TYPE peer_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE overflow_lock = portMUX_INITIALIZER_UNLOCKED;
static bool overflow = false;
static void received(const esp_now_recv_info_t *info, const uint8_t *bytes,
                     int length) {
  if (!ingress || !info || !bytes || length <= 0 || length > 250)
    return;
  if (!info->src_addr) return;
  bool known = false;
  portENTER_CRITICAL(&peer_lock);
  for (unsigned i = 0; i < peer_count; i++) known |= !memcmp(peers[i], info->src_addr, 6);
  portEXIT_CRITICAL(&peer_lock);
  if (!known) return;
  Packet packet = {.length = (uint16_t)length};
  memcpy(packet.mac, info->src_addr, 6);
  memcpy(packet.bytes, bytes, length);
  if (xQueueSend(ingress, &packet, 0) != pdTRUE) {
    portENTER_CRITICAL(&overflow_lock);
    overflow = true;
    portEXIT_CRITICAL(&overflow_lock);
  }
}
int flix_espnow_provision(const uint8_t pmk[16], uint8_t channel) {
  if (!flix_board_maintenance() || !pmk || channel == 0 || channel > 14 ||
      ingress)
    return -1;
  bool nonzero = false;
  for (int i = 0; i < 16; i++)
    nonzero |= pmk[i] != 0;
  if (!nonzero)
    return -1;
  if (esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) != ESP_OK ||
      esp_now_init() != ESP_OK || esp_now_set_pmk(pmk) != ESP_OK)
    return -2;
  ingress = xQueueCreateStatic(4, sizeof(Packet), queue_storage, &queue_state);
  return esp_now_register_recv_cb(received) == ESP_OK ? 0 : -3;
}
int flix_espnow_add(const uint8_t mac[6], const uint8_t key[16],
                    uint8_t channel) {
  if (!flix_board_maintenance() || !ingress || !mac || !key || mac[0] & 1 ||
      peer_count == 4 || channel == 0 || channel > 14)
    return -1;
  bool valid_mac = false, valid_key = false;
  for (int i = 0; i < 6; i++)
    valid_mac |= mac[i] != 0;
  for (int i = 0; i < 16; i++)
    valid_key |= key[i] != 0;
  if (!valid_mac || !valid_key)
    return -1;
  for (unsigned i = 0; i < peer_count; i++)
    if (!memcmp(peers[i], mac, 6))
      return -1;
  esp_now_peer_info_t peer = {0};
  memcpy(peer.peer_addr, mac, 6);
  memcpy(peer.lmk, key, 16);
  peer.channel = channel;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = true;
  if (esp_now_add_peer(&peer) != ESP_OK)
    return -2;
  portENTER_CRITICAL(&peer_lock);
  memcpy(peers[peer_count++], mac, 6);
  portEXIT_CRITICAL(&peer_lock);
  return 0;
}
// Exactly one attempt per boot, only after explicit stopped Maintenance.
// Transport keys persist in private NVS; MAVLink session enrollment remains
// direct wired, per boot, and is never loaded from this namespace.
static void provision_saved(void) {
#ifdef CONFIG_FLIX_ESPNOW_ENABLE
  static bool attempted;
  if (attempted || ingress || !flix_board_maintenance()) return;
  attempted = true;
  nvs_handle_t nvs;
  if (nvs_open("flixespnow", NVS_READONLY, &nvs) != ESP_OK) return;
  uint8_t mac[6], pmk[16], lmk[16], channel;
  size_t mac_len=6, pmk_len=16, lmk_len=16;
  bool valid = nvs_get_blob(nvs,"peer",mac,&mac_len)==ESP_OK && mac_len==6 &&
      nvs_get_blob(nvs,"pmk",pmk,&pmk_len)==ESP_OK && pmk_len==16 &&
      nvs_get_blob(nvs,"lmk",lmk,&lmk_len)==ESP_OK && lmk_len==16 &&
      nvs_get_u8(nvs,"channel",&channel)==ESP_OK;
  nvs_close(nvs);
  if (valid && mac[0]%2==0 && flix_espnow_provision(pmk,channel)==0)
    flix_espnow_add(mac,lmk,channel);
  memset(pmk,0,sizeof(pmk));memset(lmk,0,sizeof(lmk));
#endif
}
void flix_espnow_poll(uint64_t now) {
  provision_saved();
  if (!ingress)
    return;
  portENTER_CRITICAL(&overflow_lock);
  bool lost = overflow;
  overflow = false;
  portEXIT_CRITICAL(&overflow_lock);
  if (lost) {
    flix_board_stop();
    xQueueReset(ingress);
    return;
  }
  Packet packet;
  for (int n = 0; n < 4 && xQueueReceive(ingress, &packet, 0) == pdTRUE; n++) {
    bool known = false;
    for (unsigned i = 0; i < peer_count; i++)
      known |= !memcmp(peers[i], packet.mac, 6);
    if (known)
      flix_board_radio(packet.bytes, packet.length, now);
  }
}
int flix_espnow_remove(const uint8_t mac[6]) {
  if (!flix_board_maintenance() || !mac || !ingress)
    return -1;
  for (unsigned i = 0; i < peer_count; i++) {
    if (!memcmp(peers[i], mac, 6)) {
      if (esp_now_del_peer(mac) != ESP_OK)
        return -2;
      portENTER_CRITICAL(&peer_lock);
      memmove(&peers[i], &peers[i + 1], (peer_count - i - 1) * 6);
      memset(peers[--peer_count], 0, 6);
      portEXIT_CRITICAL(&peer_lock);
      return 0;
    }
  }
  return -3;
}
void flix_espnow_send(const uint8_t *bytes, uint32_t length) {
  if (!ingress || length > 250)
    return;
  for (unsigned i = 0; i < peer_count; i++)
    esp_now_send(peers[i], bytes, length);
}
