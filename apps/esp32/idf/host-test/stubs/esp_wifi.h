#pragma once
/* Host stand-ins for the ESP-IDF Wi-Fi calls radio.c makes; every call succeeds. */
#include <stdint.h>
typedef int esp_err_t;
typedef const char *esp_event_base_t;
typedef void (*esp_event_handler_t)(void *, esp_event_base_t, int32_t, void *);
#define ESP_OK 0
#define ESP_EVENT_ANY_ID (-1)
extern const char *const WIFI_EVENT;
enum { WIFI_EVENT_STA_START = 2, WIFI_EVENT_STA_DISCONNECTED = 5 };
enum { WIFI_STORAGE_RAM = 1, WIFI_PS_NONE = 0, WIFI_MODE_STA = 1, WIFI_IF_STA = 0 };
enum { WIFI_AUTH_WPA2_PSK = 3 };
typedef struct { int unused; } wifi_init_config_t;
#define WIFI_INIT_CONFIG_DEFAULT() ((wifi_init_config_t){0})
typedef struct {
  struct {
    uint8_t ssid[32];
    uint8_t password[64];
    struct { int authmode; } threshold;
  } sta;
} wifi_config_t;
static inline esp_err_t esp_netif_init(void) { return ESP_OK; }
static inline esp_err_t esp_event_loop_create_default(void) { return ESP_OK; }
static inline void *esp_netif_create_default_wifi_sta(void) { return 0; }
static inline esp_err_t esp_wifi_init(const wifi_init_config_t *c) { (void)c; return ESP_OK; }
static inline esp_err_t esp_event_handler_register(esp_event_base_t b, int32_t i, esp_event_handler_t h, void *a) {
  (void)b; (void)i; (void)h; (void)a; return ESP_OK;
}
static inline esp_err_t esp_wifi_set_storage(int s) { (void)s; return ESP_OK; }
static inline esp_err_t esp_wifi_set_ps(int p) { (void)p; return ESP_OK; }
static inline esp_err_t esp_wifi_set_mode(int m) { (void)m; return ESP_OK; }
static inline esp_err_t esp_wifi_set_config(int i, wifi_config_t *c) { (void)i; (void)c; return ESP_OK; }
static inline esp_err_t esp_wifi_start(void) { return ESP_OK; }
static inline esp_err_t esp_wifi_connect(void) { return ESP_OK; }
