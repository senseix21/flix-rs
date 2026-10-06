#pragma once
#include <stdint.h>
int32_t flix_board_init(const int32_t *);
void flix_board_usb(const uint8_t *, uint32_t, uint64_t);
uint32_t flix_board_tick(float *);
uint32_t flix_board_reply(uint8_t *, uint32_t);
void flix_board_stop(void);
uint32_t flix_board_maintenance(void);
void flix_board_rc(const uint8_t *, uint64_t);
int32_t flix_board_radio(const uint8_t *, uint32_t, uint64_t);
uint32_t flix_board_sign_reply(uint8_t *, uint32_t, uint32_t, uint64_t);
uint64_t flix_board_radio_epoch(void);
uint32_t flix_board_metrics(uint64_t *, uint32_t);
void flix_radio_init(void);
void flix_radio_poll(uint64_t);
void flix_radio_send(const uint8_t *, uint32_t);
void flix_hal_stop(void);
void flix_espnow_poll(uint64_t);
void flix_espnow_send(const uint8_t *, uint32_t);
