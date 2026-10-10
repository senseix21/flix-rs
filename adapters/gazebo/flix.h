#ifndef FLIX_RUST_ABI_H
#define FLIX_RUST_ABI_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
  uint32_t version, calibrated;
  uint64_t now_us, sample_us;
  float gyro[3], acceleration[3];
} FlixInput;
/* Version 2 appends the MPU6050 range-limit mask: gyro x/y/z bits 0..2, accel
   x/y/z bits 3..5. Pass it to flix_tick2; flix_tick keeps accepting version 1. */
typedef struct {
  uint32_t version, calibrated;
  uint64_t now_us, sample_us;
  float gyro[3], acceleration[3];
  uint32_t saturation, reserved;
} FlixInput2;
typedef struct {
  uint32_t version, state, reason, reserved;
  uint64_t now_us, generation;
  float motors[4];
} FlixOutput;
typedef struct {
  uint32_t version, source, target_attitude_valid, reserved;
  float estimate[4], rates[3], target_rates[3], target_attitude[4];
} FlixObservation;
int32_t flix_observe(uint64_t, FlixObservation *);
uint64_t flix_create(void);
int32_t flix_destroy(uint64_t);
int32_t flix_reset(uint64_t);
int32_t flix_tick(uint64_t, const FlixInput *, FlixOutput *);
int32_t flix_tick2(uint64_t, const FlixInput2 *, FlixOutput *);
int32_t flix_receive(uint64_t, const uint8_t *, uint32_t, uint64_t);
int32_t flix_reply(uint64_t, uint8_t *, uint32_t);
#ifdef __cplusplus
}
#endif
#endif
