/*
 * pose_stub.h — Fake pose provider for testing without the ESP32.
 *
 * During development, the ESP32 UART link doesn't exist yet.
 * pose_stub_get() returns a static pose at the origin (0, 0, 0°).
 *
 * When the ESP32 is ready, you replace calls to pose_stub_get() in
 * map_main.c with a UART read function. Everything else stays the same.
 * This is the benefit of keeping the pose source behind a function call
 * rather than accessing hardware directly from the grid update code.
 */

#ifndef POSE_STUB_H
#define POSE_STUB_H

#include "ipc_common.h"

/*
 * pose_stub_get — fill *out with the current simulated pose.
 *
 * Currently always returns (x=0, y=0, theta=0) — robot stationary
 * at origin, facing forward. Timestamp and sequence number are real.
 */
void pose_stub_get(pose_t *out);

#endif /* POSE_STUB_H */