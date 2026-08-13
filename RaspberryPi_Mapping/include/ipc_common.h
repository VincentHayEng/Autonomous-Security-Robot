#ifndef IPC_COMMON_H
#define IPC_COMMON_H

#include <stdint.h>
#include "config.h"

/* Shared struct definitions for Pi <-> ESP32 communication.
   Both processors compile against these exact layouts.

   packed attribute is critical: without it the two compilers
   (ARM64 gcc on Pi, Xtensa on ESP32) may insert different padding
   between fields and the structs silently decode as garbage */

/* Sent by ESP32 every 50ms. Odometry-integrated position estimate */
typedef struct __attribute__((packed)) {
    int32_t  x_mm;          /* displacement from power-on position */
    int32_t  y_mm;
    int32_t  theta_mrad;    /* heading, milliradians, 0 = initial forward */
    uint64_t timestamp_us;
    uint32_t seq;           /* gap in seq = dropped packet */
} pose_t;

/* Sent by Pi when frontier task selects a new target.
   Matches partner's ESP32 struct exactly - 17 bytes packed */
typedef struct __attribute__((packed)) {
    int32_t  target_x_mm;
    int32_t  target_y_mm;
    int32_t  target_theta_mrad;  /* final heading, 0 = no spin needed */
    uint8_t  mode;               /* STOP / GOTO / RESET below */
    uint32_t seq;
} waypoint_t;

/* One slot per degree. valid[] must be checked before distance_mm[] */
typedef struct {
    float    distance_mm[SCAN_POINTS];
    uint8_t  valid[SCAN_POINTS];
    uint64_t timestamp_us;
    uint32_t seq;
} scan_t;

typedef struct {
    int trig_pin;
    int echo_pin;
    int angle_deg;
} ir_sensor_config_t;

typedef struct {
    float    distance_mm;
    uint8_t  valid;
    uint64_t timestamp_us;
} ir_reading_t;

/* UART frame: [SYNC][TYPE][LEN_LO][LEN_HI][PAYLOAD][XOR CHECKSUM]
   Sync byte lets the receiver find packet boundaries in a
   continuous byte stream - same technique as the LiDAR parser */
#define PKT_SYNC            0xAA
#define PKT_TYPE_POSE       0x01
#define PKT_TYPE_WAYPOINT   0x02
#define PKT_TYPE_MODE       0x03

/* Mode values match partner's ESP32 navigation task */
#define MODE_STOP    0
#define MODE_GOTO    1
#define MODE_RESET   2

typedef struct __attribute__((packed)) {
    uint8_t  sync;
    uint8_t  type;
    uint16_t payload_len;
} uart_pkt_header_t;

#endif
