#ifndef IPC_COMMON_H
#define IPC_COMMON_H

#include <stdint.h>
#include "config.h"

typedef struct {
    int32_t  x_mm;
    int32_t  y_mm;
    int32_t  theta_mrad;
    uint64_t timestamp_us;
    uint32_t seq;
} pose_t;

typedef struct {
    int32_t  target_x_mm;
    int32_t  target_y_mm;
    uint8_t  mode;
    uint32_t seq;
} waypoint_t;

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

#define PKT_SYNC            0xAA
#define PKT_TYPE_POSE       0x01
#define PKT_TYPE_WAYPOINT   0x02
#define PKT_TYPE_MODE       0x03

typedef struct __attribute__((packed)) {
    uint8_t  sync;
    uint8_t  type;
    uint16_t payload_len;
} uart_pkt_header_t;

#define MODE_MAPPING  0
#define MODE_PATROL   1

#endif /* IPC_COMMON_H */