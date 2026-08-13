#include <stdio.h>
#include <stdint.h>

typedef struct {
    uint16_t count;
    uint16_t angle;
} encoder_data_t;

typedef struct {
    float x_accel;
    float y_accel;
    float z_accel;
    float x_rot;
    float y_rot;
    float z_rot;
    float v_x;
    float v_y;
    float v_z;
    float omega_x;
    float omega_y;
    float omega_z;
} imu_data_t;

typedef struct {
    uint16_t count;
    uint16_t angle;
} navigation_data_t;