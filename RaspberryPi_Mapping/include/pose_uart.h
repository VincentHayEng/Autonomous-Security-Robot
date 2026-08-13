#ifndef POSE_UART_H
#define POSE_UART_H

#include "ipc_common.h"

/* Pi side of the UART IPC link with the ESP32.
   Receives pose_t (odometry), sends waypoint_t (nav targets).
   pose_uart_get is a drop-in replacement for the old pose stub */

int  pose_uart_init(void);
int  pose_uart_get(pose_t *out);
void pose_uart_send_waypoint(const waypoint_t *wp);
void *pose_uart_task(void *arg);   /* receive thread body */
void  pose_uart_destroy(void);

#endif
