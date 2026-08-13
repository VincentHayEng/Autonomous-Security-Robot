#ifndef LIDAR_DRIVER_H
#define LIDAR_DRIVER_H

#include <pthread.h>
#include "ipc_common.h"

/* Driver context. The mutex protects scan - the parser thread
   writes it continuously while other tasks read snapshots.
   Never read drv->scan directly, always go through copy_scan */
typedef struct {
    int             fd;
    scan_t          scan;
    pthread_mutex_t scan_mutex;
    volatile int    running;    /* task loop exits when 0 */
} lidar_driver_t;

/* Opens serial port, configures 8N1 raw mode, inits mutex */
int  lidar_driver_init(lidar_driver_t *drv, const char *port, int baud);

/* Thread body for pthread_create. Parses packets forever */
void *lidar_driver_task(void *arg);

/* Thread-safe snapshot: lock, memcpy, unlock */
void  lidar_driver_copy_scan(lidar_driver_t *drv, scan_t *out);

/* Set running=0 and join the thread BEFORE calling this */
void  lidar_driver_destroy(lidar_driver_t *drv);

#endif
