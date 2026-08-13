/*
 * lidar_driver.h — Public interface for the LDS-02 LiDAR parser.
 *
 * The driver owns one serial port file descriptor and one mutex-protected
 * scan_t. All external access to scan data goes through
 * lidar_driver_copy_scan() — never read drv->scan directly from
 * outside this module.
 *
 * Typical use in map_main.c:
 *
 *   lidar_driver_t lidar;
 *   lidar_driver_init(&lidar, LIDAR_PORT, LIDAR_BAUD);
 *
 *   pthread_t tid;
 *   pthread_create(&tid, NULL, lidar_driver_task, &lidar);
 *
 *   scan_t snap;
 *   lidar_driver_copy_scan(&lidar, &snap);  // safe from any thread
 */

#ifndef LIDAR_DRIVER_H
#define LIDAR_DRIVER_H

#include <pthread.h>
#include "ipc_common.h"

/*
 * lidar_driver_t — driver context.
 *
 * Allocate one instance per physical LiDAR. Pass a pointer to every
 * function in this module. Do not copy or move after init.
 */
typedef struct {
    int             fd;           /* POSIX file descriptor from open()        */
    scan_t          scan;         /* Accumulated 360° scan, mutex-protected   */
    pthread_mutex_t scan_mutex;   /* Hold before reading or writing scan      */
    volatile int    running;      /* Task loop runs while this is non-zero    */
} lidar_driver_t;

/*
 * lidar_driver_init — open and configure the serial port.
 *
 * Configures the port for raw 8N1 communication (no echo, no line
 * discipline, no flow control). Initialises the mutex and sets running=1.
 *
 * port: device path, e.g. "/dev/ttyUSB0"
 * baud: integer baud rate — must be 115200 for LDS-02
 *
 * Returns 0 on success, -1 on failure with errno set.
 */
int  lidar_driver_init(lidar_driver_t *drv, const char *port, int baud);

/*
 * lidar_driver_task — packet parsing loop, designed for pthread_create().
 *
 * arg: pointer to an initialised lidar_driver_t (cast from void*)
 *
 * Reads bytes until it finds the sync byte 0x54, reads the rest of the
 * 47-byte packet, validates it, extracts 12 distance samples, rotates
 * angles into robot-space using the calibration offset, and writes into
 * drv->scan under the mutex.
 *
 * Runs until drv->running == 0.
 */
void *lidar_driver_task(void *arg);

/*
 * lidar_driver_copy_scan — thread-safe snapshot of the current scan.
 *
 * Acquires the mutex, copies drv->scan into *out, releases the mutex.
 * out: caller-allocated scan_t to receive the copy.
 */
void  lidar_driver_copy_scan(lidar_driver_t *drv, scan_t *out);

/*
 * lidar_driver_destroy — release all resources.
 *
 * Set drv->running = 0 and join the task thread BEFORE calling this.
 * Calling while the task is still running causes a use-after-free.
 */
void  lidar_driver_destroy(lidar_driver_t *drv);

#endif /* LIDAR_DRIVER_H */