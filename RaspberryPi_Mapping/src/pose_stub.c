/*
 * pose_stub.c — Fake pose returning robot stationary at origin.
 *
 * The timestamp and seq fields are real so that grid_update can use them
 * for timing diagnostics later. Only the position is faked.
 */

#include <stdint.h>
#include <time.h>
#include "pose_stub.h"

/*
 * now_us — current time in microseconds since boot.
 *
 * Duplicated from lidar_driver.c intentionally — each module is self-
 * contained. Shared utility functions go into a utils.c file later if
 * the duplication becomes a maintenance problem.
 */
static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec  * 1000000ULL
         + (uint64_t)ts.tv_nsec / 1000ULL;
}

static uint32_t seq = 0;

void pose_stub_get(pose_t *out)
{
    out->x_mm         = 0;       /* No X displacement from origin          */
    out->y_mm         = 0;       /* No Y displacement from origin          */
    out->theta_mrad   = 0;       /* No rotation — facing initial forward   */
    out->timestamp_us = now_us();
    out->seq          = seq++;   /* Increment each call for diagnostic use */
}