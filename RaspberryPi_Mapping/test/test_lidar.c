/*
 * test/test_lidar.c — Standalone verification for lidar_driver.
 *
 * Starts the driver task, runs for 5 seconds, then prints every valid
 * distance reading in the forward ±10° window. Expected output: readings
 * in the range 150–3500mm that change as you move your hand.
 *
 * Build and run from project root:
 *   make test_lidar
 *   ./test_lidar
 */

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include "config.h"
#include "lidar_driver.h"

int main(void)
{
    lidar_driver_t lidar;

    /* Initialise. If this fails, the device path or baud is wrong. */
    if (lidar_driver_init(&lidar, LIDAR_PORT, LIDAR_BAUD) != 0) {
        fprintf(stderr, "Failed to open LiDAR. Is it plugged in?\n");
        return 1;
    }

    /*
     * Start the parser as a background thread.
     * pthread_create() arguments:
     *   &tid:               receives the thread ID (we need it to join later)
     *   NULL:               default thread attributes (stack size, etc.)
     *   lidar_driver_task:  the function to run in the new thread
     *   &lidar:             passed as void* arg to that function
     */
    pthread_t tid;
    pthread_create(&tid, NULL, lidar_driver_task, &lidar);

    printf("Collecting for 5 seconds — point sensor at something...\n\n");
    sleep(5);

    /* Take a snapshot of what the driver has accumulated. */
    scan_t snap;
    lidar_driver_copy_scan(&lidar, &snap);

    printf("Forward window (±10°):\n");
    printf("  Angle   Distance\n");
    printf("  -----   --------\n");

    /*
     * Walk the ±10° window around 0° (robot forward).
     * Angles near 0° also appear near 360° due to the wrap-around,
     * so we check both ends of the 0–359 range.
     */
    for (int a = 0; a < SCAN_POINTS; a++) {
        int rel = a;                         /* distance from 0°              */
        if (rel > 180) rel = 360 - rel;      /* fold 350° → 10°, etc.         */
        if (rel > 10) continue;              /* outside our ±10° window       */

        if (snap.valid[a]) {
            printf("  %3d°    %.0f mm\n", a, snap.distance_mm[a]);
        }
    }

    /* Signal the task to stop, wait for it to exit, then release resources. */
    lidar.running = 0;
    pthread_join(tid, NULL);
    lidar_driver_destroy(&lidar);

    return 0;
}