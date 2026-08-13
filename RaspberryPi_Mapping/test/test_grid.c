/*
 * test/test_grid.c — Integration test: LiDAR + occupancy grid.
 *
 * Runs the LiDAR driver for 10 seconds, feeds each scan into the grid
 * with a stationary pose, then saves the result as both PGM and binary.
 *
 * Expected output: maps/map.pgm shows walls as black cells, floor as
 * white cells, unobserved areas as gray.
 *
 * Build:   make test_grid
 * Run:     ./test_grid
 * View:    scp pi@raspberrypi.local:~/RTS/project/mapping/maps/map.pgm ~/Desktop/
 */

#include <stdio.h>
#include <pthread.h>    /* pthread_t, pthread_create, pthread_join          */
#include <unistd.h>     /* usleep                                           */
#include "config.h"
#include "lidar_driver.h"
#include "occupancy_grid.h"
#include "pose_stub.h"

int main(void)
{
    /* ── Init LiDAR ────────────────────────────────────────────────────── */
    lidar_driver_t lidar;
    if (lidar_driver_init(&lidar, LIDAR_PORT, LIDAR_BAUD) != 0) {
        fprintf(stderr, "Failed to open LiDAR. Is it plugged in?\n");
        return 1;
    }

    /*
     * Start the parser thread.
     *
     * pthread_create arguments:
     *   &tid:               stores the thread ID so we can join it later
     *   NULL:               use default thread attributes
     *   lidar_driver_task:  function to run in the new thread
     *   &lidar:             passed as void* arg to that function
     *
     * After this call, lidar_driver_task runs in the background, filling
     * lidar.scan with fresh data every time a packet arrives.
     */
    pthread_t tid;
    pthread_create(&tid, NULL, lidar_driver_task, &lidar);

    /* ── Init grid and get starting pose ───────────────────────────────── */
    occupancy_grid_t grid;
    grid_init(&grid);

    pose_t pose;
    pose_stub_get(&pose);

    printf("Collecting scans for 10 seconds — point the sensor at a room...\n\n");

    /*
     * Main accumulation loop.
     *
     * usleep(200000) sleeps for 200,000 microseconds = 200ms = 0.2 seconds.
     * 50 iterations × 0.2s = 10 seconds total.
     *
     * Each iteration: copy the latest scan from the driver thread (thread-safe),
     * then update the grid with it. The grid accumulates log-odds evidence
     * across all 50 iterations — walls reinforce, noise averages out.
     */
    for (int i = 0; i < 50; i++) {
        usleep(200000);

        scan_t snap;
        lidar_driver_copy_scan(&lidar, &snap);   /* Thread-safe snapshot   */
        grid_update(&grid, &snap, &pose);

        /* Print progress every 2 seconds (every 10 frames at 0.2s each). */
        if ((i + 1) % 10 == 0) {
            printf("  %d seconds...\n", (i + 1) / 5);
        }
    }

    /* ── Save results ───────────────────────────────────────────────────── */
    if (grid_save_pgm(&grid, MAP_PGM_PATH, &pose) == 0) {
        printf("\nMap saved to %s\n", MAP_PGM_PATH);
    }
    if (grid_save_bin(&grid, MAP_BIN_PATH) == 0) {
        printf("Binary saved to %s\n", MAP_BIN_PATH);
    }

    /* ── Cleanup ────────────────────────────────────────────────────────── */
    /*
     * Set running to 0, then join the thread.
     *
     * pthread_join() waits for the thread to finish before continuing.
     * Without it, the thread might still be using the serial port when
     * lidar_driver_destroy() closes it — a use-after-free bug.
     */
    lidar.running = 0;
    pthread_join(tid, NULL);
    lidar_driver_destroy(&lidar);

    return 0;
}