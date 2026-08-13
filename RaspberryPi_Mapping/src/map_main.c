/*
 * map_main.c — Real-time mapping system.
 *
 * Three POSIX threads run simultaneously under SCHED_FIFO scheduling.
 * SCHED_FIFO = "first in, first out" real-time scheduler. When a higher
 * priority thread becomes ready, it immediately preempts whatever lower
 * priority thread is running. No sharing of CPU time — the highest
 * priority ready thread always runs.
 *
 * Rate Monotonic assignment: shorter period = higher priority.
 *
 *   Thread            Period    Priority
 *   lidar_read_task   100ms     6
 *   grid_update_task  100ms     4
 *   map_save_task     5000ms    1
 *
 * IR task is planned but not yet wired to the robot — stub noted below.
 * Frontier task comes after this file is verified working.
 *
 * IMPORTANT: SCHED_FIFO requires root privileges on Linux.
 * Run with: sudo ./map_main
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>          /* signal(), SIGINT for ctrl+C handling        */
#include <pthread.h>         /* pthread_t, pthread_create, pthread_attr_t   */
#include <time.h>            /* clock_gettime, clock_nanosleep              */

#include "config.h"
#include "ipc_common.h"
#include "lidar_driver.h"
#include "occupancy_grid.h"
#include "pose_stub.h"

/* ── Task priorities (SCHED_FIFO: higher number = higher priority) ───────── */
#define PRIO_LIDAR_READ   6
#define PRIO_GRID_UPDATE  4
#define PRIO_MAP_SAVE     1

/* ── Task periods in nanoseconds ─────────────────────────────────────────── */
#define PERIOD_100MS   100000000L    /* 100,000,000 ns = 100ms               */
#define PERIOD_5000MS 5000000000L    /* 5,000,000,000 ns = 5 seconds         */

/* ── Shared state ─────────────────────────────────────────────────────────── 
 * All tasks share this single struct, passed as void* to each thread.
 * Each field that multiple threads touch is protected by its own mutex.
 * A mutex (mutual exclusion lock) ensures only one thread reads or writes
 * a shared value at a time — preventing corrupted half-written data.
 */
typedef struct {
    lidar_driver_t   lidar;        /* LiDAR driver (has its own internal mutex) */
    occupancy_grid_t grid;         /* The map — protected by grid_mutex         */
    pthread_mutex_t  grid_mutex;   /* Lock before reading or writing grid       */
    volatile int     running;      /* Set to 0 to stop all threads              */
} robot_state_t;

/* ── Ctrl+C handler ───────────────────────────────────────────────────────── 
 * When the user presses ctrl+C, the OS sends SIGINT to the process.
 * Without a handler, the program dies immediately and skips cleanup.
 * This handler sets running=0 instead, letting threads finish gracefully.
 * 'state' is a global pointer here because signal handlers can't take args.
 */
static robot_state_t *g_state = NULL;

static void handle_sigint(int sig)
{
    (void)sig;   /* Suppress unused parameter warning                       */
    if (g_state) g_state->running = 0;
}

/* ── Timing helper ────────────────────────────────────────────────────────── 
 * Advances a timespec by period_ns nanoseconds.
 *
 * Used with clock_nanosleep(TIMER_ABSTIME) — sleeping until an absolute
 * time rather than for a relative duration. This prevents drift: if a task
 * runs 2ms late, the NEXT period still starts at the correct time because
 * the deadline is calculated from the previous deadline, not from "now".
 *
 * Without this pattern, a task that consistently runs 2ms long would
 * drift 2ms per period — 120ms per second at 100ms period.
 */
static void advance_timespec(struct timespec *ts, long period_ns)
{
    ts->tv_nsec += period_ns;
    /* tv_nsec must stay below 1,000,000,000 (one second).
     * If adding the period overflows it, carry one second into tv_sec. */
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_nsec -= 1000000000L;
        ts->tv_sec  += 1;
    }
}

/* ── lidar_read_task ──────────────────────────────────────────────────────── 
 * Period: 100ms  Priority: 6 (highest)
 *
 * The LiDAR driver thread already runs in the background reading packets.
 * This task's only job is to confirm the driver is alive and print a
 * heartbeat. The driver writes directly into its internal scan buffer —
 * other tasks call lidar_driver_copy_scan() to get a snapshot.
 *
 * Why have this task at all? In the full system it will also validate
 * scan freshness (check scan->seq is incrementing) and raise a fault
 * flag if the LiDAR stops sending data. For now it logs every 10 periods.
 */
static void *lidar_read_task(void *arg)
{
    robot_state_t *state = (robot_state_t *)arg;

    /* Get the current time as our first deadline. */
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    int tick = 0;

    while (state->running) {
        /* Sleep until the next absolute deadline. TIMER_ABSTIME means
         * "wake me at this exact clock value", not "wake me in X ns". */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        advance_timespec(&next, PERIOD_100MS);

        tick++;
        /* Print a status line every 10 ticks (once per second). */
        if (tick % 10 == 0) {
            scan_t snap;
            lidar_driver_copy_scan(&state->lidar, &snap);

            /* Count how many angles have valid readings this scan. */
            int valid = 0;
            for (int i = 0; i < SCAN_POINTS; i++) {
                if (snap.valid[i]) valid++;
            }
            printf("[lidar] %d/360 valid readings  seq=%u\n",
                   valid, snap.seq);
        }
    }
    return NULL;
}

/* ── grid_update_task ─────────────────────────────────────────────────────── 
 * Period: 100ms  Priority: 4
 *
 * Takes a snapshot of the current LiDAR scan and the current pose,
 * then integrates them into the occupancy grid.
 *
 * Priority 4 (lower than lidar priority 6) means if both tasks are ready
 * at the same time, the LiDAR task runs first. This is correct — we want
 * the freshest possible scan before updating the grid.
 */
static void *grid_update_task(void *arg)
{
    robot_state_t *state = (robot_state_t *)arg;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (state->running) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        advance_timespec(&next, PERIOD_100MS);

        /* Get the latest scan from the driver. Thread-safe copy. */
        scan_t snap;
        lidar_driver_copy_scan(&state->lidar, &snap);

        /* Get the current pose.
         * Pose stub returns (0, 0, 0) — stationary at origin.
         * When ESP32 UART is ready, replace this call with uart_get_pose(). */
        pose_t pose;
        pose_stub_get(&pose);

        /* Lock the grid, update it, unlock.
         * map_save_task also locks the grid mutex before saving — this
         * prevents a save from capturing a half-updated grid. */
        pthread_mutex_lock(&state->grid_mutex);
        grid_update(&state->grid, &snap, &pose);
        pthread_mutex_unlock(&state->grid_mutex);
    }
    return NULL;
}

/* ── map_save_task ────────────────────────────────────────────────────────── 
 * Period: 5000ms  Priority: 1 (lowest)
 *
 * Saves the current grid to disk every 5 seconds.
 * Lowest priority because disk I/O is slow and non-critical — it can
 * wait for every other task to finish its current period first.
 *
 * Saves both PGM (human viewable) and binary (fast reload for patrol).
 */
static void *map_save_task(void *arg)
{
    robot_state_t *state = (robot_state_t *)arg;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    /* Get a fresh pose just for the robot position marker in the PGM. */
    pose_t pose;
    pose_stub_get(&pose);

    while (state->running) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        advance_timespec(&next, PERIOD_5000MS);

        /* Lock the grid for the duration of the save.
         * grid_update_task will block until we release it. */
        pthread_mutex_lock(&state->grid_mutex);
        grid_save_pgm(&state->grid, MAP_PGM_PATH, &pose);
        grid_save_bin(&state->grid, MAP_BIN_PATH);
        pthread_mutex_unlock(&state->grid_mutex);

        printf("[map]   saved to %s\n", MAP_PGM_PATH);
    }
    return NULL;
}

/* ── create_rt_thread ─────────────────────────────────────────────────────── 
 * Creates a pthread with SCHED_FIFO scheduling at a given priority.
 *
 * Without this setup, pthreads use the default SCHED_OTHER policy which
 * does not guarantee when a thread runs — the kernel decides based on
 * fairness. SCHED_FIFO guarantees that the highest priority ready thread
 * always runs immediately.
 *
 * tid:      receives the thread ID
 * func:     the task function to run
 * arg:      passed as void* to func
 * priority: SCHED_FIFO priority (1 lowest, 99 highest on Linux)
 */
static int create_rt_thread(pthread_t *tid,
                             void *(*func)(void *),
                             void *arg,
                             int priority)
{
    pthread_attr_t attr;
    struct sched_param param;

    /* Initialise attribute struct with defaults. */
    pthread_attr_init(&attr);

    /* PTHREAD_EXPLICIT_SCHED: use the scheduling policy we set here,
     * not the one inherited from the calling thread. */
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    /* Set real-time FIFO scheduling. */
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

    /* Set the priority within SCHED_FIFO. */
    param.sched_priority = priority;
    pthread_attr_setschedparam(&attr, &param);

    int ret = pthread_create(tid, &attr, func, arg);
    pthread_attr_destroy(&attr);

    if (ret != 0) {
        fprintf(stderr, "create_rt_thread: failed (priority %d) — "
                        "are you running with sudo?\n", priority);
    }
    return ret;
}

/* ── main ─────────────────────────────────────────────────────────────────── */
int main(void)
{
    robot_state_t state;
    memset(&state, 0, sizeof(state));
    state.running = 1;
    g_state = &state;

    /* Register ctrl+C handler so we shut down cleanly. */
    signal(SIGINT, handle_sigint);

    /* ── Init LiDAR ─────────────────────────────────────────────────────── */
    if (lidar_driver_init(&state.lidar, LIDAR_PORT, LIDAR_BAUD) != 0) {
        fprintf(stderr, "Failed to open LiDAR on %s\n", LIDAR_PORT);
        return 1;
    }

    /* Start the LiDAR background parser thread.
     * This thread runs at default priority — it feeds data into the driver's
     * internal scan buffer continuously. The real-time tasks above consume
     * snapshots of that buffer. */
    pthread_t lidar_bg_tid;
    pthread_create(&lidar_bg_tid, NULL, lidar_driver_task, &state.lidar);

    /* ── Init grid ───────────────────────────────────────────────────────── */
    grid_init(&state.grid);
    pthread_mutex_init(&state.grid_mutex, NULL);

    /* ── Start real-time tasks ───────────────────────────────────────────── */
    pthread_t lidar_tid, grid_tid, save_tid;

    /* IR task would be created here when sensor is mounted on the robot:
     * create_rt_thread(&ir_tid, ir_read_task, &state, PRIO_IR_READ=7); */

    if (create_rt_thread(&lidar_tid, lidar_read_task,   &state, PRIO_LIDAR_READ)  != 0 ||
        create_rt_thread(&grid_tid,  grid_update_task,  &state, PRIO_GRID_UPDATE) != 0 ||
        create_rt_thread(&save_tid,  map_save_task,     &state, PRIO_MAP_SAVE)    != 0)
    {
        fprintf(stderr, "Failed to create real-time threads. Run with sudo.\n");
        state.running = 0;
    }

    printf("Mapping started. Press ctrl+C to stop and save final map.\n\n");

    /* ── Wait for shutdown ───────────────────────────────────────────────── 
     * pthread_join() blocks until the named thread exits.
     * We join all three real-time tasks — they exit when running becomes 0.
     * Order doesn't matter since all three check the same flag. */
    pthread_join(lidar_tid, NULL);
    pthread_join(grid_tid,  NULL);
    pthread_join(save_tid,  NULL);

    /* Stop the background parser and wait for it to exit. */
    state.lidar.running = 0;
    pthread_join(lidar_bg_tid, NULL);

    /* ── Final save ──────────────────────────────────────────────────────── */
    pose_t pose;
    pose_stub_get(&pose);
    grid_save_pgm(&state.grid, MAP_PGM_PATH, &pose);
    grid_save_bin(&state.grid, MAP_BIN_PATH);
    printf("\nFinal map saved. Shutting down.\n");

    /* ── Cleanup ─────────────────────────────────────────────────────────── */
    pthread_mutex_destroy(&state.grid_mutex);
    lidar_driver_destroy(&state.lidar);

    return 0;
}