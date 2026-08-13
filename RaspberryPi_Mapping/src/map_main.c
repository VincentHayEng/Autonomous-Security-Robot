/* Real-time mapping system - POSIX SCHED_FIFO periodic tasks.

   Rate Monotonic priority assignment: shorter period = higher
   priority (Liu & Layland 1973). Task set utilization ~0.08,
   far below the RM bound of ln(2) = 0.693, so all deadlines
   are provably met.

     Task              Period    Priority
     lidar_read_task   100ms     6
     grid_update_task  100ms     4
     frontier_task     2000ms    2
     map_save_task     5000ms    1

   SCHED_FIFO requires root - run with sudo */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include "config.h"
#include "ipc_common.h"
#include "lidar_driver.h"
#include "occupancy_grid.h"
#include "pose_uart.h"
#include "frontier.h"

#define PRIO_LIDAR_READ   6
#define PRIO_GRID_UPDATE  4
#define PRIO_FRONTIER     2
#define PRIO_MAP_SAVE     1

#define PERIOD_100MS    100000000L
#define PERIOD_2000MS  2000000000L
#define PERIOD_5000MS  5000000000L

/* Matches what the partner's navigation can physically achieve.
   Tighter causes oscillation at the target */
#define ARRIVAL_THRESHOLD_MM  150.0f

typedef struct {
    lidar_driver_t   lidar;
    occupancy_grid_t grid;
    pthread_mutex_t  grid_mutex;   /* guards the shared grid */
    volatile int     running;
} robot_state_t;

static robot_state_t *g_state = NULL;

/* SIGINT handler - only sets a flag. Signal handlers must be
   minimal (most functions aren't async-signal-safe). Tasks see
   running=0 at their next period and exit cleanly */
static void handle_sigint(int sig)
{
    (void)sig;
    if (g_state) g_state->running = 0;
}

/* Advance an absolute deadline by one period, carrying
   nanosecond overflow into seconds. Deadline computed from
   previous deadline - not "now" - so execution jitter never
   accumulates into schedule drift */
static void advance_timespec(struct timespec *ts, long period_ns)
{
    ts->tv_nsec += period_ns;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_nsec -= 1000000000L;
        ts->tv_sec  += 1;
    }
}

/* T=100ms P=6. Monitors scan health once per second */
static void *lidar_read_task(void *arg)
{
    robot_state_t *state = (robot_state_t *)arg;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    int tick = 0;
    while (state->running) {
        /* TIMER_ABSTIME: sleep until absolute clock value.
           This is the periodic task pattern from lecture */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        advance_timespec(&next, PERIOD_100MS);
        tick++;
        if (tick % 10 == 0) {
            scan_t snap;
            lidar_driver_copy_scan(&state->lidar, &snap);
            int valid = 0;
            for (int i = 0; i < SCAN_POINTS; i++) if (snap.valid[i]) valid++;
            printf("[lidar] %d/360 valid  seq=%u\n", valid, snap.seq);
        }
    }
    return NULL;
}

/* T=100ms P=4. The main pipeline: scan + pose -> grid */
static void *grid_update_task(void *arg)
{
    robot_state_t *state = (robot_state_t *)arg;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while (state->running) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        advance_timespec(&next, PERIOD_100MS);
        scan_t snap;
        lidar_driver_copy_scan(&state->lidar, &snap);
        pose_t pose;
        pose_uart_get(&pose);
        /* Critical section - save task locks the same mutex so
           it never serializes a half-updated grid */
        pthread_mutex_lock(&state->grid_mutex);
        grid_update(&state->grid, &snap, &pose);
        pthread_mutex_unlock(&state->grid_mutex);
    }
    return NULL;
}

/* T=2s P=2. The autonomy loop: check arrival, find next
   frontier, send waypoint to ESP32 */
static void *frontier_task(void *arg)
{
    robot_state_t *state = (robot_state_t *)arg;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    uint32_t wp_seq = 1;
    float    current_target_x = 0;
    float    current_target_y = 0;
    int      have_target = 0;

    while (state->running) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        advance_timespec(&next, PERIOD_2000MS);

        pose_t pose;
        pose_uart_get(&pose);

        /* Still driving to current target? Don't spam new
           waypoints - robot would constantly change its mind */
        if (have_target) {
            float dx = current_target_x - (float)pose.x_mm;
            float dy = current_target_y - (float)pose.y_mm;
            float dist = sqrtf(dx * dx + dy * dy);
            if (dist > ARRIVAL_THRESHOLD_MM) {
                continue;
            }
            have_target = 0;
            printf("[frontier] arrived at target\n");
        }

        float tx, ty;
        int found;
        pthread_mutex_lock(&state->grid_mutex);
        found = frontier_find_nearest(&state->grid, &pose, &tx, &ty);
        pthread_mutex_unlock(&state->grid_mutex);

        /* BFS found no frontier = map complete, stop the robot */
        if (!found) {
            printf("[frontier] map complete - sending STOP\n");
            waypoint_t stop = {0};
            stop.mode = MODE_STOP;
            stop.seq  = wp_seq++;
            pose_uart_send_waypoint(&stop);
            continue;
        }

        waypoint_t wp;
        wp.target_x_mm       = (int32_t)tx;
        wp.target_y_mm       = (int32_t)ty;
        wp.target_theta_mrad = 0;
        wp.mode              = MODE_GOTO;
        wp.seq               = wp_seq++;
        pose_uart_send_waypoint(&wp);

        current_target_x = tx;
        current_target_y = ty;
        have_target = 1;

        printf("[frontier] new target: x=%.0f y=%.0f (seq=%u)\n",
               tx, ty, wp.seq - 1);
    }
    return NULL;
}

/* T=5s P=1. Lowest priority - disk I/O is slow and
   non-critical, it can wait for everything else */
static void *map_save_task(void *arg)
{
    robot_state_t *state = (robot_state_t *)arg;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while (state->running) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        advance_timespec(&next, PERIOD_5000MS);
        pose_t pose;
        pose_uart_get(&pose);
        pthread_mutex_lock(&state->grid_mutex);
        grid_save_pgm(&state->grid, MAP_PGM_PATH, &pose);
        grid_save_bin(&state->grid, MAP_BIN_PATH);
        pthread_mutex_unlock(&state->grid_mutex);
        printf("[map]   saved to %s\n", MAP_PGM_PATH);
    }
    return NULL;
}

/* Creates a pthread under SCHED_FIFO at the given priority.
   EXPLICIT_SCHED is required - without it the thread silently
   inherits normal scheduling and ignores the policy we set.
   Fails without root: runaway RT threads can starve the system */
static int create_rt_thread(pthread_t *tid, void *(*func)(void *), void *arg, int priority)
{
    pthread_attr_t attr;
    struct sched_param param;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = priority;
    pthread_attr_setschedparam(&attr, &param);
    int ret = pthread_create(tid, &attr, func, arg);
    pthread_attr_destroy(&attr);
    if (ret != 0) fprintf(stderr, "create_rt_thread failed (prio %d) - sudo?\n", priority);
    return ret;
}

int main(void)
{
    robot_state_t state;
    memset(&state, 0, sizeof(state));
    state.running = 1;
    g_state = &state;
    signal(SIGINT, handle_sigint);

    /* Init in dependency order. LiDAR is a hard fail -
       mapping is pointless without it */
    if (lidar_driver_init(&state.lidar, LIDAR_PORT, LIDAR_BAUD) != 0) {
        fprintf(stderr, "Failed to open LiDAR on %s\n", LIDAR_PORT);
        return 1;
    }
    pthread_t lidar_bg_tid;
    pthread_create(&lidar_bg_tid, NULL, lidar_driver_task, &state.lidar);

    grid_init(&state.grid);
    pthread_mutex_init(&state.grid_mutex, NULL);

    /* UART is a soft fail - mapping still works from origin
       if the ESP32 isn't connected */
    if (pose_uart_init() != 0) {
        fprintf(stderr, "UART open failed - pose will stay at origin\n");
    }
    pthread_t uart_tid;
    pthread_create(&uart_tid, NULL, pose_uart_task, NULL);

    pthread_t lidar_tid, grid_tid, frontier_tid, save_tid;
    if (create_rt_thread(&lidar_tid,    lidar_read_task,  &state, PRIO_LIDAR_READ)  != 0 ||
        create_rt_thread(&grid_tid,     grid_update_task, &state, PRIO_GRID_UPDATE) != 0 ||
        create_rt_thread(&frontier_tid, frontier_task,    &state, PRIO_FRONTIER)    != 0 ||
        create_rt_thread(&save_tid,     map_save_task,    &state, PRIO_MAP_SAVE)    != 0) {
        fprintf(stderr, "Failed to create RT threads. Run with sudo.\n");
        state.running = 0;
    }

    printf("Autonomous mapping started. ctrl+C to stop.\n\n");

    /* main blocks here for the program's whole life.
       Joins complete when SIGINT flips running to 0 */
    pthread_join(lidar_tid,    NULL);
    pthread_join(grid_tid,     NULL);
    pthread_join(frontier_tid, NULL);
    pthread_join(save_tid,     NULL);

    state.lidar.running = 0;
    pthread_join(lidar_bg_tid, NULL);

    /* Final save so ctrl+C never loses the map */
    pose_t pose;
    pose_uart_get(&pose);
    grid_save_pgm(&state.grid, MAP_PGM_PATH, &pose);
    grid_save_bin(&state.grid, MAP_BIN_PATH);
    printf("\nFinal map saved.\n");

    pthread_mutex_destroy(&state.grid_mutex);
    lidar_driver_destroy(&state.lidar);
    pose_uart_destroy();
    return 0;
}
