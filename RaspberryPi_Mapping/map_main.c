cat > ~/RTS/project/mapping/src/map_main.c << 'EOF'
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>

#include "config.h"
#include "ipc_common.h"
#include "lidar_driver.h"
#include "occupancy_grid.h"
#include "pose_uart.h"

#define PRIO_LIDAR_READ   6
#define PRIO_GRID_UPDATE  4
#define PRIO_MAP_SAVE     1

#define PERIOD_100MS   100000000L
#define PERIOD_5000MS 5000000000L

typedef struct {
    lidar_driver_t   lidar;
    occupancy_grid_t grid;
    pthread_mutex_t  grid_mutex;
    volatile int     running;
} robot_state_t;

static robot_state_t *g_state = NULL;

static void handle_sigint(int sig)
{
    (void)sig;
    if (g_state) g_state->running = 0;
}

static void advance_timespec(struct timespec *ts, long period_ns)
{
    ts->tv_nsec += period_ns;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_nsec -= 1000000000L;
        ts->tv_sec  += 1;
    }
}

static void *lidar_read_task(void *arg)
{
    robot_state_t *state = (robot_state_t *)arg;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    int tick = 0;

    while (state->running) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        advance_timespec(&next, PERIOD_100MS);
        tick++;
        if (tick % 10 == 0) {
            scan_t snap;
            lidar_driver_copy_scan(&state->lidar, &snap);
            int valid = 0;
            for (int i = 0; i < SCAN_POINTS; i++) {
                if (snap.valid[i]) valid++;
            }
            printf("[lidar] %d/360 valid readings  seq=%u\n", valid, snap.seq);
        }
    }
    return NULL;
}

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

        pthread_mutex_lock(&state->grid_mutex);
        grid_update(&state->grid, &snap, &pose);
        pthread_mutex_unlock(&state->grid_mutex);
    }
    return NULL;
}

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

static int create_rt_thread(pthread_t *tid,
                             void *(*func)(void *),
                             void *arg,
                             int priority)
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
    if (ret != 0) {
        fprintf(stderr, "create_rt_thread: failed (priority %d) — "
                        "are you running with sudo?\n", priority);
    }
    return ret;
}

int main(void)
{
    robot_state_t state;
    memset(&state, 0,
