/* Pi side of the Pi <-> ESP32 UART link.
   Receives pose_t (ESP32 odometry, every 50ms).
   Sends waypoint_t (frontier targets).
   Frame: [0xAA][TYPE][LEN_LO][LEN_HI][PAYLOAD][XOR CHECKSUM] */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>

#include "pose_uart.h"
#include "ipc_common.h"
#include "config.h"

static int             fd      = -1;
static pose_t          latest_pose;
static pthread_mutex_t pose_mutex;   /* protects latest_pose */
static volatile int    running = 1;

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL
         + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* XOR of payload bytes. Catches any single corrupted byte -
   the realistic failure mode on a short wire at 115200 baud.
   Same checksum computed on both processors */
static uint8_t xor_checksum(const uint8_t *data, size_t len)
{
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++) cs ^= data[i];
    return cs;
}

int pose_uart_init(void)
{
    memset(&latest_pose, 0, sizeof(latest_pose));
    pthread_mutex_init(&pose_mutex, NULL);

    fd = open(IPC_UART_PORT, O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("pose_uart_init: open"); return -1; }

    /* Clear O_NONBLOCK so read() blocks properly */
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    /* Same 8N1 raw mode setup as the LiDAR driver */
    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |=  CS8 | CREAD | CLOCAL;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tty);

    fprintf(stdout, "pose_uart: opened %s at 115200 baud\n", IPC_UART_PORT);
    return 0;
}

int pose_uart_get(pose_t *out)
{
    /* Copy-under-lock snapshot, same pattern as lidar copy_scan.
       Before first packet arrives this returns the zeroed origin
       pose - which is the correct default (robot starts at 0,0) */
    pthread_mutex_lock(&pose_mutex);
    memcpy(out, &latest_pose, sizeof(pose_t));
    pthread_mutex_unlock(&pose_mutex);
    return (latest_pose.seq > 0) ? 1 : 0;
}

void pose_uart_send_waypoint(const waypoint_t *wp)
{
    if (fd < 0) return;

    /* theta forced to 0: frontier computes positions not headings.
       Partner's navigation treats 0 as "no final rotation" */
    waypoint_t out;
    out.target_x_mm       = wp->target_x_mm;
    out.target_y_mm       = wp->target_y_mm;
    out.target_theta_mrad = 0;
    out.mode              = MODE_GOTO;
    out.seq               = wp->seq;

    uint8_t  buf[32];
    uint16_t len = sizeof(waypoint_t);
    buf[0] = PKT_SYNC;
    buf[1] = PKT_TYPE_WAYPOINT;
    buf[2] = len & 0xFF;
    buf[3] = (len >> 8) & 0xFF;
    memcpy(&buf[4], &out, len);
    buf[4 + len] = xor_checksum((uint8_t *)&out, len);

    /* Single write so the frame transmits contiguously */
    write(fd, buf, 5 + len);
}

void *pose_uart_task(void *arg)
{
    (void)arg;
    uint8_t byte;
    uint8_t payload[64];

    while (running) {
        /* Sync hunt - same technique as LiDAR parser */
        if (read(fd, &byte, 1) != 1) {
            if (errno == EINTR) continue;
            break;
        }
        if (byte != PKT_SYNC) continue;

        uint8_t type;
        if (read(fd, &type, 1) != 1) continue;
        if (type != PKT_TYPE_POSE) continue;

        uint8_t len_bytes[2];
        if (read(fd, len_bytes, 2) != 2) continue;
        uint16_t payload_len = len_bytes[0] | (len_bytes[1] << 8);

        /* Length check rejects desynced garbage where random
           data happened to contain 0xAA */
        if (payload_len != sizeof(pose_t)) continue;
        if (payload_len > sizeof(payload)) continue;

        /* Partial-read loop - kernel may deliver in chunks */
        int received = 0;
        while (received < payload_len) {
            int n = read(fd, payload + received, payload_len - received);
            if (n < 0) { received = -1; break; }
            received += n;
        }
        if (received != payload_len) continue;

        uint8_t cs_received;
        if (read(fd, &cs_received, 1) != 1) continue;
        if (xor_checksum(payload, payload_len) != cs_received) {
            fprintf(stderr, "pose_uart: checksum mismatch - dropped\n");
            continue;
        }

        /* All checks passed - critical section to update pose.
           Corruption costs one dropped packet, never a bad map */
        pthread_mutex_lock(&pose_mutex);
        memcpy(&latest_pose, payload, sizeof(pose_t));
        latest_pose.timestamp_us = now_us();
        pthread_mutex_unlock(&pose_mutex);
    }
    return NULL;
}

void pose_uart_destroy(void)
{
    running = 0;
    close(fd);
    pthread_mutex_destroy(&pose_mutex);
    fprintf(stdout, "pose_uart: closed.\n");
}
