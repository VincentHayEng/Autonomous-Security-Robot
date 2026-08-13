#ifndef CONFIG_H
#define CONFIG_H

/* ── Grid ──────────────────────────────────────────────────────────────────
   The occupancy grid is a 2D array of cells. Each cell represents a square
   patch of floor. CELL_SIZE controls resolution vs memory tradeoff.
   200 x 200 cells at 50mm each covers a 10m x 10m room.               */
#define GRID_CELL_SIZE_MM    50
#define GRID_WIDTH_CELLS    200
#define GRID_HEIGHT_CELLS   200

/* ── LiDAR ─────────────────────────────────────────────────────────────────
   LDS-02 connects via USB and appears as /dev/ttyUSB0.
   LDS-01 uses 230400 baud — wrong here, don't change unless swapping units.
   Readings below MIN_RANGE are unreliable (sensor hardware limitation).
   SCAN_POINTS = one array slot per integer degree, 0-359.              */
#define LIDAR_PORT          "/dev/ttyUSB0"
#define LIDAR_BAUD          115200
#define LIDAR_MIN_RANGE_MM  150
#define LIDAR_MAX_RANGE_MM  3500
#define SCAN_POINTS         360

/* ── IR sensors ─────────────────────────────────────────────────────────────
   THIS IS THE ONLY LINE TO CHANGE when adding sensors.
   Also add one row to IR_SENSOR_CONFIGS[] in ir_driver.c.
   All arrays sized from this constant — nothing else needs editing.    */
#define IR_SENSOR_COUNT     1

/* ── Occupancy log-odds ─────────────────────────────────────────────────────
   Each LiDAR ray nudges cell values by these amounts.
   Negative = evidence of free space (ray passed through).
   Positive = evidence of obstacle (ray ended here).
   Clamped to MIN/MAX so a single bad reading can't dominate the map.  */
#define LOG_ODDS_FREE      -1
#define LOG_ODDS_OCCUPIED   2
#define LOG_ODDS_MIN       -10
#define LOG_ODDS_MAX        10

/* ── UART IPC (Pi to ESP32) ─────────────────────────────────────────────────
   Pi and ESP32 communicate over hardware UART.
   ttyAMA0 is the Pi 5 primary hardware UART port.
   Pose comes in from ESP32, waypoints go out to ESP32.                */
#define IPC_UART_PORT       "/dev/ttyAMA0"
#define IPC_UART_BAUD       115200

/* ── Output paths ───────────────────────────────────────────────────────────
   PGM = portable graymap: human-viewable in any image viewer (feh).
   BIN = compact binary: fast reload when switching to patrol mode.    */
#define MAP_PGM_PATH        "maps/map.pgm"
#define MAP_BIN_PATH        "maps/map.bin"

#endif /* CONFIG_H */