#ifndef CONFIG_H
#define CONFIG_H

/* Grid resolution vs memory tradeoff:
   200x200 cells at 50mm each = 10m x 10m coverage, 40KB total */
#define GRID_CELL_SIZE_MM    50
#define GRID_WIDTH_CELLS    200
#define GRID_HEIGHT_CELLS   200

/* LDS-02 specifics. LDS-01 uses 230400 baud - not interchangeable.
   Min range 150mm is a hardware limitation of the sensor,
   readings below that are unreliable and get rejected in the driver */
#define LIDAR_PORT          "/dev/ttyUSB0"
#define LIDAR_BAUD          115200
#define LIDAR_MIN_RANGE_MM  150
#define LIDAR_MAX_RANGE_MM  3500
#define SCAN_POINTS         360

/* Only line to change when adding sensors */
#define IR_SENSOR_COUNT     1

/* Log-odds evidence values. Free is weaker than occupied on purpose:
   many rays pass through a cell but only one ends in it, so
   occupied evidence needs more weight per observation.
   Clamping at +/-10 keeps cells adaptive - a wall that "moves"
   (furniture) only needs ~10 contradicting scans to flip */
#define LOG_ODDS_FREE      -1
#define LOG_ODDS_OCCUPIED   2
#define LOG_ODDS_MIN       -10
#define LOG_ODDS_MAX        10

/* UART link to the ESP32. ttyAMA0 is the Pi 5 hardware UART
   enabled via dtoverlay=uart0 in /boot/firmware/config.txt */
#define IPC_UART_PORT       "/dev/ttyAMA0"
#define IPC_UART_BAUD       115200

/* PGM = viewable grayscale image, BIN = fast reload for patrol mode */
#define MAP_PGM_PATH        "maps/map.pgm"
#define MAP_BIN_PATH        "maps/map.bin"

#endif
