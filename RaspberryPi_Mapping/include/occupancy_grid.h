/*
 * occupancy_grid.h — Public interface for the 2D occupancy grid.
 *
 * The grid is a 2D array of int8_t log-odds values.
 * Each cell represents one GRID_CELL_SIZE_MM × GRID_CELL_SIZE_MM patch of floor.
 * The robot's power-on position maps to the center cell (origin_x, origin_y).
 *
 * Cell values:
 *   0         = unknown (no observations yet)
 *   negative  = evidence of free space (LiDAR rays passed through)
 *   positive  = evidence of occupied space (LiDAR ray ended here)
 *
 * These are log-odds values, not probabilities. They accumulate over time —
 * each new observation nudges the value up or down. The MIN/MAX clamp in
 * config.h prevents any single cluster of readings from locking a cell
 * permanently regardless of future evidence.
 *
 * Visualisation:
 *   negative → white (free)
 *   zero     → gray  (unknown)
 *   positive → black (occupied wall)
 */

#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <stdint.h>
#include "config.h"
#include "ipc_common.h"

/*
 * occupancy_grid_t — the grid data structure.
 *
 * cells[row][col] — row 0 is the top of the map (furthest forward from origin).
 * origin_x, origin_y — the column and row that correspond to world (0, 0).
 * All other cells are offsets from this point.
 */
typedef struct {
    int8_t cells[GRID_HEIGHT_CELLS][GRID_WIDTH_CELLS];
    int    width;     /* GRID_WIDTH_CELLS — kept in struct for convenience     */
    int    height;    /* GRID_HEIGHT_CELLS                                     */
    int    origin_x;  /* column index corresponding to world x=0              */
    int    origin_y;  /* row    index corresponding to world y=0              */
} occupancy_grid_t;

/*
 * grid_init — zero all cells and set origin to center.
 *
 * Must be called before any other grid function.
 * All cells start at 0 (unknown).
 */
void grid_init(occupancy_grid_t *g);

/*
 * grid_update — integrate one LiDAR scan into the grid.
 *
 * For each valid reading in scan:
 *   1. Converts (angle, distance) to world (x, y) using the robot pose.
 *   2. Converts world (x, y) to grid (col, row).
 *   3. Runs Bresenham's line from the robot cell to the endpoint cell.
 *   4. Marks every cell along the ray as FREE, the endpoint as OCCUPIED.
 *
 * scan:  latest scan from lidar_driver_copy_scan()
 * pose:  current robot pose from pose_stub_get() or UART (later)
 */
void grid_update(occupancy_grid_t *g, const scan_t *scan, const pose_t *pose);

/*
 * grid_save_pgm — write the grid as a PGM grayscale image.
 *
 * PGM (Portable GrayMap) is a plain-text image format viewable with feh,
 * GIMP, or any image viewer. Each pixel corresponds to one grid cell.
 *
 * path: output file path, e.g. "maps/map.pgm"
 * pose: used to mark the robot's current position on the map (mid-gray)
 * Returns 0 on success, -1 on file error.
 */
int grid_save_pgm(const occupancy_grid_t *g, const char *path,
                  const pose_t *pose);

/*
 * grid_save_bin — write the grid to a compact binary file.
 *
 * Format: magic(4) | width(4) | height(4) | origin_x(4) | origin_y(4) | cells
 * Used for fast reload in patrol mode without re-exploring the space.
 * Returns 0 on success, -1 on file error.
 */
int grid_save_bin(const occupancy_grid_t *g, const char *path);

/*
 * grid_load_bin — load a previously saved binary grid file.
 *
 * Called by patrol_main at startup to restore the map from the last
 * mapping session without running the LiDAR again.
 * Returns 0 on success, -1 on error (wrong format or file missing).
 */
int grid_load_bin(occupancy_grid_t *g, const char *path);

#endif /* OCCUPANCY_GRID_H */