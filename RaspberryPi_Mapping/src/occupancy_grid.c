/*
 * occupancy_grid.c — 2D log-odds occupancy grid implementation.
 *
 * ── Coordinate system ───────────────────────────────────────────────────────
 *
 *  World frame (millimetres):
 *    Origin (0, 0) = robot power-on position.
 *    +X = right.
 *    +Y = forward (away from robot's initial rear).
 *
 *  Grid frame (cell indices):
 *    col = origin_x + (x_mm / CELL_SIZE)
 *    row = origin_y - (y_mm / CELL_SIZE)   ← Y is flipped because screen rows
 *                                             increase downward while world Y
 *                                             increases forward (upward on map).
 *
 * ── Bresenham ray casting ───────────────────────────────────────────────────
 *
 *  Bresenham's line algorithm steps through every grid cell that a straight
 *  line passes through between two points. Originally designed for drawing
 *  pixels, it works identically for grid cells.
 *
 *  For each LiDAR ray:
 *    - Start cell = robot's current grid position.
 *    - End cell   = where the ray hit an obstacle (or max range).
 *    - Every cell from start to (end - 1): log-odds += LOG_ODDS_FREE (negative).
 *    - The end cell:                       log-odds += LOG_ODDS_OCCUPIED (positive).
 *
 *  Over many scans, free cells accumulate negative values and occupied cells
 *  accumulate positive values. The MIN/MAX clamp prevents runaway.
 */

#include <stdio.h>       /* fopen, fprintf, fclose, perror                   */
#include <stdlib.h>      /* NULL                                              */
#include <string.h>      /* memset, memcpy                                    */
#include <math.h>        /* sinf, cosf                                        */
#include <stdint.h>      /* int8_t, uint32_t                                  */
#include <sys/stat.h>    /* mkdir                                             */

#include "occupancy_grid.h"
#include "config.h"

/* M_PI is not guaranteed by C11 without _DEFAULT_SOURCE or _GNU_SOURCE.
 * Define it here as a fallback so the code compiles regardless.           */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Magic number written at the start of binary map files.
 * Spells "GRID" in ASCII. Lets grid_load_bin verify the file is valid
 * before trying to interpret its contents as cell data.                   */
#define GRID_MAGIC 0x47524944u

/* ── Static helper functions (private to this file) ─────────────────────── */

/*
 * clamp_add — add delta to *v and clamp the result to [lo, hi].
 *
 * int8_t can hold -128 to 127. Adding to it can overflow if we're not
 * careful. We promote to int for the arithmetic, clamp, then write back.
 * This is called for every cell in every Bresenham trace, so it must be fast.
 */
static void clamp_add(int8_t *v, int delta, int lo, int hi)
{
    int result = (int)(*v) + delta;
    if      (result > hi) result = hi;
    else if (result < lo) result = lo;
    *v = (int8_t)result;
}

/*
 * cast_ray — Bresenham line from (x0,y0) to (x1,y1) in grid coordinates.
 *
 * Steps through every cell on the line. Marks intermediate cells FREE
 * and the endpoint cell OCCUPIED. Silently skips cells outside the grid
 * bounds — the algorithm still terminates correctly at (x1, y1).
 *
 * Parameters are all in grid cell units (not mm).
 */
static void cast_ray(occupancy_grid_t *g, int x0, int y0, int x1, int y1)
{
    /* Bresenham setup.
     * dx, dy: absolute deltas in each axis.
     * sx, sy: step direction (+1 or -1) for each axis.
     * err:    error accumulator — drives which axis steps on each iteration. */
    int dx  = (x1 > x0) ? (x1 - x0) : (x0 - x1);   /* abs(x1-x0) */
    int dy  = (y1 > y0) ? (y1 - y0) : (y0 - y1);   /* abs(y1-y0) */
    int sx  = (x0 < x1) ? 1 : -1;
    int sy  = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    for (;;) {
        /* Check whether current cell is within grid bounds.
         * Out-of-bounds cells are simply skipped — no write, no crash.   */
        int in_bounds = (x0 >= 0 && x0 < g->width &&
                         y0 >= 0 && y0 < g->height);

        /* Check whether we have reached the endpoint.                    */
        int at_end = (x0 == x1 && y0 == y1);

        if (in_bounds) {
            if (at_end) {
                /* Endpoint: something blocked the ray here. Mark occupied. */
                clamp_add(&g->cells[y0][x0],
                          LOG_ODDS_OCCUPIED, LOG_ODDS_MIN, LOG_ODDS_MAX);
            } else {
                /* Intermediate: ray passed through. Mark free.            */
                clamp_add(&g->cells[y0][x0],
                          LOG_ODDS_FREE, LOG_ODDS_MIN, LOG_ODDS_MAX);
            }
        }

        /* Always break at the endpoint, even if out of bounds.
         * Without this break, the loop would run forever.                */
        if (at_end) break;

        /* Bresenham step.
         * e2 is used twice so we compute it once before modifying err.   */
        int e2 = 2 * err;

        /* If the error favours X, step in X and adjust the accumulator.  */
        if (e2 > -dy) { err -= dy; x0 += sx; }

        /* If the error favours Y, step in Y and adjust the accumulator.
         * Both conditions can fire in the same iteration for diagonal lines. */
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/* ── Public function implementations ─────────────────────────────────────── */

void grid_init(occupancy_grid_t *g)
{
    /* Zero all cells. int8_t 0 = unknown (no observations yet).          */
    memset(g->cells, 0, sizeof(g->cells));

    g->width    = GRID_WIDTH_CELLS;
    g->height   = GRID_HEIGHT_CELLS;

    /* Place the origin at the exact center of the grid.
     * Integer division: 200/2 = 100.
     * Cell (100, 100) corresponds to world position (0mm, 0mm).          */
    g->origin_x = GRID_WIDTH_CELLS  / 2;
    g->origin_y = GRID_HEIGHT_CELLS / 2;
}

void grid_update(occupancy_grid_t *g, const scan_t *scan, const pose_t *pose)
{
    /*
     * Convert robot pose from world millimetres to grid cell indices.
     *
     * Integer division truncates toward zero — that is correct here.
     * A robot at x=24mm with CELL_SIZE=50mm is in the same cell as x=0mm.
     * We want cell boundaries, not rounded positions.
     */
    int robot_col = g->origin_x + (int)(pose->x_mm / GRID_CELL_SIZE_MM);
    int robot_row = g->origin_y - (int)(pose->y_mm / GRID_CELL_SIZE_MM);

    /*
     * Convert heading from milliradians to radians.
     * theta_mrad = 1000 means 1 radian of rotation.
     * At startup the stub provides 0, meaning the robot faces +Y (forward).
     */
    float theta = (float)pose->theta_mrad / 1000.0f;

    for (int a = 0; a < SCAN_POINTS; a++) {
        /* Skip angles with no valid reading.                              */
        if (!scan->valid[a]) continue;

        float dist = scan->distance_mm[a];

        /*
         * Convert the scan angle from degrees to radians.
         *
         * In our coordinate system, angle 0° points forward (+Y).
         * Converting: radians = degrees × π / 180
         */
        float angle_rad = (float)a * (float)M_PI / 180.0f;

        /*
         * Polar → Cartesian in robot frame.
         *
         * With 0° = forward = +Y axis:
         *   x_robot = dist × sin(angle)   (positive = right)
         *   y_robot = dist × cos(angle)   (positive = forward)
         *
         * This matches the Python prototype exactly.
         */
        float xr = dist * sinf(angle_rad);
        float yr = dist * cosf(angle_rad);

        /*
         * Rotate from robot frame to world frame.
         *
         * Standard 2D rotation by angle theta:
         *   x_world = xr × cos(theta) - yr × sin(theta)
         *   y_world = xr × sin(theta) + yr × cos(theta)
         *
         * Then translate by the robot's world position.
         * At theta=0 (straight ahead, no rotation), cos=1 and sin=0,
         * so this simplifies to x_world = xr + pose_x, y_world = yr + pose_y.
         */
        float xw = xr * cosf(theta) - yr * sinf(theta) + (float)pose->x_mm;
        float yw = xr * sinf(theta) + yr * cosf(theta) + (float)pose->y_mm;

        /*
         * World millimetres → grid cell indices.
         * Same formula as the robot position above, but for the ray endpoint.
         */
        int end_col = g->origin_x + (int)(xw / GRID_CELL_SIZE_MM);
        int end_row = g->origin_y - (int)(yw / GRID_CELL_SIZE_MM);

        /* Cast the ray: free along the path, occupied at the endpoint.   */
        cast_ray(g, robot_col, robot_row, end_col, end_row);
    }
}

int grid_save_pgm(const occupancy_grid_t *g, const char *path,
                  const pose_t *pose)
{
    /*
     * Ensure the output directory exists.
     * mkdir() with 0755 creates the directory with standard permissions.
     * We ignore the return value because it fails (harmlessly) if the
     * directory already exists.
     */
    mkdir("maps", 0755);

    FILE *f = fopen(path, "w");
    if (!f) {
        perror("grid_save_pgm: fopen");
        return -1;
    }

    /*
     * PGM header.
     * "P2" = ASCII grayscale (human readable, slightly larger than binary P5).
     * We use P2 so you can open the file in a text editor and see the numbers.
     * Line 2: width and height in pixels.
     * Line 3: maximum pixel value (255 = full white).
     */
    fprintf(f, "P2\n%d %d\n255\n", g->width, g->height);

    /* Calculate robot cell for the position marker.                      */
    int robot_col = g->origin_x + (int)(pose->x_mm / GRID_CELL_SIZE_MM);
    int robot_row = g->origin_y - (int)(pose->y_mm / GRID_CELL_SIZE_MM);

    for (int row = 0; row < g->height; row++) {
        for (int col = 0; col < g->width; col++) {
            int px;

            if (row == robot_row && col == robot_col) {
                px = 128;  /* Robot position — mid gray                   */
            } else {
                int8_t v = g->cells[row][col];
                if      (v > 0) px = 0;    /* Occupied — black           */
                else if (v < 0) px = 255;  /* Free — white               */
                else            px = 180;  /* Unknown — light gray       */
            }

            fprintf(f, "%d ", px);
        }
        fprintf(f, "\n");
    }

    fclose(f);
    return 0;
}

int grid_save_bin(const occupancy_grid_t *g, const char *path)
{
    mkdir("maps", 0755);

    FILE *f = fopen(path, "wb");  /* "wb" = write binary                  */
    if (!f) { perror("grid_save_bin: fopen"); return -1; }

    /*
     * Write header fields individually.
     * fwrite(pointer, size_of_one_item, number_of_items, file)
     * Returns number of items written — we ignore errors here for brevity.
     */
    uint32_t magic = GRID_MAGIC;
    fwrite(&magic,       sizeof(uint32_t), 1, f);
    fwrite(&g->width,    sizeof(int),      1, f);
    fwrite(&g->height,   sizeof(int),      1, f);
    fwrite(&g->origin_x, sizeof(int),      1, f);
    fwrite(&g->origin_y, sizeof(int),      1, f);

    /* Write all cell data in one call.
     * sizeof(g->cells) = GRID_HEIGHT × GRID_WIDTH × sizeof(int8_t) = 40000 bytes
     * for a 200×200 grid.                                                */
    fwrite(g->cells, sizeof(g->cells), 1, f);

    fclose(f);
    return 0;
}

int grid_load_bin(occupancy_grid_t *g, const char *path)
{
    FILE *f = fopen(path, "rb");  /* "rb" = read binary                   */
    if (!f) { perror("grid_load_bin: fopen"); return -1; }

    /* Read and verify magic number before trusting any other data.       */
    uint32_t magic;
    fread(&magic, sizeof(uint32_t), 1, f);
    if (magic != GRID_MAGIC) {
        fprintf(stderr, "grid_load_bin: bad magic number — not a grid file\n");
        fclose(f);
        return -1;
    }

    fread(&g->width,    sizeof(int), 1, f);
    fread(&g->height,   sizeof(int), 1, f);
    fread(&g->origin_x, sizeof(int), 1, f);
    fread(&g->origin_y, sizeof(int), 1, f);
    fread(g->cells,     sizeof(g->cells), 1, f);

    fclose(f);
    return 0;
}