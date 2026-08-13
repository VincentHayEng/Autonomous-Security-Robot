/* Log-odds occupancy grid with Bresenham ray casting.

   Coordinate system:
     World: origin = power-on position, +Y forward, +X right, mm
     Grid:  row 0 at top, row increases downward (Y flipped) */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <sys/stat.h>

#include "occupancy_grid.h"
#include "config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Magic number "GRID" - lets load_bin reject non-map files */
#define GRID_MAGIC 0x47524944u

/* Promote to int before adding to avoid int8 overflow,
   clamp so no cell gets permanently locked by evidence */
static void clamp_add(int8_t *v, int delta, int lo, int hi)
{
    int result = (int)(*v) + delta;
    if      (result > hi) result = hi;
    else if (result < lo) result = lo;
    *v = (int8_t)result;
}

/* Bresenham line algorithm - steps through every cell on a
   straight line using only integer math. No floats in the
   inner loop is what makes 300 rays x 10Hz affordable.
   Cells along the ray = FREE, endpoint = OCCUPIED */
static void cast_ray(occupancy_grid_t *g, int x0, int y0, int x1, int y1)
{
    int dx  = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int dy  = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    int sx  = (x0 < x1) ? 1 : -1;
    int sy  = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    for (;;) {
        int in_bounds = (x0 >= 0 && x0 < g->width &&
                         y0 >= 0 && y0 < g->height);
        int at_end = (x0 == x1 && y0 == y1);

        if (in_bounds) {
            if (at_end) {
                clamp_add(&g->cells[y0][x0],
                          LOG_ODDS_OCCUPIED, LOG_ODDS_MIN, LOG_ODDS_MAX);
            } else {
                clamp_add(&g->cells[y0][x0],
                          LOG_ODDS_FREE, LOG_ODDS_MIN, LOG_ODDS_MAX);
            }
        }

        if (at_end) break;

        /* Error accumulator decides which axis steps.
           Both can step in one iteration (diagonal) */
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

void grid_init(occupancy_grid_t *g)
{
    memset(g->cells, 0, sizeof(g->cells));   /* all unknown */
    g->width    = GRID_WIDTH_CELLS;
    g->height   = GRID_HEIGHT_CELLS;
    g->origin_x = GRID_WIDTH_CELLS  / 2;     /* robot starts at center */
    g->origin_y = GRID_HEIGHT_CELLS / 2;
}

void grid_update(occupancy_grid_t *g, const scan_t *scan, const pose_t *pose)
{
    /* Pose to grid cell. Integer division truncates - correct,
       we want cell boundaries not rounded positions */
    int robot_col = g->origin_x + (int)(pose->x_mm / GRID_CELL_SIZE_MM);
    int robot_row = g->origin_y - (int)(pose->y_mm / GRID_CELL_SIZE_MM);

    float theta = (float)pose->theta_mrad / 1000.0f;

    for (int a = 0; a < SCAN_POINTS; a++) {
        if (!scan->valid[a]) continue;

        float dist = scan->distance_mm[a];
        float angle_rad = (float)a * (float)M_PI / 180.0f;

        /* Polar to Cartesian in robot frame.
           sin/cos assignment looks swapped because our 0 deg
           points forward (+Y), not along +X */
        float xr = dist * sinf(angle_rad);
        float yr = dist * cosf(angle_rad);

        /* Standard 2D rotation by robot heading, then translate
           by robot position = robot frame to world frame.
           At theta=0 this collapses to identity + offset */
        float xw = xr * cosf(theta) - yr * sinf(theta) + (float)pose->x_mm;
        float yw = xr * sinf(theta) + yr * cosf(theta) + (float)pose->y_mm;

        int end_col = g->origin_x + (int)(xw / GRID_CELL_SIZE_MM);
        int end_row = g->origin_y - (int)(yw / GRID_CELL_SIZE_MM);

        cast_ray(g, robot_col, robot_row, end_col, end_row);
    }
}

int grid_save_pgm(const occupancy_grid_t *g, const char *path,
                  const pose_t *pose)
{
    mkdir("maps", 0755);   /* fails harmlessly if it exists */

    FILE *f = fopen(path, "w");
    if (!f) {
        perror("grid_save_pgm: fopen");
        return -1;
    }

    /* P2 = ASCII grayscale, chosen over binary P5 so the file
       is readable in a text editor for debugging */
    fprintf(f, "P2\n%d %d\n255\n", g->width, g->height);

    int robot_col = g->origin_x + (int)(pose->x_mm / GRID_CELL_SIZE_MM);
    int robot_row = g->origin_y - (int)(pose->y_mm / GRID_CELL_SIZE_MM);

    for (int row = 0; row < g->height; row++) {
        for (int col = 0; col < g->width; col++) {
            int px;
            if (row == robot_row && col == robot_col) {
                px = 128;                       /* robot marker */
            } else {
                int8_t v = g->cells[row][col];
                if      (v > 0) px = 0;         /* occupied = black */
                else if (v < 0) px = 255;       /* free = white */
                else            px = 180;       /* unknown = gray */
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

    FILE *f = fopen(path, "wb");
    if (!f) { perror("grid_save_bin: fopen"); return -1; }

    uint32_t magic = GRID_MAGIC;
    fwrite(&magic,       sizeof(uint32_t), 1, f);
    fwrite(&g->width,    sizeof(int),      1, f);
    fwrite(&g->height,   sizeof(int),      1, f);
    fwrite(&g->origin_x, sizeof(int),      1, f);
    fwrite(&g->origin_y, sizeof(int),      1, f);
    fwrite(g->cells, sizeof(g->cells), 1, f);

    fclose(f);
    return 0;
}

int grid_load_bin(occupancy_grid_t *g, const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror("grid_load_bin: fopen"); return -1; }

    uint32_t magic;
    fread(&magic, sizeof(uint32_t), 1, f);
    if (magic != GRID_MAGIC) {
        fprintf(stderr, "grid_load_bin: bad magic - not a grid file\n");
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
