#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <stdint.h>
#include "config.h"
#include "ipc_common.h"

/* Log-odds occupancy grid.
   Cell value meaning:
     0        = unknown, never observed
     negative = evidence of free space
     positive = evidence of obstacle
   Evidence accumulates across scans - one noisy reading means
   little, fifty scans of the same wall are overwhelming */
typedef struct {
    int8_t cells[GRID_HEIGHT_CELLS][GRID_WIDTH_CELLS];
    int    width;
    int    height;
    int    origin_x;   /* grid cell corresponding to world (0,0) */
    int    origin_y;
} occupancy_grid_t;

void grid_init(occupancy_grid_t *g);

/* Integrates one scan at the given pose. Pose is what anchors
   each scan to the right map position - odometry drift here
   is the main source of map error (what SLAM would correct) */
void grid_update(occupancy_grid_t *g, const scan_t *scan, const pose_t *pose);

int grid_save_pgm(const occupancy_grid_t *g, const char *path,
                  const pose_t *pose);
int grid_save_bin(const occupancy_grid_t *g, const char *path);
int grid_load_bin(occupancy_grid_t *g, const char *path);

#endif
