#ifndef FRONTIER_H
#define FRONTIER_H

#include "occupancy_grid.h"
#include "ipc_common.h"

/* Frontier-based exploration (Yamauchi 1997).
   A frontier cell = free cell adjacent to unknown space.
   BFS from robot position through free cells only, so the
   returned target is always reachable without crossing walls.
   Returns 0 when no frontiers remain = map complete */
int frontier_find_nearest(const occupancy_grid_t *g,
                          const pose_t            *pose,
                          float                   *target_x_mm,
                          float                   *target_y_mm);

#endif
