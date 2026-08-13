/* Frontier-based exploration - Yamauchi 1997.
   BFS outward from robot through free cells only.
   First frontier found = nearest reachable unexplored boundary */

#include <string.h>
#include <math.h>
#include <float.h>
#include <stdint.h>

#include "frontier.h"
#include "config.h"

#define MAX_CELLS (GRID_WIDTH_CELLS * GRID_HEIGHT_CELLS)

typedef struct { int col, row; } cell_t;

/* Static, not stack: queue + visited = ~360KB, would overflow
   the default 8MB thread stack margin under SCHED_FIFO.
   Tradeoff: not reentrant, fine since one thread calls this */
static cell_t   bfs_queue[MAX_CELLS];
static uint8_t  visited[GRID_HEIGHT_CELLS][GRID_WIDTH_CELLS];

/* 4-connectivity, no diagonals - a physical robot can't
   thread through a diagonal gap between two obstacles */
static const int DR[] = {-1,  1,  0,  0};
static const int DC[] = { 0,  0, -1,  1};

int frontier_find_nearest(const occupancy_grid_t *g,
                          const pose_t            *pose,
                          float                   *target_x_mm,
                          float                   *target_y_mm)
{
    int robot_col = g->origin_x + (int)(pose->x_mm / GRID_CELL_SIZE_MM);
    int robot_row = g->origin_y - (int)(pose->y_mm / GRID_CELL_SIZE_MM);

    /* Clamp against float truncation edge cases */
    if (robot_col <  0)         robot_col = 0;
    if (robot_col >= g->width)  robot_col = g->width  - 1;
    if (robot_row <  0)         robot_row = 0;
    if (robot_row >= g->height) robot_row = g->height - 1;

    memset(visited, 0, sizeof(visited));

    int head = 0, tail = 0;
    float best_dist = FLT_MAX;
    int   best_col  = -1;
    int   best_row  = -1;

    bfs_queue[tail].col = robot_col;
    bfs_queue[tail].row = robot_row;
    tail++;
    visited[robot_row][robot_col] = 1;

    while (head < tail) {
        int col = bfs_queue[head].col;
        int row = bfs_queue[head].row;
        head++;

        /* Only traverse free cells - BFS physically cannot route
           through walls, so any frontier found is reachable.
           Start cell exempt: may lack evidence early in mapping */
        int is_start = (col == robot_col && row == robot_row);
        int is_free  = (g->cells[row][col] < 0);
        if (!is_free && !is_start) continue;

        /* Frontier test: free cell with at least one unknown
           neighbor = boundary between mapped and unmapped */
        int is_frontier = 0;
        for (int d = 0; d < 4; d++) {
            int nr = row + DR[d];
            int nc = col + DC[d];
            if (nr >= 0 && nr < g->height &&
                nc >= 0 && nc < g->width  &&
                g->cells[nr][nc] == 0) {
                is_frontier = 1;
                break;
            }
        }

        if (is_frontier) {
            float dcol = (float)(col - robot_col);
            float drow = (float)(row - robot_row);
            float dist = sqrtf(dcol * dcol + drow * drow);
            if (dist < best_dist) {
                best_dist = dist;
                best_col  = col;
                best_row  = row;
            }
        }

        /* Expand the BFS ring through unvisited free neighbors */
        for (int d = 0; d < 4; d++) {
            int nr = row + DR[d];
            int nc = col + DC[d];
            if (nr < 0 || nr >= g->height) continue;
            if (nc < 0 || nc >= g->width)  continue;
            if (visited[nr][nc])            continue;
            if (g->cells[nr][nc] >= 0)     continue;
            visited[nr][nc] = 1;
            if (tail < MAX_CELLS) {
                bfs_queue[tail].col = nc;
                bfs_queue[tail].row = nr;
                tail++;
            }
        }
    }

    /* Queue drained with no frontier = every reachable cell
       observed = map is mathematically complete */
    if (best_col == -1) return 0;

    /* Cell back to world mm - reverse of the pose transform */
    *target_x_mm = (float)(best_col - g->origin_x) * (float)GRID_CELL_SIZE_MM;
    *target_y_mm = (float)(g->origin_y - best_row)  * (float)GRID_CELL_SIZE_MM;
    return 1;
}
