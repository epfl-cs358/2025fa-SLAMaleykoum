#include "../../include/common/utils.h"

void grid_to_world(int grid_pos_x, int grid_pos_y, float& x, float& y, float resolution, int width, int height) {
    x = float((grid_pos_x - float(width)/2.0 + 0.5f) * resolution);
    y = - float((grid_pos_y - float(height)/2.0 + 0.5f) * resolution); 
}

void world_to_grid(float x, float y, int& grid_pos_x, int& grid_pos_y, float resolution, int width, int height) {
    grid_pos_x = width/2 + (int)(x / resolution);
    grid_pos_y = height/2 - (int)(y / resolution);
}

bool is_frontier_cell(const BayesianOccupancyGrid& grid, int x, int y)
{
    if (grid.get_cell_probability(x, y) > FREE_BOUND_PROB) return false;
    
    // Check Neighbors for Unknowns
    static int dx[4] = {1, -1, 0, 0};
    static int dy[4] = {0, 0, 1, -1};

    for (int i = 0; i < 4; i++) {
        float np = grid.get_cell_probability(x + dx[i], y + dy[i]);
        
        if (np > FREE_BOUND_PROB && np < OCC_BOUND_PROB) return true;
    }

    return false;
}