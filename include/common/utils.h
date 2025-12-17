#pragma once
#include "data_types.h"
#include "esp1/mapping/bayesian_grid.h"

void grid_to_world(int grid_pos_x, int grid_pos_y, float& x, float& y, float resolution, int width, int height);

void world_to_grid(float x, float y, int& grid_pos_x, int& grid_pos_y, float resolution, int width, int height);

/**
 * @brief determines if a cell is frontier or not
 * A frontier is a free (explored) space that has at least one unknown (between free 
 * and occupied) cell around her.
 * 
 * @return true if it is a frontiere, false if not.
 */
bool is_frontier_cell(const BayesianOccupancyGrid& grid, int x, int y);

bool is_frontier_cell_snapshot(const OccupancyGridSnapshot& grid, int x, int y);

float get_cell_probability_snapshot(const OccupancyGridSnapshot& snap, int x, int y);