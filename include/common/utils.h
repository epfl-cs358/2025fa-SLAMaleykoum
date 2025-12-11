#pragma once
#include "data_types.h"

void grid_to_world(int grid_pos_x, int grid_pos_y, float& x, float& y, float resolution, int width, int height);

void world_to_grid(float x, float y, int& grid_pos_x, int& grid_pos_y, float resolution, int width, int height);