#include "../../include/common/utils.h"

void grid_to_world(int grid_pos_x, int grid_pos_y, float& x, float& y, float resolution, int width, int height) {
    x = float((grid_pos_x - float(width)/2.) * resolution);
    y = - float((grid_pos_y - float(height)/2.) * resolution); 
}

void world_to_grid(float x, float y, int& grid_pos_x, int& grid_pos_y, float resolution, int width, int height) {
    grid_pos_x = width/2 + (int)(x / resolution);
    grid_pos_y = height/2 - (int)(y / resolution);
}