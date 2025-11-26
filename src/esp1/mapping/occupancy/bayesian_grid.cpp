#include "../../../../include/esp1/mapping/occupancy/bayesian_grid.h"
#include <cmath>
#include <algorithm>

#define LOG_ODDS_FREE     -0.4f
#define LOG_ODDS_OCCUPIED  0.4f
#define LOG_ODDS_MIN       -4.0f
#define LOG_ODDS_MAX        4.0f

// ------------------------------------------------------
// Constructor
// ------------------------------------------------------
BayesianOccupancyGrid::BayesianOccupancyGrid(float resolution_m,
                                             uint16_t size_x,
                                             uint16_t size_y)
{
    // Sécurité : ne jamais dépasser la taille max
    if (size_x > GRID_MAX_X) size_x = GRID_MAX_X;
    if (size_y > GRID_MAX_Y) size_y = GRID_MAX_Y;

    grid_resolution = resolution_m;
    grid_size_x = size_x;
    grid_size_y = size_y;
    log_odds = new float[grid_size_x * grid_size_y];

    // Initialise toute la grille à 0
    for (int i = 0; i < grid_size_x * grid_size_y; i++)
        log_odds[i] = 0.0f;

    // Precompute sin/cos tables
    for (int i = 0; i < 3600; i++) {
        float a = i * 0.1f * M_PI / 180.0f;
        sin_table[i] = sin(a);
        cos_table[i] = cos(a);
    }
}

// ------------------------------------------------------
// Update log occupancy grid
// ------------------------------------------------------
void BayesianOccupancyGrid::update_map(const LiDARScan& scan,
                                       const Pose2D& pose,
                                       const float lidar_max_range)
{
    for (uint16_t i = 0; i < scan.count; ++i)
    {
        float angle_lidar = scan.angles[i] * (M_PI / 180.0f);
        
        float r = scan.distances[i] * 0.001f;  // mm to m

        bool is_hit = (r > 0);
        if (!is_hit) {
            r = lidar_max_range;
        }
        
        // Convert to world angle
        float angle_world = pose.theta + angle_lidar;
        int idx = (int)(angle_world * 10.0f);
        float angle_lidar_sin = sin_table[idx];
        float angle_lidar_cos = cos_table[idx];

        // Compute world hit pos
        float hit_x = pose.x + r * angle_lidar_cos;
        float hit_y = pose.y + r * angle_lidar_sin;

        int x0 = (int)(pose.x / grid_resolution) + grid_size_x / 2;
        int x1 = (int)(hit_x / grid_resolution) + grid_size_x / 2;

        // NOTE: Y axis is inverted to match your map convention
        int y0 = (int)(-pose.y / grid_resolution) + grid_size_y / 2;
        int y1 = (int)(-hit_y / grid_resolution) + grid_size_y / 2;

        // ----------------------------------------------------------
        // Bresenham free cells
        // ----------------------------------------------------------
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        int x = x0;
        int y = y0;
        
        while (true)
        {
            if (x == x1 && y == y1)
                break;

            if (x >= 0 && x < grid_size_x && y >= 0 && y < grid_size_y)
            {
                int idx = y * grid_size_x + x;
                log_odds[idx] = std::max(LOG_ODDS_MIN, log_odds[idx] + LOG_ODDS_FREE);
            }

            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx)  { err += dx; y += sy; }
        }

        // ----------------------------------------------------------
        // Occupied cell at the ray end
        // ----------------------------------------------------------
        if (is_hit) {
            if (x1 >= 0 && x1 < grid_size_x && y1 >= 0 && y1 < grid_size_y)
            {
                int idx = y1 * grid_size_x + x1;
                log_odds[idx] = std::min(LOG_ODDS_MAX, log_odds[idx] + LOG_ODDS_OCCUPIED);
            }
        }
    }
}

// ------------------------------------------------------
// Probability (not log)
// ------------------------------------------------------
float BayesianOccupancyGrid::get_cell_probability(float x_idx,
                                                   float y_idx) const
{
    int x = (int)x_idx;
    int y = (int)y_idx;

    if (x < 0 || x >= grid_size_x || y < 0 || y >= grid_size_y)
        return 0.5f;

    float L = log_odds[y * grid_size_x + x];
    float odds = std::exp(L);
    return odds / (1.0f + odds);
}

// ------------------------------------------------------
// Export compressed map (0–255 grayscale)
// ------------------------------------------------------
const uint8_t* BayesianOccupancyGrid::get_map_data_color() const
{
    static uint8_t export_data[GRID_MAX_SIZE];

    for (int y = 0; y < grid_size_y; y++)
    {
        for (int x = 0; x < grid_size_x; x++)
        {
            float prob = get_cell_probability(x, y);
            float v = 255.0f * (1 - prob);

            if (v < 0) v = 0;
            if (v > 255) v = 255;

            export_data[y * grid_size_x + x] = (uint8_t)v;
        }
    }

    return export_data;
}

const float* BayesianOccupancyGrid::get_map_data() const
{
    return log_odds;
}
