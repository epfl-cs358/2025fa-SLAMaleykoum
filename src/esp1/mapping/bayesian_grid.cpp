#include "esp1/mapping/occupancy/bayesian_grid.h"
#include <Arduino.h>
#include <cmath>
#include <vector>
#include <algorithm>

// ------------------------------------------------------
// Constants for Bayesian Occupancy update
// ------------------------------------------------------
#define LOG_ODDS_FREE     -0.4f
#define LOG_ODDS_OCCUPIED  0.85f
#define LOG_ODDS_MIN       -4.0f
#define LOG_ODDS_MAX        4.0f


// ------------------------------------------------------
// Internal grid storage
// ------------------------------------------------------
static std::vector<float> log_odds;
static float grid_resolution = 0.05f;   // meters per cell
static uint16_t grid_size_x = 0;
static uint16_t grid_size_y = 0;


// ------------------------------------------------------
// Constructor
// ------------------------------------------------------
BayesianOccupancyGrid::BayesianOccupancyGrid(float resolution_m, uint16_t size_x, uint16_t size_y)
{
    grid_resolution = resolution_m;
    grid_size_x = size_x;
    grid_size_y = size_y;
    log_odds.assign(size_x * size_y, 0.0f);
}


// ------------------------------------------------------
// Update the occupancy map based on LiDAR and robot pose
// ------------------------------------------------------
void BayesianOccupancyGrid::update_map(const LiDARScan& scan, const Pose2D& pose)
{
    for (const auto& landmark : scan.landmarks)
    {
        // 1️⃣ Angle global
        float angle_world = pose.theta + landmark.angle;

        // 2️⃣ Coordonnées du point impacté (mètres)
        float hit_x = pose.x + landmark.range * std::cos(angle_world);
        float hit_y = pose.y + landmark.range * std::sin(angle_world);

        // 3️⃣ Conversion en indices du grid
        int x0 = static_cast<int>(std::floor(pose.x / grid_resolution)) + grid_size_x / 2;
        int y0 = static_cast<int>(std::floor(pose.y / grid_resolution)) + grid_size_y / 2;
        int x1 = static_cast<int>(std::floor(hit_x / grid_resolution)) + grid_size_x / 2;
        int y1 = static_cast<int>(std::floor(hit_y / grid_resolution)) + grid_size_y / 2;

        // 4️⃣ Algorithme de Bresenham : cellules libres
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

        // 5️⃣ Cellule finale occupée
        if (x1 >= 0 && x1 < grid_size_x && y1 >= 0 && y1 < grid_size_y)
        {
            int idx = y1 * grid_size_x + x1;
            log_odds[idx] = std::min(LOG_ODDS_MAX, log_odds[idx] + LOG_ODDS_OCCUPIED);
        }
    }
}


// ------------------------------------------------------
// Get probability (0.0 – 1.0) of a cell by grid indices
// ------------------------------------------------------
float BayesianOccupancyGrid::get_cell_probability(float x_idx, float y_idx) const
{
    int x = static_cast<int>(x_idx);
    int y = static_cast<int>(y_idx);

    if (x < 0 || x >= grid_size_x || y < 0 || y >= grid_size_y)
        return 0.5f; // unknown = neutral probability

    float L = log_odds[y * grid_size_x + x];
    float odds = std::exp(L);
    return odds / (1.0f + odds);
}


// ------------------------------------------------------
// Export map as 8-bit array (each value = 0–255)
// ------------------------------------------------------
const uint8_t* BayesianOccupancyGrid::get_map_data() const
{
    static std::vector<uint8_t> export_data;
    export_data.resize(grid_size_x * grid_size_y);

    for (int y = 0; y < grid_size_y; ++y)
    {
        for (int x = 0; x < grid_size_x; ++x)
        {
            float prob = get_cell_probability(x, y);
            float value = prob * 255.0f;

            // Clamp manuellement
            if (value < 0) value = 0;
            if (value > 255) value = 255;

            export_data[y * grid_size_x + x] = static_cast<uint8_t>(value);
        }
    }

    return export_data.data();
}
