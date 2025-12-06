#include "../../../../include/esp1/mapping/occupancy/bayesian_grid.h"
#include <cmath>
#include <algorithm>
#include <cstdint>

#define L_FREE_INT      -4     // -0.4  * 10
#define L_OCC_INT       +4     // +0.4  * 10
#define L_MIN_INT      -40     // -4.0  * 10
#define L_MAX_INT       40     // +4.0  * 10

float BayesianOccupancyGrid::sin_table[3600];
float BayesianOccupancyGrid::cos_table[3600];
bool  BayesianOccupancyGrid::trig_initialized = false;

float BayesianOccupancyGrid::prob_table[81];
bool  BayesianOccupancyGrid::prob_table_initialized = false;

// ------------------------------------------------------
// Constructor
// ------------------------------------------------------
BayesianOccupancyGrid::BayesianOccupancyGrid(float resolution_m,
                                             uint16_t size_x,
                                             uint16_t size_y)
{
    if (size_x > GRID_MAX_X) size_x = GRID_MAX_X;
    if (size_y > GRID_MAX_Y) size_y = GRID_MAX_Y;

    grid_resolution = resolution_m;
    grid_size_x = size_x;
    grid_size_y = size_y;

    // Allocate compact int8 log-odds map
    log_odds = new int8_t[grid_size_x * grid_size_y];
    memset(log_odds, 0, grid_size_x * grid_size_y);

    // Precompute sin/cos tables


    if(!trig_initialized){
        for (int i = 0; i < 3600; i++)
        {
            float a = i * 0.1f * M_PI / 180.0f;
            sin_table[i] = sinf(a);
            cos_table[i] = cosf(a);
        }
        trig_initialized = true;
    }

    if (!prob_table_initialized)
    {
        for (int i = -40; i <= 40; ++i)
        {
            float L = i * 0.1f;       // log-odds in real units
            float odds = expf(L);
            prob_table[i + 40] = odds / (1.0f + odds);
        }

        prob_table_initialized = true;
    }
}

// ------------------------------------------------------
// Update log occupancy grid
// ------------------------------------------------------
void BayesianOccupancyGrid::update_map(const SyncedScan& lidar_scan,
                                       const float lidar_max_range)
{
    const LiDARScan& scan = lidar_scan.scan; 
    const Pose2D& pose = lidar_scan.pose; 

    float pose_deg = pose.theta * (180.0f/M_PI);

    for (uint16_t i = 0; i < scan.count; ++i)
    {
        float r = scan.distances[i] * 0.001f;  // mm → m
        bool is_hit = (r > 0);
        if (!is_hit)
            r = lidar_max_range;

        // -----------------------------
        // FAST ANGLE NORMALIZATION
        // -----------------------------
        float angle_world = pose_deg + scan.angles[i];

        while (angle_world >= 360.0f) angle_world -= 360.0f;
        while (angle_world <   0.0f)  angle_world += 360.0f;

        int idx = (int)(angle_world * 10.0f);  // 0.1° resolution → [0,3599]

        // Fast integer wrap instead of modulo
        if (idx >= 3600) idx -= 3600;
        if (idx < 0)     idx += 3600;

        float s = sin_table[idx];
        float c = cos_table[idx];

        // Compute world hit point
        float hit_x = pose.x + r * c;
        float hit_y = pose.y + r * s;

        int x0 = (int)(pose.x / grid_resolution) + grid_size_x / 2;
        int x1 = (int)(hit_x / grid_resolution) + grid_size_x / 2;

        int y0 = (int)(-pose.y / grid_resolution) + grid_size_y / 2;
        int y1 = (int)(-hit_y / grid_resolution) + grid_size_y / 2;

        // ------------------------------------------------------
        // Bresenham free cells
        // ------------------------------------------------------
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        int x = x0;
        int y = y0;

        while (true)
        {
            if (x == x1 && y == y1)
                break;

            if ((unsigned)x < grid_size_x && (unsigned)y < grid_size_y)
            {
                int idxg = y * grid_size_x + x;
                int v = log_odds[idxg] + L_FREE_INT;

                v = std::max((int)L_MIN_INT, std::min((int)L_MAX_INT, v));
                log_odds[idxg] = (int8_t)v;
            }

            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx)  { err += dx; y += sy; }
        }

        // ------------------------------------------------------
        // Occupied cell at ray end
        // ------------------------------------------------------
        if (is_hit &&
            (unsigned)x1 < grid_size_x &&
            (unsigned)y1 < grid_size_y)
        {
            int idxg = y1 * grid_size_x + x1;
            int v = log_odds[idxg] + L_OCC_INT;

            v = std::max((int)L_MIN_INT, std::min((int)L_MAX_INT, v));
            log_odds[idxg] = (int8_t)v;
        }
    }
}

// ------------------------------------------------------
// convert int8 log-odds → probability
// ------------------------------------------------------
float BayesianOccupancyGrid::get_cell_probability(float x_idx,
                                                   float y_idx) const
{
    int x = (int)x_idx;
    int y = (int)y_idx;

    // Out of bounds → unknown = 0.5
    if ((unsigned)x >= grid_size_x || (unsigned)y >= grid_size_y)
        return 0.5f;

    int idx = y * grid_size_x + x;

    // log_odds ∈ [-40 .. +40] → shift to [0 .. 80]
    return prob_table[ log_odds[idx] + 40 ];
}

// ------------------------------------------------------
// Export grayscale map
// ------------------------------------------------------
const uint8_t* BayesianOccupancyGrid::get_map_data_color() const
{
    static uint8_t export_data[GRID_MAX_SIZE];

    for (int y = 0; y < grid_size_y; y++)
    {
        for (int x = 0; x < grid_size_x; x++)
        {
            float p = get_cell_probability(x, y);

            float v = 255.0f * (1.0f - p);
            if (v < 0) v = 0;
            if (v > 255) v = 255;

            export_data[y * grid_size_x + x] = (uint8_t)v;
        }
    }
    return export_data;
}

// ------------------------------------------------------
// Return raw int8 grid
// ------------------------------------------------------
const int8_t* BayesianOccupancyGrid::get_map_data() const
{
    return log_odds;
}
