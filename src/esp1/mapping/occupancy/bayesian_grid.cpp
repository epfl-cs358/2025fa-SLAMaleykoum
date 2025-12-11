#include "../../../../include/esp1/mapping/occupancy/bayesian_grid.h"
#include <cmath>
// #include <algorithm>
// #include <cstdint>
#include "../../include/common/utils.h"


#define L_FREE_INT      -6     // -0.4  * 10
#define L_OCC_INT       +15     // +0.4  * 10
#define L_MIN_INT      -40     // -4.0  * 10
#define L_MAX_INT       40     // +4.0  * 10

float* BayesianOccupancyGrid::sin_table = nullptr;
float* BayesianOccupancyGrid::cos_table = nullptr;
bool  BayesianOccupancyGrid::trig_initialized = false;

float* BayesianOccupancyGrid::prob_table = nullptr;
bool  BayesianOccupancyGrid::prob_table_initialized = false;

// ------------------------------------------------------
// Constructor
// ------------------------------------------------------
BayesianOccupancyGrid::BayesianOccupancyGrid(float resolution_m, uint16_t size_x, uint16_t size_y) 
    : grid_size_x(size_x), grid_size_y(size_y), grid_resolution(resolution_m)
{

    log_odds = (int8_t*) heap_caps_malloc(grid_size_x * grid_size_y * sizeof(int8_t), MALLOC_CAP_SPIRAM);
    memset(log_odds, 0, grid_size_x * grid_size_y);

    // Precompute sin/cos tables
    if(!trig_initialized){
        sin_table = (float*) heap_caps_malloc(3600 * sizeof(float), MALLOC_CAP_SPIRAM);
        cos_table = (float*) heap_caps_malloc(3600 * sizeof(float), MALLOC_CAP_SPIRAM);
        
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
        prob_table = (float*) heap_caps_malloc(81 * sizeof(float), MALLOC_CAP_SPIRAM);
        for (int i = L_MIN_INT; i <= L_MAX_INT; ++i)
        {
            float L = i * 0.1f;       // log-odds in real units
            float odds = expf(L);
            prob_table[i + L_MAX_INT] = odds / (1.0f + odds);
        }

        prob_table_initialized = true;
    }
}

// ------------------------------------------------------
// Update log occupancy grid
// ------------------------------------------------------
void BayesianOccupancyGrid::update_map(const SyncedScan& lidar_scan)
{
    const LiDARScan& scan = lidar_scan.scan; 
    const Pose2D& pose = lidar_scan.pose; 

    // Precompute pose angle in degrees
    float pose_deg = pose.theta * (180.0f/M_PI) + 90.0f;

    for (uint16_t i = 0; i < scan.count; i++)
    {
        float rayon = scan.distances[i] * 0.001f;  // mm → m
        bool is_hit = (0 < rayon && rayon < SEARCH_BOUND_M);
        rayon = is_hit ? rayon : SEARCH_BOUND_M;

        // -----------------------------
        // FAST ANGLE NORMALIZATION
        // -----------------------------
        float angle_world = pose_deg - scan.angles[i];

        // SAFETY: Check for NaN (Not a Number) to prevent random crash
        if (isnan(angle_world)) continue;

        while (angle_world >= 360.0f) angle_world -= 360.0f;
        while (angle_world <   0.0f)  angle_world += 360.0f;

        int idx = (int)(angle_world * 10.0f);  // 0.1° resolution → [0,3599]

        // Fast integer wrap instead of modulo
        if (idx >= 3600) idx -= 3600;
        if (idx < 0)     idx += 3600;

        float sin = sin_table[idx];
        float cos = cos_table[idx];

        // Compute world hit point
        float hit_x = pose.x + rayon * cos;
        float hit_y = pose.y + rayon * sin;

        int x0, y0, xhit, yhit; 
        // int x0 = (int)(pose.x / grid_resolution) + grid_size_x / 2;
        // int xhit = (int)(hit_x / grid_resolution) + grid_size_x / 2;

        // // This ensures +Y (North) goes to Index 0 (Top)
        // int y0 = grid_size_y / 2 - (int)(pose.y / grid_resolution);
        // int yhit = grid_size_y / 2 - (int)(hit_y / grid_resolution);

        world_to_grid(pose.x, pose.y, x0, y0, grid_resolution, grid_size_x, grid_size_y);
        world_to_grid(hit_x, hit_y, xhit, yhit, grid_resolution, grid_size_x, grid_size_y);

        // ------------------------------------------------------
        // Bresenham free cells
        // ------------------------------------------------------
        int dx = abs(xhit - x0);
        int dy = abs(yhit - y0);
        int sx = (x0 < xhit) ? 1 : -1;
        int sy = (y0 < yhit) ? 1 : -1;
        int err = dx - dy;

        int x = x0;
        int y = y0;

        while (!(x == xhit && y == yhit)) {
            if ((unsigned)x < grid_size_x && (unsigned)y < grid_size_y) {
                int idxg = y * grid_size_x + x;

                int v = log_odds[idxg] + L_FREE_INT;
                log_odds[idxg] = (int8_t) (L_MIN_INT > v ? L_MIN_INT : v);
            }

            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx)  { err += dx; y += sy; }
        }

        // ------------------------------------------------------
        // Occupied cell at ray end
        // ------------------------------------------------------
        if (is_hit) {
            int idxg = yhit * grid_size_x + xhit;

            int v = log_odds[idxg] + L_OCC_INT;
            log_odds[idxg] = (int8_t) (v > L_MAX_INT ? L_MAX_INT : v);
        }
    }
}

// ------------------------------------------------------
// convert int8 log-odds → probability
// ------------------------------------------------------
float BayesianOccupancyGrid::get_cell_probability(int x,
                                                   int y) const
{
    // Out of bounds → unreachable = -1 ------------------------------- FAIRE ATTENTION
    if (x < 0 || x >= grid_size_x || y < 0 || y >= grid_size_y)
        return -1.f;

    int idx = y * grid_size_x + x;

    // log_odds ∈ [-40 .. +40] → shift to [0 .. 80]
    return prob_table[ log_odds[idx] + 40 ];
}

// // ------------------------------------------------------
// // Export grayscale map
// // ------------------------------------------------------
// const uint8_t* BayesianOccupancyGrid::get_map_data_color() const
// {
//     // static uint8_t export_data[GRID_MAX_SIZE];

//     // for (int y = 0; y < grid_size_y; y++)
//     // {
//     //     for (int x = 0; x < grid_size_x; x++)
//     //     {
//     //         float p = get_cell_probability(x, y);

//     //         float v = 255.0f * (1.0f - p);
//     //         if (v < 0) v = 0;
//     //         if (v > 255) v = 255;

//     //         export_data[y * grid_size_x + x] = (uint8_t)v;
//     //     }
//     // }
//     return (const uint8_t*)log_odds;
// }

// ------------------------------------------------------
// Return raw int8 grid
// ------------------------------------------------------
int8_t* BayesianOccupancyGrid::get_map_data() const
{
    return log_odds;
}
