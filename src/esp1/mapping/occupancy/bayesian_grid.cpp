#include "../../../../include/esp1/mapping/occupancy/bayesian_grid.h"
#include <cmath>
#include "../../include/common/utils.h"

#define L_FREE_INT      -6     // -0.4  * 10
#define L_OCC_INT       +15    // +0.4  * 10
#define L_MIN_INT      -40     // -4.0  * 10
#define L_MAX_INT       40     // +4.0  * 10

float* BayesianOccupancyGrid::sin_table = nullptr;
float* BayesianOccupancyGrid::cos_table = nullptr;
bool   BayesianOccupancyGrid::trig_initialized = false;

float* BayesianOccupancyGrid::prob_table = nullptr;
bool   BayesianOccupancyGrid::prob_table_initialized = false;

// ------------------------------------------------------
// Constructor
// ------------------------------------------------------
BayesianOccupancyGrid::BayesianOccupancyGrid(float resolution_m,
                                             uint16_t size_x,
                                             uint16_t size_y)
    : grid_size_x(size_x),
      grid_size_y(size_y),
      grid_resolution(resolution_m)
{
    log_odds = (int8_t*) heap_caps_malloc(
        grid_size_x * grid_size_y * sizeof(int8_t),
        MALLOC_CAP_SPIRAM
    );
    memset(log_odds, 0, grid_size_x * grid_size_y);

   

    // Mark borders as occupied
    for (int x = 0; x < grid_size_x; ++x) {
        log_odds[x] = L_MAX_INT;
        log_odds[(grid_size_y - 1) * grid_size_x + x] = L_MAX_INT;
    }

    for (int y = 0; y < grid_size_y; ++y) {
        log_odds[y * grid_size_x] = L_MAX_INT;
        log_odds[y * grid_size_x + (grid_size_x - 1)] = L_MAX_INT;
    }

    // Precompute sin/cos tables
    if (!trig_initialized) {
        sin_table = (float*) heap_caps_malloc(3600 * sizeof(float), MALLOC_CAP_SPIRAM);
        cos_table = (float*) heap_caps_malloc(3600 * sizeof(float), MALLOC_CAP_SPIRAM);

        for (int i = 0; i < 3600; i++) {
            float a = i * 0.1f * M_PI / 180.0f;
            sin_table[i] = sinf(a);
            cos_table[i] = cosf(a);
        }
        trig_initialized = true;
    }

    // Probability lookup table
    if (!prob_table_initialized) {
        prob_table = (float*) heap_caps_malloc(81 * sizeof(float), MALLOC_CAP_SPIRAM);
        for (int i = L_MIN_INT; i <= L_MAX_INT; ++i) {
            float L = i * 0.1f;
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

    float pose_deg = pose.theta * (180.0f / M_PI) + 90.0f;

    for (uint16_t i = 0; i < scan.count; i++) {

        float rayon = scan.distances[i] * 0.001f;
        bool is_hit = (rayon > 0.f && rayon < SEARCH_BOUND_M);
        rayon = is_hit ? rayon : SEARCH_BOUND_M;

        float angle_world = pose_deg - scan.angles[i];
        if (isnan(angle_world)) continue;

        while (angle_world >= 360.f) angle_world -= 360.f;
        while (angle_world <   0.f)  angle_world += 360.f;

        int idx = (int)(angle_world * 10.f);
        if (idx >= 3600) idx -= 3600;
        if (idx < 0)     idx += 3600;

        float sinv = sin_table[idx];
        float cosv = cos_table[idx];

        float hit_x = pose.x + rayon * cosv;
        float hit_y = pose.y + rayon * sinv;

        int x0, y0, xhit, yhit;
        world_to_grid(pose.x, pose.y, x0, y0,
                      grid_resolution, grid_size_x, grid_size_y);
        world_to_grid(hit_x, hit_y, xhit, yhit,
                      grid_resolution, grid_size_x, grid_size_y);

        // Bresenham ray tracing
        int dx = abs(xhit - x0);
        int dy = abs(yhit - y0);
        int sx = (x0 < xhit) ? 1 : -1;
        int sy = (y0 < yhit) ? 1 : -1;
        int err = dx - dy;

        int x = x0;
        int y = y0;

        while (true) {
            if (x <= 0 || x >= grid_size_x - 1 ||
                y <= 0 || y >= grid_size_y - 1)
                break;

            int idxg = y * grid_size_x + x;
            int v = log_odds[idxg] + L_FREE_INT;
            log_odds[idxg] = (int8_t)(v < L_MIN_INT ? L_MIN_INT : v);

            if (x == xhit && y == yhit)
                break;

            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 <  dx) { err += dx; y += sy; }
        }

        // Mark occupied cell (SAFE)
        if (is_hit &&
            xhit >= 0 && xhit < grid_size_x &&
            yhit >= 0 && yhit < grid_size_y)
        {
            int idxg = yhit * grid_size_x + xhit;
            int v = log_odds[idxg] + L_OCC_INT;
            log_odds[idxg] = (int8_t)(v > L_MAX_INT ? L_MAX_INT : v);
        }
    }
}

// ------------------------------------------------------
// Probability access
// ------------------------------------------------------
float BayesianOccupancyGrid::get_cell_probability(int x, int y) const
{
    // Out of bounds = obstacle (safe for A*)
    if (x < 0 || x >= grid_size_x || y < 0 || y >= grid_size_y)
        return 1.0f;

    int idx = y * grid_size_x + x;
    return prob_table[log_odds[idx] + 40];
}

// ------------------------------------------------------
// Raw grid access
// ------------------------------------------------------
int8_t* BayesianOccupancyGrid::get_map_data() const
{
    return log_odds;
}
