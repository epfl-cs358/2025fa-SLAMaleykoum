#include "../../../include/esp1/mapping/bayesian_grid.h"
#include <cmath>
#include "../../include/common/utils.h"

#define L_FREE_INT      -6
#define L_OCC_INT      +15
#define L_MIN_INT      -40
#define L_MAX_INT       40

float* BayesianOccupancyGrid::sin_table = nullptr;
float* BayesianOccupancyGrid::cos_table = nullptr;
bool   BayesianOccupancyGrid::trig_initialized = false;

float* BayesianOccupancyGrid::prob_table = nullptr;
bool   BayesianOccupancyGrid::prob_table_initialized = false;

BayesianOccupancyGrid::BayesianOccupancyGrid(float resolution_m,
                                             uint16_t size_x,
                                             uint16_t size_y)
    : grid_size_x(size_x),
      grid_size_y(size_y),
      grid_resolution(resolution_m)
{
    log_odds = (int8_t*) heap_caps_malloc(
        grid_size_x * grid_size_y * sizeof(int8_t),
        MALLOC_CAP_INTERNAL
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

// Helper to check for valid finite float
inline bool is_finite_and_valid(float val) {
    return !isnan(val) && !isinf(val) && abs(val) < 1e6f;
}

void BayesianOccupancyGrid::update_map(const SyncedScan& lidar_scan)
{
    const LiDARScan& scan = lidar_scan.scan;
    const Pose2D& pose = lidar_scan.pose;

    // Data validation - check for NaN AND Inf
    if (!is_finite_and_valid(pose.x) || 
        !is_finite_and_valid(pose.y) || 
        !is_finite_and_valid(pose.theta)) {
        Serial.println("Map update rejected: Invalid pose (NaN/Inf)");
        return;
    }

    float pose_deg = pose.theta * (180.0f / M_PI) + 90.0f;
    
    // Check result of calculation
    if (!is_finite_and_valid(pose_deg)) {
        Serial.println("Map update rejected: Invalid pose_deg");
        return;
    }

    // Add timeout protection
    uint32_t start_time = millis();
    const uint32_t MAX_UPDATE_TIME_MS = 100;  // Maximum 100ms per update
    uint16_t points_processed = 0;

    for (uint16_t i = 0; i < scan.count; i++) {
        // Watchdog: Check if we've been running too long
        if (++points_processed % 50 == 0) {
            if (millis() - start_time > MAX_UPDATE_TIME_MS) {
                Serial.printf("Map update timeout after processing %d/%d points\n", 
                             points_processed, scan.count);
                return;
            }
        }

        // Validate angle (check for NaN AND Inf)
        if (!is_finite_and_valid(scan.angles[i])) {
            continue;
        }

        // Convert distance to meters
        float rayon = scan.distances[i] * 0.001f;

        // Ignore invalid or out-of-range measurements
        bool is_hit = (rayon > 0.f && rayon < SEARCH_BOUND_M);
        rayon = is_hit ? rayon : SEARCH_BOUND_M;

        // Calculate global angle of the measurement
        float angle_world = pose_deg - scan.angles[i];
        
        // Check for NaN AND Inf immediately
        if (!is_finite_and_valid(angle_world)) {
            continue;
        }

        // Normalize with BOUNDED iterations (prevent infinite loops)
        // Use fmodf for safe normalization instead of while loops
        if (angle_world >= 360.f || angle_world < 0.f) {
            angle_world = fmodf(angle_world, 360.f);
            if (angle_world < 0.f) {
                angle_world += 360.f;
            }
        }

        // Final safety check after normalization
        if (angle_world < 0.f || angle_world >= 360.f) {
            continue;  // Skip if still out of range
        }

        int idx = (int)(angle_world * 10.f);
        
        // Strict bounds checking
        if (idx < 0 || idx >= 3600) {
            continue;  // Skip instead of clamping
        }

        float sinv = sin_table[idx];
        float cosv = cos_table[idx];

        float hit_x = pose.x + rayon * cosv;
        float hit_y = pose.y + rayon * sinv;
        
        // Validate hit coordinates
        if (!is_finite_and_valid(hit_x) || !is_finite_and_valid(hit_y)) {
            continue;
        }

        int x0, y0, x_hit_grid, y_hit_grid;
        world_to_grid(pose.x, pose.y, x0, y0,
                      grid_resolution, grid_size_x, grid_size_y);
        world_to_grid(hit_x, hit_y, x_hit_grid, y_hit_grid,
                      grid_resolution, grid_size_x, grid_size_y);

        // Validate start position
        if (x0 <= 0 || x0 >= grid_size_x - 1 ||
            y0 <= 0 || y0 >= grid_size_y - 1) {
            continue;  // Skip if robot is out of valid area
        }

        // Clamp hit position to prevent wrap-around
        // If hit is out of bounds, clamp it to the border
        if (x_hit_grid < 1) x_hit_grid = 1;
        if (x_hit_grid >= grid_size_x - 1) x_hit_grid = grid_size_x - 2;
        if (y_hit_grid < 1) y_hit_grid = 1;
        if (y_hit_grid >= grid_size_y - 1) y_hit_grid = grid_size_y - 2;

        // Bresenham ray tracing
        int dx = abs(x_hit_grid - x0);
        int dy = abs(y_hit_grid - y0);
        int sx = (x0 < x_hit_grid) ? 1 : -1;
        int sy = (y0 < y_hit_grid) ? 1 : -1;
        int err = dx - dy;

        int x = x0;
        int y = y0;

        int max_iter = dx + dy + 1;

        while (max_iter-- > 0) {
            // If we reached the hit point, STOP.
            // Do NOT update this cell as free.
            if (x == x_hit_grid && y == y_hit_grid) {
                break; 
            }

            // Update Free Space
            int idx = y * grid_size_x + x;
            if (idx >= 0 && idx < grid_size_x * grid_size_y) {
                int v = log_odds[idx] + L_FREE_INT;
                log_odds[idx] = (int8_t)(v < L_MIN_INT ? L_MIN_INT : v);
            }

            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx)  { err += dx; y += sy; }
        }

        // Update occupied cell
        // Only if it was a real hit (not max range)
        if (is_hit) {
            int idx = y_hit_grid * grid_size_x + x_hit_grid;
            if (idx >= 0 && idx < grid_size_x * grid_size_y) {
                int v = log_odds[idx] + L_OCC_INT;
                log_odds[idx] = (int8_t)(v > L_MAX_INT ? L_MAX_INT : v);
            }
        }
    }
}

float BayesianOccupancyGrid::get_cell_probability(int x, int y) const
{
    // Out of bounds = obstacle (safe for A*)
    if (x < 0 || x >= grid_size_x || y < 0 || y >= grid_size_y)
        return 1.0f;

    int idx = y * grid_size_x + x;
    
    // Additional bounds check
    if (idx < 0 || idx >= grid_size_x * grid_size_y)
        return 1.0f;
    
    int prob_idx = log_odds[idx] + 40;
    
    // Bounds check for probability table access
    if (prob_idx < 0 || prob_idx >= 81)
        return 0.5f;  // Return unknown probability
    
    return prob_table[prob_idx];
}

int8_t* BayesianOccupancyGrid::get_map_data() const
{
    return log_odds;
}