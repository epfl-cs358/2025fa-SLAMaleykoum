#include "../../include/esp1/planning/mission_planner.h"
#include <cmath>
#include "../../include/common/utils.h"

// STATIC FUNCTIONS -------------------------------------------------------------------

/**
 * @brief checks the safety of the coordinates passed in argument.
 * 
 * The point must be:
 * - in a reachable distance, within the `ROBOT_RADIUS`.
 * - within the limits of the map
 * - on an unoccupied cell, free or unknown is fine. 
 * 
 * @return ture if the point is safe to reach, false otherwise.
 */
static bool is_point_safe(const BayesianOccupancyGrid& grid, int gx, int gy)
{
    int radius_cells = (int)(ROBOT_RADIUS / grid.grid_resolution);
    int r_sq = radius_cells * radius_cells; 

    for (int dy = -radius_cells; dy <= radius_cells; dy++) {
        for (int dx = -radius_cells; dx <= radius_cells; dx++) {
            if ((dx*dx + dy*dy) > r_sq) continue;
            int nx = gx + dx;
            int ny = gy + dy;
            if (nx < 0 || ny < 0 || nx >= grid.grid_size_x || ny >= grid.grid_size_y) continue;
            if (grid.get_cell_probability(nx, ny) > OCC_BOUND_PROB) return false;
        }
    }
    return true;
}

// ------------------------------------------------------------------------------------

MissionPlanner::MissionPlanner(const Pose2D& initial_home_pose) : home_pose_(initial_home_pose) {
    current_target_.target_pose = initial_home_pose;
    current_target_.type = EXPLORATION_MODE;
    fail_count = 0;
}

inline bool MissionPlanner::is_visited(int x, int y) const {
    // Math: x/8 finds the byte, x%8 finds the bit position
    return visited_mask[y][x/8] & (1 << (x%8));
}

inline void MissionPlanner::set_visited(int x, int y) {
    visited_mask[y][x/8] |= (1 << (x%8));
}

void MissionPlanner::store_candidate_if_valid(int sum_x, int sum_y, int size, int& candidate_count) {
    if (size >= MIN_CLUSTER_SIZE) {
        // Math: Calculate Centroid (Average position)
        candidates[candidate_count].center_x = sum_x / size;
        candidates[candidate_count].center_y = sum_y / size;
        candidates[candidate_count].size = size;
        candidates[candidate_count].valid = true;
        candidate_count++;
    }
}

void MissionPlanner::find_cluster(const BayesianOccupancyGrid& grid, int start_x, int start_y, int& candidate_count) {
    bool first_set = false;

    // BFS Queue (Ring Buffer)
    struct Point { int16_t x; int16_t y; };
    static Point bfs_q[BFS_QUEUE_SIZE]; // Static: Allocated once in memory, not on stack
    int head = 0, tail = 0;
    
    // Initialize BFS
    bfs_q[tail++] = {(int16_t)start_x, (int16_t)start_y};
    set_visited(start_x, start_y);

    int size = 0, sum_x = 0, sum_y = 0;

    // BFS Loop
    while(head != tail) {
        Point current = bfs_q[head];
        head = (head + 1) % BFS_QUEUE_SIZE; // Ring buffer wrap
        
        // Accumulate Stats
        size++;
        sum_x += current.x;
        sum_y += current.y;
        if (size >= 100) break;

        // Store First Point
        if (!first_set) {
            candidates[candidate_count].first_x = current.x;
            candidates[candidate_count].first_y = current.y;
            first_set = true;
        }

        // Check Neighbors
        static int dx8[8] = { 1,-1, 0, 0, 1, 1,-1,-1 };
        static int dy8[8] = { 0, 0, 1,-1, 1,-1, 1,-1 };

        for(int k=0; k<8; k++){
            int nx = current.x + dx8[k]; 
            int ny = current.y + dy8[k];

            // Boundary Check
            if (nx < 0 || ny < 0 || nx >= grid.grid_size_x || ny >= grid.grid_size_y) continue;
            
            if (!is_visited(nx, ny) && is_frontier_cell(grid, nx, ny)) {
                int next_tail = (tail + 1) % BFS_QUEUE_SIZE;
                if (next_tail != head) {
                    bfs_q[tail] = {(int16_t)nx, (int16_t)ny};
                    tail = next_tail;
                    set_visited(nx, ny);
                }
            }
        }
    }
    store_candidate_if_valid(sum_x, sum_y, size, candidate_count);
}

static bool invalid(int x, int y, InvalidGoals invalid_goals) {
    for (int i = 0; i < invalid_goals.size; ++i) {
        if (invalid_goals.lasts[i].x == x && invalid_goals.lasts[i].y == y)
            return true;
    }

    return false;
}

void MissionPlanner::search_for_candidates(const BayesianOccupancyGrid& grid, 
                                           int x_min, int x_max, int y_min, int y_max, 
                                           int& candidate_count, const InvalidGoals& invalid_goals)
{
    for (int y = y_min; y <= y_max; y++) {
        for (int x = x_min; x <= x_max; x++) {
            
            // Skip checks (save compute time)
            if (is_visited(x, y) || invalid(x, y, invalid_goals)) continue;
            
            // Found a new frontier? Expand it.
            if (is_frontier_cell(grid, x, y)) {
                if (candidate_count >= MAX_FRONTIER_CANDIDATES) return;
                
                find_cluster(grid, x, y, candidate_count);
            }
        }
    }
}

MissionGoal MissionPlanner::update_goal(const Pose2D& pose, const BayesianOccupancyGrid& grid, 
    const InvalidGoals& invalid_goals)
{
    const int PATIENCE_LIMIT = 20; 
    const int NEAR_RANGE_CELLS = (int)(FRONTIER_NEAR_RANGE_M / grid.grid_resolution);
    const int NEAR_RANGE_SQ = NEAR_RANGE_CELLS * NEAR_RANGE_CELLS;

    if (current_target_.type == RETURN_HOME) {
        current_target_.target_pose = home_pose_;
        current_target_.type = RETURN_HOME;
        return current_target_;
    }

    if (current_target_.type == EXPLORATION_MODE) {
        int rx, ry;
        world_to_grid(pose.x, pose.y, rx, ry, grid.grid_resolution, grid.grid_size_x, grid.grid_size_y);
        
        // Reset Memory (Using memset on the member array, not stack)
        memset(visited_mask, 0, sizeof(visited_mask));
        int candidate_count = 0;
        
        // Local Search (Optimization)
        int range = (int)(SEARCH_BOUND_M / grid.grid_resolution);
        search_for_candidates(grid, 
            std::max(0, rx-range), std::min((int)grid.grid_size_x-1, rx+range),
            std::max(0, ry-range), std::min((int)grid.grid_size_y-1, ry+range),
            candidate_count, invalid_goals);

        // Global Search (Fallback)
        if (candidate_count == 0)
            search_for_candidates(grid, 2, grid.grid_size_x - 3, 2, grid.grid_size_y - 3, candidate_count, invalid_goals);

        // Select Best
        int best_idx = -1;
        float min_dist_sq = 100000.0f;

        for(int i = 0; i < candidate_count; i++) {
            if(candidates[i].valid) {
                float dx = (candidates[i].center_x - rx);
                float dy = (candidates[i].center_y - ry);
                float d_sq = dx*dx + dy*dy;

                if (candidates[i].size > MIN_CLUSTER_SIZE && d_sq < min_dist_sq) {
                    min_dist_sq = d_sq;
                    best_idx = i;
                }
            }
        }

        if (best_idx != -1) {
            grid_to_world(candidates[best_idx].first_x, candidates[best_idx].first_y,
                current_target_.target_pose.x, current_target_.target_pose.y,
                grid.grid_resolution, grid.grid_size_x, grid.grid_size_y);
            return current_target_;
        // No candidate found, check patience limit
        } else {
            fail_count++;
            if(fail_count > PATIENCE_LIMIT) {
                current_target_.type = RETURN_HOME;
                current_target_.target_pose = home_pose_;
            } else
                current_target_.target_pose = pose;
        }
    }
    return current_target_;
}
