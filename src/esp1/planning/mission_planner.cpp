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
 * - on a free cell. 
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
            if (grid.get_cell_probability(nx, ny) > FREE_BOUND_PROB) return false;
        }
    }
    return true;
}

/**
 * @brief determines if a cell is frontier or not
 * A frontier is a free (explored) space that has at least one unknown (between free 
 * and occupied) cell around her.
 * 
 * @return true if it is a frontiere, false if not.
 */
static bool is_frontier_cell(const BayesianOccupancyGrid& grid, int x, int y)
{
    // Check that the cell is in the map bounds
    if (grid.get_cell_probability(x, y) > FREE_BOUND_PROB) return false;
    
    // Check Neighbors for Unknowns
    static int dx[4] = {1, -1, 0, 0};
    static int dy[4] = {0, 0, 1, -1};

    for (int i = 0; i < 4; i++) {
        float np = grid.get_cell_probability(x + dx[i], y + dy[i]);
        
        if (np > FREE_BOUND_PROB && np < OCC_BOUND_PROB) return true;
    }

    return false;
}

/**
 * @brief determines if a point is in the invalid goals
 * 
 * @return true if the point is an invalid goal
 */
static bool invalid(int x, int y, InvalidGoals invalid_goals) {
    for (int i = 0; i < invalid_goals.size; ++i) {
        if (invalid_goals.lasts[i].x == x && invalid_goals.lasts[i].y == y) 
            return true;
    }
    return false;
}

// Helper to find a safe neighbor (surrounded by free space)
static void get_safe_neighbor(const BayesianOccupancyGrid& grid, int& gx, int& gy) {
    const int SEARCH_RADIUS = 3;
    
    // Check current spot first: is it safe? (All 8 neighbors are FREE)
    auto is_safe = [&](int cx, int cy) {
        if (grid.get_cell_probability(cx, cy) > FREE_BOUND_PROB) return false;
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;
                // If neighbor is occupied or unknown, this cell is unsafe
                if (grid.get_cell_probability(cx + dx, cy + dy) > FREE_BOUND_PROB) 
                    return false;
            }
        }
        return true;
    };

    if (is_safe(gx, gy)) return; // Current is safe

    // Spiral search for a safe cell
    for (int r = 1; r <= SEARCH_RADIUS; r++) {
        for (int dy = -r; dy <= r; dy++) {
            for (int dx = -r; dx <= r; dx++) {
                int nx = gx + dx;
                int ny = gy + dy;
                // Bounds check
                if (nx < 1 || ny < 1 || nx >= grid.grid_size_x - 1 || ny >= grid.grid_size_y - 1) continue;
                
                if (is_safe(nx, ny)) {
                    gx = nx;
                    gy = ny;
                    return; // Found safe spot, update and return
                }
            }
        }
    }
    // If no safe spot found, we keep original (better than nothing)
}

// END STATIC FUNCTIONS ------------------------------------------------------------------------------------

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
        candidates[candidate_count].center_x = sum_x / size;
        candidates[candidate_count].center_y = sum_y / size;
        candidates[candidate_count].size = size;
        candidates[candidate_count].valid = true;
        // Use the FIRST point found as a fallback if centroid is in a wall
        candidates[candidate_count].first_x = candidates[candidate_count].center_x; 
        candidates[candidate_count].first_y = candidates[candidate_count].center_y; 
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
        if (size >= MAX_CLUSTER_SIZE) break;

        // Store First Point
        if (!first_set) {
            candidates[candidate_count].first_x = current.x;
            candidates[candidate_count].first_y = current.y;
            first_set = true;
        }

        // Check Neighbors
        static int dx8[8] = { 1,-1, 0, 0, 1, 1,-1,-1 };
        static int dy8[8] = { 0, 0, 1,-1, 1,-1, 1,-1 };

        for(int k = 0; k < 8; k++){
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


void MissionPlanner::search_for_candidates(const BayesianOccupancyGrid& grid, 
                                           int x_min, int x_max, int y_min, int y_max, 
                                           int& candidate_count, InvalidGoals invalid_goals)
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


bool MissionPlanner::is_current_goal_valid(const Pose2D& robot_pose, const BayesianOccupancyGrid& grid) {
    // Check if reached
    float dx = robot_pose.x - current_target_.target_pose.x;
    float dy = robot_pose.y - current_target_.target_pose.y;
    if ((dx*dx + dy*dy) < GOAL_REACHED * GOAL_REACHED) 
        return false; // Reached, so "invalid" (needs update)
    
    // Check Map Integrity
    int gx, gy;
    world_to_grid(current_target_.target_pose.x, current_target_.target_pose.y, 
                  gx, gy, grid.grid_resolution, grid.grid_size_x, grid.grid_size_y);

    // If the goal cell itself has become occupied, it's invalid
    if (grid.get_cell_probability(gx, gy) > FREE_BOUND_PROB)
        return false;

    // Frontier Integrity
    // If the whole area is now explored/free, we should move on.
    bool still_frontier = false;
    for (int y = -2; y <= 2; y++) {
        for (int x = -2; x <= 2; x++) {
            if (is_frontier_cell(grid, gx+x, gy+y)) {
                still_frontier = true;
                break;
            }
        }
        if (still_frontier) break;
    }
    
    return still_frontier;
}

MissionGoal MissionPlanner::update_goal(const Pose2D& pose, const BayesianOccupancyGrid& grid, bool global_planner_failed, InvalidGoals invalid_goals)
{
    const int PATIENCE_LIMIT = 40; 

    if (current_target_.type == RETURN_HOME) {
        current_target_.target_pose = home_pose_;
        return current_target_;
    }

    // Keep current goal if valid and planner hasn't failed
    if (!global_planner_failed && is_current_goal_valid(pose, grid)) {
        fail_count = 0; // Success resets counter
        return current_target_;
    }

    int rx, ry;
    world_to_grid(pose.x, pose.y, rx, ry, grid.grid_resolution, grid.grid_size_x, grid.grid_size_y);
    
    memset(visited_mask, 0, sizeof(visited_mask));
    int candidate_count = 0;
    
    // Search for frontiers
    int range = (int)(SEARCH_BOUND_M / grid.grid_resolution);
    search_for_candidates(grid, 
        std::max(0, rx-range), std::min((int)grid.grid_size_x-1, rx+range),
        std::max(0, ry-range), std::min((int)grid.grid_size_y-1, ry+range),
        candidate_count, invalid_goals);

    if (candidate_count == 0)
        search_for_candidates(grid, 2, grid.grid_size_x - 3, 2, grid.grid_size_y - 3, candidate_count, invalid_goals);

    // --- STARTUP GRACE ---
    // Calculate how many cells are actually free. If map is empty, don't fail.
    // (This is a simplified check: if candidate count is 0, check if we have ANY visited cells?)
    // A better proxy: simple timeout based on system uptime could work, but here we use fail_count.
    
    if (candidate_count == 0) {
        fail_count++;
        
        // Prevent instant fail at startup
        if (fail_count > PATIENCE_LIMIT) {
            current_target_.type = RETURN_HOME;
            current_target_.target_pose = home_pose_;
        } else {
            // Wait here.
            current_target_.target_pose = pose; 
        }
        return current_target_;
    }

    // Candidates found -> Reset fail count
    fail_count = 0;

    // Select Best Candidate
    int best_idx = -1;
    float min_dist_sq = 1e9f;

    for(int i = 0; i < candidate_count; i++) {
        if(candidates[i].valid) {
            int cx = candidates[i].center_x;
            int cy = candidates[i].center_y;

            if (invalid(cx, cy, invalid_goals)) continue;

            float dx = (cx - rx);
            float dy = (cy - ry);
            float d_sq = dx*dx + dy*dy;

            if (candidates[i].size > MIN_CLUSTER_SIZE && d_sq < min_dist_sq) {
                min_dist_sq = d_sq;
                best_idx = i;
            }
        }
    }

    if (best_idx != -1) {
        int final_gx = candidates[best_idx].center_x;
        int final_gy = candidates[best_idx].center_y;

        // Verify centroid is actually FREE
        // If centroid landed on obstacle/unknown, use the first frontier point instead
        if (grid.get_cell_probability(final_gx, final_gy) > FREE_BOUND_PROB) {
            final_gx = candidates[best_idx].first_x;
            final_gy = candidates[best_idx].first_y;
        }

        // Apply Safety Padding
        // (Function get_safe_neighbor should be defined as per previous step)
        get_safe_neighbor(grid, final_gx, final_gy); // Assuming this exists in your previous copy

        grid_to_world(final_gx, final_gy,
            current_target_.target_pose.x, current_target_.target_pose.y,
            grid.grid_resolution, grid.grid_size_x, grid.grid_size_y);
            
    } else {
        // Candidates existed but were all invalid/blacklisted
        fail_count++;
        current_target_.target_pose = pose;
    }
    
    return current_target_;
}
