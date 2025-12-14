#include "esp1/planning/mission_planner.h"
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

/**
 * @brief determines if a cell is frontier or not
 * A frontier is a free (explored) space that has at least one unknown (between free 
 * and occupied) cell around her.
 * 
 * @return true if it is a frontiere, false if not.
 */
static bool is_frontier_cell(const BayesianOccupancyGrid& grid, int x, int y)
{
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

// ------------------------------------------------------------------------------------

MissionPlanner::MissionPlanner(const Pose2D& initial_home_pose) : home_pose_(initial_home_pose) {
    current_target_.target_pose = initial_home_pose;
    current_target_.type = EXPLORATION_MODE;
    fail_count = 0;
}

bool MissionPlanner::is_current_goal_valid(const Pose2D& robot_pose, const BayesianOccupancyGrid& grid) {
    // Proximity Check
    float dx = robot_pose.x - current_target_.target_pose.x;
    float dy = robot_pose.y - current_target_.target_pose.y;
    if ((dx*dx + dy*dy) < GOAL_REACHED * GOAL_REACHED) 
        return false;
    
    // Check Map Integrity
    int gx, gy;
    world_to_grid(current_target_.target_pose.x, current_target_.target_pose.y, 
                  gx, gy, grid.grid_resolution, grid.grid_size_x, grid.grid_size_y);

    if (grid.get_cell_probability(gx, gy) > OCC_BOUND_PROB) 
        return false;

    // Check Frontier Integrity
    for (int y = -1; y <= 1; y++)
        for (int x = -1; x <= 1; x++)
            if (is_frontier_cell(grid, gx+x, gy+y))
                return true;
    
    return false;
}

void MissionPlanner::search_for_candidates(const BayesianOccupancyGrid& grid, 
                                           int x_min, int x_max, int y_min, int y_max, 
                                           int& candidate_count)
{
    for (int y = y_min; y <= y_max; y++) {
        for (int x = x_min; x <= x_max; x++) {
            
            // 1. Skip if already visited (Bitmask check)
            // visited_mask is [200][25] (200 rows, 25 bytes per row)
            if (visited_mask[y][x/8] & (1 << (x%8))) continue;
            
            // 2. Check if this specific cell is a frontier
            if (is_frontier_cell(grid, x, y)) {
                
                // Stop if we have maxed out our candidate array
                if (candidate_count >= MAX_FRONTIER_CANDIDATES) return;

                // --- BFS CLUSTERING INITIALIZATION ---
                struct Point { int16_t x; int16_t y; };
                static Point bfs_q[BFS_QUEUE_SIZE]; // Static allocation for memory safety
                int head = 0, tail = 0;

                // Enqueue start point
                bfs_q[tail++] = {(int16_t)x, (int16_t)y};
                visited_mask[y][x/8] |= (1 << (x%8)); // Mark visited

                int cluster_size = 0;
                int sum_x = 0;
                int sum_y = 0;

                // --- BFS LOOP ---
                while(head != tail) {
                    Point curr = bfs_q[head];
                    head = (head + 1) % BFS_QUEUE_SIZE; // Ring buffer wrap

                    // Accumulate stats for centroid calculation
                    cluster_size++; 
                    sum_x += curr.x; 
                    sum_y += curr.y;

                    // Optimization: Stop growing if cluster is large enough
                    if (cluster_size >= 100) break;

                    // Check 8-connected neighbors
                    static int dx8[8] = {1,-1,0,0, 1,1,-1,-1};
                    static int dy8[8] = {0,0,1,-1, 1,-1,1,-1};

                    for(int k=0; k<8; k++){
                        int nx = curr.x + dx8[k]; 
                        int ny = curr.y + dy8[k];

                        // Bounds check against grid limits
                        if (nx < 0 || ny < 0 || nx >= grid.grid_size_x || ny >= grid.grid_size_y) continue;
                        
                        // Visited check
                        if (visited_mask[ny][nx/8] & (1 << (nx%8))) continue;

                        if (is_frontier_cell(grid, nx, ny)) {
                            int next_tail = (tail + 1) % BFS_QUEUE_SIZE;
                            // Add to queue if not full
                            if (next_tail != head) {
                                bfs_q[tail] = {(int16_t)nx, (int16_t)ny};
                                tail = next_tail;
                                visited_mask[ny][nx/8] |= (1 << (nx%8)); // Mark visited immediately
                            }
                        }
                    }
                }

                // --- CLUSTER VALIDATION ---
                // Only keep clusters that are large enough (filters noise)
                if (cluster_size >= MIN_CLUSTER_SIZE) {
                    candidates[candidate_count].center_x = sum_x / cluster_size;
                    candidates[candidate_count].center_y = sum_y / cluster_size;
                    candidates[candidate_count].size = cluster_size;
                    candidates[candidate_count].valid = true;
                    candidate_count++;
                }
            }
        }
    }
}

MissionGoal MissionPlanner::update_goal(const Pose2D& pose, const BayesianOccupancyGrid& grid, bool global_planner_failed)
{
    const int PATIENCE_LIMIT = 20; 

    if (current_target_.type == RETURN_HOME) {
        current_target_.target_pose = home_pose_;
        current_target_.type = RETURN_HOME;
        return current_target_;
    }

    if (current_target_.type == EXPLORATION_MODE) {
        int rx, ry;
        world_to_grid(pose.x, pose.y, rx, ry, grid.grid_resolution, grid.grid_size_x, grid.grid_size_y);
        
        // can keep the same goal ?
        if ((!global_planner_failed 
            && is_current_goal_valid(pose, grid))
            || grid.get_cell_probability(rx, ry) > FREE_BOUND_PROB)
            return current_target_;
        
        // Reset Memory (Using memset on the member array, not stack)
        memset(visited_mask, 0, sizeof(visited_mask));
        int candidate_count = 0;
        
        // Local Search (Optimization)
        int range = (int)(SEARCH_BOUND_M / grid.grid_resolution);
        search_for_candidates(grid, 
            std::max(0, rx-range), std::min((int)grid.grid_size_x-1, rx+range),
            std::max(0, ry-range), std::min((int)grid.grid_size_y-1, ry+range),
            candidate_count);

        // Global Search (Fallback)
        if (candidate_count == 0)
            search_for_candidates(grid, 2, grid.grid_size_x - 3, 2, grid.grid_size_y - 3, candidate_count);

        // Select Best
        int best_idx = -1;
        int max_size = -1;
        float min_dist_sq = 100000.0f;

        for(int i=0; i<candidate_count; i++){
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
            grid_to_world(candidates[best_idx].center_x, candidates[best_idx].center_y, 
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

