#include "esp1/planning/mission_planner.h"
#include <cmath>
#include "../../include/common/utils.h"

// ============================================================
// HELPERS : LOGODDS + FRONTIER CHECK
// ============================================================

// static inline float proba(const BayesianOccupancyGrid& grid, int x, int y)
// {
//     return grid.get_map_data()[y * grid.grid_size_x + x];
// }

// static inline bool is_unknown(float lo)
// {
//     float p = 1.f / (1.f + std::exp(-lo));
//     return (0.35f < p) && (p < 0.6f);
// }

// ============================================================
// SAFETY CHECK FOR ROBOT RADIUS
// ============================================================
// constantes center a remettre au propre + fonctions c'est la meme utilise dans global planner et d'autre endroits ----------------

static bool is_point_safe(const BayesianOccupancyGrid& grid, int gx, int gy)
{
    int W = grid.grid_size_x;
    int H = grid.grid_size_y;
    int radius_cells = (int)(ROBOT_RADIUS / grid.grid_resolution);
    int r_sq = radius_cells * radius_cells; 

    for (int dy = -radius_cells; dy <= radius_cells; dy++) {
        for (int dx = -radius_cells; dx <= radius_cells; dx++) {
            if ((dx*dx + dy*dy) > r_sq) continue;
            int nx = gx + dx;
            int ny = gy + dy;
            if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
            if (grid.get_cell_probability(nx, ny) > 0.60f) return false;
        }
    }
    return true;
}


// ============================================================
// SEARCH FOR FRONTIER CANDIDATES
// ============================================================
static bool is_frontier_cell(const BayesianOccupancyGrid& grid, int x, int y)
{
    // 1. FILTER: MAP EDGES ARE NOT FRONTIERS
    // We ignore the 2 outer layers of pixels to avoid "leaking" out of the map.
    if (x <= 1 || x >= grid.grid_size_x - 2 || y <= 1 || y >= grid.grid_size_y - 2)
        return false;

    // if occupied 
    if (grid.get_cell_probability(x, y) >= 0.5f) return false;
    
    static int dx[4] = {1, -1, 0, 0};
    static int dy[4] = {0, 0, 1, -1};

    for (int i = 0; i < 4; i++) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        // Bounds check included in edge filter above, but safety first:
        if (nx < 0 || ny < 0 || nx >= grid.grid_size_x || ny >= grid.grid_size_y) continue;

        float p = grid.get_cell_probability(nx, ny);
        // if unknown
        if (0.35f < p && p < 0.65f) return true;
    }

    return false;
}

/**
 * @brief Searches a specific rectangular area of the grid for frontier clusters.
 * * This function iterates through the grid within the specified bounds. If it finds
 * a frontier cell, it expands it into a cluster using BFS, calculates the centroid,
 * and adds it to the candidate list.
 * * @param grid Reference to the occupancy grid.
 * @param x_min, x_max, y_min, y_max The bounds of the search area.
 * @param candidate_count Reference to the current count of valid candidates (updated by this function).
 * @return void
 */
void MissionPlanner::search_for_candidates(const BayesianOccupancyGrid& grid, 
                                           int x_min, int x_max, int y_min, int y_max, 
                                           int& candidate_count)
{
    // Iterate through the defined bounding box
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


// ============================================================
// CONSTRUCTOR
// ============================================================

MissionPlanner::MissionPlanner(const Pose2D& initial_home_pose) : home_pose_(initial_home_pose), current_state_(EXPLORATION_MODE) {
    current_target_.target_pose = initial_home_pose;
    current_target_.type = EXPLORATION_MODE;
}
void MissionPlanner::set_mission_state(MissionGoalType st) { current_state_ = st; }
void MissionPlanner::add_user_waypoint(const Pose2D& wp) {}
// bool MissionPlanner::is_current_goal_achieved(const Pose2D& pose) const {
//     float dx = pose.x - current_target_.target_pose.x;
//     float dy = pose.y - current_target_.target_pose.y;
//     return (dx*dx + dy*dy) < (0.20f * 0.20f); 
// }

// ============================================================
// MAIN LOGIC
// ============================================================
MissionGoal MissionPlanner::update_goal(const Pose2D& pose, const BayesianOccupancyGrid& grid)
{
    const int PATIENCE_LIMIT = 10; // 5 Seconds (assuming 2Hz update rate)

    switch (current_state_)
    {
        case IDLE: 
            return current_target_;
            
        case RETURN_HOME:
            current_target_.target_pose = home_pose_;
            // current_target_.target_pose.x = 10.f;
            // current_target_.target_pose.y = 10.f;
            current_target_.type = RETURN_HOME;
            return current_target_;

        case EXPLORATION_MODE:
        {
            static int patience_counter = 0; 

            // --------------------------------------------------------
            // STEP 0: PREPARE MEMORY
            // --------------------------------------------------------
            // Reset the visited mask (0 = not visited)
            memset(visited_mask, 0, sizeof(visited_mask)); 
            
            // Reset candidate list
            int candidate_count = 0;
            for(int k=0; k<MAX_FRONTIER_CANDIDATES; k++) candidates[k].valid = false;

            // --------------------------------------------------------
            // STEP 1: DEFINE SEARCH BOUNDS
            // --------------------------------------------------------
            int rx, ry;
            world_to_grid(pose.x, pose.y, rx, ry, grid.grid_resolution, grid.grid_size_x, grid.grid_size_y);
            
            // Calculate Local Bounding Box (+/- SEARCH_BOUND_M)
            int range_cells = (int)(SEARCH_BOUND_M / grid.grid_resolution); 
            int local_x_min = std::max(0, rx - range_cells);
            int local_x_max = std::min((int)grid.grid_size_x - 1, rx + range_cells);
            int local_y_min = std::max(0, ry - range_cells);
            int local_y_max = std::min((int)grid.grid_size_y - 1, ry + range_cells);

            // --------------------------------------------------------
            // STEP 2: LOCAL SEARCH (Optimization)
            // --------------------------------------------------------
            // First, look only in the immediate vicinity of the robot.
            search_for_candidates(grid, local_x_min, local_x_max, local_y_min, local_y_max, candidate_count);

            // --------------------------------------------------------
            // STEP 3: GLOBAL SEARCH (Fallback)
            // --------------------------------------------------------
            // If local search found nothing, scan the ENTIRE map.
            // Note: We intentionally do NOT reset visited_mask, so we don't re-scan the local area.
            
            // @silcazesam TODO: This might be an error, consider resetting visited_mask before global search.
            if (candidate_count == 0) {
                // Bounds = Whole Map (minus edges handled by is_frontier_cell filter)
                int global_x_min = 2; 
                int global_x_max = grid.grid_size_x - 3;
                int global_y_min = 2; 
                int global_y_max = grid.grid_size_y - 3;

                search_for_candidates(grid, global_x_min, global_x_max, global_y_min, global_y_max, candidate_count);
            }

            // --------------------------------------------------------
            // STEP 4: SELECT BEST CANDIDATE
            // --------------------------------------------------------
            int best_idx = -1;
            int max_size = -1;

            // Heuristic: Prefer the largest cluster found
            for(int i=0; i<candidate_count; i++){
                if(candidates[i].valid && candidates[i].size > max_size){
                    max_size = candidates[i].size;
                    best_idx = i;
                }
            }

            // --------------------------------------------------------
            // STEP 5: VALIDATE AND RETURN
            // --------------------------------------------------------
            if (best_idx != -1)
            {
                // Safety check: Ensure the chosen centroid is not inside a wall
                if (is_point_safe(grid, candidates[best_idx].center_x, candidates[best_idx].center_y)) {
                    
                    patience_counter = 0; // Success! Reset failure counter.

                    grid_to_world(candidates[best_idx].center_x, 
                                   candidates[best_idx].center_y, 
                                   current_target_.target_pose.x, current_target_.target_pose.y,
                                   grid.grid_resolution, grid.grid_size_x, grid.grid_size_y);
                    
                    current_target_.type = MissionGoalType::EXPLORATION_MODE;
                    return current_target_;
                }
            }

            // --------------------------------------------------------
            // STEP 6: HANDLING FAILURE (No Frontiers Found)
            // --------------------------------------------------------
            patience_counter++; 

            if (patience_counter > PATIENCE_LIMIT)
            {
                // Global search failed for 5 consecutive seconds.
                // Assumption: Map is fully explored.
                current_state_ = RETURN_HOME;
                current_target_.target_pose = home_pose_;
                current_target_.type = RETURN_HOME;
                return current_target_;
            }
            else
            {
                // Temporary failure (lidar noise, occlusion).
                // Wait in IDLE state for map to update.
                current_target_.target_pose = pose; 
                current_target_.type = MissionGoalType::EXPLORATION_MODE; 
                return current_target_;
            }
        }
    }
    return current_target_;
}
