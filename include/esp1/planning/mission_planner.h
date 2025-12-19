#pragma once

#include "../../common/data_types.h"
#include "../../esp1/mapping/bayesian_grid.h" 
#include "../../common/utils.h"

#include <stdint.h>

#define MAX_FRONTIER_CANDIDATES 10
#define BFS_QUEUE_SIZE 200
#define MIN_CLUSTER_SIZE 8
#define MAX_CLUSTER_SIZE 30
#define FRONTIER_NEAR_RANGE_M  1.0f

class MissionPlanner {
public:
    /**
     * @brief Constructor of a MissionPlanner
     * Initialise the `current_target_`, sets the state to `EXPLORATION_MODE`, 
     * and the position to home.
     * 
     * Sets the `home_pose_` to the one passed in argument.
     */
    MissionPlanner(const Pose2D& initial_home_pose);

    /**
     * @brief Defines the new goal.
     * 
     * @param pose the current position of the car.
     * @param grid the actual grid.
     * @param global_planner_failed indicator if the global planner failed to find a 
     * path to the current goal.
     * 
     * Depending on the current state we do different things :
     * - RETURN_HOME: we stay in this same state and keep going home.
     * - EXPLORATION_MODE: 
     *   0. check if we NEED to change the goal. Don't change if:
     * Global Planner didn't failed AND the current goal is still valid
     * OR the map has not yet been updated so we just wait a little longer. (this is detected
     * by checking the cell under the robot, if it is not free then the map is not up to date)
     *   
     *   1. We define the search area, start with a local one, around the car
     *   2. Then we search in the whole map : 
     * Note: We don't reset `visited_mask` to avoid re-scanning local area
     *   3. We select the best candidate
     * These have been computed in the `search_for_candidates()` function. We pick the 
     * closest one that is big enough to avoid jumping across the map.
     *   4. If no candidate found, we increment the patience counter.
     * If we haven't timed out yet, just stay put. Do NOT return (0,0) which causes the robot to drive to origin.
     * 
     * @return The new goal to reach.
     */
    MissionGoal update_goal(const Pose2D& pose, const BayesianOccupancyGrid& grid, bool global_planner_failed, InvalidGoals invalid_goals);

private:
    Pose2D home_pose_;
    MissionGoal current_target_;
    int fail_count;

    struct ClusterCandidate {
        int center_x;
        int center_y;
        int size;
        int first_x;
        int first_y;
        bool valid;
    };

    // List of candidates found during the search.
    ClusterCandidate candidates[MAX_FRONTIER_CANDIDATES];

    // 1bit per cell version of the grid for visited cells during BFS
    uint8_t visited_mask[GP_MAX_H][GP_MAX_W / 8 + 1];

    /**
     * @brief Helper to check and mark a cell as visited (Bit manipulation).
     * Inline for performance as it is called millions of times.
     * 
     * It will prevent infinite loops during BFS.
     */
    inline bool is_visited(int x, int y) const;
    inline void set_visited(int x, int y);

    /**
     * @brief Takes a valid frontier seed and runs BFS to find the whole cluster.
     * 
     * @param grid Reference to the occupancy grid.
     * @param start_x, start_y The starting cell coordinates of the frontier seed.
     * @param candidate_count Reference to the current count of valid candidates (updated by this function).
     * 
     * Breadth-First Search (BFS) starting from the given frontier cell. It explores all connected cells, 8 neighbors,
     * and if the neighbor is a frontier cell, then it is added to the BFS queue. The cells visited during the search
     * form a potential cluster. Once the BFS is complete, it calls `store_candidate_if_valid()` to validate
     * and store the cluster as a candidate.
     */
    void find_cluster(const BayesianOccupancyGrid& grid, int start_x, int start_y, int& candidate_count);

    /**
     * @brief Validates the final cluster and adds it to the array if good.
     * 
     * @param sum_x, sum_y The accumulated x and y coordinates of all cells in the cluster.
     * @param size The total number of cells in the cluster.
     * @param candidate_count Reference to the current count of valid candidates (updated by this function).
     * 
     * If the cluster has at least `MIN_CLUSTER_SIZE` cells, it calculates the geometric centroid
     * and adds it to the `candidates` array, marking it as valid.
     */
    void store_candidate_if_valid(int sum_x, int sum_y, int size, int& candidate_count);

    /**
     * @brief Searches a specific rectangular area of the grid for frontier clusters.
     * * @param grid Reference to the occupancy grid.
     * @param x_min, x_max, y_min, y_max The bounds of the search area (in grid coordinates).
     * @param candidate_count Reference to the current count of valid candidates. This is 
     * incremented whenever a new valid cluster is found.
     * * The process follows these steps:
     * 1. Iteration: Loops through every cell within the specified bounding box.
     * 2. Optimization Check: Checks the `visited_mask` first. If a cell was already 
     * processed (part of a previously found cluster), we skip it immediately.
     * 3. Frontier Detection: Checks if the current cell is a "seed" (an unvisited frontier point).
     * 4. Expansion: If a seed is found, we call `expand_cluster()` that runs a BFS to identify the entire
     * connected frontier, calculates its centroid, and adds it to the `candidates` array if it's valid.
     * * @return void
     */
    void search_for_candidates(const BayesianOccupancyGrid& grid, 
                               int x_min, int x_max, int y_min, int y_max, 
                               int& candidate_count, InvalidGoals invalid_goals);

    /**
     * @brief Checks if the current goal is still valid.
     * 
     * - Proximity Check: If we are very close to the goal, consider it achieved.
     * - Check Map Integrity: If the goal is now inside a wall (Dynamic obstacle appeared) -> goal invalid
     * - Check Frontier Integrity: If the area around the goal is fully explored (all free), 
     * the "frontier" has disappeared. We should move on. We check a small 3x3 box around the goal.
     * 
     * @return true if still valid, false if it needs to be updated.
     */
    bool is_current_goal_valid(const Pose2D& robot_pose, const BayesianOccupancyGrid& grid);
};
