#pragma once

#include "../../common/data_types.h"
#include "../../esp1/mapping/occupancy/bayesian_grid.h" 

#include <stdint.h>

#define MAX_FRONTIER_CANDIDATES 10
#define BFS_QUEUE_SIZE 200
#define MIN_CLUSTER_SIZE 5

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
    MissionGoal update_goal(const Pose2D& pose, const BayesianOccupancyGrid& grid, bool global_planner_failed);

private:
    Pose2D home_pose_;
    MissionGoal current_target_;
    static int fail_count;

    struct ClusterCandidate {
        int center_x;
        int center_y;
        int size;
        bool valid;
    };

    ClusterCandidate candidates[MAX_FRONTIER_CANDIDATES];
    uint8_t visited_mask[GP_MAX_H][GP_MAX_W / 8 + 1]; // Bitmask for visited cells during BFS

    /**
     * @brief Searches a specific rectangular area of the grid for frontier clusters.
     * 
     * @param grid Reference to the occupancy grid.
     * @param x_min, x_max, y_min, y_max The bounds of the search area.
     * @param candidate_count Reference to the current count of valid candidates (updated by this function).
     * 
     * This function iterates through the grid within the specified bounds. If it finds
     * a frontier cell, it expands it into a cluster using BFS, calculates the centroid,
     * and adds it to the candidate list.
     * 
     * @return void
     */
    void search_for_candidates(const BayesianOccupancyGrid& grid, 
                               int x_min, int x_max, int y_min, int y_max, 
                               int& candidate_count);

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
