// Filename: esp1/planning/global_planner.h
// Description: Global path planning using A* on the Bayesian Occupancy Grid.

#pragma once

#include "../../../include/common/data_types.h"
#include "../../../include/esp1/mapping/occupancy/bayesian_grid.h"

/**
 * @brief Global Planner implementing an (coarse) A* search on the full grid.
 * 
 * Produces a list of world-coordinate waypoints for the robot to follow.
 * Rarely called (only when goal changes or if the map changes a lot)
 */
class GlobalPlanner {
public:
    GlobalPlanner();

    /**
     * @brief Compute a global path from the robot pose to the mission goal.
     *
     * @param current_pose  Current pose from localization.
     * @param goal          Mission goal
     * @param map           Bayesian occupancy grid (coarse). 
     *
     * @return GlobalPathMessage containing a few waypoints in world coordinates.
     */
    PathMessage generate_path(
        const Pose2D& current_pose,
        const MissionGoal& goal,
        const BayesianOccupancyGrid& map
    );

};
