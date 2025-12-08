// Filename: esp1/planning/global_planner.h
// Description: Global path planning using A* on the Bayesian Occupancy Grid.

#pragma once

#include "../../../include/common/data_types.h"
#include "../../../include/esp1/mapping/occupancy/bayesian_grid.h"

/**
 * @brief Global Planner implementing an A* search on the occupancy grid.
 * Produces a list of world-coordinate waypoints for the robot to follow.
 */
class GlobalPlanner {
public:
    GlobalPlanner();

    /**
     * @brief Compute a global path from the robot pose to the mission goal.
     *
     * @param current_pose  Current pose from localization.
     * @param goal          Mission goal from Goal Manager.
     * @param map           Coarse Bayesian occupancy grid.
     *
     * @return GlobalPathMessage containing waypoints in world coordinates.
     */
    GlobalPathMessage generate_path(
        const Pose2D& current_pose,
        const MissionGoal& goal,
        const BayesianOccupancyGrid& map
    );

};
