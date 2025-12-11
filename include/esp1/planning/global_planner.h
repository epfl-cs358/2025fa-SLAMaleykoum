// Filename: esp1/planning/global_planner.h
// Description: Contract for generating a global path (list of Waypoints).
// Contracted Team: Planning

#pragma once

#include "common/data_types.h"
#include "esp1/mapping/occupancy/bayesian_grid.h" 
#include <vector>

/**
 * @brief Generates a list of Waypoints based on the current mission goal, using the Occupancy Grid.
 * This will typically involve a search algorithm like A* or Dijkstra's.
 */
class GlobalPlanner {
public:
    GlobalPlanner();

    /**
     * @brief Computes a new global path from the current pose to the target goal using the map.
     * @param current_pose The robot's current location (from EKF_SLAM).
     * @param goal The target destination/objective from the Goal Manager.
     * @param map The current coarse occupancy grid (required for pathfinding).
     * @return A vector of Waypoints for ESP_2 to follow.
     */
    PathMessage generate_path(
        const Pose2D& current_pose,
        const MissionGoal& goal,
        const BayesianOccupancyGrid& map
    );

    /**
     * @brief Checks if the path needs re-planning (e.g., goal achieved, path too old).
     * @return true if the path needs re-planning.
     */
    bool needs_replanning() const;
};
