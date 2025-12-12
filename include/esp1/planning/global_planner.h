// Filename: esp1/planning/global_planner.h
// Description: Global path planning using A* on the Bayesian Occupancy Grid.

#pragma once

#include "../../../include/common/data_types.h"
#include "../../../include/esp1/mapping/occupancy/bayesian_grid.h"

struct GlobalPlannerWorkspace {
    float g_cost[GP_MAX_H][GP_MAX_W];
    
    int16_t parent_x[GP_MAX_H][GP_MAX_W];
    int16_t parent_y[GP_MAX_H][GP_MAX_W];
    
    bool closed[GP_MAX_H][GP_MAX_W];
    
    struct Node { int16_t x; int16_t y; float f; } pq[GP_MAX_N];
    
    int16_t px[GP_MAX_N];
    int16_t py[GP_MAX_N];
};

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
     * @param ws            Workspace for A* (to avoid reallocation).
     *
     * @return GlobalPathMessage containing a few waypoints in world coordinates.
     */
    PathMessage generate_path(const Pose2D& current_pose,
                              const MissionGoal& goal,
                              const BayesianOccupancyGrid& map,
                              GlobalPlannerWorkspace* ws);

};
