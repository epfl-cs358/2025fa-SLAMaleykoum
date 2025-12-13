// Filename: esp1/planning/global_planner.h
// Description: Global path planning using A* on the Bayesian Occupancy Grid.

#pragma once

#include "../../../include/common/data_types.h"
#include "../../../include/esp1/mapping/occupancy/bayesian_grid.h"

struct GlobalPlannerWorkspace {
    float g_cost[GP_MAX_CELLS];
    
    // Stores the INDEX of the parent (idx = y*W + x). -1 if no parent.
    int16_t parent_index[GP_MAX_CELLS]; 
    
    // We use uint8_t for boolean flags (closed set)
    uint8_t closed[GP_MAX_CELLS];
    
    // Binary Heap Node
    struct Node { int16_t idx; float f; } pq[GP_PQ_SIZE];
    int pq_size;
    
    // Output Path Buffers
    int16_t px[GP_MAX_CELLS];
    int16_t py[GP_MAX_CELLS];
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
