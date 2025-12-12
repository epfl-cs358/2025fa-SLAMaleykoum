#pragma once

#include "../../common/data_types.h"
#include "../../esp1/mapping/occupancy/bayesian_grid.h" 

#include <stdint.h>

// =============================================================
// TUNING & MEMORY LIMITS
// =============================================================
#define MAX_FRONTIER_CANDIDATES 10
#define BFS_QUEUE_SIZE 200
#define MIN_CLUSTER_SIZE 5

class MissionPlanner {
public:
    /**
     * @brief Defines the available mission states or modes.
     */
    MissionPlanner(const Pose2D& initial_home_pose);

    MissionGoal update_goal(const Pose2D& pose, const BayesianOccupancyGrid& grid, bool global_planner_failed);

    void set_mission_state(MissionGoalType new_state);
    void add_user_waypoint(const Pose2D& waypoint);
    // bool is_current_goal_achieved(const Pose2D& current_pose) const;

    MissionGoalType get_current_state() const { return current_state_; }

private:
    void search_for_candidates(const BayesianOccupancyGrid& grid, 
                               int x_min, int x_max, int y_min, int y_max, 
                               int& candidate_count);

    MissionGoalType current_state_;
    Pose2D home_pose_;
    MissionGoal current_target_;
    
    struct ClusterCandidate {
        int center_x;
        int center_y;
        int size;
        bool valid;
    };

    ClusterCandidate candidates[MAX_FRONTIER_CANDIDATES];

    uint8_t visited_mask[GP_MAX_H][GP_MAX_W / 8 + 1]; // Bitmask for visited cells during BFS

    bool is_current_goal_valid(const Pose2D& robot_pose, const BayesianOccupancyGrid& grid);
};
