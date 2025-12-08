// Filename: esp1/planning/mission_planner.h
// Description: Contract for managing the high-level mission objective and determining
// the robot's next goal target (e.g., exploration points, return home).

#pragma once

#include "../../common/data_types.h"
#include "../../esp1/mapping/occupancy/bayesian_grid.h" 

#include <stdint.h>
#include <vector>

/**
 * @brief Manages the overall mission state and provides the current target for the Global Planner.
 */
class MissionPlanner {
public:
    /**
     * @brief Defines the available mission states or modes.
     */


    MissionPlanner(const Pose2D& initial_home_pose);

    /**
     * @brief The main update loop for the Goal Manager.
     * Determines the next target goal based on the current state and pose.
     * @param current_pose The latest pose from the EKF_SLAM engine.
     * @return The MissionGoal (Pose2D target and type) for the Global Planner.
     */
    MissionGoal update_goal(const Pose2D& current_pose, const BayesianOccupancyGrid& grid);

    /**
     * @brief Changes the robot's high-level mission state (e.g., from IDLE to EXPLORING).
     */

    void set_mission_state(MissionGoalType new_state);

    /**
     * @brief Registers a new user-defined waypoint for NAVIGATING state.
     */
    void add_user_waypoint(const Pose2D& waypoint);

    /**
     * @brief Checks if the current goal has been achieved within a tolerance.
     */
    bool is_current_goal_achieved(const Pose2D& current_pose) const;

   const std::vector<std::vector<std::pair<int,int>>>& get_frontier_clusters() const {
    return remaining_frontier_clusters_;
    }



    MissionGoalType get_current_state() const { return current_state_; }

private:
    MissionGoalType current_state_;
    Pose2D home_pose_;
    std::vector<Pose2D> waypoint_queue_;
    MissionGoal current_target_;
    std::vector<std::vector<std::pair<int,int>>> remaining_frontier_clusters_;
    static bool first; 
};
