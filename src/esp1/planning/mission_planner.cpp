#include "esp1/planning/mission_planner.h"
#include "esp1/mapping/occupancy/bayesian_grid.h"
#include <iostream>
#include <cmath>
#include <limits>

// -----------------------------------------------------------
//      HELPERS : conversions + grid access
// -----------------------------------------------------------

static inline bool world_to_grid(const BayesianOccupancyGrid& grid,
                                 float x_m, float y_m,
                                 int& gx, int& gy)
{
    gx = int(x_m / grid.grid_resolution);
    gy = int(y_m / grid.grid_resolution);

    return gx >= 0 && gy >= 0 &&
           gx < grid.grid_size_x &&
           gy < grid.grid_size_y;
}

static inline float get_logodds(const BayesianOccupancyGrid& grid,
                                int x, int y)
{
    return grid.get_map_data()[y * grid.grid_size_x + x];
}

static inline bool is_unknown(float lo)
{
    return std::fabs(lo) < 0.05f;
}

static inline bool is_free(float lo)
{
    float p = 1.f / (1.f + std::exp(-lo));
    return p < 0.35f;
}

static bool is_frontier_cell(const BayesianOccupancyGrid& grid, int x, int y)
{
    float lo = get_logodds(grid, x, y);
    if (!is_free(lo)) return false;

    static int dx[4] = {1, -1, 0, 0};
    static int dy[4] = {0, 0, 1, -1};

    for (int i = 0; i < 4; i++)
    {
        int nx = x + dx[i];
        int ny = y + dy[i];

        if (nx < 0 || ny < 0 ||
            nx >= grid.grid_size_x ||
            ny >= grid.grid_size_y)
            continue;

        if (is_unknown(get_logodds(grid, nx, ny)))
            return true;
    }
    return false;
}

// -----------------------------------------------------------
//               CONSTRUCTOR
// -----------------------------------------------------------

GoalManager::GoalManager(const Pose2D& initial_home_pose)
    : current_state_(STATE_IDLE),
      home_pose_(initial_home_pose)
{
    current_target_.target_pose = home_pose_;
    current_target_.type        = MissionGoalType::IDLE;

    std::cout << "[GoalManager] Home pose initialized at ("
              << home_pose_.x << ", " << home_pose_.y << ")\n";
}

// -----------------------------------------------------------
//                 CHANGE MISSION STATE
// -----------------------------------------------------------

void GoalManager::set_mission_state(GoalManager::MissionState new_state)
{
    current_state_ = new_state;

    switch (new_state)
    {
        case STATE_IDLE:           std::cout << "[GoalManager] → IDLE\n"; break;
        case STATE_EXPLORING:      std::cout << "[GoalManager] → EXPLORING\n"; break;
        case STATE_RETURNING_HOME: std::cout << "[GoalManager] → RETURNING_HOME\n"; break;
        case STATE_EMERGENCY_STOP: std::cout << "[GoalManager] → EMERGENCY STOP\n"; break;
    }
}

// -----------------------------------------------------------
//            CHECK IF CURRENT GOAL IS ACHIEVED
// -----------------------------------------------------------

bool GoalManager::is_current_goal_achieved(const Pose2D& current_pose) const
{
    float dx = current_pose.x - current_target_.target_pose.x;
    float dy = current_pose.y - current_target_.target_pose.y;
    return std::sqrt(dx*dx + dy*dy) < 0.20f;
}

// -----------------------------------------------------------
//                MAIN UPDATE FUNCTION
// -----------------------------------------------------------

MissionGoal GoalManager::update_goal(const Pose2D& current_pose,
                                     const BayesianOccupancyGrid& grid)
{
    switch (current_state_)
    {
        // ------------------ IDLE ------------------
        case STATE_IDLE:
        {
            current_target_.target_pose = current_pose;
            current_target_.type        = MissionGoalType::IDLE;
            return current_target_;
        }

        // ------------------ EXPLORING ------------------
        case STATE_EXPLORING:
        {
            if (is_current_goal_achieved(current_pose))
            {
                int robot_gx, robot_gy;
                if (!world_to_grid(grid, current_pose.x, current_pose.y,
                                   robot_gx, robot_gy))
                {
                    std::cout << "[GoalManager] Robot outside map!\n";
                    return current_target_;
                }

                float best_dist = std::numeric_limits<float>::max();
                int best_x = -1, best_y = -1;

                for (int y = 0; y < grid.grid_size_y; y++)
                {
                    for (int x = 0; x < grid.grid_size_x; x++)
                    {
                        if (!is_frontier_cell(grid, x, y)) continue;

                        float dx = (x - robot_gx) * grid.grid_resolution;
                        float dy = (y - robot_gy) * grid.grid_resolution;
                        float d  = std::sqrt(dx*dx + dy*dy);

                        if (d < best_dist)
                        {
                            best_dist = d;
                            best_x = x;
                            best_y = y;
                        }
                    }
                }

                if (best_x < 0)
                {
                    std::cout << "[GoalManager] No more frontiers → IDLE.\n";
                    current_state_ = STATE_IDLE;
                    current_target_.type = MissionGoalType::IDLE;
                    return current_target_;
                }

                current_target_.target_pose.x = best_x * grid.grid_resolution;
                current_target_.target_pose.y = best_y * grid.grid_resolution;
                current_target_.target_pose.theta = 0;
                current_target_.type = MissionGoalType::EXPLORATION_NODE;

                std::cout << "[GoalManager] New frontier chosen: ("
                          << current_target_.target_pose.x << ", "
                          << current_target_.target_pose.y << ")\n";
            }

            return current_target_;
        }

        // ------------------ RETURNING HOME ------------------
        case STATE_RETURNING_HOME:
        {
            current_target_.target_pose = home_pose_;
            current_target_.type        = MissionGoalType::RETURN_HOME;

            if (is_current_goal_achieved(current_pose))
            {
                std::cout << "[GoalManager] Home reached → IDLE\n";
                current_state_ = STATE_IDLE;
            }
            return current_target_;
        }

        // ------------------ EMERGENCY STOP ------------------
        case STATE_EMERGENCY_STOP:
        default:
        {
            current_target_.type = MissionGoalType::IDLE;
            return current_target_;
        }
    }
}
