#include "esp1/planning/mission_planner.h"
#include "esp1/mapping/occupancy/bayesian_grid.h"
#include <iostream>
#include <cmath>
#include <limits>
#include <queue>


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
    : current_state_(STATE_EXPLORING),
      home_pose_(initial_home_pose)
{
    current_target_.target_pose = home_pose_;
    current_target_.type        = MissionGoalType::EXPLORATION_NODE;

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
//            CLUSTER FRONTIER CELLS
// -----------------------------------------------------------
static std::vector<std::vector<std::pair<int,int>>> 
cluster_frontiers(const BayesianOccupancyGrid& grid,
                  const std::vector<std::pair<int,int>>& frontiers)
{
    int W = grid.grid_size_x;
    int H = grid.grid_size_y;

    // Visité = tableau 2D
    std::vector<std::vector<bool>> visited(H, std::vector<bool>(W, false));

    // Directions 8-connectivity
    int dx[8] = {1,-1,0,0,  1,1,-1,-1};
    int dy[8] = {0,0,1,-1,  1,-1,1,-1};

    std::vector<std::vector<std::pair<int,int>>> clusters;

    for (auto& f : frontiers)
    {
        int fx = f.first;
        int fy = f.second;

        if (visited[fy][fx]) continue;

        // BFS pour un cluster
        std::vector<std::pair<int,int>> cluster;
        std::queue<std::pair<int,int>> q;
        q.push({fx, fy});
        visited[fy][fx] = true;

        while (!q.empty())
        {
            std::pair<int,int> p = q.front();
            q.pop();

            int x = p.first;
            int y = p.second;
            cluster.push_back(std::make_pair(x,y));

            for (int i = 0; i < 8; i++)
            {
                int nx = x + dx[i];
                int ny = y + dy[i];

                if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                if (visited[ny][nx]) continue;
                if (!is_frontier_cell(grid, nx, ny)) continue;

                visited[ny][nx] = true;
                q.push({nx, ny});
            }
        }

        clusters.push_back(cluster);
    }

    return clusters;
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
            // Si on a atteint le goal précédent → nouveau goal
            if (is_current_goal_achieved(current_pose))
            {
                int robot_gx, robot_gy;
                if (!world_to_grid(grid, current_pose.x, current_pose.y,
                                   robot_gx, robot_gy))
                {
                    std::cout << "[GoalManager] Robot outside map!\n";
                    return current_target_;
                }

                // ----------------------------------------------------
                // 1. Collecter TOUTES les cellules frontières
                // ----------------------------------------------------
                std::vector<std::pair<int,int>> all_frontiers;

                for (int y = 0; y < grid.grid_size_y; y++)
                {
                    for (int x = 0; x < grid.grid_size_x; x++)
                    {
                        if (is_frontier_cell(grid, x, y))
                            all_frontiers.push_back({x, y});
                    }
                }

                if (all_frontiers.empty())
                {
                    std::cout << "[GoalManager] No more frontiers → IDLE.\n";
                    current_state_ = STATE_IDLE;
                    current_target_.type = MissionGoalType::IDLE;
                    return current_target_;
                }

                // ----------------------------------------------------
                // 2. CLUSTERING des frontières (segments)
                // ----------------------------------------------------
                auto clusters = cluster_frontiers(grid, all_frontiers);

                if (clusters.empty())
                {
                    std::cout << "[GoalManager] WARNING: clusters empty.\n";
                    current_state_ = STATE_IDLE;
                    return current_target_;
                }

                // ----------------------------------------------------
                // 3. Sélection du segment le plus long
                // ----------------------------------------------------
                int best_idx = -1;
                int best_len = -1;

                for (int i = 0; i < clusters.size(); i++)
                {
                    if (clusters[i].size() > best_len)
                    {
                        best_len = clusters[i].size();
                        best_idx = i;
                    }
                }

                if (best_idx < 0 || best_idx >= clusters.size())
                {
                    std::cout << "[GoalManager] WARNING: best_idx invalid.\n";
                    current_state_ = STATE_IDLE;
                    return current_target_;
                }

                auto& best_cluster = clusters[best_idx];

                if (best_cluster.empty())
                {
                    std::cout << "[GoalManager] WARNING: best cluster empty.\n";

                    // fallback → chercher la frontière la plus proche
                    float best_d = 1e9f;
                    int bx = -1, by = -1;

                    for (auto& f : all_frontiers)
                    {
                        float dx = (f.first - robot_gx) * grid.grid_resolution;
                        float dy = (f.second - robot_gy) * grid.grid_resolution;
                        float d2 = dx*dx + dy*dy;

                        if (d2 < best_d)
                        {
                            best_d = d2;
                            bx = f.first;
                            by = f.second;
                        }
                    }

                    if (bx >= 0)
                    {
                        current_target_.target_pose.x = bx * grid.grid_resolution;
                        current_target_.target_pose.y = by * grid.grid_resolution;
                        current_target_.target_pose.theta = 0;
                        current_target_.type = MissionGoalType::EXPLORATION_NODE;
                        return current_target_;
                    }

                    current_state_ = STATE_IDLE;
                    return current_target_;
                }

                // ----------------------------------------------------
                // 4. Calculer le CENTRE du segment
                // ----------------------------------------------------
                float sx = 0.f;
                float sy = 0.f;

                for (auto& c : best_cluster)
                {
                    sx += c.first;
                    sy += c.second;
                }

                sx /= float(best_cluster.size());
                sy /= float(best_cluster.size());

                // ----------------------------------------------------
                // 5. Conversion en coordonnées monde
                // ----------------------------------------------------
                current_target_.target_pose.x = sx * grid.grid_resolution;
                current_target_.target_pose.y = sy * grid.grid_resolution;
                current_target_.target_pose.theta = 0;
                current_target_.type = MissionGoalType::EXPLORATION_NODE;

                std::cout << "[GoalManager] New frontier segment center: ("
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

