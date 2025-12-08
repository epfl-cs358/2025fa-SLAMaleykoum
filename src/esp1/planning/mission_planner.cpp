#include "esp1/planning/mission_planner.h"
#include <cmath>
#include <queue>
#include <iostream>
#include <limits>

bool MissionPlanner::first = true;

// ============================================================
// HELPERS : CONVERT + LOGODDS + FRONTIER CHECK
// ============================================================

static inline float lo(const BayesianOccupancyGrid& grid, int x, int y)
{
    return grid.get_map_data()[y * grid.grid_size_x + x];
}

static inline bool is_unknown(float lo)
{
    return lo == 0.0f;
}

static inline bool is_free(float lo)
{
    float p = 1.f / (1.f + std::exp(-lo));
    return p < 0.35f;
}

static bool is_frontier_cell(const BayesianOccupancyGrid& grid, int x, int y)
{
    float v = lo(grid, x, y);
    if (!is_free(v))
        return false;

    static int dx[4] = {1, -1, 0, 0};
    static int dy[4] = {0, 0, 1, -1};

    for (int i = 0; i < 4; i++)
    {
        int nx = x + dx[i];
        int ny = y + dy[i];

        if (nx < 0 || ny < 0 || nx >= grid.grid_size_x || ny >= grid.grid_size_y)
            continue;

        if (is_unknown(lo(grid, nx, ny)))
            return true;
    }
    return false;
}

// ============================================================
// CLUSTER FRONTIERS (8-connectivity BFS)
// ============================================================

static std::vector<std::vector<std::pair<int,int>>>
cluster_frontiers(const BayesianOccupancyGrid& grid,
                  const std::vector<std::pair<int,int>>& pts)
{
    int W = grid.grid_size_x;
    int H = grid.grid_size_y;

    std::vector<std::vector<bool>> visited(H, std::vector<bool>(W, false));

    static int dx[8] = {1,-1,0,0, 1,1,-1,-1};
    static int dy[8] = {0,0,1,-1, 1,-1,1,-1};

    std::vector<std::vector<std::pair<int,int>>> clusters;

    for (auto &p : pts)
    {
        int sx = p.first;
        int sy = p.second;

        if (visited[sy][sx])
            continue;

        std::vector<std::pair<int,int>> cluster;
        std::queue<std::pair<int,int>> q;

        q.push({sx, sy});
        visited[sy][sx] = true;

        while (!q.empty())
        {
            auto [x, y] = q.front();
            q.pop();

            cluster.push_back({x, y});

            for (int i = 0; i < 8; i++)
            {
                int nx = x + dx[i];
                int ny = y + dy[i];

                if (nx < 0 || ny < 0 || nx >= W || ny >= H)
                    continue;
                if (visited[ny][nx])
                    continue;
                if (!is_frontier_cell(grid, nx, ny))
                    continue;

                visited[ny][nx] = true;
                q.push({nx, ny});
            }
        }

        clusters.push_back(cluster);
    }

    return clusters;
}

// ============================================================
// CONSTRUCTOR
// ============================================================

MissionPlanner::MissionPlanner(const Pose2D& initial_home_pose)
    : home_pose_(initial_home_pose),
      current_state_(EXPLORATION_NODE)
{
    current_target_.target_pose = initial_home_pose;
    current_target_.type = EXPLORATION_NODE;
}

// ============================================================
// CHANGE STATE
// ============================================================

void MissionPlanner::set_mission_state(MissionGoalType st)
{
    current_state_ = st;
}

// ============================================================
// ADD WAYPOINT
// ============================================================

void MissionPlanner::add_user_waypoint(const Pose2D& wp)
{
    waypoint_queue_.push_back(wp);
}

// ============================================================
// GOAL ACHIEVED?
// ============================================================

bool MissionPlanner::is_current_goal_achieved(const Pose2D& pose) const
{
    if(current_target_.type == EXPLORATION_NODE &&
        current_target_.target_pose.x == home_pose_.x &&
        current_target_.target_pose.y == home_pose_.y){
        return false; 
    }
    else{
        float dx = pose.x - current_target_.target_pose.x;
        float dy = pose.y - current_target_.target_pose.y;
        return std::sqrt(dx*dx + dy*dy) < 0.20f;
    }
}

// ============================================================
// MAIN LOGIC
// ============================================================

MissionGoal MissionPlanner::update_goal(const Pose2D& pose,
                                        const BayesianOccupancyGrid& grid)
{
    switch (current_state_)
    {
        case IDLE:
            return current_target_;

        case RETURN_HOME:
            current_target_.target_pose = home_pose_;
            current_target_.type = RETURN_HOME;
            return current_target_;

        case EXPLORATION_NODE:
        {
            // ============================================================
            // 1) SI CE N'EST PAS LA PREMIÈRE FOIS → garder le même goal
            // ============================================================
            if (!first) {
                return current_target_;
            }

            // Marquer que le goal a déjà été calculé une fois
            first = false;

            // ============================================================
            // 2) COLLECTE DES FRONTIERS
            // ============================================================
            std::vector<std::pair<int,int>> pts;
            for (int y = 0; y < grid.grid_size_y; y++)
                for (int x = 0; x < grid.grid_size_x; x++)
                    if (is_frontier_cell(grid, x, y))
                        pts.push_back({x, y});

            std::cout << "frontier count = " << pts.size() << std::endl;


            if (pts.empty()) {
                std::cout << "[MissionPlanner] No frontiers.\n";
                current_state_ = IDLE;
                return current_target_;
            }

            // ============================================================
            // 3) CLUSTERISATION
            // ============================================================
            remaining_frontier_clusters_ = cluster_frontiers(grid, pts);

            std::cout << "remaining_frontier_clusters_ = " << remaining_frontier_clusters_.size() << std::endl;

            // Choisir cluster le plus grand
            int best_i = -1;
            int best_size = -1;

            for (int i = 0; i < remaining_frontier_clusters_.size(); i++)      

            {
                std::cout << "frontiere size = " <<  remaining_frontier_clusters_[i].size() << std::endl;
                std::cout << "pts of biggest frontier = " << best_size << std::endl;

                std::cout << "address of best_size = " << &best_size << std::endl;
                std::cout << "address of cluster size = " << &remaining_frontier_clusters_[i] << std::endl;


                if (best_size < 0 || remaining_frontier_clusters_[i].size() > (size_t)best_size)
                {
                    std::cout << "frontiere size = " <<  remaining_frontier_clusters_[i].size() << std::endl;
                    puts("in"); 
                    best_size = remaining_frontier_clusters_[i].size();
                    best_i = i;
                }
            }

            std::cout << "pts of biggest frontier = " << best_size << std::endl;

            if (best_i < 0)
                return current_target_;

            auto& cluster = remaining_frontier_clusters_[best_i];

            // ============================================================
            // 4) CENTRE DU CLUSTER → nouveau goal
            // ============================================================
            float sx = 0, sy = 0;
            for (auto& p : cluster) {
                sx += p.first;
                sy += p.second;
            }
            sx /= cluster.size();
            sy /= cluster.size();

            current_target_.target_pose.x = sx * grid.grid_resolution;
            current_target_.target_pose.y = sy * grid.grid_resolution;
            current_target_.target_pose.theta = 0;

            current_target_.type = EXPLORATION_NODE;

            return current_target_;
        }
    }

    return current_target_;
}

