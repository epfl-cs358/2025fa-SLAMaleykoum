#include "esp1/planning/mission_planner.h"
#include <cmath>
#include <queue>
#include <iostream>
#include <limits>

bool MissionPlanner::first = true;

// ============================================================
// HELPERS : LOGODDS + FRONTIER CHECK
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
    return p < 0.35f;     // FREE
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
    first = true;
    current_target_.target_pose = initial_home_pose;
    current_target_.type = EXPLORATION_NODE;
}

// ============================================================
// STATE CHANGE
// ============================================================

void MissionPlanner::set_mission_state(MissionGoalType st)
{
    current_state_ = st;
}

// ============================================================
// ADD USER WP
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
    float dx = pose.x - current_target_.target_pose.x;
    float dy = pose.y - current_target_.target_pose.y;
    return std::sqrt(dx*dx + dy*dy) < 0.20f;
}

// ============================================================
// MAIN LOGIC (ROBOT-SIZE AWARE)
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
            // 1) Only compute a goal ONCE
            if (!first)
                return current_target_;
            first = false;

            // 2) Detect frontier cells
            std::vector<std::pair<int,int>> pts;
            for (int y = 0; y < grid.grid_size_y; y++)
                for (int x = 0; x < grid.grid_size_x; x++)
                    if (is_frontier_cell(grid, x, y))
                        pts.push_back({x, y});

            if (pts.empty()) {
                current_state_ = IDLE;
                return current_target_;
            }

            // 3) Cluster frontiers
            remaining_frontier_clusters_ = cluster_frontiers(grid, pts);

            // 4) FILTER clusters too small for the robot to pass
            float robot_diameter = ROBOT_RADIUS * 2.0f;
            int min_cells = std::max(3, (int)std::ceil(robot_diameter / grid.grid_resolution));

            std::vector<std::vector<std::pair<int,int>>> filtered;
            for (auto& c : remaining_frontier_clusters_)
                if ((int)c.size() >= min_cells)
                    filtered.push_back(c);

            remaining_frontier_clusters_ = filtered;

            if (remaining_frontier_clusters_.empty()) {
                std::cout << "No cluster wide enough for robot.\n";
                current_state_ = IDLE;
                return current_target_;
            }

            // 5) Select the largest cluster
            int best_i = -1;
            size_t best_size = 0;

            for (int i = 0; i < remaining_frontier_clusters_.size(); i++)
            {
                size_t s = remaining_frontier_clusters_[i].size();
                if (s > best_size) {
                    best_size = s;
                    best_i = i;
                }
            }

            if (best_i < 0)
                return current_target_;

            auto& cluster = remaining_frontier_clusters_[best_i];

            // 6) Compute centroid (default goal)
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
