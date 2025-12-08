#include "esp1/planning/mission_planner.h"
#include <cmath>
#include <queue>
#include <iostream>
#include <limits>



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
// SAFETY CHECK FOR ROBOT RADIUS
// ============================================================

static bool is_point_safe(const BayesianOccupancyGrid& grid, int gx, int gy)
{
    int W = grid.grid_size_x;
    int H = grid.grid_size_y;

    int radius_cells = (int)(ROBOT_RADIUS / grid.grid_resolution);

    for (int dy = -radius_cells; dy <= radius_cells; dy++)
    {
        for (int dx = -radius_cells; dx <= radius_cells; dx++)
        {
            int nx = gx + dx;
            int ny = gy + dy;

            if (nx < 0 || ny < 0 || nx >= W || ny >= H)
                continue;

            float d = std::sqrt(dx*dx + dy*dy);
            if (d > radius_cells)
                continue;

            float lo_val = lo(grid, nx, ny);
            float p = 1.f / (1.f + std::exp(-lo_val));

            if (p > 0.65f) // OCCUPIED
                return false;
        }
    }

    return true;
}

// ============================================================
// CLUSTER FRONTIERS (8-NEIGHBOR BFS)
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
// STATE MANAGEMENT
// ============================================================

void MissionPlanner::set_mission_state(MissionGoalType st)
{
    current_state_ = st;
}

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
            

            // -----------------------------------------
            // 1) COLLECT FRONTIER CELLS
            // -----------------------------------------
            std::vector<std::pair<int,int>> pts;
            for (int y = 0; y < grid.grid_size_y; y++)
                for (int x = 0; x < grid.grid_size_x; x++)
                    if (is_frontier_cell(grid, x, y))
                        pts.push_back({x, y});

            if (pts.empty())
            {
                std::cout << "[MissionPlanner] No frontier cells.\n";
                current_state_ = IDLE;
                return current_target_;
            }

            // -----------------------------------------
            // 2) CLUSTER THEM
            // -----------------------------------------
            remaining_frontier_clusters_ = cluster_frontiers(grid, pts);

            // -----------------------------------------
            // 3) SORT CLUSTERS BY SIZE (largest → smallest)
            // -----------------------------------------
            std::vector<int> order(remaining_frontier_clusters_.size());
            for (int i = 0; i < order.size(); i++) order[i] = i;

            std::sort(order.begin(), order.end(),
                [&](int a, int b)
                {
                    return remaining_frontier_clusters_[a].size() >
                           remaining_frontier_clusters_[b].size();
                }
            );

            // Robot in grid coordinates
            int rx = (int)(pose.x / grid.grid_resolution);
            int ry = (int)(pose.y / grid.grid_resolution);

            // -----------------------------------------
            // 4) TRY CLUSTERS ONE BY ONE
            // -----------------------------------------

            bool found_goal = false;
            int best_x = -1;
            int best_y = -1;

            for (int idx : order)
            {
                auto& cluster = remaining_frontier_clusters_[idx];

                float best_dist = std::numeric_limits<float>::max();
                int local_x = -1;
                int local_y = -1;

                // Try to find safe closest point in THIS cluster
                for (auto& p : cluster)
                {
                    int gx = p.first;
                    int gy = p.second;

                    if (!is_point_safe(grid, gx, gy))
                        continue;

                    float dx = (gx - rx) * grid.grid_resolution;
                    float dy = (gy - ry) * grid.grid_resolution;
                    float dist = std::sqrt(dx*dx + dy*dy);

                    if (dist < best_dist)
                    {
                        best_dist = dist;
                        local_x = gx;
                        local_y = gy;
                    }
                }

                if (local_x >= 0)
                {
                    best_x = local_x;
                    best_y = local_y;
                    found_goal = true;
                    break;
                }
            }

            // -----------------------------------------
            // 5) IF NO SAFE POINT IN ANY CLUSTER
            // -----------------------------------------
            if (!found_goal)
            {
                std::cout << "[MissionPlanner] No SAFE frontier in ANY cluster.\n";
                current_state_ = IDLE;
                return current_target_;
            }

            // -----------------------------------------
            // 6) SET GOAL ON SAFE POINT FOUND
            // -----------------------------------------
            current_target_.target_pose.x = best_x * grid.grid_resolution;
            current_target_.target_pose.y = best_y * grid.grid_resolution;
            current_target_.target_pose.theta = 0;
            current_target_.type = EXPLORATION_NODE;

            return current_target_;
        }
    }

    return current_target_;
}
