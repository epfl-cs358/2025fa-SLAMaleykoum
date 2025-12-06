#include "esp1/planning/mission_planner.h"
#include <cmath>
#include <queue>
#include <iostream>
#include <limits>

// -----------------------------------------------------------
// HELPERS
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

    int dx[4] = {1,-1,0,0};
    int dy[4] = {0,0,1,-1};

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
// CLUSTERING (8-connectivity BFS) — C++11 compatible
// -----------------------------------------------------------

static std::vector<std::vector<std::pair<int,int> > >
cluster_frontiers(const BayesianOccupancyGrid& grid,
                  const std::vector<std::pair<int,int> >& points)
{
    int W = grid.grid_size_x;
    int H = grid.grid_size_y;

    std::vector<std::vector<bool> > visited(H, std::vector<bool>(W,false));

    int dx[8] = {1,-1,0,0,  1,1,-1,-1};
    int dy[8] = {0,0,1,-1,  1,-1,1,-1};

    std::vector<std::vector<std::pair<int,int> > > clusters;

    for (size_t k = 0; k < points.size(); k++)
    {
        int fx = points[k].first;
        int fy = points[k].second;

        if (visited[fy][fx]) continue;

        std::vector<std::pair<int,int> > cluster;
        std::queue<std::pair<int,int> > q;

        q.push(std::make_pair(fx, fy));
        visited[fy][fx] = true;

        while (!q.empty())
        {
            std::pair<int,int> pt = q.front();
            q.pop();

            int x = pt.first;
            int y = pt.second;

            cluster.push_back(std::make_pair(x, y));

            for (int i = 0; i < 8; i++)
            {
                int nx = x + dx[i];
                int ny = y + dy[i];

                if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                if (visited[ny][nx]) continue;
                if (!is_frontier_cell(grid, nx, ny)) continue;

                visited[ny][nx] = true;
                q.push(std::make_pair(nx, ny));
            }
        }

        clusters.push_back(cluster);
    }

    return clusters;
}

// -----------------------------------------------------------
// CONSTRUCTOR
// -----------------------------------------------------------

GoalManager::GoalManager(const Pose2D& initial_home_pose)
    : home_pose_(initial_home_pose),
      current_state_(EXPLORATION_NODE)
{
    current_target_.target_pose = initial_home_pose;
    current_target_.type = EXPLORATION_NODE;
}

// -----------------------------------------------------------
// STATE CHANGE
// -----------------------------------------------------------

void GoalManager::set_mission_state(MissionGoalType st)
{
    current_state_ = st;
}

// -----------------------------------------------------------
// GOAL ACHIEVED?
// -----------------------------------------------------------

bool GoalManager::is_current_goal_achieved(const Pose2D& pose) const
{
    float dx = pose.x - current_target_.target_pose.x;
    float dy = pose.y - current_target_.target_pose.y;
    return std::sqrt(dx*dx + dy*dy) < 0.20f;
}

// -----------------------------------------------------------
// UPDATE GOAL (MAIN LOGIC)
// -----------------------------------------------------------

MissionGoal GoalManager::update_goal(const Pose2D& pose,
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
            // Robot garde son goal si déjà défini
            if (current_target_.type == EXPLORATION_NODE &&
                !(current_target_.target_pose.x == home_pose_.x &&
                  current_target_.target_pose.y == home_pose_.y))
            {
                return current_target_;
            }

            // Collecter les frontières
            std::vector<std::pair<int,int> > points;
            for (int y = 0; y < grid.grid_size_y; y++)
                for (int x = 0; x < grid.grid_size_x; x++)
                    if (is_frontier_cell(grid,x,y))
                        points.push_back(std::make_pair(x,y));

            if (points.empty())
            {
                std::cout << "[GoalManager] No frontiers.\n";
                current_state_ = IDLE;
                return current_target_;
            }

            // Clustering
            remaining_frontier_clusters_ = cluster_frontiers(grid, points);

            // Plus grande frontière
            int best_i = -1;
            int best_size = -1;

            for (size_t i = 0; i < remaining_frontier_clusters_.size(); i++)
            {
                if (remaining_frontier_clusters_[i].size() > best_size)
                {
                    best_size = remaining_frontier_clusters_[i].size();
                    best_i = (int)i;
                }
            }

            if (best_i < 0)
            {
                current_state_ = IDLE;
                return current_target_;
            }

            std::vector<std::pair<int,int> >& frontier =
                remaining_frontier_clusters_[best_i];

            float sx = 0, sy = 0;
            for (size_t i = 0; i < frontier.size(); i++)
            {
                sx += frontier[i].first;
                sy += frontier[i].second;
            }

            sx /= frontier.size();
            sy /= frontier.size();

            current_target_.target_pose.x = sx * grid.grid_resolution;
            current_target_.target_pose.y = sy * grid.grid_resolution;
            current_target_.target_pose.theta = 0;
            current_target_.type = EXPLORATION_NODE;

            std::cout << "[GoalManager] Single frontier goal = ("
                      << current_target_.target_pose.x << ", "
                      << current_target_.target_pose.y << ")\n";

            return current_target_;
        }
    }

    return current_target_;
}
