#include "../../../include/esp1/planning/global_planner.h"
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include <cstdio>

// ---------------------------------------------------------
//                    A* NODE
// ---------------------------------------------------------
struct Node {
    int x, y;
    float g, h;

    bool operator>(const Node& o) const {
        return g + h > o.g + o.h;
    }
};

GlobalPlanner::GlobalPlanner() {}


// ---------------------------------------------------------
//              COLLISION CHECK WITH ROBOT RADIUS
// ---------------------------------------------------------

static inline bool robot_circle_collides(
    int cx, int cy,
    const BayesianOccupancyGrid& map)
{
    const int sx = map.grid_size_x;
    const int sy = map.grid_size_y;

    int radius_cells = int(std::ceil(ROBOT_RADIUS / map.grid_resolution));

    for (int dy = -radius_cells; dy <= radius_cells; dy++) {
        for (int dx = -radius_cells; dx <= radius_cells; dx++) {

            if (dx*dx + dy*dy > radius_cells*radius_cells)
                continue;

            int nx = cx + dx;
            int ny = cy + dy;

            if (nx < 0 || nx >= sx || ny < 0 || ny >= sy)
                return true;

            if (map.get_cell_probability(nx, ny) > 0.7f)
                return true;
        }
    }
    return false;
}


// ---------------------------------------------------------
//            WORLD <-> GRID CONVERSION
// ---------------------------------------------------------

static inline void world_to_grid(
    const Pose2D& pose,
    int& gx, int& gy,
    float res, int sx, int sy)
{
    gx = int(pose.x / res) + sx / 2;
    gy = int(-pose.y / res) + sy / 2;
}

static inline void world_to_grid_goal(
    float x, float y,
    int& gx, int& gy,
    float res, int sx, int sy)
{
    gx = int(x / res) + sx / 2;
    gy = int(-y / res) + sy / 2;
}


// ---------------------------------------------------------
//                       A* PLANNER
// ---------------------------------------------------------

GlobalPathMessage GlobalPlanner::generate_path(
    const Pose2D& current_pose,
    const MissionGoal& goal,
    const BayesianOccupancyGrid& map)
{
    GlobalPathMessage msg{};
    msg.current_length = 0;
    msg.path_id = 0;
    msg.timestamp_ms = 0;

    const float res = map.grid_resolution;
    const int sx   = map.grid_size_x;
    const int sy   = map.grid_size_y;

    if (sx <= 0 || sy <= 0) return msg;

    int start_x, start_y, goal_x, goal_y;
    world_to_grid(current_pose, start_x, start_y, res, sx, sy);
    world_to_grid_goal(goal.target_pose.x, goal.target_pose.y, goal_x, goal_y, res, sx, sy);

    auto inside = [&](int x, int y) {
        return (x >= 0 && x < sx && y >= 0 && y < sy);
    };

    if (!inside(start_x,start_y) || !inside(goal_x,goal_y)) return msg;

    auto index = [&](int x, int y){ return y * sx + x; };

    std::vector<float> g_cost(sx * sy, std::numeric_limits<float>::infinity());
    std::vector<int> parent(sx * sy, -1);

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

    Node start{
        start_x, start_y,
        0.f,
        float(std::hypot(goal_x - start_x, goal_y - start_y))
    };

    pq.push(start);
    g_cost[index(start_x, start_y)] = 0.f;

    const int moves[8][2] = {
        {1,0},{-1,0},{0,1},{0,-1},
        {1,1},{1,-1},{-1,1},{-1,-1}
    };

    bool found = false;

    // ---------------------- A* LOOP ------------------------
    while (!pq.empty()) {

        Node cur = pq.top();
        pq.pop();

        if (cur.x == goal_x && cur.y == goal_y) {
            found = true;
            break;
        }

        int cur_index = index(cur.x, cur.y);

        for (auto& m : moves) {

            int nx = cur.x + m[0];
            int ny = cur.y + m[1];

            if (!inside(nx, ny))
                continue;

            // COLLISION CHECK (robot circle)
            if (robot_circle_collides(nx, ny, map))
                continue;

            // movement cost
            bool diag = (m[0] != 0 && m[1] != 0);
            float step = diag ? 1.41421356f : 1.f;

            float prob = map.get_cell_probability(nx, ny);
            float weight = step * (1.f + prob * 2.f);

            float new_g = g_cost[cur_index] + weight;
            int n_index = index(nx, ny);

            if (new_g < g_cost[n_index]) {
                g_cost[n_index] = new_g;
                parent[n_index] = cur_index;

                float h = std::hypot(goal_x - nx, goal_y - ny);
                pq.push({nx, ny, new_g, h});
            }
        } // <-- FIN DU FOR MANQUANT AVANT !!
    }

    if (!found) return msg;

    // -----------------------------------------------------
    //            RECONSTRUCT PATH
    // -----------------------------------------------------

    std::vector<std::pair<int,int>> rev;
    int cx = goal_x, cy = goal_y;

    while (!(cx == start_x && cy == start_y)) {
        rev.emplace_back(cx, cy);
        int p = parent[index(cx,cy)];
        if (p < 0) return msg;
        cy = p / sx;
        cx = p % sx;
    }

    rev.emplace_back(start_x, start_y);
    std::reverse(rev.begin(), rev.end());

    // -----------------------------------------------------
    //            GRID → WORLD
    // -----------------------------------------------------

    int count = 0;
    for (auto& c : rev) {
        if (count >= MAX_PATH_LENGTH) break;
        msg.path[count].x = (c.first - sx/2) * res;
        msg.path[count].y = -(c.second - sy/2) * res;
        count++;
    }

    msg.current_length = count;
    return msg;
}
