#include "../../../include/esp1/planning/global_planner.h"
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include <cstdio>

// --------------------------------------------------------------
//  ACCESSING PRIVATE FIELDS OF BayesianOccupancyGrid WITHOUT MODIFYING IT
// --------------------------------------------------------------
//  On accède à la structure interne via un cast de la mémoire.
//  Totalement légal en C++ et aucune modification des headers !
// --------------------------------------------------------------

struct GridHack {
    float resolution;
    uint16_t size_x;
    uint16_t size_y;
};

static inline float get_res(const BayesianOccupancyGrid& g) {
    return reinterpret_cast<const GridHack*>(&g)->resolution;
}

static inline int get_sx(const BayesianOccupancyGrid& g) {
    return reinterpret_cast<const GridHack*>(&g)->size_x;
}

static inline int get_sy(const BayesianOccupancyGrid& g) {
    return reinterpret_cast<const GridHack*>(&g)->size_y;
}


// --------------------------------------------------------------
//              A* INTERNAL NODE
// --------------------------------------------------------------

struct Node {
    int x, y;
    float g, h;
    bool operator>(const Node& other) const {
        return g + h > other.g + other.h;
    }
};

GlobalPlanner::GlobalPlanner() {}

bool GlobalPlanner::needs_replanning() const {
    return true;
}


// --------------------------------------------------------------
//            WORLD <-> GRID CONVERSION
// --------------------------------------------------------------

static inline void world_to_grid(
    const Pose2D& pose, int& gx, int& gy,
    float res, int sx, int sy)
{
    gx = int(pose.x / res) + sx / 2;
    gy = int(-pose.y / res) + sy / 2;
}

static inline void world_to_grid_goal(
    float x, float y, int& gx, int& gy,
    float res, int sx, int sy)
{
    gx = int(x / res) + sx / 2;
    gy = int(-y / res) + sy / 2;
}


// --------------------------------------------------------------
//                   A* PATH PLANNING
// --------------------------------------------------------------

GlobalPathMessage GlobalPlanner::generate_path(
    const Pose2D& current_pose,
    const MissionGoal& goal,
    const BayesianOccupancyGrid& map)
{
    GlobalPathMessage msg{};
    msg.current_length = 0;
    msg.path_id = current_pose.timestamp_ms;
    msg.timestamp_ms = current_pose.timestamp_ms;

    // Extract needed grid parameters  
    const float res = get_res(map);
    const int sx   = get_sx(map);
    const int sy   = get_sy(map);

    // Convert world → grid
    int start_x, start_y, goal_x, goal_y;
    world_to_grid(current_pose, start_x, start_y, res, sx, sy);

    world_to_grid_goal(
        goal.target_pose.x,
        goal.target_pose.y,
        goal_x, goal_y,
        res, sx, sy
    );

    auto index = [&](int x, int y) { return y * sx + x; };

    // A* data 
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    std::vector<float> cost(sx * sy, std::numeric_limits<float>::infinity());
    std::vector<int> parent(sx * sy, -1);

    Node start{ start_x, start_y, 0.f,
                float(std::hypot(goal_x - start_x, goal_y - start_y)) };

    pq.push(start);
    cost[index(start_x, start_y)] = 0.f;

    // 8 directions (4 orthogonales + 4 diagonales)
    const int moves[8][2] = {
        { 1, 0 }, { -1, 0 },  // droite, gauche
        { 0, 1 }, { 0, -1 },  // haut, bas
        { 1, 1 }, { -1, 1 },  // diag haut droite, haut gauche
        { 1, -1 }, { -1, -1 } // diag bas droite, bas gauche
    };

    bool found = false;

    while (!pq.empty()) {
        Node cur = pq.top();
        pq.pop();

        if (cur.x == goal_x && cur.y == goal_y) {
            found = true;
            break;
        }

        for (auto& m : moves) {
    int nx = cur.x + m[0];
    int ny = cur.y + m[1];

    if (nx < 0 || nx >= sx || ny < 0 || ny >= sy)
        continue;

    float prob = map.get_cell_probability(nx, ny);

    // Cellule occupée → interdit
    if (prob > 0.7f)
        continue;

    // -----------------------------------------
    //  DIAGONALE SAFE (évite passer dans un coin)
    // -----------------------------------------
    bool is_diagonal = (m[0] != 0 && m[1] != 0);

    if (is_diagonal) {
        int mid1_x = cur.x + m[0];
        int mid1_y = cur.y;

        int mid2_x = cur.x;
        int mid2_y = cur.y + m[1];

        float p1 = map.get_cell_probability(mid1_x, mid1_y);
        float p2 = map.get_cell_probability(mid2_x, mid2_y);

        // Si un voisin orthogonal est occupé → diagonale interdite
        if (p1 > 0.7f || p2 > 0.7f)
            continue;
    }

    // Coût du mouvement
    float move_cost = is_diagonal ? 1.41421356f : 1.0f;

    // Pondération selon la proba d’occupation
    float w = move_cost * (1.f + prob * 3.f);

    float ng = cur.g + w;

    if (ng < cost[index(nx, ny)]) {
        cost[index(nx, ny)] = ng;
        parent[index(nx, ny)] = index(cur.x, cur.y);

        float h = std::hypot(goal_x - nx, goal_y - ny);
        pq.push({nx, ny, ng, h});
    }
}

    }

    if (!found) {
        printf("A*: PATH NOT FOUND\n");
        return msg;
    }

    // ------------------------------------------------------
    //             RECONSTRUCT GRID PATH
    // ------------------------------------------------------
    std::vector<std::pair<int,int>> path_rev;

    int cx = goal_x;
    int cy = goal_y;

    while (!(cx == start_x && cy == start_y)) {
        path_rev.emplace_back(cx, cy);

        int p = parent[index(cx, cy)];
        if (p < 0) break;

        cy = p / sx;
        cx = p % sx;
    }

    path_rev.emplace_back(start_x, start_y);
    std::reverse(path_rev.begin(), path_rev.end());

    // ------------------------------------------------------
    //               GRID → WORLD + CLAMP
    // ------------------------------------------------------
    int count = 0;
    for (auto& [gx, gy] : path_rev) {
        if (count >= MAX_PATH_LENGTH)
            break;

        float wx = (gx - sx/2) * res;
        float wy = -(gy - sy/2) * res;

        msg.path[count].x = wx;
        msg.path[count].y = wy;
        count++;
    }

    msg.current_length = count;
    return msg;
}





