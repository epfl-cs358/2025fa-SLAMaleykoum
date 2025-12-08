#ifndef ARDUINO
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>

#include <iostream>
#include <cmath>
#include <limits>

#include "../../../include/common/data_types.h"
#include "../../../include/esp1/mapping/occupancy/bayesian_grid.h"
#include "../../../include/esp1/planning/mission_planner.h"

// ===============================================================
//                 CONFIG
// ===============================================================

static const int CELL_SIZE = 5;  // px per grid cell
static const float MOVE_STEP = 0.05f;

std::vector<sf::Color> cluster_colors = {
    sf::Color(255, 80, 80),   // rouge clair
    sf::Color(80, 255, 80),   // vert clair
    sf::Color(80, 80, 255)    // bleu clair
};


// ===============================================================
//                 MAP FILLER (TEST MAP)
// ===============================================================
static void fill_test_map(BayesianOccupancyGrid& grid)
{
    int8_t* map = const_cast<int8_t*>(grid.get_map_data());
    int W = grid.grid_size_x;
    int H = grid.grid_size_y;

    // Tout en inconnu
    for (int i = 0; i < W * H; i++)
        map[i] = 0;

    int cx = W / 2;
    int cy = H / 2;

    struct P { int x,y; };
    std::vector<P> poly = {
        {cx + 50, cy},
        {cx + 30, cy + 40},
        {cx,      cy + 60},
        {cx - 40, cy + 30},
        {cx - 60, cy},
        {cx - 30, cy - 40},
        {cx,      cy - 70},
        {cx + 40, cy - 20}
    };

    // ============
    // INSIDE = FREE
    // ============
    auto inside_poly = [&](int x, int y)
    {
        bool c = false;
        for (int i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
        {
            int xi = poly[i].x, yi = poly[i].y;
            int xj = poly[j].x, yj = poly[j].y;

            bool inter = ((yi > y) != (yj > y)) &&
                (x < float(xj - xi) * (y - yi) / (float(yj - yi) + 1e-6f) + xi);

            if (inter) c = !c;
        }
        return c;
    };

    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            if (inside_poly(x,y))
                map[y * W + x] = -4; // FREE


    // =====================================
    // DRAW ONLY 3 SIDES OF THE POLYGON
    // => PRODUCES EXACTLY 3 FRONTIER CLUSTERS
    // =====================================

    auto draw_line = [&](P a, P b)
    {
        int x0 = a.x, y0 = a.y;
        int x1 = b.x, y1 = b.y;

        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        while (true)
        {
            if (x0 >= 0 && y0 >= 0 && x0 < W && y0 < H)
                map[y0 * W + x0] = 4; // OCC

            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    };

    // On ferme seulement 3 côtés → 3 frontières séparées

    int N = poly.size();

    int f1 = 0;   // côté entre poly[0] et poly[1]
    int f2 = 2;   // côté entre poly[2] et poly[3]
    int f3 = 5;   // côté entre poly[5] et poly[6]
    int f4 = 6; 

    draw_line(poly[f1], poly[(f1+1)%N]);
    draw_line(poly[f2], poly[(f2+1)%N]);
    draw_line(poly[f3], poly[(f3+1)%N]);
    draw_line(poly[f4], poly[(f4+1)%N]);

    // Les autres côtés restent ouverts → pas de frontier là-bas
}

// ===============================================================
//                 HELPERS
// ===============================================================

static inline bool world_to_grid(const BayesianOccupancyGrid& grid,
                                 float x_m, float y_m,
                                 int& gx, int& gy)
{
    gx = int(x_m / grid.grid_resolution);
    gy = int(y_m / grid.grid_resolution);
    return (gx >= 0 && gy >= 0 && gx < grid.grid_size_x && gy < grid.grid_size_y);
}

static inline float lo(const BayesianOccupancyGrid& grid, int x, int y)
{
    return grid.get_map_data()[y * grid.grid_size_x + x];
}

static inline bool is_unknown(float lo) { return lo == 0.0f; }

static inline bool is_free(float lo)
{
    float p = 1.f / (1.f + std::exp(-lo));
    return p < 0.35f;
}

static inline bool is_occupied(float lo)
{
    float p = 1.f / (1.f + std::exp(-lo));
    return p > 0.65f;
}

static bool is_frontier_cell(const BayesianOccupancyGrid& grid, int x, int y)
{
    float l = lo(grid, x, y);
    if (!is_free(l)) return false;

    int dx[4] = {1,-1,0,0};
    int dy[4] = {0,0,1,-1};

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

static void move_robot_towards(Pose2D& robot, const Pose2D& goal)
{
    float dx = goal.x - robot.x;
    float dy = goal.y - robot.y;
    float dist = std::sqrt(dx*dx + dy*dy);

    if (dist < MOVE_STEP) {
        robot.x = goal.x;
        robot.y = goal.y;
        return;
    }

    robot.x += MOVE_STEP * dx / dist;
    robot.y += MOVE_STEP * dy / dist;
}

// ===============================================================
//                 MAIN TEST
// ===============================================================

int main()
{
    int W = 200, H = 200;
    float resolution = 0.07f;

    BayesianOccupancyGrid grid(resolution, W, H);
    fill_test_map(grid);

    Pose2D robot_pose;
    robot_pose.x = (W/2) * resolution;
    robot_pose.y = (H/2) * resolution;
    robot_pose.theta = 0;

    MissionPlanner planner(robot_pose);
    planner.set_mission_state(EXPLORATION_NODE);

    sf::RenderWindow window(
        sf::VideoMode(sf::Vector2u(W * CELL_SIZE, H * CELL_SIZE)),
        "MissionPlanner Frontier Simulation"
    );
    window.setFramerateLimit(30);

    while (window.isOpen())
    {
        while (auto ev = window.pollEvent())
        {
            if (ev->is<sf::Event::Closed>())
                window.close();
        }

        // UPDATE MISSION
        MissionGoal goal = planner.update_goal(robot_pose, grid);
 

        if (planner.get_current_state() == EXPLORATION_NODE)
            move_robot_towards(robot_pose, goal.target_pose);


   // =============================
//           DRAW MAP
// =============================
window.clear(sf::Color::Black);

const int8_t* map = grid.get_map_data();
sf::RectangleShape cell(sf::Vector2f(CELL_SIZE, CELL_SIZE));

for (int y = 0; y < H; y++)
{
    for (int x = 0; x < W; x++)
    {
        float l = map[y * W + x];
        sf::Color c;

        if (is_unknown(l))              c = sf::Color(100,100,100);
        else if (is_occupied(l))        c = sf::Color::Black;
        else if (is_frontier_cell(grid,x,y))  c = sf::Color::Yellow;
        else                            c = sf::Color::White;

        cell.setFillColor(c);
        cell.setPosition(sf::Vector2f(x * CELL_SIZE, y * CELL_SIZE));
        window.draw(cell);
    }
}

// =============================
//        DRAW FRONTIER CLUSTERS
// =============================
auto clusters = planner.get_frontier_clusters();

sf::RectangleShape fcell(sf::Vector2f(CELL_SIZE, CELL_SIZE));

for (int c = 0; c < clusters.size(); c++)
{
    // Sélectionne couleur : 3 premières → rouge / vert / bleu
    sf::Color col = (c < cluster_colors.size())
                        ? cluster_colors[c]
                        : sf::Color(255, 255, 0);  // les autres → jaune

    for (auto &p : clusters[c])
    {
        int x = p.first;
        int y = p.second;

        fcell.setFillColor(col);
        fcell.setPosition(sf::Vector2f(x * CELL_SIZE, y * CELL_SIZE));
        window.draw(fcell);
    }
}


// =============================
//        DRAW ROBOT (RED)
// =============================
int rx, ry;
world_to_grid(grid, robot_pose.x, robot_pose.y, rx, ry);

float robot_radius_px = 10.f;

sf::CircleShape rob(robot_radius_px);
rob.setFillColor(sf::Color::Red);
rob.setOrigin(sf::Vector2f(robot_radius_px, robot_radius_px));
rob.setPosition(sf::Vector2f(rx * CELL_SIZE, ry * CELL_SIZE));

window.draw(rob);

// =============================
//        DRAW GOAL (BLUE)
// =============================
if (planner.get_current_state() == EXPLORATION_NODE)
{
    int gx, gy;
    world_to_grid(grid, goal.target_pose.x, goal.target_pose.y, gx, gy);

    float goal_radius_px = 6.f;

    sf::CircleShape tgt(goal_radius_px);
    tgt.setFillColor(sf::Color(0, 120, 255));  // bleu
    tgt.setOrigin(sf::Vector2f(goal_radius_px, goal_radius_px));
    tgt.setPosition(sf::Vector2f(gx * CELL_SIZE, gy * CELL_SIZE));

    window.draw(tgt);

    // Highlight si le robot touche le goal
    float dx = robot_pose.x - goal.target_pose.x;
    float dy = robot_pose.y - goal.target_pose.y;
    float dist = std::sqrt(dx*dx + dy*dy);

    if (dist < 0.1f)
    {
        tgt.setFillColor(sf::Color(0, 200, 255)); // cyan = goal atteint
        window.draw(tgt);
    }
}

window.display();
} // ← FIN DU while(window.isOpen())

return 0;
}
#endif

