
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

static const int CELL_SIZE = 4;  // px per grid cell
static const float MOVE_STEP = 0.05f;

// ===============================================================
//                 MAP FILLER (TEST MAP)
// ===============================================================

static void fill_test_map(BayesianOccupancyGrid& grid)
{
    float* map = (float*)grid.get_map_data();
    int W = grid.grid_size_x;
    int H = grid.grid_size_y;

    // Tout en inconnu
    for (int i = 0; i < W * H; i++)
        map[i] = 0.0f;

    // Centre
    int cx = W / 2;
    int cy = H / 2;

    // ------------------------------
    // POLYGONE IRRÉGULIER (8 points)
    // ------------------------------

    struct P { int x,y; };

    std::vector<P> poly = {
        {cx + 50, cy},        // droite
        {cx + 30, cy + 40},   // bas droite
        {cx,      cy + 60},   // bas
        {cx - 40, cy + 30},   // bas gauche
        {cx - 60, cy},        // gauche
        {cx - 30, cy - 40},   // haut gauche
        {cx,      cy - 70},   // haut (loin du centre)
        {cx + 40, cy - 20}    // haut droite
    };

    // ------------------------------
    // REMPLIR L’INTÉRIEUR DU POLYGONE
    // ------------------------------

    auto point_in_poly = [&](int x, int y)
    {
        bool c = false;
        for (int i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
        {
            int xi = poly[i].x, yi = poly[i].y;
            int xj = poly[j].x, yj = poly[j].y;

            bool intersect = ((yi > y) != (yj > y)) &&
                             (x < (float)(xj - xi) * (y - yi) / (float)(yj - yi + 1e-6f) + xi);
            if (intersect) c = !c;
        }
        return c;
    };

    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            if (point_in_poly(x, y))
                map[y * W + x] = -2.0f;   // FREE

    // ------------------------------
    // DESSINER LES CONTOURS OCCUPÉS
    // ------------------------------

    auto draw_line = [&](P a, P b)
    {
        int x0 = a.x, y0 = a.y;
        int x1 = b.x, y1 = b.y;

        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        while (true)
        {
            if (x0 >= 0 && y0 >= 0 && x0 < W && y0 < H)
                map[y0 * W + x0] = 3.0f; // OCCUPIED

            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    };

    // Créer les passages en sautant 3 segments

    int N = poly.size();

    // 1) Passage proche du centre (petit)
    int skip1 = 0;     // segment entre poly[0] et poly[1]

    // 2) Passage à distance moyenne
    int skip2 = 3;     // segment poly[3]→poly[4]

    // 3) Passage très loin du centre (au nord)
    int skip3 = 6;     // segment poly[6]→poly[7]

    for (int i = 0; i < N; i++)
    {
        int j = (i + 1) % N;

        if (i == skip1 || i == skip2 || i == skip3)
            continue; // passage → pas de bordure

        draw_line(poly[i], poly[j]);
    }
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
    return gx >= 0 && gy >= 0 &&
           gx < grid.grid_size_x &&
           gy < grid.grid_size_y;
}

static inline float logodds(const BayesianOccupancyGrid& grid, int x, int y)
{
    return grid.get_map_data()[y * grid.grid_size_x + x];
}

static inline bool is_unknown(float lo) { return std::fabs(lo) < 0.05f; }

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
    float lo = logodds(grid, x, y);
    if (!is_free(lo)) return false;

    int dx[4] = {1,-1,0,0};
    int dy[4] = {0,0,1,-1};

    for (int i = 0; i < 4; i++)
    {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (nx < 0 || ny < 0 || nx >= grid.grid_size_x || ny >= grid.grid_size_y)
            continue;

        if (is_unknown(logodds(grid, nx, ny)))
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
//                 MAIN
// ===============================================================



int main()
{
    int W = 200, H = 200;
    float resolution = 0.05f;

    BayesianOccupancyGrid grid(resolution, W, H);
    fill_test_map(grid);

    Pose2D robot_pose;
    robot_pose.x = 100 * resolution;
    robot_pose.y = 100 * resolution;
    robot_pose.theta = 0;

    GoalManager manager(robot_pose);
    manager.set_mission_state(EXPLORATION_NODE);

    // ============================
    //   SFML 3.0 WINDOW
    // ============================
    sf::RenderWindow window(
        sf::VideoMode(sf::Vector2u(W * CELL_SIZE, H * CELL_SIZE)),
        "GoalManager Frontier Simulation"
    );

    window.setFramerateLimit(30);

    while (window.isOpen())
    {
        // === NOUVELLE API SFML 3 ===
        while (auto ev = window.pollEvent())
        {
            // Fermer fenêtre
            if (ev->is<sf::Event::Closed>())
                window.close();

            // Escape → fermer
            if (auto* key = ev->getIf<sf::Event::KeyPressed>())
            {
                if (key->scancode == sf::Keyboard::Scancode::Escape)
                    window.close();
            }
        }

        // =============================
        //      UPDATE GOAL MANAGER
        // =============================
        MissionGoal goal = manager.update_goal(robot_pose, grid);

        if (manager.get_current_state() == EXPLORATION_NODE)
            move_robot_towards(robot_pose, goal.target_pose);

        // =============================
        //           DRAW MAP
        // =============================
        window.clear(sf::Color::Black);

        float* map = (float*)grid.get_map_data();

        sf::RectangleShape cell(sf::Vector2f(CELL_SIZE, CELL_SIZE));

        for (int y = 0; y < H; y++)
        {
            for (int x = 0; x < W; x++)
            {
                float lo = map[y * W + x];
                sf::Color c;

                if (is_unknown(lo))
                    c = sf::Color(100,100,100);
                else if (is_occupied(lo))
                    c = sf::Color::Black;
                else if (is_frontier_cell(grid, x, y))
                    c = sf::Color::Yellow;
                else
                    c = sf::Color::White;

                cell.setFillColor(c);
                cell.setPosition(sf::Vector2f(x * CELL_SIZE, y * CELL_SIZE));
                window.draw(cell);
            }
        }

        // =============================
        //         DRAW ROBOT
        // =============================
        int rx, ry;
        world_to_grid(grid, robot_pose.x, robot_pose.y, rx, ry);

        sf::CircleShape rob(4);
        rob.setFillColor(sf::Color::Blue);
        rob.setPosition(sf::Vector2f(rx * CELL_SIZE - 2, ry * CELL_SIZE - 2));
        window.draw(rob);

        // =============================
        //         DRAW GOAL (RED)
        // =============================
        if (manager.get_current_state() == EXPLORATION_NODE)
        {
            int gx, gy;
            world_to_grid(grid, goal.target_pose.x, goal.target_pose.y, gx, gy);

            sf::CircleShape tgt(4);
            tgt.setFillColor(sf::Color::Red);
            tgt.setPosition(sf::Vector2f(gx * CELL_SIZE - 2, gy * CELL_SIZE - 2));
            window.draw(tgt);
        }

        window.display();
    }

    return 0;
}

#endif
