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

    // --- Centre de la grille = (0,0) monde ---
    int cx = W / 2;
    int cy = H / 2;

    // Taille du carré free : 50 × 50
    int half = 25;

    int x0 = cx - half;
    int x1 = cx + half;
    int y0 = cy - half;
    int y1 = cy + half;

    // --- 1. Intérieur FREE ---
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++)
            map[y * W + x] = -2.0f;   // FREE

    // --- 2. Bordures OCCUPIED ---
    for (int x = x0; x <= x1; x++)
    {
        map[y0 * W + x] = 3.0f; // haut
        map[y1 * W + x] = 3.0f; // bas
    }

    for (int y = y0; y <= y1; y++)
    {
        map[y * W + x0] = 3.0f; // gauche
        map[y * W + x1] = 3.0f; // droite
    }

    // --- 3. Passage dans l’angle supérieur droit ---
    map[y0 * W + x1] = -2.0f;          // angle free
    map[(y0 + 1) * W + x1] = -2.0f;    // en dessous
    map[y0 * W + (x1 - 1)] = -2.0f;    // à gauche
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
    manager.set_mission_state(GoalManager::STATE_EXPLORING);

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

        if (manager.get_current_state() == GoalManager::STATE_EXPLORING)
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
        if (manager.get_current_state() == GoalManager::STATE_EXPLORING)
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

