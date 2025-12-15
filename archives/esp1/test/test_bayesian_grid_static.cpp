

#ifndef ARDUINO
#include <SFML/Graphics.hpp>
#include <cmath>
#include <vector>
#include <iostream>
#include <thread>
#include <chrono>

#include "../../../include/common/data_types.h"
#include "../../../include/esp1/mapping/occupancy/bayesian_grid.h"

int main() {
    const int grid_size_x = 80;
    const int grid_size_y = 80;
    const float resolution = 0.07f;

    BayesianOccupancyGrid grid(resolution, grid_size_x, grid_size_y);

    const int cell_size = 5;

    sf::RenderWindow window(
        sf::VideoMode({grid_size_x * cell_size, grid_size_y * cell_size}),
        "Bayesian Grid Test (SFML3 + Code 1)"
    );

    // Robot pose
    Pose2D robot_pose{0.0f, 0.0f, 0.0f};

    // Obstacles already on the map
    std::vector<std::pair<float, float>> obstacles = {
        {1.0f,  1.0f},
        {0.0f, -2.0f},
        {-1.5f, -0.5f},
        {2.0f,  2.0f}
    };

    // STACK of dynamic obstacles added with X key
    std::vector<std::pair<float, float>> obstacles_stack = {
        {0.5f, 0.5f},
        {-2.0f, 2.0f},
        {0.5f, 2.5f}
    };

    // Moving obstacle (trajectory: circle around robot)
    std::pair<float,float> moving_obstacle = {1.5f, 0.0f};
    float moving_angle = 0.0f;

    const float lidar_max_range = 3.0f;
    const int lidar_points = 3000;

    int iteration = 0;

    while (window.isOpen()) {

        // ------------------- EVENTS -------------------
        while (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>())
                window.close();

            if (auto* keyEvent = event->getIf<sf::Event::KeyPressed>()) {

                // Press X → pop from stack and add to obstacles
                if (keyEvent->scancode == sf::Keyboard::Scan::X) {

                    if (!obstacles_stack.empty()) {
                        auto newObs = obstacles_stack.back();
                        obstacles_stack.pop_back();
                        obstacles.push_back(newObs);

                        std::cout << "Added obstacle: "
                                  << newObs.first << ", " << newObs.second
                                  << std::endl;
                    }
                    else {
                        std::cout << "Stack empty! No more obstacles." << std::endl;
                    }
                }
            }
        }

        // ------------------- UPDATE MOVING OBSTACLE -------------------
        moving_angle += 0.05f;  // speed of rotation
        moving_obstacle.first  = 1.5f * std::cos(moving_angle);
        moving_obstacle.second = 1.5f * std::sin(moving_angle);

        // ------------------- LIDAR SIM -------------------
        LiDARScan scan;
        scan.timestamp_ms = iteration * 100;
        scan.count = lidar_points;

        for (int i = 0; i < scan.count; ++i) {

            float angle_deg = (360.0f / lidar_points) * i;
            float angle_rad = angle_deg * (M_PI / 180.0f);

            float range = 0;

            // ---- STATIC OBSTACLES ----
            for (auto &obs : obstacles) {
                float dx = obs.first - robot_pose.x;
                float dy = obs.second - robot_pose.y;

                float dist = std::sqrt(dx*dx + dy*dy);
                float obs_angle = std::atan2(dy, dx);

                float diff = std::fabs(std::atan2(
                    std::sin(angle_rad - obs_angle),
                    std::cos(angle_rad - obs_angle)
                ));

                if (diff < 0.1 && dist < lidar_max_range) {
                    if (range == 0 || dist < range)
                        range = dist;
                }
            }

            // ---- MOVING OBSTACLE ----
            {
                float dx = moving_obstacle.first - robot_pose.x;
                float dy = moving_obstacle.second - robot_pose.y;

                float dist = std::sqrt(dx*dx + dy*dy);
                float obs_angle = std::atan2(dy, dx);

                float diff = std::fabs(std::atan2(
                    std::sin(angle_rad - obs_angle),
                    std::cos(angle_rad - obs_angle)
                ));

                if (diff < 0.1 && dist < lidar_max_range) {
                    if (range == 0 || dist < range)
                        range = dist;
                }
            }

            // Fill scan
            scan.angles[i]    = angle_deg;
            scan.distances[i] = range * 1000.0f; // mm
            scan.qualities[i] = 255;
        }

        // ---------------- MAP UPDATE -------------------
        SyncedScan synced;
        synced.pose = robot_pose;
        synced.scan = scan;

        grid.update_map(synced, lidar_max_range);

        // ---------------- DRAW GRID -------------------
        window.clear(sf::Color::Black);

        const uint8_t* data = grid.get_map_data_color();

        sf::Image img;
        img.resize(sf::Vector2u(grid_size_x, grid_size_y));

        for (int y = 0; y < grid_size_y; ++y) {
            for (int x = 0; x < grid_size_x; ++x) {
                uint8_t value = data[y * grid_size_x + x];
                img.setPixel(sf::Vector2u(x, y), sf::Color(value, value, value));
            }
        }

        sf::Texture tex;
        tex.loadFromImage(img);

        sf::Sprite sprite(tex);
        sprite.setScale(sf::Vector2f(cell_size, cell_size));
        window.draw(sprite);

        // ---------------- DRAW ROBOT -------------------
        sf::CircleShape robot(3);
        robot.setFillColor(sf::Color::Red);
        robot.setPosition(sf::Vector2f(
            (grid_size_x * cell_size) / 2.0f,
            (grid_size_y * cell_size) / 2.0f)
        );
        window.draw(robot);

        // ---------------- DRAW MOVING OBSTACLE -------------------
        sf::CircleShape mob(4);
        mob.setFillColor(sf::Color::Green);
        mob.setPosition(sf::Vector2f(
            (moving_obstacle.first  / resolution + grid_size_x/2) * cell_size,
            (grid_size_y/2 - moving_obstacle.second / resolution) * cell_size)
        );
        window.draw(mob);

        window.display();

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        iteration++;
    }

    return 0;
}
#endif
