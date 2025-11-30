#include <SFML/Graphics.hpp>
#include <cmath>
#include <vector>
#include <iostream>
#include <thread>
#include <chrono>

#include "../../../include/common/data_types.h"
#include "../../../include/esp1/mapping/occupancy/bayesian_grid.h"

int main() {
    const int grid_size_x = 150;
    const int grid_size_y = 150;
    const float resolution = 0.05f;

    BayesianOccupancyGrid grid(resolution, grid_size_x, grid_size_y);

    const int cell_size = 5;

    sf::RenderWindow window(
        sf::VideoMode({grid_size_x * cell_size, grid_size_y * cell_size}),
        "Bayesian Grid Test (SFML3 + Code 1)"
    );

    // Robot pose — IMPORTANT : theta = radians
    Pose2D robot_pose;
    robot_pose.x = 0.0f;
    robot_pose.y = 0.0f;
    robot_pose.theta = 0.0f;   // MUST BE ZERO OR RADIANS

    // Obstacles in meters
    std::vector<std::pair<float, float>> obstacles = {
        {1.0f,  1.0f},
        {0.0f, -2.0f},
        {-1.5f, -0.5f},
        {2.0f,  2.0f}
    };

    const float lidar_max_range = 3.0f; // meters
    const int lidar_points = 20000;     // 0.1° resolution


    int iteration = 0;

    while (window.isOpen()) {

        // SFML 3.0 event handling
        while (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }

        LiDARScan scan;
        scan.timestamp_ms = iteration * 100;
        scan.count = lidar_points;

        // ---------------- SIMULATE LIDAR ----------------
        for (int i = 0; i < scan.count; ++i) {

            // Angle in degrees
            float angle_deg = i * 0.1f;///
            float angle_rad = angle_deg * (M_PI / 180.0f);///

            float range = 0; 

            // Check all obstacles
            for (auto &obs : obstacles) {
                float dx = obs.first - robot_pose.x;
                float dy = obs.second - robot_pose.y;

                float dist = std::sqrt(dx*dx + dy*dy);
                float obs_angle = std::atan2(dy, dx);

                // Angular diff in a stable way
                float diff = std::fabs(std::atan2(
                    std::sin(angle_rad - obs_angle),
                    std::cos(angle_rad - obs_angle)
                ));

        

                if (diff < 0.1 && dist < lidar_max_range) {
                    if (range == 0 || dist < range) {
                        range = dist;
                    }
                }
            }

            // Fill scan (format expected by SLAM Code 1)
            scan.angles[i]    = angle_deg;       // ° degrees
            scan.distances[i] = range * 1000.0f; // mm                             
            scan.qualities[i] = 255;
        }

        // Pack for the SLAM update
        SyncedScan synced;
        synced.pose = robot_pose;
        synced.scan = scan;

        // ---------------- UPDATE OCCUPANCY GRID ----------------
        grid.update_map(synced, lidar_max_range);

        // ---------------- DRAW MAP ----------------
        window.clear(sf::Color::Black);

        const uint8_t* data = grid.get_map_data_color();

        // SFML 3.0 constructor for Image
        sf::Image img;
        img.resize(sf::Vector2u(grid_size_x, grid_size_y));

        // IMPORTANT: CORRECT DISPLAY (Y NOT FLIPPED)
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

        // ----------- DRAW ROBOT -------------
        sf::CircleShape robot(3);
        robot.setFillColor(sf::Color::Red);

        robot.setPosition(sf::Vector2f(
            (grid_size_x * cell_size) / 2.0f,
            (grid_size_y * cell_size) / 2.0f
        ));

        window.draw(robot);

        window.display();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        iteration++;
    }

    return 0;
}
