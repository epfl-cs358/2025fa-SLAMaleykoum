#include <SFML/Graphics.hpp>
#include <cmath>
#include <vector>
#include <iostream>
#include <thread>
#include <chrono>
#include "../../../include/common/data_types.h"
#include "esp1/mapping/occupancy/bayesian_grid.h"

int main() {
    // --- Parameters ---
    const int grid_size_x = 100;
    const int grid_size_y = 100;
    const float resolution = 0.05f; // 5 cm per cell

    BayesianOccupancyGrid grid(resolution, grid_size_x, grid_size_y);

    // Create SFML window
    const int cell_size = 5;
    sf::RenderWindow window(sf::VideoMode(grid_size_x * cell_size, grid_size_y * cell_size), "Bayesian Grid Test");

    // Fixed robot pose at center
    Pose2D robot_pose;
    robot_pose.x = grid_size_x * resolution / 2;
    robot_pose.y = grid_size_y * resolution / 2;
    robot_pose.theta = 0.0f;

    // Simulated obstacles (in meters)
    std::vector<std::pair<float, float>> obstacles = {
        {2.0f, 0.0f},
        {1.0f, 1.0f},
        {-1.5f, -1.0f},
        {0.0f, -2.0f}
    };

    const int lidar_points = 90; // 90 beams per scan
    const float lidar_max_range = 3.0f;

    int iteration = 0;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // Simulate LiDAR scan
        LiDARScan scan;
        scan.timestamp_ms = iteration * 100;
        scan.landmarks.clear();

        for (int i = 0; i < lidar_points; ++i) {
            float angle = (i / (float)lidar_points) * 2 * M_PI;
            float range = lidar_max_range;

            // Check if beam hits an obstacle
            for (auto &obs : obstacles) {
                float dx = obs.first - robot_pose.x;
                float dy = obs.second - robot_pose.y;
                float dist = std::sqrt(dx * dx + dy * dy);
                float obs_angle = std::atan2(dy, dx);

                float diff = std::fabs(std::atan2(std::sin(angle - obs_angle), std::cos(angle - obs_angle)));
                if (diff < 0.1 && dist < range)
                    range = dist;
            }

            LiDARLandmark landmark;
            landmark.range = range;
            landmark.angle = angle;
            landmark.quality = 1.0f;
            scan.landmarks.push_back(landmark);
        }

        // Update the Bayesian grid
        grid.update_map(scan, robot_pose);

        // Render grid
        window.clear(sf::Color::Black);
        const uint8_t* data = grid.get_map_data();

        sf::Image img;
        img.create(grid_size_x, grid_size_y);

        for (int y = 0; y < grid_size_y; ++y) {
            for (int x = 0; x < grid_size_x; ++x) {
                uint8_t value = data[y * grid_size_x + x];
                img.setPixel(x, y, sf::Color(value, value, value));
            }
        }

        sf::Texture tex;
        tex.loadFromImage(img);
        sf::Sprite sprite(tex);
        sprite.setScale(cell_size, cell_size);
        window.draw(sprite);

        // Draw robot
        sf::CircleShape robot(3);
        robot.setFillColor(sf::Color::Cyan);
        robot.setPosition(grid_size_x * cell_size / 2, grid_size_y * cell_size / 2);
        window.draw(robot);

        window.display();

        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Slow update
        iteration++;
    }

    return 0;
}
