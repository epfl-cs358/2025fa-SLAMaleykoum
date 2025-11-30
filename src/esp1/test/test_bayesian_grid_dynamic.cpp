#include <SFML/Graphics.hpp>
#include <cmath>
#include <vector>
#include <iostream>
#include <thread>
#include <chrono>

#include "../../../include/common/data_types.h"
#include "../../../include/esp1/mapping/occupancy/bayesian_grid.h"

struct BoxObstacle {
    sf::Vector2f corners[4];
    BoxObstacle(std::pair<float,float> p1,
                std::pair<float,float> p2,
                std::pair<float,float> p3,
                std::pair<float,float> p4)
    {
        corners[0] = sf::Vector2f{p1.first, p1.second};
        corners[1] = sf::Vector2f{p2.first, p2.second};
        corners[2] = sf::Vector2f{p3.first, p3.second};
        corners[3] = sf::Vector2f{p4.first, p4.second};
    }
};

bool lineIntersectsSegment(
    const sf::Vector2f& p0, const sf::Vector2f& p1,
    const sf::Vector2f& s0, const sf::Vector2f& s1,
    sf::Vector2f& intersection)
{
    sf::Vector2f r = p1 - p0;
    sf::Vector2f s = s1 - s0;

    float rxs = r.x * s.y - r.y * s.x;
    float qpxr = (s0.x - p0.x)*r.y - (s0.y - p0.y)*r.x;

    if (std::abs(rxs) < 1e-8) return false;

    float t = ((s0.x - p0.x)*s.y - (s0.y - p0.y)*s.x) / rxs;
    float u = qpxr / rxs;

    if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
    {
        intersection = sf::Vector2f(p0.x + t*r.x, p0.y + t*r.y);
        return true;
    }
    return false;
}

// Retourne distance hit ou -1
float raycastQuad(const Pose2D& robot, float angle_world_rad,
                  float maxRange, const BoxObstacle& box)
{
    sf::Vector2f p0(robot.x, robot.y);
    sf::Vector2f p1(robot.x + maxRange * std::cos(angle_world_rad),
                    robot.y + maxRange * std::sin(angle_world_rad));

    float bestDist = maxRange;
    bool found = false;

    for (int i = 0; i < 4; i++)
    {
        sf::Vector2f a = box.corners[i];
        sf::Vector2f b = box.corners[(i+1)%4];

        sf::Vector2f hit;
        if (lineIntersectsSegment(p0, p1, a, b, hit))
        {
            float d = std::hypot(hit.x - robot.x, hit.y - robot.y);
            if (d < bestDist) { bestDist = d; found = true; }
        }
    }

    return found ? bestDist : -1;
}

int main() {
    const int grid_size_x = 150;
    const int grid_size_y = 150;
    const float resolution = 0.05f;

    BayesianOccupancyGrid grid(resolution, grid_size_x, grid_size_y);
    const int cell_size = 5;

    sf::RenderWindow window(
        sf::VideoMode(sf::Vector2u(grid_size_x * cell_size, grid_size_y * cell_size)),
        "Bayesian Grid Test (Adapted)"
    );

    Pose2D robot_pose{0.0f, 0.0f, 0.0f};

    std::vector<std::pair<float,float>> point_obstacles = {
        {1.0f, 1.0f},
        {0.0f, -2.0f},
        {-1.5f, -0.5f},
        {2.0f, 2.0f}
    };

    std::vector<BoxObstacle> boxes = {
        BoxObstacle({1.5f, 0.5f}, {2.5f, 0.5f}, {2.5f, 1.5f}, {1.5f, 1.5f}),
        BoxObstacle({-1.0f, -1.0f}, {-0.5f, -1.0f}, {-0.5f, -0.5f}, {-1.0f, -0.5f})
    };

    const int lidar_points = 20000;      // 0.018° resolution
    const float lidar_max_range = 3.0f;

    int iteration = 0;

    while (window.isOpen()) {

        // ---- CONTROLES ----
        bool up=false, down=false, left=false, right=false;
        const float v = 0.1f;
        const float rot = 0.1f;

        while (auto ev = window.pollEvent()) {
            if (ev->is<sf::Event::Closed>()) window.close();

            if (auto* k = ev->getIf<sf::Event::KeyPressed>()) {
                if (k->scancode == sf::Keyboard::Scancode::Up) up = true;
                if (k->scancode == sf::Keyboard::Scancode::Down) down = true;
                if (k->scancode == sf::Keyboard::Scancode::Left) left = true;
                if (k->scancode == sf::Keyboard::Scancode::Right) right = true;
            }
        }

        if (up) {
            robot_pose.x += v * sin(robot_pose.theta);
            robot_pose.y += v * cos(robot_pose.theta);
        }
        if (down) {
            robot_pose.x -= v * sin(robot_pose.theta);
            robot_pose.y -= v * cos(robot_pose.theta);
        }
        if (left) robot_pose.theta -= rot;
        if (right) robot_pose.theta += rot;

        // ---- LIDAR SIMU ----
        LiDARScan scan;
        scan.count = lidar_points;
        scan.timestamp_ms = iteration * 100;

        for (int i = 0; i < lidar_points; i++) {

            float angle_lidar_deg = (i * 360.0f) / lidar_points;
            float angle_world_rad = robot_pose.theta + angle_lidar_deg * (M_PI/180.0f);

            float best = -1;

            // Points
            for (auto& obs : point_obstacles) {
                float dx = obs.first - robot_pose.x;
                float dy = obs.second - robot_pose.y;
                float d = std::hypot(dx, dy);
                float obsA = std::atan2(dy, dx);

                float diff = std::fabs(std::atan2(
                    std::sin(angle_world_rad - obsA),
                    std::cos(angle_world_rad - obsA)
                ));

                if (diff < 0.05 && d < lidar_max_range) {
                    if (best < 0 || d < best) best = d;
                }
            }

            // Boxes
            for (auto& b : boxes) {
                float d = raycastQuad(robot_pose, angle_world_rad, lidar_max_range, b);
                if (d > 0 && (best < 0 || d < best)) best = d;
            }

            scan.angles[i]    = angle_lidar_deg;        // degrés
            scan.distances[i] = best > 0 ? best*1000.0f // → mm
                                         : 0;
            scan.qualities[i] = 255;
        }

        // ---- UPDATE MAP ----

        SyncedScan synced;
        synced.pose = robot_pose; 
        synced.scan = scan; 
        grid.update_map(synced, lidar_max_range);

        // ---- DRAW GRID ----
        window.clear();
        const uint8_t* data = grid.get_map_data_color();

        sf::Image img;
        img.resize({grid_size_x, grid_size_y});

        for (int y = 0; y < grid_size_y; y++)
            for (int x = 0; x < grid_size_x; x++)
                img.setPixel({(unsigned)x,(unsigned)y},
                             sf::Color(data[y*grid_size_x+x],
                                       data[y*grid_size_x+x],
                                       data[y*grid_size_x+x]));

        sf::Texture tex;
        tex.loadFromImage(img);
        sf::Sprite sprite(tex);
        sprite.setScale(sf::Vector2f(cell_size, cell_size));

        window.draw(sprite);

        // ---- DRAW ROBOT ----
        float gx = robot_pose.x / resolution + grid_size_x/2.0f;
        float gy = -robot_pose.y / resolution + grid_size_y/2.0f;

        sf::CircleShape rob(3);
        rob.setFillColor(sf::Color::Red);
        rob.setPosition(sf::Vector2f(gx * cell_size, gy * cell_size));
        window.draw(rob);

        window.display();

        iteration++;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
