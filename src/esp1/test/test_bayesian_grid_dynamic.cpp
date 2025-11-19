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

bool pointInQuad(float px, float py, const BoxObstacle& box)
{
    float angleSum = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        sf::Vector2f A = box.corners[i];
        sf::Vector2f B = box.corners[(i + 1) % 4];

        sf::Vector2f v1(A.x - px, A.y - py);
        sf::Vector2f v2(B.x - px, B.y - py);

        float dot = v1.x * v2.x + v1.y * v2.y;
        float det = v1.x * v2.y - v1.y * v2.x;
        angleSum += std::atan2(det, dot);
    }

    return std::fabs(angleSum) > 3.14f;
}

bool lineIntersectsSegment(
    const sf::Vector2f& p0, const sf::Vector2f& p1,
    const sf::Vector2f& s0, const sf::Vector2f& s1,
    sf::Vector2f& intersection)
{
    sf::Vector2f r = p1 - p0;
    sf::Vector2f s = s1 - s0;

    float rxs = r.x * s.y - r.y * s.x;
    float qpxr = (s0.x - p0.x)*r.y - (s0.y - p0.y)*r.x;

    if (std::abs(rxs) < 1e-8) return false; // parallel

    float t = ((s0.x - p0.x)*s.y - (s0.y - p0.y)*s.x) / rxs;
    float u = qpxr / rxs;

    if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
    {
        intersection = sf::Vector2f(p0.x + t*r.x, p0.y + t*r.y);
        return true;
    }
    return false;
}

float checkHitQuad(const Pose2D& robot, float angle, float maxRange,
                   const BoxObstacle& box)
{
    sf::Vector2f p0(robot.x, robot.y);
    sf::Vector2f p1(robot.x + maxRange * std::cos(angle),
                    robot.y + maxRange * std::sin(angle));

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
            if (d < bestDist)
            {
                bestDist = d;
                found = true;
            }
        }
    }

    return found ? bestDist : -1.0f;
}

int main() {
    const int grid_size_x = 150;
    const int grid_size_y = 150;
    const float resolution = 0.05f;
    BayesianOccupancyGrid grid(resolution, grid_size_x, grid_size_y);

    const int cell_size = 5;

    sf::RenderWindow window(
        sf::VideoMode(sf::Vector2u(grid_size_x * cell_size, grid_size_y * cell_size)),
        "Bayesian Grid Test"
    );

    // ⛔ AVANT : robot_pose = {grid_size_x*resolution/2, grid_size_y*resolution/2}
    // 👍 MAINTENANT : robot au centre des coordonnées réelles
    Pose2D robot_pose{0.0f, 0.0f, 0.0f};

    // Obstacles (en mètres)
    std::vector<std::pair<float, float>> obstacles = {
        {1.0f, 1.0f},
        {0.0f, -2.0f},
        {-1.5, -0.5}, 
        {2.0, 2.0}
    };

    std::vector<BoxObstacle> boxes = {
        BoxObstacle({1.5f, 0.5f}, {2.5f, 0.5f}, {2.5f, 1.5f}, {1.5f, 1.5f}),
        BoxObstacle({-1.0f, -1.0f}, {-0.5f, -1.0f}, {-0.5f, -0.5f}, {-1.0f, -0.5f})
    };

    const int lidar_points = 200000;
    const float lidar_max_range = 1.f;
    int iteration = 0; 

    while (window.isOpen()) {

        
        // Events SFML
        bool key_up = false, key_down = false;
        bool key_left = false, key_right = false;

        const float speed = 0.3f;      // vitesse linéaire (m par frame)
        const float rot_speed = 0.3f;

        while (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>())
                window.close();

            if (auto* e = event->getIf<sf::Event::KeyPressed>()) {
                switch (e->scancode) {
                    case sf::Keyboard::Scancode::Up: key_up = true; break;
                    case sf::Keyboard::Scancode::Down: key_down = true; break;
                    case sf::Keyboard::Scancode::Left: key_left = true; break;
                    case sf::Keyboard::Scancode::Right: key_right = true; break;
                    default: break;
                }
            }
        }

        if (key_up) {
            robot_pose.x += speed * sin(robot_pose.theta);
            robot_pose.y += speed * cos(robot_pose.theta);
        }
        if (key_down) {
            robot_pose.x -= speed * sin(robot_pose.theta);
            robot_pose.y -= speed * cos(robot_pose.theta);
        }
        if (key_left) robot_pose.theta -= rot_speed;
        if (key_right) robot_pose.theta += rot_speed;


        // ----------- LIDAR SIMULATION -------------
        LiDARScan scan;
        scan.timestamp_ms = iteration * 100;

        for (int i = 0; i < lidar_points; ++i) {
            float angle = (i / (float)lidar_points) * 2 * M_PI;
            float range = 0;
            float angle_world = robot_pose.theta + angle;

            for (auto &obs : obstacles) {
                float dx = obs.first - robot_pose.x;
                float dy = obs.second - robot_pose.y;
                float dist = std::sqrt(dx * dx + dy * dy);
                float obs_angle = std::atan2(dy, dx);
                
                float diff = std::fabs(std::atan2(std::sin(angle_world - obs_angle),
                                                   std::cos(angle_world - obs_angle)));

                if (diff < 0.2 && dist < lidar_max_range){
                    if(range == 0 || dist < range){ // si plusieurs obstacles sur meme trajectoire 
                        range = dist;               // on prend le plus proche comme ferait un vrai lidar
                    }
                }
            }

            // BOX obstacles
            for (auto &box : boxes) {
                float d = checkHitQuad(robot_pose, angle_world, lidar_max_range, box);
                if (d > 0 && (range == 0 || d < range)) range = d;
            }


            LiDARLandmark landmark{range, angle, 1.0f};
            scan.landmarks.push_back(landmark);
        } 

        // ----------- UPDATE OCCUPANCY GRID -------------
        grid.update_map(scan, robot_pose, lidar_max_range);

        window.clear(sf::Color::Black);

        const uint8_t* data = grid.get_map_data_color();

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
    
        float gx = robot_pose.x / resolution + grid_size_x / 2.0f;
        float gy = -robot_pose.y / resolution + grid_size_y / 2.0f;

        sf::ConvexShape robot;
        robot.setPointCount(3); 
        robot.setFillColor(sf::Color::Red);
        float L = 10.0f;  // longueur
        float W = 6.0f;   // largeur
        robot.setPoint(0, sf::Vector2f(0, -L));     // pointe
        robot.setPoint(1, sf::Vector2f(W/2, L/2));  // coin droit
        robot.setPoint(2, sf::Vector2f(-W/2, L/2)); // coin gauche

        robot.setFillColor(sf::Color::Red);

        // Position réelle du robot
        robot.setPosition(sf::Vector2f(gx * cell_size, gy * cell_size));

        // Orientation : rad → degrés
        robot.setRotation(sf::degrees(robot_pose.theta * 180.0f / M_PI));

        window.draw(robot);

        window.display();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        iteration++;
    }

    return 0;
}

