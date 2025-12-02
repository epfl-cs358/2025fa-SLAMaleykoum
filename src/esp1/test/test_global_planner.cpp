#include <SFML/Graphics.hpp>
#include <cmath>
#include <vector>
#include <thread>
#include <chrono>

#include "../../../include/common/data_types.h"
#include "../../../include/esp1/mapping/occupancy/bayesian_grid.h"
#include "../../../include/esp1/planning/global_planner.h"

// ---------------------------------------------------------
//                RECTANGLE OBSTACLE
// ---------------------------------------------------------

struct BoxObstacle {
    sf::Vector2f c[4];
    BoxObstacle(std::pair<float,float> p1,
                std::pair<float,float> p2,
                std::pair<float,float> p3,
                std::pair<float,float> p4)
    {
        c[0] = {p1.first, p1.second};
        c[1] = {p2.first, p2.second};
        c[2] = {p3.first, p3.second};
        c[3] = {p4.first, p4.second};
    }
};

bool lineIntersectsSegment(const sf::Vector2f& p0, const sf::Vector2f& p1,
                           const sf::Vector2f& s0, const sf::Vector2f& s1,
                           sf::Vector2f& out)
{
    sf::Vector2f r = p1 - p0;
    sf::Vector2f s = s1 - s0;

    float rxs  = r.x * s.y - r.y * s.x;
    float qpxr = (s0.x - p0.x) * r.y - (s0.y - p0.y) * r.x;

    if (std::abs(rxs) < 1e-9f) return false;

    float t = ((s0.x - p0.x)*s.y - (s0.y - p0.y)*s.x) / rxs;
    float u = qpxr / rxs;

    if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
    {
        out = { p0.x + t*r.x, p0.y + t*r.y };
        return true;
    }
    return false;
}

float raycastQuad(const Pose2D& robot, float angle_world,
                  float maxRange, const BoxObstacle& b)
{
    sf::Vector2f p0(robot.x, robot.y);
    sf::Vector2f p1(robot.x + maxRange * std::cos(angle_world),
                    robot.y + maxRange * std::sin(angle_world));

    float best = maxRange;
    bool hit   = false;

    for (int i = 0; i < 4; i++)
    {
        sf::Vector2f a = b.c[i];
        sf::Vector2f c = b.c[(i+1)%4];

        sf::Vector2f out;
        if (lineIntersectsSegment(p0, p1, a, c, out))
        {
            float d = std::hypot(out.x - robot.x, out.y - robot.y);
            if (d < best) { best = d; hit = true; }
        }
    }
    return hit ? best : -1.f;
}

// ---------------------------------------------------------
//               FOLLOW WAYPOINTS
// ---------------------------------------------------------

void follow_waypoints(Pose2D& robot, GlobalPathMessage& path)
{
    if (path.current_length == 0) return;

    float tx = path.path[0].x;
    float ty = path.path[0].y;

    float dx = tx - robot.x;
    float dy = ty - robot.y;
    float dist = std::hypot(dx, dy);

    if (dist < 0.08f) {
        for (int i = 1; i < path.current_length; i++)
            path.path[i-1] = path.path[i];
        path.current_length--;
        return;
    }

    float tgt_theta = std::atan2(dy, dx);

    float dtheta = std::atan2(
        std::sin(tgt_theta - robot.theta),
        std::cos(tgt_theta - robot.theta)
    );

    robot.theta += dtheta * 0.18f;

    const float speed = 0.03f;
    robot.x += speed * std::cos(robot.theta);
    robot.y += speed * std::sin(robot.theta);
}

// ---------------------------------------------------------
//                        MAIN
// ---------------------------------------------------------

int main()
{
    const int grid_x = 200;
    const int grid_y = 200;
    const float res  = 0.05f;
    const int cell   = 4;

    BayesianOccupancyGrid grid(res, grid_x, grid_y);
    GlobalPlanner planner;

    sf::RenderWindow window(
        sf::VideoMode(sf::Vector2u(grid_x*cell, grid_y*cell)),
        "GLOBAL PLANNER + DYNAMIC GRID UPDATE"
    );

    Pose2D robot{0.f, 0.f, 0.f};

    MissionGoal goal;
    goal.target_pose.x = -4.0f;
    goal.target_pose.y = -2.0f;
    goal.type = NAVIGATION_TO_WAYPOINT;

    GlobalPathMessage path{};
    path.current_length = 0;

    std::vector<std::pair<float,float>> pts = {
        {1.0f, 1.0f},
        {0.0f, -2.0f},
        {-1.5f, -0.5f},
        {2.0f, 2.0f}
    };

    std::vector<BoxObstacle> boxes = {
        BoxObstacle({1.5f,0.5f},{2.5f,0.5f},{2.5f,1.5f},{1.5f,1.5f}),
        BoxObstacle({-1.f,-1.f},{-0.5f,-1.f},{-0.5f,-0.5f},{-1.f,-0.5f})
    };

    const int   lidar_pts   = 20000;
    const float lidar_range = 4.0f;

    int iter = 0;

    while (window.isOpen())
    {
        while (auto ev = window.pollEvent())
            if (ev->is<sf::Event::Closed>()) window.close();

        // ---------------- LIDAR ----------------
        LiDARScan scan;
        scan.count        = lidar_pts;
        scan.timestamp_ms = iter * 100;

        for (int i = 0; i < lidar_pts; i++)
        {
            float ang_deg = (i * 360.f) / lidar_pts;
            float ang_w   = robot.theta + ang_deg * (M_PI/180.f);

            float best = -1.f;

            // Point obstacles
            for (auto &p : pts)
            {
                float dx = p.first  - robot.x;
                float dy = p.second - robot.y;
                float d  = std::hypot(dx,dy);
                float a  = std::atan2(dy,dx);

                float diff = std::fabs(std::atan2(
                    std::sin(ang_w - a),
                    std::cos(ang_w - a)
                ));

                if (diff < 0.05f && d < lidar_range)
                    if (best < 0 || d < best) best = d;
            }

            // Box obstacles
            for (auto &b : boxes) {
                float d = raycastQuad(robot, ang_w, lidar_range, b);
                if (d > 0 && (best < 0 || d < best)) best = d;
            }

            scan.angles[i]    = ang_deg;
            scan.distances[i] = (best > 0 ? best*1000.f : 0.f);
            scan.qualities[i] = 255;
        }

        // -------------- MAP UPDATE --------------
        SyncedScan ss;
        ss.pose = robot;
        ss.scan = scan;
        grid.update_map(ss, lidar_range);

        // -------------- A* (replanned every frame) --------------
        path = planner.generate_path(robot, goal, grid);

        // -------------- Robot movement --------------
        follow_waypoints(robot, path);

        // -------------- DRAW MAP --------------
        window.clear(sf::Color::Black);

        const uint8_t* dat = grid.get_map_data_color();

        sf::Image img;
        img.resize({grid_x, grid_y});

        for (int y = 0; y < grid_y; y++)
            for (int x = 0; x < grid_x; x++)
            {
                uint8_t v = dat[y*grid_x + x];
                img.setPixel({(unsigned)x, (unsigned)y}, sf::Color(v,v,v));
            }

        sf::Texture tex;
        tex.loadFromImage(img);

        sf::Sprite sprite(tex);
        sprite.setScale(sf::Vector2f(cell, cell));
        window.draw(sprite);

        // -------------- DRAW PATH --------------
        for (int i = 0; i < path.current_length - 1; i++)
        {
            sf::Vertex L[2];
            float x0 = path.path[i].x;
            float y0 = path.path[i].y;
            float x1 = path.path[i+1].x;
            float y1 = path.path[i+1].y;

            L[0].position = {
                (x0/res + grid_x/2.f)*cell,
                (-y0/res + grid_y/2.f)*cell
            };
            L[1].position = {
                (x1/res + grid_x/2.f)*cell,
                (-y1/res + grid_y/2.f)*cell
            };
            L[0].color = L[1].color = sf::Color::Green;

            window.draw(L, 2, sf::PrimitiveType::Lines);
        }

        // -------------- DRAW ROBOT --------------
        float gx = robot.x / res + grid_x/2.f;
        float gy = -robot.y / res + grid_y/2.f;

        sf::CircleShape bot(4);
        bot.setOrigin({4,4});
        bot.setFillColor(sf::Color::Red);
        bot.setPosition({gx*cell, gy*cell});
        window.draw(bot);

        window.display();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        iter++;
    }

    return 0;
}
