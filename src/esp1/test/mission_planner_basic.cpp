#include "test_common_esp1.h"

static BayesianOccupancyGrid Map_(0.05f, 150, 150);

Pose2D pose = {0.0f, 0.0f, 0.0f, 0};
static MissionGoal goal = {pose, MissionGoalType::EXPLORATION_NODE};
static GoalManager goal_manager(pose);

int p = 0;

void setup_mission_planner_basic() {
    lidar.start();
    delay(1000);

    // Start WiFi in AP mode
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    delay(3000);
    
    // Start TCP server
    tcpServer.begin();
    tcpServer.setNoDelay(true);
    delay(1000);

    goal_manager.set_mission_state(GoalManager::STATE_EXPLORING);
}

void loop_mission_planner_basic() {
    // Accept new client if none
    if (!tcpClient || !tcpClient.connected()) {
        WiFiClient newClient = tcpServer.available();
        if (newClient) {
            tcpClient = newClient;
            tcpClient.setNoDelay(true);
        }
    }

    float lastAngleESP = 0.0f;
    lidar.build_scan(&scan, scanComplete, lastAngleESP);

    if ((scanComplete || millis() - lastSendTime > 100) && scan.count > 0) {
        scanComplete = false;

        // Update Bayesian map
        Map_.update_map(scan,
                           {0.0f, 0.0f, 0.0f, millis()},
                           8.0f);

        // Send goal info over TCP
        if (tcpClient && tcpClient.connected()) {
            
            const uint8_t* map = Map_.get_map_data_color();
            uint16_t w = Map_.grid_size_x;
            uint16_t h = Map_.grid_size_y;
            uint32_t map_size = w * h;

            uint8_t header[20];

            // Sync bytes
            header[0] = 0xAB;
            header[1] = 0xCD;

            // Grid size
            header[2] = w >> 8; header[3] = w & 0xFF;
            header[4] = h >> 8; header[5] = h & 0xFF;

            // Pose
            memcpy(&header[6],  &pose.x,     4);
            memcpy(&header[10], &pose.y,     4);
            memcpy(&header[14], &pose.theta, 4);

            // Map size
            header[18] = (map_size >> 8) & 0xFF;
            header[19] = (map_size & 0xFF);

            // Send header + map
            tcpClient.write(header, 20);
            tcpClient.write(map, map_size);

            // Send current goal info
            char buffer[64];
            int len = snprintf(buffer, sizeof(buffer), "Goal: x=%.2f, y=%.2f, theta=%.2f, type=%d\n",
                            goal.target_pose.x, goal.target_pose.y,
                            goal.target_pose.theta, static_cast<int>(goal.type));
            tcpClient.write((const uint8_t*)buffer, len);
        }

        lastSendTime = millis();
    }

    if (p == 100) {
        Pose2D current_pose = {0.0f, 0.0f, 0.0f, millis()};
        goal = goal_manager.update_goal(current_pose, Map_);
        p = 0;
    }

    p++;
}