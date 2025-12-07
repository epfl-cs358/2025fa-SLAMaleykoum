/*
 * ESP32-S3 PATH PLANNER - BINARY DATA PROTOCOL
 * 
 * Sends compact binary data to Python visualizer (no ASCII lag!)
 * Protocol: Header (32 bytes) + Grid (NxM int8) + Path (N*8 bytes)
 * 
 * MUCH FASTER than ASCII and provides perfect data for plotting
 */

#include <Arduino.h>
#include <WiFi.h>

#include "../../../include/esp1/mapping/occupancy/bayesian_grid.h"
#include "../../../include/esp1/planning/global_planner.h"

// ------------ SETTINGS ------------
#define RES       0.05f
#define SX        100
#define SY        100
#define TCP_PORT  9000

// ------------ GLOBALS ------------
static WiFiServer tcpServer(TCP_PORT);
static WiFiClient tcpClient;

constexpr const char* WIFI_SSID = "LIDAR_AP";
constexpr const char* WIFI_PASSWORD = "l1darpass";

static BayesianOccupancyGrid* grid_ptr = nullptr;
static GlobalPlanner planner;

// Test scenario index
static int current_test = 0;
static bool auto_send_all = false;

// ------------------------------------------------------
// WORLD <-> GRID CONVERSION
// ------------------------------------------------------
static inline void world_to_grid(float wx, float wy, int& gx, int& gy)
{
    gx = int(wx / RES + SX / 2.0f);
    gy = int(-wy / RES + SY / 2.0f);
    gx = constrain(gx, 0, SX - 1);
    gy = constrain(gy, 0, SY - 1);
}

// ------------------------------------------------------
// OBSTACLE BUILDERS
// ------------------------------------------------------
void clear_obstacles(BayesianOccupancyGrid& grid)
{
    int8_t* log_odds = (int8_t*)grid.get_map_data();
    memset(log_odds, 0, SX * SY);
}

void set_obstacle_rect(BayesianOccupancyGrid& grid,
                       float x_min, float x_max,
                       float y_min, float y_max)
{
    int8_t* log_odds = (int8_t*)grid.get_map_data();
    
    for (float wx = x_min; wx <= x_max; wx += RES) {
        for (float wy = y_min; wy <= y_max; wy += RES) {
            int gx, gy;
            world_to_grid(wx, wy, gx, gy);
            if (gx >= 0 && gx < SX && gy >= 0 && gy < SY) {
                log_odds[gy * SX + gx] = 40;
            }
        }
    }
}

void set_obstacle_circle(BayesianOccupancyGrid& grid,
                        float cx, float cy, float radius)
{
    int8_t* log_odds = (int8_t*)grid.get_map_data();
    
    for (float wx = cx - radius; wx <= cx + radius; wx += RES) {
        for (float wy = cy - radius; wy <= cy + radius; wy += RES) {
            float dist = sqrt((wx - cx) * (wx - cx) + (wy - cy) * (wy - cy));
            if (dist <= radius) {
                int gx, gy;
                world_to_grid(wx, wy, gx, gy);
                if (gx >= 0 && gx < SX && gy >= 0 && gy < SY) {
                    log_odds[gy * SX + gx] = 40;
                }
            }
        }
    }
}

// ------------------------------------------------------
// TEST SCENARIOS
// ------------------------------------------------------
struct TestCase {
    const char* name;
    Pose2D start;
    Pose2D goal;
    void (*build_obstacles)(BayesianOccupancyGrid&);
};

void build_test1(BayesianOccupancyGrid& grid) {
    clear_obstacles(grid);
    set_obstacle_rect(grid, -1.5f, 1.5f, 0.5f, 1.0f);
}

void build_test2(BayesianOccupancyGrid& grid) {
    clear_obstacles(grid);
    set_obstacle_rect(grid, -1.0f, -0.8f, -1.0f, 1.0f);
    set_obstacle_rect(grid, 0.8f, 1.0f, -1.0f, 1.0f);
    set_obstacle_rect(grid, -1.0f, 1.0f, 0.8f, 1.0f);
}

void build_test3(BayesianOccupancyGrid& grid) {
    clear_obstacles(grid);
    set_obstacle_circle(grid, 0.5f, 0.5f, 0.3f);
    set_obstacle_circle(grid, -0.5f, 1.0f, 0.25f);
    set_obstacle_circle(grid, 0.0f, 1.5f, 0.2f);
}

void build_test4(BayesianOccupancyGrid& grid) {
    clear_obstacles(grid);
    set_obstacle_rect(grid, -1.0f, 0.0f, 0.3f, 0.5f);
    set_obstacle_rect(grid, 0.3f, 1.0f, 0.8f, 1.0f);
    set_obstacle_rect(grid, -0.5f, -0.3f, 1.2f, 1.8f);
}

void build_test5(BayesianOccupancyGrid& grid) {
    clear_obstacles(grid);
    set_obstacle_rect(grid, -2.0f, -0.15f, 0.5f, 1.5f);
    set_obstacle_rect(grid, 0.15f, 2.0f, 0.5f, 1.5f);
}

TestCase test_cases[] = {
    {"Horizontal Wall", {0.0f, 0.0f, M_PI/2, 0}, {-0.3f, 1.5f, 0, 0}, build_test1},
    {"U-Shaped Corridor", {0.0f, -0.5f, M_PI/2, 0}, {0.0f, 1.5f, 0, 0}, build_test2},
    {"Scattered Obstacles", {0.0f, 0.0f, M_PI/2, 0}, {0.0f, 2.0f, 0, 0}, build_test3},
    {"Maze Navigation", {0.0f, 0.0f, M_PI/2, 0}, {0.5f, 2.0f, 0, 0}, build_test4},
    {"Narrow Passage", {0.0f, 0.0f, M_PI/2, 0}, {0.0f, 2.0f, 0, 0}, build_test5}
};

const int NUM_TESTS = sizeof(test_cases) / sizeof(TestCase);

// ------------------------------------------------------
// BINARY DATA PROTOCOL
// ------------------------------------------------------
void send_binary_data(WiFiClient& client,
                     uint8_t test_num,
                     BayesianOccupancyGrid& grid,
                     const GlobalPathMessage& path_msg,
                     const Pose2D& start,
                     const Pose2D& goal)
{
    if (!client || !client.connected()) {
        Serial.println("[TCP] Client not connected!");
        return;
    }
    
    // Build header (32 bytes)
    uint8_t header[32];
    memset(header, 0, 32);
    
    // Sync bytes
    header[0] = 0xAA;
    header[1] = 0xBB;
    
    // Test number
    header[2] = test_num;
    
    // Grid dimensions
    uint16_t grid_x = SX;
    uint16_t grid_y = SY;
    memcpy(&header[3], &grid_x, 2);
    memcpy(&header[5], &grid_y, 2);
    
    // Resolution
    float res = RES;
    memcpy(&header[7], &res, 4);
    
    // Start pose
    memcpy(&header[11], &start.x, 4);
    memcpy(&header[15], &start.y, 4);
    memcpy(&header[19], &start.theta, 4);
    
    // Goal pose
    memcpy(&header[23], &goal.x, 4);
    memcpy(&header[27], &goal.y, 4);
    
    // Path length
    uint8_t path_len = (uint8_t)path_msg.current_length;
    header[31] = path_len;
    
    // Send header
    size_t written = client.write(header, 32);
    if (written != 32) {
        Serial.printf("[TCP] Header send failed: %d/32 bytes\n", written);
        return;
    }
    
    // Send grid data (int8 array)
    const int8_t* grid_data = grid.get_map_data();
    size_t grid_size = SX * SY;
    
    written = client.write((const uint8_t*)grid_data, grid_size);
    if (written != grid_size) {
        Serial.printf("[TCP] Grid send failed: %d/%d bytes\n", written, grid_size);
        return;
    }
    
    // Send path waypoints (float x, float y pairs)
    for (int i = 0; i < path_msg.current_length; i++) {
        float wx = path_msg.path[i].x;
        float wy = path_msg.path[i].y;
        
        client.write((uint8_t*)&wx, 4);
        client.write((uint8_t*)&wy, 4);
    }
    
    Serial.printf("[TCP] Sent test %d: %d bytes (header + %d grid + %d path)\n",
                  test_num, 32 + grid_size + path_len * 8,
                  grid_size, path_len * 8);
    
    client.flush();
}

// ------------------------------------------------------
// RUN TEST CASE
// ------------------------------------------------------
void run_test(int test_idx)
{
    if (test_idx >= NUM_TESTS) {
        test_idx = 0;
    }
    
    current_test = test_idx;
    TestCase& test = test_cases[test_idx];
    
    Serial.printf("\n[TEST %d/%d] Running: %s\n", test_idx + 1, NUM_TESTS, test.name);
    
    // Build obstacles
    test.build_obstacles(*grid_ptr);
    
    // Setup mission goal
    MissionGoal goal;
    goal.type = MissionGoalType::NAVIGATION_TO_WAYPOINT;
    goal.target_pose = test.goal;
    
    // Run A*
    unsigned long start_time = millis();
    GlobalPathMessage msg = planner.generate_path(test.start, goal, *grid_ptr);
    unsigned long planning_time = millis() - start_time;
    
    // Print to serial
    Serial.printf("[RESULT] Path: %s (%d waypoints)\n", 
                  msg.current_length > 0 ? "FOUND" : "NOT FOUND",
                  msg.current_length);
    Serial.printf("[RESULT] Planning time: %lu ms\n", planning_time);
    
    // Send binary data
    if (tcpClient && tcpClient.connected()) {
        send_binary_data(tcpClient, test_idx, *grid_ptr, msg, test.start, test.goal);
    }
}

// ------------------------------------------------------
// SETUP
// ------------------------------------------------------
void setup_fake_occupancy_grid()
{
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n╔════════════════════════════════════════════════════════╗");
    Serial.println("║  ESP32-S3 PATH PLANNER - BINARY DATA MODE             ║");
    Serial.println("╚════════════════════════════════════════════════════════╝\n");
    
    // WiFi AP
    Serial.println("[WIFI] Starting Access Point...");
    WiFi.mode(WIFI_AP);
    
    if (!WiFi.softAP(WIFI_SSID, WIFI_PASSWORD)) {
        Serial.println("[ERROR] Failed to start AP!");
        while(1) delay(1000);
    }
    
    Serial.printf("[WIFI] ✓ SSID: %s\n", WIFI_SSID);
    Serial.printf("[WIFI] ✓ IP: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("[WIFI] ✓ Password: %s\n\n", WIFI_PASSWORD);
    
    // TCP Server
    tcpServer.begin();
    tcpServer.setNoDelay(true);
    Serial.printf("[TCP] ✓ Listening on port %d\n\n", TCP_PORT);
    
    // Allocate grid
    Serial.printf("[GRID] Initializing %dx%d @ %.3fm/cell\n", SX, SY, RES);
    grid_ptr = new BayesianOccupancyGrid(RES, SX, SY);
    
    if (!grid_ptr) {
        Serial.println("[ERROR] Failed to allocate grid!");
        while(1) delay(1000);
    }
    
    Serial.printf("[GRID] ✓ Allocated (~%d KB)\n", (SX * SY) / 1024);
    Serial.printf("[HEAP] Free: %u bytes\n\n", ESP.getFreeHeap());
    Serial.printf("[READY] %d test scenarios loaded\n", NUM_TESTS);
    Serial.println("[READY] Waiting for Python visualizer...\n");
}

// ------------------------------------------------------
// LOOP
// ------------------------------------------------------
void loop_fake_occupancy_grid()
{
    // Accept TCP connections
    if (!tcpClient || !tcpClient.connected()) {
        WiFiClient newClient = tcpServer.available();
        if (newClient) {
            tcpClient = newClient;
            tcpClient.setNoDelay(true);
            
            Serial.println("[TCP] ✓ Client connected!");
            Serial.println("[TCP] Ready to send test data\n");
            
            // Automatically send all tests
            auto_send_all = true;
        }
        delay(100);
        return;
    }
    
    // Auto-send all tests
    if (auto_send_all) {
        for (int i = 0; i < NUM_TESTS; i++) {
            run_test(i);
            delay(500);  // Small delay between tests
        }
        
        Serial.println("\n[COMPLETE] All tests sent!");
        Serial.println("[INFO] Python visualizer will now plot all results\n");
        
        auto_send_all = false;
    }
    
    // Check for manual commands (optional)
    if (tcpClient.available()) {
        String input = tcpClient.readStringUntil('\n');
        input.trim();
        
        if (input.length() > 0) {
            int test_num = input.toInt();
            if (test_num >= 0 && test_num < NUM_TESTS) {
                run_test(test_num);
            }
        }
    }
    
    delay(10);
}