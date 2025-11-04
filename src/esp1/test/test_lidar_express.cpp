/**
 * @file test_lidar_express.cpp
 * @brief Runs RPLIDAR S2 in EXPRESS mode, downsamples points, and publishes compact JSON over MQTT.
 *
 * Output topic: slamaleykoum77/lidar
 *
 * Tunables:
 *  - FOV:    setAngleOfInterest(LEFT_DEG, RIGHT_DEG)
 *  - RATE:   PUBLISH_PERIOD_MS
 *  - DENSITY: keep every Nth point (DOWNSAMPLE_N)
 */

#include "test_common_esp1.h"


static const char* MQTT_TOPIC_LIDAR = "slamaleykoum77/lidar";

// ---- Tunables ----
static const uint16_t LEFT_DEG         = 0;     // inclusive
static const uint16_t RIGHT_DEG        = 360;   // inclusive
static const uint32_t PUBLISH_PERIOD_MS= 200;   // publish every 200 ms
static const uint16_t DOWNSAMPLE_N     = 8;     // keep every Nth point (bigger => fewer points)
static const uint16_t MAX_POINTS_OUT   = 180;   // hard cap to keep payload small

// Scratch
static uint32_t last_pub_ms = 0;

// Small helper: append one point as JSON into a char buffer
static void append_point(char* out, size_t out_sz, double angle_deg, uint16_t dist_mm, uint8_t quality) {
    char tmp[64];
    // Quality for express mode isn’t exposed; use 0 for now (or compute from cabin bits if you add it)
    snprintf(tmp, sizeof(tmp), "{\"angle\":%.2f,\"distance\":%u,\"quality\":%u}", angle_deg, (unsigned)dist_mm, (unsigned)quality);
    strlcat(out, tmp, out_sz);
}

void setup_test_lidar_express() {
    Serial.begin(115200);
    delay(1000);

    // --- WiFi/MQTT ---
    connection.setupWifi();

    // --- UART1 pins for LiDAR (set to your wiring) ---
    Serial1.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_UART_RX, LIDAR_UART_TX);
    delay(50);

    // rpLidar driver serial/buffer init
    lidar.begin(LIDAR_BAUD, 4096);

    // FOV restriction (smaller FOV => fewer points)
    lidar.setAngleOfInterest(LEFT_DEG, RIGHT_DEG);

    // Start EXPRESS mode
    if (!lidar.start(express)) {
        Serial.println("[LiDAR] Failed to start EXPRESS mode. Rebooting device...");
        lidar.resetDevice();
        delay(500);
        lidar.start(express);
    }

    Serial.println("[LiDAR] EXPRESS mode started.");
}

void loop_test_lidar_express() {
    // Keep MQTT session alive
    connection.check_connection();

    // Read whatever the device has; in EXPRESS this parses packets and
    // fills lidar.Data[] with (angle, distance) pairs sorted by angle.
    uint16_t blocks = lidar.readMeasurePoints();  // returns number of express packets (not points)
    (void)blocks; // not used directly

    // Publish at a controlled rate
    uint32_t now = millis();
    if (now - last_pub_ms < PUBLISH_PERIOD_MS) return;
    last_pub_ms = now;

    // Build a compact JSON payload with downsampled points
    // We traverse the driver’s Data[] and keep every Nth valid point in FOV.
    // NOTE: The driver doesn’t expose "how many points are filled".
    // We conservatively scan a window and stop when MAX_POINTS_OUT is reached.
    // Distance==0 is used as a sentinel for "not valid".
    static const uint16_t SCAN_WINDOW = 1200; // safe window to scan
    char msg[4096];
    strlcpy(msg, "{\"type\":\"lidar\",\"mode\":\"express\",\"points\":[", sizeof(msg));

    uint16_t kept = 0;
    uint16_t stride = (DOWNSAMPLE_N == 0 ? 1 : DOWNSAMPLE_N);

    // Walk through the Data[] buffer and pick every Nth valid point
    for (uint16_t i = 0, step = 0; i < SCAN_WINDOW && kept < MAX_POINTS_OUT; ++i, ++step) {
        double angle = lidar.Data[i].angle;
        uint16_t dist = lidar.Data[i].distance;

        // Skip zero/empty entries
        if (!lidar.isDataValid(dist)) continue;
        if (!lidar.isDataBetweenBorders(angle)) continue;

        // Downsample
        if (step % stride != 0) continue;

        if (kept > 0) strlcat(msg, ",", sizeof(msg));
        append_point(msg, sizeof(msg), angle, dist, /*quality*/0);
        kept++;
    }

    strlcat(msg, "]}", sizeof(msg));

    // Publish JSON
    connection.publish(MQTT_TOPIC_LIDAR, msg);

    // Optional: also log a brief summary
    Serial.print("[LiDAR] published points: ");
    Serial.println(kept);
}
