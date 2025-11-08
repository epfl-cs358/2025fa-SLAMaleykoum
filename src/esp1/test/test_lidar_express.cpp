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
static const char* MQTT_TOPIC_LIDAR_debug = "slamaleykoum77/print";

static const uint16_t LEFT_DEG         = 0;
static const uint16_t RIGHT_DEG        = 360;
static const uint32_t PUBLISH_PERIOD_MS= 200;
static const uint16_t DOWNSAMPLE_N     = 8;
static const uint16_t MAX_POINTS_OUT   = 180;
static const uint16_t SCAN_WINDOW      = 1200;

// 👉 Move big arrays to static/global (outside stack)
static char msg[4096];
static char debugmsg[128];

uint32_t last_pub_ms = 0;

static void append_point(char* out, size_t out_sz, double angle_deg, uint16_t dist_mm, uint8_t quality) {
    char tmp[64];
    snprintf(tmp, sizeof(tmp), "{\"angle\":%.2f,\"distance\":%u,\"quality\":%u}", angle_deg, (unsigned)dist_mm, (unsigned)quality);
    strlcat(out, tmp, out_sz);
}

void setup_test_lidar_express() {
    Serial.begin(115200);
    delay(500);

    connection.setupWifi();

    //Serial2.setRxBufferSize(5000);
    lidar->resetDevice();

    stDeviceStatus_t sdst = lidar->getDeviceHealth();
    printf("sdst.errorCode_high=%d  sdst.errorCode_low=%d sdst.status=%d\r\n",
           sdst.errorCode_high, sdst.errorCode_low, sdst.status);

    lidar->setAngleOfInterest(LEFT_DEG, RIGHT_DEG);
    connection.check_connection();

    bool ret = lidar->start(express);
    if (ret) {
        connection.publish(MQTT_TOPIC_LIDAR_debug, "🟢 Rplidar C1 started correctly!\r\n");
    } else {
        connection.publish(MQTT_TOPIC_LIDAR_debug, "🔴 Error starting Rplidar C1\r\n");
    }

    // Retry loop
    while (!lidar->start(express)) {
        connection.publish(MQTT_TOPIC_LIDAR_debug, "⚠️ Retrying LIDAR start...");
        lidar->resetDevice();
        delay(500);
    }

    Serial.println("[LiDAR] EXPRESS mode started.");
}

void loop_test_lidar_express() {
    connection.check_connection();

    uint16_t blocks = lidar->readMeasurePoints();
    (void)blocks;

    uint32_t now = millis();
    if (now - last_pub_ms < PUBLISH_PERIOD_MS) return;
    last_pub_ms = now;

    // Build JSON (start)
    strlcpy(msg, "{\"type\":\"lidar\",\"mode\":\"express\",\"points\":[", sizeof(msg));

    uint16_t kept = 0;
    uint16_t stride = (DOWNSAMPLE_N == 0 ? 1 : DOWNSAMPLE_N);

    for (uint16_t i = 0, step = 0; i < SCAN_WINDOW && kept < MAX_POINTS_OUT; ++i, ++step) {
        double angle = lidar->Data[i].angle;
        uint16_t dist = lidar->Data[i].distance;

        // Avoid publishing debug per point — too heavy!
        //snprintf(debugmsg, sizeof(debugmsg), "angle: %.1f ; dist: %u", angle, dist);
        //connection.publish(MQTT_TOPIC_LIDAR_debug, debugmsg);

        if (dist == 0) continue;
        if (step % stride != 0) continue;

        if (kept > 0) strlcat(msg, ",", sizeof(msg));
        append_point(msg, sizeof(msg), angle, dist, 0);
        kept++;
    }

    strlcat(msg, "]}", sizeof(msg));

    // Publish final payload
    connection.publish(MQTT_TOPIC_LIDAR, msg);

    Serial.printf("[LiDAR] published %d points\n", kept);
}
