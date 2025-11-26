/**
 * @file test_lidar_standard.cpp
 * @brief Runs RPLIDAR S2 in STANDARD mode, downsamples points, and publishes compact JSON over MQTT.
 *
 * Output topic: slamaleykoum77/lidar ; slamaleykoum77/print
 */

#include "test_common_esp1.h"

static const char* MQTT_TOPIC_LIDAR_STANDARD = "slamaleykoum77/lidar";
static const char* MQTT_TOPIC_LIDAR_STANDARD_DEBUG = "slamaleykoum77/print1";

static const uint32_t PUBLISH_PERIOD_MS= 1000;
static const uint16_t DOWNSAMPLE_N     = 8;
static const uint16_t MAX_POINTS_OUT   = 400;

uint32_t last_pub_ms_st = 0;

static void append_point(char* out, size_t out_sz, double angle_deg, uint16_t dist_mm, uint8_t quality) {
    char tmp[64];
    snprintf(tmp, sizeof(tmp), "{\"angle\":%.2f,\"distance\":%u,\"quality\":%u}", angle_deg, (unsigned)dist_mm, (unsigned)quality);
    strlcat(out, tmp, out_sz);
}

void setup_test_lidar_standard() {
    Serial.begin(115200);

    connection.setupWifi();
    connection.check_connection();

    connection.client.setBufferSize(16384);

    bool ret = lidar->start(standard);
    delay(1000);
    if (ret) {
        char msgdbg[60];
        snprintf(msgdbg, sizeof(msgdbg), "🟢 Rplidar C1 started correctly!\r\n");
        connection.publish(MQTT_TOPIC_LIDAR_STANDARD_DEBUG, msgdbg);
    } else {
        char msgdbg[60];
        snprintf(msgdbg, sizeof(msgdbg), "🔴 Error starting Rplidar C1\r\n");
        connection.publish(MQTT_TOPIC_LIDAR_STANDARD_DEBUG, msgdbg);
    }

    lidar->setAngleOfInterest(LIDAR_ANGLE_OF_INTEREST_START, LIDAR_ANGLE_OF_INTEREST_END);

    Serial.println("[LiDAR] STANDARD mode started.");
}

void loop_test_lidar_standard() {
    connection.check_connection();

    static uint32_t lastPub = millis();
    static char msg[8000];
    static int pointCount = 0;

    // initialize JSON
    if (pointCount == 0) strcpy(msg, "[");

    int count = lidar->readMeasurePoints();

    if (count == 0) {
        // try to recover if lidar stopped sending
        lidar->resetDevice();
        bool ret = lidar->start(standard);
        if (ret) Serial.println("🟢 Rplidar restarted correctly");
        delay(100);
        return;
    }

    int valid = 0;
    count = min(count, MAX_LIDAR_POINTS);

    for (int i = 0; i < count; i++) {
        float angle = (lidar->DataBuffer[i].angle_high * 128 + lidar->DataBuffer[i].angle_low / 2) / 64.0f;
        float distance = (lidar->DataBuffer[i].distance_high * 256 + lidar->DataBuffer[i].distance_low) / 4.0f;

        if (distance <= 0 || distance > MAX_RANGE) continue;

        char tmp[64];
        snprintf(tmp, sizeof(tmp), "{\"angle\":%.1f,\"distance\":%.1f},", angle, distance);
        strlcat(msg, tmp, sizeof(msg));

        lidar->Data[valid].angle = angle;
        lidar->Data[valid].distance = distance;
        valid++;
        pointCount++;
    }

    uint32_t now = millis();
    if (now - lastPub >= PUBLISH_PERIOD_MS) {
        if (pointCount > 0) {
            if (msg[strlen(msg) - 1] == ',')
                msg[strlen(msg) - 1] = ']';
            else
                strlcat(msg, "]", sizeof(msg));

            connection.publish(MQTT_TOPIC_LIDAR_STANDARD, msg);

            pointCount = 0;
            lastPub = now;
        }
    }
}
