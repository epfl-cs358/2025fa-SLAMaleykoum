/**
 * @file test_lidar_standard.cpp
 * @brief Runs RPLIDAR S2 in EXPRESS mode, downsamples points, and publishes compact JSON over MQTT.
 *
 * Output topic: slamaleykoum77/lidar ; slamaleykoum77/print
 */

#include "test_common_esp1.h"

static const char* MQTT_TOPIC_LIDAR_EXPRESS = "slamaleykoum77/lidar";
static const char* MQTT_TOPIC_LIDAR_EXPRESS_DEBUG = "slamaleykoum77/print1";

static const uint32_t PUBLISH_PERIOD_MS= 200;
static const uint16_t DOWNSAMPLE_N     = 8;
static const uint16_t MAX_POINTS_OUT   = 180;
static const uint16_t SCAN_WINDOW      = 1200;

// 👉 Move big arrays to static/global (outside stack)
static char msg[4096];

uint32_t last_pub_ms_st = 0;

static void append_point(char* out, size_t out_sz, double angle_deg, uint16_t dist_mm, uint8_t quality) {
    char tmp[64];
    snprintf(tmp, sizeof(tmp), "{\"angle\":%.2f,\"distance\":%u,\"quality\":%u}", angle_deg, (unsigned)dist_mm, (unsigned)quality);
    strlcat(out, tmp, out_sz);
}

void setup_test_lidar_standard() {
    Serial.begin(115200);

    uint8_t getInfoCmd[2] = {0xA5, 0x50};
    LIDAR_SERIAL.write(getInfoCmd, 2);
    LIDAR_SERIAL.flush();
    delay(200);

    connection.setupWifi();
    connection.check_connection();

    bool ret = lidar->start(standard);
    delay(1000);
    if (ret) {
        char msgdbg[60];
        snprintf(msgdbg, sizeof(msgdbg), "🟢 Rplidar C1 started correctly!\r\n");
        connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_DEBUG, msgdbg);
    } else {
        char msgdbg[60];
        snprintf(msgdbg, sizeof(msgdbg), "🔴 Error starting Rplidar C1\r\n");
        connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_DEBUG, msgdbg);
    }

    stDeviceStatus_t sdst = lidar->getDeviceHealth();
    printf("sdst.errorCode_high=%u  sdst.errorCode_low=%u sdst.status=%u\r\n",
            sdst.errorCode_high, sdst.errorCode_low, sdst.status);
    char msgdbg2[180];
    snprintf(msgdbg2, sizeof(msgdbg2), "sdst.errorCode_high=%u  sdst.errorCode_low=%u sdst.status=%u\r\n", 
            sdst.errorCode_high, sdst.errorCode_low, sdst.status);
    connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_DEBUG, msgdbg2);

    lidar->setAngleOfInterest(LIDAR_ANGLE_OF_INTEREST_START, LIDAR_ANGLE_OF_INTEREST_END);

    Serial.println("[LiDAR] STANDARD mode started.");
}

void loop_test_lidar_standard() {
    connection.check_connection();

    uint32_t now = millis();
    if (now - last_pub_ms_st < PUBLISH_PERIOD_MS) return;
    last_pub_ms_st = now;

    uint16_t count = lidar->readMeasurePoints();  // Each Express scan returns multiple cabins
    if (count == 0) return;

    // Build JSON (start)
    strlcpy(msg, "{\"type\":\"lidar\",\"mode\":\"express\",\"points\":[", sizeof(msg));

    uint16_t kept = 0;
    uint16_t stride = (DOWNSAMPLE_N == 0 ? 1 : DOWNSAMPLE_N);

    for (uint16_t i = 0, step = 0; i < SCAN_WINDOW && kept < MAX_POINTS_OUT; ++i, ++step) {
        // double angle = lidar->Data[i].angle;
        // float angle = ((lidar->Data[i].angle_high << 7) | (lidar->DataBuffer[i].angle_low >> 1)) / 64.0f;
        // uint16_t dist = lidar->Data[i].distance;
        // uint16_t dist = (lidar->Data[i].distance_high * 256 + lidar->DataBuffer[i].distance_low) / 4.0f;
        float angle = ((lidar->DataBuffer[i].angle_high << 7) | (lidar->DataBuffer[i].angle_low >> 1)) / 64.0f;
        uint16_t dist = (lidar->DataBuffer[i].distance_high * 256 + lidar->DataBuffer[i].distance_low) / 4.0f;

        // if (angle >= 360.0f) angle = fmod(angle, 360.0f);

        // Avoid publishing debug per point — too heavy!
        char msgdbg[60];
        snprintf(msgdbg, sizeof(msgdbg), "angle: %.1f ; dist: %u", angle, dist);
        connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_DEBUG, msgdbg);

        if (dist == 0) continue;
        if (step % stride != 0) continue;

        if (kept > 0) strlcat(msg, ",", sizeof(msg));
        append_point(msg, sizeof(msg), angle, dist, 0);
        kept++;
    }

    strlcat(msg, "]}", sizeof(msg));

    // Publish final payload
    connection.publish(MQTT_TOPIC_LIDAR_EXPRESS, msg);

    Serial.printf("[LiDAR] published %d points\n", kept);
}
