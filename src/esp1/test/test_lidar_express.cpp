/**
 * @file test_lidar_express.cpp
 * @brief Runs RPLIDAR S2 in EXPRESS mode, downsamples points, and publishes compact JSON over MQTT.
 *
 * Output topic: slamaleykoum77/lidar ; slamaleykoum77/print
 */

#include "test_common_esp1.h"

static const char* MQTT_TOPIC_LIDAR_EXPRESS = "slamaleykoum77/lidar";
static const char* MQTT_TOPIC_LIDAR_EXPRESS_DEBUG = "slamaleykoum77/print1";

static const uint32_t PUBLISH_PERIOD_MS= 1000;
static const uint16_t DOWNSAMPLE_N     = 8;
static const uint16_t MAX_POINTS_OUT   = 180;
static const uint16_t SCAN_WINDOW      = 1200;

// 👉 Move big arrays to static/global (outside stack)
static char msg[4096];

uint32_t last_pub_ms = 0;

static void append_point(char* out, size_t out_sz, double angle_deg, uint16_t dist_mm, uint8_t quality) {
    char tmp[64];
    snprintf(tmp, sizeof(tmp), "{\"angle\":%.2f,\"distance\":%u,\"quality\":%u}", angle_deg, (unsigned)dist_mm, (unsigned)quality);
    strlcat(out, tmp, out_sz);
}

void setup_test_lidar_express() {
    Serial.begin(115200);

    uint8_t getInfoCmd[2] = {0xA5, 0x50};
    LIDAR_SERIAL.write(getInfoCmd, 2);
    LIDAR_SERIAL.flush();
    delay(200);

    connection.setupWifi();
    connection.check_connection();

    connection.client.setBufferSize(16384);

    char msgdbg[60];
    snprintf(msgdbg, sizeof(msgdbg), "I'm in the setup");
    connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_DEBUG, msgdbg);

    bool ret = lidar->start(express);
    char msgdbg2[60];
    snprintf(msgdbg2, sizeof(msgdbg2), "ret is : %u", ret);
    connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_DEBUG, msgdbg2); // if 0 then bug
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

    lidar->setAngleOfInterest(LIDAR_ANGLE_OF_INTEREST_START, LIDAR_ANGLE_OF_INTEREST_END);

    Serial.println("[LiDAR] STANDARD mode started.");
}

void loop_test_lidar_express() {
    connection.check_connection();

    uint32_t now = millis();
    if (now - last_pub_ms < PUBLISH_PERIOD_MS) return;
    last_pub_ms = now;

    uint16_t count = lidar->readMeasurePoints();  // Each Express scan returns multiple cabins
    // char db[60];
    // snprintf(db, sizeof(db), "count is: %u", count);
    // connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_DEBUG, db);
    if (count == 0) return;

    uint16_t kept = 0;
    // uint16_t stride = (DOWNSAMPLE_N == 0 ? 1 : DOWNSAMPLE_N);

    msg[0] = '\0';
    strlcat(msg, "[", sizeof(msg));

    for (uint16_t i = 0; i < SCAN_WINDOW && kept < MAX_POINTS_OUT; i += DOWNSAMPLE_N) {
        float angle = lidar->Data[i].angle;
        uint16_t dist = lidar->Data[i].distance;

        if (!std::isfinite(angle) || dist == 0) continue;

        // char msgdbg[60];
        // snprintf(msgdbg, sizeof(msgdbg), "angle: %.1f ; dist: %u", angle, dist);
        // connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_DEBUG, msgdbg);

        char tmp[64];
        snprintf(tmp, sizeof(tmp), "{\"angle\":%.1f,\"distance\":%u},", angle, dist);
        strlcat(msg, tmp, sizeof(msg));

        kept++;
    }

    if (kept > 0 && msg[strlen(msg)-1] == ',') msg[strlen(msg)-1] = ']';
    else strlcat(msg, "]", sizeof(msg));

    connection.publish(MQTT_TOPIC_LIDAR_EXPRESS, msg);

    Serial.printf("[LiDAR] published %d points\n", kept);
}
/*for (uint16_t i = 0, step = 0; i < SCAN_WINDOW && kept < MAX_POINTS_OUT; ++i, ++step) {
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
    }*/


    // Build JSON (start)
    // strlcpy(msg, "{\"type\":\"lidar\",\"mode\":\"express\",\"points\":[", sizeof(msg));

    // // append to JSON
    //     if (kept > 0) strlcat(msg, ",", sizeof(msg));
    //     append_point(msg, sizeof(msg), angle, dist, 0);

    // strlcat(msg, "]}", sizeof(msg));

    // // Publish final payload
    // connection.publish(MQTT_TOPIC_LIDAR_EXPRESS, msg);