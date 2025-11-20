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

    static uint32_t lastPub = millis();       // last publish timestamp
    static uint16_t pointCount = 0;           // number of points buffered
    static char msg[3000];                    // JSON buffer

    // build / append data into JSON buffer
    if (pointCount == 0) strlcpy(msg, "[", sizeof(msg));

    // always read lidar data (don’t rate-limit reading)
    uint16_t count = lidar->readMeasurePoints();
    if (count > 0) {
        for (uint16_t i = 0; i < count && pointCount < MAX_POINTS_OUT; i += DOWNSAMPLE_N) {
            float angle = lidar->Data[i].angle;
            uint16_t dist = lidar->Data[i].distance;

            if (!std::isfinite(angle) || dist == 0) continue;

            char tmp[64];
            snprintf(tmp, sizeof(tmp), "{\"angle\":%.1f,\"distance\":%u},", angle, dist);
            strlcat(msg, tmp, sizeof(msg));

            pointCount++;
        }
    }

    // publish every PUBLISH_PERIOD_MS
    uint32_t now = millis();
    if (now - lastPub >= PUBLISH_PERIOD_MS) {
        if (pointCount > 0) {
            // close the JSON array cleanly
            if (msg[strlen(msg) - 1] == ',')
                msg[strlen(msg) - 1] = ']';
            else
                strlcat(msg, "]", sizeof(msg));

            connection.publish(MQTT_TOPIC_LIDAR_EXPRESS, msg);
            Serial.printf("[LiDAR] published %u points\n", pointCount);
        }

        // reset for next period
        pointCount = 0;
        lastPub = now;
    }
}
