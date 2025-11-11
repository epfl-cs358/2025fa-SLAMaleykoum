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

static const char* MQTT_TOPIC_LIDAR_EXPRESS = "slamaleykoum77/lidar";
static const char* MQTT_TOPIC_LIDAR_EXPRESS_DEBUG = "slamaleykoum77/print1";

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

    uint8_t getInfoCmd[2] = {0xA5, 0x50};
    LIDAR_SERIAL.write(getInfoCmd, 2);
    LIDAR_SERIAL.flush();
    delay(200);

    connection.setupWifi();
    connection.check_connection();

    /*uint8_t getInfoCmd[2] = {0xA5, 0x50}; // commande GET_INFO
    LIDAR_expr.write(getInfoCmd, 2);

    delay(1000);  // petite attente de réponse

    if (LIDAR_expr.available()) {
        connection.publish(MQTT_TOPIC_LIDAR_debug,"[LIDAR] Data available after GET_INFO:");
        
        char buffer[512];
        int idx = 0;
        int count = 0;
        while (LIDAR_expr.available() && idx < sizeof(buffer) - 3) {
            uint8_t b = LIDAR_expr.read();
            count++;
            idx += snprintf(buffer + idx, sizeof(buffer) - idx, "%02X ", b);
        }
        buffer[idx] = '\0';
        char msg[64];
        snprintf(msg, sizeof(msg), "[LIDAR] read %d bytes", count);
        connection.publish(MQTT_TOPIC_LIDAR_debug, msg);

        if (idx > 0) {
            connection.publish(MQTT_TOPIC_LIDAR_debug, buffer);
        }
    } else {
        connection.publish(MQTT_TOPIC_LIDAR_debug,"[LIDAR] No data received (check wiring, power, baudrate)");
    }*/

    bool ret = lidar->start(express);
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

void loop_test_lidar_express() {
    connection.check_connection();

    // uint16_t blocks = lidar->readMeasurePoints();
    // (void)blocks;

    stDeviceInfo_t devinfo = lidar->getDeviceInfo();
    char msgdbg[200];
    snprintf(msgdbg, sizeof(msgdbg),
            "✅ RPLIDAR Device Info:\r\n"
            " Model: %d\r\n"
            " Firmware: %d.%d\r\n"
            " Hardware: %d\r\n"
            " Serial: ",
            devinfo.model,
            devinfo.firmware_major,
            devinfo.firmware_minor,
            devinfo.hardware);
    
    // publish the first part
    connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_DEBUG, msgdbg);

    // print serial number separately (hexadecimal)
    char serialStr[100];
    serialStr[0] = '\0';
    for (int i = 0; i < 16; ++i) {
        char byteStr[4];
        snprintf(byteStr, sizeof(byteStr), "%02X", devinfo.serialnumber[i]);
        strncat(serialStr, byteStr, sizeof(serialStr) - strlen(serialStr) - 1);
    }

    char msgSerial[120];
    snprintf(msgSerial, sizeof(msgSerial), " SerialNumber: %s\r\n", serialStr);
    connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_DEBUG, msgSerial);

    delay(1000);

    /*if (lidar.available()) {
        connection.publish(MQTT_TOPIC_LIDAR_debug,"[LIDAR] Data available after GET_INFO:");
        
        char buffer[512];
        int idx = 0;
        int count = 0;
        while (lidar.available() && idx < sizeof(buffer) - 3) {
            uint8_t b = LIDAR_expr.read();
            count++;
            idx += snprintf(buffer + idx, sizeof(buffer) - idx, "%02X ", b);
        }
        buffer[idx] = '\0';
        char msg[64];
        snprintf(msg, sizeof(msg), "[LIDAR] read %d bytes", count);
        connection.publish(MQTT_TOPIC_LIDAR_debug, msg);

        if (idx > 0) {
            connection.publish(MQTT_TOPIC_LIDAR_debug, buffer);
        }
    } else {
        connection.publish(MQTT_TOPIC_LIDAR_debug,"[LIDAR] No data received (check wiring, power, baudrate)");
    }*/

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
        snprintf(debugmsg, sizeof(debugmsg), "angle: %.1f ; dist: %u", angle, dist);
        connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_DEBUG, debugmsg);

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
