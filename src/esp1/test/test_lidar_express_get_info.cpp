/**
 * @file test_lidar_express_get_info.cpp
 * @brief Runs RPLIDAR S2 in EXPRESS mode, downsamples points, and publishes compact JSON over MQTT.
 *
 * Output topic: slamaleykoum77/lidar ; slamaleykoum77/print
 */

#include "test_common_esp1.h"

static const char* MQTT_TOPIC_LIDAR_EXPRESS_GET_INFO = "slamaleykoum77/lidar";
static const char* MQTT_TOPIC_LIDAR_EXPRESS_GET_INFO_DEBUG = "slamaleykoum77/print1";

static const uint32_t PUBLISH_PERIOD_MS= 200;
static const uint16_t DOWNSAMPLE_N     = 8;
static const uint16_t MAX_POINTS_OUT   = 180;
static const uint16_t SCAN_WINDOW      = 1200;

void setup_test_lidar_express_get_info() {
    Serial.begin(115200);

    connection.setupWifi();
    connection.check_connection();

    bool ret = lidar->start(express);
    delay(1000);
    if (ret) {
        char msgtest[60];
        snprintf(msgtest, sizeof(msgtest), "🟢 Rplidar C1 started correctly!\r\n");
        connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_GET_INFO_DEBUG, msgtest);
    } else {
        char msgtest[60];
        snprintf(msgtest, sizeof(msgtest), "🔴 Error starting Rplidar C1\r\n");
        connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_GET_INFO_DEBUG, msgtest);
    }

    delay(2000);

    lidar->setAngleOfInterest(LIDAR_ANGLE_OF_INTEREST_START, LIDAR_ANGLE_OF_INTEREST_END);

    Serial.println("[LiDAR] STANDARD mode started.");
}

void loop_test_lidar_express_get_info() {
    connection.check_connection();

    stDeviceInfo_t devinfo = lidar->getDeviceInfo();
    char infos[200];
    snprintf(infos, sizeof(infos),
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
    connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_GET_INFO, infos);

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
    connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_GET_INFO_DEBUG, msgSerial);

    delay(1000);

    stDeviceStatus_t sdst = lidar->getDeviceHealth();

    // Combine the two bytes into one 16-bit error code
    uint16_t errorCode = ((uint16_t)sdst.errorCode_high << 8) | sdst.errorCode_low;

    // Check health status
    const char* healthStr;
    if (sdst.status == 0) {
        healthStr = "🟢 OK";
    } else if (sdst.status == 1) {
        healthStr = "🟡 WARNING";
    } else {
        healthStr = "🔴 NOT OK";
    }

    char health[200];
    snprintf(health, sizeof(health),
         "LiDAR Health: %s | status=%u | errorCode=0x%04X (%u)\r\n",
         healthStr, sdst.status, errorCode, errorCode);

    // Publish to MQTT
    connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_GET_INFO, health);

    delay(1000);

    Serial.printf("[LiDAR] get info and health finished\n");
}
