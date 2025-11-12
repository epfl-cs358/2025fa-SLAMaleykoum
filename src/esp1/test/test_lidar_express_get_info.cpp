/**
 * @file test_lidar_express_get_info.cpp
 * @brief Runs RPLIDAR S2 in EXPRESS mode, downsamples points, and publishes compact JSON over MQTT.
 *
 * Output topic: slamaleykoum77/lidar ; slamaleykoum77/print
 */

#include "test_common_esp1.h"

static const char* MQTT_TOPIC_LIDAR_EXPRESS_GET_INFO = "slamaleykoum77/lidar";
static const char* MQTT_TOPIC_LIDAR_EXPRESS_GET_INFO_DEBUG = "slamaleykoum77/print";

static const uint32_t PUBLISH_PERIOD_MS= 200;
static const uint16_t DOWNSAMPLE_N     = 8;
static const uint16_t MAX_POINTS_OUT   = 180;
static const uint16_t SCAN_WINDOW      = 1200;

// Move big arrays to static/global (outside stack)
static char debugmsg[128];

void setup_test_lidar_express_get_info() {
    Serial.begin(115200);

    uint8_t getInfoCmd[2] = {0xA5, 0x50};
    LIDAR_SERIAL.write(getInfoCmd, 2);
    LIDAR_SERIAL.flush();
    delay(200);

    connection.setupWifi();
    connection.check_connection();

    bool ret = lidar->start(express);
    delay(1000);
    if (ret) {
        strlcpy(debugmsg, "🟢 Rplidar C1 started correctly!\r\n", sizeof(debugmsg));
        connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_GET_INFO_DEBUG, debugmsg);
    } else {
        strlcpy(debugmsg, "🔴 Error starting Rplidar C1\r\n", sizeof(debugmsg));
        connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_GET_INFO_DEBUG, debugmsg);
    }

    delay(2000);

    stDeviceStatus_t sdst = lidar->getDeviceHealth();
    snprintf(debugmsg, sizeof(debugmsg),
            "sdst.errorCode_high=%u  sdst.errorCode_low=%u  sdst.status=%u\r\n",
            sdst.errorCode_high, sdst.errorCode_low, sdst.status);
    connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_GET_INFO_DEBUG, debugmsg);

    lidar->setAngleOfInterest(LIDAR_ANGLE_OF_INTEREST_START, LIDAR_ANGLE_OF_INTEREST_END);

    Serial.println("[LiDAR] STANDARD mode started.");
}

void loop_test_lidar_express_get_info() {
    connection.check_connection();

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
    connection.publish(MQTT_TOPIC_LIDAR_EXPRESS_GET_INFO, msgdbg);

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

    Serial.printf("[LiDAR] get info finish\n");
}
