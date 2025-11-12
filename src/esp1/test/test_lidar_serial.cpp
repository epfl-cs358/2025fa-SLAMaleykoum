/**
 * @file test_lidar_serial.cpp
 * @brief Test to try lidar with Serial1 (to not use after !)
 * @result Create a fake map with the mqtt_client.py file
 */
#include "test_common_esp1.h"

const char* MQTT_TOPIC_SERIAL = "slamaleykoum77/lidar";
const char* MQTT_TOPIC_SERIAL_DEBUG = "slamaleykoum77/print";

void setup_test_lidar_serial() {
    Serial.begin(115200);
    Serial1.begin(460800, SERIAL_8N1, 5, 4);

    connection.setupWifi();
}

void loop_test_lidar_serial() {
    connection.check_connection();

    char frstmsg[60];
    snprintf(frstmsg, sizeof(frstmsg), "i'm in the loop");
    connection.publish(MQTT_TOPIC_SERIAL_DEBUG, frstmsg);

    static unsigned long lastMsg = 0;
    static char buffer[64];

    while (Serial1.available()) {
        uint8_t val = Serial1.read();

        // Only send data occasionally to avoid MQTT spam
        if (millis() - lastMsg > 500) {
        snprintf(buffer, sizeof(buffer), "Received byte: 0x%02X (%u)", val, val);
        connection.publish(MQTT_TOPIC_SERIAL, buffer);
        lastMsg = millis();
        }
    }

    delay(100);
}
