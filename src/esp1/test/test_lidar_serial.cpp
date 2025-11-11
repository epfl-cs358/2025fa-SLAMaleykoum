#include "test_common_esp1.h"

const char* mqtt_topic_serial = "slamaleykoum77/print";
const char* mqtt_topic_serial2 = "slamaleykoum77/lidar";

void setup_test_lidar_serial() {
    Serial.begin(115200);
    Serial1.begin(460800, SERIAL_8N1, 5, 4);  // or 256000 depending on model

    connection.setupWifi();
}

void loop_test_lidar_serial() {
    connection.check_connection();

    char frstmsg[60];
    snprintf(frstmsg, sizeof(frstmsg), "i'm in the loop");
    connection.publish(mqtt_topic_serial, frstmsg);

    static unsigned long lastMsg = 0;
    static char buffer[64];

    while (Serial1.available()) {
        uint8_t val = Serial1.read();

        // Only send data occasionally to avoid MQTT spam
        if (millis() - lastMsg > 500) {
        snprintf(buffer, sizeof(buffer), "Received byte: 0x%02X (%u)", val, val);
        connection.publish(mqtt_topic_serial2, buffer);
        lastMsg = millis();
        }
    }

    delay(100);
}
