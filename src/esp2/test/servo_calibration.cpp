#include <Arduino.h>
#include "test_common_esp2.h"   // provides servo_dir + connection
#include "I2C_wire.h"

const char* mqtt_topic_calib = "slamaleykoum77/calibration";

void setup_servo_calibration() {
    Serial.begin(115200);
    delay(1000);

    // WiFi + MQTT
    connection.setupWifi();
    connection.check_connection();

    connection.publish(mqtt_topic_calib, "=== SERVO CALIBRATION START ===");

    // Init servo
    if (!servo_dir.begin()) {
        connection.publish(mqtt_topic_calib, "❌ Servo init failed!");
        while (1);
    }

    servo_dir.setAngle(90);
    connection.publish(mqtt_topic_calib, "Center → 90° (Straight)");
    delay(3000);
}

void loop_servo_calibration() {
    int angles[] = {60, 70, 80, 90, 100, 110, 120};
    int count = sizeof(angles) / sizeof(angles[0]);

    char bufcal[100];

    for (int i = 0; i < count; i++) {
        int a = angles[i];

        servo_dir.setAngle(a);

        snprintf(bufcal, sizeof(bufcal), "Servo set to %d° — measure wheel angle now.", a);
        connection.publish(mqtt_topic_calib, bufcal);

        // Publish a countdown every second (10 → 0)
        for (int t = 10; t > 0; t--) {
            snprintf(bufcal, sizeof(bufcal), "Angle %d° — %d sec remaining", a, t);
            connection.publish(mqtt_topic_calib, bufcal);
            delay(1000);
        }

        connection.publish(mqtt_topic_calib, "--------------------------------");
    }

    connection.publish(mqtt_topic_calib,
                       "=== Sweep complete. Restarting in 5 sec ===");
    delay(5000);
}
