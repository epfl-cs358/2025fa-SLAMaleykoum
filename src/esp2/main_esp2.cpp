/**
 * @file main_esp2.cpp
 * @brief Main control program for the SLAMaleykoum autonomous car.
 *
 * This file contains the primary firmware logic executed on the ESP32-S3 board.  
 * It integrates all hardware modules — motor, servo, ultrasonic sensor, IMU, and encoder — 
 * as well as the Wi-Fi and MQTT connection system.  
 * The current implementation serves as a comprehensive integration test, 
 * which will later evolve into the final autonomous driving program.
 *
 * @note
 * This version is primarily designed for validation and debugging.  
 * The final release will include autonomous navigation logic, data fusion, 
 * and closed-loop control algorithms.
 *
 * @author SLAMaleykoum
 * @date October 2025
 */
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "common/wifi_connection.h"

Connection connection;

const char* mqtt_topic_esp2esp1 = "slamaleykoum77/esps";

void mqtt_print(const char* str) {
    char msg[80];
    snprintf(msg, sizeof(msg), str);
    connection.publish(mqtt_topic_esp2esp1, msg);
}

void setup() {
    connection.setupWifi();
    connection.check_connection();

    mqtt_print("[ESP2] Setup ok");

    Serial1.begin(2000000, SERIAL_8N1, 13, 12); // RX=13, TX=12
    delay(1000);

    mqtt_print("ESP2 ready");
}

void loop() {
    if (Serial1.available()) {
        String msg = Serial1.readStringUntil('\n');
        mqtt_print("[ESP2] got: ");
        mqtt_print(msg.c_str());

        Serial1.println("PONG from ESP2");
    }

    delay(1000);
}