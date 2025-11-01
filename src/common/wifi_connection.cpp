/**
 * @file wifi_connection.cpp
 * @brief Implementation of the Connection class for managing Wi-Fi and MQTT communication.
 *
 * This source file defines the methods declared in `wifi_connection.h`.
 * It establishes a Wi-Fi connection, manages MQTT setup and reconnection,
 * and provides utilities for publishing and maintaining the MQTT link.
 *
 * @see wifi_connection.h
 */
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "common/wifi_connection.h"

const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic_commands = "slamaleykoum77/commands";

Connection::Connection() : 
    client(_espClient) {}

void Connection::setupWifi(){
    delay(100);
    Serial.print("\nConnecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, pass);

    while(WiFi.status() != WL_CONNECTED) {
        delay(100);
        Serial.print("-");
    }

    Serial.print("\nConnected to ");
    Serial.println(ssid);

    delay(100);
    client.setServer(mqtt_server, mqtt_port);
}

void Connection::reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (client.connect("ESP32_Client")) {
        Serial.println("connected!");
        client.subscribe(mqtt_topic_commands); // inutile pour nous mais on garde au cas où (manque callback function)
    } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" - retrying in 2 seconds");
        delay(2000);
    }
  }
}

void Connection::check_connection() {
    if (!client.connected()) reconnect();
    client.loop();
}

void Connection::publish(const char* mqtt_topic, const char* msg) {
    if (client.publish(mqtt_topic, msg)) {
        Serial.print("Published: ");
        Serial.println(msg);
    } else {
        Serial.println("Failed to publish message!");
    }
}
