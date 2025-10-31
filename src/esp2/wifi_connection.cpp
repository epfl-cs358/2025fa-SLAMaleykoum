#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "wifi_connection.h"

const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic_data = "slamaleykoum77/TheSLAM";
const char* mqtt_topic_commands = "slamaleykoum77/commands";

Connection::Connection(WiFiClient& espClient) : 
    client(espClient), _espClient(espClient) {}

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
        client.subscribe(mqtt_topic_commands);
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

void Connection::publish(const char* msg) {
    if (client.publish(mqtt_topic_data, msg)) {
        Serial.print("Published: ");
        Serial.println(msg);
    } else {
        Serial.println("Failed to publish message!");
    }
}
