#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// HiveMQ public broker
extern const char* mqtt_server;
extern const int mqtt_port;
extern const char* mqtt_topic_data;
extern const char* mqtt_topic_commands;

class Connection {
    private:
    const char* ssid = "CMAISONNIER 9694";
    const char* pass = "8>73rR97"; 

    WiFiClient _espClient;
    PubSubClient client;

    public:
    Connection(WiFiClient& espClient);

    // setup wifi function
    void setupWifi();

    // reconnect mqtt function
    void reconnect();

    // checks the connection and reconnect if needed
    void check_connection();

    void publish(const char* msg);
};
