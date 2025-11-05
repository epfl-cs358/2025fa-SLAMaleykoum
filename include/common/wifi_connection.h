/**
 * @file wifi_connection.h
 * @brief Defines the Connection class for managing Wi-Fi and MQTT communication on ESP32.
 *
 * This header declares the `Connection` class and the associated MQTT broker configuration 
 * variables. The class handles all aspects of establishing and maintaining Wi-Fi and MQTT 
 * connections on the ESP32 platform, including:
 * - Connecting to a predefined Wi-Fi network
 * - Connecting and reconnecting to an MQTT broker
 * - Subscribing and publishing to specific MQTT topics
 *
 * @details
 * The implementation uses the Arduino `WiFi` and `PubSubClient` libraries to provide
 * a simple interface for communication with MQTT brokers such as HiveMQ.
 *
 * Typical usage example:
 * @code
 * Connection conn;
 * conn.setupWifi();
 * conn.reconnect();
 * conn.publish("Hello world!");
 * conn.check_connection();
 * @endcode
 * 
 * @author SLAMaleykoum & TurboSLAM no changes
 * @date Oct 2025
 */
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

/**
 * @section MQTT Configuration
 * @brief Global constants defining the MQTT broker connection parameters.
 *
 * These external variables are defined in `wifi_connection.cpp`
 * and are shared across modules requiring MQTT access.
 */
extern const char* mqtt_server;
extern const int mqtt_port;
extern const char* mqtt_topic_data;
extern const char* mqtt_topic_commands;

/**
 * @class Connection
 * @brief Manages Wi-Fi and MQTT communication for the ESP32.
 *
 * The Connection class encapsulates Wi-Fi setup and MQTT client management.
 * It provides utilities to connect to a Wi-Fi network, establish and maintain 
 * an MQTT connection, publish messages, and handle reconnection automatically.
 */
class Connection {
    private:
    const char* ssid = "Wifi de am";
    const char* pass = "chibani5"; 

    WiFiClient _espClient;  // Underlying TCP client used by the MQTT client.
    PubSubClient client;    // MQTT client for broker communication.

    public:
    /**
     * @brief Constructs a new Connection object.
     *
     * Initializes the internal MQTT client (`PubSubClient`) using the ESP32 
     * Wi-Fi client (`WiFiClient`).  
     * No network communication is performed during construction.
     *
     * @note You must call `setupWifi()` and `reconnect()` to establish 
     *       Wi-Fi and MQTT connections.
     */
    Connection();

    /**
     * @brief Connects the ESP32 to Wi-Fi and configures MQTT.
     * 
     * Attempts to connect to the specified Wi-Fi network using the stored 
     * SSID and password. The function blocks until a connection is established, 
     * displaying progress on the serial monitor.  
     * Once connected, the MQTT client is configured with the target broker.
     * 
     * @note This function blocks until Wi-Fi is successfully connected.
     */
    void setupWifi();

    /** 
     * @brief Reconnects to the MQTT broker if disconnected.
     *
     * Continuously retries connecting to the configured MQTT broker until 
     * successful. Once connected, the device subscribes to the predefined 
     * command topic to receive control messages.
     * 
     * @note This function blocks until the MQTT connection succeeds.
     * 
     * @details
     * - Prints connection attempts and error codes to the Serial monitor.
     * - Subscribes to `mqtt_topic_commands` after successful reconnection.
     */
    void reconnect();

    /**
     * @brief Keeps the MQTT connection alive and processes messages.
     *
     * Checks if the MQTT client is still connected.  
     * If disconnected, automatically calls `reconnect()`.  
     * Also executes `client.loop()` to handle background MQTT operations 
     * (keep-alive, message processing, etc.).
     *
     * @note Should be called regularly inside the main `loop()` function.
     */
    void check_connection();

    /**
     * @brief Publishes a message to the MQTT broker.
     *
     * Sends a message to the predefined MQTT topic used for data transmission.
     * Logs the result of the publish attempt to the serial monitor.
     *
     * @param msg The message payload to publish (as a null-terminated string).
     *
     * @note The function does not buffer messages if disconnected.  
     *       Ensure `check_connection()` is called regularly.
     */
    void publish(const char* mqtt_topic, const char* msg);
};
