/**
 * @file test_common_esp1.cpp
 * @brief Test suite for validating ESP-to-ESP communication on ESP1.
 * It is designed to be launch at the same time as its twin on esp2 
 * (esp1 sends the first message that esp2 will catch and then send back a message to esp1)
 *
 * This file contains a collection of utilities and test routines used to
 * verify the UART communication layer implemented by Esp_link. It sends
 * predefined Pose2D and GlobalPathMessage structures, receives their
 * counterparts transmitted by ESP2, and reports correctness through MQTT.
 *
 * Main components:
 *   - Predefined test payloads (poses and paths for ESP1 and ESP2)
 *   - String-formatting helpers for Pose2D and GlobalPathMessage
 *   - Equality operators for structure comparison
 *   - MQTT logging utilities (one topic per purpose so that it is more readable)
 *   - test_pos() and test_path() routines for functional validation
 *
 * Control loop:
 *   - setup_esps_comm_esp1() initializes WiFi, MQTT, and the UART link
 *   - loop_esps_comm_esp1() polls for incoming bytes, executes tests, and logs results
 *
 * This file is intended for integration testing, not production use.
 */
#include "test_common_esp1.h"

Esp_link esp_link(ESPS);

const Pose2D pos_eps1 = {5, 5, 5, 0};
const Pose2D pos_eps2 = {2, 2, 2, 2};

const GlobalPathMessage path_esp1 = {
    .path = {
        {0.0f, 0.0f},
        {1.0f, 0.0f},
        {1.0f, 1.0f},
        {2.0f, 1.0f}
    },
    .current_length = 4,
    .path_id = 12,
    .timestamp_ms = 5
};
const GlobalPathMessage path_esp2 = {
    .path = {
        { 5.0f,  5.0f},
        { 6.0f,  5.0f},
        { 7.0f,  6.0f},
        { 7.5f, 7.2f},
    },
    .current_length = 4,
    .path_id = 67,
    .timestamp_ms = 987
};

const char* mqtt_topic_esps1_pos = "slamaleykoum77/pos1";
const char* mqtt_topic_esps1_gpm = "slamaleykoum77/gpm1";

void mqtt_print(const char* str, const char* topic = "slamaleykoum77/esps");
const char* pos_to_cstr(const Pose2D& p);
const char* gpm_to_cstr(const GlobalPathMessage& gpm);
inline bool operator==(const Pose2D& a, const Pose2D& b);
inline bool operator==(const Waypoint& a, const Waypoint& b);
inline bool operator==(const GlobalPathMessage& a, const GlobalPathMessage& b);

void test_pos();
void test_path();

void setup_esps_comm_esp1() {
    Serial.begin(115200);
    delay(1000);

    connection.setupWifi();
    connection.check_connection();
    delay(500);

    esp_link.begin();
    delay(1000);

    mqtt_print("ESP1 ready");
}

void loop_esps_comm_esp1() {
    connection.check_connection();
    delay(1000);

    esp_link.poll();

    test_path();

    delay(2000);
}

void test_pos() {
    esp_link.sendPos(pos_eps1);
    delay(1000);

    Pose2D out;
    if (esp_link.get_pos(out)) {
        mqtt_print("pos recevied on esp1");
        mqtt_print(pos_to_cstr(out), mqtt_topic_esps1_pos);
        if (out == pos_eps2) {
            mqtt_print("pos is exact on esp1");
        }
    }
}

void test_path() {
    esp_link.sendPath(path_esp1);
    mqtt_print(gpm_to_cstr(path_esp1), mqtt_topic_esps1_gpm);

    delay(1000);

    GlobalPathMessage out;
    if (esp_link.get_path(out)) {
        mqtt_print("path recevied on esp1");
        mqtt_print(gpm_to_cstr(out), mqtt_topic_esps1_gpm);
        if (out == path_esp2) {
            mqtt_print("path is exact on esp1");
        }
    }
}

void mqtt_print(const char* str, const char* topic) {
    Serial.println(str);

    char msg[200];
    snprintf(msg, sizeof(msg), str);
    connection.publish(topic, msg);
}

const char* pos_to_cstr(const Pose2D& p) {
    static char buf[64];
    snprintf(buf, sizeof(buf),
             "x=%.3f, y=%.3f, th=%.3f, t=%u",
             p.x, p.y, p.theta, p.timestamp_ms);
    return buf;
}

const char* gpm_to_cstr(const GlobalPathMessage& gpm) {
    static char buf[512];
    size_t pos = 0;

    pos += snprintf(buf + pos, sizeof(buf) - pos,
                    "PathID=%u, time=%u, len=%u\n",
                    gpm.path_id, gpm.timestamp_ms, gpm.current_length);

    for (uint16_t i = 0; i < gpm.current_length; i++) {
        pos += snprintf(buf + pos, sizeof(buf) - pos,
                        "  [%u] (%.3f, %.3f)\n",
                        i,
                        gpm.path[i].x,
                        gpm.path[i].y);
    }
    return buf;
}

inline bool operator==(const Pose2D& a, const Pose2D& b) {
    return
        a.x == b.x &&
        a.y == b.y &&
        a.theta == b.theta &&
        a.timestamp_ms == b.timestamp_ms;
}

inline bool operator==(const Waypoint& a, const Waypoint& b) {
    return a.x == b.x && a.y == b.y;
}

inline bool operator==(const GlobalPathMessage& a, const GlobalPathMessage& b) {
    if (a.path_id        != b.path_id)        return false;
    if (a.timestamp_ms   != b.timestamp_ms)   return false;

    for (uint16_t i = 0; i < 4; i++) {
        if (!(a.path[i] == b.path[i]))
            return false;
    }
    return true;
}
