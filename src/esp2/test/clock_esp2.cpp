#include "test_common_esp2.h"

void mqtt_print_clock_esp2(const char* str, const char* topic = "slamaleykoum77/esp2");
int i = 0;

void setup_clock_esp2() {
    connection.setupWifi();
    connection.check_connection();
    delay(1000);

    char msg[100];
    snprintf(msg, sizeof(msg), "setup,    millis_esp2 = %lu ms", (unsigned long)millis());
    mqtt_print_clock_esp2(msg);

    mqtt_print_clock_esp2("ESP2 ready");
}

void loop_clock_esp2() {
    connection.check_connection();
    delay(2000);

    char msg[100];
    snprintf(msg, sizeof(msg), "loop %d,    millis_esp2 = %lu ms", i++, (unsigned long)millis());
    mqtt_print_clock_esp2(msg);
}

void mqtt_print_clock_esp2(const char* str, const char* topic) {
    char msg[200];
    snprintf(msg, sizeof(msg), str);
    connection.publish(topic, msg);
}