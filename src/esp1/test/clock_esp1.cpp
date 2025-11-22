#include "test_common_esp1.h"

void mqtt_print_clock_esp1(const char* str, const char* topic = "slamaleykoum77/esp1");
int j = 0;

void setup_clock_esp1() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("serial ok");
    connection.setupWifi();
    connection.check_connection();
    delay(1000);

    char msg[100];
    snprintf(msg, sizeof(msg), "setup,    millis_esp1 = %lu", (unsigned long)millis());
    mqtt_print_clock_esp1(msg);

    mqtt_print_clock_esp1("ESP1 ready");
}

void loop_clock_esp1() {
    connection.check_connection();
    delay(2000);

    char msg[100];
    snprintf(msg, sizeof(msg), "loop %d,    millis_esp1 = %lu ms", j++, (unsigned long)millis());
    mqtt_print_clock_esp1(msg);
}

void mqtt_print_clock_esp1(const char* str, const char* topic) {
    char msg[200];
    snprintf(msg, sizeof(msg), str);
    connection.publish(topic, msg);
}