#include "test_common_esp1.h"

int i = 0;
uint64_t t0 = 0;

void setup_clock_esp1() {
    connection.setupWifi();
    connection.check_connection();
    delay(1000);

    connection.publish("slamaleykoum77/esp1", "ESP1 ready");
    t0 = esp_wifi_get_tsf_time(WIFI_IF_STA);
}

void loop_clock_esp1() {
    connection.check_connection();
    delay(4000);

    char msg[200];
    snprintf(msg, sizeof(msg), "loop %d,    millis = %10lu ms,   tsf = %.3f ms", 
            i++, (unsigned long)millis(), ((double)esp_wifi_get_tsf_time(WIFI_IF_STA)-t0)/1000.0);
    connection.publish("slamaleykoum77/esp1", msg);
}
