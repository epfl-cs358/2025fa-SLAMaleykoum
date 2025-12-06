#include "test_common_esp2.h"

int i = 0;
uint64_t t0 = 0;

void setup_clock_esp2() {
    connection.setupWifi();
    connection.check_connection();
    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin("TSF_AP", "12345678");
    delay(1000);

    connection.publish("slamaleykoum77/esp2", "ESP2 ready");
    t0 = esp_wifi_get_tsf_time(WIFI_IF_STA);
}

void loop_clock_esp2() {
    connection.check_connection();
    delay(4000);

    char msg[200];
    snprintf(msg, sizeof(msg), "loop %d,    millis = %10lu ms,   tsf = %.3f ms", 
            i++, (unsigned long)millis(), ((double)esp_wifi_get_tsf_time(WIFI_IF_STA)-t0)/1000.0);    
    connection.publish("slamaleykoum77/esp2", msg);
}
