#include "test_common_esp1.h"
 
// #define LED_PIN LED_BUILTIN

void setup_led_basic() {
    pinMode(LED_PIN, OUTPUT);
    delay(1000);
}

void loop_led_basic() {
    digitalWrite(LED_PIN, HIGH);
    delay(10000);
}