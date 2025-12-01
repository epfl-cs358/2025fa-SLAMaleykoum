#include "test_common_esp1.h"
 
// #define LED_PIN LED_BUILTIN

void setup_led_basic() {
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115600);
    delay(1000);
}

void loop_led_basic() {
    Serial.println("hi");
    digitalWrite(LED_PIN, HIGH);
    delay(2000);
    digitalWrite(LED_PIN, LOW);

    delay(1000);
}