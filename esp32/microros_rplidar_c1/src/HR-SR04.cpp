#include "UltraSonicSensor.h"

UltraSonicSensor::UltraSonicSensor(uint8_t triggerPin, uint8_t echoPin)
    : trigPin(triggerPin), echoPin(echoPin) {}

bool UltraSonicSensor::begin() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    digitalWrite(trigPin, LOW);
    delay(50);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH, 38UL * 1000UL);
    if (duration == 0) {
        // no echo received
        Serial.println("Ultrasonic begin: no echo (sensor not detected?)");
        return false;
    }

    range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    range_msg.field_of_view  = 0.26f;   // ~15°
    range_msg.min_range      = 0.02f;   // 2 cm
    range_msg.max_range      = 4.00f;   // 4 m
    range_msg.header.frame_id.data     = (char *)"ultrasonic_frame";
    range_msg.header.frame_id.size     = strlen(range_msg.header.frame_id.data);
    range_msg.header.frame_id.capacity = range_msg.header.frame_id.size + 1;

    Serial.printf("Ultrasonic begin: got echo pulse %lu µs\n", duration);
    return true;
}


float UltraSonicSensor::readDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 25000); // timeout 25ms
    float distance = duration * 0.0343 / 2.0;

    if (duration == 0) {
        return -1.0; // timeout or no object
    }
    
    return distance / 100.0; // convert to meters
}
