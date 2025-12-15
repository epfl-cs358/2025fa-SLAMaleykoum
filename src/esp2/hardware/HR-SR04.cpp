/**
 * @file HR-SR04.cpp
 * @brief Implementation of the UltraSonicSensor class for the HR-SE04 sensor
 */

#include "hardware/UltraSonicSensor.h"

UltraSonicSensor::UltraSonicSensor(uint8_t triggerPin, uint8_t echoPin)
    : trigPin(triggerPin), echoPin(echoPin) {}

bool UltraSonicSensor::begin() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    digitalWrite(trigPin, LOW);
    delay(50);

    // Sends pulse
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Checks how long it took for the pulse to be received
    // Prints message accordingly
    unsigned long duration = pulseIn(echoPin, HIGH, 38UL * 1000UL); 
    if (duration == 0) {
        // no echo received
        Serial.println("Ultrasonic begin: no echo (sensor not detected?)");
        return false;
    }

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
