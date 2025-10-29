/**
 * @file main.cpp
 * @brief Main program for manually controlling the car via keyboard inputs (WDAS Q)
 * 
 * @author SLAMaleykoum
 * @date October 2025
 */


#include <Arduino.h>
#include "MotorManager.h"
#include "DMS15.h"
#include "UltraSonicSensor.h"
#include "ImuSensor.h"
#include "I2C_mutex.h"
#include "AS5600Encoder.h"
#include "WiFi.h"
#include <PubSubClient.h>

// Pins used by the hardware components on the esp 2
#define ESC_PIN 15          // pin used for the motor
#define SERVO_DIR_PIN 6     // the servo that modifies the direction of the wheels
#define US_TRIG_PIN 5       // changé par rapport a avant sur conseil de chat
#define US_ECHO_PIN 19      // changé par rapport a avant sur conseil de chat
#define SDA_PIN 8           // pin used for SDA of IMU
#define SCL_PIN 9           // pin used for SCL of IMU

const char* ssid = "PCSlam"; // change it
const char* pass = "SlamAleykoum"; // change it

// HiveMQ public broker
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic_data = "slamaleykoum77/TheSLAM";
const char* mqtt_topic_commands = "slamaleykoum77/commands";

WiFiClient espClient;
PubSubClient client(espClient);

// Harwdare objects
MotorManager motor(ESC_PIN); // motor
DMS15 servo_dir(SERVO_DIR_PIN); // servo
UltraSonicSensor ultrasonic(US_TRIG_PIN, US_ECHO_PIN); // ultra sonic sensor
ImuSensor imu(SDA_PIN, SCL_PIN); // imu
AS5600Encoder encoder(SDA_PIN, SCL_PIN); // encoder

// Parameters
int drivePower = 20;
int angle = 90;
const int stepDeg = 5;
const int minAngle = 60;
const int maxAngle = 120;

// setup wifi function
void setupWifi(){
    delay(100);
    Serial.print("\nConnecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, pass);

    while(WiFi.status() != WL_CONNECTED) {
        delay(100);
        Serial.print("-");
    }

    Serial.print("\nConnected to ");
    Serial.println(ssid);
}

// reconnect mqtt function
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (client.connect("ESP32_Client")) {
        Serial.println("connected!");
        client.subscribe(mqtt_topic_commands);
    } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" - retrying in 2 seconds");
        delay(2000);
    }
  }
}

// callback to receive messages from mqtt explorer
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.println(topic);

    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.print("Message: ");
    Serial.println(message);

    if (String(topic) == mqtt_topic_commands) {
        if (message == "forward") {
            motor.forward(drivePower / 100.0f);
        } else if (message == "stop") {
            motor.stop();
        }
    }
}

void setup() {
    
    Serial.begin(115200);

    setupWifi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);

    delay(2000);

    // Initialize motor
    if (!motor.begin()) {
        Serial.println("Motor init failed!");
        while (true);
    }

    // Initialize servo
    if (!servo_dir.begin()) {
        Serial.println("Servo init failed!");
        while (true);
    }
    servo_dir.setAngle(angle);

    // Initialize ultrasonic sensor
    // if (!ultrasonic.begin()) {
    //     Serial.println("Ultrasonic init failed!");
    //     while (true);
    // }

    i2cMutexInit();

    // Initialize imu
    if (!imu.begin()) {
        Serial.println("IMU init failed!");
        while(true);
    }

    // Initialize encoder
    if (!encoder.begin()) {
        Serial.println("encoder init failed!");
        while(true);
    }

    Serial.println("Use W/S to drive, SPACE to stop, A/D to steer, Q to quit.");
}

void loop() {
    if (!client.connected()) reconnect();
    client.loop();

    // Print the current state at the beginning of the loop
    Serial.println(imu.data());

    // Print the current motor angle at the beginning of the loop
    Serial.print("Motor angle = ");
    Serial.println(encoder.update());

    float dist = ultrasonic.readDistance(); //read once per loop so no multiple actions at the same time

    // ***** TEST FOR MQTT (SEND SENSOR DISTANCE)
    char msg[50];
    snprintf(msg, sizeof(msg), "Distance: %.2f cm", dist);

    // Publish data to MQTT
    if (client.publish(mqtt_topic_data, msg)) {
        Serial.print("Published: ");
        Serial.println(msg);
    } else {
        Serial.println("Failed to publish message!");
    }
    // ****** END TEST

    //read keyboard serial input
    if (Serial.available() > 0) {
        char c = tolower(Serial.read());

        switch (c) {

            //Forward
            case 'w':
                if (dist < 0 || dist > UltraSonicSensor::STOP_THRESHOLD) {
                    motor.forward(drivePower / 100.0f);
                    Serial.println("Forward");
                } else {
                    motor.stop();
                    Serial.println("Obstacle! Stop");
                }
                break;
            
            //Backwards
            case 's':
                motor.backward(drivePower / 100.0f);
                Serial.println("Backward");
                break;
            
            //Stop
            case ' ':
                motor.stop();
                Serial.println("Stop");
                break;
            
            //Turn left    
            case 'a':
                angle = max(minAngle, angle - stepDeg);
                servo_dir.setAngle(angle);
                Serial.print("Steer Left: "); Serial.println(angle);
                break;

            //Turn right
            case 'd':
                angle = min(maxAngle, angle + stepDeg);
                servo_dir.setAngle(angle);
                Serial.print("Steer Right: "); Serial.println(angle);
                break;
            
            //Quit
            case 'q':
                motor.stop();
                Serial.println("Quit");
                while (true);
        }
    }

    // Emergency stop if an obstacle is detected
    if (dist > 0 && dist < UltraSonicSensor::STOP_THRESHOLD) {
        motor.stop();
        Serial.println("Emergency Stop! Obstacle detected.");
    }

    motor.update();
    delay(100);
    imu.readAndUpdate();
    encoder.update();
    delay(100);
}