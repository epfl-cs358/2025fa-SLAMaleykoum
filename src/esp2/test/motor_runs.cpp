/**
 * @file motor_runs
 * @brief Test to make the motor run simply
 */
#include "test_common_esp2.h"
#include "EncoderCarVelocity.h"

const char* mqtt_topic_connection_motor_runs = "slamaleykoum77/print";

//Wheel & motor parameters
//const float GEAR_RATIO = 10.0f; //it takes the motor 10 full rotations to make the wheel turn a full rotation
const float WHEEL_RADIUS_M = 0.045f; //wheel radius in meters
const float TARGET_MOTOR_DEGREES = 360.0f *30; //target to reach to make the motor stop -> *# of rotations


static unsigned long lastVelocity = 0; //previous velocity

static int drivePower = 25; 

const int ANGLE_INTERVAL_MS = 100;//5; //if odd go back to 300 but one full motor revolution takes around 370 milliseconds
// 300; // velocity sample interval in milliseconds
const float TARGET_DISTANCE_M =30.1f;

static float estimatedDistance = 0.0f;
static unsigned long lastSampleTime = 0;


//static EncoderCarVelocity carVelocity(&encoder);
//EncoderCarVelocity encoder;

static unsigned long lastVelocitySample = 0;
static unsigned long lastDistanceSample = 0;
static float lastValidVelocity = 0.0f;

// Sample velocity every 50ms (must be longer than internal delay in getMotorAngularVelocity)
const int VELOCITY_SAMPLE_INTERVAL_MS = 40;
const int DISTANCE_UPDATE_INTERVAL_MS = 100;  // Update distance less frequently


void setup_motor_runs() {
    
    Serial.begin(115200);
    delay(2000);

    //Wifi setup
    connection.setupWifi();

    //Light setup
    pinMode(LED_BUILTIN, OUTPUT);

    //I2C setup for IMU and Encoder
    i2cMutexInit();

    //Wifi connection 
    connection.check_connection();
    
    // Initialize motor
    if (!motor.begin()) {
        char msgMotor[50];
        snprintf(msgMotor, sizeof(msgMotor), "Motor init failed!");
        connection.publish(mqtt_topic_connection_motor_runs, msgMotor);



        while (true);
    }
    
    //Initialize encoder
    encoder.begin();
    /*if (!encoder.begin()) {
        char msgEncoder[50];
        snprintf(msgEncoder, sizeof(msgEncoder), "Encoder init failed!");
        connection.publish(mqtt_topic_connection_motor_runs,msgEncoder);
        while (true);
    }*/

    //check again the connection before sending the message to see if the setup succeeded
    connection.check_connection();
    char msgSetup[50];
    snprintf(msgSetup, sizeof(msgSetup), "Setup success!");
    connection.publish(mqtt_topic_connection_motor_runs,msgSetup);

   

    
}

unsigned long startTime = 0;
bool motorRunning = false;

void loop_motor_runs() {
    
    //motor.forward(drivePower/100.0f);
    //motor.update();
    /*uint16_t angleRaw = encoder.as5600.readAngle(); // 0..4095
    float angleDeg = angleRaw * AS5600_RAW_TO_DEGREES; // convert to degrees

     connection.check_connection();
    char msgSetup[50];
    snprintf(msgSetup, sizeof(msgSetup), "Angle is in degrees %0.2f", angleDeg);
    connection.publish(mqtt_topic_connection_motor_runs,msgSetup);*/

    //delay(3000);

    /*
    float velocityAngle = encoder.getFilteredAngularVelocity();//getFilteredMotorAngularVelocity(); // 0..4095

    char msgSpeed[50];
    snprintf(msgSpeed, sizeof(msgSpeed), "Angular speed %0.3f", velocityAngle);
    connection.publish(mqtt_topic_connection_motor_runs,msgSpeed);
*/
    /*
    float carSpeed = encoder.getWheelLinearVelocity(); // 0..4095
    connection.check_connection();
    char msgCarSpeed[50];
    snprintf(msgCarSpeed, sizeof(msgCarSpeed), "Car speed %0.3f in m/s", carSpeed);
    connection.publish(mqtt_topic_connection_motor_runs,msgCarSpeed);
*/

    unsigned long currentTime = millis();
    if (currentTime - startTime < 13000) {
            motor.forward(drivePower / 100.0f);
            motor.update();
        } else {
            motor.stop();
            motor.update();
            //motorShouldRun = false;  // no longer run forward
    }

    // Example telemetry publishing
    float carSpeed = encoder.getWheelLinearVelocity();
    connection.check_connection();
    char msgCarSpeed[50];
    snprintf(msgCarSpeed, sizeof(msgCarSpeed), "Car speed %0.3f in m/s", carSpeed);
    connection.publish(mqtt_topic_connection_motor_runs, msgCarSpeed);


    /*connection.check_connection();

    // Set motor power
    motor.forward(drivePower / 100.0f);
    motor.update();

    unsigned long now = millis();

    // --- Sample velocity at controlled intervals ---
    if (now - lastVelocitySample >= VELOCITY_SAMPLE_INTERVAL_MS) {

        // Get velocities directly from EncoderCarVelocity
        float motorAngularVel = encoder.getMotorAngularVelocity();  // rad/s
        float wheelAngularVel = encoder.getWheelAngularVelocity();  // rad/s
        float carLinearVel    = encoder.getWheelLinearVelocity();   // m/s

        // Store valid velocity for distance integration
        if (carLinearVel >= 0) {
            lastValidVelocity = carLinearVel;
        }

        // Publish/print velocity info
        char msgVel[150];
        snprintf(msgVel, sizeof(msgVel), 
                 "Motor: %.2f rad/s | Wheel: %.2f rad/s | Car: %.3f m/s",
                 motorAngularVel, wheelAngularVel, carLinearVel);
        connection.publish(mqtt_topic_connection_motor_runs, msgVel);
        Serial.println(msgVel);

        lastVelocitySample = now;
    }

    // --- Update distance estimate ---
    if (now - lastDistanceSample >= DISTANCE_UPDATE_INTERVAL_MS) {
        unsigned long dt = now - lastDistanceSample;
        float dt_seconds = dt / 1000.0f;

        // Integrate velocity to get distance
        estimatedDistance += lastValidVelocity * dt_seconds;

        // Publish/print distance
        char msgDist[100];
        snprintf(msgDist, sizeof(msgDist), 
                 "Distance: %.3f m (target: %.1f m)", 
                 estimatedDistance, TARGET_DISTANCE_M);
        connection.publish(mqtt_topic_connection_motor_runs, msgDist);
        Serial.println(msgDist);

        // Check if target distance reached
        if (estimatedDistance >= TARGET_DISTANCE_M) {
            // Emergency stop
            motor.stop();
            motor.update();
            digitalWrite(LED_BUILTIN, HIGH);

            char msgStop[100];
            snprintf(msgStop, sizeof(msgStop), "STOPPED after %.3f meters!", estimatedDistance);
            connection.publish(mqtt_topic_connection_motor_runs, msgStop);
            Serial.println(msgStop);

            while (true) {
                delay(5000);
            }
        }

        lastDistanceSample = now;
    }*/
}
