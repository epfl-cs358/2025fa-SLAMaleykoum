/**
 * @file motor_runs
 * @brief Test to make the motor run simply
 */
#include "test_common_esp2.h"
#include "EncoderCarVelocity.h"

const char* mqtt_topic_connection_motor_runs = "slamaleykoum77/print";

//Wheel & motor parameters
const float GEAR_RATIO = 10.0f; //it takes the motor 10 full rotations to make the wheel turn a full rotation
const float WHEEL_RADIUS_M = 0.045f; //wheel radius in meters
const float TARGET_MOTOR_DEGREES = 360.0f *30; //target to reach to make the motor stop -> *# of rotations


static unsigned long lastVelocity = 0; //previous velocity

static int drivePower = 19; 

const int ANGLE_INTERVAL_MS = 5;//5; //if odd go back to 300 but one full motor revolution takes around 370 milliseconds
// 300; // velocity sample interval in milliseconds
const float TARGET_DISTANCE_M = 1.0f;

static float estimatedDistance = 0.0f;
static unsigned long lastSampleTime = 0;


static EncoderCarVelocity carVelocity(&encoder);


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
    if (!encoder.begin()) {
        char msgEncoder[50];
        snprintf(msgEncoder, sizeof(msgEncoder), "Encoder init failed!");
        connection.publish(mqtt_topic_connection_motor_runs,msgEncoder);
        while (true);
    }

    //check again the connection before sending the message to see if the setup succeeded
    connection.check_connection();
    char msgSetup[50];
    snprintf(msgSetup, sizeof(msgSetup), "Setup success!");
    connection.publish(mqtt_topic_connection_motor_runs,msgSetup);

    
}

// test loop
void loop_motor_runs() {
    connection.check_connection();

    motor.setTargetPercent(-drivePower / 100.0f);
    motor.update();
    
    unsigned long now = millis();
    
    // Sample at your desired interval
    if (now - lastVelocity >= ANGLE_INTERVAL_MS) {
        //carVelocity.update(now);  // Only call update() here at controlled rate
        
        float motorAngularVel = carVelocity.getMotorAngularVelocity();
        float wheelAngularVel = carVelocity.getWheelAngularVelocity();
        float carLinearVel    = carVelocity.getWheelLinearVelocity();

        char msgVel[120];
        snprintf(msgVel, sizeof(msgVel), 
                "angular_v_motor=%.2f rad/s | angular_v_wheel=%.2f rad/s | v_car=%.3f m/s",
                motorAngularVel, wheelAngularVel, carLinearVel);
        connection.publish(mqtt_topic_connection_motor_runs, msgVel);
        Serial.println(msgVel);

        /*float wheelDeg = carVelocity.getUnwrappedAngle() / GEAR_RATIO;
        if (wheelDeg >= TARGET_MOTOR_DEGREES) {
            for(int i = 0; i < 3; i++) {
                motor.emergencyStop();
                motor.update();
            }
            
            char msgStop[80];
            snprintf(msgStop, sizeof(msgStop), "Stopped after %.1f° wheel rotation", wheelDeg);
            connection.publish(mqtt_topic_connection_motor_runs, msgStop);
            Serial.println(msgStop);

            while (true) {
                delay(5000);
            }
        }*/
        unsigned long dt = now - lastSampleTime;
        float dt_seconds = dt / 1000.0f;

        float carLinearVelo = carVelocity.getWheelLinearVelocity();  // m/s
        estimatedDistance += carLinearVel * dt_seconds;

        lastSampleTime = now;

        if (estimatedDistance >= TARGET_DISTANCE_M) {
            for (int i = 0; i < 3; i++) {
                motor.emergencyStop();
                motor.update();
            }

            char msgStop[80];
            snprintf(msgStop, sizeof(msgStop), "Stopped after approx %.2f meters", estimatedDistance);
            connection.publish(mqtt_topic_connection_motor_runs, msgStop);
            Serial.println(msgStop);

            while (true) delay(5000);
        }

       

        lastVelocity = now;
    }
    
    //make sure connection is established
    /*connection.check_connection();


    //Make the motor go forward 
    motor.setTargetPercent(drivePower / 100.0f);
    //motor.forward(drivePower / 100.0f);
    //motor.backward(drivePower/100.0f);
    motor.update();
    
    unsigned long now = millis();
    carVelocity.update(now);

    //Sample every given ms interval 
    if (now - lastVelocity >= ANGLE_INTERVAL_MS) {
        float motorAngularVel = carVelocity.getMotorAngularVelocity();
        float wheelAngularVel = carVelocity.getWheelAngularVelocity();
        float carLinearVel    = carVelocity.getWheelLinearVelocity();

        //Message to see if the values are coherent
        char msgVel[120];
        snprintf(msgVel, sizeof(msgVel), 
                "angular_v_motor=%.2f rad/s | angular_v_wheel=%.2f rad/s | v_car=%.3f m/s",
                motorAngularVel, wheelAngularVel, carLinearVel);
        connection.publish(mqtt_topic_connection_motor_runs,msgVel);
        Serial.println(msgVel);

        //Stop car after desired target
        float wheelDeg = carVelocity.getUnwrappedAngle() / GEAR_RATIO;
        if (wheelDeg >= TARGET_MOTOR_DEGREES) {
            for(int i  = 0; i<3 ; i++){
                motor.emergencyStop();
                //motor.stop();
                motor.update();
            }       
            char msgStop[80];
            snprintf(msgStop, sizeof(msgStop), "Stopped after %.1f° wheel rotation", wheelDeg);
            connection.publish(mqtt_topic_connection_motor_runs,msgStop);
            Serial.println(msgStop);

            
            char msgStopVel[160];
            snprintf(msgStopVel, sizeof(msgStopVel), 
            "For motor STOP : angular_v_motor=%.2f rad/s | angular_v_wheel=%.2f rad/s | v_car=%.3f m/s",
            motorAngularVel, wheelAngularVel, carLinearVel);
            connection.publish(mqtt_topic_connection_motor_runs,msgStopVel);

            //completly stop with delay
            while (true) {
                delay(5000);
            }
        }

        lastVelocity = now;
    }*/
/*
     // make sure connection is established
    connection.check_connection();

    unsigned long now = millis();

    // Update encoder & compute velocity internally
    carVelocity.update(now);

    // Make motor go forward
    motor.forward(drivePower / 100.0f);
    motor.update();

    // Only print if the velocity actually updated
    static float lastPrintedVelocity = 0.0f;
    float carLinearVel = carVelocity.getWheelLinearVelocity();
    if (carLinearVel != lastPrintedVelocity) {
        float motorAngularVel = carVelocity.getMotorAngularVelocity();
        float wheelAngularVel = carVelocity.getWheelAngularVelocity();

        char msgVel[120];
        snprintf(msgVel, sizeof(msgVel), 
                "angular_v_motor=%.2f rad/s | angular_v_wheel=%.2f rad/s | v_car=%.3f m/s",
                motorAngularVel, wheelAngularVel, carLinearVel);
        connection.publish(mqtt_topic_connection_motor_runs,msgVel);
        Serial.println(msgVel);

        lastPrintedVelocity = carLinearVel;
    }

    // Stop after target wheel rotation
    float wheelDeg = carVelocity.getUnwrappedAngle() / GEAR_RATIO;
    if (wheelDeg >= TARGET_MOTOR_DEGREES) {
        for(int i  = 0; i<3 ; i++){
            motor.stop();
            motor.update();
        }       
        char msgStop[80];
        snprintf(msgStop, sizeof(msgStop), "Stopped after %.1f° wheel rotation", wheelDeg);
        connection.publish(mqtt_topic_connection_motor_runs,msgStop);
        Serial.println(msgStop);

        while (true) delay(5000);
    }*/
}