/**
 * @file pid_velocity
 * @brief Fixed PID velocity control test
 */
#include "test_common_esp2.h"
#include "EncoderCarVelocity.h"
#include "motor_pid.h"

const char* mqtt_topic_connection_pid_velocity = "slamaleykoum77/print";

// Wheel & motor parameters
//const float GEAR_RATIO = 10.0f;
const float WHEEL_RADIUS_M = 0.045f;
const float TARGET_MOTOR_DEGREES = 360.0f * 30;

// Timing variables
static unsigned long lastVelocityUpdate = 0;
const int VELOCITY_UPDATE_INTERVAL_MS = 10; // Update velocity every 50ms

// Control parameters
const float DESIRED_VELOCITY = 1.5f; // m/s target speed
const float VELOCITY_TOLERANCE = 0.02f; // ±2 cm/s tolerance for "reached"
const float PWM_SAFE_MIN = 0.17f;
const float PWM_SAFE_MAX = 0.25f;

// PID controller and velocity estimator
//MotorPID pid(2.0f, 0.0f, 0.00f);

void setup_pid_velocity() {
    Serial.begin(115200);
    delay(2000);

    // Wifi setup
    connection.setupWifi();

    // Light setup
    pinMode(LED_BUILTIN, OUTPUT);

    // I2C setup for IMU and Encoder
    i2cMutexInit();

    // Wifi connection 
    connection.check_connection();
    
    // Initialize motor
    if (!motor.begin()) {
        char msgMotor[50];
        snprintf(msgMotor, sizeof(msgMotor), "Motor init failed!");
        connection.publish(mqtt_topic_connection_pid_velocity, msgMotor);
        while (true);
    }
    
    // Initialize encoder
    encoder.begin();

    // Confirm setup success
    connection.check_connection();
    char msgSetup[50];
    snprintf(msgSetup, sizeof(msgSetup), "Setup success!");
    connection.publish(mqtt_topic_connection_pid_velocity, msgSetup);
}

void loop_pid_velocity() {
    // Ensure connection is established
    connection.check_connection();

    static unsigned long lastPIDTime = 0;
    static unsigned long targetReachedTime = 0;
    static bool velocityReached = false;
    
    unsigned long now = millis();

    // Initialize on first loop
    if (lastPIDTime == 0) {
        lastPIDTime = now;
        lastVelocityUpdate = now;
        return;
    }

    // Calculate dt for PID (in seconds)
    float dt = (now - lastPIDTime) / 1000.0f;
    lastPIDTime = now;

    // Skip if dt is too small (shouldn't happen with millis)
    if (dt <= 0.001f) return;

    // ========== 1️⃣ Update velocity at fixed intervals ==========
    if (now - lastVelocityUpdate >= VELOCITY_UPDATE_INTERVAL_MS) {
        encoder.getMotorAngularVelocity();  // Updates internal angle measurements
        lastVelocityUpdate = now;
    }
    
    float currentLinearVel = encoder.getWheelLinearVelocity();

    // ========== 2️⃣ Set target velocity ==========
    Velocity targetVel = { .v_linear = DESIRED_VELOCITY, .v_angular = 0.0f };
    Velocity currentVel = { .v_linear = currentLinearVel, .v_angular = 0.0f };

    // ========== 3️⃣ Compute PID output ==========
    float pwm_output = pid.compute_pwm_output(targetVel, currentVel, dt);

    // ========== 4️⃣ Apply safety limits and send to motor ==========
    float safe_pwm = constrain(pwm_output, PWM_SAFE_MIN, PWM_SAFE_MAX);
    motor.forward(safe_pwm);
    //motor.setTargetPercent(safe_pwm);
    motor.update();

    // ========== 5️⃣ Debug output ==========
    char msgStatus[200];
    snprintf(msgStatus, sizeof(msgStatus),
             "dt=%.3fs | Target=%.3f m/s | Current=%.3f m/s | PWM=%.3f (safe=%.3f)",
             dt, targetVel.v_linear, currentLinearVel, pwm_output, safe_pwm);
    connection.publish(mqtt_topic_connection_pid_velocity, msgStatus);
    Serial.println(msgStatus);

    // ========== 6️⃣ Check if target velocity reached ==========
    float velocityError = fabs(targetVel.v_linear - currentLinearVel);
    
    if (velocityError <= VELOCITY_TOLERANCE) {
        if (!velocityReached) {
            // Just reached target
            velocityReached = true;
            targetReachedTime = now;
            
            char msgReached[100];
            snprintf(msgReached, sizeof(msgReached), 
                     "Target velocity reached! Error: %.4f m/s", velocityError);
            connection.publish(mqtt_topic_connection_pid_velocity, msgReached);
            Serial.println(msgReached);
        }
        
        // If maintained for 6 seconds, stop
        if (now - targetReachedTime >= 6000) {
            // Stop the motor
            for (int i = 0; i < 3; i++) {
                motor.stop();
                //motor.emergencyStop();
                motor.update();
                delay(10);
            }
            
            char msgStop[100];
            snprintf(msgStop, sizeof(msgStop), 
                     "Maintained target for 6s - stopping motor");
            connection.publish(mqtt_topic_connection_pid_velocity, msgStop);
            Serial.println(msgStop);

            // Final velocity reading
            char msgFinal[180];
            snprintf(msgFinal, sizeof(msgFinal), 
                     "FINAL: motor_angular_v=%.2f rad/s | wheel_angular_v=%.2f rad/s | linear_v=%.3f m/s",
                     encoder.getMotorAngularVelocity(), 
                     encoder.getWheelAngularVelocity(), 
                     encoder.getWheelLinearVelocity());
            connection.publish(mqtt_topic_connection_pid_velocity, msgFinal);
            
            // Hold for a moment then stop completely
            delay(4000);
            while(true) {
                motor.stop();
                //motor.emergencyStop();
                motor.update();
                delay(100);
            }
        }
    } else {
        // Reset if we drop out of tolerance
        velocityReached = false;
    }
}