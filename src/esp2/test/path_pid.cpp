/**
 * @file pid_velocity
 * @brief Fixed PID velocity control test
 */
#include "test_common_esp2.h"
#include "EncoderCarVelocity.h"
#include "motor_pid.h"

const char* mqtt_topic_connection_path_pid = "slamaleykoum77/print";

// ===== Motor and control parameters =====
const float GEAR_RATIO1 = 10.0f;
const float WHEEL_RADIUS_M = 0.0495f;
const float TARGET_SEGMENT_1_M = 1.5f;
const float TARGET_SEGMENT_2_M = 0.1f;
const float DESIRED_VELOCITY = 0.25f; // target speed in m/s
const float VELOCITY_TOLERANCE = 0.05f;
const float PWM_SAFE_MIN = 0.0f;
const float PWM_SAFE_MAX = 0.24f;

// ===== Servo angles =====
const int STRAIGHT_ANGLE = 90;
const int LEFT_TURN_ANGLE = 20;

// ===== Timing =====
const int VELOCITY_UPDATE_INTERVAL_MS = 5;

// ===== State machine =====
enum DriveState { DRIVE_FIRST_SEGMENT, TURN_LEFT, DRIVE_SECOND_SEGMENT, STOPPED };
static DriveState state = DRIVE_FIRST_SEGMENT;

// ===== Objects =====
//static EncoderCarVelocity carVelocity(&encoder);
//static EncoderCarVelocity carVelocity(&encoder);
//EncoderCarVelocity encoder;
//MotorPID pid(0.7f, 0.0f, 0.0f);

// ===== Tracking variables =====
static float estimatedDistance = 0.0f;
static float lastVelocity = 0.0f;
static unsigned long lastTime = 0;
static unsigned long lastVelocityUpdate = 0;
void setup_path_pid() {
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
        connection.publish(mqtt_topic_connection_path_pid, msgMotor);
        while (true);
    }
    
    // Initialize encoder
    encoder.begin();
    /*if (!encoder.begin()) {
        char msgEncoder[50];
        snprintf(msgEncoder, sizeof(msgEncoder), "Encoder init failed!");
        connection.publish(mqtt_topic_connection_path_pid, msgEncoder);
        while (true);
    }*/

    if (!servo_dir.begin()) {
        Serial.println("Servo init failed!");
        while (true);
    }
    servo_dir.setAngle(STRAIGHT_ANGLE);


    // Confirm setup success
    connection.check_connection();
    char msgSetup[50];
    snprintf(msgSetup, sizeof(msgSetup), "Setup success!");
    connection.publish(mqtt_topic_connection_path_pid, msgSetup);

    lastTime = millis();
}

void loop_path_pid() {
    connection.check_connection();
    unsigned long now = millis();
    float dt = (now - lastTime)/1000.0f;
    lastTime = now;

    // --- Update velocity estimation periodically ---
    if (now - lastVelocityUpdate >= VELOCITY_UPDATE_INTERVAL_MS) {
        encoder.getWheelLinearVelocity();
        //carVelocity.getWheelLinearVelocity();
        //carVelocity.getMotorAngularVelocity(); // refreshes internal state
        lastVelocityUpdate = now;
    }
    float currentVel = encoder.getWheelLinearVelocity();

    // --- Integrate distance ---
    estimatedDistance += currentVel * dt;

    // --- PID control ---
    Velocity target = { .v_linear = DESIRED_VELOCITY, .v_angular = 0.0f };
    Velocity current = { .v_linear = currentVel, .v_angular = 0.0f };
    float pwm_output = pid.compute_pwm_output(target, current, dt);
    
    
    char msgDT[50];
    snprintf(msgDT, sizeof(msgDT), "pid given dt = %0.2f", dt);
    //connection.publish(mqtt_topic_connection_path_pid, msgDT);

    char msgComputedPWM[50];
    snprintf(msgComputedPWM, sizeof(msgComputedPWM), "Computed pwm by pid = %0.2f", pwm_output);
    //connection.publish(mqtt_topic_connection_path_pid, msgComputedPWM);

    float safe_pwm = constrain(pwm_output, PWM_SAFE_MIN, PWM_SAFE_MAX);

    // --- Execute state machine ---
    switch (state) {
        case DRIVE_FIRST_SEGMENT:
            servo_dir.setAngle(STRAIGHT_ANGLE);
            motor.forward(safe_pwm);
            motor.update();
            if (estimatedDistance >= TARGET_SEGMENT_1_M) {
                motor.stop();
                motor.update();
                delay(500);
                state = TURN_LEFT;
                estimatedDistance = 0.0f;

                char msgReachTarget1[50];
                snprintf(msgReachTarget1, sizeof(msgReachTarget1), "Reached 1.5m, turning left");
                connection.publish(mqtt_topic_connection_path_pid, msgReachTarget1);
            }
            break;

        case TURN_LEFT: {
            static unsigned long turnStartTime = 0;
            static bool turning = false;

                if (!turning) {
                turning = true;
                turnStartTime = now;
                estimatedDistance = 0.0f;
                servo_dir.setAngle(LEFT_TURN_ANGLE);
                connection.publish(mqtt_topic_connection_path_pid, "Starting left turn with forward motion");
            }

            // Drive forward slowly while turning left
            float turn_pwm = constrain(safe_pwm*0.8 , PWM_SAFE_MIN, 0.24f); // slightly slower * 0.8f
            motor.forward(turn_pwm);
            //motor.forward(max(turn_pwm, 0.25f));

            motor.update();

            // Integrate forward motion
            estimatedDistance += currentVel * dt;

            // End condition: after 1.5 s or small distance (~0.5 m)
            if ((estimatedDistance >= 1.0f)) { //(now - turnStartTime > 7500) || 
                //motor.emergencyStop();
                //motor.update();
                servo_dir.setAngle(STRAIGHT_ANGLE);
                delay(400);
                estimatedDistance = 0.0f;
                turning = false;
                state = DRIVE_SECOND_SEGMENT;
                connection.publish(mqtt_topic_connection_path_pid, "Turn complete, entering second segment");
            }
            break;
        }

        case DRIVE_SECOND_SEGMENT: {
        static bool initialized = false;

    if (!initialized) {
        initialized = true;
        estimatedDistance = 0.0f;
        pid.reset(); // reset integral/derivative state if supported
        connection.publish(mqtt_topic_connection_path_pid, "Starting second straight segment");
    }

    // --- PID control for second segment ---
    Velocity target2 = { .v_linear = DESIRED_VELOCITY, .v_angular = 0.0f };
    Velocity current2 = { .v_linear = currentVel, .v_angular = 0.0f };
    float pwm_output2 = pid.compute_pwm_output(target2, current2, dt);
    float safe_pwm2   = constrain(pwm_output2, PWM_SAFE_MIN, PWM_SAFE_MAX);

    // Optional boost at very start to overcome static friction
    if (estimatedDistance < 0.05f) {
        motor.forward(PWM_SAFE_MAX);
    } else {
        motor.forward(safe_pwm2);  // use PID output
    }

    motor.update();
    estimatedDistance += currentVel * dt;

    if (estimatedDistance >= TARGET_SEGMENT_2_M) {
        motor.stop();
        motor.update();
        servo_dir.setAngle(STRAIGHT_ANGLE);
        digitalWrite(LED_BUILTIN, HIGH);
        connection.publish(mqtt_topic_connection_path_pid, "Reached final segment, stopping");

        initialized = false;
        state = STOPPED;
    }
    break;
}



        case STOPPED:
            motor.stop();
            motor.update();
            digitalWrite(LED_BUILTIN, HIGH);
            break;
    }

    // --- Debug output ---
    char msg[180];
    snprintf(msg, sizeof(msg),
             "State=%d | v=%.3f m/s | PWM=%.3f | Dist=%.3f m",
             state, currentVel, safe_pwm, estimatedDistance);
    connection.publish(mqtt_topic_connection_path_pid, msg);
    Serial.println(msg);

    //delay(10);
}