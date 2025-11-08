/**
 * @file pid_velocity
 * @brief Test to 
 */
#include "test_common_esp2.h"
#include "EncoderCarVelocity.h"
#include "motor_pid.h"
const char* mqtt_topic_connection_pid_velocity = "slamaleykoum77/print";

//Wheel & motor parameters
const float GEAR_RATIO = 10.0f; //it takes the motor 10 full rotations to make the wheel turn a full rotation
const float WHEEL_RADIUS_M = 0.045f; //wheel radius in meters
const float TARGET_MOTOR_DEGREES = 360.0f *30; //target to reach to make the motor stop -> *# of rotations


static unsigned long lastVelocity = 0; //previous velocity

static int drivePower = 20; 

const int ANGLE_INTERVAL_MS = 5; //if odd go back to 300 but one full motor revolution takes around 370 milliseconds
// 300; // velocity sample interval in milliseconds



const float desiredVelocity = 0.4f; // 0.3 m/s (example target speed)




MotorPID pid(2.0f, 0.0f, 0.00f);
static EncoderCarVelocity carVelocity(&encoder);


void setup_pid_velocity() {
    
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
        connection.publish(mqtt_topic_connection_pid_velocity, msgMotor);



        while (true);
    }
    
    //Initialize encoder
    if (!encoder.begin()) {
        char msgEncoder[50];
        snprintf(msgEncoder, sizeof(msgEncoder), "Encoder init failed!");
        connection.publish(mqtt_topic_connection_pid_velocity,msgEncoder);
        while (true);
    }

    //check again the connection before sending the message to see if the setup succeeded
    connection.check_connection();
    char msgSetup[50];
    snprintf(msgSetup, sizeof(msgSetup), "Setup success!");
    connection.publish(mqtt_topic_connection_pid_velocity,msgSetup);

    
}


// test loop
void loop_pid_velocity() {
    
    //make sure connection is established
    connection.check_connection();

    
    static unsigned long lastTime = 0; // persists across loops
    unsigned long now = millis();      // using milliseconds

    // Initialize first loop
    if (lastTime == 0) {
        lastTime = now;
        return;
    }

    // dt in seconds - divide by 1000 since millis() returns milliseconds
    float dt = (now - lastTime) / 1000.0f;  // ← FIXED: divide by 1000, not 1000000
    lastTime = now;

    // ignore extremely small dt
    //if (dt <= 0.001f) return;  // ← FIXED: 1ms minimum instead of 0.00001




    char msgdt[80];
    snprintf(msgdt, sizeof(msgdt), "The dt = %.6f  s", dt);
    connection.publish(mqtt_topic_connection_pid_velocity,msgdt);
    Serial.println(msgdt);



    // 1️⃣ Update encoder data and compute current car velocity
    
    if (now - lastVelocity >= ANGLE_INTERVAL_MS) {
        //carVelocity.update(now);
        lastVelocity = now;
    }
    float currentLinearVel = carVelocity.getWheelLinearVelocity();

    // 2️⃣ Get your desired velocity (manual test or pure pursuit output)
    Velocity targetVel = { .v_linear = desiredVelocity, .v_angular = 0.0f };
    Velocity currentVel = { .v_linear = currentLinearVel, .v_angular = 0.0f };

    // 3️⃣ Compute PID PWM output
    MotorOutputs pwm = pid.compute_pwm_output(targetVel, currentVel, dt);

    // 4️⃣ Apply PWM to the motor
    float safe_signal = constrain(pwm, -0.25f, 0.25f); // your known safe range
    
    motor.setTargetPercent(safe_signal);
    

    motor.update();

    // 5️⃣ (Optional) Debug info
    
    char msgTarget[180];
    snprintf(msgTarget, sizeof(msgTarget),"Target=%.3f m/s | Current=%.3f m/s | PWM=%d\n",
                  targetVel.v_linear, currentLinearVel, pwm);
    connection.publish(mqtt_topic_connection_pid_velocity,msgTarget);
    Serial.println(msgTarget);

    //float wheelDeg = carVelocity.getUnwrappedAngle() / GEAR_RATIO;
    if (targetVel.v_linear == currentLinearVel) {
        delay(6000);// to check if the speed stay constant let it continnue for 6 seconds
        for(int i =0; i <3; i++){
            motor.emergencyStop();
            motor.update();
            Serial.println("Target reached — stopping motor.");
        }
        char msgStop[80];
        snprintf(msgStop, sizeof(msgStop), "Stopped after since it got to desired speed");
        connection.publish(mqtt_topic_connection_pid_velocity,msgStop);
        Serial.println(msgStop);


        char msgStopVel[160];
        snprintf(msgStopVel, sizeof(msgStopVel), 
        "For motor STOP : angular_v_motor=%.2f rad/s | angular_v_wheel=%.2f rad/s | v_car=%.3f m/s",
        carVelocity.getMotorAngularVelocity(), 
        carVelocity.getWheelAngularVelocity(), 
        carVelocity.getWheelLinearVelocity());
        connection.publish(mqtt_topic_connection_pid_velocity,msgStopVel);

        delay(4000);
    }  
    
    



}