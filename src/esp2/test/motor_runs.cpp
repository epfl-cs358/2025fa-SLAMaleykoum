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

static int drivePower = 20; 

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

// // test loop
// void loop_motor_runs() {
//     connection.check_connection();

//     motor.setTargetPercent(drivePower / 100.0f);
//     motor.update();
    
//     unsigned long now = millis();
    
//     // Sample at your desired interval
//     if (now - lastVelocity >= ANGLE_INTERVAL_MS) {
//         //carVelocity.update(now);  // Only call update() here at controlled rate
        
//         float motorAngularVel = carVelocity.getMotorAngularVelocity();
//         float wheelAngularVel = carVelocity.getWheelAngularVelocity();
//         float carLinearVel    = carVelocity.getWheelLinearVelocity();

//         char msgVel[120];
//         snprintf(msgVel, sizeof(msgVel), 
//                 "angular_v_motor=%.2f rad/s | angular_v_wheel=%.2f rad/s | v_car=%.3f m/s",
//                 motorAngularVel, wheelAngularVel, carLinearVel);
//         connection.publish(mqtt_topic_connection_motor_runs, msgVel);
//         Serial.println(msgVel);

//         /*float wheelDeg = carVelocity.getUnwrappedAngle() / GEAR_RATIO;
//         if (wheelDeg >= TARGET_MOTOR_DEGREES) {
//             for(int i = 0; i < 3; i++) {
//                 motor.emergencyStop();
//                 motor.update();
//             }
            
//             char msgStop[80];
//             snprintf(msgStop, sizeof(msgStop), "Stopped after %.1f° wheel rotation", wheelDeg);
//             connection.publish(mqtt_topic_connection_motor_runs, msgStop);
//             Serial.println(msgStop);

//             while (true) {
//                 delay(5000);
//             }
//         }*/
//         unsigned long dt = now - lastSampleTime;
//         float dt_seconds = dt / 1000.0f;

//         float carLinearVelo = carVelocity.getWheelLinearVelocity();  // m/s
//         estimatedDistance += carLinearVel * dt_seconds;

//         float timeDT = carVelocity.getTimeElapsed();
//         char msgTimeElapsed[80];
//         snprintf(msgTimeElapsed, sizeof(msgTimeElapsed), "STime elapsed %.6f in ms = ", timeDT);
//         connection.publish(mqtt_topic_connection_motor_runs, msgTimeElapsed);
//         Serial.println(msgTimeElapsed);

//         float Angle1 = carVelocity.getAngle1();
//         char msgAngle1[80];
//         snprintf(msgAngle1, sizeof(msgAngle1), "Angle 1 =  %.6f  ", Angle1);
//         connection.publish(mqtt_topic_connection_motor_runs, msgAngle1);
//         Serial.println(msgAngle1);

//         float Angle2 = carVelocity.getAngle2();
//         char msgAngle2[80];
//         snprintf(msgAngle2, sizeof(msgAngle2), "Angle 2  = %.6f  ", Angle2);
//         connection.publish(mqtt_topic_connection_motor_runs, msgAngle2);
//         Serial.println(msgAngle2);

//         //float carSpeed = carVelocity.getWheelLinearVelocity();
//         char msgSpeed[80];
//         snprintf(msgSpeed, sizeof(msgSpeed), "car speed  = %.6f  ", carLinearVelo);
//         connection.publish(mqtt_topic_connection_motor_runs, msgSpeed);
//         Serial.println(msgSpeed);

//         lastSampleTime = now;
//         /*
//         if (estimatedDistance >= TARGET_DISTANCE_M) {
//             for (int i = 0; i < 3; i++) {
//                 motor.emergencyStop();
//                 motor.update();
//             }

//             char msgStop[80];
//             snprintf(msgStop, sizeof(msgStop), "Stopped after approx %.2f meters", estimatedDistance);
//             connection.publish(mqtt_topic_connection_motor_runs, msgStop);
//             Serial.println(msgStop);

//             while (true) delay(5000);
//         }*/

       

//         lastVelocity = now;
//     }
    
//     //make sure connection is established
//     /*connection.check_connection();


//     //Make the motor go forward 
//     motor.setTargetPercent(drivePower / 100.0f);
//     //motor.forward(drivePower / 100.0f);
//     //motor.backward(drivePower/100.0f);
//     motor.update();
    
//     unsigned long now = millis();
//     carVelocity.update(now);

//     //Sample every given ms interval 
//     if (now - lastVelocity >= ANGLE_INTERVAL_MS) {
//         float motorAngularVel = carVelocity.getMotorAngularVelocity();
//         float wheelAngularVel = carVelocity.getWheelAngularVelocity();
//         float carLinearVel    = carVelocity.getWheelLinearVelocity();

//         //Message to see if the values are coherent
//         char msgVel[120];
//         snprintf(msgVel, sizeof(msgVel), 
//                 "angular_v_motor=%.2f rad/s | angular_v_wheel=%.2f rad/s | v_car=%.3f m/s",
//                 motorAngularVel, wheelAngularVel, carLinearVel);
//         connection.publish(mqtt_topic_connection_motor_runs,msgVel);
//         Serial.println(msgVel);

//         //Stop car after desired target
//         float wheelDeg = carVelocity.getUnwrappedAngle() / GEAR_RATIO;
//         if (wheelDeg >= TARGET_MOTOR_DEGREES) {
//             for(int i  = 0; i<3 ; i++){
//                 motor.emergencyStop();
//                 //motor.stop();
//                 motor.update();
//             }       
//             char msgStop[80];
//             snprintf(msgStop, sizeof(msgStop), "Stopped after %.1f° wheel rotation", wheelDeg);
//             connection.publish(mqtt_topic_connection_motor_runs,msgStop);
//             Serial.println(msgStop);

            
//             char msgStopVel[160];
//             snprintf(msgStopVel, sizeof(msgStopVel), 
//             "For motor STOP : angular_v_motor=%.2f rad/s | angular_v_wheel=%.2f rad/s | v_car=%.3f m/s",
//             motorAngularVel, wheelAngularVel, carLinearVel);
//             connection.publish(mqtt_topic_connection_motor_runs,msgStopVel);

//             //completly stop with delay
//             while (true) {
//                 delay(5000);
//             }
//         }

//         lastVelocity = now;
//     }*/
// /*
//      // make sure connection is established
//     connection.check_connection();

//     unsigned long now = millis();

//     // Update encoder & compute velocity internally
//     carVelocity.update(now);

//     // Make motor go forward
//     motor.forward(drivePower / 100.0f);
//     motor.update();

//     // Only print if the velocity actually updated
//     static float lastPrintedVelocity = 0.0f;
//     float carLinearVel = carVelocity.getWheelLinearVelocity();
//     if (carLinearVel != lastPrintedVelocity) {
//         float motorAngularVel = carVelocity.getMotorAngularVelocity();
//         float wheelAngularVel = carVelocity.getWheelAngularVelocity();

//         char msgVel[120];
//         snprintf(msgVel, sizeof(msgVel), 
//                 "angular_v_motor=%.2f rad/s | angular_v_wheel=%.2f rad/s | v_car=%.3f m/s",
//                 motorAngularVel, wheelAngularVel, carLinearVel);
//         connection.publish(mqtt_topic_connection_motor_runs,msgVel);
//         Serial.println(msgVel);

//         lastPrintedVelocity = carLinearVel;
//     }

//     // Stop after target wheel rotation
//     float wheelDeg = carVelocity.getUnwrappedAngle() / GEAR_RATIO;
//     if (wheelDeg >= TARGET_MOTOR_DEGREES) {
//         for(int i  = 0; i<3 ; i++){
//             motor.stop();
//             motor.update();
//         }       
//         char msgStop[80];
//         snprintf(msgStop, sizeof(msgStop), "Stopped after %.1f° wheel rotation", wheelDeg);
//         connection.publish(mqtt_topic_connection_motor_runs,msgStop);
//         Serial.println(msgStop);

//         while (true) delay(5000);
//     }*/
// }

/*
void loop_motor_runs() {
    connection.check_connection();

    // Set motor power
    motor.forward(drivePower/100.0f);
    //motor.setTargetPercent(drivePower / 100.0f);
    motor.update();
    
    unsigned long now = millis();
    
    // Sample velocity at controlled intervals
    if (now - lastVelocitySample >= VELOCITY_SAMPLE_INTERVAL_MS) {
        float motorVel = encoder.getMotorAngularVelocity();
    float wheelVel = encoder.getWheelLinearVelocity();
    int32_t pos = encoder.getCumulativePosition();

    Serial.print("Motor rad/s: "); Serial.println(motorVel);
    Serial.print("Wheel m/s: "); Serial.println(wheelVel);
    Serial.print("Cumulative: "); Serial.println(pos);

    delay(50); // 20Hz update
        float motorAngularVel = carVelocity.getAverageMotorAngularVelocity();
        float wheelAngularVel = carVelocity.getWheelAngularVelocity();
        float carLinearVel = carVelocity.getWheelLinearVelocity();//getWheelLinearVelocity();//getWheelLinearVelocity();
        
        // Store valid velocity for distance calculation
        if (carLinearVel >= 0) {  // Simple validity check
            lastValidVelocity = carLinearVel;
        }

        // Print velocity data
        char msgVel[150];
        snprintf(msgVel, sizeof(msgVel), 
                "Motor: %.2f rad/s | Wheel: %.2f rad/s | Car: %.3f m/s",
                motorAngularVel, wheelAngularVel, carLinearVel);
        connection.publish(mqtt_topic_connection_motor_runs, msgVel);
        Serial.println(msgVel);
    
        // Debug info
        /*float timeDT = carVelocity.getTimeElapsed();
        float angle1 = carVelocity.getAngle1();
        float angle2 = carVelocity.getAngle2();
        
        char msgDebug[150];
        snprintf(msgDebug, sizeof(msgDebug), 
                "dt: %.2f ms | Angle1: %.2f° | Angle2: %.2f° | Delta: %.2f°",
                timeDT / 1000.0f, angle1, angle2, angle2 - angle1);
        connection.publish(mqtt_topic_connection_motor_runs, msgDebug);
        Serial.println(msgDebug);*/
/*
        lastVelocitySample = now;
    }
    
    // Update distance estimate less frequently to reduce noise
    if (now - lastDistanceSample >= DISTANCE_UPDATE_INTERVAL_MS) {
        unsigned long dt = now - lastDistanceSample;
        float dt_seconds = dt / 1000.0f;
        
        // Integrate velocity to get distance
        estimatedDistance += lastValidVelocity * dt_seconds;
        
        char msgDist[80];
        snprintf(msgDist, sizeof(msgDist), "Distance: %.3f m (target: %.1f m)", 
                estimatedDistance, TARGET_DISTANCE_M);
        connection.publish(mqtt_topic_connection_motor_runs, msgDist);
        Serial.println(msgDist);
        
        // Check if target distance reached
        if (estimatedDistance >= TARGET_DISTANCE_M) {
            // Emergency stop
            for (int i = 0; i < 3; i++) {
                motor.stop();
                //motor.emergencyStop();
                motor.update();
                digitalWrite(LED_BUILTIN, HIGH);
                
            }

            char msgStop[80];
            snprintf(msgStop, sizeof(msgStop), "STOPPED after %.3f meters!", estimatedDistance);
            connection.publish(mqtt_topic_connection_motor_runs, msgStop);
            Serial.println(msgStop);

            while (true) {
                delay(5000);
            }
        }
        
        lastDistanceSample = now;
    }
}*/

void loop_motor_runs() {
    
    motor.forward(drivePower/100.0f);
    motor.update();
    /*uint16_t angleRaw = encoder.as5600.readAngle(); // 0..4095
    float angleDeg = angleRaw * AS5600_RAW_TO_DEGREES; // convert to degrees

     connection.check_connection();
    char msgSetup[50];
    snprintf(msgSetup, sizeof(msgSetup), "Angle is in degrees %0.2f", angleDeg);
    connection.publish(mqtt_topic_connection_motor_runs,msgSetup);*/

    //delay(3000);

    
    float velocityAngle = encoder.getFilteredAngularVelocity();//getFilteredMotorAngularVelocity(); // 0..4095

    char msgSpeed[50];
    snprintf(msgSpeed, sizeof(msgSpeed), "Angular speed %0.3f", velocityAngle);
    connection.publish(mqtt_topic_connection_motor_runs,msgSpeed);

    
    float carSpeed = encoder.getWheelLinearVelocity(); // 0..4095

    char msgCarSpeed[50];
    snprintf(msgCarSpeed, sizeof(msgCarSpeed), "Car speed %0.3f in m/s", carSpeed);
    connection.publish(mqtt_topic_connection_motor_runs,msgCarSpeed);



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
