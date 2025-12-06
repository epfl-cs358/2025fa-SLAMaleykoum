#include "test_common_esp2.h"

#include <Arduino.h>
#include <cmath>


const char* mqtt_topic_connection_pure_pursuit = "slamaleykoum77/print";
const char* mqtt_topic_connection_location = "slamaleykoum77/purepursuit";


// === Global instances ===


bool emergency_Stop = false;
bool finished_Path = false;
float motorPowerDrive = 0.18f;  // 18%


char buf[100];


// --- Pose estimation ---

float velocityV  = 0.0f;
float pos_X = 0.0f;
float pos_Y = 0.0f;
float yawAng = 0.0f;
float newY = 0.0f;

// --- Path definition ---
float path_X[] = {0,0};//{0,0};//{-1.00, -0.95, -0.90, -0.85, -0.80, -0.75, -0.70, -0.65, -0.60, -0.55, -0.50, -0.45, -0.40, -0.35, -0.30, -0.25, -0.20, -0.15, -0.10, -0.05, 0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.00} ;
///{0.0, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};//{0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0};//{0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.5};//{0.0, 0.5, 1.0, 1.5, 2.0};
float path_Y[] ={0,-1.0};//{0,0.30};//{ 1.00,  1.00,  1.00,  1.00,  1.00,  1.00,  1.00,  1.00,  1.00,  1.00,  1.00,  1.00,  1.00,  1.00,  1.00,  1.00,  1.00,  1.00,  1.00,  1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00};//{0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.00};//{0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0};//{0.0, 0.5, 1.0, 1.5, 2.0, 2.0, 2.0};//{0.0, 0.2, 0.4, 0.6, 0.6};
const int path_Size = sizeof(path_X) / sizeof(path_X[0]);

// --- Constants ---
const float EMERGENCY_DISTANCE = 0.25f;
const float GOAL_TOLERANCE = 0.05f;  // 5 cm

// --- FreeRTOS handles ---
TaskHandle_t motorTestTask, ultrasonicTestTask, imuTestTask, poseTestTask, pursuitTestTask;

// ===============================================================
// TASK: Motor Control
// ===============================================================
void TaskTestMotor(void *pvParameters) {
    
    for (;;) {
        if (emergency_Stop || finished_Path) {
            motor.stop();
            servo_dir.setAngle(90);
        } else {
            motor.forward(motorPowerDrive);  // already normalized [0,1]
            
        }
        motor.update();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ===============================================================
// TASK: Ultrasonic Emergency Stop
// ===============================================================
void TaskTestUltrasonic(void *pvParameters) {
    
    for (;;) {
        float dist = ultrasonic.readDistance();
        if (dist > 0 && dist < EMERGENCY_DISTANCE) {
            motor.stop();
            motor.update();

            emergency_Stop = true;
            
            snprintf(buf, sizeof(buf), "⚠️ Emergency stop! Distance=%.2f m", dist);
            connection.publish(mqtt_topic_connection_location, buf);

             

        } /*else if (dist > (EMERGENCY_DISTANCE + 0.05f)) {
            emergency_Stop = false; //this will make it restart so if you want it to stay stopped keeep commented out
        }*/
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ===============================================================
// TASK: IMU Orientation (yaw extraction)
// ===============================================================


float getYawIMU(const IMUData& imu_data) {
  // Extract quaternion components
  float qw = imu_data.qw;
  float qx = imu_data.qx;
  float qy = imu_data.qy;
  float qz = imu_data.qz;

  // Yaw (z-axis rotation)
  // This is the standard formula for quaternion-to-euler conversion
  float siny_cosp = 2.0 * (qw * qz + qx * qy);
  float cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  float yaw = atan2(siny_cosp, cosy_cosp);

  return yaw; // Yaw angle in radians
}



void TaskTestIMU(void *pvParameters) {
   
    for (;;) {
        i2c_lock();
        imu.readAndUpdate();
        IMUData q = imu.data();
        i2c_unlock();

        // ✅ PRINT QUATERNIONS FIRST TO SEE IF THEY CHANGE
        snprintf(buf, sizeof(buf), "RAW: qw=%.4f qx=%.4f qy=%.4f qz=%.4f", 
                 q.qw, q.qx, q.qy, q.qz);
        //connection.publish(mqtt_topic_connection_pure_pursuit, buf);
        
        // ✅ COMPUTE YAW
        float yaw_radians = getYawIMU(q);
        yawAng = yaw_radians;
        
        float yaw_degrees = yaw_radians * 180.0f / PI;
        
        
        snprintf(buf, sizeof(buf), "YAW: %.2f deg (%.4f rad)", yaw_degrees, yaw_radians);
        //connection.publish(mqtt_topic_connection_pure_pursuit, buf);

        vTaskDelay(pdMS_TO_TICKS(200));  // Slow down for debugging
    }
}






//Task for encodervelocity
void TaskTestEncoder(void *pvParameters) {
    
    for (;;) {
        i2c_lock();
        float currVel = encoder.getWheelLinearVelocity();   // updates internal velocity state
        newY = encoder.getDistance();
        i2c_unlock();

        velocityV  = currVel;
        //connection.check_connection();
        snprintf(buf, sizeof(buf), "Current Vel=%.3f",
                     currVel);
        //connection.publish(mqtt_topic_connection_pure_pursuit, buf);

        vTaskDelay(pdMS_TO_TICKS(5));       // 200 Hz
    }
}/*

void TaskOdometry(void *pvParameters)
{
    for(;;)
    {
        i2c_lock();
        imu.readAndUpdate();
        IMUData imuData = imu.data();
        i2c_unlock();

        i2c_lock();
        float currVel = encoder.getWheelLinearVelocity();   // updates internal velocity state
        newY = encoder.getDistance();
        i2c_unlock();

        velocity = currVel;

        odom.update(imuData, currVel);

        vTaskDelay(pdMS_TO_TICKS(20));       // 50 Hz
    }
}*/

// ===============================================================
// TASK: Pose Estimation (dead reckoning)
// ===============================================================
void TaskTestPose(void *pvParameters) {
    float prevTime = millis();
    static uint32_t lastPublishTime = 0;

    for (;;) {
        float now = millis();
        float dt = (now - prevTime) / 1000.0f;
        prevTime = now;

        // yaw and velocity are updated by TaskIMU and TaskEncoder
        //pos_X += velocity * sinf(yaw) * dt;  //to be changed depending if increases clockwise or not
        pos_X -= velocityV  * sinf(yawAng) * dt;
        //pos_Y += velocity * cosf(yaw) * dt;
        pos_Y += velocityV  * cosf(yawAng) * dt;
        /*
        snprintf(buf, sizeof(buf),
                     "Pose: X=%.2f Y=%.2f  newY=%.2f Yaw=%.2f Vel=%.3f",
                     pos_X, pos_Y, newY, yaw, velocity);
        connection.publish(mqtt_topic_connection_pure_pursuit, buf);*/

        if (now - lastPublishTime >= 200) {
            connection.check_connection();
            snprintf(buf, sizeof(buf),
                     "Pose: X=%.2f Y=%.2f  newY=%.2f Yaw=%.2f Vel=%.3f",
                     pos_X, pos_Y, newY, yawAng, velocityV );
            connection.publish(mqtt_topic_connection_pure_pursuit, buf);
            lastPublishTime = now;
        }

        vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz
    }
}/*
void TaskPose(void *pvParameters) {
    static uint32_t lastPublish = 0;
    for (;;) {
        if (millis() - lastPublish >= 200) {
            snprintf(buf, sizeof(buf),
                "Pose: X=%.2f Y=%.2f NewY Yaw=%.2f Vel=%.3f ",
                odom.x(), odom.y(), newY, odom.yaw(), velocity);
            connection.publish(mqtt_topic_connection_location, buf);
            lastPublish = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}*/


// ===============================================================
// TASK: Pure Pursuit Controller
// ===============================================================
void TaskTestPurePursuit(void *pvParameters) {
    // Prepare path
    GlobalPathMessage msg;
    msg.current_length = path_Size;
    msg.timestamp_ms = millis();
    msg.path_id = 1;
    for (int i = 0; i < path_Size; i++) {
        msg.path[i].x = path_X[i];
        msg.path[i].y = path_Y[i];
    }

    purePursuit.set_path(msg);

    for (;;) {
        if (emergency_Stop) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        Pose2D currentPose = {pos_X, pos_Y, yawAng};
        //Pose2D currentPose = { odom.x(), odom.y(), odom.yaw() };
        Velocity velStruct = {velocityV , 0.0f};

        MotionCommand cmd = purePursuit.compute_command(currentPose, velStruct);



        // Steering 
        //float steeringDeg = cmd.delta_target * 180.0f / M_PI;
        servo_dir.setAngle(cmd.delta_target);

        // Maintain constant speed (for now)
        motorPowerDrive = motorPowerDrive;// cmd.v_target;  // static 0.18f, no adaptation yet

        snprintf(buf, sizeof(buf),
                "Pose: X=%.2f Y=%.2f Yaw=%.2f Vel=%.3f",
                pos_X, pos_Y, yawAng, velocityV );
            connection.publish(mqtt_topic_connection_location, buf);


        // Goal check
        float dx = path_X[path_Size - 1] - pos_X;
        float dy = path_Y[path_Size - 1] - pos_Y;
        float distToGoal = sqrtf(dx * dx + dy * dy);

        float targetX = path_X[path_Size - 1];
        float targetY = path_Y[path_Size - 1];

        bool reachedX = fabs(pos_X - targetX) <= GOAL_TOLERANCE;
        bool reachedY = fabs(pos_Y - targetY) <= GOAL_TOLERANCE;

        // Check if bot is past the goal in both axes
        bool passedX = (targetX >= 0 && pos_X > targetX) || (targetX < 0 && pos_X < targetX);
        bool passedY = (targetY >= 0 && pos_Y > targetY) || (targetY < 0 && pos_Y < targetY);
        bool passedGoal = passedX && passedY;

        // Allow overshoot only if we've passed the goal
        bool reachedByOvershoot = passedGoal && (distToGoal <= 5.0f);


        if ((reachedX && reachedY)||(reachedByOvershoot)|| (cmd.v_target == 0)){//(distToGoal < GOAL_TOLERANCE)) {
            
            connection.check_connection();
            snprintf(buf, sizeof(buf), "✅ Reached goal at (%.2f, %.2f, %.2f), stopping.", pos_X, pos_Y,newY);
            connection.publish(mqtt_topic_connection_location, buf);



            finished_Path = true;
            
            motor.stop();
            motor.update();
            
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ===============================================================
// SETUP & LOOP
// ===============================================================
void setup_test_freertos_path_follow() {
    Serial.begin(115200);
    delay(2000);

    connection.setupWifi();

    I2C_wire.begin(8, 9);
    // I2C setup for IMU and Encoder
    i2cMutexInit();

    // Wifi connection 
    connection.check_connection();


    if (!servo_dir.begin()) {
        Serial.println("Servo init failed!");
        while (true);
    }
    servo_dir.setAngle(90);

    
    if (!ultrasonic.begin()) {
        char msgSonic[50];
        snprintf(msgSonic, sizeof(msgSonic), "UltraSonic Sensor init failed!");
        connection.publish(mqtt_topic_connection_pure_pursuit , msgSonic);
        while (true);
    }

    snprintf(buf, sizeof(buf), "✅ Ultra Sonic initialized successfully");
    connection.publish(mqtt_topic_connection_pure_pursuit, buf);



    if (!imu.begin()) {
        snprintf(buf, sizeof(buf), "❌ IMU failed to initialize!");
        connection.publish(mqtt_topic_connection_pure_pursuit, buf);
        vTaskDelete(NULL);
        return;
    }
   

    snprintf(buf, sizeof(buf), "✅ IMU initialized successfully");
    connection.publish(mqtt_topic_connection_pure_pursuit, buf);
    

    if (!encoder.begin()) {
        char msgEncoder[50];
        snprintf(msgEncoder, sizeof(msgEncoder), "Encoder init failed!");
        connection.publish(mqtt_topic_connection_pure_pursuit,msgEncoder);
        while (true);
    }
    

    char msgEncoder[50];
        snprintf(msgEncoder, sizeof(msgEncoder), "Encoder setup successful!");
    connection.publish(mqtt_topic_connection_pure_pursuit,msgEncoder);

    

    if (!motor.begin()) {
        char msgMotor[50];
        snprintf(msgMotor, sizeof(msgMotor), "Motor init failed!");
        connection.publish(mqtt_topic_connection_pure_pursuit , msgMotor);
        while (true);
    }

    snprintf(buf, sizeof(buf), "✅ Motor initialized successfully");
    connection.publish(mqtt_topic_connection_pure_pursuit, buf);

    //odom.reset();





     


    snprintf(buf, sizeof(buf), "🚗 FreeRTOS Path-Follow Test Start");
    //connection.publish(mqtt_topic_connection_pure_pursuit, buf);


    
    //Core 0 from high to low priority : I2C
    xTaskCreatePinnedToCore(TaskTestIMU,          "IMU",          4096, NULL, 3,  &imuTestTask,        0); //core 0
    xTaskCreatePinnedToCore(TaskTestEncoder,      "Encoder",      4096, NULL, 2,  NULL,            0); //core 0 
    //encoder might not need high priority if we consider speed is constant most of the time


    //xTaskCreatePinnedToCore(TaskOdometry,    "Odometry",    4096, NULL, 3, NULL,         0); // core 0
    xTaskCreatePinnedToCore(TaskTestMotor,       "Motor",       4096, NULL, 2, &motorTestTask,   1); // core 1
    xTaskCreatePinnedToCore(TaskTestUltrasonic,  "Ultrasonic",  4096, NULL, 2, &ultrasonicTestTask,1); // core 1
    xTaskCreatePinnedToCore(TaskTestPose,        "Pose",        4096, NULL, 2, &poseTestTask,    1); // core 1
    xTaskCreatePinnedToCore(TaskTestPurePursuit, "PurePursuit", 4096, NULL, 1, &pursuitTestTask, 1); // core 1


}

void loop_test_freertos_path_follow() { 
}
