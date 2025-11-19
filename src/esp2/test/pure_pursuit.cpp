#include "test_common_esp2.h"

#include "common/data_types.h"
#include <Arduino.h>
#include <cmath>
#include "I2C_wire.h"


const char* mqtt_topic_connection_pure_pursuit = "slamaleykoum77/print";


// === Global instances ===


bool emergencyStop = false;
bool finishedPath = false;
float motorPowerDrive = 0.18f;  // 18%


char buf[100];


// --- Pose estimation ---

float velocity = 0.0f;

float newY = 0.0f;

// --- Path definition ---
float pathX[] = {0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0};//{0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.5};//{0.0, 0.5, 1.0, 1.5, 2.0};
float pathY[] = {0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0};//{0.0, 0.5, 1.0, 1.5, 2.0, 2.0, 2.0};//{0.0, 0.2, 0.4, 0.6, 0.6};
const int pathSize = sizeof(pathX) / sizeof(pathX[0]);

// --- Constants ---
const float EMERGENCY_DISTANCE = 0.25f;
const float GOAL_TOLERANCE = 0.00f;  // 5 cm

// --- FreeRTOS handles ---
TaskHandle_t motorTask, ultrasonicTask, imuTask, poseTask, pursuitTask;

// ===============================================================
// TASK: Motor Control
// ===============================================================
void TaskMotor(void *pvParameters) {
    
    for (;;) {
        if (emergencyStop || finishedPath) {
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
void TaskUltrasonic(void *pvParameters) {
    
    for (;;) {
        float dist = ultrasonic.readDistance();
        if (dist > 0 && dist < EMERGENCY_DISTANCE) {
            motor.stop();
            motor.update();

            emergencyStop = true;
            
            snprintf(buf, sizeof(buf), "⚠️ Emergency stop! Distance=%.2f m", dist);
            connection.publish(mqtt_topic_connection_pure_pursuit, buf);

             

        } /*else if (dist > (EMERGENCY_DISTANCE + 0.05f)) {
            emergencyStop = false; //this will make it restart so if you want it to stay stopped keeep commented out
        }*/
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ===============================================================
// TASK: IMU Orientation (yaw extraction)
// ===============================================================

/*float getYawIMU(const IMUData& imu_data) {
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
}*/



/*void TaskIMU(void *pvParameters) {
   



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
        yaw = yaw_radians;
        
        float yaw_degrees = yaw_radians * 180.0f / PI;
        
        
        snprintf(buf, sizeof(buf), "YAW: %.2f deg (%.4f rad)", yaw_degrees, yaw_radians);
        //connection.publish(mqtt_topic_connection_pure_pursuit, buf);

        vTaskDelay(pdMS_TO_TICKS(200));  // Slow down for debugging
    }
}*/

/*




//Task for encodervelocity
void TaskEncoder(void *pvParameters) {
    
    for (;;) {
        i2c_lock();
        float currVel = encoder.getWheelLinearVelocity();   // updates internal velocity state
        newY = encoder.getDistance();
        i2c_unlock();

        velocity = currVel;
        connection.check_connection();
        snprintf(buf, sizeof(buf), "Current Vel=%.3f",
                     currVel);
        //connection.publish(mqtt_topic_connection_pure_pursuit, buf);

        vTaskDelay(pdMS_TO_TICKS(5));       // 200 Hz
    }
}*/

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
}

// ===============================================================
// TASK: Pose Estimation (dead reckoning)
// ===============================================================
/*void TaskPose(void *pvParameters) {
    float prevTime = millis();
    static uint32_t lastPublishTime = 0;

    for (;;) {
        float now = millis();
        float dt = (now - prevTime) / 1000.0f;
        prevTime = now;

        // yaw and velocity are updated by TaskIMU and TaskEncoder
        //posX += velocity * sinf(yaw) * dt;  //to be changed depending if increases clockwise or not
        posX -= velocity * sinf(yaw) * dt;
        //posY += velocity * cosf(yaw) * dt;
        posY += velocity * cosf(yaw) * dt;

        if (now - lastPublishTime >= 200) {
            connection.check_connection();
            snprintf(buf, sizeof(buf),
                     "Pose: X=%.2f Y=%.2f  newY=%.2f Yaw=%.2f Vel=%.3f",
                     posX, posY, newY, yaw, velocity);
            connection.publish(mqtt_topic_connection_pure_pursuit, buf);
            lastPublishTime = now;
        }

        vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz
    }
}*/
void TaskPose(void *pvParameters) {
    static uint32_t lastPublish = 0;
    for (;;) {
        if (millis() - lastPublish >= 200) {
            snprintf(buf, sizeof(buf),
                "Pose: X=%.2f Y=%.2f Yaw=%.2f Vel=%.3f",
                odom.x(), odom.y(), odom.yaw(), velocity);
            connection.publish(mqtt_topic_connection_pure_pursuit, buf);
            lastPublish = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


// ===============================================================
// TASK: Pure Pursuit Controller
// ===============================================================
void TaskPurePursuit(void *pvParameters) {
    // Prepare path
    GlobalPathMessage msg;
    msg.current_length = pathSize;
    msg.timestamp_ms = millis();
    msg.path_id = 1;
    for (int i = 0; i < pathSize; i++) {
        msg.path[i].x = pathX[i];
        msg.path[i].y = pathY[i];
    }

    purePursuit.set_path(msg);

    for (;;) {
        if (emergencyStop) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        //Pose2D currentPose = {posX, posY, yaw};
        Pose2D currentPose = { odom.x(), odom.y(), odom.yaw() };
        Velocity velStruct = {velocity, 0.0f};

        MotionCommand cmd = purePursuit.compute_command(currentPose, velStruct);



        // Steering 
        float steeringDeg = cmd.delta_target * 180.0f / M_PI;
        servo_dir.setAngle(90.0f + steeringDeg);

        // Maintain constant speed (for now)
        motorPowerDrive = motorPowerDrive;// cmd.v_target;  // static 0.18f, no adaptation yet

        // Goal check
        float dx = pathX[pathSize - 1] - odom.x();
        float dy = pathY[pathSize - 1] - odom.y();
        float distToGoal = sqrtf(dx * dx + dy * dy);

        float targetX = pathX[pathSize - 1];
        float targetY = pathY[pathSize - 1];

        bool reachedX = fabs(odom.x() - targetX) <= GOAL_TOLERANCE;
        bool reachedY = fabs(odom.y() - targetY) <= GOAL_TOLERANCE;

        // Check if bot is past the goal in both axes
        bool passedX = (targetX >= 0 && odom.x() > targetX) || (targetX < 0 && odom.x() < targetX);
        bool passedY = (targetY >= 0 && odom.y() > targetY) || (targetY < 0 && odom.y() < targetY);
        bool passedGoal = passedX && passedY;

        // Allow overshoot only if we've passed the goal
        bool reachedByOvershoot = passedGoal && (distToGoal <= 5.0f);


        if ((reachedX && reachedY)||(reachedByOvershoot)|| (cmd.v_target == 0)){//(distToGoal < GOAL_TOLERANCE)) {
            
            snprintf(buf, sizeof(buf), "✅ Reached goal at (%.2f, %.2f, %.2f), stopping.", odom.x(), odom.y(),newY);
            connection.publish(mqtt_topic_connection_pure_pursuit, buf);



            finishedPath = true;
            
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


    

    if (!encoder.begin()) {
        char msgEncoder[50];
        snprintf(msgEncoder, sizeof(msgEncoder), "Encoder init failed!");
        connection.publish(mqtt_topic_connection_pure_pursuit,msgEncoder);
        while (true);
    }
    

    char msgEncoder[50];
        snprintf(msgEncoder, sizeof(msgEncoder), "Encoder setup successful!");
    connection.publish(mqtt_topic_connection_pure_pursuit,msgEncoder);

    
    if (!imu.begin()) {
        snprintf(buf, sizeof(buf), "❌ IMU failed to initialize!");
        connection.publish(mqtt_topic_connection_pure_pursuit, buf);
        vTaskDelete(NULL);
        return;
    }
    

    snprintf(buf, sizeof(buf), "✅ IMU initialized successfully");
    connection.publish(mqtt_topic_connection_pure_pursuit, buf);

    
    if (!ultrasonic.begin()) {
        char msgSonic[50];
        snprintf(msgSonic, sizeof(msgSonic), "UltraSonic Sensor init failed!");
        connection.publish(mqtt_topic_connection_pure_pursuit , msgSonic);
        while (true);
    }


    if (!motor.begin()) {
        char msgMotor[50];
        snprintf(msgMotor, sizeof(msgMotor), "Motor init failed!");
        connection.publish(mqtt_topic_connection_pure_pursuit , msgMotor);
        while (true);
    }





     


    snprintf(buf, sizeof(buf), "🚗 FreeRTOS Path-Follow Test Start");
    //connection.publish(mqtt_topic_connection_pure_pursuit, buf);


    
    //Core 0 from high to low priority : I2C
    /*xTaskCreatePinnedToCore(TaskIMU,          "IMU",          4096, NULL, 3,  &imuTask,        0); //core 0
    xTaskCreatePinnedToCore(TaskEncoder,      "Encoder",      4096, NULL, 2,  NULL,            0); //core 0 
    //encoder might not need high priority if we consider speed is constant most of the time


    xTaskCreatePinnedToCore(TaskOdometry,    "Odometry",    4096, NULL, 3, NULL,         0); // core 0
    xTaskCreatePinnedToCore(TaskMotor,       "Motor",       4096, NULL, 2, &motorTask,   1); // core 1
    //xTaskCreatePinnedToCore(TaskUltrasonic,  "Ultrasonic",  4096, NULL, 2, &ultrasonicTask,1); // core 1
    xTaskCreatePinnedToCore(TaskPose,        "Pose",        4096, NULL, 2, &poseTask,    1); // core 1
    xTaskCreatePinnedToCore(TaskPurePursuit, "PurePursuit", 4096, NULL, 1, &pursuitTask, 1); // core 1


}

void loop_test_freertos_path_follow() { 
}
