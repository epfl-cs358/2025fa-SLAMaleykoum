#include <Arduino.h>
#include "test_common_esp2.h"        // gives imu, encoder, connection, mutex etc.
#include "esp2/localization/ekf_localizer.h"
#include "common/data_types.h"

// MQTT topic for debugging EKF
const char* mqtt_topic_ekf_debug = "slamaleykoum77/ekf";

// Create the EKF localizer (start at origin)
EKFLocalizer ekf({0,0,0,0});

// Helper to publish JSON over MQTT
void publish_mqtt(const char* fmt, ...) {
    char buf[300];
    snprintf(buf, sizeof(buf), fmt);
    connection.publish(mqtt_topic_ekf_debug, buf);
    delay(500);
}

void setup_ekfTest() {
    Serial.begin(115200);
    delay(1500);

    Serial.println("\n=== EKF MQTT REAL CAR TEST ===");

    // Init WiFi & MQTT
    connection.setupWifi();
    delay(1000);
    connection.check_connection();

    I2C_wire.begin(8,9);

    // Init IMU
    i2cMutexInit();
    if (!imu.begin()) {
        Serial.println("IMU FAILED to init!");
        while (true) delay(1000);
    }
    imu.readAndUpdate();
    if (!encoder.begin()) {
        Serial.println("Encoder FAILED to init!");
        while (true) delay(1000);
    }
    if (!motor.begin()) {
        Serial.println("Motor FAILED to init!");
        while (true) delay(1000);
    }

    publish_mqtt("{\"type\":\"print\",\"msg\":\"EKF MQTT test initialized\"}");
}

// Main test loop
void loop_ekfTest() {
    motor.forward(0.17);
    motor.update();
    // -----------------------------
    // 1) Read IMU
    // -----------------------------
    imu.readAndUpdate();
    IMUData imu_data = imu.imu_data;   // direct copy from driver

    // -----------------------------
    // 2) Encoder → odometry
    // -----------------------------
    static uint32_t last_ms = imu_data.timestamp_ms;
    uint32_t now = imu_data.timestamp_ms;

    float dt = (now - last_ms) * 0.001f;
    if (dt < 0.001f) dt = 0.001f;
    last_ms = now;

    float wheel_vel = encoder.getWheelLinearVelocity();   // m/s
    float delta_d   = wheel_vel * dt;                     // distance traveled in dt

    OdometryData odom;
    odom.delta_distance = delta_d;
    odom.timestamp_ms   = now;

    // -----------------------------
    // 3) EKF full update
    // -----------------------------
    ekf.update_pose(odom, imu_data);

    // -----------------------------
    // 4) Read EKF state
    // -----------------------------
    Pose2D pose = ekf.get_current_pose();
    Velocity v  = ekf.get_current_velocity();

    // -----------------------------
    // 5) Publish everything
    // -----------------------------
    publish_mqtt(
        "{"
          "\"type\":\"ekf\","
          "\"x\":%.3f,"
          "\"y\":%.3f,"
          "\"theta\":%.3f,"
          "\"v\":%.3f,"
          "\"omega\":%.3f,"
          "\"acc_x\":%.3f,"
          "\"acc_y\":%.3f,"
          "\"acc_z\":%.3f,"
          "\"odom_v\":%.3f,"
          "\"timestamp\":%lu"
        "}",
        pose.x,
        pose.y,
        pose.theta,
        v.v_linear,
        v.v_angular,
        imu_data.acc_x,
        imu_data.acc_y,
        imu_data.acc_z,
        wheel_vel,
        (unsigned long)imu_data.timestamp_ms
    );

    // -----------------------------
    // 6) Keep MQTT alive
    // -----------------------------
    connection.check_connection();

    // Slow enough to be readable (20 Hz)
}

/** 
#include <Arduino.h>
#include "esp2/localization/ekf_localizer.h"
#include "common/data_types.h"   // your Pose2D, IMUData, OdometryData

EKFLocalizer ekf({0.0f, 0.0f, 0.0f, 0});  // initial pose (x, y, theta, timestamp)


// ==========================
// Fake IMU generator
// ==========================
IMUData makeFakeIMU(uint32_t t_ms, float yaw, float acc_x, float acc_y)
{
    IMUData imu{};
    imu.timestamp_ms = t_ms;

    // ==========================
    // quaternion from yaw only
    // roll = pitch = 0
    // ==========================
    float half = yaw * 0.5f;
    imu.qw = cosf(half);
    imu.qx = 0.0f;
    imu.qy = 0.0f;
    imu.qz = sinf(half);

    // IMPORTANT:
    // Your struct says:
    //   acc_x = forward
    //   acc_y = lateral
    imu.acc_x = acc_x;  // forward acceleration
    imu.acc_y = acc_y;  // lateral accel
    imu.acc_z = 0.0f;

    return imu;
}

// ==========================
// Fake odometry generator
// ==========================
OdometryData makeFakeOdom(uint32_t t_ms, float delta_distance)
{
    OdometryData od{};
    od.timestamp_ms = t_ms;
    od.delta_distance = delta_distance;
    return od;
}

// ==========================
// Arduino setup
// ==========================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== EKF TEST START ===");
}

// ==========================
// MAIN LOOP – runs simulation
// ==========================
void loop() {
    static uint32_t last_ms = millis();
    uint32_t now = millis();

    float dt = (now - last_ms) / 1000.0f;
    if (dt <= 0.0f)
        return;
    last_ms = now;

    // ==============================
    // CIRCULAR MOTION SIMULATION
    // ==============================

    // Constant forward speed
    float speed = 0.4f; // m/s

    // Constant turn rate (angular velocity)
    float omega = 0.5f;  // rad/s  (about 30 deg/s)

    // Update ground-truth yaw
    static float true_yaw = 0.0f;
    true_yaw += omega * dt;
    if (true_yaw > M_PI) true_yaw -= 2*M_PI;

    // Distance traveled from encoder
    float delta_distance = speed * dt;

    // IMU forward acceleration (none for constant speed)
    float acc_forward = 0.0f;

    // Lateral acceleration from circular motion:  a = v²/R = v * omega
    float acc_lateral = speed * omega;

    // Fake IMU measurement
    IMUData imu = makeFakeIMU(now, true_yaw, acc_forward, acc_lateral);

    // Fake odometry (only linear distance)
    OdometryData odom = makeFakeOdom(now, delta_distance);

    // ==============================
    // UPDATE EKF
    // ==============================
    ekf.update_pose(odom, imu);

    // ==============================
    // PRINT RESULTS
    // ==============================
    Pose2D pose = ekf.get_current_pose();
    Velocity vel = ekf.get_current_velocity();

    Serial.print("t=");
    Serial.print((now) / 1000.0f);
    Serial.print("  x=");
    Serial.print(pose.x, 4);
    Serial.print("  y=");
    Serial.print(pose.y, 4);
    Serial.print("  th=");
    Serial.print(pose.theta, 4);
    Serial.print("  v=");
    Serial.println(vel.v_linear, 4);

    delay(100);
}
**/

