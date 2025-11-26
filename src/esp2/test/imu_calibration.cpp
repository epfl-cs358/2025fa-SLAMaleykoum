/**
 * @file imu_calibration.cpp
 * @brief IMU accuracy and calibration test for the BNO086.
 *
 * This test allows you to:
 *  - Measure gyro and accel bias (mean and std while static)
 *  - Test the gyro accuracy by integrating angular velocity during a manual rotation
 *  - Test linear acceleration integration on a short 1D translation
 *
 * Serial commands:
 *   1 <duration_ms>  : measure bias while static (e.g. "1 8000")
 *   2 <angle_deg>    : gyro test; enter true rotation angle (e.g. "2 90")
 *   3 <distance_m>   : translation test on x; enter true distance (e.g. "3 1.20")
 *   4 <distance_m>   : translation test on y; enter true distance (e.g. "4 1.20")
 *
 * Notes:
 *   - For (1): keep the car perfectly still on a flat surface.
 *   - For (2): hold the car and rotate it manually (yaw, around Z axis).
 *   - For (3): push the car straight on x axis for ~1–2 m, then stop it abruptly.
 *   - For (4): push the car straight on y axis for ~1–2 m, then stop it abruptly.
 * 
 * Later the data can be corrected this way :
 *  acc_corr = (acc_raw - acc_bias) * acc_scale;
 *  gyro_corr = (gyro_raw - gyro_bias) * gyro_scale;
 *  quat_corr = quat_raw - quat_bias;
 * 
 * Complete the values of the following data depending on the results of the test:
 *  - acc_bias_x =
 *  - acc_bias_y =
 *  - acc_bias_z =
 *  - gyro_bias_x =
 *  - gyro_bias_y =
 *  - gyro_bias_z =
 *  - acc_scale_x =
 *  - acc_scale_y = 
 *  - gyro_scale_z =
 */

#include "test_common_esp2.h"

const char* mqtt_topic_calibration = "slamaleykoum77/imu";

// --- Constants ---
static constexpr float GYRO_START_TH   = 0.15f;   // rad/s threshold to detect rotation start
static constexpr float GYRO_STOP_TH    = 0.10f;   // rad/s threshold to detect stop
static constexpr uint32_t STABLE_MS    = 1000;     // ms under threshold before considered stable
static constexpr float ACC_START_TH    = 0.20f;   // m/s² threshold to detect movement start
static constexpr float ACC_STOP_TH     = 0.20f;   // m/s² threshold to detect stop
static constexpr float V_STOP_TH       = 0.08f;   // m/s velocity threshold for stop
static constexpr uint32_t LOOP_DT_US   = 5;      // delay between 2 fetch of imu data
static constexpr uint32_t PRINT_EVERY  = 100;     // print every 100 ms during integration
const float ACC_STILL_THRESH  = 0.01f;   // m/s^2
const float GYRO_STILL_THRESH = 0.02f;   // rad/s

// --- Calibration data ---
struct BiasStats {
  float acc_bias_x = -0.001039, acc_bias_y = 0.000227, acc_bias_z = -0.016825;
  float quat_bias_x = -0.021069, quat_bias_y = -0.025061, quat_bias_z = 0.001512, quat_bias_w = 0.999;
  float acc_std_x = 0.025340, acc_std_y = 0.014414, acc_std_z = 0.037240;
  float quat_std_x = 0.00015, quat_std_y = 0.00024, quat_std_z = 0.00032, quat_std_w = 0;
  uint32_t samples = 0;
};

struct ScaleEst {
  float gyro_scale_z = 1.0f; // multiplier for Z-axis gyro
  float acc_scale_x  = 1.0f; // multiplier for X-axis accel
  float acc_scale_y  = 1.0f; // multiplier for Y-axis accel
};

struct RunningStats {
  int n = 0;
  double mean = 0;
  double m2 = 0;

  void update_running_stats(float x) {
    n++;
    double delta = x - mean;
    mean += delta / n;
    m2   += delta * (x - mean);
  }

  float get_std() {
    return (n > 1) ? sqrt(m2 / (n - 1)) : 0.0f;
  }
};

struct quat_t {
    float x;   // quaternion (absolute orientation)
    float y;
    float z;
    float w;
};

static BiasStats g_bias;
static ScaleEst  g_scale;

static inline float dt_s(uint32_t newer_ms, uint32_t older_ms) {
  return (float)(newer_ms - older_ms) * 1e-3f;
}

// --- Online statistics (Welford) ---
struct OnlineStats {
  uint32_t n = 0;
  double mean_x=0, mean_y=0, mean_z=0;
  double m2_x=0, m2_y=0, m2_z=0;

  void add(float x, float y, float z) {
    n++;
    double dx = x - mean_x; mean_x += dx / n; m2_x += dx * (x - mean_x);
    double dy = y - mean_y; mean_y += dy / n; m2_y += dy * (y - mean_y);
    double dz = z - mean_z; mean_z += dz / n; m2_z += dz * (z - mean_z);
  }
  void finalize(float& mx,float& my,float& mz, float& sx,float& sy,float& sz) {
    mx = (float)mean_x; my = (float)mean_y; mz = (float)mean_z;
    sx = (n>1) ? (float)sqrt(m2_x/(n-1)) : 0.f;
    sy = (n>1) ? (float)sqrt(m2_y/(n-1)) : 0.f;
    sz = (n>1) ? (float)sqrt(m2_z/(n-1)) : 0.f;
  }
};

void update_global_bias_stats(BiasStats& global, const BiasStats& run, int n) {
  // n = numéro du run courant (1-indexé)
  float k = 1.0f / n;

  // moyenne cumulative (formule en ligne)
  global.acc_bias_x += (run.acc_bias_x - global.acc_bias_x) * k;
  global.acc_bias_y += (run.acc_bias_y - global.acc_bias_y) * k;
  global.acc_bias_z += (run.acc_bias_z - global.acc_bias_z) * k;

  global.quat_bias_x += (run.quat_bias_x - global.quat_bias_x) * k;
  global.quat_bias_y += (run.quat_bias_y - global.quat_bias_y) * k;
  global.quat_bias_z += (run.quat_bias_z - global.quat_bias_z) * k;
  global.quat_bias_w += (run.quat_bias_w - global.quat_bias_w) * k;

  // moyenne des std (même logique, approximation simple)
  global.acc_std_x += (run.acc_std_x - global.acc_std_x) * k;
  global.acc_std_y += (run.acc_std_y - global.acc_std_y) * k;
  global.acc_std_z += (run.acc_std_z - global.acc_std_z) * k;

  global.quat_std_x += (run.quat_std_x - global.quat_std_x) * k;
  global.quat_std_y += (run.quat_std_y - global.quat_std_y) * k;
  global.quat_std_z += (run.quat_std_z - global.quat_std_z) * k;
  global.quat_std_w += (run.quat_std_w - global.quat_std_w) * k;
}

/**
 * @brief Calculates the Yaw (heading) angle from the IMU's quaternion.
 * @param q The quaternion (qw, qx, qy, qz) from imu_data.
 * @return The yaw angle in radians.
 */
float getYaw(const IMUData& imu_data) {
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

// --- Minimal JSON logger over MQTT ---
static void mqtt_printf(const char* tag, const char* fmt, ...) {
  char msg[160];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);

  // JSON payload: {"type":"print","tag":"Bias","message":"..."}
  char payload[256];
  snprintf(payload, sizeof(payload),
           "{\"type\":\"imu\",\"tag\":\"%s\",\"message\":\"%s\"}",
           tag, msg);
  Serial.printf("%s\n", msg);
  // connection.publish(mqtt_topic_calibration, payload);
}

// --- (1) Bias measurement ---
static void measure_bias(uint32_t duration_ms) {
  mqtt_printf("Bias", "Keep device still, collecting data...");
  OnlineStats acc_st, quat_st;

  uint32_t t0 = millis();
  uint32_t last_print = t0;

  // small warm-up
  for (int i=0;i<10;i++) { 
    imu.readAndUpdate();
    delay(20);
  }

  while ((uint32_t)(millis()-t0) < duration_ms) {
    imu.readAndUpdate();
    acc_st.add(imu.imu_data.acc_x, imu.imu_data.acc_y, imu.imu_data.acc_z);
    quat_st.add(imu.imu_data.qx, imu.imu_data.qy, imu.imu_data.qz);

    if ((uint32_t)(millis()-last_print) > PRINT_EVERY) {
      last_print = millis();
      mqtt_printf("Bias",
        "t=%lu ms acc(%.3f %.3f %.3f) quat(%.3f %.3f %.3f (w)%.3f)",
        (unsigned long)(millis()-t0),
        imu.imu_data.acc_x, imu.imu_data.acc_y, imu.imu_data.acc_z,
        imu.imu_data.qx, imu.imu_data.qy, imu.imu_data.qz, imu.imu_data.qw
      );
    }
    delay(LOOP_DT_US);
  }
  delay(10);

  g_bias.samples = acc_st.n;
  acc_st.finalize(g_bias.acc_bias_x, g_bias.acc_bias_y, g_bias.acc_bias_z,
                  g_bias.acc_std_x,  g_bias.acc_std_y,  g_bias.acc_std_z);
  quat_st.finalize(g_bias.quat_bias_x, g_bias.quat_bias_y, g_bias.quat_bias_z,
                  g_bias.quat_std_x,  g_bias.quat_std_y,  g_bias.quat_std_z);

  mqtt_printf("Bias",
    "Acc bias [m/s^2]: (%.5f, %.5f, %.5f)  std:(%.5f, %.5f, %.5f)",
    g_bias.acc_bias_x, g_bias.acc_bias_y, g_bias.acc_bias_z,
    g_bias.acc_std_x,  g_bias.acc_std_y,  g_bias.acc_std_z
  );
  mqtt_printf("Bias",
    "Quat bias [rad/s]: (%.5f, %.5f, %.5f, (w)%.5f)  std:(%.5f, %.5f, %.5f, (w)%.5f)",
    g_bias.quat_bias_x, g_bias.quat_bias_y, g_bias.quat_bias_z, g_bias.quat_bias_w,
    g_bias.quat_std_x,  g_bias.quat_std_y,  g_bias.quat_std_z, g_bias.quat_std_w
  );
}

// --- (2) Gyro test (yaw rotation around Z) ---
static void test_gyro_turn_basic() {
  delay(LOOP_DT_US);
  imu.readAndUpdate();
  mqtt_printf("Gyro", "Hold the car and start rotating around Z.\n");

  while (true) {
    // Get the stable yaw angle from the IMU's fusion engine
    float yaw_radians = getYaw(imu.data());
    float yaw_degrees = yaw_radians * 180.0f / PI;

    // Print the stable heading
    mqtt_printf("IMU", "Fused IMU Heading: %.2f degrees\n", yaw_degrees);

    delay(LOOP_DT_US);
    imu.readAndUpdate();
  }
}

// --- (3) Translation test (1D along X) ---
static void test_translation_1d_x(float dist_true_m) {
  mqtt_printf("Trans", "Keep still, then push straight ~1–2 m and stop sharply.");

  // Wait for start
  while (fabsf(imu.imu_data.acc_x - g_bias.acc_bias_x) <= ACC_START_TH) {
    delay(LOOP_DT_US);
    imu.readAndUpdate();
  }
  mqtt_printf("Trans", "Detected start. Integrating...");

  float v = 0.f, x = 0.f;
  uint32_t t_prev = imu.imu_data.timestamp_ms;
  float ax_prev   = imu.imu_data.acc_x - g_bias.acc_bias_x;
  uint32_t last_print = millis();

  uint32_t stable_cnt = 0;
  while (true) {
    imu.readAndUpdate();
    float ax = imu.imu_data.acc_x - g_bias.acc_bias_x;
    uint32_t t_now = imu.imu_data.timestamp_ms;
    float dt = dt_s(t_now, t_prev);

    // trapezoidal integration: accel -> vel -> pos
    float a_mid = 0.5f * (ax_prev + ax);
    float v_new = v + a_mid * dt;
    float x_new = x + 0.5f * (v + v_new) * dt;

    v = v_new; x = x_new;

    bool acc_still = fabsf(ax) < ACC_STOP_TH;
    bool v_still   = fabsf(v)  < V_STOP_TH;
    if (acc_still && v_still) {
      stable_cnt += (uint32_t)(dt * 1000.f);
      if (stable_cnt >= STABLE_MS) break;
    } else {
      stable_cnt = 0;
    }

    if ((uint32_t)(millis() - last_print) > PRINT_EVERY) {
      last_print = millis();
      mqtt_printf("Trans", "x=%.3f m, v=%.2f m/s, ax=%.2f m/s^2", x, v, ax);
    }

    t_prev = t_now;
    ax_prev = ax;
    delay(LOOP_DT_US);
  }

  float dist_meas = x;
  float scale = (fabsf(dist_meas) > 1e-3f) ? (dist_true_m / dist_meas) : 1.0f;
  g_scale.acc_scale_x = scale;

  mqtt_printf("Trans", "Measured=%.3f m, True=%.3f m, acc_scale_x=%.6f",
              dist_meas, dist_true_m, g_scale.acc_scale_x);
}

static void test_translation_1d_x_basic() {
  mqtt_printf("Trans", "Translation on x");

  const float ACC_STILL = 0.03f;   // m/s^2
  const float Q_STILL   = 1e-4f;   // variation de quat très faible

  delay(LOOP_DT_US);
  imu.readAndUpdate();

  quat_t q_prev = {imu.imu_data.qx, imu.imu_data.qy, imu.imu_data.qz, imu.imu_data.qw};
  float v = 0.f, x = 0.f;
  uint32_t t_prev = imu.imu_data.timestamp_ms;
  float ax_prev   = imu.imu_data.acc_x;
  uint32_t last_print = millis();

  delay(LOOP_DT_US);

  while (true) {
    imu.readAndUpdate();

    uint32_t t_now = imu.imu_data.timestamp_ms;
    float dt = dt_s(t_now, t_prev);

    t_prev = t_now;

    float ax = imu.imu_data.acc_x ;

    // petit deadband pour tuer le bruit
    if (fabsf(ax) < ACC_STILL) ax = 0.f;

    float a_mid = 0.5f * (ax_prev + ax);
    float v_new = v + a_mid * dt;
    float x_new = x + v * dt + 0.5f * a_mid * dt * dt;

    quat_t q = {imu.imu_data.qx, imu.imu_data.qy, imu.imu_data.qz, imu.imu_data.qw};
    float dq =
        fabsf(q.w - q_prev.w) +
        fabsf(q.x - q_prev.x) +
        fabsf(q.y - q_prev.y) +
        fabsf(q.z - q_prev.z);

    if (ax == 0.f && dq < Q_STILL) {
        v_new = 0.f;  // ZUPT soft
    }

    v = v_new; x = x_new, ax_prev = ax, q_prev = q;

    if ((uint32_t)(millis() - last_print) > PRINT_EVERY) {
      last_print = millis();
      mqtt_printf("Trans", "t=%lu ms, x=%.2f cm, v=%.2f m/s, ax=%.2f m/s^2", 
                  (unsigned long)millis(), x*100, v, ax);
    }

    delay(LOOP_DT_US);
  }
}

static void test_translation_1d_y_basic() {
  mqtt_printf("Trans", "Translation on x");

  const float ACC_STILL = 0.03f;   // m/s^2
  const float Q_STILL   = 1e-4f;   // variation de quat très faible

  delay(LOOP_DT_US);
  imu.readAndUpdate();

  quat_t q_prev = {imu.imu_data.qx, imu.imu_data.qy, imu.imu_data.qz, imu.imu_data.qw};
  float v = 0.f, y = 0.f;
  uint32_t t_prev = imu.imu_data.timestamp_ms;
  float ay_prev   = imu.imu_data.acc_y;
  uint32_t last_print = millis();

  delay(LOOP_DT_US);

  while (true) {
    imu.readAndUpdate();

    uint32_t t_now = imu.imu_data.timestamp_ms;
    float dt = dt_s(t_now, t_prev);

    t_prev = t_now;

    float ay = imu.imu_data.acc_y ;

    // petit deadband pour tuer le bruit
    if (fabsf(ay) < ACC_STILL) ay = 0.f;

    float a_mid = 0.5f * (ay_prev + ay);
    float v_new = v + a_mid * dt;
    float y_new = y + v * dt + 0.5f * a_mid * dt * dt;

    quat_t q = {imu.imu_data.qx, imu.imu_data.qy, imu.imu_data.qz, imu.imu_data.qw};
    float dq =
        fabsf(q.w - q_prev.w) +
        fabsf(q.x - q_prev.x) +
        fabsf(q.y - q_prev.y) +
        fabsf(q.z - q_prev.z);

    if (ay == 0.f && dq < Q_STILL) {
        v_new = 0.f;  // ZUPT soft
    }

    v = v_new; y = y_new, ay_prev = ay, q_prev = q;

    if ((uint32_t)(millis() - last_print) > PRINT_EVERY) {
      last_print = millis();
      mqtt_printf("Trans", "t=%lu ms, y=%.2f cm, v=%.2f m/s, ay=%.2f m/s^2", 
                  (unsigned long)millis(), y*100, v, ay);
    }

    delay(LOOP_DT_US);
  }
}

// --- (4) Translation test (1D along Y) ---
static void test_translation_1d_y(float dist_true_m) {
  mqtt_printf("Trans", "Keep still, then push straight ~1–2 m and stop sharply.");

  // Wait for start
  while (fabsf(imu.imu_data.acc_y - g_bias.acc_bias_y) <= ACC_START_TH) {
    delay(LOOP_DT_US);
    imu.readAndUpdate();
  }
  mqtt_printf("Trans", "Detected start. Integrating...");

  float v = 0.f, y = 0.f;
  uint32_t t_prev = imu.imu_data.timestamp_ms;
  float ay_prev   = imu.imu_data.acc_y - g_bias.acc_bias_y;
  uint32_t last_print = millis();

  uint32_t stable_cnt = 0;
  while (true) {
    imu.readAndUpdate();
    float ay = imu.imu_data.acc_y - g_bias.acc_bias_y;
    uint32_t t_now = imu.imu_data.timestamp_ms;
    float dt = dt_s(t_now, t_prev);

    // trapezoidal integration: accel -> vel -> pos
    float a_mid = 0.5f * (ay_prev + ay);
    float v_new = v + a_mid * dt;
    float y_new = y + 0.5f * (v + v_new) * dt;

    v = v_new; y = y_new;

    bool acc_still = fabsf(ay) < ACC_STOP_TH;
    bool v_still   = fabsf(v)  < V_STOP_TH;
    if (acc_still && v_still) {
      stable_cnt += (uint32_t)(dt * 1000.f);
      if (stable_cnt >= STABLE_MS) break;
    } else {
      stable_cnt = 0;
    }

    if ((uint32_t)(millis() - last_print) > PRINT_EVERY) {
      last_print = millis();
      mqtt_printf("Trans", "y=%.3f m, v=%.2f m/s, ay=%.2f m/s^2", y, v, ay);
    }

    t_prev = t_now;
    ay_prev = ay;
    delay(LOOP_DT_US);
  }

  float dist_meas = y;
  float scale = (fabsf(dist_meas) > 1e-3f) ? (dist_true_m / dist_meas) : 1.0f;
  g_scale.acc_scale_y = scale;

  mqtt_printf("Trans", "Measured=%.3f m, True=%.3f m, acc_scale_y=%.6f",
              dist_meas, dist_true_m, g_scale.acc_scale_y);
}

static void handle_command(const String& line) {
  char cmd = line.charAt(0);
  float val = line.substring(1).toFloat();

  int runs = 5;
  switch (cmd) {
    case '1': {
      BiasStats global_stats; // to accumulate the final stats
      
      for (int i = 1; i <= runs; ++i) {
        measure_bias(val);
        update_global_bias_stats(global_stats, g_bias, i);
        imu.readAndUpdate();
        delay(1000);
      }

      mqtt_printf("IMU", "\n[Final Stats after %d runs]\n", runs);
      mqtt_printf("IMU", "Acc bias [m/s²]: (%.6f, %.6f, %.6f), std: (%.6f, %.6f, %.6f)\n",
                    global_stats.acc_bias_x, global_stats.acc_bias_y, global_stats.acc_bias_z,
                    global_stats.acc_std_x, global_stats.acc_std_y, global_stats.acc_std_z);
      mqtt_printf("IMU", "Quat bias [rad/s]: (%.6f, %.6f, %.6f), std: (%.6f, %.6f, %.6f)\n",
                    global_stats.quat_bias_x, global_stats.quat_bias_y, global_stats.quat_bias_z,
                    global_stats.quat_std_x, global_stats.quat_std_y, global_stats.quat_std_z);
      break;
    }
    case '2': {
      test_gyro_turn_basic();
      break;
    }
    case '3': {
      RunningStats accel_x_scale_stats;

      for (int i = 1; i <= runs; i++) {
        mqtt_printf("IMU", "New test begin\n");
        test_translation_1d_x(val);
        accel_x_scale_stats.update_running_stats(g_scale.acc_scale_x);

        mqtt_printf("IMU", "Test %d finit \n", i);
        delay(500);
      }

      mqtt_printf("IMU", "\n[Final Accel on X axis calibration after %d runs]\n", accel_x_scale_stats.n);
      mqtt_printf("IMU", "Mean scale_x = %.6f  |  Std = %.6f\n", 
                  accel_x_scale_stats.mean, accel_x_scale_stats.get_std());
      break;
    }
    case '4': {
      RunningStats accel_y_scale_stats;

      for (int i = 1; i <= runs; i++) {
        test_translation_1d_y(val);
        accel_y_scale_stats.update_running_stats(g_scale.acc_scale_y);

        Serial.printf("Test %d finit \n", i);
        delay(2000);
      }

      mqtt_printf("IMU", "\n[Final Accel on Y axis calibration after %d runs]\n", accel_y_scale_stats.n);
      mqtt_printf("IMU", "Mean scale_y = %.6f  |  Std = %.6f\n", 
                  accel_y_scale_stats.mean, accel_y_scale_stats.get_std());
      break;
    }
    case '5': {
      test_translation_1d_x_basic();
      break;
    }
    case '6': {
      test_translation_1d_y_basic();
    }
    default:
      break;
  }
}

// === Test entry points ===
void setup_imu_calibration() {
  Serial.begin(115200);
  delay(2000);

  // connection.setupWifi();

  // connection.check_connection();
  
  mqtt_printf("IMU", "Calibration and accuracy test started");  

  i2cMutexInit();

  delay(2000);
  if (!imu.begin()) {
      Serial.println("IMU init failed!");
      while(true);
  }
  imu.readAndUpdate();
  delay(150);
}

/** Complete line with serial command:
 *   1 <duration_ms>  : measure bias while static (e.g. "1 8000")
 *   2                : gyro test;
 *   3 <distance_m>   : translation test for x; enter true distance (e.g. "3 1.20")
 *   4 <distance_m>   : translation test for y; enter true distance (e.g. "4 1.20")
 *   5                : accel basic on x 
 */
void loop_imu_calibration() {
  // connection.check_connection();

  handle_command("2");

  while (true) {delay(10000);}
  
}