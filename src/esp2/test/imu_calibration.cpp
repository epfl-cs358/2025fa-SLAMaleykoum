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
 *   3 <distance_m>   : translation test; enter true distance (e.g. "3 1.20")
 *   p                : print last IMU sample
 *
 * Notes:
 *   - For (1): keep the car perfectly still on a flat surface.
 *   - For (2): hold the car and rotate it manually (yaw, around Z axis).
 *   - For (3): push the car straight for ~1–2 m, then stop it abruptly.
 * 
 * Later the data can be corrected this way :
 *  acc_corr = (acc_raw - acc_bias) * acc_scale;
 *  gyro_corr = (gyro_raw - gyro_bias) * gyro_scale;
 * 
 * Complete the values of the following data depending on the results of the test:
 *  - acc_bias_x =
 *  - acc_bias_y =
 *  - acc_bias_z =
 *  - gyro_bias_x =
 *  - gyro_bias_y =
 *  - gyro_bias_z =
 *  - acc_scale_x =
 *  - acc_scale_y ????? 
 *  - gyro_scale_z =
 */

#include "test_common_esp2.h"

const char* mqtt_topic_connection = "slamaleykoum77/print";

// --- Constants ---
static constexpr float GYRO_START_TH   = 0.35f;   // rad/s threshold to detect rotation start
static constexpr float GYRO_STOP_TH    = 0.15f;   // rad/s threshold to detect stop
static constexpr uint32_t STABLE_MS    = 300;     // ms under threshold before considered stable
static constexpr float ACC_START_TH    = 0.40f;   // m/s² threshold to detect movement start
static constexpr float ACC_STOP_TH     = 0.25f;   // m/s² threshold to detect stop
static constexpr float V_STOP_TH       = 0.08f;   // m/s velocity threshold for stop
static constexpr uint32_t LOOP_DT_US   = 3000;    // ~333 Hz sampling if possible
static constexpr uint32_t PRINT_EVERY  = 50;      // print every 50 ms during integration

// --- Calibration data ---
struct BiasStats {
  float acc_bias_x = 0, acc_bias_y = 0, acc_bias_z = 0;
  float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
  float acc_std_x = 0, acc_std_y = 0, acc_std_z = 0;
  float gyro_std_x = 0, gyro_std_y = 0, gyro_std_z = 0;
  uint32_t samples = 0;
};

struct ScaleEst {
  float gyro_scale_z = 1.0f; // multiplier for Z-axis gyro
  float acc_scale_x  = 1.0f; // multiplier for X-axis accel
};

static BiasStats g_bias;
static ScaleEst  g_scale;

// --- Time helpers ---
static inline float dt_ms(uint32_t newer_ms, uint32_t older_ms) {
  return (float)((uint32_t)(newer_ms - older_ms));
}
static inline float dt_s(uint32_t newer_ms, uint32_t older_ms) {
  return dt_ms(newer_ms, older_ms) * 1e-3f;
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
           "{\"type\":\"print\",\"tag\":\"%s\",\"message\":\"%s\"}",
           tag, msg);
  connection.publish(mqtt_topic_connection, payload);
}

// --- (1) Bias measurement ---
static void measure_bias(uint32_t duration_ms) {
  mqtt_printf("Bias", "Keep device still, collecting data...");
  OnlineStats acc_st, gyr_st;

  uint32_t t0 = millis();
  uint32_t last_print = t0;

  // small warm-up
  for (int i=0;i<10;i++) { imu.readAndUpdate(); delay(5); }

  while ((uint32_t)(millis()-t0) < duration_ms) {
    imu.readAndUpdate();
    acc_st.add(imu_data.acc_x, imu_data.acc_y, imu_data.acc_z);
    gyr_st.add(imu_data.omega_x, imu_data.omega_y, imu_data.omega_z);

    if ((uint32_t)(millis()-last_print) > 500) {
      last_print = millis();
      mqtt_printf("Bias",
        "t=%lu ms acc(%.3f %.3f %.3f) gyro(%.3f %.3f %.3f)",
        (unsigned long)(millis()-t0),
        imu_data.acc_x, imu_data.acc_y, imu_data.acc_z,
        imu_data.omega_x, imu_data.omega_y, imu_data.omega_z
      );
    }
    delayMicroseconds(LOOP_DT_US);
  }

  g_bias.samples = acc_st.n;
  acc_st.finalize(g_bias.acc_bias_x, g_bias.acc_bias_y, g_bias.acc_bias_z,
                  g_bias.acc_std_x,  g_bias.acc_std_y,  g_bias.acc_std_z);
  gyr_st.finalize(g_bias.gyro_bias_x, g_bias.gyro_bias_y, g_bias.gyro_bias_z,
                  g_bias.gyro_std_x,  g_bias.gyro_std_y,  g_bias.gyro_std_z);

  mqtt_printf("Bias", "N=%lu", (unsigned long)g_bias.samples);
  mqtt_printf("Bias",
    "Acc bias [m/s^2]: (%.5f, %.5f, %.5f)  std:(%.5f, %.5f, %.5f)",
    g_bias.acc_bias_x, g_bias.acc_bias_y, g_bias.acc_bias_z,
    g_bias.acc_std_x,  g_bias.acc_std_y,  g_bias.acc_std_z
  );
  mqtt_printf("Bias",
    "Gyro bias [rad/s]: (%.5f, %.5f, %.5f)  std:(%.5f, %.5f, %.5f)",
    g_bias.gyro_bias_x, g_bias.gyro_bias_y, g_bias.gyro_bias_z,
    g_bias.gyro_std_x,  g_bias.gyro_std_y,  g_bias.gyro_std_z
  );
  mqtt_printf("Bias", "=> Use these as software offsets.");
}

// --- (2) Gyro test (yaw rotation around Z) ---
static void test_gyro_turn(float angle_true_deg) {
  mqtt_printf("Gyro", "Hold the car and start rotating around Z.");

  // Wait for motion start
  uint32_t stable_cnt = 0;
  while (true) {
    imu.readAndUpdate();
    float wz = imu_data.omega_z - g_bias.gyro_bias_z;
    if (fabsf(wz) > GYRO_START_TH) break;
    delayMicroseconds(LOOP_DT_US);
  }
  mqtt_printf("Gyro", "Detected start. Integrating...");

  float angle_rad = 0.f;
  uint32_t t_prev = imu_data.timestamp_ms;
  float wz_prev   = imu_data.omega_z - g_bias.gyro_bias_z;
  uint32_t last_print = millis();

  while (true) {
    imu.readAndUpdate();
    float wz   = imu_data.omega_z - g_bias.gyro_bias_z;
    uint32_t t_now = imu_data.timestamp_ms;
    float dt   = dt_s(t_now, t_prev);

    // trapezoidal integration
    angle_rad += 0.5f * (wz_prev + wz) * dt;

    // stop condition (below threshold for STABLE_MS)
    if (fabsf(wz) < GYRO_STOP_TH) {
      stable_cnt += (uint32_t)(dt * 1000.f);
      if (stable_cnt >= STABLE_MS) break;
    } else {
      stable_cnt = 0;
    }

    if ((uint32_t)(millis() - last_print) > PRINT_EVERY) {
      last_print = millis();
      mqtt_printf("Gyro", "angle=%.2f deg, wz=%.3f rad/s",
                  angle_rad * 180.f / PI, wz);
    }

    t_prev = t_now;
    wz_prev = wz;
    delayMicroseconds(LOOP_DT_US);
  }

  float angle_deg_meas = angle_rad * 180.f / PI;
  float scale = (fabsf(angle_deg_meas) > 1e-3f) ? (angle_true_deg / angle_deg_meas) : 1.0f;
  g_scale.gyro_scale_z = scale;

  mqtt_printf("Gyro", "Measured=%.2f deg, True=%.2f deg, scale_z=%.6f",
              angle_deg_meas, angle_true_deg, g_scale.gyro_scale_z);
  mqtt_printf("Gyro", "=> Apply: corrected_angle = measured_angle * scale_z");

  // Optional: publish a structured JSON "result" (handy to parse on the broker/UI)
//   {
//     char payload[192];
//     snprintf(payload, sizeof(payload),
//              "{\"type\":\"gyro_result\",\"tag\":\"Gyro\","
//              "\"angle_deg_meas\":%.2f,\"angle_deg_true\":%.2f,\"scale_z\":%.6f}",
//              angle_deg_meas, angle_true_deg, g_scale.gyro_scale_z);
//     connection.publish(mqtt_topic_connection, payload);
//   }
}

// --- (3) Translation test (1D along X) ---
static void test_translation_1d(float dist_true_m) {
  mqtt_printf("Trans", "Keep still, then push straight ~1–2 m and stop sharply.");

  // Wait for start
  uint32_t stable_cnt = 0;
  while (true) {
    imu.readAndUpdate();
    float ax = imu_data.acc_x - g_bias.acc_bias_x;
    if (fabsf(ax) > ACC_START_TH) break;
    delayMicroseconds(LOOP_DT_US);
  }
  mqtt_printf("Trans", "Detected start. Integrating...");

  float v = 0.f, x = 0.f;
  uint32_t t_prev = imu_data.timestamp_ms;
  float ax_prev   = imu_data.acc_x - g_bias.acc_bias_x;
  uint32_t last_print = millis();

  while (true) {
    imu.readAndUpdate();
    float ax   = imu_data.acc_x - g_bias.acc_bias_x;
    uint32_t t_now = imu_data.timestamp_ms;
    float dt   = dt_s(t_now, t_prev);

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
    delayMicroseconds(LOOP_DT_US);
  }

  float dist_meas = x;
  float scale = (fabsf(dist_meas) > 1e-3f) ? (dist_true_m / dist_meas) : 1.0f;
  g_scale.acc_scale_x = scale;

  mqtt_printf("Trans", "Measured=%.3f m, True=%.3f m, acc_scale_x=%.6f",
              dist_meas, dist_true_m, g_scale.acc_scale_x);
  mqtt_printf("Trans", "=> Apply: corrected_acc = (measured_acc - bias) * acc_scale_x");

  // Structured JSON result (facilitates parsing on your UI)
//   {
//     char payload[224];
//     snprintf(payload, sizeof(payload),
//              "{\"type\":\"trans_result\",\"tag\":\"Trans\","
//              "\"dist_meas_m\":%.3f,\"dist_true_m\":%.3f,"
//              "\"acc_scale_x\":%.6f}", dist_meas, dist_true_m, g_scale.acc_scale_x);
//     connection.publish(mqtt_topic_connection, payload);
//   }
}

// --- Utility ---
static void print_sample() {
  mqtt_printf("Sample",
              "t=%lu ms | acc(%.3f %.3f %.3f) m/s^2 | gyro(%.3f %.3f %.3f) rad/s | q(%.4f %.4f %.4f)",
              (unsigned long)imu_data.timestamp_ms,
              imu_data.acc_x, imu_data.acc_y, imu_data.acc_z,
              imu_data.omega_x, imu_data.omega_y, imu_data.omega_z,
              imu_data.qx, imu_data.qy, imu_data.qz);

  // Optional: send structured JSON for dashboards
//   char payload[256];
//   snprintf(payload, sizeof(payload),
//            "{\"type\":\"sample\",\"timestamp_ms\":%lu,"
//            "\"acc_x\":%.3f,\"acc_y\":%.3f,\"acc_z\":%.3f,"
//            "\"gyro_x\":%.3f,\"gyro_y\":%.3f,\"gyro_z\":%.3f,"
//            "\"qx\":%.4f,\"qy\":%.4f,\"qz\":%.4f}",
//            (unsigned long)imu_data.timestamp_ms,
//            imu_data.acc_x, imu_data.acc_y, imu_data.acc_z,
//            imu_data.omega_x, imu_data.omega_y, imu_data.omega_z,
//            imu_data.qx, imu_data.qy, imu_data.qz);
//   connection.publish(mqtt_topic_connection, payload);
}

static void print_menu() {
  mqtt_printf("Menu", "=== IMU CALIBRATION MENU ===");
  mqtt_printf("Menu", "1 <duration_ms>  : bias while static (e.g. 1 8000)");
  mqtt_printf("Menu", "2 <angle_deg>    : gyro test (e.g. 2 90)");
  mqtt_printf("Menu", "3 <distance_m>   : translation test (e.g. 3 1.20)");
  mqtt_printf("Menu", "p                : print one IMU sample");

  // Optional: structured menu JSON for UI
//   const char* menu_json =
//     "{\"type\":\"menu\",\"commands\":["
//     "{\"cmd\":\"1\",\"desc\":\"bias while static (e.g. 1 8000)\"},"
//     "{\"cmd\":\"2\",\"desc\":\"gyro test (e.g. 2 90)\"},"
//     "{\"cmd\":\"3\",\"desc\":\"translation test (e.g. 3 1.20)\"},"
//     "{\"cmd\":\"p\",\"desc\":\"print one IMU sample\"}"
//     "]}";
//   connection.publish(mqtt_topic_connection, menu_json);
}

static void handle_command(const String& line) {
  if (line.length() == 0) return;
  if (line == "p") { imu.readAndUpdate(); print_sample(); return; }

  char cmd = line.charAt(0);
  float val = 0.f;
  if (line.length() > 1) val = line.substring(1).toFloat();

  switch (cmd) {
    case '1': {
      uint32_t dur = (val > 0.f) ? (uint32_t)val : 6000u;
      measure_bias(dur);
      break;
    }
    case '2': {
      float angle_deg = (fabsf(val) > 0.f) ? val : 90.f;
      test_gyro_turn(angle_deg);
      break;
    }
    case '3': {
      float dist_m = (fabsf(val) > 0.f) ? val : 1.0f;
      test_translation_1d(dist_m);
      break;
    }
    default:
      print_menu();
      break;
  }
}

// === Test entry points ===
void setup_imu_calibration() {
  Serial.begin(115200);
  delay(200);

  connection.setupWifi();
  
  mqtt_printf("IMU", "Calibration and accuracy test started");  
  print_menu();

  // Optional: initialize IMU here if not already done.
  for (int i=0;i<20;i++) { imu.readAndUpdate(); delay(10); }
}

/** Complete line with serial command:
 *   1 <duration_ms>  : measure bias while static (e.g. "1 8000")
 *   2 <angle_deg>    : gyro test; enter true rotation angle (e.g. "2 90")
 *   3 <distance_m>   : translation test; enter true distance (e.g. "3 1.20")
 *   p                : print last IMU sample
 */
void loop_imu_calibration() {
  connection.check_connection();

  String line = "";
  if (line != "") handle_command(line);
  imu.readAndUpdate();
  delay(1);
}