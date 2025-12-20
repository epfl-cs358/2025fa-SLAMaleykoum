## Known Issues & Recommendations

### Common ESP1 & ESP2

#### Hardware

**Hardware wear and wiring issues:**

Reused components had weak solder joints and loose connectors, causing intermittent failures (especially the encoder).

**Fix**: Replaced fragile connectors with screw terminal blocks → stable signals.

**Recommendation**: Inspect wiring early, re-solder weak joints, standardize connectors, and consider a small PCB for reliability.

#### Software

**Wi-Fi communication issues:**

MQTT introduced significant latency and was unable to reliably transmit the full Bayesian map, while TCP streams sometimes produced incomplete or corrupted data.

**Fix**: MQTT was used only for basic hardware debugging due to its simplicity. For debugging and transmitting complex algorithms and large data structures, a TCP server was implemented and a custom Python client was used to retrieve and visualize data from the ESP32. When errors occurred in the final map, task priorities were reviewed and mutexes were added to protect shared resources in FreeRTOS.

**Recommendation**: Consider using UDP instead of TCP when occasional packet loss is acceptable, but ensure proper packet structuring, ordering, and validation to maintain data consistency.

### ESP1-Specific Issues

#### Hardware

**LiDAR serial initialization:**

Difficulty starting the LiDAR due to incorrect `RX/TX pin` configuration.

**Fix**: Ensured the ESP32 RX pin in the code matched the physical LiDAR TX connection.

**Recommendation**: Clearly document pin assignments and validate serial communication with minimal test code first. If the pins you selected don't work, it might be that they are broken on the ESP. Try other pins RX/TX.

#### Software

**High LiDAR data throughput:**

MQTT could not handle the volume of scan data, causing delays and packet loss.

**Fix**: Configured the ESP32 as a Wi-Fi access point with a TCP server and streamed data to a Python client for real-time visualization.

**Recommendation**: Evaluate data bandwidth early and choose TCP/UDP streaming over MQTT for high-frequency sensor data.

---

**Latency and incomplete scans:**

Attempting to process every LiDAR point introduced lag and instability.

**Fix**: Implemented downsampling (one point every 5, or one scan every 3), maintaining accuracy while reducing load.

**Recommendation**: Always include configurable downsampling and monitor real-time latency.

---

**LiDAR buffer decoding and angle conversion:**

Incorrect parsing and degree/radian mismatches caused distorted maps.

**Fix**: Carefully decoded LiDAR buffers and standardized angle units throughout the pipeline.

**Recommendation**: Check the documentation of your LiDAR, centralize unit conversions and validate data using simple geometric test cases.

---

**Bayesian occupancy grid performance:**

Grid computation was initially too heavy for the ESP32.

**Fix**: Optimized grid size and resolution to match the Python visualization and fit ESP32 memory constraints.

**Recommendation**: Design mapping resolution based on hardware limits and test memory usage early.

---

**Mission Planner boundary detection:**

Defining meaningful exploration boundaries was ambiguous.

**Fix**: Defined boundaries as white cells adjacent to gray cells, clustered them, and selected the closest valid cluster (≥8 cells).

**Recommendation**: Use simple, well-defined heuristics for boundary detection and validate them visually.

---

**Global Planner (A*) efficiency:**

For the **Global Planner**, the problems encountered were mainly related to the efficiency of the **A-Star algorithm**. We limited the number of waypoints to a maximum of 5 and evenly distributed them along the found path. We decided to use a heuristic function with the distance from the current point to the goal to find the optimal path as quickly as possible.

Unrestricted A* planning was computationally expensive.

**Fix**: Boost the heuristic function to search the path

**Recommendation**: If your ESP32 can compute it, add a Local Planner to make it more precise when it detects smaller objects

### ESP2-Specific Issues

#### Hardware

**Encoder jitter:**

The encoder produced noisy tick timing due to electrical noise, slight mechanical misalignment, and (before switching connectors) intermittent contact. This caused unstable velocity estimates, especially at higher speeds.

**Fix**: The AS5600 library (Rob Tillaart) was used to reject obviously invalid readings. While it does not smooth data, it prevented major miscalculations and improved low-speed velocity estimation.

**Recommendation**: Sample encoder at high, consistent rates; Use screw terminal blocks. Consider a wheel-mounted encoder for higher effective resolution (gearbox ratio ≈ 1:10 reduces motor-shaft resolution at the wheel).

---

**Motor PID control not feasible:**

The stock THW-1060-RTR ESC only provides discrete throttle steps. Small PWM changes often produced no speed change, then suddenly jumped to a higher speed. This made PID velocity control impossible, hence the controller oscillated and never converged.

**Fix**: PID control was abandoned; the car was operated at a constant low throttle.

**Recommendation**: Replace the ESC with an IBT-4 (BTS7960) motor driver for smooth PWM.

---

**IMU acceleration unreliable for odometry and EKF:**

Drift and bias made acceleration unreliable for odometry/EKF fusion.

**Fix**: Acceleration was deprioritized in favor of encoder-based velocity estimation.

**Recommendation**: Perform proper IMU calibration, apply bias estimation and low-pass filtering, and rely more heavily on gyroscope + encoder fusion rather than raw acceleration.

#### Software

**Odometry breakdown at higher speeds:**

At higher speeds, the combination of encoder jitter, IMU drift, and discrete ESC throttle steps caused odometry to diverge. The car eventually lost track of its position.

**Fix**: No full fix was possible with the existing hardware. The car was limited to one of its slowest stable speeds.

**Recommendation**: Verify whether the odometry task is running fast enough and with high priority.

---

**IMU yaw jump filtering:**

The IMU occasionally produced sudden yaw jumps, which corrupted the estimated pose sent to ESP1 and affected how the Bayesian map was built.

**Fix**: A yaw-stability filter was implemented: ESP2 only stores and transmits its position if the yaw change is ≤ 15° between updates. This prevented corrupted odometry packets from polluting the global map.

**Recommendation**: Always include sanity checks on IMU orientation.


## Troubleshooting

If you encounter issues, check the list below before reaching out.

| Problem | Possible Cause | Solution |
| :--- | :--- | :--- |
| **Map has "fuzzy" walls** | The environment might contain windows, or reflective surfaces, maybe even the obstacles are too thin or the walls have holes in them (mesh / fence) | Since the lidar can't correctly detect those kinds of objects, try mapping something else |
| **WiFi doesn't appear** | You cannot flash the ESP32 from the left port, hence the code was never uploaded OR you might have uploaded the code of the ESP2 on the ESP1, it happens way too often | Flash the ESPs again, with their respective code on their RIGHT port (In our setup, it's the center ports, since the ESP1 is upside-down) |
| **Wheels turn, car doesn't move** | The motor doesn't get any power | The ESC must be switched on immediately after connecting the battery |
| **ESP doesn't initialize or crashes often** | Cables might have come loose | Double check the pin connections |
| **ESP keeps crashing** | Battery might be too low | Check the battery level, charge it |
| **Map is not displaying but python code is up** | Incoherent map sizes | Double check the values in the `ground_station.py` file and the rest of the code, for the size of the map (grid size) and their max bounds are the same |
| **ESP1 crashes immediately** | The map size is too big | Reduce the map size. Note: The max nb of cells we managed to run with is 70x70 but if the real world size is not enough, you can increase the `RESOLUTION` value which will increase what each cell represents in the real world |

### Hardware Issues

**Symptom: Wheels spin but car doesn't move**

**Causes:**
1. ESC not powered
2. Battery voltage too low
3. Motor pinion gear not meshing

**Fixes:**
1. Turn ESC ON immediately after battery connection
2. Charge battery (check voltage >6.5V)
3. Adjust motor mount (should hear gear engagement)

---

**Symptom: Servo jitters or doesn't respond**

**Causes:**
1. Insufficient power (voltage drop)
2. PWM signal noise
3. Loose servo horn

**Fixes:**
1. Check buck converter output (should be 5V ±0.2V)
2. Add capacitor (100μF) across servo power pins
3. Tighten servo horn screw

---

**Symptom: Encoder reads constant zero**

**Causes:**
1. Magnet too far from sensor (>3mm gap)
2. I2C connection issue
3. Wrong I2C address

**Fixes:**
1. Adjust encoder mount (2-3mm gap optimal)
2. Verify SDA/SCL connections, check pull-up resistors
3. Run I2C scanner, confirm 0x36 address

---

**Symptom: ESP32 frequently crashes**

**Causes:**
1. Insufficient power
2. Stack overflow (task stack too small)
3. Watchdog timeout

**Fixes:**
1. Check battery voltage, verify buck converter stable
2. Increase `STACK_SIZE_*` in config
3. Add `vTaskDelay()` in long loops

### Software Issues

**Symptom: Map shows fuzzy/doubled walls**

**Causes:**
1. Reflective surfaces (glass, mirrors)
2. LiDAR seeing through thin obstacles
3. Odometry drift

**Fixes:**
1. Map in environment without windows
2. Avoid mesh fences, thin curtains
3. Recalibrate encoder, check IMU mounting

---

**Symptom: Robot ignores path commands**

**Causes:**
1. UART connection lost
2. Path message corrupted
3. Pure Pursuit stuck

**Fixes:**
1. Verify TX/RX wiring, check baud rate match
2. Enable CRC checking in ESP Link
3. Add timeout to reset Pure Pursuit state

### Emergency Procedures

**Robot stuck in infinite loop:**
1. Press emergency stop button (if installed)
2. Turn off ESC switch
3. Disconnect battery
4. Reflash ESP2 with safety limits

**Robot drives into obstacle despite ultrasonic:**
1. Check ultrasonic connections
2. Verify `EMERGENCY_THRESHOLD` not too small
3. Test sensor in isolation (upload test code)

**Cannot reflash ESP32:**
1. Try different USB cable (data, not charge-only)
2. Hold BOOT button while connecting USB
3. Use esptool.py directly:
   ```bash
   esptool.py --port /dev/ttyUSB0 erase_flash
   ```
