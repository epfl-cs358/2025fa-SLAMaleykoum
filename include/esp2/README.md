# ESP-2: Real-Time Control & Actuation Module

ESP-2 is dedicated to real-time execution, low-latency control, and sensor management.
It operates at a high frequency (50-100 Hz) to ensure stable actuation and precise velocity tracking.

> **Core Mission**: Execute the global path plan stably, maintain smooth velocity control, and gather high-frequency sensor data for both local and global localization.

## Pipeline

**Data Flow**: The ESP-2 runs multiple independent FreeRTOS tasks to handle high-frequency sensor polling and decoupled communication with the ESP-1.

#### 1. Sensor Acquisition and Data Split

This stage is the Producer of all motion data, running continuously at the maximum possible hardware rate.

**Module**: Actuation Hardware Task (High Priority)
**Input**:
    - Raw Encoder Pulses (Left & Right wheels).
    - Raw 9-Axis IMU Readings (Gyro, Accelerometer, Magnetometer).

**Process**: Reads raw hardware counts and converts them into physical motion deltas ($\Delta s$, $\Delta \theta$, $a, \omega$).

**Data Split**: Writes the resulting structs to two parallel destinations:

- **Local (Control)**: The latest data overwrites a Mutex-Protected Shared Struct for low-latency EKF access.

- **Remote (SLAM)**: Every single data packet is pushed into a FreeRTOS Queue/Stream Buffer for aggregation and later transmission to ESP-1.

**Output**:
- **Path 1**: Updated Mutex-Protected Struct (OdometryData, IMUData).

- **Path 2**: Enqueued packets for the UART Sender Task.

#### 2. Local Localization Core: EKFLocalizer

This EKF runs at a high frequency (50-100 Hz) solely to provide the PID controller with a filtered, stable velocity estimate.

**Module**: EKFLocalizer Task

**Input**:
- **Motion Data**: The latest OdometryData $(\Delta s, \Delta \theta)$.

- **Correction Data**: Raw Accelerometer data $(a_x, a_y)$.

- **Alignment Data**: `LoopClosureCorrection` $(\Delta x, \Delta y, \Delta \theta)$ from ESP-1 (infrequent).

**Process**:

- **Prediction**: Fuses Odometry and Gyroscope ($\omega_z$) to track the predicted local **pose** and **velocity**.

- **Correction**: Uses the Accelerometer's lateral axis ($a_y$) to enforce the Non-Holonomic Constraint. This filters out instantaneous wheel slippage noise, resulting in a highly stable velocity estimate.

- **Alignment**: If a `LoopClosureCorrection` is received, the entire local pose state is nudged back into alignment with the global map, preventing long-term drift in the control execution.

**Output**: Corrected Local Pose and Velocity $(\mathbf{v}_{\text{linear}}, \mathbf{v}_{\text{angular}})$.

#### 3. Execution and Actuation Loop

This loop takes the plan from ESP-1 and executes it with high-frequency control stability.

**Module**: Local Planner (Pure Pursuit) & PID Controller

**Input**:

**Plan**: `GlobalPathMessage` (Vector of `Waypoints`) from ESP-1.

**Feedback**: Current Velocity $(\mathbf{v}_{\text{linear}}, \mathbf{v}_{\text{angular}})$ from `EKFLocalizer`.

**Process** (Pure Pursuit):
This path-tracking algorithm translates the static waypoints into dynamic steering commands.

- **Lookahead Point Selection**: The process determines a lookahead distance ($L_d$), which is a tunable parameter controlling the aggressiveness of the steering. The algorithm searches the `GlobalPathMessage` for the `waypoint` that is $L_d$ away from the robot's current `Pose`.

- **Curvature Calculation**: Once the lookahead point is identified, the core geometry calculates the curvature ($\kappa$) of the unique circular arc that connects the robot's current position to the lookahead point while respecting the robot's current orientation.

- **Command Generation**: This curvature is converted into the necessary **Target Angular Velocity** $(\mathbf{v}_{\text{angular}})$. The **Target Linear Velocity** $(\mathbf{v}_{\text{linear}})$ should be dictated by path constraints or a fixed speed limit (//TODO: tbd).


    $$\mathbf{v}_{\text{angular}} = \mathbf{v}_{\text{linear}} \times \kappa$$


    The output is the **Target Velocity Command**.

- **PID Control**: Uses the Target Velocity (Setpoint) and the EKF's Current Velocity (Process Variable) to calculate the necessary Left and Right motor PWM Signals. The EKF output is critical here for preventing jittery, noisy control.

**Output**: Motor PWM Signals (sent to motor drivers).

---

## architecture diagram
//TODO: UPDATE THIS DIAGRAM.
Here is the current architecture.
![alt text](/docs/global-architecture-2.png)