# ESP-1: Mapping & Global Planning Module
ESP-1 is responsible for the **global understanding** of the robot's environment and mission. It maintains the authoritative map, tracks the robot's global position, and makes high-level navigation decisions. Operating at lower frequencies (1-10 Hz) but with higher computational complexity.

>**Core Mission**: Build accurate maps, know where we are globally, decide where to go next.


---

## Pipeline
**Data Flow**: The entire process is a continuous loop on the ESP-1, primarily governed by the slow-but-accurate EKF-SLAM correction phase.

#### 1. The Starting Point: LiDAR Data

- **Module**: LiDAR Hardware / Pre-processing Module

- **Input**: Raw LiDAR distance and angle readings (from RPLIDARC1).

- **Process**: The raw scan is processed to identify and extract distinctive environmental features (e.g., corners, endpoints, high-confidence reflective surfaces).

- **Output**: `std::vector<LiDARLandmark>` (A list of range, angle, and quality for each detected feature).

#### 2. The Localization Core: EKF_SLAM

The EKF_SLAM is where the car figures out where it is and what it is looking at.

**Phase A**: Prediction (Driven by Motion)

- **Input**: Aggregated Raw Motion Deltas (OdometryData and IMUData) from the ESP-2 board.

- **Process**: The EKF uses the motion model (e.g., non-holonomic drive model) to:

1. Advance the robot's current pose $(x_r​,y_r,\theta_r)$.

2. Use the Jacobians to update the positions of all existing landmarks in the map, based on the predicted robot motion.

3. Significantly increase the Covariance Matrix $(P)$, reflecting the new uncertainty added by motion (slippage, gyro noise).

**Phase B**: Correction (Driven by Observation)

- **Input**: The `std::vector<LiDARLandmark>` from the pre-processing module.

- **Process**:
     1. **Data Association** (Mahalanobis Gate): Each new observation $z_{new}$ is compared against every existing landmark $L_j$ in the **Landmark Map** using the Mahalanobis Distance.
     2. **Matching**: If a match is found, the observation is used to calculate the Kalman Gain. The EKF then performs the correction, adjusting the pose and the landmark's position, which **shrinks the covariance** ($P$).
     3. **Initialization**: If no match is found, the observation is initialized as a **new landmark** and added to the **Landmark Map** (the EKF's extended state vector).
- **Output**: The single, best, globally corrected `Pose2D` $(x_{global},y_{global},\theta_{global})$.

#### 3. Map Building and Planning

The corrected pose is immediately used by **two parallel processes**: the map builder and the loop detector.

**A. Bayesian Occupancy Grid Update**
- **Input**:
     - The Corrected Global Pose from `EKF_SLAM`.
     - The full, raw LiDAR Scan data (distances and angles).

- **Process**: The **BayesianOccupancyGrid** module:
     1. Uses the Global Pose to accurately determine the origin and orientation of the LiDAR scan in the global coordinate frame.
     2. Applies a **ray-casting** algorithm to project the LiDAR beams onto the grid.
     3. Uses **Bayes' theorem** to probabilistically update the log-odds ratio of each cell being occupied, integrating certainty over time and smoothing out transient noise.

- **Role**: This grid is the dense, persistent map used by the A* Global Planner to find collision-free paths to the mission goal.

**B. Loop Closure Detection**
- **Input**:
     - The **Corrected Global Pose** from EKF_SLAM.
     - Keyframes/Sub-maps created by the EKF (representing past robot poses).

- **Process**: The LoopClosure module periodically checks if the robot has returned to a previously visited area (place recognition). If a match is detected:
     1. It calculates the cumulative long-term drift between the current pose and the old pose (the "pose graph residual").

     2. It uses a separate, global optimization method (like g2o or a simple least-squares adjustment) to calculate the necessary correction.

- **Output**: A single `LoopClosureCorrection` message $(\Delta x,\Delta y,\Delta \theta)$ is sent back to the ESP-2 to reset its `EKFLocalizer` frame, ensuring the low-latency control also aligns with the global map correction.
Transmission: UART (ESP-1 → ESP-2)

**C. MissionPlanner**
The Mission Manager module is responsible for:
- **Goal Selection**: Deciding the next highest-priority task (e.g., Explore an unmapped region, Navigate to a specific waypoint, or Return Home).

- **State Management**: Tracking the robot's current mode (Exploring, Path Following, Idle, Error).

- **Error Handling**: Detecting failures (e.g., Global Planner fails to find a path) and transitioning to recovery states.

It acts as the intermediary between the high-level system goal (e.g., "Map the entire area") and the low-level planner (A*).
- **Input**:
     - **Current Global Pose** (x,y,θ): From EKF_SLAM
     Needed to check if the robot has reached its current target.
     - **Map Coverage Status**: From Bayesian Occupancy Grid
     Used to determine if there are unexplored areas left. This dictates the shift to the `EXPLORATION_NODE` goal type.
     - **Global Planner Failure Status**: From System Status
     If A* returns an empty path, the manager switches to a recovery behavior (e.g., rotate and replan, or declare mission failed).
- **Output**: `MissionGoal` it's packaged with coordinates, defining the target for the `GlobalPlanner`.

**D. GlobalPlanner (A\*)**

- **Input**: The Bayesian Occupancy Grid (the map) and the Mission Goal (the target coordinates).

- **Process**: The `GlobalPlanner` runs the **A\*** search algorithm on the grid, finding the shortest, collision-free route from the robot's current EKF pose to the target.

- **Output**: A list of coordinates, packaged as a `GlobalPathMessage` (a vector of `Waypoint` structs).
Note: The `GlobalPlanner` hands this path message to the UART Sender task on ESP-1. The UART Receiver task on ESP-2 receives the path and passes it to the Local Planner (Pure Pursuit) module.



---

## architecture diagram
//TODO: UPDATE THIS DIAGRAM.
Here is the current architecture.
![alt text](/docs/global-architecture-2.png)