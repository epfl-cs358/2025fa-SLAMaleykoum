# ESP-1: Mapping & Global Planning Module
ESP-1 is responsible for the **global understanding** of the robot's environment and mission. It maintains the authoritative map, and makes high-level navigation decisions. Operating at lower frequencies (1-10 Hz) but with higher computational complexity.

>**Core Mission**: Build accurate maps, know where we are globally, decide where to go next.


---

## Hardware
### Lidar :
#### 1. The Starting Point: LiDAR Data

- **Module**: LiDAR Hardware / Pre-processing Module

- **Input**: Raw LiDAR distance and angle readings (from RPLIDARC1).

- **Process**: The raw scan is processed to identify and extract distinctive environmental features (e.g., corners, endpoints, high-confidence reflective surfaces).

- **Output**: `std::vector<LiDARLandmark>` (A list of range, angle, and quality for each detected feature).

## Mapping
### The Bayesian Grid :
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

## Planning
### Mission Planner :
The Mission Manager module is responsible for:
- **Finding the frontiers**: Selecting all the frontieres of the map. We begin by searching close to the robot and if nothing is found we extend the research to the whole map. These are the limits between what we have explored and what needs to be visited. Visually it corresponds to the white cells that have at least one gray cell as neighbour. 

- **Goal Selection**: Finding the next region to explore depending on the frontieres. We regroup the ones that are close to each other in clusters using a BFS algorithm. Then we select the closest cluster that is big enough.

- **State Management**: Tracking the robot's current mode (`EXPLORATION_MODE` or `RETURN_HOME`). It is in `EXPLORATION_MODE` by default, and changes to return home when no more frontiers are left unexplored. Once in the `RETURN_HOME` mode it stays this way, unless it is changed manually via the `set_mission_state` function.

It acts as the intermediary between the high-level system (Bayesian grid) and the low-level planner (A*).

- **Output**: `MissionGoal`, it is packaged with coordinates, defining the target for the `GlobalPlanner`.


### Global Planner :
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