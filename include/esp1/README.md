# ESP-1: Mapping & Global Planning Module
ESP-1 is responsible for the **global understanding** of the robot's environment and mission. It maintains the authoritative map, and makes high-level navigation decisions. Operating at lower frequencies (1-10 Hz) but with higher computational complexity.

>**Core Mission**: Build accurate map depending on the position given by ESP-2, decide where to go next.


---

## Hardware
### Lidar :
- **Goal**: Extract the LiDAR points from the Serial2 buffer and convert them into readable data for Bayesian Grid.

- **Input**: Raw LiDAR data bytes (from RPLIDARC1).

- **Process**: Each raw point of 5 bytes is converted into a LiDAR point containing : an angle (in degrees), a distance (in millimeters), a quality (between 0 and 255).
Then, we put all the points of a "scan" (which corresponds to a 360° turn of the LiDAR) into a `LiDARScan`, which contains each points, the count of the scan and a timestamp to know when the scan has been calculated.

- **Output**: `LiDARScan` with a maximum of 100 points.

## Mapping
### The Bayesian Grid :

The **BayesianOccupancyGrid** class implements a 2D probabilistic occupancy grid used to build a persistent map of the environment from LiDAR data.
Each grid cell stores a log-odds representation of the probability of being occupied, allowing robust incremental updates over time.

Each cell represents a square area of the world with a given metric resolution (meters per cell) and stores:
- Low probability → free space
- High probability → obstacle

This module is designed to run on an ESP32, with strict memory and performance constraints.

**Bayesian Occupancy Grid Update**
- **Input**:
     A `SyncedScan` containing :
     - The Global `Pose2D` of the car from ESP-2.
     - A full `LiDARScan` data (distances, angles and qualities).

- **Process**: The **BayesianOccupancyGrid** module:
     1. Uses the Global Pose to accurately determine the origin and orientation of the LiDAR scan in the global coordinate frame.
     2. Grid borders are initialized as occupied in order to guarantee:
     - Safe behavior for planners
     - No out-of-bounds exploration
     3. Applies a **Bresenham ray tracing** algorithm to project the LiDAR beams onto the grid. All traversed cells are converted to free cells.
     4. Uses **Bayes' theorem** to probabilistically update the log-odds ratio of each cell being occupied, integrating certainty over time and smoothing out transient noise.

- **Output**: A grid of size `size_x × size_y` and of resolution `grid_resolution` (meters per cell). This grid is the dense, persistent map used by the Global Planner to find collision-free paths to the mission goal.

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

The **GlobalPlanner** module computes a global, collision-free path on a Bayesian occupancy grid using the A* search algorithm.
It is designed to be deterministic, memory-safe, and compatible with real-time constraints.

- **Input**: The `BayesianOccupancyGrid`, the `MissionGoal` (the target coordinates) and the `Pose2D` of the car. The `GlobalPlannerWorkspace` is a pre-allocated memory buffer that stores A* state.

- **Process**: The robot pose and mission goal are converted from world coordinates to grid indices. The `GlobalPlanner` runs the **A\*** search algorithm on the grid, finding the shortest, collision-free route from the robot's current pose to the target.
We've got two functions that we sum to know what is the better path to follow : 
     - g(n) → accumulated path cost
     - h(n) → Euclidean distance to the goal (admissible heuristic)

- **Output**: A list of coordinates, packaged as a `PathMessage` (a vector of `Waypoint` structs).
Note: The `GlobalPlanner` hands this path message to the UART Sender task on ESP-1.

---

## architecture diagram
//TODO: UPDATE THIS DIAGRAM.
Here is the current architecture.
![alt text](/docs/global-architecture-2.png)