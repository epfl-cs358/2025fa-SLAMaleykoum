
🏠 [Backlink to the main README](/README.md)

# ESP-1: Mapping & Global Planning Module
ESP-1 is responsible for the **global understanding** of the robot's environment and mission. It maintains the authoritative map, and makes high-level navigation decisions. Operating at lower frequencies (1-10 Hz) but with higher computational complexity.

>**Core Mission**: Build accurate map depending on the position given by ESP-2, decide where to go next.

## architecture diagram
Here is the current architecture.
![ESP-1 Diagram](/assets/docs/esp1/esp1-diagram.png)

---

## Hardware
### Lidar :
- **Goal**: Extract the LiDAR points from the Serial2 buffer and convert them into readable data for Bayesian Grid.

- **Input**: Raw LiDAR data bytes (from RPLIDARC1).

- **Process**: Each raw point of 5 bytes is converted into a LiDAR point containing : an angle (in degrees), a distance (in millimeters), a quality (between 0 and 255).
Then, we put all the points of a "scan" (which corresponds to a 360° turn of the LiDAR) into a `LiDARScan`, which contains each points, the count of the scan and a timestamp to know when the scan has been calculated.

- **Output**: `LiDARScan` with a maximum of 500 points.

## Mapping
### The Bayesian Grid :

The **BayesianOccupancyGrid** class implements a 2D log-odds occupancy grid for building a persistent map of the environment from LiDAR data on an ESP32.

Each grid cell stores an **int8 log-odds** value, clamped between predefined bounds, representing the belief that the cell is occupied.

**Bayesian Occupancy Grid Update**
- **Input**:
     A `SyncedScan` containing :
     - The Global `Pose2D` of the car from ESP-2.
     - A full `LiDARScan` data (distances, angles and qualities).

- **Strict input validation**:
     - Rejects NaN or infinite pose, angles, or coordinates
     - Ensures bounded angle normalization
     - Skips updates if the robot pose is outside valid grid bounds

- **Ray tracing**:
     - Applies a Bresenham line algorithm from the robot position to the hit point
     - Traversed cells are updated as free space
     - The final hit cell is updated as occupied only if the measurement is within valid range

- **Log-odds update**:
     - Free cells: additive negative log-odds update
     - Occupied cells: additive positive log-odds update
     - Values are clamped to predefined minimum and maximum limits

- **Real-time safety**:
     - A watchdog aborts the update if processing exceeds a fixed time budget (100 ms)

- **Output**: 
     - A grid of size `size_x × size_y` and of resolution `grid_resolution` (meters per cell). This grid is the dense, persistent map used by the Global Planner to find collision-free paths to the mission goal.

## Planning
### Mission Planner :
The **MissionPlanner** module selects high-level exploration goals based on the current Bayesian occupancy grid and robot state.
It bridges the gap between mapping and global path planning.

- **Finding the frontiers**: 
     - A **frontier** is defined as a free cell (`p ≤ FREE_BOUND_PROB`) that has at least one unknown neighbor (`FREE < p < OCC`).
     - Frontier detection uses 4-connected neighbors.
     - The planner first searches for frontiers near the robot, then expands to the full map if none are found.

- **Frontier Clustering**:
     - Adjacent frontier cells are grouped into clusters using a BFS algorithm.
     - Each cluster stores:
          - Size
          - Centroid
          - First discovered frontier cell (fallback)
     - Clusters smaller than `MIN_CLUSTER_SIZE` are discarded.

- **Goal Selection**: 
     - Among valid clusters, the planner selects the closest sufficiently large cluster.
     - If the cluster centroid lies in an unsafe or unknown cell, the planner falls back to the first frontier point.
     - A safety padding step ensures the final goal is surrounded by free space, accounting for the robot’s physical radius

- **Safety and Validity Checks**:
     - Goals are rejected if:
          - They overlap occupied or unknown cells
          - They violate robot clearance constraints
          - They are listed in the invalid-goal blacklist
     - The planner continuously verifies whether the current goal is still valid as the map evolves.

- **Failure Handling and State Management**:
     - The planner operates in two modes:
          - `EXPLORATION_MODE` (default)
          - `RETURN_HOME`
     - If no valid frontiers are found:
          - The planner waits and retries for a limited number of cycles
          - A patience counter prevents premature failure at startup
     - After repeated failures, the planner switches permanently to `RETURN_HOME`

- **Output**:
     - The planner outputs a MissionGoal, containing:
     - Target pose (world coordinates)
     - Mission type
     - This goal is consumed by the GlobalPlanner for path computation.

**D. GlobalPlanner (A\*)**

The **GlobalPlanner** module computes a global, grid-based path on a Bayesian occupancy grid using a constrained A*-style search.
It is designed to be deterministic, memory-safe, and suitable for real-time embedded systems.

- **Input**: The `BayesianOccupancyGrid`, the `MissionGoal` (the target coordinates) and the `Pose2D` of the car. The `GlobalPlannerWorkspace` is a pre-allocated memory buffer that stores A* state.

- **Process**: 
     1. Coordinate conversion
          - Start and goal poses are converted from world coordinates to grid indices.
          - Planning aborts if either lies outside the grid or on invalid cells.
     2. A*-style search
          - Search is performed on a 4-connected grid (no diagonal motion).
          - Movement cost g(n) is uniform for each grid step.
          - Heuristic h(n) is a weighted Manhattan distance, favoring straighter paths over strict optimality.
          - The heuristic is intentionally non-admissible for faster convergence.
     3. Collision and clearance handling
          - Cells are rejected if occupied or unknown.
          - Additional safety buffering is enforced: a cell is considered invalid if any of its 8 neighbors is occupied or unknown.
          - This implicitly inflates obstacles to preserve clearance around walls.
     4. Search limits
          - The search enforces strict limits on:
               - Maximum grid size
               - Priority queue size
               - Total A* iterations
          - If any limit is exceeded, planning fails gracefully.

- **Path Reconstruction**:
     - Parent indices are stored during search.
     - The path is reconstructed from goal to start and reversed.
     - If the full path exceeds `MAX_PATH_LENGTH`, it is downsampled while preserving start and end points.

- **Output**: 
     - A list of coordinates, packaged as a `PathMessage` (a vector of `Waypoint` structs).
     - If no valid path is found, an empty path message is returned.
     - The path is forwarded to the UART sender task on ESP-1.