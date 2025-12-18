# Pure Pursuit – Local Path Tracker

## Overview

This module implements a **Pure Pursuit local path-following controller** designed to run on the ESP32.  
Its role is to convert a **short global path** (received from ESP1) into **low-level motion commands** that allow the vehicle to follow the path smoothly and reliably.

Due to hardware limitations of the ESC, the current implementation uses a **fixed target speed** and a **fixed lookahead distance**, focusing on robust steering computation rather than full dynamic control.

---

## Core Concept

The Pure Pursuit algorithm follows a path by continuously steering the vehicle toward a **lookahead point** located at a fixed distance ahead of the robot on the path.

The algorithm works as follows:

1. **Path Update**  
   A new path is frequently received from ESP1 and **completely overwrites** the previous one.  
   Since paths are always recomputed from the robot’s current position, no path continuity is required.

2. **Lookahead Point Selection**  
   - The algorithm searches forward along the path starting from the last valid index.
   - It computes the intersection between path segments and a **lookahead circle** centered on the robot.
   - If multiple intersections exist, the **furthest valid one** along the path is selected.
   - If no intersection is found, the closest point on the path is used as a fallback.

3. **Coordinate Transformation**  
   The selected lookahead point is transformed from the **global frame** to the **robot frame** using the `Transforms` utility.

4. **Steering Computation (Bicycle Model)**  
   The curvature is computed using the Pure Pursuit formula:
   $(\mathbf{curvature = 2 * x}_{\text{lateral}} \mathbf{/ Ld²})$
  where:
  - `x_lateral` is the lateral offset of the lookahead point in robot coordinates
  - `Ld` is the fixed lookahead distance    
  The steering angle is then computed as:   
  $(\mathbf{steering\_angle} \mathbf{= atan(curvature * L)})$      
  where `L` is the vehicle wheelbase.

5. **Command Output**  
- The steering angle is clamped to the physical limits of the vehicle.
- It is converted to **servo degrees** (`0–180`, with `90` as straight).
- A fixed forward speed is applied.

---

## Design Choices & Constraints

- **Fixed Speed and Lookahead**  
Adaptive speed and lookahead are disabled due to ESC limitations.

- **Forward Progress Enforcement**  
A sliding index ensures the robot always follows points ahead of its current position and never tracks backward along the path.

- **Continuous Path Refresh**  
The controller assumes paths are continuously updated and does not rely on explicit “path completed” events.

---

## Key Parameters

Defined in `pure_pursuit.h`:

- `L_` – Vehicle wheelbase (meters)
- `Ld_fixed_` – Fixed lookahead distance
- `fixed_speed_` – Fixed target speed
- `k_p` – Proportional gain applied to curvature
- `MIN_STEERING_ANGLE_RAD_ / MAX_STEERING_ANGLE_RAD_` – Steering limits
- `goal_tolerance_` – Distance threshold for goal detection
---

## Public API

### `void set_path(const PathMessage& path_msg)`

**Purpose**  
Loads a new path to follow.

**Behavior**
- Copies the received waypoints into an internal buffer
- Resets the lookahead index
- Overwrites any previously active path

---

### `MotionCommand compute_command(const Pose2D& current_pose, const Velocity& vel)`

**Purpose**  
Main control function, called at each control cycle.

**Inputs**
- `current_pose`: current robot pose from localization
- `vel`: current robot velocity (currently unused)

**Output**
- `MotionCommand` containing:
- Fixed target speed (m/s)
- Steering command in **servo degrees**

If no valid path exists or the goal is reached, a **stop command** is returned.

---

### `bool is_path_complete()`

Checks whether the robot is within a tolerance radius of the final waypoint.  
Currently not critical, as paths are continuously refreshed.

---

## Limitations & Future Improvements

- Replace fixed speed with proper **velocity PID control**
- Enable **adaptive lookahead distance**
- Add a **local obstacle-aware planner** on ESP2
- Improve steering smoothness and curvature-based speed modulation





