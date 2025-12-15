// Filename: pure-pursuit.cpp
// Description: Implementation of the Pure Pursuit path tracking algorithm.
//      Converts a series of waypoints into steering and velocity commands.


// Note: The waypoints are given in global coordinates, they're ordered from start to goal.

// Note: Lookahead distance is a funciton of speed, higher speed = larger lookahead distance.
//      This helps smooth the path following at higher speeds.
//      Speed is a function of the steering angle required to reach the waypoint. Larger steering
//      angles = lower speed (slow down to turn, avoid slipping).

// ---

#include <cstring>
#include <cmath>
#include "common/data_types.h"
#include "common/transforms.h"
#include "esp2/control/pure_pursuit.h"

template <typename T>
T clamp(T value, T minVal, T maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

// Calculates the squared Euclidean distance between the current pose and a waypoint.
float PurePursuit::get_dist_sq(const Pose2D& pose, const Waypoint& wp) const {
    float dx = wp.x - pose.x;
    float dy = wp.y - pose.y;
    return dx * dx + dy * dy;
}

// Note: Since we're constanly re-computing the path from the current pose to the goal,
//      we can just overwrite the current path even if we're in the middle of following one.
//      It's assumed that the new path is always better than the old one. (e.g. discovered a "cul de sac",
//      or in further versions, dynamic obstacles).

void PurePursuit::set_path(const PathMessage& path_msg) {
    path_length_ = path_msg.current_length;

    // Perform a fast, direct memory copy. This is efficient and avoids dynamic allocation.
    std::memcpy(current_path_, path_msg.path, path_length_ * sizeof(Waypoint));
        
    // IMPORTANT: Reset the lookahead index. When a new path is received, 
    // we must start tracking from the beginning of this new path.
    last_target_index_ = 0; 
}

/**
 * @brief Computes the motion command (speed and steering angle) using the Pure Pursuit algorithm.
 * @param current_pose The robot's current position and orientation.
 * @param vel The robot's current velocity.
 * @return The computed MotionCommand containing speed (in m/s) and steering angle (in radians).
 */
MotionCommand PurePursuit::compute_command(const Pose2D& current_pose, const Velocity& vel) {
    // If no path to follow, or if we've reached the end of the path, stop.
    if (path_length_ == 0 || is_path_complete(current_pose)) {
        if (path_length_ > 0) {
            // Mark as "at goal" if the path exists
            last_target_index_ = path_length_ - 1; 
        }
        return {0.0f, 0.0f}; // Return STOP command
    }

    // ---
    // This part is commented out since we don't have a useful PID for the car speed yet.
    // Check out issue #42 for more details.
    // For now we'll use a fixed lookahead distance. (Defined in .h as Ld_fixed_)

    // // Calculate Adaptive Lookahead Distance
    // float Ld = calculate_lookahead_distance(vel.v_linear);
    // ---

    // Find the Lookahead Target Point (in GLOBAL frame)
    Waypoint target_point_global = find_lookahead_point(current_pose, Ld_fixed_);
    
    // Create a copy to hold the robot-frame coordinates
    Waypoint target_point_robot = target_point_global;
    // Transform to Robot Frame
    Transforms::to_robot_frame(current_pose, target_point_robot);

    // Calculate curvature / Steering
    // Pure Pursuit logic: curvature = 2 * x / Ld^2 (Standard frame)
    // L_ is the car's wheelbase. NOTE: the LATERAL component is target_point_robot.x
    float curvature = k_p * (2.0f * target_point_robot.x) / (Ld_fixed_ * Ld_fixed_);
    
    // delta = atan(curvature * wheelbase)
    float delta_target = std::atan(curvature * L_);

    // ---
    // This part is commented out since we don't have a useful PID for the car speed yet.
    // For now we'll just use a fixed target speed.
    // (The speed is fixed in the .h as fixed_speed_)
    // // Calculate Adaptive Target Speed
    // float v_target = calculate_target_speed(delta_target);
    // ---

    // Clamp the theoretical angle to the car's physical limits.
    float steering_command_rad = clamp(delta_target, 
                                            MIN_STEERING_ANGLE_RAD_, 
                                            MAX_STEERING_ANGLE_RAD_);

    // The servo takes angles in degrees from 0 to 180, where 90 is straight.
    float steering_to_servo = (steering_command_rad * (180.0f / M_PI)) + 90.0f;

    // return {v_target, steering_command_rad};
    return { /* Fixed Speed */ fixed_speed_, steering_to_servo};
}


/**
 * @brief Finds the exact point on the path segment that intersects the Lookahead Circle.
 * This creates a "Sliding Target" that moves smoothly as the car moves.
 */
Waypoint PurePursuit::find_lookahead_point(const Pose2D& current_pose, float Ld) {
    
    // --- STEP 1: Find Closest Point to enforce Forward Motion ---
    float min_dist_sq = 1e9f;
    Waypoint closest_point = current_path_[last_target_index_];
    uint16_t search_limit = std::min((uint16_t)(path_length_ - 1), (uint16_t)(last_target_index_ + 50));

    for (uint16_t i = last_target_index_; i < search_limit; ++i) {
        Waypoint start = current_path_[i];
        Waypoint end = current_path_[i + 1];
        
        float dx = end.x - start.x; float dy = end.y - start.y;
        float len_sq = dx*dx + dy*dy;
        float t = 0.0f;
        if (len_sq > 0) t = ((current_pose.x - start.x) * dx + (current_pose.y - start.y) * dy) / len_sq;
        t = clamp(t, 0.0f, 1.0f);
        
        float px = start.x + t * dx; float py = start.y + t * dy;
        float d_sq = (current_pose.x - px)*(current_pose.x - px) + (current_pose.y - py)*(current_pose.y - py);
        
        if (d_sq < min_dist_sq) {
            min_dist_sq = d_sq;
            closest_point = {px, py};
            last_target_index_ = i; // Enforce forward progress
        }
    }

    // --- STEP 2: Standard Lookahead Search (using new index) ---
    // Default to the closest point found above (Fallback for "Corner Cutting")
    Waypoint target_point = closest_point; 
    
    // We iterate from the last known target index to find the active segment
    for (uint16_t i = last_target_index_; i < path_length_ - 1; ++i) {
        Waypoint start = current_path_[i];
        Waypoint end = current_path_[i + 1];

        // --- Shift coordinate system to robot position ---
        // (Math is easier if robot is at 0,0)
        float dx = end.x - start.x;
        float dy = end.y - start.y;
        float fx = start.x - current_pose.x;
        float fy = start.y - current_pose.y;

        // --- Solve Intersection of Line and Circle ---
        // Line: P = start + t * (end - start)
        // Circle: x^2 + y^2 = Ld^2
        float a = dx*dx + dy*dy;
        float b = 2 * (fx*dx + fy*dy);
        float c = (fx*fx + fy*fy) - (Ld*Ld);

        float discriminant = b*b - 4*a*c;

        if (discriminant >= 0) {
            // We have an intersection!
            discriminant = std::sqrt(discriminant);
            
            float t1 = (-b - discriminant) / (2*a);
            float t2 = (-b + discriminant) / (2*a);

            // FIX: Instead of returning immediately, we update the target point
            // and CONTINUE the loop. This ensures we pick the valid intersection
            // with the HIGHEST index (furthest along the path).
            
            // Check t2 first (usually forward exit of the circle relative to segment)
            if (t2 >= 0.0f && t2 <= 1.0f) {
                target_point.x = start.x + t2 * dx;
                target_point.y = start.y + t2 * dy;
                // DO NOT return here!
            }
            // Fallback to t1
            else if (t1 >= 0.0f && t1 <= 1.0f) {
                target_point.x = start.x + t1 * dx;
                target_point.y = start.y + t1 * dy;
                // DO NOT return here!
            }
        }
    }

    // If no intersection found, this returns 'closest_point' (Step 1).
    // If intersection(s) found, this returns the one with the highest index (Step 2).
    return target_point;
}



// These functions are currently unused since we're using fixed lookahead distance and speed due to our ESC limitations.
float PurePursuit::calculate_lookahead_distance(float current_speed) const {
    // Linear relationship between speed and lookahead distance. (when going faster, look further ahead)
    float Ld = K_dd_ * current_speed;
    return clamp(Ld, min_lookahead_dist_, max_lookahead_dist_);
}

float PurePursuit::calculate_target_speed(float steering_angle) const {
    // Inverse relationship: Faster for smaller angles, slower for larger.
    float v_target = max_speed_ * (1.0f - K_v_ * std::abs(steering_angle));
    return clamp(v_target, min_speed_, max_speed_);
}

// Checks if the robot is within the goal tolerance of the final waypoint.
bool PurePursuit::is_path_complete(const Pose2D& current_pose) const {
    if (path_length_ == 0) {
        return true; // An empty path is "complete"
    }

    // Get the final waypoint
    const Waypoint& final_goal = current_path_[path_length_ - 1];
    
    // Check squared distance
    float dist_to_goal_sq = get_dist_sq(current_pose, final_goal);

    return dist_to_goal_sq < (goal_tolerance_ * goal_tolerance_);
}