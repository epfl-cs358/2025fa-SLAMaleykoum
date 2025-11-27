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

void PurePursuit::set_path(const GlobalPathMessage& path_msg) {
    // Determine the number of points to copy, clamping to the max size.
    uint16_t length_to_copy = std::min((uint16_t)MAX_PATH_LENGTH, path_msg.current_length);

    // Perform a fast, direct memory copy. This is efficient and avoids dynamic allocation.
    std::memcpy(current_path_, path_msg.path, length_to_copy * sizeof(Waypoint));
    
    // Update the internal state variables.
    path_length_ = length_to_copy;
    
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
 * @brief Finds the waypoint on the path that is closest to the intersection 
 * of the lookahead distance circle and the path segment.
 * @param current_pose The robot's current position.
 * @param Ld The calculated lookahead radius.
 * @return The Waypoint structure of the lookahead target point.
 */
Waypoint PurePursuit::find_lookahead_point(const Pose2D& current_pose, float Ld) {
    // If the path is empty, return the current pose
    if (path_length_ == 0) {
        return {current_pose.x, current_pose.y};
    }

    // Start Search from the Last Target Index to avoid re-searching waypoints that have already been passed.
    for (uint16_t i = last_target_index_; i < path_length_ - 1; ++i) {
                
        // const Waypoint& w1 = current_path_[i];
        const Waypoint& w2 = current_path_[i + 1];

        // TODO: Last index used
        // Find the *last* waypoint that is within the lookahead distance.
        // Then, find the *first* waypoint beyond the lookahead distance. 

        // Distance from current pose to w2
        float dist_sq = get_dist_sq(current_pose, w2);
        
        // We're looking for the first point just outside the lookahead distance.
        // If the current waypoint is *well* within the lookahead distance, we've passed it.


        if (dist_sq < Ld * Ld * 0.5f) { 
            // We have passed this waypoint (i+1), so start the next search from it.
            last_target_index_ = i + 1;
        }

        // Look for a point *just outside* the lookahead distance.
        if (dist_sq >= Ld * Ld) {
            // Waypoint w2 is now outside Ld. Interpolate on the segment (w1, w2) to find the intersection.
            // This is the ideal target point calculation.
            
            last_target_index_ = i; // Update for next search
            // Simplified return: Just use w2 as the target point.            
            return w2; 
        }
    }
    
    // Edge Case:
    // If we reach the end of the path array and haven't found an intersection (meaning all remaining points
    // are *inside* Ld), the target point is simply the last point on the path.
    last_target_index_ = path_length_ > 0 ? path_length_ - 1 : 0;
    return current_path_[path_length_ - 1];
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
