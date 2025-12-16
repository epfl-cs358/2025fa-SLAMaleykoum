/**
 * @file recovery_maneuvers.h
 * @brief Utility functions for performing recovery maneuvers and orientation corrections.
 *
 * This module provides helper functions used by the control system to compute orientation targets, 
 * normalize angular errors, and execute recovery maneuvers such as multi-point turns when the robot becomes misaligned or stuck.
 * 
 * @author SLAMaleykoum 
 * @date Dec 2025
 */

#ifndef RECOVERY_MANEUVERS_H
#define RECOVERY_MANEUVERS_H

namespace Control {

    /**
     * @brief Computes the yaw angle required to face a target waypoint.
     *
     * Given the car's current (x, y) position and a target waypoint,
     * this function returns the desired yaw angle in degrees that points directly toward the target.
     *
     * @param currentX Current X position of the robot.
     * @param currentY Current Y position of the robot.
     * @param targetX  Target waypoint X position.
     * @param targetY  Target waypoint Y position.
     * @return The yaw angle required to face the waypoint.
     */
    float calculateTargetYaw(float currentX, float currentY, 
                            float targetX, float targetY);

    /**
     * @brief Normalizes an angle difference to the range [-180, 180] degrees.
     *
     * Ensures the angular error between a target and current heading is minimized by wrapping around correctly. 
     * Prevents large or discontinuous angle jumps when crossing the ±180° boundary.
     *
     * @param target  Target angle.
     * @param current Current angle.
     * @return Normalized angle difference within [-180, 180].
     */

    float normalizeAngleDiff(float target, float current);

    /**
     * @brief Executes a multi-point turning maneuver to reach a desired heading.
     *
     * Used when the robot cannot rotate directly toward the target yaw due
     * to steering or space limitations. This function performs a sequence
     * of forward and reverse motions to realign the robot to the desired
     * orientation.
     *
     * @param targetYaw Desired final yaw angle after the recovery maneuver.
    */
    void performMultiPointTurn(float targetYaw);

}

#endif