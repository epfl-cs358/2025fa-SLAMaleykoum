/**
 * @file recovery_maneuvers.h
 * @brief Utility functions for performing recovery maneuvers and orientation corrections.
 *
 * This module provides helper functions used by the control system to execute recovery maneuvers such as 
 * multi-point turns when the robot becomes misaligned or stuck.
 * 
 * @author SLAMaleykoum 
 * @date Dec 2025
 */

#ifndef RECOVERY_MANEUVERS_H
#define RECOVERY_MANEUVERS_H

namespace Control {

    /**
     * @brief Executes a multi-point turning maneuver to reach a desired heading.
     *
     * Used when the robot cannot rotate 180 degrees due
     * to steering or space limitations. This function performs a sequence
     * of forward and reverse motions to realign the robot to the desired
     * orientation depending on a predefined number of iterations (here 2 is around
     * 35 degree rotation).
     *
    */
    void performMultiPointTurn();

}

#endif