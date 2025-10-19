// Filename: esp1/mapping/occupancy/bayesian_grid.h
// Description: Contract for the coarse Bayesian Occupancy Grid map.


#pragma once

#include "common/data_types.h"
#include <stdint.h>


// TODO: UPGRADE -> using a Hash Map (or a fast custom hash table)
//      could be a V2/V3 upgrade to achieve virtually unlimited map size,
//      as we would only pay for the explored space.
// However, for now, keep it as a fixed-size map.

// TODO: For now we will use a 4bit value per cell, might change later ?

/**
 * @brief Manages the coarse occupancy map using Bayesian filtering.
 */
class BayesianOccupancyGrid {
public:
    /**
     * @brief Initializes the grid with a size and resolution.
     */
    BayesianOccupancyGrid(float resolution_m, uint16_t size_x, uint16_t size_y);

    /**
     * @brief Updates the occupancy grid based on a new LiDAR scan and the robot's pose.
     * @param scan The new LiDAR observation.
     * @param pose The robot's estimated pose during the scan (from ESP2).
     */
    void update_map(const LiDARScan& scan, const Pose2D& pose);

    /**
     * @brief Retrieves the occupancy probability of a specific cell.
     * @return Probability (0.0 to 1.0).
     */
    float get_cell_probability(float x_m, float y_m) const;

    /**
     * @brief Exports the map data for use by the Global Planner.
     */
    // Returns a simplified map representation (e.g. pointer or some light compressed array).
    const uint8_t* get_map_data() const; 

private:
    // Internal 2D array or vector to hold log-odds values
};
