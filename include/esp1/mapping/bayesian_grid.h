/** @filename: esp1/mapping/bayesian_grid.h
 *  @description: Contract for the Bayesian Occupancy Grid map.
 * 
 *  @job: Manages the occupancy map using Bayesian filtering.
 */

#pragma once

#include "../../common/data_types.h"
#include <stdint.h>

// TODO: UPGRADE -> using a Hash Map (or a fast custom hash table)
//      could be a V2/V3 upgrade to achieve virtually unlimited map size,
//      as we would only pay for the explored space.
// However, for now, keep it as a fixed-size map.

/**
 * @brief Manages the occupancy map using Bayesian filtering.
 */
class BayesianOccupancyGrid {
public:
    /**
     * @brief Initializes the grid with a size and resolution.
     */
    BayesianOccupancyGrid(float resolution_m, uint16_t size_x, uint16_t size_y);

    /**
     * @brief Updates the occupancy grid based on a new LiDAR scan and the robot's pose.
     * It keeps the previous map and applies Bayesian updates to the log-odds values.
     * The log-odds values are clamped to avoid overconfidence.
     * 
     * The update uses Bresenham's line algorithm for ray tracing.
     * Given two grid cells, the algorithm lists all the grid cells that a straight line between them passes through.
     * 
     * @param lidar_scan Synchronized LiDAR scan and robot pose.
     */
    void update_map(const SyncedScan& lidar_scan);

    /**
     * @brief Retrieves the occupancy probability of a specific cell.
     * @return Probability (0.0 to 1.0).
     */
    float get_cell_probability(int x, int y) const;

    /**
     * @brief Exports the map data for use by the Global Planner.
     */
    // Returns a simplified map representation (e.g. pointer or some light compressed array).
    int8_t* get_map_data() const;

    // Internal 2D array to hold log-odds values
    float grid_resolution;
    uint16_t grid_size_x;
    uint16_t grid_size_y;

    // Optimisation:
    int8_t* get_raw_data_pointer() { return log_odds; }
    uint32_t get_total_size() const { return grid_size_x * grid_size_y; }

    // Probability lookup table
    static float* prob_table;

private:
    //  tableau interne, contigu, sans allocation dynamique
    int8_t* log_odds;

    // Precomputed sin/cos tables for efficiency
    static float* sin_table;
    static float* cos_table;
    
    static bool trig_initialized;

    static bool prob_table_initialized;
};
