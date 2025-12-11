#pragma once
#include "../../../include/common/data_types.h"
#include "../../../include/esp1/mapping/occupancy/bayesian_grid.h"

/**
 * @brief Local Planner computing a detailed local path toward a single waypoint.
 *
 * Travaille dans une fenêtre locale autour du robot sur la carte fine.
 * Fonctionne à haute fréquence (10–50 Hz selon CPU).
 */
class LocalPlanner {
public:
    /**
     * @brief Constructor
     *
     * @param window_radius_m  Rayon de la fenêtre locale (en mètres)
     */
    LocalPlanner(float window_radius_m);

    /**
     * @brief Calcule un chemin local jusqu'au prochain waypoint global.
     *
     * @param current_pose     Pose du robot
     * @param target_waypoint  Waypoint global (un seul point, coord. monde)
     * @param map              Carte fine (occupancy grid haute resolution)
     *
     * @return LocalPathMessage = chemin court pour navigation immédiate
     */
    PathMessage compute_local_path(
        const Pose2D& current_pose,
        const Waypoint& target_waypoint,
        const BayesianOccupancyGrid& map
    );

    /**
     * @brief Détermine si le waypoint est atteint.
     */
    bool is_waypoint_reached(
        const Pose2D& current_pose,
        const Pose2D& waypoint
    ) const;

private:
    float window_radius_m_; ///< Taille de la fenêtre locale

    // Ici on mettra plus tard des méthodes privées :
    // - extraction de la sous-grille locale
    // - A* local
    // - conversion grid→world locale
};