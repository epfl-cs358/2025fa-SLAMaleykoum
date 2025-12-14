#include "../../../include/esp1/planning/local_planner.h"
#include "esp_wifi.h"

// NOT USED -> to delete ----------------------------------

static inline void world_to_grid(
    int x, int y,
    int& gx, int& gy,
    float res, int sx, int sy)
{
    gx = int(x / res) + sx / 2;
    gy = int(y / res) + sy / 2;
}

LocalPlanner::LocalPlanner (float window_radius_m) : window_radius_m_(window_radius_m) {}

PathMessage LocalPlanner::compute_local_path (const Pose2D& current_pose,
    const Waypoint& target_waypoint, const BayesianOccupancyGrid& map) {

    PathMessage msg{};
    msg.current_length = 0;
    msg.path_id = 0;
    msg.timestamp_ms = esp_wifi_get_tsf_time(WIFI_IF_AP);

    const float res = map.grid_resolution;

    // -----------------------------------------------------
    // 1) EXTRACTION DE LA FENÊTRE LOCALE (INDEXES)
    // -----------------------------------------------------
    // Rayon en cellules
    const int R = window_radius_m_ / res;

    // Conversion pose monde -> cellule globale
    int robot_gx = 0, robot_gy = 0;
    world_to_grid(current_pose.x, current_pose.y, robot_gx, robot_gy,
                  res, map.grid_size_x, map.grid_size_y);

    // Conversion waypoint monde -> cellule globale
    int wp_gx = 0, wp_gy = 0;
    world_to_grid(target_waypoint.x, target_waypoint.y, wp_gx, wp_gy,
                  res, map.grid_size_x, map.grid_size_y);

    // Start local coords = centre de la fenêtre locale (R,R)
    const int local_size = 2 * R + 1; // ----------------------------------------------
    constexpr int MAX_LOCAL = 50;
    if (local_size > MAX_LOCAL) return msg;

    // -----------------------------------------------------
    // BUFFER LOCAL DE LA CARTE
    // -----------------------------------------------------
    // local_cost[y][x] = probabilité d'occupation (0→libre, 1→occupé)
    static float local_cost[MAX_LOCAL][MAX_LOCAL];

    // revoir peut etre l'orientation -------------------------------------------------
    // Remplir le buffer local (autour du robot)
    for (int dy = -R; dy <= R; dy++) {
        for (int dx = -R; dx <= R; dx++) {

            int gx = robot_gx + dx;
            int gy = robot_gy + dy;

            int lx = dx + R;   // coord locale
            int ly = dy + R;

            // Si hors de la carte globale → on marque comme obstacle
            if (gx < 0 || gy < 0 || gx >= map.grid_size_x || gy >= map.grid_size_y) {
                local_cost[ly][lx] = 1.0f; // plein obstacle
                continue;
            }

            local_cost[ly][lx] = map.get_cell_probability(gx, gy);
        }
    }

    // -----------------------------------------------------
    // 2) A* LOCAL SUR LA FENÊTRE
    // -----------------------------------------------------

    // Coordinates du start et du goal dans la fenêtre locale
    int start_x = R;
    int start_y = R;

    int goal_x = wp_gx - robot_gx + R;
    int goal_y = wp_gy - robot_gy + R;

    // Si le waypoint global n'est pas dans la fenêtre → on "clamp"
    if (goal_x < 0) goal_x = 0;
    if (goal_x >= local_size) goal_x = local_size - 1;
    if (goal_y < 0) goal_y = 0;
    if (goal_y >= local_size) goal_y = local_size - 1;

    // Tables pour A*
    static float g_cost[MAX_LOCAL][MAX_LOCAL];
    static int parent_x[MAX_LOCAL][MAX_LOCAL];
    static int parent_y[MAX_LOCAL][MAX_LOCAL];
    static bool visited[MAX_LOCAL][MAX_LOCAL];

    // Init
    for (int y = 0; y < local_size; y++) {
        for (int x = 0; x < local_size; x++) {
            g_cost[y][x] = 1e9f;
            parent_x[y][x] = -1;
            parent_y[y][x] = -1;
            visited[y][x] = false;
        }
    }

    // Petit struct interne pour A*
    struct Node {
        int x, y;
        float f;  // f = g + h
        bool operator>(const Node& other) const {
            return f > other.f;
        }
    };

    // Petit PQ statique (pas de vector)
    static Node pq_buf[MAX_LOCAL * MAX_LOCAL];
    int pq_size = 0;

    auto pq_push = [&](Node n){
        if (pq_size < MAX_LOCAL * MAX_LOCAL)
            pq_buf[pq_size++] = n;
    };
    auto pq_pop = [&](){
        int best = -1;
        for (int i = 0; i < pq_size; i++) {
            if (best < 0 || pq_buf[i].f < pq_buf[best].f) best = i;
        }
        Node ret = pq_buf[best];
        pq_buf[best] = pq_buf[--pq_size];
        return ret;
    };

    // Start
    g_cost[start_y][start_x] = 0.f;
    pq_push({start_x, start_y,
             float(std::hypot(goal_x - start_x, goal_y - start_y))});

    const int MOVES[4][2] = {
        {1,0},{-1,0},{0,1},{0,-1},
    };

    bool found = false;

    while (pq_size > 0) {
        Node cur = pq_pop();
        int cx = cur.x;
        int cy = cur.y;

        if (visited[cy][cx]) continue;
        visited[cy][cx] = true;

        if (cx == goal_x && cy == goal_y) {
            found = true;
            break;
        }

        for (auto& m: MOVES) {
            int nx = cx + m[0];
            int ny = cy + m[1];

            if (nx < 0 || ny < 0 || nx >= local_size || ny >= local_size)
                continue;

            if (visited[ny][nx]) continue;

            // collision check
            if (local_cost[ny][nx] > 0.65f) continue;

            bool diag = (m[0] != 0 && m[1] != 0);
            float step_cost = diag ? 1.4142f : 1.f;

            float new_g = g_cost[cy][cx] + step_cost;

            if (new_g < g_cost[ny][nx]) {
                g_cost[ny][nx] = new_g;
                parent_x[ny][nx] = cx;
                parent_y[ny][nx] = cy;

                float h = std::hypot(goal_x - nx, goal_y - ny);
                pq_push({nx, ny, new_g + h});
            }
        }
    }

    if (!found)
        return msg; // pas de chemin local

    // -----------------------------------------------------
    // 3) RECONSTRUCTION DU MINI-CHEMIN LOCAL
    // -----------------------------------------------------
    static int path_lx[MAX_LOCAL_PATH_LENGTH];
    static int path_ly[MAX_LOCAL_PATH_LENGTH];
    int path_len = 0;

    int cx = goal_x;
    int cy = goal_y;

    while (!(cx == start_x && cy == start_y) &&
           path_len < MAX_LOCAL_PATH_LENGTH)
    {
        path_lx[path_len] = cx;
        path_ly[path_len] = cy;
        path_len++;

        int px = parent_x[cy][cx];
        int py = parent_y[cy][cx];

        if (px < 0 || py < 0) break;

        cx = px;
        cy = py;
    }

    // Ajouter le start
    if (path_len < MAX_LOCAL_PATH_LENGTH) {
        path_lx[path_len] = start_x;
        path_ly[path_len] = start_y;
        path_len++;
    }

    // Inverser pour avoir start → goal
    int hl = path_len / 2;
    for (int i = 0; i < hl; i++) {
        std::swap(path_lx[i], path_lx[path_len-1-i]);
        std::swap(path_ly[i], path_ly[path_len-1-i]);
    }

    // -----------------------------------------------------
    // 4) CONVERSION GRILLE LOCALE → MONDE
    // -----------------------------------------------------
    msg.current_length = (uint16_t)path_len;

    for (int i = 0; i < path_len; i++) {
        int lx = path_lx[i];
        int ly = path_ly[i];

        int gx = robot_gx + (lx - R);
        int gy = robot_gy + (ly - R);

        msg.path[i].x = gx * res;
        msg.path[i].y = gy * res;
    }

    return msg;
}


/**
     * @brief Détermine si le waypoint est atteint.
     */
bool LocalPlanner::is_waypoint_reached(
    const Pose2D& current_pose,
    const Pose2D& waypoint
) const {

    const float dx = current_pose.x - waypoint.x;
    const float dy = current_pose.y - waypoint.y;

    return std::sqrt(dx * dx + dy * dy) < 0.20f;
}