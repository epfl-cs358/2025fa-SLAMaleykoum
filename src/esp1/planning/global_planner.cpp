#include "../../../include/esp1/planning/global_planner.h"
#include "esp_wifi.h"
#include "../../include/common/utils.h"
#include <cmath>

GlobalPlanner::GlobalPlanner() {}

// ---------------------------------------------------------
//                       A* PLANNER
// ---------------------------------------------------------
PathMessage GlobalPlanner::generate_path(
    const Pose2D& current_pose,
    const MissionGoal& goal,
    const BayesianOccupancyGrid& map,
    GlobalPlannerWorkspace* ws
)
{  
    PathMessage msg{};
    msg.current_length = 0;
    msg.path_id = 0;
    msg.timestamp_ms = esp_wifi_get_tsf_time(WIFI_IF_AP);

    if (!ws) return msg;

    const int W = map.grid_size_x;
    const int H = map.grid_size_y;
    const float res = map.grid_resolution;
    const int MAX_CELLS = W * H;

    if (W <= 0 || H <= 0 || MAX_CELLS > GP_MAX_CELLS)
        return msg;

    // -------------------------------------------
    // Convert start & goal to grid
    // -------------------------------------------
    int sx, sy, gx, gy;
    world_to_grid(current_pose.x, current_pose.y, sx, sy, res, W, H);
    world_to_grid(goal.target_pose.x, goal.target_pose.y, gx, gy, res, W, H);

    if (sx < 0 || sx >= W || sy < 0 || sy >= H) return msg;
    if (gx < 0 || gx >= W || gy < 0 || gy >= H) return msg;

    int start_idx = sy * W + sx;
    int goal_idx  = gy * W + gx;

    // -------------------------------------------
    // Reset workspace
    // -------------------------------------------
    for (int i = 0; i < MAX_CELLS; i++) {
        ws->g_cost[i] = 1e9f;
        ws->closed[i] = 0;
    }

    // -------------------------------------------
    // Binary heap (MIN-HEAP)
    // -------------------------------------------
    ws->pq_size = 0;

    auto pq_push = [&](int16_t idx, float f) {
        if (ws->pq_size >= GP_PQ_SIZE) return;

        int i = ws->pq_size++;
        ws->pq[i] = {idx, f};

        while (i > 0) {
            int p = (i - 1) / 2;
            if (ws->pq[p].f <= ws->pq[i].f) break;
            std::swap(ws->pq[p], ws->pq[i]);
            i = p;
        }
    };

    auto pq_pop = [&]() {
        GlobalPlannerWorkspace::Node ret = ws->pq[0];
        ws->pq_size--;
        if (ws->pq_size > 0) {
            ws->pq[0] = ws->pq[ws->pq_size];
            int i = 0;
            while (true) {
                int l = 2 * i + 1;
                int r = 2 * i + 2;
                int s = i;

                if (l < ws->pq_size && ws->pq[l].f < ws->pq[s].f) s = l;
                if (r < ws->pq_size && ws->pq[r].f < ws->pq[s].f) s = r;
                if (s == i) break;
                std::swap(ws->pq[i], ws->pq[s]);
                i = s;
            }
        }
        return ret;
    };

    // -------------------------------------------
    // Start A*
    // -------------------------------------------
    ws->g_cost[start_idx] = 0.f;
    float h0 = sqrtf((gx - sx)*(gx - sx) + (gy - sy)*(gy - sy));
    pq_push((int16_t)start_idx, h0);

    const int MOVES[4] = { 1, -1, W, -W };
    const int DX[4]    = { 1, -1, 0, 0 };
    const int DY[4]    = { 0, 0, 1, -1 };

    bool found = false;
    int loop_counter = 0;

    while (ws->pq_size > 0) {

        loop_counter++;
        if (loop_counter % 50 == 0) {
            vTaskDelay(1); // Sleep for 1 tick (allows OS to breathe)
        }

        GlobalPlannerWorkspace::Node cur = pq_pop();
        int c_idx = cur.idx;

        if (ws->closed[c_idx]) continue;
        ws->closed[c_idx] = 1;

        if (c_idx == goal_idx) {
            found = true;
            break;
        }

        int cx = c_idx % W;
        int cy = c_idx / W;
        float gc = ws->g_cost[c_idx];

        for (int i = 0; i < 4; i++) {
            int nx = cx + DX[i];
            int ny = cy + DY[i];
            int n_idx = c_idx + MOVES[i];

            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
            if (ws->closed[n_idx]) continue;

            // Obstacle check
            if (map.get_cell_probability(nx, ny) > FREE_BOUND_PROB) continue;

            // -----------------------------
            // 🔴 SAFETY COST (KEY FIX)
            // -----------------------------
            float obstacle_penalty = 0.0f;
            for (int oy = -1; oy <= 1; oy++) {
                for (int ox = -1; ox <= 1; ox++) {
                    int tx = nx + ox;
                    int ty = ny + oy;
                    if (map.get_cell_probability(tx, ty) > FREE_BOUND_PROB) {
                        obstacle_penalty += 5.0f;
                    }
                }
            }

            float new_g = gc + 1.0f + obstacle_penalty;

            if (new_g < ws->g_cost[n_idx]) {
                ws->g_cost[n_idx] = new_g;
                ws->parent_index[n_idx] = (int16_t)c_idx;

                float h = sqrtf((gx - nx)*(gx - nx) + (gy - ny)*(gy - ny));
                pq_push((int16_t)n_idx, new_g + h);
            }
        }
    }

    if (!found) return msg;

    // -------------------------------------------
    // Reconstruction (SAFE)
    // -------------------------------------------
    int plen = 0;
    int curr = goal_idx;

    while (curr != start_idx && plen < GP_MAX_CELLS - 1) {
        ws->px[plen] = curr % W;
        ws->py[plen] = curr / W;
        plen++;
        curr = ws->parent_index[curr];
    }

    ws->px[plen] = sx;
    ws->py[plen] = sy;
    plen++;

    // -------------------------------------------
    // Output
    // -------------------------------------------
    msg.current_length = std::min(plen, MAX_PATH_LENGTH);

    for (int i = 0; i < msg.current_length; i++) {
        int idx = plen - 1 - i;
        grid_to_world(ws->px[idx], ws->py[idx],
                      msg.path[i].x, msg.path[i].y,
                      res, W, H);
    }

    return msg;
}
