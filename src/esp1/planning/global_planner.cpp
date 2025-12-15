#include "../../../include/esp1/planning/global_planner.h"
#include "esp_wifi.h"
#include "../../include/common/utils.h"

GlobalPlanner::GlobalPlanner() {}

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

    // Convert start & goal to grid
    int sx, sy, gx, gy;
    world_to_grid(current_pose.x, current_pose.y, sx, sy, res, W, H);
    world_to_grid(goal.target_pose.x, goal.target_pose.y, gx, gy, res, W, H);

    if (sx < 0 || sx >= W || sy < 0 || sy >= H) return msg;
    if (gx < 0 || gx >= W || gy < 0 || gy >= H) return msg;

    int start_idx = sy * W + sx;
    int goal_idx  = gy * W + gx;

    // Reset workspace
    for (int i = 0; i < MAX_CELLS; i++) {
        ws->g_cost[i] = 1e9f;
        ws->closed[i] = 0;
    }

    // Binary Heap (MIN-HEAP)
    ws->pq_size = 0;

    auto pq_push = [&](int16_t idx, float f) {
        if (ws->pq_size >= GP_PQ_SIZE) return;

        int i = ws->pq_size++;
        ws->pq[i] = {idx, f};

        while (i > 0) {
            int p = (i - 1) / 2;
            if (ws->pq[p].f <= ws->pq[i].f) break; // Parent is smaller, heap property satisfied
            
            // Swap
            GlobalPlannerWorkspace::Node temp = ws->pq[i];
            ws->pq[i] = ws->pq[p];
            ws->pq[p] = temp;
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
                int left = 2 * i + 1;
                int right = 2 * i + 2;
                int smallest = i;

                if (left < ws->pq_size && ws->pq[left].f < ws->pq[smallest].f)
                    smallest = left;
                if (right < ws->pq_size && ws->pq[right].f < ws->pq[smallest].f)
                    smallest = right;

                if (smallest == i) break;

                GlobalPlannerWorkspace::Node temp = ws->pq[i];
                ws->pq[i] = ws->pq[smallest];
                ws->pq[smallest] = temp;
                i = smallest;
            }
        }
        return ret;
    };

    // A STAR SEARCH
    ws->g_cost[start_idx] = 0.f;
    float h0 = sqrtf((gx - sx)*(gx - sx) + (gy - sy)*(gy - sy));
    pq_push((int16_t)start_idx, h0);

    const int MOVES[4] = { 1, -1, W, -W };
    const int DX[4]    = { 1, -1, 0, 0 };
    const int DY[4]    = { 0, 0, 1, -1 };

    bool found = false;
    int loop_counter = 0;

    // Main A* Loop
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

        // Explore neighbors (4-neighborhood)
        for (int i = 0; i < 4; i++) {
            int nx = cx + DX[i];
            int ny = cy + DY[i];
            int n_idx = c_idx + MOVES[i];

            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
            if (ws->closed[n_idx]) continue;

            // Obstacle check
            if (map.get_cell_probability(nx, ny) > FREE_BOUND_PROB) continue;

            // Additional cost for nearby obstacles
            // Wall Buffering (Simplified for 1D)
            bool too_close = false;
            // Check 8 neighbors of (nx, ny)
            for (int buf_y = -1; buf_y <= 1; buf_y++) {
                for (int buf_x = -1; buf_x <= 1; buf_x++) {
                    if (buf_x == 0 && buf_y == 0) continue;
                    int bx = nx + buf_x;
                    int by = ny + buf_y;
                    if (bx >= 0 && bx < W && by >= 0 && by < H) {
                         // Note: get_cell_probability handles 1D conversion internally usually, 
                         // but here we use (x,y) API of the map class.
                         if (map.get_cell_probability(bx, by) > FREE_BOUND_PROB) {
                             too_close = true; buf_y = 2; break; // Break outer loop
                         }
                    }
                }
            }

            if (too_close) continue;

            // Update Path
            float new_g = gc + 1.0f;

            if (new_g < ws->g_cost[n_idx]) {
                ws->g_cost[n_idx] = new_g;
                ws->parent_index[n_idx] = (int16_t)c_idx; // Store 1D index
                
                // Euclidean Heuristic
                float dist = 2 * (abs(gx - nx) + abs(gy - ny)); // Weighted to favor straight paths
                pq_push((int16_t)n_idx, new_g + dist);
            }
        }
    }

    if (!found) return msg;

    // Reconstruction (Using 1D indices)
    int plen = 0;
    int curr = goal_idx;

    // We use px/py arrays to store grid coords, derived from 1D curr
    while (curr != start_idx && plen < GP_MAX_CELLS) {
        ws->px[plen] = curr % W;
        ws->py[plen] = curr / W;
        plen++;
        curr = ws->parent_index[curr];
    }

    ws->px[plen] = sx;
    ws->py[plen] = sy;
    plen++;

    // Output path
    if (plen == 0) {
        msg.current_length = 1;
        msg.path[0].x = goal.target_pose.x;
        msg.path[0].y = goal.target_pose.y;
        return msg;
    }

    if (plen <= MAX_PATH_LENGTH) {
        msg.current_length = plen;
        int j = 0;
        for (int i = plen - 1; i >= 0; i--) {
            grid_to_world(ws->px[i], ws->py[i], msg.path[j].x, msg.path[j].y, res, W, H);
            j++; 
        }
    } else {
        msg.current_length = MAX_PATH_LENGTH;

        // Start
        grid_to_world(ws->px[plen - 1], ws->py[plen - 1], msg.path[0].x, msg.path[0].y, res, W, H);
        
        // End
        grid_to_world(ws->px[0], ws->py[0], msg.path[MAX_PATH_LENGTH - 1].x, msg.path[MAX_PATH_LENGTH - 1].y, res, W, H);

        // Middle
        float ratio = float(plen - 1) / (MAX_PATH_LENGTH - 1);
        for (int k = 1; k < MAX_PATH_LENGTH - 1; k++) {
            int idx = (plen - 1) - int(k * ratio);
            if (idx < 0) idx = 0; 
            if (idx >= plen) idx = plen - 1;
            grid_to_world(ws->px[idx], ws->py[idx], msg.path[k].x, msg.path[k].y, res, W, H);
        }
    }

    return msg;
}
