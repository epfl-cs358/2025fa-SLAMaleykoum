#include "../../../include/esp1/planning/global_planner.h"
#include "esp_wifi.h"
#include "../../include/common/utils.h"

// ---------------------------------------------------------
//                    A* NODE
// ---------------------------------------------------------
struct Node {
    int x, y;
    float g, h;

    bool operator>(const Node& o) const {
        return g + h > o.g + o.h;
    }
};

GlobalPlanner::GlobalPlanner() {}

// ---------------------------------------------------------
//              COLLISION CHECK WITH ROBOT RADIUS
// ---------------------------------------------------------

static inline bool robot_circle_collides(int cx, int cy, const BayesianOccupancyGrid& map) {
    const int sx = map.grid_size_x;
    const int sy = map.grid_size_y;

    int radius_cells = int(std::ceil(ROBOT_RADIUS / map.grid_resolution));

    for (int dy = -radius_cells; dy <= radius_cells; dy++) {
        for (int dx = -radius_cells; dx <= radius_cells; dx++) {

            if (dx*dx + dy*dy > radius_cells*radius_cells) continue;

            int nx = cx + dx;
            int ny = cy + dy;

            if (nx < 0 || nx >= sx || ny < 0 || ny >= sy) return true;

            if (map.get_cell_probability(nx, ny) > OCC_BOUND_PROB) return true;
        }
    }
    return false;
}

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

    // Safety check
    if (!ws) return msg;

    const int W = map.grid_size_x;
    const int H = map.grid_size_y;
    const float res = map.grid_resolution;

    if (W <= 0 || H <= 0 || W > GP_MAX_W || H > GP_MAX_H)
        return msg;

    // -------------------------------------------
    // Convert start & goal to grid
    // -------------------------------------------
    int sx, sy, gx, gy;
    world_to_grid(current_pose.x, current_pose.y, sx, sy, res, W, H);
    world_to_grid(goal.target_pose.x, goal.target_pose.y, gx, gy, res, W, H);

    if (sx<0||sx>=W||sy<0||sy>=H) return msg;
    if (gx<0||gx>=W||gy<0||gy>=H) return msg;

    // -------------------------------------------
    // Reset Workspace Memory (Fast Reset)
    // -------------------------------------------
    // We only need to reset the arrays we use. 
    // memset is safer and faster than nested loops.
    for(int y=0; y<H; y++) {
        for(int x=0; x<W; x++) {
            ws->g_cost[y][x] = 1e9f;
            ws->closed[y][x] = false;
        }
    }
    
    // -------------------------------------------
    // 4. Priority Queue Setup
    // -------------------------------------------
    int pq_size = 0;
    
    // Update lambda to take int16_t
    auto pq_push = [&](int16_t x, int16_t y, float f) {
        if (pq_size < GP_MAX_N) {
            ws->pq[pq_size] = {x, y, f};
            pq_size++;
        }
    };

    auto pq_pop = [&]() {
        int best = 0;
        for(int i=1; i<pq_size; i++){
            if(ws->pq[i].f < ws->pq[best].f) best = i;
        }
        GlobalPlannerWorkspace::Node r = ws->pq[best];
        ws->pq[best] = ws->pq[--pq_size];
        return r;
    };

    // -------------------------------------------
    // 5. A* Algorithm
    // -------------------------------------------
    ws->g_cost[sy][sx] = 0.f;
    float h0 = sqrtf((gx-sx)*(gx-sx) + (gy-sy)*(gy-sy));
    pq_push(sx, sy, h0);

    const int MOVES[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
    bool found = false;

    while(pq_size > 0){
        GlobalPlannerWorkspace::Node cur = pq_pop();
        int cx = cur.x, cy = cur.y;

        if (ws->closed[cy][cx]) continue;
        ws->closed[cy][cx] = true;

        if (cx == gx && cy == gy) {
            found = true;
            break;
        }

        float gc = ws->g_cost[cy][cx];

        for (int i=0;i<4;i++){
            int nx = cx + MOVES[i][0];
            int ny = cy + MOVES[i][1];

            if (nx<0||ny<0||nx>=W||ny>=H) continue;
            if (ws->closed[ny][nx]) continue;
            
            // --- SAFETY MARGIN CHECK ---
            // 1. Is the cell itself occupied?
            if (map.get_cell_probability(nx, ny) > 0.55f) continue;

            // 2. Are any neighbors occupied? (The "Buffer Zone")
            bool too_close_to_wall = false;
            
            // Check the 8 cells surrounding the target node (nx, ny)
            for (int buf_y = -1; buf_y <= 1; buf_y++) {
                for (int buf_x = -1; buf_x <= 1; buf_x++) {
                    if (buf_x == 0 && buf_y == 0) continue; // Skip center

                    int bx = nx + buf_x;
                    int by = ny + buf_y;

                    // If neighbor is out of bounds or occupied, this node is unsafe
                    if (bx >= 0 && bx < W && by >= 0 && by < H) {
                        if (map.get_cell_probability(bx, by) > OCC_BOUND_PROB) {
                            too_close_to_wall = true;
                            break; // Stop checking, we found a wall
                        }
                    }
                }
                if (too_close_to_wall) break;
            }

            if (too_close_to_wall) continue; // Treat this cell as a wall

            float new_g = gc + 1.0f;

            if(new_g < ws->g_cost[ny][nx]){
                ws->g_cost[ny][nx] = new_g;
                ws->parent_x[ny][nx] = cx;
                ws->parent_y[ny][nx] = cy;

                float h = sqrtf((gx-nx)*(gx-nx) + (gy-ny)*(gy-ny));
                pq_push(nx, ny, new_g + h);
            }
        }
    }

    if (!found) return msg;

    // -------------------------------------------
    // 6. Reconstruction
    // -------------------------------------------
    int plen = 0;
    
    // Safety check on max path length (using GP_MAX_N)
    while(!(gx==sx && gy==sy) && plen < GP_MAX_N) {
        ws->px[plen] = gx;
        ws->py[plen] = gy;
        plen++;

        int px2 = ws->parent_x[gy][gx];
        int py2 = ws->parent_y[gy][gx];

        gx = px2;
        gy = py2;
    }

    // -------------------------------------------
    // 7. Sampling & Output
    // -------------------------------------------
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
