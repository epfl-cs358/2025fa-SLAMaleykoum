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
    const BayesianOccupancyGrid& map
)
{
    PathMessage msg{};
    msg.current_length = 0;
    msg.path_id = 0;
    msg.timestamp_ms = esp_wifi_get_tsf_time(WIFI_IF_AP);

    const int W = map.grid_size_x;
    const int H = map.grid_size_y;
    const float res = map.grid_resolution;

    if (W <= 0 || H <= 0 || W > GP_MAX_W || H > GP_MAX_H)
        return msg;

    // -------------------------------------------
    // 1) Convert start & goal to grid
    // -------------------------------------------
    int sx, sy, gx, gy;
    world_to_grid(current_pose.x, current_pose.y, sx, sy, res, W, H);
    world_to_grid(goal.target_pose.x, goal.target_pose.y, gx, gy, res, W, H);

    if (sx<0||sx>=W||sy<0||sy>=H) return msg;
    if (gx<0||gx>=W||gy<0||gy>=H) return msg;

    // -------------------------------------------
    // A* Buffers (static)
    // -------------------------------------------
    static float g_cost[GP_MAX_H][GP_MAX_W];
    static int parent_x[GP_MAX_H][GP_MAX_W];
    static int parent_y[GP_MAX_H][GP_MAX_W];
    static bool closed[GP_MAX_H][GP_MAX_W];

    for (int y=0;y<H;y++){
        for (int x=0;x<W;x++){
            g_cost[y][x] = 1e9f;
            parent_x[y][x] = -1;
            parent_y[y][x] = -1;
            closed[y][x] = false;
        }
    }

    // -------------------------------------------
    // priority queue simple
    // -------------------------------------------
    struct Node { int x, y; float f; };
    static Node pq[GP_MAX_N];
    int pq_size = 0;

    auto pq_push = [&](int x,int y,float f){
        if (pq_size < GP_MAX_N) pq[pq_size++] = {x,y,f};
    };

    auto pq_pop = [&](){
        int best = 0;
        for(int i=1;i<pq_size;i++){
            if(pq[i].f < pq[best].f)
                best = i;
        }
        Node r = pq[best];
        pq[best] = pq[--pq_size];
        return r;
    };

    // -------------------------------------------
    // Start A*
    // -------------------------------------------
    g_cost[sy][sx] = 0.f;
    float h0 = sqrtf((gx-sx)*(gx-sx) + (gy-sy)*(gy-sy));
    pq_push(sx, sy, h0);

    const int MOVES[4][2] = {
        {1,0},{-1,0},{0,1},{0,-1},
    };

    bool found = false;

    // -------------------------------------------
    // 2) A*
    // -------------------------------------------
    while(pq_size > 0){
        Node cur = pq_pop();
        int cx = cur.x, cy = cur.y;

        if (closed[cy][cx]) continue;
        closed[cy][cx] = true;

        if (cx == gx && cy == gy) {
            found = true;
            break;
        }

        float gc = g_cost[cy][cx];

        for (int i=0;i<4;i++){
            int nx = cx + MOVES[i][0];
            int ny = cy + MOVES[i][1];

            if (nx<0||ny<0||nx>=W||ny>=H) continue;
            if (closed[ny][nx]) continue;
            if (map.get_cell_probability(nx, ny) >= 0.5f) continue;

            float new_g = gc + 1;

            if(new_g < g_cost[ny][nx]){
                g_cost[ny][nx] = new_g;
                parent_x[ny][nx] = cx;
                parent_y[ny][nx] = cy;

                float h = sqrtf((gx-nx)*(gx-nx) + (gy-ny)*(gy-ny));
                pq_push(nx, ny, new_g + h);
            }
        }
    }

    if (!found)
        return msg;

    // -------------------------------------------
    // 3) Reconstruction backward
    // -------------------------------------------
    static int px[GP_MAX_N];
    static int py[GP_MAX_N];
    int plen = 0;

    while(!(gx==sx && gy==sy) && plen < GP_MAX_N){
        px[plen] = gx;
        py[plen] = gy;
        plen++;

        int px2 = parent_x[gy][gx];
        int py2 = parent_y[gy][gx];

        if (px2 < 0) break;

        gx = px2;
        gy = py2;
    }

    // position 0 dans tableau = goal
    // -------------------------------------------
    // 4) ÉCHANTILLONNER pour max 5 points
    // -------------------------------------------
    if (plen <= MAX_PATH_LENGTH && plen >= 1) {
        msg.current_length = plen;

        int j = 0;
        for (int i = plen - 1; i >= 0; i--) {
            grid_to_world(px[i], py[i], msg.path[j].x, msg.path[j].y, res, W, H);
            j++; 
        }
    } else {
        msg.current_length = MAX_PATH_LENGTH;

        // 1) Le premier waypoint = juste après notre position
        //    donc px[plen-1]
        grid_to_world(px[plen - 1], py[plen - 1], msg.path[0].x, msg.path[0].y, res, W, H);

        // 2) Le dernier waypoint = goal → px[0]
        grid_to_world(px[0], py[0], msg.path[MAX_PATH_LENGTH - 1].x, msg.path[MAX_PATH_LENGTH - 1].y, res, W, H);

        float ratio = float(plen - 2) / (MAX_PATH_LENGTH - 1);

        for (int k = 1; k < MAX_PATH_LENGTH - 1; k++) {
            int idx = (plen - 2) - int(k * ratio);

            grid_to_world(px[idx], py[idx], msg.path[k].x, msg.path[k].y, res, W, H);
        }
    }
    return msg;
}
