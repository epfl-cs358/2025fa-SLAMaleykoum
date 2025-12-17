import socket
import struct
import pygame
import numpy as np
import math
import time
import sys
import csv
from datetime import datetime

# =============================================================================
# CONFIGURATION
# =============================================================================
ESP_IP = "192.168.4.1" 
PORT = 9000
GRID_W, GRID_H = 70, 70
RESOLUTION = 0.2
WINDOW_SCALE = 15
LOG_FILENAME = f"telemetry_debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# =============================================================================
# HELPER FUNCTIONS
# =============================================================================
def world_to_screen(wx, wy):
    try:
        if not (math.isfinite(wx) and math.isfinite(wy)): return None
        cx, cy = (GRID_W / 2) * WINDOW_SCALE, (GRID_H / 2) * WINDOW_SCALE
        ppm = (1.0 / RESOLUTION) * WINDOW_SCALE
        return int(cx + (wx * ppm)), int(cy - (wy * ppm))
    except: return None

def rle_decode(rle_data, total_size):
    decoded = np.zeros(total_size, dtype=np.int8)
    idx, rle_idx = 0, 0
    while rle_idx < len(rle_data) - 1 and idx < total_size:
        count = int(rle_data[rle_idx]) 
        val   = np.int8(rle_data[rle_idx+1]) 
        write_len = min(count, total_size - idx)
        decoded[idx : idx + write_len] = val
        idx += write_len; rle_idx += 2 
    return decoded

# =============================================================================
# MAIN
# =============================================================================
def main():
    print(f"Logging to: {LOG_FILENAME}")
    csv_file = open(LOG_FILENAME, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "Timestamp", "RX_Interval_ms", 
        "Map_Calc_us", "Global_Calc_us", "Mission_Calc_us", 
        "Lidar_Frames", "Map_Frames", "Planner_Fails", 
        "Planner_Status", "Queue_Load_Pct",
        "Heap_Free"
    ])

    pygame.init()
    # Added extra space at bottom for diagnostics
    screen = pygame.display.set_mode((GRID_W * WINDOW_SCALE, GRID_H * WINDOW_SCALE + 150)) 
    pygame.display.set_caption("ESP32 Debug Station")
    font = pygame.font.SysFont("Consolas", 16, bold=True)
    
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.settimeout(3.0) 
    
    try: client.connect((ESP_IP, PORT)); print("Connected!")
    except: print("Connection Failed"); return

    rx_buffer = b''
    HEADER_SIZE = 96
    
    # State
    map_surface = None
    path_points = []
    car_pose = {'x':0, 'y':0, 'theta':0}
    goal_pose = {'x':0, 'y':0, 'theta':0}
    
    last_packet_time = time.time()
    
    # Diagnostic Vars
    lidar_cnt, map_cnt, plan_fail_cnt = 0, 0, 0
    plan_status, q_load = 0, 0
    t_map, t_gplan, t_mplan, h_free = 0, 0, 0, 0
    
    PLANNER_ERRORS = {
        0: "IDLE", 1: "RUNNING", 2: "SUCCESS",
        3: "ERR_START_BLOCKED", 4: "ERR_GOAL_BLOCKED", 
        5: "ERR_NO_PATH", 6: "ERR_TIMEOUT"
    }

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False

        try:
            chunk = client.recv(65536)
            if not chunk: break
            rx_buffer += chunk

            while len(rx_buffer) >= HEADER_SIZE:
                header_idx = rx_buffer.find(b'\xBE\xEF')
                if header_idx == -1: rx_buffer = rx_buffer[-1:]; break
                rx_buffer = rx_buffer[header_idx:]
                if len(rx_buffer) < HEADER_SIZE: break
                
                map_sz = struct.unpack('<I', rx_buffer[2:6])[0]
                g_len = struct.unpack('<H', rx_buffer[6:8])[0]
                total_sz = HEADER_SIZE + (g_len * 8) + map_sz
                if len(rx_buffer) < total_sz: break 
                
                packet = rx_buffer[:total_sz]
                rx_buffer = rx_buffer[total_sz:] 
                
                # --- METRICS ---
                now = time.time()
                rx_ms = (now - last_packet_time) * 1000.0
                last_packet_time = now
                
                cx, cy, ct = struct.unpack('<fff', packet[8:20])
                gx, gy, gt = struct.unpack('<fff', packet[20:32])
                car_pose = {'x':cx, 'y':cy, 'theta':ct}
                goal_pose = {'x':gx, 'y':gy, 'theta':gt}

                # --- NEW UNPACK FORMAT ---
                # Based on the C++ struct update:
                # 5 * uint32 (Heap/Times)
                # 4 * uint32 (Stacks)
                # 3 * uint32 (Counters: Lidar, Map, PlanFail)
                # 2 * uint8 (Status, Queue)
                # 2 * uint8 (Padding/Alignment - usually added by compiler)
                # 1 * uint32 (Time)
                
                # Assuming 4-byte alignment on ESP32
                try:
                    # 16 unsigned ints (I) = 64 bytes total
                    # This matches the SystemHealth struct exactly now (16 * 4 bytes)
                    fmt = '<IIIIIIIIIIIIIIII' 
                    health_bytes = packet[33 : 33 + 64] # Read 64 bytes
                    
                    data = struct.unpack(fmt, health_bytes)
                    
                    h_free = data[0]
                    # h_min  = data[1]
                    t_map = data[2]
                    t_gplan = data[3]
                    t_mplan = data[4]
                    # stacks = h_data[5:9]
                    
                    lidar_cnt       = data[10]
                    map_cnt         = data[11]
                    plan_fail_cnt   = data[12]
                    plan_status     = data[13] # Now an Int32
                    q_load          = data[14] # Now an Int32
                    
                    # Log to CSV
                    csv_writer.writerow([
                        f"{now:.4f}", f"{rx_ms:.1f}", 
                        t_map, t_gplan, t_mplan, 
                        lidar_cnt, map_cnt, plan_fail_cnt, 
                        plan_status, q_load, h_free
                    ])
                except Exception as e:
                    print(f"Health Decode Error: {e}")

                # --- VISUALIZATION UPDATE ---
                
                # Path
                offset = HEADER_SIZE
                path_payload = packet[offset : offset + (g_len * 8)]
                path_points = []
                for i in range(g_len):
                    px, py = struct.unpack('<ff', path_payload[i*8 : (i+1)*8])
                    path_points.append((px, py))

                # Map
                if map_sz > 0:
                    rle = np.frombuffer(packet[HEADER_SIZE + (g_len*8):], dtype=np.uint8)
                    grid = rle_decode(rle, GRID_W * GRID_H).reshape((GRID_H, GRID_W))
                    
                    rgb = np.zeros((GRID_H, GRID_W, 3), dtype=np.uint8)
                    rgb[:,:] = (100,100,100)
                    rgb[grid > 5] = (0,0,0)
                    rgb[grid < -5] = (255,255,255)
                    surf = pygame.surfarray.make_surface(np.transpose(rgb, (1,0,2)))
                    map_surface = pygame.transform.scale(surf, (GRID_W*WINDOW_SCALE, GRID_H*WINDOW_SCALE))

            # --- DRAW ---
            screen.fill((30, 30, 30))
            if map_surface: screen.blit(map_surface, (0,0))
            
            # Robot
            cp = world_to_screen(car_pose['x'], car_pose['y'])
            if cp: 
                pygame.draw.circle(screen, (255,50,50), cp, 8)
                end_x = cp[0] - 15 * math.sin(car_pose['theta'])
                end_y = cp[1] - 15 * math.cos(car_pose['theta'])
                pygame.draw.line(screen, (255,50,50), cp, (end_x, end_y), 3)

            # Goal
            gp = world_to_screen(goal_pose['x'], goal_pose['y'])
            if gp: pygame.draw.circle(screen, (50,255,50), gp, 6)

            # Path
            for p in path_points:
                pp = world_to_screen(p[0], p[1])
                if pp: pygame.draw.circle(screen, (255,200,0), pp, 3)

            # --- HUD ---
            y_base = GRID_H * WINDOW_SCALE + 10
            
            # Diagnosis Logic
            starvation_warning = (lidar_cnt > map_cnt + 50)
            c_map = (255, 0, 0) if starvation_warning else (0, 255, 0)
            c_plan = (255, 0, 0) if plan_status > 2 else (0, 255, 0)
            
            status_str = PLANNER_ERRORS.get(plan_status, f"Unknown({plan_status})")
            
            lines = [
                f"Lidar Frames: {lidar_cnt}  |  Map Frames: {map_cnt}  |  Queue: {q_load}%",
                f"Planner Status: {status_str} (Fails: {plan_fail_cnt})",
                f"Global Plan Time: {t_gplan} us",
                f"Map Update Time : {t_map} us",
                f"Packet Jitter   : {rx_ms:.1f} ms",
                f"Heap Free       : {h_free} B"
            ]
            
            colors = [c_map, c_plan, (255,255,255), (255,255,255), (255,255,255), (255,255,255)]

            for i, line in enumerate(lines):
                screen.blit(font.render(line, True, colors[i]), (10, y_base + i*25))

            pygame.display.flip()

        except socket.timeout: pass
        except Exception as e: print(e); break
    
    csv_file.close()
    pygame.quit()

if __name__ == "__main__": main()
