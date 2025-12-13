import socket
import struct
import pygame
import numpy as np
import math
import time
import sys

# =============================================================================
# CONFIGURATION & CONSTANTS
# =============================================================================

# Network Settings
ESP_IP = "192.168.4.1" 
PORT = 9000

# Map & Rendering Settings
GRID_W = 70        # Width of the grid in cells
GRID_H = 70        # Height of the grid in cells
RESOLUTION = 0.2   # Size of one cell in meters (5cm)
WINDOW_SCALE = 15  # Scale factor for the display window

# Colors (RGB Tuples)
C_BLACK  = (0, 0, 0)        # Occupied cell (Wall)
C_WHITE  = (255, 255, 255)  # Free cell (Open space)
C_GREY   = (100, 100, 100)  # Unknown cell (Not yet explored)
C_RED    = (255, 50, 50)    # Robot Position
C_GREEN  = (50, 255, 50)    # Current Navigation Goal
C_YELLOW = (255, 200, 0)    # Planned Path & HUD Text
C_CYAN   = (0, 255, 255)    # Detected Frontier Points

# =============================================================================
# HELPER FUNCTIONS: COORDINATE TRANSFORMS
# =============================================================================

def world_to_screen(wx, wy):
    """
    Converts World coordinates (meters) to Screen coordinates (pixels).
    Handles scale, centering, and Y-axis inversion (screen Y increases downwards).
    """
    try:
        # Check for valid number input
        if not (math.isfinite(wx) and math.isfinite(wy)): return None
        
        # Calculate center offsets
        cx_screen = (GRID_W / 2) * WINDOW_SCALE
        cy_screen = (GRID_H / 2) * WINDOW_SCALE
        pixels_per_meter = (1.0 / RESOLUTION) * WINDOW_SCALE
        
        # Transform
        sx = cx_screen + (wx * pixels_per_meter)
        sy = cy_screen - (wy * pixels_per_meter) # Invert Y for screen
        
        # Clamp values to prevent crash on huge numbers
        sx = max(-20000, min(20000, sx))
        sy = max(-20000, min(20000, sy))
        return int(sx), int(sy)
    except: return None

def world_to_index(wx, wy):
    """
    Converts World coordinates (meters) to Grid Indices (row, col).
    """
    center_x = GRID_W // 2
    center_y = GRID_H // 2
    ix = int(wx / RESOLUTION) + center_x
    iy = center_y - int(wy / RESOLUTION) # Invert Y for grid index
    return ix, iy

def index_to_world(ix, iy):
    """
    Converts Grid Indices (row, col) back to World coordinates (meters).
    """
    center_x = GRID_W // 2
    center_y = GRID_H // 2
    wx = (ix - center_x) * RESOLUTION
    wy = (center_y - iy) * RESOLUTION
    return wx, wy

# =============================================================================
# HELPER FUNCTIONS: DATA PROCESSING
# =============================================================================

def rle_decode(rle_data, total_size):
    """
    Decodes Run-Length Encoded (RLE) map data.
    Format: [Count, Value, Count, Value, ...]
    Returns a numpy array of signed int8 (Log-Odds values).
    """
    decoded = np.zeros(total_size, dtype=np.int8)
    idx = 0
    rle_idx = 0
    
    # Process pairs of (Count, Value)
    while rle_idx < len(rle_data) - 1 and idx < total_size:
        count = int(rle_data[rle_idx]) 
        val   = np.int8(rle_data[rle_idx+1]) 
        
        # Ensure we don't write past the end of the buffer
        write_len = min(count, total_size - idx)
        decoded[idx : idx + write_len] = val
        
        idx += write_len
        rle_idx += 2 # Move to next pair
        
    return decoded

def is_frontier_cell(grid, x, y):
    rows, cols = grid.shape  # get height and width
    
    # Edge filter
    if x <= 1 or x >= cols - 2 or y <= 1 or y >= rows - 2:
        return False
    
    # If occupied → not frontier
    if grid[y, x] >= 5:  # use log-odds threshold as in your RLE decoding
        return False
    
    neighbors = [(1,0), (-1,0), (0,1), (0,-1)]
    for dx, dy in neighbors:
        nx = x + dx
        ny = y + dy
        if nx < 0 or ny < 0 or nx >= cols or ny >= rows:
            continue
        p = grid[ny, nx]
        if 0.35 < p < 0.6:  # adapt this threshold for your log-odds
            return True
    return False

def find_frontiers_python(grid):
    frontiers = []
    rows, cols = grid.shape
    for y in range(rows):
        for x in range(cols):
            if is_frontier_cell(grid, x, y):
                wx = (x - cols/2) * RESOLUTION
                wy = - (y - rows/2) * RESOLUTION
                frontiers.append((wx, wy))
    return frontiers

# =============================================================================
# MAIN
# =============================================================================
def main():
    # --- Pygame Setup ---
    pygame.init()
    screen = pygame.display.set_mode((GRID_W * WINDOW_SCALE, GRID_H * WINDOW_SCALE))
    pygame.display.set_caption("ESP32 Mission Control")
    font = pygame.font.SysFont("Consolas", 16, bold=True)
    
    trail_surface = pygame.Surface((GRID_W * WINDOW_SCALE, GRID_H * WINDOW_SCALE), pygame.SRCALPHA)

    # --- Network Connection ---
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.settimeout(5.0) 
    try: 
        print(f"Connecting to {ESP_IP}:{PORT}...")
        client.connect((ESP_IP, PORT))
        print("Connected!")
    except Exception as e: 
        print(f"Connection Failed: {e}")
        return

    rx_buffer = b''
    # UPDATED: Increased header size to accommodate extra stack info
    HEADER_SIZE = 100 
    
    # Robot State
    car_pose = {'x': 0, 'y': 0, 'theta': 0}
    goal_pose = {'x': 0, 'y': 0, 'theta': 0}
    robot_state_idx = 0 # 0=Explore, 1=Home, 2=Idle
    
    # Enum Map
    STATE_NAMES = {0: "EXPLORING", 1: "RETURNING", 2: "IDLE"}

    path_points = []
    history_trail = []
    
    grid_log_odds = np.zeros((GRID_H, GRID_W), dtype=np.int8)
    map_surface = None
    detected_frontiers = [] 

    # --- LOGGER VARIABLES (Init to 0 for safety) ---
    t_map, t_gplan, t_mplan = 0, 0, 0
    h_min = 0
    s_tcp, s_gplan, s_mplan, s_lidar, s_map, s_sync = 0, 0, 0, 0, 0, 0
    t_esp2 = 0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_c:
                history_trail = []
                trail_surface.fill((0,0,0,0))

        try:
            chunk = client.recv(65536)
            if not chunk: break
            rx_buffer += chunk

            while True:
                if len(rx_buffer) < HEADER_SIZE: break 
                
                header_idx = rx_buffer.find(b'\xBE\xEF')
                if header_idx == -1: 
                    rx_buffer = rx_buffer[-1:]
                    break
                if header_idx > 0: 
                    rx_buffer = rx_buffer[header_idx:]
                
                if len(rx_buffer) < HEADER_SIZE: break
                
                # Read Sizes
                map_bytes_size = struct.unpack('<I', rx_buffer[2:6])[0]
                global_len = struct.unpack('<H', rx_buffer[6:8])[0]
                
                total_packet_size = HEADER_SIZE + (global_len * 8) + map_bytes_size

                if len(rx_buffer) < total_packet_size: break 
                
                packet = rx_buffer[:total_packet_size]
                rx_buffer = rx_buffer[total_packet_size:] 

                # --- DECODING ---
                
                # Poses
                cx, cy, ct = struct.unpack('<fff', packet[8:20])
                gx, gy, gt = struct.unpack('<fff', packet[20:32])
                
                robot_state_idx = packet[32] 
                
                # UPDATED UNPACKING: Now reads 36 bytes (extra 4 bytes for Map/Sync stack)
                try:
                    # Health offset starts at 33. We read 36 bytes.
                    health_data = packet[33:33+36]
                    # Format: 5 uint32, 6 uint16, 1 uint32
                    h_free, h_min, t_map, t_gplan, t_mplan, s_tcp, s_gplan, s_mplan, s_lidar, s_map, s_sync, t_esp2 = struct.unpack('<IIIIIHHHHHHI', health_data)
                except Exception as e:
                    print(f"Packet size mismatch: {e}")
                    continue

                car_pose = {'x': cx, 'y': cy, 'theta': ct}
                goal_pose = {'x': gx, 'y': gy, 'theta': gt}

                # Update Trail
                if not history_trail or math.hypot(cx - history_trail[-1][0], cy - history_trail[-1][1]) > 0.1:
                    history_trail.append((cx, cy))
                    if len(history_trail) > 2000: history_trail.pop(0)
                    pt = world_to_screen(cx, cy)
                    if pt: pygame.draw.circle(trail_surface, (0, 150, 255, 50), pt, 4)

                # Path
                offset = HEADER_SIZE
                path_payload = packet[offset : offset + (global_len * 8)]
                path_points = []
                for i in range(global_len):
                    px, py = struct.unpack('<ff', path_payload[i*8 : (i+1)*8])
                    path_points.append((px, py))

                # Map
                if map_bytes_size > 0:
                    map_payload = packet[HEADER_SIZE + (global_len * 8) :]
                    rle_data = np.frombuffer(map_payload, dtype=np.uint8)
                    try:
                        grid_log_odds = rle_decode(rle_data, GRID_W * GRID_H).reshape((GRID_H, GRID_W))
                        
                        rgb_map = np.zeros((GRID_H, GRID_W, 3), dtype=np.uint8)
                        occ_mask  = grid_log_odds > 4
                        free_mask = grid_log_odds < -6
                        unk_mask  = (~free_mask) & (~occ_mask)
                        
                        rgb_map[free_mask] = C_WHITE
                        rgb_map[occ_mask] = C_BLACK
                        rgb_map[unk_mask] = C_GREY
                        
                        surf = pygame.surfarray.make_surface(np.transpose(rgb_map, (1, 0, 2)))
                        map_surface = pygame.transform.scale(surf, (GRID_W * WINDOW_SCALE, GRID_H * WINDOW_SCALE))
                        
                        detected_frontiers = find_frontiers_python(grid_log_odds)

                    except Exception as e: print(f"Map Error: {e}")

            # --- RENDERING ---
            screen.fill(C_GREY)
            if map_surface: screen.blit(map_surface, (0, 0))
            
            for fx, fy in detected_frontiers:
                fpt = world_to_screen(fx, fy)
                if fpt: pygame.draw.circle(screen, C_CYAN, fpt, 2)

            screen.blit(trail_surface, (0,0))
            
            pts_screen = [world_to_screen(p[0], p[1]) for p in path_points]
            valid_pts = [p for p in pts_screen if p is not None]
            for pt in valid_pts: pygame.draw.circle(screen, C_YELLOW, pt, 3)

            # Goal
            gx_s, gy_s = world_to_screen(goal_pose['x'], goal_pose['y'])
            if gx_s:
                pygame.draw.circle(screen, C_GREEN, (gx_s, gy_s), 8, 2) 
                ge_x = gx_s - 20 * math.sin(goal_pose['theta']) 
                ge_y = gy_s - 20 * math.cos(goal_pose['theta'])
                pygame.draw.line(screen, C_GREEN, (gx_s, gy_s), (ge_x, ge_y), 2)

            # Robot
            cx_s, cy_s = world_to_screen(car_pose['x'], car_pose['y'])
            if cx_s:
                pygame.draw.circle(screen, C_RED, (cx_s, cy_s), 7)
                ce_x = cx_s + 20 * (-math.sin(car_pose['theta']))
                ce_y = cy_s - 20 * (math.cos(car_pose['theta']))
                pygame.draw.line(screen, C_RED, (cx_s, cy_s), (ce_x, ce_y), 3)

            # HUD
            state_str = STATE_NAMES.get(robot_state_idx, "UNKNOWN")
            
            status_lines = [
                f"POS   : X{car_pose['x']:5.2f} Y{car_pose['y']:5.2f}",
                f"GOAL  : X{goal_pose['x']:5.2f} Y{goal_pose['y']:5.2f}",
                f"STATE : {state_str}",
                f"--- PERF (us) ---",
                f"MAP : {t_map}  GPL: {t_gplan}",
                f"MPL : {t_mplan}",
                f"--- MEM ---",
                f"MIN HEAP: {h_min / 1024:.1f} KB",
                f"--- STACKS (Bytes) ---",
                f"TCP:{s_tcp} GPL:{s_gplan} MPL:{s_mplan}",
                f"LID:{s_lidar} MAP:{s_map} SYN:{s_sync}",
                f"ESP2 LINK: {t_esp2} ms ago"
            ]

            # --- LOGGING SECTION ---

            # --- 1. CONFIGURATION & THRESHOLDS ---
            # CPU Time (microseconds) - High is Bad
            WARN_MAP, CRIT_MAP     = 3000, 10000
            WARN_GPLAN, CRIT_GPLAN = 8000, 50000
            WARN_MPLAN, CRIT_MPLAN = 5000, 20000

            # Heap Memory (Bytes) - Low is Bad
            WARN_HEAP, CRIT_HEAP   = 20000, 10000  

            # Stack High Water Mark (Bytes) - Low is Bad
            # Increased Crit to 500 because 200 is basically one printf away from crashing
            WARN_STACK, CRIT_STACK = 1500, 500 

            # ESP2 Link Latency (ms) - High is Bad
            WARN_LINK, CRIT_LINK   = 200, 2000

            # --- 2. COLORS ---
            C_RST_l = "\033[0m"
            C_RED_l = "\033[91m"
            C_YEL_l = "\033[93m"
            C_GRN_l = "\033[92m"
            C_GRY_l = "\033[90m"
            C_CYN_l = "\033[96m"

            # Helper: Determines color based on value vs thresholds
            def col(val, warn, crit, inverse=False):
                if inverse: # For Memory/Stacks (Lower is Worse)
                    return C_RED_l if val < crit else (C_YEL_l if val < warn else C_GRN_l)
                else:       # For Time/Latency (Higher is Worse)
                    return C_RED_l if val > crit else (C_YEL_l if val > warn else C_GRN_l)

            # --- 3. FORMATTING DATA ---

            # A. State & Position
            st_short = state_str[:3].upper() 
            pos_str = f"{C_CYN_l}[{st_short}]{C_RST_l} {car_pose['x']:>5.1f},{car_pose['y']:>5.1f}"

            # B. Performance (us) - Map, Global, Mission
            c_map = col(t_map, WARN_MAP, CRIT_MAP)
            c_gpl = col(t_gplan, WARN_GPLAN, CRIT_GPLAN)
            c_mpl = col(t_mplan, WARN_MPLAN, CRIT_MPLAN)
            perf_str = f"M{c_map}{t_map:>5}{C_RST_l} G{c_gpl}{t_gplan:>5}{C_RST_l} P{c_mpl}{t_mplan:>5}{C_RST_l}"

            # C. Heap (KB)
            h_min_kb = h_min / 1024.0
            c_heap = col(h_min, WARN_HEAP, CRIT_HEAP, inverse=True)
            mem_str = f"H_Min{c_heap}{h_min_kb:>5.1f}k{C_RST_l}"

            # D. Stacks (Bytes) - Updated for 6 tasks
            # TCP, Global, Mission, Lidar, Map, Sync
            c_stcp = col(s_tcp, WARN_STACK, CRIT_STACK, inverse=True)
            c_sgpl = col(s_gplan, WARN_STACK, CRIT_STACK, inverse=True)
            c_smpl = col(s_mplan, WARN_STACK, CRIT_STACK, inverse=True)
            c_slid = col(s_lidar, WARN_STACK, CRIT_STACK, inverse=True)
            c_smap = col(s_map, WARN_STACK, CRIT_STACK, inverse=True)
            c_ssyn = col(s_sync, WARN_STACK, CRIT_STACK, inverse=True)
            
            # Formatted in two groups: Logic (T,G,M) and Sensors (L,m,S)
            stk_str = (f"Stk:{C_GRY_l}T{C_RST_l}{c_stcp}{s_tcp:>4}{C_RST_l} "
                       f"{C_GRY_l}G{C_RST_l}{c_sgpl}{s_gplan:>4}{C_RST_l} "
                       f"{C_GRY_l}M{C_RST_l}{c_smpl}{s_mplan:>4}{C_RST_l} | "
                       f"{C_GRY_l}L{C_RST_l}{c_slid}{s_lidar:>4}{C_RST_l} "
                       f"{C_GRY_l}m{C_RST_l}{c_smap}{s_map:>4}{C_RST_l} "
                       f"{C_GRY_l}S{C_RST_l}{c_ssyn}{s_sync:>4}{C_RST_l}")

            # E. Link Age (ms)
            c_link = col(t_esp2, WARN_LINK, CRIT_LINK)
            link_str = f"Lnk{c_link}{t_esp2:>4}ms{C_RST_l}"

            # --- 4. FINAL PRINT ---
            print(f"{pos_str} | {perf_str} | {mem_str} | {stk_str} | {link_str}")
            # --- END LOGGING SECTION ---

            for i, line in enumerate(status_lines):
                screen.blit(font.render(line, True, C_BLACK), (12, 12 + i * 20))
                screen.blit(font.render(line, True, C_YELLOW), (10, 10 + i * 20))

            pygame.display.flip()

        except socket.timeout: pass 
        except Exception as e: print(e); break

    client.close(); pygame.quit()

if __name__ == "__main__": main()
