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
GRID_W = 25        # Width of the grid in cells
GRID_H = 25        # Height of the grid in cells
RESOLUTION = 0.4   # Size of one cell in meters (5cm)
WINDOW_SCALE = 25    # Scale factor for the display window

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
    """
    Python-equivalent of the C++ static bool is_frontier_cell(...)
    """

    # 1. Edge filter (identique au C++)
    if x <= 1 or x >= grid.w - 2 or y <= 1 or y >= grid.h - 2:
        return False

    # 2. If occupied → not a frontier
    if grid.prob(x, y) >= 0.5:
        return False

    # 4-neighborhood offsets
    neighbors = [(1,0), (-1,0), (0,1), (0,-1)]

    for dx, dy in neighbors:
        nx = x + dx
        ny = y + dy

        # Bound check (sécurité)
        if nx < 0 or ny < 0 or nx >= grid.w or ny >= grid.h:
            continue

        p = grid.prob(nx, ny)

        # Unknown cell = FREE_BOUND_PROB < p < OCC_BOUND_PROB
        if 0.35 < p < 0.6:
            return True

    return False

def find_frontiers_python(grid):
    frontiers = []

    for y in range(grid.h):
        for x in range(grid.w):

            if is_frontier_cell(grid, x, y):
                # Convert to world coords EXACTLY like C++
                wx = (x - grid.w/2) * grid.res
                wy = - (y - grid.h/2) * grid.res
                frontiers.append((wx, wy))

    return frontiers

# def find_frontiers_python(grid):
#     """
#     Detects frontier points in the occupancy grid.
#     Replicates the exact logic used in the ESP32 C++ code for visual verification.
#     """
#     frontiers = []
#     rows, cols = grid.shape
    
#     # Iterate through grid, skipping the outer 2-pixel border (Edge Filter)
#     for y in range(2, rows - 2):
#         for x in range(2, cols - 2):
#             val = grid[y, x]
            
#             # Condition 1: Cell must be Free
#             # Threshold: >= -2 is Unknown/Occupied. < -2 is Free.
#             if val >= -8: continue 
            
#             # Condition 2: Must have at least one Unknown neighbor (LogOdds == 0)
#             is_frontier = False
#             if grid[y+1, x] == 0: is_frontier = True
#             elif grid[y-1, x] == 0: is_frontier = True
#             elif grid[y, x+1] == 0: is_frontier = True
#             elif grid[y, x-1] == 0: is_frontier = True
            
#             if is_frontier:
#                 wx, wy = index_to_world(x, y)
#                 frontiers.append((wx, wy))
                
#     return frontiers

# =============================================================================
# MAIN
# =============================================================================

def main():
    # --- Pygame Setup ---
    pygame.init()
    screen = pygame.display.set_mode((GRID_W * WINDOW_SCALE, GRID_H * WINDOW_SCALE))
    pygame.display.set_caption("ESP32 Mission Control")
    font = pygame.font.SysFont("Consolas", 16, bold=True)
    
    # Surface for the "Ghost Trail" (path history) with transparency support
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

    # --- Variables ---
    rx_buffer = b''
    HEADER_SIZE = 64
    
    # Robot State
    car_pose = {'x': 0, 'y': 0, 'theta': 0}
    goal_pose = {'x': 0, 'y': 0, 'theta': 0}
    path_points = []
    history_trail = []
    
    # Map Data
    grid_log_odds = np.zeros((GRID_H, GRID_W), dtype=np.int8)
    map_surface = None
    detected_frontiers = [] 

    running = True
    while running:
        # --- Event Handling ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            # Press 'C' to clear the trail history
            if event.type == pygame.KEYDOWN and event.key == pygame.K_c:
                history_trail = []
                trail_surface.fill((0,0,0,0))

        try:
            # --- Network Data Reception ---
            chunk = client.recv(65536)
            if not chunk: break
            rx_buffer += chunk

            while True:
                # 1. Check for complete header
                if len(rx_buffer) < HEADER_SIZE: break 
                
                # 2. Sync to Magic Bytes (0xBEEF)
                header_idx = rx_buffer.find(b'\xBE\xEF')
                if header_idx == -1: 
                    # Discard all but last byte (might be start of magic bytes)
                    rx_buffer = rx_buffer[-1:]
                    break
                if header_idx > 0: 
                    # Discard garbage before header
                    rx_buffer = rx_buffer[header_idx:]
                
                # Double check header size after trim
                if len(rx_buffer) < HEADER_SIZE: break
                
                # 3. Read Dynamic Packet Sizes
                map_bytes_size = struct.unpack('<I', rx_buffer[2:6])[0]
                global_len = struct.unpack('<H', rx_buffer[6:8])[0]
                # local_len  = struct.unpack('<H', rx_buffer[8:10])[0]
                # total_packet_size = HEADER_SIZE + (global_len * 8) + (local_len * 8) + map_bytes_size
                total_packet_size = HEADER_SIZE + (global_len * 8) + map_bytes_size

                # Wait for full packet
                if len(rx_buffer) < total_packet_size: break 
                
                # Extract Packet
                packet = rx_buffer[:total_packet_size]
                rx_buffer = rx_buffer[total_packet_size:] # Remove processed packet from buffer

                # --- Packet Decoding ---
                
                # Decode Poses
                cx, cy, ct = struct.unpack('<fff', packet[8:20])
                gx, gy, gt = struct.unpack('<fff', packet[20:32])
                car_pose = {'x': cx, 'y': cy, 'theta': ct}
                goal_pose = {'x': gx, 'y': gy, 'theta': gt}

                # Update Trail (only if moved > 10cm)
                if not history_trail or math.hypot(cx - history_trail[-1][0], cy - history_trail[-1][1]) > 0.1:
                    history_trail.append((cx, cy))
                    if len(history_trail) > 2000: history_trail.pop(0)
                    pt = world_to_screen(cx, cy)
                    if pt: pygame.draw.circle(trail_surface, (0, 150, 255, 50), pt, 4)

                # Decode Path
                offset = HEADER_SIZE
                path_payload = packet[offset : offset + (global_len * 8)]
                path_points = []
                for i in range(global_len):
                    px, py = struct.unpack('<ff', path_payload[i*8 : (i+1)*8])
                    path_points.append((px, py))

                offset += global_len * 8

                # LOCAL PATH
                # local_path_payload = packet[offset : offset + (local_len * 8)]
                # local_path_payload = packet[offset : offset + (local_len * 8)]

                # local_path_points = []
                # for i in range(local_len):
                #     px, py = struct.unpack('<ff', local_path_payload[i*8 : (i+1)*8])
                #     local_path_points.append((px, py))

                # offset += local_len * 8

                # Decode Map (only if present)
                if map_bytes_size > 0:
                    map_payload = packet[HEADER_SIZE + (global_len * 8) :]
                    rle_data = np.frombuffer(map_payload, dtype=np.uint8)
                    try:
                        # Decompress RLE to 2D Grid
                        grid_log_odds = rle_decode(rle_data, GRID_W * GRID_H).reshape((GRID_H, GRID_W))
                        
                        # Generate RGB Surface
                        rgb_map = np.zeros((GRID_H, GRID_W, 3), dtype=np.uint8)
                        occ_mask  = grid_log_odds > 4
                        free_mask = grid_log_odds < -6
                        unk_mask  = (~free_mask) & (~occ_mask)
                        
                        rgb_map[free_mask] = C_WHITE
                        rgb_map[occ_mask] = C_BLACK
                        rgb_map[unk_mask] = C_GREY
                        
                        surf = pygame.surfarray.make_surface(np.transpose(rgb_map, (1, 0, 2)))
                        map_surface = pygame.transform.scale(surf, (GRID_W * WINDOW_SCALE, GRID_H * WINDOW_SCALE))
                        
                        # Run Python Frontier Detection (for visual debug)
                        detected_frontiers = find_frontiers_python(grid_log_odds)

                    except Exception as e: print(f"Map Error: {e}")

            # --- Rendering ---
            screen.fill(C_GREY)
            
            # Layer 1: Map
            if map_surface: screen.blit(map_surface, (0, 0))
            
            # Layer 2: Detected Frontiers (Cyan Dots)
            
            for fx, fy in detected_frontiers:
                fpt = world_to_screen(fx, fy)
                if fpt: pygame.draw.circle(screen, C_CYAN, fpt, 2)

            # Layer 3: Ghost Trail
            screen.blit(trail_surface, (0,0))
            
            # Layer 4: Planned Path
            pts_screen = [world_to_screen(p[0], p[1]) for p in path_points]
            valid_pts = [p for p in pts_screen if p is not None]
            if len(valid_pts) > 1:
                # pygame.draw.lines(screen, C_YELLOW, False, valid_pts, 3)
                for pt in valid_pts: pygame.draw.circle(screen, C_YELLOW, pt, 2)

            # GLOBAL PATH = jaune
            for px, py in path_points:
                pt = world_to_screen(px, py)
                if pt: pygame.draw.circle(screen, C_YELLOW, pt, 3)

            # LOCAL PATH = cyan
            # for px, py in local_path_points:
            #     pt = world_to_screen(px, py)
            #     if pt: pygame.draw.circle(screen, C_CYAN, pt, 3)

            # Layer 5: Goal (Green)
            gx_s, gy_s = world_to_screen(goal_pose['x'], goal_pose['y'])
            if gx_s:
                pygame.draw.circle(screen, C_GREEN, (gx_s, gy_s), 8, 2) 
                # Heading line for goal
                ge_x = gx_s - 20 * math.sin(goal_pose['theta']) 
                ge_y = gy_s - 20 * math.cos(goal_pose['theta'])
                pygame.draw.line(screen, C_GREEN, (gx_s, gy_s), (ge_x, ge_y), 2)

            # Layer 6: Robot (Red)
            cx_s, cy_s = world_to_screen(car_pose['x'], car_pose['y'])
            if cx_s:
                pygame.draw.circle(screen, C_RED, (cx_s, cy_s), 7)
                # Heading line for robot
                ce_x = cx_s + 20 * (-math.sin(car_pose['theta']))
                ce_y = cy_s - 20 * (math.cos(car_pose['theta']))
                pygame.draw.line(screen, C_RED, (cx_s, cy_s), (ce_x, ce_y), 3)

            # Formater global path
            if len(path_points) > 0:
                gp_str = " ".join([f"({px:.1f},{py:.1f})" for px, py in path_points])
            else:
                gp_str = "NONE"

            # HUD (Text Overlay)
            status_lines = [
                f"POSITION: X{car_pose['x']:5.2f} Y{car_pose['y']:5.2f}",
                f"FRONTIERS: {len(detected_frontiers)}",
                f"GOAL     : X{goal_pose['x']:5.2f} Y{goal_pose['y']:5.2f}",
                f"STATE    : {'NOTHING' if abs(goal_pose['x'])<0.01 and len(detected_frontiers)==0 else 'EXPLORING'}",
                f"GLOBAL   : {gp_str}"
            ]
            for i, line in enumerate(status_lines):
                # Draw black shadow for readability
                screen.blit(font.render(line, True, C_BLACK), (12, 12 + i * 20))
                # Draw yellow text
                screen.blit(font.render(line, True, C_YELLOW), (10, 10 + i * 20))

            pygame.display.flip()

        except socket.timeout: pass 
        except Exception as e: print(e); break

    client.close(); pygame.quit()

if __name__ == "__main__": main()