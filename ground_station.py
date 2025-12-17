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

# Task Colors for Timeline
TASK_COLORS = {
    1: (180, 0, 255),   # LIDAR (Purple)
    2: (0, 0, 255),     # SYNC (Blue)
    3: (255, 165, 0),   # MAP (Orange)
    4: (0, 255, 255),   # IPC (Cyan)
    5: (255, 255, 0),   # GPLAN (Yellow)
    6: (255, 0, 0),     # MPLAN (Red)
    7: (128, 128, 128), # TCP (Grey)
    8: (0, 255, 0)      # VALIDATOR (Green)
}

TASK_NAMES = {
    0: "IDLE", 1: "LIDAR", 2: "SYNC", 3: "MAP",
    4: "IPC", 5: "GPLAN", 6: "MPLAN", 7: "TCP", 8: "VALID"
}

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
# TIMELINE VISUALIZER CLASS
# =============================================================================

class Timeline:
    def __init__(self, x, y, w, h):
        self.rect = pygame.Rect(x, y, w, h)
        self.history = [] # List of dicts: {'start', 'end', 'core', 'id'}
        self.active_tasks = {} # Key: task_id, Value: start_time
        self.max_history_us = 1000000 # Show last 1 second of data
        self.font = pygame.font.SysFont("Arial", 11)
        
    def add_events(self, events):
        """
        Process trace events: list of (timestamp_us, task_id, event_type, core_id)
        event_type: 1=START, 0=END
        """
        for ts, tid, typ, core in events:
            if typ == 1: # START
                self.active_tasks[tid] = ts
            elif typ == 0: # END
                if tid in self.active_tasks:
                    start = self.active_tasks.pop(tid)
                    duration = ts - start
                    # Filter outliers (duration must be positive and reasonable)
                    if 0 < duration < 5000000:
                        self.history.append({
                            'start': start, 
                            'end': ts, 
                            'core': core, 
                            'id': tid
                        })

        # Prune old history (keep only last max_history_us worth of data)
        if self.history:
            self.history.sort(key=lambda x: x['end'])
            latest = self.history[-1]['end']
            limit = latest - self.max_history_us
            self.history = [e for e in self.history if e['end'] > limit]

    def draw(self, surface):
        """Draw the timeline visualization"""
        # Background
        pygame.draw.rect(surface, (20, 20, 20), self.rect)
        pygame.draw.rect(surface, (255, 255, 255), self.rect, 2) # Border

        # Core Separator Line
        mid_y = self.rect.centery
        pygame.draw.line(surface, (100, 100, 100), 
                        (self.rect.left, mid_y), 
                        (self.rect.right, mid_y))
        
        # Core Labels
        core0_label = self.font.render("CORE 0 (Comms/Plan)", True, (200, 200, 200))
        core1_label = self.font.render("CORE 1 (Sensors)", True, (200, 200, 200))
        surface.blit(core0_label, (self.rect.left + 5, self.rect.top + 5))
        surface.blit(core1_label, (self.rect.left + 5, mid_y + 5))

        if not self.history: 
            return

        latest_time = self.history[-1]['end']
        # Scale: pixels per microsecond
        scale = self.rect.width / float(self.max_history_us)
        
        for e in self.history:
            # Calculate position relative to the right edge (latest time)
            time_ago = latest_time - e['end']
            duration = e['end'] - e['start']
            
            width = max(2, duration * scale)
            x_end = self.rect.right - (time_ago * scale)
            x_start = x_end - width
            
            # Skip if off-screen left
            if x_end < self.rect.left: 
                continue

            # Determine Y position based on Core
            row_height = (self.rect.height / 2) - 25
            if e['core'] == 0:
                y = self.rect.top + 25
            else:
                y = mid_y + 25
            
            col = TASK_COLORS.get(e['id'], (255, 255, 255))
            
            # Draw Task Block
            task_rect = pygame.Rect(x_start, y, width, row_height)
            
            # Clip to timeline view
            draw_rect = task_rect.clip(self.rect)
            if draw_rect.width > 0:
                pygame.draw.rect(surface, col, draw_rect)

# =============================================================================
# MAIN
# =============================================================================
def main():
    # CSV Logging
    log_filename = f"telemetry_trace_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    print(f"Logging trace data to: {log_filename}")
    csv_file = open(log_filename, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Timestamp_PC", "ESP_Time_us", "Task_ID", "Core", "Event_Type"])

    # --- Pygame Setup ---
    pygame.init()
    
    # Increase window height for timeline
    timeline_height = 150
    screen_height = GRID_H * WINDOW_SCALE + timeline_height
    screen = pygame.display.set_mode((GRID_W * WINDOW_SCALE, screen_height))
    pygame.display.set_caption("ESP32 Mission Control + Timeline")
    
    font = pygame.font.SysFont("Consolas", 16, bold=True)
    small_font = pygame.font.SysFont("Consolas", 12)
    
    trail_surface = pygame.Surface((GRID_W * WINDOW_SCALE, GRID_H * WINDOW_SCALE), 
                                   pygame.SRCALPHA)

    # Create Timeline at bottom of screen
    timeline = Timeline(0, GRID_H * WINDOW_SCALE, GRID_W * WINDOW_SCALE, timeline_height)

    # --- Network Connection ---
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.settimeout(5.0) 
    try: 
        print(f"Connecting to {ESP_IP}:{PORT}...")
        client.connect((ESP_IP, PORT))
        print("Connected!")
    except Exception as e: 
        print(f"Connection Failed: {e}")
        csv_file.close()
        return

    rx_buffer = b''
    HEADER_SIZE = 96
    
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

    # Performance tracking
    last_packet_time = time.time()
    packets_received = 0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: 
                running = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_c:
                history_trail = []
                trail_surface.fill((0, 0, 0, 0))

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
                
                # Read Sizes from header
                map_bytes_size = struct.unpack('<I', rx_buffer[2:6])[0]
                global_len = struct.unpack('<H', rx_buffer[6:8])[0]
                
                # NEW: Read trace count (comes right after header)
                if len(rx_buffer) < HEADER_SIZE + 2: break
                trace_count = struct.unpack('<H', rx_buffer[HEADER_SIZE:HEADER_SIZE+2])[0]
                
                # Calculate total packet size
                # Header(96) + TraceCount(2) + Path + Map + TraceEvents
                trace_bytes = trace_count * 8  # 8 bytes per event
                total_packet_size = HEADER_SIZE + 2 + (global_len * 8) + map_bytes_size + trace_bytes

                if len(rx_buffer) < total_packet_size: break 
                
                packet = rx_buffer[:total_packet_size]
                rx_buffer = rx_buffer[total_packet_size:] 
                packets_received += 1

                # Track packet timing
                now = time.time()
                packet_interval_ms = (now - last_packet_time) * 1000.0
                last_packet_time = now

                # --- DECODING ---
                
                # Poses
                cx, cy, ct = struct.unpack('<fff', packet[8:20])
                gx, gy, gt = struct.unpack('<fff', packet[20:32])
                
                robot_state_idx = packet[32] 
                
                # Health Data
                try:
                    health_data = packet[33:33+64]  # Updated to 64 bytes
                    h_free, h_min, t_map, t_gplan, t_mplan, s_tcp, s_gplan, s_mplan, s_lidar, t_esp2 = \
                        struct.unpack('<IIIIIHHHHI', health_data[:32])
                except:
                    print("Health data decode error")
                    continue

                car_pose = {'x': cx, 'y': cy, 'theta': ct}
                goal_pose = {'x': gx, 'y': gy, 'theta': gt}

                # Update Trail
                if not history_trail or math.hypot(cx - history_trail[-1][0], cy - history_trail[-1][1]) > 0.1:
                    history_trail.append((cx, cy))
                    if len(history_trail) > 2000: history_trail.pop(0)
                    pt = world_to_screen(cx, cy)
                    if pt: pygame.draw.circle(trail_surface, (0, 150, 255, 50), pt, 4)

                # Path (comes after trace count)
                path_offset = HEADER_SIZE + 2
                path_payload = packet[path_offset : path_offset + (global_len * 8)]
                path_points = []
                for i in range(global_len):
                    px, py = struct.unpack('<ff', path_payload[i*8 : (i+1)*8])
                    path_points.append((px, py))

                # Map
                if map_bytes_size > 0:
                    map_offset = path_offset + (global_len * 8)
                    map_payload = packet[map_offset : map_offset + map_bytes_size]
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
                        map_surface = pygame.transform.scale(surf, 
                                                            (GRID_W * WINDOW_SCALE, 
                                                             GRID_H * WINDOW_SCALE))
                        
                        detected_frontiers = find_frontiers_python(grid_log_odds)

                    except Exception as e: 
                        print(f"Map Error: {e}")

                # NEW: Decode Trace Events
                trace_offset = HEADER_SIZE + 2 + (global_len * 8) + map_bytes_size
                trace_payload = packet[trace_offset : trace_offset + trace_bytes]
                
                parsed_events = []
                for i in range(trace_count):
                    event_bytes = trace_payload[i*8 : (i+1)*8]
                    # Format: uint32 timestamp, uint8 task_id, uint8 type, uint8 core, uint8 padding
                    ts, tid, typ, core, _ = struct.unpack('<IBBBB', event_bytes)
                    parsed_events.append((ts, tid, typ, core))
                    
                    # Log to CSV
                    csv_writer.writerow([now, ts, tid, core, typ])
                
                # Add events to timeline
                if parsed_events:
                    timeline.add_events(parsed_events)

            # --- RENDERING ---
            screen.fill(C_GREY)
            
            # 1. Map
            if map_surface: 
                screen.blit(map_surface, (0, 0))
            
            # 2. Frontiers
            for fx, fy in detected_frontiers:
                fpt = world_to_screen(fx, fy)
                if fpt: pygame.draw.circle(screen, C_CYAN, fpt, 2)

            # 3. Trail
            screen.blit(trail_surface, (0, 0))
            
            # 4. Path
            pts_screen = [world_to_screen(p[0], p[1]) for p in path_points]
            valid_pts = [p for p in pts_screen if p is not None]
            for pt in valid_pts: 
                pygame.draw.circle(screen, C_YELLOW, pt, 3)

            # 5. Goal
            gx_s, gy_s = world_to_screen(goal_pose['x'], goal_pose['y'])
            if gx_s:
                pygame.draw.circle(screen, C_GREEN, (gx_s, gy_s), 8, 2) 
                ge_x = gx_s - 20 * math.sin(goal_pose['theta']) 
                ge_y = gy_s - 20 * math.cos(goal_pose['theta'])
                pygame.draw.line(screen, C_GREEN, (gx_s, gy_s), (ge_x, ge_y), 2)

            # 6. Robot
            cx_s, cy_s = world_to_screen(car_pose['x'], car_pose['y'])
            if cx_s:
                pygame.draw.circle(screen, C_RED, (cx_s, cy_s), 7)
                ce_x = cx_s + 20 * (-math.sin(car_pose['theta']))
                ce_y = cy_s - 20 * (math.cos(car_pose['theta']))
                pygame.draw.line(screen, C_RED, (cx_s, cy_s), (ce_x, ce_y), 3)

            # 7. Timeline
            timeline.draw(screen)

            # 8. Task Legend (below timeline)
            legend_y = GRID_H * WINDOW_SCALE + 5
            legend_x = 10
            for tid, name in TASK_NAMES.items():
                if tid == 0: continue  # Skip IDLE
                col = TASK_COLORS.get(tid, (255, 255, 255))
                pygame.draw.rect(screen, col, (legend_x, legend_y, 12, 12))
                screen.blit(small_font.render(name, True, (200, 200, 200)), 
                           (legend_x + 16, legend_y - 2))
                legend_x += 70

            # 9. HUD (Performance Stats)
            state_str = STATE_NAMES.get(robot_state_idx, "UNKNOWN")
            
            # DIAGNOSTIC: Extract extended health data
            try:
                if len(health_data) >= 64:
                    extended_health = struct.unpack('<IIIIIIIIIIIIIIII', health_data)
                    lidar_rx = extended_health[10] if len(extended_health) > 10 else 0
                    map_updates = extended_health[11] if len(extended_health) > 11 else 0
                    sync_drops = extended_health[12] if len(extended_health) > 12 else 0
                    mutex_fails = extended_health[13] if len(extended_health) > 13 else 0
                else:
                    lidar_rx = map_updates = sync_drops = mutex_fails = 0
            except:
                lidar_rx = map_updates = sync_drops = mutex_fails = 0
            
            status_lines = [
                f"POS   : X{car_pose['x']:5.2f} Y{car_pose['y']:5.2f}",
                f"GOAL  : X{goal_pose['x']:5.2f} Y{goal_pose['y']:5.2f}",
                f"STATE : {state_str}",
                f"--- PERF (us) ---",
                f"MAP UPDATE : {t_map}",
                f"GLOB PLAN  : {t_gplan}",
                f"MISS PLAN  : {t_mplan}",
                f"--- CORE 1 DIAG ---",
                f"LIDAR RX  : {lidar_rx}",
                f"MAP UPDT  : {map_updates}",
                f"SYNC DROP : {sync_drops}",
                f"MTX FAIL  : {mutex_fails}",
                f"--- MEM (KB) ---",
                f"FREE HEAP : {h_free / 1024:.1f} KB",
                f"MIN HEAP  : {h_min / 1024:.1f} KB",
                f"--- STACKS ---",
                f"TCP:{s_tcp} GP:{s_gplan} MP:{s_mplan} LI:{s_lidar}",
                f"ESP2:{t_esp2}ms PKT:{packets_received}",
                f"TRACE: {trace_count} events",
            ]

            # Draw HUD with shadow
            for i, line in enumerate(status_lines):
                screen.blit(font.render(line, True, C_BLACK), (12, 12 + i * 20))
                screen.blit(font.render(line, True, C_YELLOW), (10, 10 + i * 20))

            pygame.display.flip()

        except socket.timeout: 
            pass 
        except Exception as e: 
            print(f"Error: {e}")
            break

    csv_file.close()
    client.close()
    pygame.quit()
    print(f"\nSession complete. Log saved to: {log_filename}")

if __name__ == "__main__": 
    main()