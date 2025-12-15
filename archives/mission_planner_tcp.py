import pygame
import numpy as np
import socket
import struct
import threading
import math
import re

# ============ Viewer Settings ============
GRID_RESOLUTION = 0.05
CELL_SIZE = 5
WINDOW_W, WINDOW_H = 800, 800

zoom = 1.0
offset_x = 0
offset_y = 0

# TCP settings
ESP32_IP = "192.168.4.1"  # AP mode
ESP32_PORT = 9000

# Globals updated by network thread
grid_data = None
robot_pose = [0.0, 0.0, 0.0]
goal_pose = [None, None, None]  # x, y, theta
goal_type = -1
grid_w = 0
grid_h = 0

lock = threading.Lock()

# ================================
# TCP READER THREAD
# ================================
def tcp_thread_func():
    global grid_data, robot_pose, goal_pose, goal_type, grid_w, grid_h

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ESP32_IP, ESP32_PORT))
    print("[Python] Connected to ESP32")

    buffer = b""

    while True:
        # ---- Read header ----
        header = sock.recv(20)
        if len(header) < 20:
            continue

        if header[0] != 0xAB or header[1] != 0xCD:
            continue

        # grid size
        w = (header[2] << 8) | header[3]
        h = (header[4] << 8) | header[5]

        # robot pose (floats)
        rx = struct.unpack("f", header[6:10])[0]
        ry = struct.unpack("f", header[10:14])[0]
        rt = struct.unpack("f", header[14:18])[0]

        map_size = (header[18] << 8) | header[19]

        # ---- Read map ----
        map_bytes = b""
        while len(map_bytes) < map_size:
            map_bytes += sock.recv(map_size - len(map_bytes))

        arr = np.frombuffer(map_bytes, dtype=np.uint8).reshape((h, w))

        # ---- Read goal line ----
        goal_line = sock.recv(64).decode(errors="ignore")

        match = re.search(
            r"Goal:\s*x=([-\d\.]+),\s*y=([-\d\.]+),\s*theta=([-\d\.]+),\s*type=(\d+)",
            goal_line
        )

        gpx = gpy = gpt = None
        gtype = -1

        if match:
            gpx = float(match.group(1))
            gpy = float(match.group(2))
            gpt = float(match.group(3))
            gtype = int(match.group(4))

        with lock:
            grid_data = arr
            robot_pose = [rx, ry, rt]
            goal_pose = [gpx, gpy, gpt]
            goal_type = gtype
            grid_w, grid_h = w, h


# ================================
# FUNCTIONS FOR DRAWING
# ================================
def world_to_grid(x, y):
    gx = x / GRID_RESOLUTION + grid_w/2
    gy = -y / GRID_RESOLUTION + grid_h/2
    return int(gx), int(gy)


def draw_robot(surface, pose, offset_x, offset_y, scale_factor):
    if grid_w == 0:
        return

    x_m, y_m, theta = pose
    gx, gy = world_to_grid(x_m, y_m)

    gx = gx * CELL_SIZE * zoom * scale_factor + offset_x
    gy = gy * CELL_SIZE * zoom * scale_factor + offset_y

    L = 10 * zoom * scale_factor
    W =  6 * zoom * scale_factor

    pts = [(0, -L), (W/2, L/2), (-W/2, L/2)]

    cos_r = math.cos(theta)
    sin_r = math.sin(theta)

    tpts = []
    for px, py in pts:
        rx = px*cos_r - py*sin_r
        ry = px*sin_r + py*cos_r
        tpts.append((gx+rx, gy+ry))

    pygame.draw.polygon(surface, (255, 0, 0), tpts)


def draw_goal(surface, goal, offset_x, offset_y, scale_factor):
    if goal[0] is None:
        return

    gx, gy, _ = goal
    gx, gy = world_to_grid(gx, gy)

    gx = gx * CELL_SIZE * zoom * scale_factor + offset_x
    gy = gy * CELL_SIZE * zoom * scale_factor + offset_y

    pygame.draw.circle(surface, (0, 120, 255), (int(gx), int(gy)), 6)


# ================================
# MAIN VIEWER LOOP
# ================================
pygame.init()
window = pygame.display.set_mode((WINDOW_W, WINDOW_H))
clock = pygame.time.Clock()

# Start TCP thread
threading.Thread(target=tcp_thread_func, daemon=True).start()

running = True
while running:
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False

        if e.type == pygame.MOUSEWHEEL:
            zoom *= 1.1 if e.y > 0 else 0.9

        if pygame.mouse.get_pressed()[0]:
            dx, dy = pygame.mouse.get_rel()
            offset_x += dx
            offset_y += dy
        else:
            pygame.mouse.get_rel()

    window.fill((0, 0, 0))

    with lock:
        if grid_data is not None:
            gx = grid_w * CELL_SIZE * zoom
            gy = grid_h * CELL_SIZE * zoom

            scale_factor = min(WINDOW_W / gx, WINDOW_H / gy)

            new_w = int(gx * scale_factor)
            new_h = int(gy * scale_factor)

            surf = pygame.Surface((grid_w, grid_h))
            pygame.surfarray.blit_array(surf, np.stack([grid_data]*3, axis=-1))
            surf = pygame.transform.scale(surf, (new_w, new_h))

            pos_x = (WINDOW_W - new_w) // 2 + offset_x
            pos_y = (WINDOW_H - new_h) // 2 + offset_y

            window.blit(surf, (pos_x, pos_y))

            draw_robot(window, robot_pose, pos_x, pos_y, scale_factor)
            draw_goal(window, goal_pose, pos_x, pos_y, scale_factor)

    pygame.display.update()
    clock.tick(60)

pygame.quit()
