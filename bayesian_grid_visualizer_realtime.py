import pygame
import numpy as np
import socket
import struct
import threading
import math

# ============ Viewer Settings ============
GRID_RESOLUTION = 0.05
CELL_SIZE = 5
WINDOW_W, WINDOW_H = 800, 800

zoom = 1.0
offset_x = 0
offset_y = 0

# TCP settings
ESP32_IP = "172.21.72.29"  # AP mode
ESP32_PORT = 9000

# Globals updated by network thread
grid_data = None
robot_pose = [0.0, 0.0, 0.0]
grid_w = 0
grid_h = 0

lock = threading.Lock()

# ================================
# TCP READER THREAD
# ================================
def tcp_thread_func():
    global grid_data, robot_pose, grid_w, grid_h

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ESP32_IP, ESP32_PORT))
    print("[Python] Connected to ESP32")

    while True:
        # ---- Read header ----
        header = sock.recv(20)
        if len(header) < 20:
            continue

        sync1, sync2 = header[0], header[1]
        if sync1 != 0xAB or sync2 != 0xCD:
            continue  # wait for sync

        # grid size
        w = (header[2] << 8) | header[3]
        h = (header[4] << 8) | header[5]

        # robot pose (floats)
        rx = struct.unpack('f', header[6:10])[0]
        ry = struct.unpack('f', header[10:14])[0]
        rt = struct.unpack('f', header[14:18])[0]

        map_size = (header[18] << 8) | header[19]

        # ---- Read map ----
        map_bytes = b''
        while len(map_bytes) < map_size:
            map_bytes += sock.recv(map_size - len(map_bytes))

        arr = np.frombuffer(map_bytes, dtype=np.uint8)
        arr = arr.reshape((h, w))

        # ---- Update globals safely ----
        with lock:
            grid_data = arr
            robot_pose = [rx, ry, rt]
            grid_w, grid_h = w, h


# ================================
# FUNCTIONS FOR DRAWING
# ================================
def world_to_grid(x, y):
    gx = x / GRID_RESOLUTION + grid_w/2
    gy = -y / GRID_RESOLUTION + grid_h/2
    return int(gx), int(gy)

def grid_to_surface(grid):
    surf = pygame.Surface((grid.shape[1], grid.shape[0]))
    pygame.surfarray.blit_array(surf, np.stack([grid]*3, axis=-1))
    return surf

def draw_robot(surface, pose):
    if grid_w == 0: return

    x_m, y_m, theta = pose
    gx, gy = world_to_grid(x_m, y_m)

    gx = gx * CELL_SIZE * zoom + offset_x
    gy = gy * CELL_SIZE * zoom + offset_y

    L = 10 * zoom
    W = 6 * zoom

    pts = [(0,-L), (W/2, L/2), (-W/2, L/2)]

    cos_r = math.cos(theta)
    sin_r = math.sin(theta)

    tpts = []
    for px,py in pts:
        rx = px*cos_r - py*sin_r
        ry = px*sin_r + py*cos_r
        tpts.append((gx+rx, gy+ry))

    pygame.draw.polygon(surface, (255,0,0), tpts)


# ================================
# MAIN VIEWER LOOP
# ================================
pygame.init()
window = pygame.display.set_mode((WINDOW_W, WINDOW_H))
clock = pygame.time.Clock()

# start TCP thread
threading.Thread(target=tcp_thread_func, daemon=True).start()

running = True
while running:
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False

        if e.type == pygame.MOUSEWHEEL:
            zoom *= 1.1 if e.y > 0 else 0.9

        if pygame.mouse.get_pressed()[0]:
            mx,my = pygame.mouse.get_pos()
            dx,dy = pygame.mouse.get_rel()
            offset_x += dx
            offset_y += dy
        else:
            pygame.mouse.get_rel()

    window.fill((0,0,0))

    with lock:
        if grid_data is not None:
            # scale map to fill full window while keeping aspect ratio
            gx = grid_w * CELL_SIZE * zoom
            gy = grid_h * CELL_SIZE * zoom

            scale_factor = min(WINDOW_W / gx, WINDOW_H / gy)

            new_w = int(gx * scale_factor)
            new_h = int(gy * scale_factor)

            surf = grid_to_surface(grid_data)
            surf = pygame.transform.scale(surf, (new_w, new_h))

            # center on screen
            pos_x = (WINDOW_W - new_w) // 2
            pos_y = (WINDOW_H - new_h) // 2

            window.blit(surf, (pos_x + offset_x, pos_y + offset_y))
            draw_robot(window, robot_pose)

    pygame.display.update()
    clock.tick(60)

pygame.quit()