#!/usr/bin/env python3
# INSTALL pygame
# BEFORE RUNNING THIS FILE, CONNECT TO THE WIFI (SPOT-iot)
# The ESP32 will broadcast UDP packets on port 9000

import socket
import math
import time
import pygame

UDP_PORT = 9000
SCREEN_SIZE = 700


class UDPViewer:
    def __init__(self, screen_size=700, draw_rays_default=True):
        # viewer state
        self.screenSize = screen_size
        self.drawRaysDefault = draw_rays_default
        self.drawRays = draw_rays_default

        self.screen = None
        self.drawScale = 0.08 * screen_size
        self.centerX = 0
        self.centerY = 0
        self.pointSize = 2
        self.myfont = None

        self.aziCorr = 0.0   # rotation correction for the whole scene

        # colors
        self.csXColour = [255, 0, 0]
        self.csYColour = [0, 255, 0]
        self.rayColour = [40, 40, 40]
        self.pointColour = [0, 0, 255]
        self.sensorColour = [0, 0, 0]
        self.bgColour = [20, 20, 20]
        self.textColour = [255, 255, 255]

        # mouse state
        self.leftButton = False
        self.rightButton = False
        self.mouseClickPos = None
        self.rightClickAzi = 0
        self.lastLeftClickTime = 0
        self.lastRightClickTime = 0
        self.lastMidClickTime = 0
        self.doubleClickTime = 0.25

        # scan state
        self.new_points = []
        self.current_scan = []  # Accumulate points for current scan
        self.zeroPts = 0
        self.rotationRate = 0.0
        self.scanCount = 0
        self.totalNumPts = 0
        self.startTime = 0
        self.last_angle = None
        self.last_scan_time = time.time()

    def init_pygame(self):
        pygame.init()
        pygame.display.set_caption("UDP LIDAR Viewer")
        self.myfont = pygame.font.SysFont("Arial", 13)
        self.screen = pygame.display.set_mode([self.screenSize, self.screenSize])
        self.screen.fill(self.bgColour)
        self.centerX = int(self.screenSize / 2)
        self.centerY = int(self.screenSize / 2)
        pygame.display.update()
        self.startTime = time.time()
        self.lastLeftClickTime = self.startTime
        self.lastRightClickTime = self.startTime
        self.lastMidClickTime = self.startTime

    def drawGrid(self):
        """Draws a metric grid: 1 m spacing (thin white), 5 m spacing (thicker)."""
        # grid color
        thin = (180, 180, 180)      # light grey/white thin lines
        thick = (255, 255, 255)     # bright white for 5 m lines

        # compute pixels per meter
        px_per_m = self.drawScale

        # Determine how many meters fit on screen
        max_m = int(self.screenSize / px_per_m) + 5

        # Vertical lines
        for m in range(-max_m, max_m + 1):
            x = int(self.centerX + m * px_per_m)
            if m % 5 == 0:
                color = thick
                width = 2
            else:
                color = thin
                width = 1
            pygame.draw.line(self.screen, color, (x, 0), (x, self.screenSize), width)

        # Horizontal lines
        for m in range(-max_m, max_m + 1):
            y = int(self.centerY + m * px_per_m)
            if m % 5 == 0:
                color = thick
                width = 3
            else:
                color = thin
                width = 1
            pygame.draw.line(self.screen, color, (0, y), (self.screenSize, y), width)

    def drawPoints(self):
        self.screen.fill(self.bgColour)
        self.drawGrid()

        # draw all points from latest scan
        for x_v, y_v in self.new_points:
            X = int(x_v * self.drawScale) + self.centerX
            Y = int(y_v * self.drawScale) + self.centerY
            rect = (X, Y, self.pointSize, self.pointSize)
            if self.drawRays:
                pygame.draw.line(self.screen, self.rayColour, (X, Y), (self.centerX, self.centerY))
            pygame.draw.rect(self.screen, self.pointColour, rect, 0)

        # text overlay (rotation, counts, etc.)
        def put_text(label, value, row):
            ts = self.myfont.render(label, False, self.textColour)
            pygame.draw.rect(self.screen, self.bgColour, (5, 5 + 15 * row, ts.get_width() + 20, ts.get_height()), 0)
            self.screen.blit(ts, (5, 5 + 15 * row))

            ts2 = self.myfont.render(value, False, self.textColour)
            pygame.draw.rect(self.screen, self.bgColour, (105, 5 + 15 * row, ts2.get_width() + 20, ts2.get_height()), 0)
            self.screen.blit(ts2, (105, 5 + 15 * row))

        put_text("Rotation Rate:", f"{self.rotationRate:.1f} Hz", 0)
        put_text("Points in Scan:", f"{len(self.new_points)} / {self.zeroPts}", 1)
        put_text("Pulses / sec:", f"{int((len(self.new_points) + self.zeroPts) * self.rotationRate)}", 2)
        put_text("Duration:", f"{time.time() - self.startTime:.1f} sec", 3)
        put_text("Total Scans:", f"{self.scanCount}", 4)
        put_text("Total Points:", f"{self.totalNumPts}", 5)

        ts = self.myfont.render(
            "Controls: Mouse  +  -  [  ]  <  >  arrows  r  c  z  o  i  Esc",
            False,
            self.textColour,
        )
        pygame.draw.rect(
            self.screen,
            self.bgColour,
            (5, self.screenSize - 25, ts.get_width() + 20, ts.get_height()),
            0,
        )
        self.screen.blit(ts, (5, self.screenSize - 25))

        # draw sensor circle
        pygame.draw.circle(self.screen, self.sensorColour, [self.centerX, self.centerY], 7, 0)
        pygame.draw.circle(self.screen, [255, 255, 255], [self.centerX, self.centerY], 7, 1)

        # coordinate axes that rotate with aziCorr
        csAngle = self.aziCorr * math.pi / 180.0
        x1 = y1 = x2 = y2 = 0.0

        if int(self.aziCorr) in (0, 360):
            x1, y1 = self.screenSize, self.centerY
            x2, y2 = self.centerX, 0
        elif int(self.aziCorr) == 90:
            x1, y1 = self.centerX, 0
            x2, y2 = 0, self.centerY
        elif int(self.aziCorr) == 180:
            x1, y1 = 0, self.centerY
            x2, y2 = self.centerX, self.screenSize
        elif int(self.aziCorr) == 270:
            x1, y1 = self.centerX, self.screenSize
            x2, y2 = self.screenSize, self.centerY
        elif 0 < self.aziCorr < 90:
            x1 = self.screenSize
            y1 = self.centerY - math.tan(csAngle) * (self.screenSize - self.centerX)
            x2 = self.centerX - math.tan(csAngle) * self.centerY
            y2 = 0
        elif 90 < self.aziCorr < 180:
            csAngle -= 1.5 * math.pi
            x1 = self.centerX - math.tan(csAngle) * self.centerY
            y1 = 0
            x2 = 0
            y2 = self.centerY + math.tan(csAngle) * self.centerX
        elif 180 < self.aziCorr < 270:
            csAngle -= math.pi
            x1 = 0
            y1 = self.centerY + math.tan(csAngle) * self.centerX
            x2 = self.centerX + math.tan(csAngle) * (self.screenSize - self.centerY)
            y2 = self.screenSize
        elif 270 < self.aziCorr < 360:
            csAngle -= 0.5 * math.pi
            x1 = self.centerX + math.tan(csAngle) * (self.screenSize - self.centerY)
            y1 = self.screenSize
            x2 = self.screenSize
            y2 = self.centerY - math.tan(csAngle) * (self.screenSize - self.centerX)

        pygame.draw.line(self.screen, self.csXColour, (self.centerX, self.centerY), (int(x1), int(y1)))
        pygame.draw.line(self.screen, self.csYColour, (self.centerX, self.centerY), (int(x2), int(y2)))

        pygame.display.update()

        # reset scan-specific counters
        self.zeroPts = 0

    def handle_input(self, stopper):
        funcStopper = stopper
        for events in pygame.event.get():
            if events.type == pygame.QUIT:
                funcStopper = True
            elif events.type == pygame.KEYDOWN:
                if events.key == pygame.K_ESCAPE:
                    funcStopper = True
                elif events.key == pygame.K_RIGHTBRACKET:
                    self.pointSize = min(self.pointSize + 1, 5)
                elif events.key == pygame.K_LEFTBRACKET:
                    self.pointSize = max(self.pointSize - 1, 1)
                elif events.key == pygame.K_EQUALS:
                    self.drawScale = min(self.drawScale * 1.1, 500)
                elif events.key == pygame.K_MINUS:
                    self.drawScale = max(self.drawScale * 0.9, 10)
                elif events.key == pygame.K_DOWN:
                    self.centerY = min(self.centerY + 20, int(self.screenSize * 1.25))
                elif events.key == pygame.K_UP:
                    self.centerY = max(self.centerY - 20, int(-0.25 * self.screenSize))
                elif events.key == pygame.K_RIGHT:
                    self.centerX = min(self.centerX + 20, int(self.screenSize * 1.25))
                elif events.key == pygame.K_LEFT:
                    self.centerX = max(self.centerX - 20, int(-0.25 * self.screenSize))
                elif events.key == pygame.K_i:
                    self.centerX = int(self.screenSize / 2)
                    self.centerY = int(self.screenSize / 2)
                    self.drawScale = 0.08 * self.screenSize
                    self.pointSize = 2
                    self.aziCorr = 0.0
                    self.drawRays = self.drawRaysDefault
                elif events.key == pygame.K_PERIOD:
                    self.aziCorr -= 2
                    if self.aziCorr < 0:
                        self.aziCorr += 360.0
                elif events.key == pygame.K_COMMA:
                    self.aziCorr += 2
                    if self.aziCorr > 360:
                        self.aziCorr -= 360.0
                elif events.key == pygame.K_z:
                    self.drawScale = 0.08 * self.screenSize
                elif events.key == pygame.K_r:
                    self.drawRays = not self.drawRays
                elif events.key == pygame.K_c:
                    self.centerX = int(self.screenSize / 2)
                    self.centerY = int(self.screenSize / 2)
                elif events.key == pygame.K_o:
                    self.aziCorr = 0.0

            elif events.type == pygame.MOUSEBUTTONDOWN:
                if events.button == 1:
                    self.leftButton = True
                    self.mouseClickPos = pygame.mouse.get_pos()
                elif events.button == 3:
                    self.rightButton = True
                    self.mouseClickPos = pygame.mouse.get_pos()
                    refAzi = math.atan2(
                        (self.mouseClickPos[0] - self.centerX),
                        (self.centerY - self.mouseClickPos[1]),
                    )
                    if refAzi < 0:
                        refAzi += 2.0 * math.pi
                    self.rightClickAzi = refAzi
                elif events.button == 4:
                    self.drawScale = min(self.drawScale * 1.1, 500)
                elif events.button == 5:
                    self.drawScale = max(self.drawScale * 0.9, 10)

            elif events.type == pygame.MOUSEBUTTONUP:
                nowTime = time.time()
                if events.button == 1:
                    self.leftButton = False
                    if nowTime - self.lastLeftClickTime < self.doubleClickTime:
                        self.centerX = int(self.screenSize / 2)
                        self.centerY = int(self.screenSize / 2)
                    self.lastLeftClickTime = nowTime
                elif events.button == 2:
                    if nowTime - self.lastMidClickTime < self.doubleClickTime:
                        self.drawScale = 0.08 * self.screenSize
                    self.lastMidClickTime = nowTime
                elif events.button == 3:
                    self.rightButton = False
                    if nowTime - self.lastRightClickTime < self.doubleClickTime:
                        self.aziCorr = 0.0
                    self.lastRightClickTime = nowTime

            elif events.type == pygame.MOUSEMOTION:
                if self.leftButton:
                    currentMousePos = pygame.mouse.get_pos()
                    relX = currentMousePos[0] - self.mouseClickPos[0]
                    relY = currentMousePos[1] - self.mouseClickPos[1]
                    self.centerX += relX
                    self.centerY += relY
                    self.mouseClickPos = currentMousePos
                elif self.rightButton:
                    currentMousePos = pygame.mouse.get_pos()
                    currentAzi = math.atan2(
                        (currentMousePos[0] - self.centerX),
                        (self.centerY - currentMousePos[1]),
                    )
                    deltaAzi = (currentAzi - self.rightClickAzi) * (180.0 / math.pi)
                    self.aziCorr -= deltaAzi
                    if self.aziCorr < 0:
                        self.aziCorr += 360.0
                    elif self.aziCorr > 360:
                        self.aziCorr -= 360.0
                    self.rightClickAzi = currentAzi

        return funcStopper

    def run_as_udp_receiver(self, port):
        self.init_pygame()

        # UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', port))
        sock.setblocking(False)  # Non-blocking mode for pygame event loop
        
        print(f"Listening for UDP packets on port {port}...")

        stopper = False
        points_received = 0

        try:
            while not stopper:
                # Handle pygame events
                stopper = self.handle_input(stopper)
                
                # Try to receive UDP data
                try:
                    data, addr = sock.recvfrom(2048)  # Larger buffer for batched packets
                    lines = data.decode().strip().split('\n')
                    
                    for line in lines:
                        if not line:
                            continue
                        try:
                            angle_str, dist_str = line.split(',')
                            angle = float(angle_str)
                            distance = float(dist_str)
                            
                            # Convert polar to Cartesian (distance in mm, convert to meters)
                            distance_m = distance / 1000.0
                            
                            # Apply azimuth correction
                            angle_rad = math.radians(angle + self.aziCorr)
                            
                            # Convert to x, y coordinates
                            x = distance_m * math.sin(angle_rad)
                            y = distance_m * math.cos(angle_rad)
                            
                            # Add to current scan
                            self.current_scan.append((x, y))
                            points_received += 1
                            
                            # Detect scan completion (angle wraps around)
                            if self.last_angle is not None:
                                if angle < self.last_angle - 180:  # Wrapped around 0°
                                    # Complete scan received
                                    current_time = time.time()
                                    scan_duration = current_time - self.last_scan_time
                                    
                                    if scan_duration > 0:
                                        self.rotationRate = 1.0 / scan_duration
                                    
                                    # Update display data
                                    self.new_points = self.current_scan.copy()
                                    self.totalNumPts += len(self.current_scan)
                                    self.scanCount += 1
                                    
                                    # Draw the completed scan
                                    self.drawPoints()
                                    
                                    # Reset for next scan
                                    self.current_scan = []
                                    self.last_scan_time = current_time
                                    
                                    print(f"Scan #{self.scanCount}: {len(self.new_points)} points, {self.rotationRate:.1f} Hz")
                            
                            self.last_angle = angle
                            
                        except ValueError as e:
                            print(f"Parse error: {e} - Line: {line}")
                            pass

                except socket.error:
                    # No data available, just continue
                    pass

                # Small delay to prevent busy waiting
                pygame.time.wait(1)

        finally:
            sock.close()
            pygame.quit()
            print(f"\nTotal points received: {points_received}")


if __name__ == "__main__":
    viewer = UDPViewer(SCREEN_SIZE, draw_rays_default=True)
    viewer.run_as_udp_receiver(UDP_PORT)