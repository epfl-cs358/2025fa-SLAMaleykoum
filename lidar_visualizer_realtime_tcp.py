#!/usr/bin/env python3
# INSTALL pygame
# BEFORE RUNNING THIS FILE, CONNECT TO THE WIFI CREATED BY THE ESP:
# NAME : LIDAR_AP
# PASSWORD : l1darpass

import socket
import math
import time
import pygame

HOST = "192.168.4.1"
PORT = 9000      # must match your C++ HOST/PORT

SCREEN_SIZE = 700


class TCPViewer:
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
        self.zeroPts = 0
        self.rotationRate = 0.0
        self.scanCount = 0
        self.totalNumPts = 0
        self.startTime = 0

    def init_pygame(self):
        pygame.init()
        pygame.display.set_caption("TCP LIDAR Viewer")
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

    def drawPoints(self):
        self.screen.fill(self.bgColour)

        # draw all points from latest scan
        for x_v, y_v in self.new_points:
            X = int(x_v * self.drawScale) + self.centerX
            Y = int(y_v * self.drawScale) + self.centerY
            rect = (X, Y, self.pointSize, self.pointSize)
            if self.drawRays:
                pygame.draw.line(self.screen, self.rayColour, (X, Y), (self.centerX, self.centerY))
            pygame.draw.rect(self.screen, self.pointColour, rect, 0)
        # update the screen
        pygame.display.update()

        # clear only the *new* points, not the accumulated map
        self.new_points = []
        self.zeroPts = 0
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
        self.new_points = []
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

    def run_as_tcp_client(self, host, port):
        self.init_pygame()

        # TCP client
        cli = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"Connecting to {host}:{port} ...")
        cli.connect((host, port))
        print("Connected!")

        f = cli.makefile("r")
        stopper = False

        last_angle = None
        last_scan_time = time.time()
        try:
            for line in f:
                line = line.strip()
                if not line:
                    stopper = self.handle_input(stopper)
                    if stopper:
                        break
                    continue

                # angle,distance_mm
                try:
                    angle_str, dist_str = line.split(",")
                    angle_deg = float(angle_str)
                    dist_mm = float(dist_str)
                except ValueError:
                    continue

                # end-of-scan detection: angle wrapped around
                if last_angle is not None and angle_deg < last_angle:
                    now = time.time()
                    dt = now - last_scan_time
                    if dt > 0:
                        self.rotationRate = 1.0 / dt
                    last_scan_time = now
                    self.scanCount += 1
                    self.drawPoints()

                last_angle = angle_deg

                if dist_mm > 0:
                    dist_m = dist_mm / 1000.0
                    theta = math.radians(angle_deg)
                    deg90 = math.pi / 2.0

                    Xv = math.cos(theta - (self.aziCorr * math.pi / 180.0)) * dist_m
                    Yv = math.sin(theta - (self.aziCorr * math.pi / 180.0)) * dist_m

                    self.new_points.append([Xv, Yv])
                    self.totalNumPts += 1
                else:
                    self.zeroPts += 1

                stopper = self.handle_input(stopper)
                if stopper:
                    break

        finally:
            cli.close()
            pygame.quit()


if __name__ == "__main__":
    viewer = TCPViewer(SCREEN_SIZE, draw_rays_default=True)
    # Replace with ESP32 AP IP and port
    ESP32_IP = "192.168.4.1"
    ESP32_PORT = 9000
    viewer.run_as_tcp_client(ESP32_IP, ESP32_PORT)