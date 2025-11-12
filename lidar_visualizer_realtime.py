import paho.mqtt.client as mqtt
import json
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import threading
import re

# MQTT setup
BROKER = "broker.hivemq.com"
TOPIC_BASE = "slamaleykoum77/#"

# Throttling for debug messages
PRINT_INTERVAL = 0.5
last_print_times = {}

# Shared data
lidar_points = []
lock = threading.Lock()

# ---- Real-time plotting ----
fig, ax = plt.subplots()
scat = ax.scatter([], [], s=4)
ax.set_xlim(-20, 20)   # in meters
ax.set_ylim(-20, 20)
ax.set_aspect('equal')
ax.set_title("🌐 LIDAR EXPRESS Mode - Real-time Map")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.grid(True)

def update_plot(frame):
    """Called periodically to update the scatter points."""
    with lock:
        if not lidar_points:
            return scat,
        xs, ys = [], []
        for p in lidar_points[-1000:]:  # keep last N points visible
            angle = p.get("angle")
            dist = p.get("distance")
            if angle is None or dist is None or dist == 0:
                continue
            dist_m = dist / 1000.0
            angle_rad = math.radians(angle)
            xs.append(dist_m * math.cos(angle_rad))
            ys.append(dist_m * math.sin(angle_rad))
        scat.set_offsets(list(zip(xs, ys)))
    return scat,

# ---- MQTT callbacks ----
def on_message(client, userdata, msg):
    global lidar_points
    topic = msg.topic
    payload = msg.payload.decode(errors="ignore").strip()

    # Print debug lines
    if topic.endswith("/print1"):
        now = time.time()
        if now - last_print_times.get(topic, 0) >= PRINT_INTERVAL:
            print("🪵", payload)
            last_print_times[topic] = now

        # Try to parse lidar values from the text
        match = re.search(r'angle:\s*([-+]?[0-9]*\.?[0-9]+)\s*;\s*dist:\s*([0-9]+)', payload)
        if match:
            angle = float(match.group(1))
            dist = float(match.group(2))
            with lock:
                lidar_points.append({"angle": angle, "distance": dist})
                if len(lidar_points) > 5000:
                    lidar_points = lidar_points[-3000:]
        return
    
    if topic.endswith("/lidar"):
        try:
            points = json.loads(payload)
            with lock:
                lidar_points.extend(points)
                lidar_points = lidar_points[-3000:]
        except Exception as e:
            print("JSON parse error:", e)

    # # (keep this if you also send JSON in /lidar)
    # if topic.endswith("/lidar"):
    #     try:
    #         data = json.loads(payload)
    #         if data.get("type") != "lidar":
    #             return
    #         points = data.get("points", [])
    #         with lock:
    #             lidar_points.extend(points)
    #             if len(lidar_points) > 5000:
    #                 lidar_points = lidar_points[-3000:]
    #     except Exception as e:
    #         print("⚠️ Error decoding LIDAR payload:", e)

def mqtt_loop():
    client = mqtt.Client()
    client.on_message = on_message
    client.connect(BROKER, 1883)
    client.subscribe(TOPIC_BASE)
    print(f"📡 Connected to {BROKER}, subscribed to {TOPIC_BASE}")
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("\nArrêt par l'utilisateur.")
        client.disconnect()

# ---- Start MQTT listener in background thread ----
mqtt_thread = threading.Thread(target=mqtt_loop, daemon=True)
mqtt_thread.start()

# ---- Start matplotlib animation ----
ani = animation.FuncAnimation(fig, update_plot, interval=200, blit=True)
plt.show()