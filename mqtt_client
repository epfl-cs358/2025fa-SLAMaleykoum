# To run this file, run in a terminal this command: python mqtt_client.py
# You must have python installed and paho-mqtt (pip install paho-mqtt matplotlib)
import paho.mqtt.client as mqtt
import json
import math
import matplotlib.pyplot as plt
import time

# Minimum time (in seconds) between prints per topic
PRINT_INTERVAL = 0.5  
last_print_times = {}

# Stockage temporaire des points LIDAR
lidar_points = []
last_angle = None
previous_angle = None
rotation_detected = False

def plot_lidar_points(points):
    """Trace les points LIDAR en coordonnées polaires converties en XY."""
    if not points:
        print("Aucun point à afficher.")
        return

    xs = []
    ys = []

    for p in points:
        angle_deg = p.get("angle")
        dist = p.get("distance")
        if dist is None or angle_deg is None:
            continue

        # Conversion polaire → cartésienne
        angle_rad = math.radians(angle_deg)
        x = dist * math.cos(angle_rad)
        y = dist * math.sin(angle_rad)
        xs.append(x)
        ys.append(y)

    plt.figure()
    plt.scatter(xs, ys, s=5)
    plt.title("Carte LIDAR - 1 tour complet")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.axis("equal")
    plt.grid(True)
    plt.show()

def on_message(client, userdata, msg):
    global last_angle, previous_angle, rotation_detected, lidar_points
    topic = msg.topic
    now = time.time()
    last_time = last_print_times.get(topic, 0)

    if now - last_time < PRINT_INTERVAL:
        return
    last_print_times[topic] = now

    try:
        data = json.loads(msg.payload.decode())
        sensor_type = data.get("type")

        if sensor_type == "lidar":
            points = data.get("points", [])
            for p in points:
                angle = p.get("angle")
                dist = p.get("distance")

                # Détection de tour complet
                if previous_angle is not None and angle is not None:
                    # Si on revient à un angle proche de 0 après 360, c'est un nouveau tour
                    if previous_angle > 350 and angle < 10:
                        rotation_detected = True

                previous_angle = angle
                lidar_points.append(p)

            if rotation_detected:
                print("\n=== TOUR COMPLET DÉTECTÉ ===")
                print(f"{len(lidar_points)} points collectés. Génération de la carte...")
                plot_lidar_points(lidar_points)
                lidar_points = []  # reset pour le prochain tour
                rotation_detected = False

        else:
            # Les autres capteurs
            if sensor_type == "print":
                print(data.get("message"))
            elif sensor_type == "imu":
                print(f"IMU: ax={data.get('ax')}, ay={data.get('ay')}, az={data.get('az')}")
            elif sensor_type == "ultrasonic":
                print(f"Ultrasonic: {data.get('distance')} m")
            elif sensor_type == "encoder":
                print(f"Encoder: {data.get('angle')}°")
            elif sensor_type == "servo":
                print(f"Servo: {data.get('angle')}°")
            else:
                print("Unknown sensor type or format:", data)

    except Exception as e:
        print("Error decoding message:", e)

# MQTT setup
client = mqtt.Client()
client.connect("broker.hivemq.com", 1883)
client.subscribe("slamaleykoum77/#")  # adapte à ton topic
client.on_message = on_message

print("📡 En écoute sur le broker... (Ctrl+C pour arrêter)")
try:
    client.loop_forever()
except KeyboardInterrupt:
    print("\nArrêt par l'utilisateur.")
    client.disconnect()