import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math

CSV_FILE = 'rc_car_telemetry.csv'

# --- 1. Define the Expected Path ---
pathX = [
    0.00, 0.00, 0.50, 1.00, 1.50, 2.00, 2.50, 3.00, 3.50, 4.00, 4.00, 4.00, 4.00, 4.00, 4.00, 3.50, 3.00, 2.50, 2.00, 1.50, 1.00, 0.50, 0.00, -0.20
]

pathY = [
    0.00, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.50, 1.00, 1.50, 2.00, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40
]

def plot_data():
    try:
        df = pd.read_csv(CSV_FILE)
    except FileNotFoundError:
        print(f"Could not find {CSV_FILE}. Run the logger script first.")
        return

    fig = plt.figure(figsize=(10, 8))
    gs = fig.add_gridspec(3, 1)

    # --- PLOT 1: Top-Down Map ---
    ax_map = fig.add_subplot(gs[0:2, 0])
    
    # 1. Expected Path
    ax_map.plot(pathX, pathY, 'g--', linewidth=2, label='Waypoints')
    ax_map.scatter(pathX, pathY, color='green', s=30, alpha=0.5)

    # 2. Real Path
    ax_map.plot(df['X_Current'], df['Y_Current'], 'b-', linewidth=1, alpha=0.6, label='Real Car Path')
    
    # 3. Lookahead Points (Target)
    # REMOVED: Red cross markers as requested to reduce overcrowding
    # ax_map.scatter(df['Lookahead_X'], df['Lookahead_Y'], color='red', s=15, marker='x', label='Lookahead Target', alpha=0.5)

    # Define density of visualization (Higher divisor = More cars/lines)
    # Changed from 30 to 100 to show significantly more car positions
    step = max(1, len(df) // 32) 

    # 4. Connect Car to Lookahead (Visualizes the "String" pulling the car)
    for i in range(0, len(df), step):
        ax_map.plot([df['X_Current'][i], df['Lookahead_X'][i]], 
                    [df['Y_Current'][i], df['Lookahead_Y'][i]], 
                    color='gray', linestyle=':', alpha=0.3)

    # 5. Draw Car Box
    CAR_L = 0.15  # 15cm length
    CAR_W = 0.10  # 10cm width
    
    for i in range(0, len(df), step):
        x = df['X_Current'][i]
        y = df['Y_Current'][i]
        yaw = df['Yaw_Current'][i]
        
        dir_x = -math.sin(yaw)
        dir_y = math.cos(yaw)
        perp_x = math.cos(yaw)
        perp_y = math.sin(yaw)
        
        fl_x = x + (dir_x * CAR_L/2) - (perp_x * CAR_W/2)
        fl_y = y + (dir_y * CAR_L/2) - (perp_y * CAR_W/2)
        fr_x = x + (dir_x * CAR_L/2) + (perp_x * CAR_W/2)
        fr_y = y + (dir_y * CAR_L/2) + (perp_y * CAR_W/2)
        br_x = x - (dir_x * CAR_L/2) + (perp_x * CAR_W/2)
        br_y = y - (dir_y * CAR_L/2) + (perp_y * CAR_W/2)
        bl_x = x - (dir_x * CAR_L/2) - (perp_x * CAR_W/2)
        bl_y = y - (dir_y * CAR_L/2) - (perp_y * CAR_W/2)
        
        poly = plt.Polygon([[fl_x, fl_y], [fr_x, fr_y], [br_x, br_y], [bl_x, bl_y]], 
                           closed=True, color='blue', alpha=0.3, edgecolor='black', zorder=10)
        ax_map.add_patch(poly)
        
        # Front dot
        head_x = x + (dir_x * CAR_L * 0.4)
        head_y = y + (dir_y * CAR_L * 0.4)
        ax_map.plot(head_x, head_y, 'w.', markersize=3, zorder=11)

    # 6. Display Kp and Ld Settings
    if 'Kp' in df.columns and 'Ld' in df.columns:
        last_kp = df['Kp'].iloc[-1]
        last_ld = df['Ld'].iloc[-1]
        text_str = f'Settings:\nKp = {last_kp}\nLd = {last_ld} m'
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
        ax_map.text(0.02, 0.95, text_str, transform=ax_map.transAxes, fontsize=10,
                    verticalalignment='top', bbox=props)

    ax_map.set_title("Autonomous Car Trajectory & Lookahead")
    ax_map.set_xlabel("X (m)")
    ax_map.set_ylabel("Y (m)")
    ax_map.legend()
    ax_map.grid(True)
    ax_map.set_aspect('equal')

    # --- PLOT 2: Steering ---
    ax_steer = fig.add_subplot(gs[2, 0])
    ax_steer.plot(df['time_s'], df['delta_target_CMD'], 'r-', label='Steering Angle')
    ax_steer.axhline(y=90, color='k', linestyle='--', alpha=0.5)
    ax_steer.set_title("Steering Command")
    ax_steer.set_xlabel("Time (s)")
    ax_steer.set_ylabel("Degrees")
    ax_steer.legend()
    ax_steer.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_data()