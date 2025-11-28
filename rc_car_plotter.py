import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math

CSV_FILE = 'rc_car_telemetry.csv'

# --- 1. Define the Expected Path (From C++ Code) ---
pathX = [
    0.00, 0.00, 0.00, 0.00, 0.50, 1.50, 2.00, 2.50, 2.80, 2.80, 2.80, 2.80, 2.80
]

pathY = [
    0.00, 0.50, 1.00, 1.50, 1.50, 1.50, 1.50, 1.50, 1.50, 2.00, 2.50, 2.00, 3.40
]

def plot_data():
    try:
        # Load Data
        df = pd.read_csv(CSV_FILE)
    except FileNotFoundError:
        print(f"Could not find {CSV_FILE}. Run the logger script first.")
        return

    # Create figure with 2 subplots (Map and Steering/Control)
    fig = plt.figure(figsize=(10, 8))
    gs = fig.add_gridspec(3, 1)

    # --- PLOT 1: Top-Down Map (2/3 of screen) ---
    ax_map = fig.add_subplot(gs[0:2, 0])
    
    # 1. Plot Expected Path
    ax_map.plot(pathX, pathY, 'g--', linewidth=2, label='Expected Path (Waypoints)')
    ax_map.scatter(pathX, pathY, color='green', s=20, alpha=0.5)

    # 2. Plot Real Path Trace
    ax_map.plot(df['X_Current'], df['Y_Current'], 'b-', linewidth=1, alpha=0.5, label='Real Path Trace')
    
    # 3. Draw "Car" Rectangles to show orientation clearly
    # Car dimensions (approximate, adjust based on your real world scale)
    CAR_L = 0.08  # Length
    CAR_W = 0.04  # Width
    
    # Subsample to avoid overcrowding the plot
    step = max(1, len(df) // 15) 
    
    for i in range(0, len(df), step):
        x = df['X_Current'][i]
        y = df['Y_Current'][i]
        yaw = df['Yaw_Current'][i] # Radians
        
        # Calculate directional vectors based on Yaw
        # Note: In C++ logic: 0 rad = +Y (Up), rotates Counter-Clockwise
        dir_x = -math.sin(yaw)
        dir_y = math.cos(yaw)
        
        # Perpendicular vector (Right side of car)
        perp_x = math.cos(yaw)
        perp_y = math.sin(yaw)
        
        # Calculate the 4 corners of the rotated rectangle
        # Center + (Forward/Backward * L/2) +/- (Right/Left * W/2)
        
        # Front Left
        fl_x = x + (dir_x * CAR_L/2) - (perp_x * CAR_W/2)
        fl_y = y + (dir_y * CAR_L/2) - (perp_y * CAR_W/2)
        
        # Front Right
        fr_x = x + (dir_x * CAR_L/2) + (perp_x * CAR_W/2)
        fr_y = y + (dir_y * CAR_L/2) + (perp_y * CAR_W/2)
        
        # Back Right
        br_x = x - (dir_x * CAR_L/2) + (perp_x * CAR_W/2)
        br_y = y - (dir_y * CAR_L/2) + (perp_y * CAR_W/2)
        
        # Back Left
        bl_x = x - (dir_x * CAR_L/2) - (perp_x * CAR_W/2)
        bl_y = y - (dir_y * CAR_L/2) - (perp_y * CAR_W/2)
        
        # Create Polygon
        poly = plt.Polygon([[fl_x, fl_y], [fr_x, fr_y], [br_x, br_y], [bl_x, bl_y]], 
                           closed=True, color='blue', alpha=0.6, edgecolor='black', zorder=10)
        ax_map.add_patch(poly)
        
        # Add a small white dot to indicate the "Head" (Front) of the car
        head_x = x + (dir_x * CAR_L * 0.3)
        head_y = y + (dir_y * CAR_L * 0.3)
        ax_map.plot(head_x, head_y, 'w.', markersize=4, zorder=11)

    # Custom Legend for the Car Box
    # (We add a dummy patch to the legend to represent the car)
    from matplotlib.patches import Patch
    legend_elements = [
        plt.Line2D([0], [0], color='g', linestyle='--', label='Expected Path'),
        plt.Line2D([0], [0], color='b', alpha=0.5, label='Real Path Trace'),
        Patch(facecolor='blue', edgecolor='black', alpha=0.6, label='Car Position & Orientation')
    ]
    ax_map.legend(handles=legend_elements)

    ax_map.set_title("Autonomous Car Trajectory Analysis")
    ax_map.set_xlabel("X Position (Right)")
    ax_map.set_ylabel("Y Position (Forward)")
    ax_map.grid(True)
    ax_map.set_aspect('equal') # Crucial for accurate spatial representation

    # --- PLOT 2: Steering Command vs Time (1/3 of screen) ---
    ax_steer = fig.add_subplot(gs[2, 0])
    
    # Plot Steering Command
    ax_steer.plot(df['time_s'], df['delta_target_CMD'], 'r-', label='Steering Servo Angle (deg)')
    
    # Draw a line at 90 (Straight)
    ax_steer.axhline(y=90, color='k', linestyle='--', alpha=0.5, label='Straight (90°)')

    ax_steer.set_title("Steering Command Over Time")
    ax_steer.set_xlabel("Time (s)")
    ax_steer.set_ylabel("Servo Angle (Degrees)")
    ax_steer.legend()
    ax_steer.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_data()

