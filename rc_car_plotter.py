import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math
import matplotlib.cm as cm

TELEMETRY_FILE = 'rc_car_telemetry.csv'
PATHS_FILE = 'rc_car_paths.csv'

def plot_data():
    try:
        df_telem = pd.read_csv(TELEMETRY_FILE)
    except FileNotFoundError:
        print(f"Could not find {TELEMETRY_FILE}. Run logger.")
        return

    # Try to read paths file (might not exist if no path sent yet)
    df_paths = pd.DataFrame()
    try:
        df_paths = pd.read_csv(PATHS_FILE)
    except FileNotFoundError:
        print(f"No paths file found yet. Plotting only car position.")

    fig = plt.figure(figsize=(10, 8))
    gs = fig.add_gridspec(3, 1)

    # --- PLOT 1: Top-Down Map ---
    ax_map = fig.add_subplot(gs[0:2, 0])
    
    # 1. Plot All Received Paths
    if not df_paths.empty and 'path_id' in df_paths.columns:
        path_ids = df_paths['path_id'].unique()
        # Create a color map of Greens. 
        # We skip the very light greens (0.0-0.3) so they are visible against white background
        colors = cm.Greens(np.linspace(0.4, 1.0, len(path_ids)))
        
        for i, pid in enumerate(path_ids):
            path_data = df_paths[df_paths['path_id'] == pid].sort_values('index')
            color = colors[i]
            label = f'Path ID {pid}' if i == len(path_ids)-1 else None # Only label last one to avoid clutter
            
            # Plot line
            ax_map.plot(path_data['x'], path_data['y'], color=color, linestyle='--', linewidth=2, label=label)
            # Plot dots
            ax_map.scatter(path_data['x'], path_data['y'], color=color, s=20, alpha=0.6)
            
    # 2. Real Path
    ax_map.plot(df_telem['X_Current'], df_telem['Y_Current'], 'b-', linewidth=1, alpha=0.6, label='Real Car Path')
    
    # 3. Lookahead Lines
    step = max(1, len(df_telem) // 35)  # Limit to ~35 lines for clarity
    for i in range(0, len(df_telem), step):
        ax_map.plot([df_telem['X_Current'][i], df_telem['Lookahead_X'][i]], 
                    [df_telem['Y_Current'][i], df_telem['Lookahead_Y'][i]], 
                    color='gray', linestyle=':', alpha=0.3)

    # 4. Draw Car Box
    CAR_L = 0.15 
    CAR_W = 0.10 
    
    for i in range(0, len(df_telem), step):
        x = df_telem['X_Current'][i]
        y = df_telem['Y_Current'][i]
        yaw = df_telem['Yaw_Current'][i]
        
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
        
        head_x = x + (dir_x * CAR_L * 0.4)
        head_y = y + (dir_y * CAR_L * 0.4)
        ax_map.plot(head_x, head_y, 'w.', markersize=3, zorder=11)

    # 5. Settings Box
    if 'Kp' in df_telem.columns:
        last_kp = df_telem['Kp'].iloc[-1]
        last_ld = df_telem['Ld'].iloc[-1]
        text_str = f'Settings:\nKp = {last_kp}\nLd = {last_ld} m'
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
        ax_map.text(0.02, 0.95, text_str, transform=ax_map.transAxes, fontsize=10,
                    verticalalignment='top', bbox=props)

    ax_map.set_title("Dynamic Path Following")
    ax_map.set_xlabel("X (m)")
    ax_map.set_ylabel("Y (m)")
    ax_map.legend()
    ax_map.grid(True)
    ax_map.set_aspect('equal')

    # --- PLOT 2: Steering ---
    ax_steer = fig.add_subplot(gs[2, 0])
    ax_steer.plot(df_telem['time_s'], df_telem['delta_target_CMD'], 'r-', label='Steering Angle')
    ax_steer.axhline(y=90, color='k', linestyle='--', alpha=0.5)
    ax_steer.set_title("Steering Command")
    ax_steer.set_xlabel("Time (s)")
    ax_steer.set_ylabel("Degrees")
    ax_steer.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_data()

