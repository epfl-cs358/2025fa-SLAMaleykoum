import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys

# =========================================================
# CONFIGURATION
# =========================================================
# Replace with your actual CSV filename
CSV_FILE = "telemetry_trace.csv" 

# FIXED: Use ESP timestamps, not PC timestamps!
# Set to None to plot entire capture, or specify a time window
ZOOM_WINDOW_SECONDS = None  # Show all data
START_OFFSET_SECONDS = 0.0  # Start from beginning

# Task definitions matching your C++
TASK_COLORS = {
    1: 'purple', # LIDAR
    2: 'blue',   # SYNC
    3: 'orange', # MAP
    4: 'cyan',   # IPC
    5: 'gold',   # GPLAN
    6: 'red',    # MPLAN
    7: 'grey',   # TCP
    8: 'green'   # VALIDATOR
}

TASK_NAMES = {
    1: "LIDAR (C1)", 
    2: "SYNC (C1)", 
    3: "MAP (C1)",
    4: "IPC (C0)", 
    5: "GPLAN (C0)", 
    6: "MPLAN (C0)", 
    7: "TCP (C0)", 
    8: "VALID (C0)"
}

# =========================================================
# PLOTTING LOGIC
# =========================================================
def plot_gantt(filename):
    try:
        df = pd.read_csv(filename)
    except FileNotFoundError:
        print(f"Error: File {filename} not found.")
        return

    print(f"Loaded {len(df)} events")
    print(f"PC Time range: {df['Timestamp_PC'].min():.3f} to {df['Timestamp_PC'].max():.3f}")
    print(f"ESP Time range: {df['ESP_Time_us'].min()} to {df['ESP_Time_us'].max()} us")
    
    # FIXED: Use ESP timestamps for plotting, not PC timestamps!
    # Normalize to start at 0
    min_esp_time = df['ESP_Time_us'].min()
    df['Rel_Time_ms'] = (df['ESP_Time_us'] - min_esp_time) / 1000.0
    
    print(f"Relative time range: 0 to {df['Rel_Time_ms'].max():.1f} ms")

    # Apply time window filters if specified
    if START_OFFSET_SECONDS:
        start_ms = START_OFFSET_SECONDS * 1000
        df = df[df['Rel_Time_ms'] >= start_ms]
        print(f"After START_OFFSET: {len(df)} events remaining")
        
    if ZOOM_WINDOW_SECONDS:
        if START_OFFSET_SECONDS:
            end_ms = START_OFFSET_SECONDS * 1000 + ZOOM_WINDOW_SECONDS * 1000
        else:
            end_ms = ZOOM_WINDOW_SECONDS * 1000
        df = df[df['Rel_Time_ms'] <= end_ms]
        print(f"After ZOOM_WINDOW: {len(df)} events remaining")

    if df.empty:
        print("ERROR: No data in specified window!")
        print("Try setting START_OFFSET_SECONDS = 0.0 and ZOOM_WINDOW_SECONDS = None")
        return

    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(16, 10), sharex=True)
    
    # Process events to find (Start, End) pairs
    active_stacks = {} # Key: TaskID, Value: StartTime
    bars_drawn = 0

    print(f"\nProcessing {len(df)} events...")

    for index, row in df.iterrows():
        tid = int(row['Task_ID'])
        core = int(row['Core'])
        typ = int(row['Event_Type']) # 1=Start, 0=End
        time_ms = row['Rel_Time_ms']
        
        # Select Axis based on Core
        ax = ax1 if core == 1 else ax0

        if typ == 1: # START
            active_stacks[tid] = time_ms
        elif typ == 0: # END
            if tid in active_stacks:
                start_ms = active_stacks.pop(tid)
                duration = time_ms - start_ms
                
                # Filter noise (negative durations or too long)
                if duration <= 0 or duration > 100:
                    continue

                bars_drawn += 1
                
                # Y-position based on Task ID
                y_pos = tid
                color = TASK_COLORS.get(tid, 'black')
                label = TASK_NAMES.get(tid, str(tid))
                
                rect = patches.Rectangle(
                    (start_ms, y_pos - 0.4),  # (x, y)
                    duration,                  # width
                    0.8,                       # height
                    linewidth=0.5,
                    edgecolor='black',
                    facecolor=color,
                    alpha=0.8
                )
                ax.add_patch(rect)
                
                # Add text label if bar is wide enough (>1ms)
                if duration > 1.0: 
                    ax.text(start_ms + duration/2, y_pos, label, 
                            ha='center', va='center', fontsize=7, 
                            color='white', fontweight='bold')

    print(f"Drew {bars_drawn} task bars")

    # Formatting Core 0
    ax0.set_title("CORE 0 (Communication & Planning)", fontsize=14, fontweight='bold')
    ax0.set_ylabel("Task", fontsize=12)
    ax0.set_ylim(3.5, 8.5)
    ax0.set_yticks([4, 5, 6, 7, 8])
    # ax0.set_yticklabels([TASK_NAMES.get(i, str(i)) for i in [4,5,6,7,8]])
    ax0.grid(True, which='both', axis='x', linestyle='--', alpha=0.3)
    ax0.set_xlim(df['Rel_Time_ms'].min(), df['Rel_Time_ms'].max())

    # Formatting Core 1
    ax1.set_title("CORE 1 (Sensors & Mapping)", fontsize=14, fontweight='bold')
    ax1.set_ylabel("Task", fontsize=12)
    ax1.set_xlabel("Time (ms since start of capture)", fontsize=12)
    ax1.set_ylim(0.5, 3.5)
    ax1.set_yticks([1, 2, 3])
    # ax1.set_yticklabels([TASK_NAMES.get(i, str(i)) for i in [1,2,3]])
    ax1.grid(True, which='both', axis='x', linestyle='--', alpha=0.3)
    ax1.set_xlim(df['Rel_Time_ms'].min(), df['Rel_Time_ms'].max())

    # Add legend
    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor=TASK_COLORS[i], label=TASK_NAMES[i]) 
                      for i in sorted(TASK_NAMES.keys())]
    fig.legend(handles=legend_elements, loc='upper right', ncol=2)

    plt.tight_layout()
    plt.savefig('gantt_chart.png', dpi=150, bbox_inches='tight')
    print("\nSaved to: gantt_chart.png")
    plt.show()

if __name__ == "__main__":
    # If passed arg, use that file
    if len(sys.argv) > 1:
        plot_gantt(sys.argv[1])
    else:
        plot_gantt(CSV_FILE)

        