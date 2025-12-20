import pandas as pd
import matplotlib.pyplot as plt

# Load Data
try:
    df = pd.read_csv('telemetry.csv')
except FileNotFoundError:
    print("Error: 'telemetry.csv' not found. Please rename your log file.")
    exit()

# Create relative time (Seconds starting from 0)
df['Relative_Time_s'] = df['Timestamp'] - df['Timestamp'].iloc[0]

# Setup Plotting
fig, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

# PLOT 1: Computation Time (Latency)
# We plot in Milliseconds (ms) for readability
axes[0].plot(df['Relative_Time_s'], df['Global_Calc_us'] / 1000.0, 
             label='Global Planner', color='red', marker='o', markersize=3)
axes[0].plot(df['Relative_Time_s'], df['Map_Calc_us'] / 1000.0, 
             label='Map Update', color='blue')
axes[0].plot(df['Relative_Time_s'], df['Mission_Calc_us'] / 1000.0, 
             label='Mission Calc', color='orange', alpha=0.7)
axes[0].set_ylabel('Compute Time (ms)')
axes[0].set_title('Task Execution Time')
axes[0].legend()
axes[0].grid(True, alpha=0.3)

# PLOT 2: Frame Counters (The "Starvation" Check)
# If these lines are parallel, the system is healthy. 
# If Map goes flat while Lidar rises, you have starvation.
axes[1].plot(df['Relative_Time_s'], df['Lidar_Frames'], 
             label='Lidar Frames (Prod)', color='green')
axes[1].plot(df['Relative_Time_s'], df['Map_Frames'], 
             label='Map Frames (Cons)', color='purple')
axes[1].set_ylabel('Total Frames')
axes[1].set_title('Throughput Analysis (Lidar vs Map)')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

# PLOT 3: Failures and Queue Load
ax3a = axes[2]
ax3b = ax3a.twinx()

# Plot failures on left axis
ax3a.plot(df['Relative_Time_s'], df['Planner_Fails'], 
          label='Planner Fail Count', color='black', linewidth=2)
ax3a.set_ylabel('Failure Count', color='black')
ax3a.tick_params(axis='y', labelcolor='black')

# Plot queue load on right axis
ax3b.plot(df['Relative_Time_s'], df['Queue_Load_Pct'], 
          label='Queue Load %', color='darkorange', linewidth=1.5, alpha=0.7)
ax3b.set_ylabel('Queue Load (%)', color='darkorange')
ax3b.tick_params(axis='y', labelcolor='darkorange')
ax3b.set_ylim(0, 100)

ax3a.set_xlabel('Time (seconds)')
ax3a.set_title('Global Planner Failures & Queue Load')
ax3a.grid(True, alpha=0.3)

# Combine legends
lines1, labels1 = ax3a.get_legend_handles_labels()
lines2, labels2 = ax3b.get_legend_handles_labels()
ax3a.legend(lines1 + lines2, labels1 + labels2, loc='upper left')

plt.tight_layout()
plt.show()

print(f"\n📊 Summary Statistics:")
print(f"   Total runtime: {df['Relative_Time_s'].iloc[-1]:.2f} seconds")
print(f"   Total lidar frames: {df['Lidar_Frames'].iloc[-1]}")
print(f"   Total map frames: {df['Map_Frames'].iloc[-1]}")
print(f"   Total planner failures: {df['Planner_Fails'].iloc[-1]}")
print(f"   Avg Global Calc time: {df['Global_Calc_us'].mean() / 1000:.2f} ms")
print(f"   Avg Map Calc time: {df['Map_Calc_us'].mean() / 1000:.2f} ms")
print(f"   Avg Queue Load: {df['Queue_Load_Pct'].mean():.1f}%")
print(f"   Max Queue Load: {df['Queue_Load_Pct'].max():.1f}%")