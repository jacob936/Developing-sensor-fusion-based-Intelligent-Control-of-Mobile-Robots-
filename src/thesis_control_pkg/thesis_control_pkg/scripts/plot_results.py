#!/usr/bin/env python3
"""
Thesis Plot Script - FRESH VERSION
Uses YOUR exact CSV columns: time_sec,state_x,state_y,...
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import sys

print("Loading CSV...")

# Get filename from command line
csv_file = sys.argv[1] if len(sys.argv) > 1 else None
output_dir = sys.argv[2] if len(sys.argv) > 2 else '/root/sensor_fusion_ws/experiment_data/plots/'

if not csv_file:
    print("Error: No CSV file specified!")
    print("Usage: python3 plot_thesis_results.py <csv_file> [output_dir]")
    sys.exit(1)

print(f"Input: {csv_file}")
print(f"Output: {output_dir}")

# Create output directory
os.makedirs(output_dir, exist_ok=True)

# Load CSV
df = pd.read_csv(csv_file)
print(f"Columns: {list(df.columns)}")
print(f"Rows: {len(df)}")

# YOUR COLUMNS: time_sec,state_x,state_y,state_theta,ref_x,ref_y,error_x,error_y,error_dist,cmd_v,cmd_omega
# Use them directly - NO renaming!

# Compute metrics
time_col = 'time_sec'
error_col = 'error_dist'
rmse = np.sqrt(np.mean(df[error_col]**2))
max_err = np.max(df[error_col])
mean_err = np.mean(df[error_col])
duration = df[time_col].max() - df[time_col].min()

print(f"\n📈 METRICS:")
print(f"   RMSE: {rmse:.3f} m")
print(f"   Max Error: {max_err:.3f} m")
print(f"   Duration: {duration:.1f} s")

# Plot 1: Trajectory
plt.figure(figsize=(8, 8))
plt.plot(df['ref_x'], df['ref_y'], 'go--', label='Waypoints', markersize=10, linewidth=2)
plt.plot(df['state_x'], df['state_y'], 'b-', label='Robot Path', linewidth=1.5)
plt.plot(df['state_x'].iloc[0], df['state_y'].iloc[0], 'ks', markersize=15, label='Start')
plt.plot(df['state_x'].iloc[-1], df['state_y'].iloc[-1], 'r*', markersize=20, label='End')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Robot Trajectory')
plt.legend()
plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.tight_layout()
traj_file = os.path.join(output_dir, 'trajectory.png')
plt.savefig(traj_file, dpi=300)
plt.close()
print(f"✓ Saved: {traj_file}")

# Plot 2: Error
plt.figure(figsize=(10, 4))
plt.plot(df[time_col], df[error_col], 'b-', linewidth=1.5, label='Tracking Error')
plt.axhline(y=rmse, color='r', linestyle='--', linewidth=1.5, label=f'RMSE = {rmse:.3f}m')
plt.axhline(y=mean_err, color='g', linestyle=':', linewidth=1.5, label=f'Mean = {mean_err:.3f}m')
plt.xlabel('Time (s)')
plt.ylabel('Error (m)')
plt.title('Tracking Error Over Time')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
error_file = os.path.join(output_dir, 'error.png')
plt.savefig(error_file, dpi=300)
plt.close()
print(f"✓ Saved: {error_file}")

# Plot 3: Control
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
ax1.plot(df[time_col], df['cmd_v'], 'b-', linewidth=1.5)
ax1.set_ylabel('v (m/s)')
ax1.set_title('Control Commands')
ax1.grid(True, alpha=0.3)
ax2.plot(df[time_col], df['cmd_omega'], 'r-', linewidth=1.5)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('ω (rad/s)')
ax2.grid(True, alpha=0.3)
plt.tight_layout()
control_file = os.path.join(output_dir, 'control.png')
plt.savefig(control_file, dpi=300)
plt.close()
print(f"✓ Saved: {control_file}")

# Save metrics
with open(os.path.join(output_dir, 'metrics.txt'), 'w') as f:
    f.write(f'RMSE: {rmse:.3f} m\n')
    f.write(f'Max Error: {max_err:.3f} m\n')
    f.write(f'Mean Error: {mean_err:.3f} m\n')
    f.write(f'Duration: {duration:.1f} s\n')
    f.write(f'Samples: {len(df)}\n')

print(f"\n✅ DONE! All files in: {output_dir}")



