#!/usr/bin/env python3
"""Script to plot MPC tracking metrics and lateral error from the logged telemetry CSV file."""

import sys
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

def main():
    csv_path = Path("/home/dhruv/amcaf/gazebo_sim/gazebo_worlds/mpc_telemetry.csv")
    output_plot = Path("/home/dhruv/amcaf/gazebo_sim/gazebo_worlds/mpc_performance_plot.png")

    if not csv_path.exists():
        print(f"Error: Telemetry file {csv_path} does not exist. Run the controller first to generate data!")
        sys.exit(1)

    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        sys.exit(1)

    if df.empty:
        print("Error: Telemetry CSV file is empty!")
        sys.exit(1)

    # Calculate time elapsed relative to start
    df['time_elapsed'] = df['timestamp'] - df['timestamp'].iloc[0]

    # Convert to numpy arrays to avoid matplotlib/pandas indexer incompatibility
    t = df['time_elapsed'].to_numpy()
    lat_err = df['lateral_error'].to_numpy()
    speed = df['speed'].to_numpy()
    target_speed = df['target_speed'].to_numpy()
    steer_cmd = df['steer_cmd'].to_numpy()

    # Create figure
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("MPC Controller Performance Analysis", fontsize=14, fontweight='bold')

    # 1. Lateral Error Plot
    axes[0].plot(t, lat_err, color='crimson', label='Lateral Error (m)', linewidth=1.5)
    axes[0].axhline(0, color='black', linestyle='--', alpha=0.6)
    axes[0].set_ylabel("Lateral Error (meters)", fontweight='bold')
    axes[0].grid(True, linestyle=':', alpha=0.6)
    axes[0].legend(loc='upper right')
    
    # Calculate performance metrics for display
    rmse = (lat_err ** 2).mean() ** 0.5
    max_error = np.abs(lat_err).max()
    axes[0].text(0.02, 0.08, f"RMSE: {rmse:.3f}m | Max Error: {max_error:.3f}m", 
                 transform=axes[0].transAxes, bbox=dict(facecolor='white', alpha=0.8, boxstyle='round,pad=0.3'))

    # 2. Speed Profile Plot
    axes[1].plot(t, speed, color='dodgerblue', label='Actual Speed (m/s)', linewidth=1.5)
    axes[1].plot(t, target_speed, color='darkorange', linestyle='--', label='Target Speed (m/s)', linewidth=1.5)
    axes[1].set_ylabel("Velocity (m/s)", fontweight='bold')
    axes[1].grid(True, linestyle=':', alpha=0.6)
    axes[1].legend(loc='lower right')

    # 3. Control Inputs (Steering) Plot
    axes[2].plot(t, steer_cmd * 57.2958, color='forestgreen', label='Steer Cmd (deg)', linewidth=1.5)
    axes[2].set_ylabel("Steering angle (deg)", fontweight='bold')
    axes[2].set_xlabel("Time Elapsed (seconds)", fontweight='bold')
    axes[2].grid(True, linestyle=':', alpha=0.6)
    axes[2].legend(loc='upper right')

    plt.tight_layout()
    
    try:
        plt.savefig(output_plot, dpi=300)
        print(f"Successfully generated performance plot and saved to: {output_plot}")
    except Exception as e:
        print(f"Error saving plot: {e}")

if __name__ == "__main__":
    main()
