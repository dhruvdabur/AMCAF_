#!/usr/bin/env python3
import sys
from pathlib import Path

try:
    import pandas as pd
    import matplotlib.pyplot as plt
except ImportError:
    print("Error: pandas and matplotlib are required to run this script.")
    print("Please run it using a python interpreter that has them installed (e.g. /usr/bin/python3)")
    sys.exit(1)

def generate_analysis_graphs(csv_file_path):
    csv_file_path = Path(csv_file_path).resolve()
    print(f"Loading pivoted telemetry data from: {csv_file_path}")
    
    # Load the data
    df = pd.read_csv(csv_file_path)
    
    # Handle timestamp and time column mapping
    time_col = 'time_s'
    if 'time_s' not in df.columns:
        ts_col = 'timestamp' if 'timestamp' in df.columns else 'timestamp_ns'
        if ts_col in df.columns:
            df['time_s'] = (df[ts_col] - df[ts_col].iloc[0]) / 1e9
        else:
            print("Error: Could not find timestamp or time column in CSV.")
            sys.exit(1)

    # Resolve column names (supporting both prefixed and stripped names)
    def resolve_col(standard_name, prefixed_name):
        if prefixed_name in df.columns:
            return prefixed_name
        if standard_name in df.columns:
            return standard_name
        # Suffix matching
        suffix = prefixed_name.split('/')[-1]
        if suffix in df.columns:
            return suffix
        return standard_name

    rhs_col = resolve_col('cbf_qp_rhs', '/dynamic_straight/tuning/cbf_qp_rhs')
    throttle_col = resolve_col('throttle_pwm', '/dynamic_straight/tuning/throttle_pwm')
    target_col = resolve_col('target_speed_pps', '/dynamic_straight/tuning/target_speed_pps')
    h_col = resolve_col('cbf_qp_h', '/dynamic_straight/tuning/cbf_qp_h')

    # Initialize a figure with three subplots stacked vertically
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle('CBF-QP Telemetry Analysis Report', fontsize=16)

    # --- Plot 1: The Danger Trigger (RHS) ---
    if rhs_col in df.columns:
        ax1.plot(df['time_s'].to_numpy(), df[rhs_col].to_numpy(), 
                 color='red', linewidth=2, label='CBF RHS')
    ax1.axhline(0, color='black', linestyle='--', alpha=0.5)
    ax1.set_ylabel('RHS Value\n(Positive = Danger)')
    ax1.legend(loc='upper left')
    ax1.grid(True, alpha=0.3)

    # --- Plot 2: The Physical Response ---
    if throttle_col in df.columns:
        ax2.plot(df['time_s'].to_numpy(), df[throttle_col].to_numpy(), 
                 color='blue', linewidth=2, label='Actual Throttle PWM')
    ax2.set_ylabel('Motor Command')
    ax2.legend(loc='upper left')
    ax2.grid(True, alpha=0.3)

    # --- Plot 3: The Ellipse Boundary (h) ---
    if h_col in df.columns:
        ax3.plot(df['time_s'].to_numpy(), df[h_col].to_numpy(), 
                 color='purple', linewidth=2, label='Ellipse Boundary (h)')
    ax3.axhline(0, color='black', linestyle='--', alpha=0.5, label='Collision Zone')
    ax3.set_ylabel('Distance (h)')
    ax3.set_xlabel('Time (seconds)')
    ax3.legend(loc='upper left')
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    
    # Save the plot image
    out_img_path = csv_file_path.parent / "cbf_qp_analysis.png"
    plt.savefig(out_img_path, dpi=300)
    print(f"Graph successfully saved to: {out_img_path}")
    
    # Attempt to show the plot if Display is available
    try:
        plt.show()
    except Exception as e:
        print(f"Could not open graphical display: {e}")

if __name__ == '__main__':
    # Accept CLI path to pivoted CSV
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        path = '/home/dhruv/amcaf/bags/head_on_qp_20260617_183359/pivoted_messages.csv'
    generate_analysis_graphs(path)
