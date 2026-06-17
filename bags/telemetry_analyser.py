#!/usr/bin/env python3
import sys
import os
import struct
from pathlib import Path

# Try to import pandas and numpy. If they fail, print warning.
try:
    import pandas as pd
    import numpy as np
except ImportError:
    print("Error: pandas and numpy are required to run this script.")
    print("Please run it using a python interpreter that has them installed (e.g. /usr/bin/python3)")
    sys.exit(1)

def analyze_bag_csv(pivoted_csv_path, output_md_path):
    print(f"Reading decoded pivoted telemetry: {pivoted_csv_path}")
    df = pd.read_csv(pivoted_csv_path)
    
    # Create dictionary for metrics
    metrics = {}
    
    if 'time_s' in df.columns:
        metrics['duration_s'] = df['time_s'].iloc[-1] - df['time_s'].iloc[0]
    else:
        metrics['duration_s'] = (df['timestamp_ns'].iloc[-1] - df['timestamp_ns'].iloc[0]) / 1e9
        
    metrics['samples'] = len(df)
    
    if 'lateral_error_px' in df.columns:
        ctes = df['lateral_error_px'].dropna()
        metrics['mean_abs_cte_px'] = ctes.abs().mean()
        metrics['rmse_cte_px'] = np.sqrt((ctes ** 2).mean())
        metrics['max_abs_cte_px'] = ctes.abs().max()
        
    if 'heading_error_rad' in df.columns:
        head_errs = df['heading_error_rad'].dropna()
        metrics['mean_abs_heading_error_deg'] = np.degrees(head_errs.abs().mean())
        
    if 'track_speed_pps' in df.columns and 'target_speed_pps' in df.columns:
        speeds = df['track_speed_pps'].dropna()
        targets = df['target_speed_pps'].dropna()
        # align them
        err = (speeds - targets).abs()
        metrics['mean_abs_speed_error_pps'] = err.mean()
        metrics['rmse_speed_error_pps'] = np.sqrt(((speeds - targets) ** 2).mean())
        metrics['mean_speed_pps'] = speeds.mean()
        metrics['target_speed_pps'] = targets.mean()
        
    if 'roll_pwm' in df.columns:
        rolls = df['roll_pwm'].dropna()
        metrics['mean_roll_pwm'] = rolls.mean()
        metrics['std_roll_pwm'] = rolls.std()
        metrics['steering_effort'] = (rolls - 1500).abs().mean()
        
    if 'throttle_pwm' in df.columns:
        throttles = df['throttle_pwm'].dropna()
        metrics['mean_throttle_pwm'] = throttles.mean()
        metrics['std_throttle_pwm'] = throttles.std()
        metrics['throttle_effort'] = (throttles - 1500).abs().mean()
        
    if 'cbf_qp_h' in df.columns:
        h_vals = df['cbf_qp_h'].dropna()
        metrics['min_cbf_h'] = h_vals.min()
        metrics['mean_cbf_h'] = h_vals.mean()
        metrics['near_collision_samples'] = (h_vals <= 0.1).sum()
        metrics['collision_samples'] = (h_vals < 0.0).sum()
        
    if 'pose_position_error_px' in df.columns:
        pose_pos_errs = df['pose_position_error_px'].dropna()
        metrics['mean_pose_pos_err'] = pose_pos_errs.mean()
        metrics['rmse_pose_pos_err'] = np.sqrt((pose_pos_errs ** 2).mean())
        metrics['max_pose_pos_err'] = pose_pos_errs.max()
        
    if 'pose_heading_error_rad' in df.columns:
        pose_head_errs = df['pose_heading_error_rad'].dropna()
        metrics['mean_pose_head_err_deg'] = np.degrees(pose_head_errs.mean())
        
    if 'pose_update_frequency_hz' in df.columns:
        pose_freqs = df['pose_update_frequency_hz'].dropna()
        metrics['mean_pose_freq'] = pose_freqs.mean()
        
    if 'control_smoothness' in df.columns:
        smooth = df['control_smoothness'].dropna()
        metrics['mean_control_smoothness'] = smooth.mean()
        
    if 'actuator_steer_saturated' in df.columns:
        metrics['steer_sat_ratio'] = df['actuator_steer_saturated'].dropna().mean() * 100.0
        
    if 'actuator_throttle_saturated' in df.columns:
        metrics['throttle_sat_ratio'] = df['actuator_throttle_saturated'].dropna().mean() * 100.0
        
    if 'safety_violated' in df.columns:
        metrics['safety_violation_ratio'] = df['safety_violated'].dropna().mean() * 100.0
        
    if 'safety_override_magnitude' in df.columns:
        metrics['mean_safety_override'] = df['safety_override_magnitude'].dropna().mean()
        
    if 'ftg_solve_time_ms' in df.columns:
        solves = df['ftg_solve_time_ms'].dropna()
        metrics['mean_ftg_solve_ms'] = solves.mean()
        
    if 'ftg_no_gap_found' in df.columns:
        metrics['no_gap_ratio'] = df['ftg_no_gap_found'].dropna().mean() * 100.0
        
    if 'ftg_target_angle_rate_rad_s' in df.columns:
        metrics['mean_target_angle_rate'] = df['ftg_target_angle_rate_rad_s'].dropna().mean()
        
    # Generate the report content
    report = []
    report.append("# HIL Telemetry Analysis Report")
    report.append(f"**Source File:** `{pivoted_csv_path.name}`  ")
    report.append(f"**Duration:** {metrics.get('duration_s', 0.0):.2f} seconds | **Samples:** {metrics.get('samples', 0)}")
    report.append("\n## Key Performance Indicators (KPIs)\n")
    
    # Format table of metrics
    table = []
    table.append("| Metric | Value | Description |")
    table.append("| :--- | :--- | :--- |")
    
    if 'mean_abs_cte_px' in metrics:
        table.append(f"| **Mean Abs Lateral Error (CTE)** | {metrics['mean_abs_cte_px']:.2f} px | Average lateral offset from lane center |")
        table.append(f"| **RMSE Lateral Error** | {metrics['rmse_cte_px']:.2f} px | Root mean square error of lateral tracking |")
        table.append(f"| **Max Abs Lateral Error** | {metrics['max_abs_cte_px']:.2f} px | Maximum deviation from lane center |")
        
    if 'mean_abs_heading_error_deg' in metrics:
        table.append(f"| **Mean Abs Heading Error** | {metrics['mean_abs_heading_error_deg']:.2f}° | Average alignment angle error |")
        
    if 'mean_speed_pps' in metrics:
        table.append(f"| **Mean Track Speed** | {metrics['mean_speed_pps']:.2f} px/s | Average actual speed of vehicle progress |")
        table.append(f"| **Target Speed** | {metrics['target_speed_pps']:.2f} px/s | Commanded target track speed |")
        table.append(f"| **Speed Tracking RMSE** | {metrics['rmse_speed_error_pps']:.2f} px/s | Speed tracking error root-mean-square |")
        
    if 'steering_effort' in metrics:
        table.append(f"| **Steering Effort (Roll)** | {metrics['steering_effort']:.2f} PWM | Average absolute deviation from center (1500) |")
        table.append(f"| **Steering Std Dev** | {metrics['std_roll_pwm']:.2f} PWM | Steering control input standard deviation |")
        
    if 'throttle_effort' in metrics:
        table.append(f"| **Throttle Effort** | {metrics['throttle_effort']:.2f} PWM | Average absolute throttle above neutral (1500) |")
        
    if 'min_cbf_h' in metrics:
        table.append(f"| **Minimum CBF h Value** | {metrics['min_cbf_h']:.2f} | Closest distance to the active safety barrier |")
        table.append(f"| **Near-collision Samples (h <= 0.1)** | {metrics['near_collision_samples']} | Time spent in near-collision state |")
        table.append(f"| **Collision Samples (h < 0.0)** | {metrics['collision_samples']} | Time spent in violated safety state |")
        
    if 'mean_pose_pos_err' in metrics:
        table.append(f"| **ArUco Position Error MAE** | {metrics['mean_pose_pos_err']:.2f} px | Mean absolute error of ArUco pose vs. Ground Truth |")
        table.append(f"| **ArUco Position Error RMSE** | {metrics['rmse_pose_pos_err']:.2f} px | Root mean square error of ArUco pose vs. Ground Truth |")
        table.append(f"| **ArUco Heading Error MAE** | {metrics['mean_pose_head_err_deg']:.2f}° | Mean alignment error between estimated and actual heading |")
        table.append(f"| **ArUco Update Frequency** | {metrics['mean_pose_freq']:.1f} Hz | Mean frame processing rate |")
        
    if 'mean_control_smoothness' in metrics:
        table.append(f"| **Control Path Smoothness** | {metrics['mean_control_smoothness']:.3f} px/frame | Average frame-to-frame change rate of CTE |")
        
    if 'steer_sat_ratio' in metrics:
        table.append(f"| **Steer Saturation Ratio** | {metrics['steer_sat_ratio']:.1f}% | Percentage of time steering was at saturation limits |")
        table.append(f"| **Throttle Saturation Ratio** | {metrics['throttle_sat_ratio']:.1f}% | Percentage of time throttle was at limits or neutral |")
        
    if 'safety_violation_ratio' in metrics:
        table.append(f"| **CBF Safety Violation Ratio** | {metrics['safety_violation_ratio']:.1f}% | Percentage of samples breaching safety boundary ($h < 0$) |")
        table.append(f"| **Mean Safety Override Magnitude** | {metrics['mean_safety_override']:.1f} PWM | Average throttle reduction commanded by CBF QP |")
        
    if 'mean_ftg_solve_ms' in metrics:
        table.append(f"| **FTG Solve Latency** | {metrics['mean_ftg_solve_ms']:.2f} ms | Average time taken to calculate gaps and target |")
        table.append(f"| **FTG No Gap Found Ratio** | {metrics['no_gap_ratio']:.1f}% | Percentage of time no suitable driving gap was found |")
        table.append(f"| **FTG Target Angle Rate** | {metrics['mean_target_angle_rate']:.3f} rad/s | Rate of change of selected gap target direction |")
        
    report.extend(table)
    
    # Safety assessment
    report.append("\n## Safety & Control Assessment\n")
    if metrics.get('collision_samples', 0) > 0:
        report.append("> [!WARNING]\n> **Safety Violation Detected!** The safety barrier function `h` was violated (dropped below 0.0) in "
                      f"{metrics['collision_samples']} samples. This indicates a simulated collision occurred.")
    elif metrics.get('near_collision_samples', 0) > 0:
        report.append("> [!IMPORTANT]\n> **Safety Maintained but Tight Clearance:** The vehicle successfully avoided collision, but spent "
                      f"{metrics['near_collision_samples']} samples very close to the obstacle boundary (h <= 0.1).")
    else:
        report.append("> [!NOTE]\n> **Safety Successfully Maintained:** The vehicle navigated the scenario safely. The safety barrier function `h` remained "
                      f"above 0.1 at all times (min h: {metrics.get('min_cbf_h', 0.0):.2f}).")
                      
    report_text = "\n".join(report)
    print("\n" + report_text + "\n")
    
    with open(output_md_path, 'w') as f:
        f.write(report_text)
    print(f"Saved report to: {output_md_path}")

def main():
    if len(sys.argv) < 2:
        bag_dir = Path("/home/dhruv/amcaf/bags/head_on_qp_20260617_183359")
    else:
        bag_dir = Path(sys.argv[1]).resolve()
        
    pivoted_csv = bag_dir / "pivoted_messages.csv"
    if not pivoted_csv.exists():
        print(f"Decoded data not found at {pivoted_csv}. Running decode_bag.py first...")
        script_dir = Path(__file__).parent
        decode_script = script_dir / "decode_bag.py"
        if not decode_script.exists():
            decode_script = Path("/home/dhruv/amcaf/bags/decode_bag.py")
        ret = os.system(f"/usr/bin/python3 '{decode_script}' '{bag_dir}'")
        if ret != 0:
            print("Error running decode_bag.py")
            sys.exit(1)
        
    if not pivoted_csv.exists():
        print(f"Failed to find or generate pivoted_messages.csv in {bag_dir}")
        sys.exit(1)
        
    output_report = bag_dir / "telemetry_analysis_report.md"
    analyze_bag_csv(pivoted_csv, output_report)

if __name__ == "__main__":
    main()
