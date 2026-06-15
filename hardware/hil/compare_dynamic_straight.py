#!/usr/bin/env python3
"""Compare metrics from multiple dynamic_straight HIL runs."""

import json
import sys
from pathlib import Path


def load_metrics(path):
    """Load and return the summary dictionary from a metrics JSON file."""
    try:
        with open(path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except (json.JSONDecodeError, OSError) as e:
        print(f"Error loading {path}: {e}")
        return None


def format_table(headers, rows):
    """Print a simple ASCII table."""
    # Find max width for each column
    widths = [len(h) for h in headers]
    for row in rows:
        for i, val in enumerate(row):
            widths[i] = max(widths[i], len(str(val)))
    
    # Print headers
    header_str = " | ".join(h.ljust(widths[i]) for i, h in enumerate(headers))
    print(header_str)
    print("-" * len(header_str))
    
    # Print rows
    for row in rows:
        print(" | ".join(str(val).ljust(widths[i]) for i, val in enumerate(row)))


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 compare_dynamic_straight.py <metrics_file1> <metrics_file2> ...")
        print("Or: python3 compare_dynamic_straight.py *.json")
        return

    files = [Path(f) for f in sys.argv[1:] if Path(f).is_file()]
    if not files:
        print("No valid metrics files provided.")
        return

    headers = [
        "Controller",
        "RMSE CTE (px)",
        "Max CTE (px)",
        "Speed Err (pps)",
        "Effort (PWM*s)",
        "Collisions",
        "CBF Acts"
    ]
    
    rows = []
    for f in sorted(files):
        data = load_metrics(f)
        if not data:
            continue
            
        # Handle cases where some keys might be missing in older files
        mode = data.get('controller_mode', f.stem)
        rmse_cte = f"{data.get('rmse_cte_px', 0.0):.2f}"
        max_cte = f"{data.get('max_abs_cte_px', 0.0):.2f}"
        speed_err = f"{data.get('mean_abs_speed_error_pps', 0.0):.2f}"
        
        # Effort can be steering + throttle
        steer_effort = data.get('steering_effort_pwm_s', 0.0)
        throttle_effort = data.get('throttle_effort_pwm_s', 0.0)
        effort = f"{steer_effort + throttle_effort:.0f}"
        
        collisions = data.get('collision_samples', 0)
        cbf_acts = data.get('cbf_interventions', 0)
        
        rows.append([mode, rmse_cte, max_cte, speed_err, effort, collisions, cbf_acts])

    print(f"\nComparing {len(rows)} runs:\n")
    format_table(headers, rows)
    print("")


if __name__ == "__main__":
    main()
