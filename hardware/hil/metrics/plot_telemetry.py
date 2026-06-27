#!/usr/bin/env python3
"""Plot comparison graphs of unstable vs stable controller runs."""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load telemetry
df = pd.read_csv('/home/dhruv/amcaf/hardware/hil/metrics/time_series_log.csv')

# Split into unstable and stable based on steering_kp value
unstable_df = df[df['steering_kp'] > 2.0].copy()
stable_df = df[df['steering_kp'] <= 2.0].copy()

# Reset time to relative seconds
if not unstable_df.empty:
    unstable_df['time'] = unstable_df['time'] - unstable_df['time'].iloc[0]
if not stable_df.empty:
    stable_df['time'] = stable_df['time'] - stable_df['time'].iloc[0]

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=False)

# Plot CTE Comparison
if not unstable_df.empty:
    ax1.plot(unstable_df['time'], unstable_df['cte'], 'r-', label='Unstable (Kp=2.44, clf_alpha=0.67)')
if not stable_df.empty:
    ax1.plot(stable_df['time'], stable_df['cte'], 'g-', label='Stable (Kp=1.4, clf_alpha=0.10)')
ax1.set_title('Lateral Tracking Performance (Cross-Track Error)', fontsize=14, fontweight='bold')
ax1.set_ylabel('CTE (pixels)', fontsize=12)
ax1.grid(True, linestyle='--', alpha=0.6)
ax1.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='none')

# Plot Steering Command Comparison
if not unstable_df.empty:
    ax2.plot(unstable_df['time'], unstable_df['steering'], 'r--', label='Unstable Steering Command')
if not stable_df.empty:
    ax2.plot(stable_df['time'], stable_df['steering'], 'g--', label='Stable Steering Command')
ax2.set_title('Steering Input Saturated/Chattering vs Smooth Control', fontsize=14, fontweight='bold')
ax2.set_xlabel('Relative Time (seconds)', fontsize=12)
ax2.set_ylabel('Steering Angle (rad)', fontsize=12)
ax2.grid(True, linestyle='--', alpha=0.6)
ax2.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='none')

plt.tight_layout()

# Save plot to artifacts folder
output_path = '/home/dhruv/.gemini/antigravity-cli/brain/c98afdd5-c0c6-4628-b2fe-ac6a8d66cbd6/vibration_comparison.png'
plt.savefig(output_path, dpi=150, bbox_inches='tight')
print(f"Comparison plot saved successfully to: {output_path}")
