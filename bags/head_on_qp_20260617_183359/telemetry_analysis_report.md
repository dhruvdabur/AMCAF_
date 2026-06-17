# HIL Telemetry Analysis Report
**Source File:** `pivoted_messages.csv`  
**Duration:** 12.90 seconds | **Samples:** 3524

## Key Performance Indicators (KPIs)

| Metric | Value | Description |
| :--- | :--- | :--- |
| **Mean Abs Lateral Error (CTE)** | 14.05 px | Average lateral offset from lane center |
| **RMSE Lateral Error** | 17.51 px | Root mean square error of lateral tracking |
| **Max Abs Lateral Error** | 57.04 px | Maximum deviation from lane center |
| **Mean Abs Heading Error** | 178.10° | Average alignment angle error |
| **Mean Track Speed** | 5.88 px/s | Average actual speed of vehicle progress |
| **Target Speed** | 0.00 px/s | Commanded target track speed |
| **Speed Tracking RMSE** | 8.32 px/s | Speed tracking error root-mean-square |
| **Steering Effort (Roll)** | 139.13 PWM | Average absolute deviation from center (1500) |
| **Steering Std Dev** | 182.85 PWM | Steering control input standard deviation |
| **Throttle Effort** | 49.33 PWM | Average absolute throttle above neutral (1500) |
| **Minimum CBF h Value** | -1.83 | Closest distance to the active safety barrier |
| **Near-collision Samples (h <= 0.1)** | 2649 | Time spent in near-collision state |
| **Collision Samples (h < 0.0)** | 1600 | Time spent in violated safety state |

## Safety & Control Assessment

> [!WARNING]
> **Safety Violation Detected!** The safety barrier function `h` was violated (dropped below 0.0) in 1600 samples. This indicates a simulated collision occurred.