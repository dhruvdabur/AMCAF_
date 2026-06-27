---
name: Robot Controller Auto-Diagnosis
description: Automatically diagnose why a robot behaves badly when testing different controllers by reading telemetry, extracting metrics, detecting symptoms, and mapping them to likely root causes.
---

# Robot Controller Auto-Diagnosis Skill (`AutoDx-Robot`)

This skill allows the agent to automatically diagnose why a robot/vehicle is tracking poorly or violating safety boundaries by analyzing the local `time_series_log.csv` file.

## Diagnostic Usage

To run the telemetry diagnosis rules and generate a Markdown report, execute the diagnosis script:

```bash
python3 .agents/skills/autodx_robot/scripts/diagnose.py
```

This script will parse your active telemetry log, calculate the key features, detect symptoms, and output the report, saving it to:
`/home/dhruv/amcaf/hardware/hil/metrics/diagnosis_report.md`

## Observability Rules Enforced

The engine evaluates rule-based checks for:
1. **Violent Lateral Oscillation**: Detects high-frequency steering reversal rate (>2.0 Hz) coupled with large cross-track tracking error.
2. **Aggressive DCLF Rate**: Flags when `clf_alpha` is tuned too aggressively (>0.40), causing saturation chatter.
3. **Actuator Steering Saturation**: Flags when commands exceed Ackerman bounds (>15% of trial duration).
4. **Safety Constraint Violations**: Flags when the safety barrier values $h(x)$ cross below $0.0$.

## Performance Scorer

A summary controller performance score (out of 100) is computed on each run using the following metrics:
* **Tracking accuracy** (Max 30 pts)
* **Control stability** (Max 25 pts)
* **Safety clearance** (Max 30 pts)
* **Input smoothness** (Max 15 pts)
