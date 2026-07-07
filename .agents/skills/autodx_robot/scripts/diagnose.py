#!/usr/bin/env python3
"""AutoDx-Robot: Telemetry Diagnosis Rule Engine script."""

import os
import sys
import csv

from pathlib import Path

# Add observability folder to Python path to load autodiag_engine
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../hardware/hil/observability')))

import autodiag_engine

REPO_ROOT = Path(__file__).resolve().parents[4]
CSV_PATH = str(REPO_ROOT / 'hardware' / 'hil' / 'metrics' / 'time_series_log.csv')
REPORT_PATH = str(REPO_ROOT / 'hardware' / 'hil' / 'metrics' / 'diagnosis_report.md')

def load_telemetry():
    if not os.path.exists(CSV_PATH):
        print(f"Error: Telemetry log not found at {CSV_PATH}. Please run a simulator test first.")
        sys.exit(1)

    data = []
    with open(CSV_PATH, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append({
                "time": float(row["time"]),
                "cte": float(row["cte"]),
                "he": float(row["he"]),
                "steering": float(row["steering"]),
                "h": float(row["h"]),
                "clf_alpha": float(row["clf_alpha"]),
                "steering_kp": float(row["steering_kp"])
            })
    return data

def main():
    samples = load_telemetry()
    features = autodiag_engine.extract_features(samples)
    diagnoses = autodiag_engine.diagnose(features)
    score = autodiag_engine.controller_score(features)

    # Generate Markdown Report
    report = []
    report.append("# Robot Controller Auto-Diagnosis Report")
    report.append(f"**Trial Duration**: {features.get('duration_s', 0.0):.2f} s | **Total Samples**: {len(samples)}")
    report.append(f"**Controller Score**: {score:.1f} / 100\n")

    report.append("## Telemetry Evidence Summary")
    report.append(f"* **Lateral Tracking RMSE**: {features.get('lateral_rmse', 0.0)*100:.2f} px")
    report.append(f"* **Max Lateral Deviation**: {features.get('max_lateral_error', 0.0)*100:.2f} px")
    report.append(f"* **Heading Tracking RMSE**: {math.degrees(features.get('heading_rmse', 0.0)) if features.get('heading_rmse') else 0.0:.2f} deg")
    report.append(f"* **Minimum Safety Margin h(x)**: {features.get('min_cbf_h', 0.0):.3f}")
    report.append(f"* **Steering Saturation**: {features.get('steering_saturation_percent', 0.0):.1f}%")
    report.append(f"* **Steering Reversal Frequency**: {features.get('steering_reversal_rate', 0.0):.2f} Hz\n")

    report.append("## Diagnostic Findings")
    for idx, diag in enumerate(diagnoses, 1):
        severity_class = "🔴 ERROR" if diag.severity == "ERROR" else ("🟡 WARN" if diag.severity == "WARN" else "🟢 OK")
        report.append(f"### {idx}. {diag.issue} ({severity_class})")
        report.append("**Evidence**:")
        for ev in diag.evidence:
            report.append(f"- {ev}")
        report.append("**Likely Causes**:")
        for c in diag.likely_causes:
            report.append(f"- {c}")
        report.append("**Recommended Fixes**:")
        for f in diag.recommended_fixes:
            report.append(f"- {f}")
        report.append("")

    report_content = "\n".join(report)
    
    # Save report
    with open(REPORT_PATH, 'w') as f:
        f.write(report_content)

    print(report_content)

if __name__ == '__main__':
    # Import math inside script to support degrees conversion
    import math
    main()
