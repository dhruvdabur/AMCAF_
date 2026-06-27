# Robot Observability & Controller Debugging Dashboard

This is a dedicated web-based observability tool designed to record, visualize, trace, and quantitatively compare robot controller telemetry (such as PID, DCLF-DCBF, MPC, etc.).

## Key Features

1. **Dual Trial Comparison**:
   * Automatically parses the time-series logs and splits the dataset into **unstable/violently oscillating runs** (where `steering_kp_px > 2.0`) and **stable runs** (`steering_kp_px <= 2.0`).
2. **Interactive Time-Series Signal Plots** (using Chart.js):
   * **Lateral Tracking Performance**: Plots Cross-Track Error (CTE) in pixels for both runs side-by-side.
   * **Control Commands & Safety Filter**: Plots the steering command inputs in radians alongside the Control Barrier Function values $h(x)$ to check if the safety filter is fighting the controller or if the inputs are saturated.
3. **Observability Metrics Dashboard**:
   * **CTE RMSE & Max Deviation**: Compares the root-mean-squared and peak lateral error.
   * **Min Safety Margin $h(x)$**: Identifies if the safety filter became active or if constraints were violated ($h(x) < 0$).
   * **Steering Chattering Index**: Measures control smoothness to diagnose actuator vibration.
4. **Diagnostic Recommendations**:
   * Offers real-time tuning suggestions based on telemetry ratios.

---

## Folder Contents

* [dashboard_server.py](file:///home/dhruv/amcaf/hardware/hil/observability/dashboard_server.py): The Python API server (built with no external dependencies).
* [index.html](file:///home/dhruv/amcaf/hardware/hil/observability/index.html): Semantic HTML5 front-end template.
* [style.css](file:///home/dhruv/amcaf/hardware/hil/observability/style.css): Premium modern dark-theme styles.
* [app.js](file:///home/dhruv/amcaf/hardware/hil/observability/app.js): Chart rendering and API fetch logic.

---

## How to Run

1. Run the Python backend server in your terminal:
   ```bash
   python3 hardware/hil/observability/dashboard_server.py
   ```
2. Open your web browser and navigate to:
   ```text
   http://localhost:8050
   ```
3. Click **Sync Telemetry** to pull the latest simulation run data!
