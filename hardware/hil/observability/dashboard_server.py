#!/usr/bin/env python3
"""Lightweight HTTP server serving the Robot Observability Dashboard API."""

import os
import json
import math
import csv
from dataclasses import asdict
from http.server import SimpleHTTPRequestHandler, HTTPServer
from pathlib import Path
import autodiag_engine

PORT = 8050
METRICS_DIR = Path(__file__).resolve().parent.parent / 'metrics'
CSV_PATH = str(METRICS_DIR / 'time_series_log.csv')

class ObservabilityAPIHandler(SimpleHTTPRequestHandler):
    """Custom request handler that exposes JSON endpoints and serves static web UI files."""

    def do_GET(self):
        if self.path == '/api/telemetry':
            self.send_telemetry()
        elif self.path == '/api/summary':
            self.send_summary()
        elif self.path == '/api/diagnosis':
            self.send_diagnosis()
        else:
            # Serve index.html, style.css, app.js from this directory
            super().do_GET()

    def send_telemetry(self):
        """Read both stable and unstable telemetry CSVs and return them."""
        unstable_path = str(METRICS_DIR / 'telemetry_unstable.csv')
        stable_path = str(METRICS_DIR / 'telemetry_stable.csv')
        
        response = {
            "unstable": self.parse_csv_data(unstable_path),
            "stable": self.parse_csv_data(stable_path)
        }
        self.send_json_response(response)

    def parse_csv_data(self, path):
        if not os.path.exists(path):
            return []
        data = []
        try:
            with open(path, 'r') as f:
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
        except Exception:
            pass
        return data

    def get_latest_csv_path(self):
        unstable_path = str(METRICS_DIR / 'telemetry_unstable.csv')
        stable_path = str(METRICS_DIR / 'telemetry_stable.csv')
        
        t_unstable = os.path.getmtime(unstable_path) if os.path.exists(unstable_path) else 0.0
        t_stable = os.path.getmtime(stable_path) if os.path.exists(stable_path) else 0.0
        
        if t_stable > t_unstable:
            return stable_path
        elif t_unstable > 0.0:
            return unstable_path
        return None

    def send_summary(self):
        """Compute key tracking & safety metrics from the latest CSV data."""
        csv_path = self.get_latest_csv_path()
        if not csv_path:
            self.send_json_response({"error": "No telemetry logs found. Run a simulation trial first."}, 404)
            return

        try:
            ctes = []
            hes = []
            steerings = []
            hs = []
            steering_kps = []
            
            with open(csv_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    ctes.append(float(row["cte"]))
                    hes.append(float(row["he"]))
                    steerings.append(float(row["steering"]))
                    hs.append(float(row["h"]))
                    steering_kps.append(float(row["steering_kp"]))

            if not ctes:
                self.send_json_response({"error": "No telemetry data collected yet."}, 400)
                return

            # Compute stats
            abs_ctes = [abs(c) for c in ctes]
            mean_abs_cte = sum(abs_ctes) / len(abs_ctes)
            rmse_cte = math.sqrt(sum(c**2 for c in ctes) / len(ctes))
            max_abs_cte = max(abs_ctes)
            
            abs_hes_deg = [abs(math.degrees(h)) for h in hes]
            mean_abs_he_deg = sum(abs_hes_deg) / len(abs_hes_deg)
            
            min_safety_margin = min(hs)
            
            # Control effort / chattering metric: sum of squared differences of sequential steering inputs
            chattering = 0.0
            if len(steerings) > 1:
                chattering = sum((steerings[i] - steerings[i-1])**2 for i in range(1, len(steerings)))

            # Split runs count
            unstable_count = sum(1 for kp in steering_kps if kp > 2.0)
            stable_count = sum(1 for kp in steering_kps if kp <= 2.0)

            summary = {
                "total_samples": len(ctes),
                "mean_abs_cte_px": mean_abs_cte,
                "rmse_cte_px": rmse_cte,
                "max_abs_cte_px": max_abs_cte,
                "mean_abs_he_deg": mean_abs_he_deg,
                "min_safety_margin_h": min_safety_margin,
                "steering_chattering": chattering,
                "unstable_samples": unstable_count,
                "stable_samples": stable_count
            }
            self.send_json_response(summary)
        except Exception as e:
            self.send_json_response({"error": f"Failed to compute metrics: {str(e)}"}, 500)

    def send_diagnosis(self):
        """Run the autodiag_engine on the latest CSV telemetry and return findings."""
        csv_path = self.get_latest_csv_path()
        if not csv_path:
            self.send_json_response({"error": "No telemetry logs found."}, 404)
            return

        try:
            samples = []
            with open(csv_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    samples.append({
                        "time": float(row["time"]),
                        "cte": float(row["cte"]),
                        "he": float(row["he"]),
                        "steering": float(row["steering"]),
                        "h": float(row["h"]),
                        "clf_alpha": float(row["clf_alpha"]),
                        "steering_kp": float(row["steering_kp"])
                    })

            features = autodiag_engine.extract_features(samples)
            diagnoses = autodiag_engine.diagnose(features)
            score = autodiag_engine.controller_score(features)

            response = {
                "controller": "DCLF-DCBF",
                "run_id": "hil_trial",
                "features": features,
                "diagnoses": [asdict(d) for d in diagnoses],
                "score": score
            }
            self.send_json_response(response)
        except Exception as e:
            self.send_json_response({"error": f"Failed to compute diagnosis: {str(e)}"}, 500)

    def send_json_response(self, data, status_code=200):
        """Helper to send JSON response."""
        self.send_response(status_code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode('utf-8'))

def main():
    # Change working directory to where this file resides to serve index.html etc.
    server_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(server_dir)
    
    server = HTTPServer(('0.0.0.0', PORT), ObservabilityAPIHandler)
    print(f"Robot Observability Dashboard server running at http://localhost:{PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping dashboard server.")
        server.server_close()

if __name__ == '__main__':
    main()
