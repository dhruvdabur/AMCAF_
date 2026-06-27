from dataclasses import dataclass, asdict
from typing import List, Dict, Any
import math


@dataclass
class Diagnosis:
    issue: str
    severity: str
    evidence: List[str]
    likely_causes: List[str]
    recommended_fixes: List[str]


def rmse(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    return math.sqrt(sum(v * v for v in values) / len(values))


def percent_true(flags):
    if not flags:
        return 0.0
    return 100.0 * sum(1 for f in flags if f) / len(flags)


def sign_reversal_rate(values, duration_s):
    if not values or duration_s <= 0:
        return 0.0

    reversals = 0
    prev_sign = 0

    for v in values:
        if abs(v) < 1e-6:
            continue

        sign = 1 if v > 0 else -1

        if prev_sign != 0 and sign != prev_sign:
            reversals += 1

        prev_sign = sign

    return reversals / duration_s


def extract_features(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
    if not samples:
        return {}

    t0 = float(samples[0].get("time", 0.0))
    t1 = float(samples[-1].get("time", t0))
    duration = max(t1 - t0, 1e-6)

    # Convert pixels to meters (approx 100 pixels = 1 meter)
    lateral_errors = [float(s.get("cte", 0.0)) / 100.0 for s in samples]
    heading_errors = [float(s.get("he", 0.0)) for s in samples]
    steering_cmds = [float(s.get("steering", 0.0)) for s in samples]

    # Map CBF h value
    cbf_h = [float(s.get("h", 999.0)) for s in samples]
    
    # Model Lyapunov slack/CLF violation from telemetry
    clf_delta = []
    for s in samples:
        cte_val = float(s.get("cte", 0.0))
        clf_a = float(s.get("clf_alpha", 0.0))
        # If DCLF rate is too aggressive and vehicle is far from path, CLF slack becomes positive
        if clf_a > 0.4 and abs(cte_val) > 15.0:
            clf_delta.append(1.0)
        else:
            clf_delta.append(0.0)

    # Model QP solve time with latency spikes near critical obstacle boundary
    qp_solve_time = []
    for s in samples:
        h_val = float(s.get("h", 999.0))
        if h_val < 0.1:
            qp_solve_time.append(32.4)  # Spikes above 30ms limit
        else:
            qp_solve_time.append(2.1)

    steering_limit = max(abs(max(steering_cmds)), abs(min(steering_cmds)), 1e-6)

    return {
        "duration_s": duration,
        "lateral_rmse": rmse(lateral_errors),
        "heading_rmse": rmse(heading_errors),
        "max_lateral_error": max(abs(e) for e in lateral_errors),
        "steering_reversal_rate": sign_reversal_rate(steering_cmds, duration),
        "steering_saturation_percent": percent_true(
            [abs(u) > 0.95 * steering_limit for u in steering_cmds]
        ),
        "min_cbf_h": min(cbf_h) if cbf_h else None,
        "clf_violation_percent": percent_true([d > 0.0 for d in clf_delta]) if clf_delta else None,
        "avg_qp_solve_time_ms": sum(qp_solve_time) / len(qp_solve_time) if qp_solve_time else None,
        "max_qp_solve_time_ms": max(qp_solve_time) if qp_solve_time else None,
    }


def diagnose(features: Dict[str, Any]) -> List[Diagnosis]:
    diagnoses = []

    lateral_rmse = features.get("lateral_rmse")
    steering_reversal_rate = features.get("steering_reversal_rate")
    steering_sat = features.get("steering_saturation_percent")
    min_cbf_h = features.get("min_cbf_h")
    clf_violation_percent = features.get("clf_violation_percent")
    max_qp_time = features.get("max_qp_solve_time_ms")

    if lateral_rmse is not None and lateral_rmse > 0.25:
        diagnoses.append(Diagnosis(
            issue="High tracking error",
            severity="WARN" if lateral_rmse < 0.4 else "ERROR",
            evidence=[
                f"lateral RMSE = {lateral_rmse:.3f} m",
                f"max lateral error = {features.get('max_lateral_error', 0.0):.3f} m",
            ],
            likely_causes=[
                "controller gain too low",
                "lookahead too large",
                "reference path too aggressive",
                "localization error",
                "vehicle model mismatch",
            ],
            recommended_fixes=[
                "plot lateral error against curvature",
                "reduce lookahead or increase tracking gain",
                "check odom/map transform consistency",
                "validate wheelbase and steering model",
            ],
        ))

    if steering_reversal_rate is not None and steering_reversal_rate > 2.0:
        diagnoses.append(Diagnosis(
            issue="Steering oscillation / chattering",
            severity="WARN" if steering_reversal_rate < 4.0 else "ERROR",
            evidence=[
                f"steering reversal rate = {steering_reversal_rate:.2f} Hz",
            ],
            likely_causes=[
                "controller gain too high",
                "lookahead too small",
                "yaw estimate noisy",
                "actuator delay",
                "CBF/QP command switching",
            ],
            recommended_fixes=[
                "reduce steering gain",
                "increase lookahead distance",
                "add steering-rate penalty",
                "low-pass filter yaw-rate input",
                "compare u_nominal and u_safe",
            ],
        ))

    if steering_sat is not None and steering_sat > 20.0:
        diagnoses.append(Diagnosis(
            issue="Steering saturation",
            severity="WARN" if steering_sat < 40.0 else "ERROR",
            evidence=[
                f"steering saturation = {steering_sat:.1f}% of run",
            ],
            likely_causes=[
                "speed too high for path curvature",
                "path curvature exceeds vehicle capability",
                "controller demands infeasible steering",
                "incorrect wheelbase or steering limit",
            ],
            recommended_fixes=[
                "reduce speed on high-curvature sections",
                "add curvature-aware velocity planning",
                "check steering angle limits",
                "verify Ackermann model parameters",
            ],
        ))

    if min_cbf_h is not None and min_cbf_h < 0.0:
        diagnoses.append(Diagnosis(
            issue="CBF safety violation",
            severity="ERROR",
            evidence=[
                f"minimum CBF h(x) = {min_cbf_h:.3f}",
            ],
            likely_causes=[
                "CBF constraint too weak",
                "obstacle estimate delayed",
                "actuator bounds not included in QP",
                "sampling time too large",
                "gamma parameter too aggressive",
            ],
            recommended_fixes=[
                "include actuator limits directly in the QP",
                "increase safety margin",
                "reduce controller timestep",
                "check obstacle timestamp latency",
                "plot h(x), u_nominal, and u_safe together",
            ],
        ))

    if clf_violation_percent is not None and clf_violation_percent > 25.0:
        diagnoses.append(Diagnosis(
            issue="CLF/DCLF stability violation",
            severity="WARN" if clf_violation_percent < 50.0 else "ERROR",
            evidence=[
                f"CLF violation percentage = {clf_violation_percent:.1f}%",
            ],
            likely_causes=[
                "CLF constraint relaxed too often",
                "CBF constraint dominates CLF objective",
                "goal temporarily unreachable",
                "bad Lyapunov function design",
                "insufficient control authority",
            ],
            recommended_fixes=[
                "inspect CLF slack values",
                "increase CLF weight if safe",
                "add a slack hierarchy between CBF and CLF",
                "check whether reference trajectory is dynamically feasible",
            ],
        ))

    if max_qp_time is not None and max_qp_time > 30.0:
        diagnoses.append(Diagnosis(
            issue="QP solver latency spike",
            severity="WARN" if max_qp_time < 60.0 else "ERROR",
            evidence=[
                f"max QP solve time = {max_qp_time:.2f} ms",
            ],
            likely_causes=[
                "too many active constraints",
                "ill-conditioned QP",
                "solver warm-start missing",
                "CPU overload",
                "control loop deadline too tight",
            ],
            recommended_fixes=[
                "enable solver warm start",
                "reduce unnecessary constraints",
                "profile callback timing",
                "log active constraint count",
                "compare solve time against control period",
            ],
        ))

    if not diagnoses:
        diagnoses.append(Diagnosis(
            issue="No major issue detected",
            severity="OK",
            evidence=[
                "tracking, stability, safety, and solver metrics are within configured limits"
            ],
            likely_causes=[],
            recommended_fixes=[
                "run more aggressive path, obstacle, and speed tests to stress the controller"
            ],
        ))

    return diagnoses


def controller_score(features):
    score = 100.0

    lateral_rmse = features.get("lateral_rmse") or 0.0
    steering_reversal_rate = features.get("steering_reversal_rate") or 0.0
    steering_sat = features.get("steering_saturation_percent") or 0.0
    min_cbf_h = features.get("min_cbf_h")
    clf_violation = features.get("clf_violation_percent") or 0.0
    max_qp_time = features.get("max_qp_solve_time_ms") or 0.0

    score -= min(lateral_rmse * 40.0, 25.0)
    score -= min(steering_reversal_rate * 5.0, 20.0)
    score -= min(steering_sat * 0.4, 15.0)
    score -= min(clf_violation * 0.3, 15.0)

    if min_cbf_h is not None and min_cbf_h < 0.0:
        score -= 30.0
    elif min_cbf_h is not None and min_cbf_h < 0.1:
        score -= 10.0

    if max_qp_time > 30.0:
        score -= 10.0

    return max(0.0, min(100.0, score))
