"""QP-based control barrier function safety filter with elliptical barriers."""

from dataclasses import dataclass
import time
from typing import Optional

import numpy as np


@dataclass
class Car:
    """Bicycle-model car state consumed by the elliptical CBF-QP."""

    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0
    v: float = 2.0
    prev_y_error: float = 0.0
    prev_psi_error: float = 0.0
    prev_v_error: float = 0.0
    v_cmd: float = 2.0


VehicleState = Car


@dataclass
class PointObstacle:
    """Obstacle center and optional dimensions consumed by the CBF-QP."""

    x: float
    y: float
    half_length: float = 0.0
    half_width: float = 0.0
    vx: float = 0.0
    vy: float = 0.0


@dataclass
class EllipseCBFQPConfig:
    """Tunable constants for the elliptical CBF-QP safety filter."""

    a_ell: float = 2.0
    b_ell: float = 1.0
    wheelbase: float = 2.0
    gamma1: float = 10.0
    gamma2: float = 1.0
    gamma3: float = 1.0
    min_accel: float = -10.0
    max_accel: float = 0.01 
    min_delta: float = -1.3
    max_delta: float = 1.3
    solver: str = 'OSQP'
    slack_weight: float = 2000.0


class EllipseCBFQPSafetyFilter:
    """Filter nominal commands through a CBF-QP with elliptical barriers."""

    def __init__(self, config: Optional[EllipseCBFQPConfig] = None):
        self.config = config or EllipseCBFQPConfig()
        self.counter = 0
        self.last_status = None
        self.last_h = 0.0
        self.last_h_dot = 0.0
        self.last_h_ddot = 0.0
        self.last_lhs_a_coeff = 0.0
        self.last_lhs_delta_coeff = 0.0
        self.last_h_ddot_base = 0.0
        self.last_rhs = 0.0
        self.last_solve_time_ms = 0.0
        self.last_slack = 0.0
        self.last_obstacle_x = None
        self.last_obstacle_y = None
        self.last_brake_gate_active = False

    def solve(self, car, obstacles, accel_ref, delta_ref):
        """Return safe acceleration and steering commands."""
        try:
            import cvxpy as cp
        except ImportError as exc:
            raise RuntimeError(
                'cvxpy is required to solve CBF-QP controls.'
            ) from exc

        a = cp.Variable()
        delta = cp.Variable()
        cfg = self.config
        constraints = []
        slack_terms = []
        selected_slack = None
        use_slack = float(cfg.slack_weight) > 0.0

        self.last_h = 0.0
        self.last_h_dot = 0.0
        self.last_h_ddot = 0.0
        self.last_lhs_a_coeff = 0.0
        self.last_lhs_delta_coeff = 0.0
        self.last_h_ddot_base = 0.0
        self.last_rhs = 0.0
        self.last_obstacle_x = None
        self.last_obstacle_y = None
        self.last_brake_gate_active = False
        min_h = float('inf')

        for obs in obstacles:
            dx = float(obs.x) - float(car.x)
            dy = float(obs.y) - float(car.y)
            v = float(car.v)
            phi = float(car.psi)

            cos_p = np.cos(phi)
            sin_p = np.sin(phi)

            # Obstacle coordinates in the car body frame. p is longitudinal
            # ahead of the car; q is lateral left/right of the car.
            p = dx * cos_p + dy * sin_p
            q = -dx * sin_p + dy * cos_p

            A = max(1e-6, float(cfg.a_ell) + float(getattr(obs, 'half_length', 0.0)))
            B = max(1e-6, float(cfg.b_ell) + float(getattr(obs, 'half_width', 0.0)))
            A2 = A**2
            B2 = B**2

            # Braking CBF: treat obstacles as stop boundaries for ego motion.
            # Using obstacle velocity here makes the QP try to regulate relative
            # speed; for this controller we want ego speed to go to zero when
            # the barrier requires deceleration.
            p_dot = -v
            q_dot = 0.0

            h = (p**2 / A2) + (q**2 / B2) - 2.8
            h_dot = 2.0 * p * p_dot / A2 + 2.0 * q * q_dot / B2
            h_ddot_base = 2.0 * (p_dot**2 / A2 + q_dot**2 / B2)
            lhs_a_coeff = -2.0 * p / A2
            lhs_delta_coeff = (
                cfg.gamma1
                * (v / max(1e-6, float(cfg.wheelbase)))
                * 2.0
                * p
                * q
                * (1.0 / A2 - 1.0 / B2)
            )
            rhs = -h_ddot_base - cfg.gamma1 * h_dot - cfg.gamma2 * h

            obstacle_slack = cp.Variable(nonneg=True) if use_slack else None
            if obstacle_slack is None:
                constraints.append(
                    lhs_a_coeff * a + lhs_delta_coeff * delta >= rhs
                )
            else:
                constraints.append(
                    lhs_a_coeff * a
                    + lhs_delta_coeff * delta
                    + obstacle_slack
                    >= rhs
                )
                slack_terms.append(obstacle_slack)
            brake_gate_active = h < 0.0 or rhs > 0.0

            if h < min_h:
                min_h = h
                self.last_h = float(h)
                self.last_h_dot = float(h_dot)
                self.last_lhs_a_coeff = float(lhs_a_coeff)
                self.last_lhs_delta_coeff = float(lhs_delta_coeff)
                self.last_h_ddot_base = float(h_ddot_base)
                self.last_rhs = float(rhs)
                self.last_obstacle_x = float(obs.x)
                self.last_obstacle_y = float(obs.y)
                self.last_brake_gate_active = bool(brake_gate_active)
                selected_slack = obstacle_slack

        constraints += [
            a >= cfg.min_accel,
            a <= cfg.max_accel,
            delta >= cfg.min_delta,
            delta <= cfg.max_delta,
        ]

        steering_weight = max(1e-6, float(cfg.gamma3))
        objective_terms = [
            (a - accel_ref) ** 2,
            steering_weight * (delta - delta_ref) ** 2,
        ]
        for obstacle_slack in slack_terms:
            objective_terms.append(
                float(cfg.slack_weight) * cp.square(obstacle_slack)
            )
        objective = cp.Minimize(sum(objective_terms))
        problem = cp.Problem(objective, constraints)
        start_time = time.perf_counter()
        problem.solve(solver=cfg.solver, warm_start=True)
        self.last_solve_time_ms = (time.perf_counter() - start_time) * 1000.0

        self.counter += 1
        self.last_status = problem.status

        if problem.status in ('optimal', 'optimal_inaccurate'):
            if selected_slack is not None and selected_slack.value is not None:
                self.last_slack = float(selected_slack.value)
            else:
                self.last_slack = 0.0
            if self.last_rhs > 0.0:
                print(f"[CBF_QP_DEBUG] status={problem.status} | rhs={self.last_rhs:.3f} | h={self.last_h:.3f} | lhs_a={self.last_lhs_a_coeff:.4f} | lhs_delta={self.last_lhs_delta_coeff:.4f} | accel_ref={accel_ref:.3f} | delta_ref={delta_ref:.3f} | solved_a={a.value:.3f} | solved_delta={delta.value:.3f} | slack={self.last_slack:.4f}")
            self.last_h_ddot = (
                self.last_h_ddot_base
                + self.last_lhs_a_coeff * float(a.value)
                + self.last_lhs_delta_coeff * float(delta.value)
            )
            return float(a.value), float(delta.value)

        self.last_slack = 0.0
        fail_accel, fail_delta = self.fail_safe(delta_ref)
        self.last_h_ddot = (
            self.last_h_ddot_base
            + self.last_lhs_a_coeff * fail_accel
            + self.last_lhs_delta_coeff * fail_delta
        )
        return fail_accel, fail_delta

    def clip(self, accel_ref, delta_ref):
        """Clip a nominal command to actuator limits."""
        cfg = self.config
        return (
            float(np.clip(accel_ref, cfg.min_accel, cfg.max_accel)),
            float(np.clip(delta_ref, cfg.min_delta, cfg.max_delta)),
        )

    def fail_safe(self, delta_ref):
        """Return conservative braking when the QP cannot certify safety."""
        cfg = self.config
        return (
            float(cfg.min_accel),
            float(np.clip(delta_ref, cfg.min_delta, cfg.max_delta)),
        )
