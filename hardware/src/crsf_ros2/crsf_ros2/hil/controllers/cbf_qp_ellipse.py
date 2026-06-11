"""QP-based control barrier function safety filter with elliptical barriers."""

from dataclasses import dataclass
from typing import Iterable
from typing import Optional

import numpy as np


@dataclass
class VehicleState:
    """Minimal bicycle-model state consumed by the CBF-QP."""

    x: float
    y: float
    v: float
    psi: float


@dataclass
class PointObstacle:
    """Circular obstacle center consumed by the CBF-QP."""

    x: float
    y: float


@dataclass
class EllipseCBFQPConfig:
    """Tunable constants for the elliptical CBF-QP safety filter."""

    a_ell: float = 2.0
    b_ell: float = 1.0
    wheelbase: float = 2.0
    gamma1: float = 1.0
    gamma2: float = 1.0
    gamma3: float = 1.0
    min_accel: float = -5.0
    max_accel: float = 0.5
    min_delta: float = -0.4
    max_delta: float = 0.4
    solver: str = 'OSQP'


class EllipseCBFQPSafetyFilter:
    """Filter nominal commands through a CBF-QP with elliptical barriers."""

    def __init__(self, config: Optional[EllipseCBFQPConfig] = None):
        self.config = config or EllipseCBFQPConfig()
        self.counter = 0
        self.last_status = None
        self.last_h = 0.0
        self.last_lhs_a_coeff = 0.0
        self.last_lhs_delta_coeff = 0.0
        self.last_rhs = 0.0

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
        
        # Reset debug values
        self.last_h = 0.0
        self.last_lhs_a_coeff = 0.0
        self.last_lhs_delta_coeff = 0.0
        self.last_rhs = 0.0
        min_h = float('inf')

        for obs in obstacles:
            # Relative position in world coordinates
            dx = float(car.x) - float(obs.x)
            dy = float(car.y) - float(obs.y)
            v = float(car.v)
            phi = float(car.psi)

            # Local coordinates (longitudinal X, lateral Y) relative to vehicle heading
            cos_p = np.cos(phi)
            sin_p = np.sin(phi)
            X = dx * cos_p + dy * sin_p
            Y = -dx * sin_p + dy * cos_p

            A2 = cfg.a_ell**2
            B2 = cfg.b_ell**2

            # Elliptical barrier function h >= 0
            h = (X**2 / A2) + (Y**2 / B2) - 1.0
            
            # First-order condition components
            # h_dot = (2X/A2)*X_dot + (2Y/B2)*Y_dot
            # We use a second-order CBF condition: h_ddot + gamma1*h_dot + gamma2*h >= 0
            # Simplified approach:
            lhs_a_coeff = 2.0 * X / A2
            lhs_delta_coeff = (2.0 * v**2 / cfg.wheelbase) * Y * (1.0/A2 - 1.0/B2)
            rhs = -(2.0 * v**2 / A2) - cfg.gamma1 * (2.0 * X * v / A2) - cfg.gamma2 * h

            constraints.append(lhs_delta_coeff * delta + lhs_a_coeff * a >= rhs)

            # Store debug info for the "closest" obstacle (minimum h)
            if h < min_h:
                min_h = h
                self.last_h = float(h)
                self.last_lhs_a_coeff = float(lhs_a_coeff)
                self.last_lhs_delta_coeff = float(lhs_delta_coeff)
                self.last_rhs = float(rhs)

        constraints += [
            a >= cfg.min_accel,
            a <= cfg.max_accel,
            delta >= cfg.min_delta,
            delta <= cfg.max_delta,
        ]

        steering_weight = max(1e-6, float(cfg.gamma3))
        objective = cp.Minimize(
            (a - accel_ref) ** 2 + steering_weight * (delta - delta_ref) ** 2
        )
        problem = cp.Problem(objective, constraints)
        problem.solve(solver=cfg.solver, warm_start=True)

        self.counter += 1
        self.last_status = problem.status

        if problem.status in ('optimal', 'optimal_inaccurate'):
            return float(a.value), float(delta.value)

        return self.fail_safe(delta_ref)

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
