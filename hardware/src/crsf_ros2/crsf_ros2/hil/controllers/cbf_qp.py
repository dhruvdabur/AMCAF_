"""QP-based control barrier function safety filter."""

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
class CBFQPConfig:
    """Tunable constants for the CBF-QP safety filter."""

    r_safe: float = 2.0
    wheelbase: float = 2.0
    gamma1: float = 1.0
    gamma2: float = 1.0
    gamma3: float = 1.0
    min_accel: float = -5.0
    max_accel: float = 0.5
    min_delta: float = -0.4
    max_delta: float = 0.4
    solver: str = 'OSQP'


class CBFQPSafetyFilter:
    """Filter nominal acceleration and steering commands through a CBF-QP."""

    def __init__(self, config: Optional[CBFQPConfig] = None):
        self.config = config or CBFQPConfig()
        self.counter = 0
        self.last_status = None

    def solve(self, car, obstacles, accel_ref, delta_ref):
        """
        Return safe acceleration and steering commands.

        The car object must expose x, y, v, and psi attributes. Each obstacle
        must expose x and y attributes.
        """
        try:
            import cvxpy as cp
        except ImportError as exc:
            raise RuntimeError(
                'cvxpy is required to solve CBF-QP controls. '
                'Install cvxpy in this environment before using cbf_qp.'
            ) from exc

        a = cp.Variable()
        delta = cp.Variable()
        cfg = self.config

        constraints = []
        for obs in obstacles:
            dx = float(car.x) - float(obs.x)
            dy = float(car.y) - float(obs.y)
            v = float(car.v)
            phi = float(car.psi)

            h = dx**2 + dy**2 - cfg.r_safe**2
            heading_projection = dx * np.cos(phi) + dy * np.sin(phi)
            lateral_projection = -dx * np.sin(phi) + dy * np.cos(phi)
            v_dot_h = 2.0 * v * heading_projection

            lhs_delta_coeff = (2.0 * v**2 / cfg.wheelbase) * lateral_projection
            lhs_a_coeff = 2.0 * heading_projection
            rhs = -2.0 * v**2 - cfg.gamma1 * v_dot_h - cfg.gamma2 * h

            constraints.append(lhs_delta_coeff * delta + lhs_a_coeff * a >= rhs)

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

        print(f'CBF-QP failed: {problem.status}', self.counter)
        return self.clip(accel_ref, delta_ref)

    def clip(self, accel_ref, delta_ref):
        """Clip a nominal command to actuator limits."""
        cfg = self.config
        return (
            float(np.clip(accel_ref, cfg.min_accel, cfg.max_accel)),
            float(np.clip(delta_ref, cfg.min_delta, cfg.max_delta)),
        )


def cbf_qp(
    car,
    obstacles: Iterable,
    accel_ref: float,
    delta_ref: float,
    config: Optional[CBFQPConfig] = None,
):
    """Functional wrapper around CBFQPSafetyFilter.solve()."""
    return CBFQPSafetyFilter(config).solve(car, obstacles, accel_ref, delta_ref)
