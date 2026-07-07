"""Model Predictive Control (MPC) tracking controller using do_mpc and CasADi."""

from dataclasses import dataclass
import math
import time
from typing import Optional, Tuple
import warnings

import numpy as np

try:
    import casadi as ca
    import do_mpc
    HAS_MPC_DEPS = True
except ImportError:
    HAS_MPC_DEPS = False


@dataclass
class MPCConfig:
    """Configuration parameter defaults for the MPC path-following tracking controller."""

    wheelbase: float = 2.0
    delta_t: float = 0.02
    horizon_T: int = 10
    max_steer: float = 1.3
    min_steer: float = -1.3
    max_accel: float = 0.01
    min_accel: float = -10.0
    v_min: float = 0.0
    v_max: float = 2000.0
    ipopt_max_iter: int = 40
    ipopt_print_level: int = 0



def transform_reference_to_vehicle_frame(car_x: float, car_y: float, car_yaw: float, ref_trajectory: np.ndarray) -> np.ndarray:
    """Transform reference trajectory from world frame to vehicle/body frame.
    
    ref_trajectory: numpy array of shape (4, N) where:
                    row 0: ref_x (world)
                    row 1: ref_y (world)
                    row 2: ref_yaw (world)
                    row 3: ref_vel
    returns: numpy array of shape (4, N) transformed to local vehicle frame:
             row 0: ref_x_local (relative to vehicle: x = 0, y = 0, yaw = 0)
             row 1: ref_y_local
             row 2: ref_yaw_local (relative yaw normalized to [-pi, pi])
             row 3: ref_vel (unchanged)
    """
    N = ref_trajectory.shape[1]
    local_trajectory = np.zeros_like(ref_trajectory)
    
    cos_yaw = math.cos(car_yaw)
    sin_yaw = math.sin(car_yaw)
    
    for k in range(N):
        dx = ref_trajectory[0, k] - car_x
        dy = ref_trajectory[1, k] - car_y
        
        # Transform translation
        local_trajectory[0, k] = cos_yaw * dx + sin_yaw * dy
        local_trajectory[1, k] = -sin_yaw * dx + cos_yaw * dy
        
        # Transform rotation (heading)
        dyaw = ref_trajectory[2, k] - car_yaw
        # Normalize to [-pi, pi]
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
        local_trajectory[2, k] = dyaw
        
        # Velocity remains unchanged
        local_trajectory[3, k] = ref_trajectory[3, k]
        
    return local_trajectory


class MPCController:
    """Path-following Model Predictive Controller (MPC) using a kinematic bicycle model.

    Tracks a given set of centerline path points using discrete-time CasADi/do-mpc optimization.
    """

    def __init__(self, config: Optional[MPCConfig] = None):
        self.config = config or MPCConfig()
        self.delta_t = self.config.delta_t
        self.T = self.config.horizon_T

        # Course tracking variables
        self.track_points = None
        self._n_course = 0
        self._course_length = 0.0
        self._ext_x = None
        self._ext_y = None
        self._ext_yaw = None
        self._ext_arc = None
        self._prev_waypoint_idx = 0

        self.solve_time_ms = 0.0
        self.solver_success = False
        self.optimal_trajectory = None

        self._initialized = False
        self._model = None
        self._mpc = None

        if HAS_MPC_DEPS:
            self._setup_optimizer()

    def _setup_optimizer(self):
        """Set up the do_mpc model and controller."""
        cfg = self.config
        L = cfg.wheelbase
        dt = self.delta_t

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            model = do_mpc.model.Model("discrete")

        model.set_variable("_x", "px")
        model.set_variable("_x", "py")
        model.set_variable("_x", "yaw")
        model.set_variable("_x", "vel")
        model.set_variable("_u", "steer")
        model.set_variable("_u", "accel")
        model.set_variable("_tvp", "ref_x")
        model.set_variable("_tvp", "ref_y")
        model.set_variable("_tvp", "ref_yaw")
        model.set_variable("_tvp", "ref_vel")

        # Use wrapped atan2 sin/cos to compute phase-independent yaw error
        yaw_err = ca.atan2(
            ca.sin(model.x["yaw"] - model.tvp["ref_yaw"]),
            ca.cos(model.x["yaw"] - model.tvp["ref_yaw"]),
        )
        model.set_expression("yaw_err", yaw_err)

        # Kinematic bicycle model transitions in image space (positive steer = clockwise / right turn)
        model.set_rhs("px", model.x["px"] + model.x["vel"] * ca.cos(model.x["yaw"]) * dt)
        model.set_rhs("py", model.x["py"] + model.x["vel"] * ca.sin(model.x["yaw"]) * dt)
        model.set_rhs("yaw", model.x["yaw"] + model.x["vel"] / L * ca.tan(model.u["steer"]) * dt)
        model.set_rhs("vel", model.x["vel"] + model.u["accel"] * dt)

        model.setup()
        self._model = model

        # Setup MPC
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            mpc = do_mpc.controller.MPC(model)

        ipopt_opts = {
            "ipopt.max_iter": cfg.ipopt_max_iter,
            "ipopt.print_level": cfg.ipopt_print_level,
            "ipopt.sb": "yes",
            "print_time": 0,
        }

        mpc.set_param(
            n_horizon=self.T,
            t_step=dt,
            n_robust=0,
            store_full_solution=True,
            nlpsol_opts=ipopt_opts,
        )

        tvp_template = mpc.get_tvp_template()
        self._current_ref = np.zeros((4, self.T + 1))

        def tvp_fun(t_now):
            for k in range(self.T + 1):
                tvp_template["_tvp", k, "ref_x"] = float(self._current_ref[0, k])
                tvp_template["_tvp", k, "ref_y"] = float(self._current_ref[1, k])
                tvp_template["_tvp", k, "ref_yaw"] = float(self._current_ref[2, k])
                tvp_template["_tvp", k, "ref_vel"] = float(self._current_ref[3, k])
            return tvp_template

        mpc.set_tvp_fun(tvp_fun)

        # Cost weights
        # Stage weights: [px, py, yaw, vel]
        sw = np.array([2.0, 2.0, 500.0, 1.0])
        tw = sw * 2.0
        # Control weights: [steer, accel]
        cw = np.array([10.0, 0.5])
        # Smoothness weights: [steer_rate, accel_rate]
        smw = np.array([10.0, 1.0])

        x = model.x
        tvp = model.tvp

        lterm = (
            sw[0] * (x["px"] - tvp["ref_x"]) ** 2
            + sw[1] * (x["py"] - tvp["ref_y"]) ** 2
            + sw[2] * model.aux["yaw_err"] ** 2
            + sw[3] * (x["vel"] - tvp["ref_vel"]) ** 2
        )
        mterm = (
            tw[0] * (x["px"] - tvp["ref_x"]) ** 2
            + tw[1] * (x["py"] - tvp["ref_y"]) ** 2
            + tw[2] * model.aux["yaw_err"] ** 2
            + tw[3] * (x["vel"] - tvp["ref_vel"]) ** 2
        )

        mpc.set_objective(lterm=lterm, mterm=mterm)
        mpc.set_rterm(
            steer=float(cw[0] + smw[0]),
            accel=float(cw[1] + smw[1]),
        )

        mpc.bounds["lower", "_u", "steer"] = cfg.min_steer
        mpc.bounds["upper", "_u", "steer"] = cfg.max_steer
        mpc.bounds["lower", "_u", "accel"] = cfg.min_accel
        mpc.bounds["upper", "_u", "accel"] = cfg.max_accel
        mpc.bounds["lower", "_x", "vel"] = cfg.v_min
        mpc.bounds["upper", "_x", "vel"] = cfg.v_max

        mpc.setup()
        self._mpc = mpc

    def update_track(self, track_points: np.ndarray):
        """Set up path coordinates and precompute arclength/heading table."""
        if track_points is None or len(track_points) < 2:
            return

        self.track_points = np.asarray(track_points, dtype=np.float32)
        n = len(self.track_points)

        # Compute unit tangents
        previous_points = np.vstack((self.track_points[-1], self.track_points[:-1]))
        next_points = np.vstack((self.track_points[1:], self.track_points[0]))
        tangents = next_points - previous_points
        norms = np.linalg.norm(tangents, axis=1)
        norms[norms <= 1e-6] = 1.0
        tangents = tangents / norms[:, None]

        # Compute heading angles (yaw) from tangents
        yaws = np.array([math.atan2(t[1], t[0]) for t in tangents])

        # Compute cumulative arclength
        seg = np.hypot(np.diff(self.track_points[:, 0]), np.diff(self.track_points[:, 1]))
        last_seg = np.hypot(
            self.track_points[0, 0] - self.track_points[-1, 0],
            self.track_points[0, 1] - self.track_points[-1, 1],
        )
        seg = np.append(seg, last_seg)
        arc = np.concatenate([[0.0], np.cumsum(seg)])
        total = arc[-1]

        self._n_course = n
        self._course_length = total

        # Tile 3 times to wrap around closed track seamlessly
        self._ext_x = np.tile(self.track_points[:, 0], 3)
        self._ext_y = np.tile(self.track_points[:, 1], 3)
        self._ext_yaw = np.tile(yaws, 3)
        self._ext_arc = np.concatenate([arc[:-1], arc[:-1] + total, arc[:-1] + 2.0 * total])
        self._prev_waypoint_idx = n
        self._initialized = False

    def _get_reference_trajectory(self, px: float, py: float, current_speed: float, target_speed: float) -> np.ndarray:
        if self._ext_arc is None or self._n_course == 0:
            return np.zeros((4, self.T + 1))

        n = self._n_course
        dt = self.delta_t
        step_len = target_speed * dt
        max_search_idx = len(self._ext_x)

        # Sliding window search for the closest waypoint to retain path continuity
        WINDOW = 60
        best_idx = self._prev_waypoint_idx
        best_d = float("inf")
        for off in range(-WINDOW // 2, WINDOW // 2):
            ci = self._prev_waypoint_idx + off
            ci = max(0, min(ci, max_search_idx - 1))
            d = (self._ext_x[ci] - px) ** 2 + (self._ext_y[ci] - py) ** 2
            if d < best_d:
                best_d = d
                best_idx = ci

        # Force best_idx to remain in the middle tiled segment [n, 2 * n - 1]
        best_idx = (best_idx - n) % n + n
        self._prev_waypoint_idx = best_idx
        s0 = self._ext_arc[best_idx]
        ref = np.zeros((4, self.T + 1))

        for k in range(self.T + 1):
            target_s = s0 + k * step_len
            search_start = best_idx
            search_end = best_idx + n
            diffs = np.abs(self._ext_arc[search_start:search_end] - target_s)
            ki = search_start + int(np.argmin(diffs))
            ki = ki % max_search_idx

            ref[0, k] = self._ext_x[ki]
            ref[1, k] = self._ext_y[ki]
            ref[2, k] = self._ext_yaw[ki]
            ref[3, k] = target_speed

        return ref

    def step(self, px: float, py: float, yaw: float, vel: float, target_speed: float) -> Tuple[float, float]:
        """Compute the nominal control commands (accel, steer) for the current state."""
        if not HAS_MPC_DEPS or self._mpc is None or self._ext_arc is None:
            # Fallback to zero steering and soft deceleration if dependencies are missing
            return self.config.min_accel * 0.1, 0.0

        ref_world = self._get_reference_trajectory(px, py, vel, target_speed)
        self._current_ref = transform_reference_to_vehicle_frame(px, py, yaw, ref_world)

        x0_local = np.array([[0.0], [0.0], [0.0], [vel]])

        if not self._initialized:
            self._mpc.x0 = x0_local
            self._mpc.set_initial_guess()
            self._initialized = True

        t_start = time.perf_counter()
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                u0 = self._mpc.make_step(x0_local)
            self.solver_success = True
            steer = float(np.clip(np.asarray(u0).flat[0], self.config.min_steer, self.config.max_steer))
            accel = float(np.clip(np.asarray(u0).flat[1], self.config.min_accel, self.config.max_accel))
        except Exception as exc:
            self.solver_success = False
            import traceback
            print(f"[MPC_ERROR] make_step failed: {exc}")
            traceback.print_exc()
            # Fallback to decel and neutral steer on solver crash
            steer = 0.0
            accel = self.config.min_accel * 0.2
            self._initialized = False

        self.solve_time_ms = (time.perf_counter() - t_start) * 1000.0

        # Save optimal predicted path for visualization
        try:
            px_pred = [float(np.asarray(self._mpc.opt_x_num["_x", k, 0, "px"]).flat[0]) for k in range(self.T + 1)]
            py_pred = [float(np.asarray(self._mpc.opt_x_num["_x", k, 0, "py"]).flat[0]) for k in range(self.T + 1)]
            self.optimal_trajectory = (px_pred, py_pred)
        except Exception:
            self.optimal_trajectory = None

        return accel, steer

    def reset(self):
        """Reset internal optimization state."""
        self._initialized = False
        self._prev_waypoint_idx = 0
        self.optimal_trajectory = None
