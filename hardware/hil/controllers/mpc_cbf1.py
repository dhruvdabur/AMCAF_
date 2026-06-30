
from dataclasses import dataclass
import heapq
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np

# Assuming MPCController is imported from the other file
from .mpc_cbf import MPCController
from .cbf_qp_ellipse import Car as CBFCar
from .cbf_qp_ellipse import EllipseCBFQPConfig
from .cbf_qp_ellipse import EllipseCBFQPSafetyFilter
from .cbf_qp_ellipse import PointObstacle
# Define constants at the top for clarity and easy modification.
MAX_STEERING_ANGLE = 0.785  # Example max steering angle in radians (45 degrees)

@dataclass
class VehicleState:
    """Represents the state of a vehicle in a 2D plane."""
    px: float
    py: float
    yaw: float
    v: float

@dataclass
class VehicleEllipse:
    """Represents a vehicle as an ellipse."""
    length: float
    width: float
    center_x: float
    center_y: float
    center_offset_x: float = 0.0
 
    def get_a(self) -> float:
        """Returns the semi-major axis (a) of the ellipse."""
        return self.length / 2.0
 
    def get_b(self) -> float:
        """Returns the semi-minor axis (b) of the ellipse."""
        return self.width / 2.0
    
    def barrier_value(self, px: float , py: float) -> float:
        """
        Computes the barrier function value for a point (px, py) relative to the ellipse.
        
        The barrier function is defined as:
            h(px, py) = ((px - center_x)^2 / a^2 + (py - center_y)^2 / b^2) - 1
        A value >= 0 indicates the point is outside or on the ellipse boundary (safe).
        """
        a = self.get_a()
        b = self.get_b()
        x_term = ((px - self.center_x) ** 2) / (a ** 2)
        y_term = ((py - self.center_y) ** 2) / (b ** 2)
        h = x_term + y_term - 1.0
        return h



@dataclass
class LidarProcessor:
    """Processes LIDAR data to detect obstacles."""
    def process_lidar_data(self, lidar_points: list) -> list:
        """Return finite obstacle points from raw lidar hits.

        Accepts the repo's LidarPoint objects, PointObstacle objects, dicts with
        x/y-style keys, or simple 2-item coordinate sequences.
        """
        obstacles = []
        for point in lidar_points or []:
            obstacle = self._to_point_obstacle(point)
            if obstacle is not None:
                obstacles.append(obstacle)
        return obstacles

    @staticmethod
    def _to_point_obstacle(point) -> Optional[PointObstacle]:
        if isinstance(point, PointObstacle):
            return point

        if isinstance(point, dict):
            x = point.get('x', point.get('x_px'))
            y = point.get('y', point.get('y_px'))
            half_length = point.get('half_length', point.get('length_px', 0.0))
            half_width = point.get('half_width', point.get('width_px', 0.0))
            vx = point.get('vx', 0.0)
            vy = point.get('vy', 0.0)
            if x is None and 'center' in point:
                center = point['center']
                x, y = center[0], center[1]
            if x is None or y is None:
                return None
            return PointObstacle(
                x=float(x),
                y=float(y),
                half_length=float(half_length) * 0.5
                if 'length_px' in point else float(half_length),
                half_width=float(half_width) * 0.5
                if 'width_px' in point else float(half_width),
                vx=float(vx),
                vy=float(vy),
            )

        x = getattr(point, 'x_px', getattr(point, 'x', None))
        y = getattr(point, 'y_px', getattr(point, 'y', None))
        if x is None or y is None:
            try:
                x, y = point[0], point[1]
            except (TypeError, IndexError):
                return None
        if not np.isfinite(float(x)) or not np.isfinite(float(y)):
            return None
        return PointObstacle(x=float(x), y=float(y))


class Path:
    """Represents a path in a 2D plane."""
    def __init__(self, waypoints: list):
        self.waypoints = waypoints
    def reference_trajectory(self, state, horizon):
        if self.waypoints is None or len(self.waypoints) == 0:
            return []
        return self.waypoints[:horizon]



    
class AckermanConverter:
    """Converts between Ackermann steering angles and vehicle kinematics."""
    def __init__(self, wheelbase: float):
        self.wheelbase = wheelbase
        self.max_steering_angle = MAX_STEERING_ANGLE

    def convert(self, v, omega):
        """
        Converts linear velocity (v) and angular velocity (omega) to Ackermann steering angle.
        This is a placeholder for the actual conversion logic.
        """
        if abs(v) < 0.001:  # Avoid division by zero
            steering_angle = 0.0
        else:
            steering_angle = np.arctan(self.wheelbase*omega/v)
        
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
        speed = v
        return speed, steering_angle
    

class MPC_CBF_Controller:
    """Model Predictive Controller with Control Barrier Function for safety."""
    def __init__(self,
                 vehicle_state: VehicleState,
                 path: Path,
                 lidar_processor: LidarProcessor,
                 mpc_controller: MPCController,
                 ackerman_converter: AckermanConverter,
                 dt: float,
                 cbf_filter: Optional[EllipseCBFQPSafetyFilter] = None,
                 cbf_config: Optional[EllipseCBFQPConfig] = None,
                 max_obstacles: int = 4,
                 target_speed: Optional[float] = None):
        self.vehicle_state = vehicle_state
        self.path = path
        self.lidar_processor = lidar_processor
        self.mpc_controller = mpc_controller
        self.ackerman_converter = ackerman_converter
        self.dt = dt
        self.cbf_filter = cbf_filter or EllipseCBFQPSafetyFilter(cbf_config)
        self.max_obstacles = max(1, int(max_obstacles))
        self.target_speed = target_speed
        self.last_nominal_control = (0.0, 0.0)
        self.last_safe_control = (0.0, 0.0)
        self.last_obstacles = []
        self.cbf_active = False

    def reset(self):
        """Reset the wrapped MPC controller if it exposes reset state."""
        if hasattr(self.mpc_controller, 'reset'):
            self.mpc_controller.reset()
        self.last_nominal_control = (0.0, 0.0)
        self.last_safe_control = (0.0, 0.0)
        self.last_obstacles = []
        self.cbf_active = False

    def compute_control(self, lidar_points: list):
        """
        Computes the control commands for the vehicle based on its current state,
        the desired path, and LIDAR data.

        Returns:
            (accel, steer): MPC's nominal acceleration and steering filtered by
            the elliptical CBF-QP when obstacles are present.
        """
        state = self.vehicle_state
        if state is None:
            return 0.0, 0.0  # No control if vehicle state is not defined

        accel_ref, delta_ref = self._nominal_mpc_control(state)
        self.last_nominal_control = (accel_ref, delta_ref)

        obstacles = self._closest_obstacles(
            self.lidar_processor.process_lidar_data(lidar_points=lidar_points),
            state,
        )
        self.last_obstacles = obstacles
        if not obstacles:
            safe_control = self.cbf_filter.clip(accel_ref, delta_ref)
            self.last_safe_control = safe_control
            self.cbf_active = False
            return safe_control

        car = CBFCar(
            x=float(state.px),
            y=float(state.py),
            psi=float(state.yaw),
            v=max(0.0, float(state.v)),
            v_cmd=max(0.0, float(self.target_speed if self.target_speed is not None else state.v)),
        )
        accel, delta = self.cbf_filter.solve(
            car,
            obstacles,
            accel_ref,
            delta_ref,
        )
        self.last_safe_control = (accel, delta)
        self.cbf_active = (
            abs(accel - accel_ref) > 1e-6
            or abs(delta - delta_ref) > 1e-6
        )
        return self.last_safe_control

    def _nominal_mpc_control(self, state: VehicleState) -> Tuple[float, float]:
        """Return the nominal MPC acceleration and steering command."""
        target_speed = float(
            self.target_speed if self.target_speed is not None else state.v
        )
        if hasattr(self.mpc_controller, 'step'):
            accel, steer = self.mpc_controller.step(
                px=float(state.px),
                py=float(state.py),
                yaw=float(state.yaw),
                vel=float(state.v),
                target_speed=target_speed,
            )
            return float(accel), float(steer)

        reference_trajectory = self.path.reference_trajectory(
            state=state,
            horizon=10,
        )
        u_mpc = self.mpc_controller.solve(
            state=state,
            reference_trajectory=reference_trajectory,
        )
        return self._parse_nominal_control(u_mpc)

    def _parse_nominal_control(self, u_mpc) -> Tuple[float, float]:
        """Normalize a controller output into (accel, steer)."""
        if isinstance(u_mpc, dict):
            accel = u_mpc.get('accel', u_mpc.get('a', 0.0))
            steer = u_mpc.get('steer', u_mpc.get('delta', 0.0))
            return float(accel), float(steer)

        values = np.asarray(u_mpc, dtype=float).reshape(-1)
        if values.size < 2:
            return 0.0, 0.0
        return float(values[0]), float(values[1])

    def _closest_obstacles(
        self,
        obstacle_points: Iterable[PointObstacle],
        state: VehicleState,
    ) -> Sequence[PointObstacle]:
        """Return closest finite obstacles for the CBF-QP."""
        finite_obstacles = [
            obs for obs in obstacle_points or []
            if np.isfinite(obs.x) and np.isfinite(obs.y)
        ]
        return heapq.nsmallest(
            self.max_obstacles,
            finite_obstacles,
            key=lambda obs: (obs.x - state.px) ** 2 + (obs.y - state.py) ** 2,
        )
