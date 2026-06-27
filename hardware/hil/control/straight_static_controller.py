"""Reusable controller core for the straight-road ArUco follower."""

import numpy as np

from ..road import make_straight_road_scene
from .ellipse_static_controller import EllipseControlResult
from .ellipse_static_controller import EllipseStaticController


def is_road_boundary_obstacle(obstacle):
    """Return whether an obstacle record represents a track boundary wall."""
    return str(obstacle.get('kind', '')).startswith('road_boundary_wall')


class StraightStaticController(EllipseStaticController):
    """ROS-free controller state for the open straight road follower."""

    def build_track_scene(self, width, height):
        """Create the straight road scene for the current image size."""
        self.last_image_size = (width, height)
        scene = make_straight_road_scene(width, height, self.config)
        self.road_boundaries = scene['boundaries']
        self.road_tangents = scene['tangents']
        self.road_normals = scene['normals']
        self.road_half_width_px = scene['road_half_width_px']
        self.road_boundary_obstacles = [
            obstacle for obstacle in scene['obstacles']
            if is_road_boundary_obstacle(obstacle)
        ]
        self.static_obstacles = [
            obstacle for obstacle in scene['obstacles']
            if not is_road_boundary_obstacle(obstacle)
        ]
        self.latest_free_space_target = None
        self.latest_free_space_interval = None
        self.latest_free_space_lateral_target_px = 0.0
        self.latest_free_space_blocked_intervals = []
        self.latest_free_space_intervals = []
        self.smoothed_free_space_lateral_target_px = None
        self.track_points = scene['centerline']
        self.mpc_controller.update_track(self.track_points)

    def closed_road_scene_enabled(self):
        """Return whether the road scene should wrap around as a loop."""
        return False

    def track_error(self, center, heading):
        """Find an open-road lookahead target and signed steering error."""
        self.track_points = self.select_free_space_path(center)
        distances = np.linalg.norm(self.track_points - center, axis=1)
        nearest_index = int(np.argmin(distances))
        target_index = min(
            nearest_index + self.config.lookahead_points,
            len(self.track_points) - 1,
        )
        self.latest_nearest_index = nearest_index
        self.latest_target_index = target_index
        target = self.free_space_target(target_index)
        self.latest_free_space_target = target
        next_index = min(target_index + 1, len(self.track_points) - 1)
        previous_index = max(target_index - 1, 0)
        next_target = self.track_points[next_index]
        previous_target = self.track_points[previous_index]
        tangent = next_target - previous_target
        tangent_norm = np.linalg.norm(tangent)
        if tangent_norm > 0.0:
            tangent = tangent / tangent_norm
        vehicle_right = np.array(
            [-np.sin(heading), np.cos(heading)],
            dtype=np.float32,
        )
        error = float(np.dot(target - center, vehicle_right))
        return target, tangent, error, nearest_index

    def target_lap_reached(self):
        """Straight roads do not use lap-limit stopping."""
        return False


StraightControlResult = EllipseControlResult
