"""Reusable controller core for dynamic straight-road HIL scenarios."""

from .straight_static_controller import StraightControlResult
from .straight_static_controller import StraightStaticController
from .straight_static_controller import is_road_boundary_obstacle


class DynamicStraightController(StraightStaticController):
    """Straight-road controller with explicit dynamic-obstacle layers."""

    def __init__(self, config):
        super().__init__(config)
        self.static_scene_obstacles = []
        self.detected_random_obstacles = []
        self.dynamic_obstacles = []
        self.use_random_static_obstacles = False

    def set_obstacle_layers(
        self,
        static_scene_obstacles,
        detected_random_obstacles,
        dynamic_obstacles,
        use_random_static_obstacles=False,
    ):
        """Set current static, random, and moving obstacle layers."""
        self.static_scene_obstacles = list(static_scene_obstacles)
        self.detected_random_obstacles = list(detected_random_obstacles)
        self.dynamic_obstacles = list(dynamic_obstacles)
        self.use_random_static_obstacles = bool(use_random_static_obstacles)
        self.static_obstacles = self.combined_obstacles(use_random_static_obstacles)

    def gap_planner_obstacles(self):
        """Return static/random plus dynamic traffic for free-space planning."""
        return self.combined_obstacles(self.use_random_static_obstacles)

    def combined_obstacles(self, use_random_static_obstacles=False):
        """Return the obstacle list used by the inherited safety logic."""
        obstacles = []
        if getattr(self.config, 'include_road_boundary_walls', False):
            obstacles.extend(getattr(self, 'road_boundary_obstacles', []))
        if use_random_static_obstacles:
            obstacles.extend(self.detected_random_obstacles)
        else:
            obstacles.extend(self.static_scene_obstacles)
        obstacles.extend(self.dynamic_obstacles)
        if getattr(self.config, 'include_road_boundary_walls', False):
            return obstacles
        return [obstacle for obstacle in obstacles if not is_road_boundary_obstacle(obstacle)]


DynamicStraightControlResult = StraightControlResult
