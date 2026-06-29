#!/usr/bin/env python3
"""Standalone HIL-style virtual vehicle visualization."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


FREE_FLOW_DYNAMIC_DIRECTION = 1.0
FREE_FLOW_DYNAMIC_SPEED_FACTORS = (0.30, 0.34)


@dataclass
class VehicleState:
    """Synthetic image-space ego vehicle state."""

    x: float
    y: float
    heading: float
    speed: float

    @property
    def center(self) -> np.ndarray:
        return np.array([self.x, self.y], dtype=np.float32)


@dataclass
class SimResult:
    """Lightweight state summary for one visual simulation step."""

    lateral_error_px: float
    heading_error_rad: float
    safety_clearance_px: float


@dataclass
class RuntimeStats:
    """Lightweight simulation summary."""

    steps: int = 0
    min_clearance_px: float = float('inf')
    max_abs_lateral_error_px: float = 0.0
    max_speed_pps: float = 0.0
    stopped_early: bool = False
    stop_reason: str = 'completed'


class RoadModel:
    """Image-space straight-road geometry for the standalone visual sim."""

    def __init__(self, config):
        self.config = config
        self.width = int(config.virtual_width)
        self.height = int(config.virtual_height)
        self.half_width_px = float(config.road_half_width_px)
        self.center_y = float(config.track_center_y)
        margin_x = max(40.0, self.width * 0.08)
        self.start_x = margin_x
        self.end_x = self.width - margin_x
        self.track_points = self.make_track_points()
        self.road_tangents = np.tile(np.array([1.0, 0.0], dtype=np.float32), (len(self.track_points), 1))
        self.road_normals = np.tile(np.array([0.0, 1.0], dtype=np.float32), (len(self.track_points), 1))
        self.road_boundaries = [
            self.track_points + self.road_normals * self.half_width_px,
            self.track_points - self.road_normals * self.half_width_px,
        ]

    def make_track_points(self):
        x = np.linspace(self.start_x, self.end_x, 240, dtype=np.float32)
        y = np.full_like(x, self.center_y)
        return np.column_stack((x, y))

    @property
    def path_length_px(self):
        return max(1.0, self.end_x - self.start_x)

    def point_at_progress(self, progress, lateral_offset_px=0.0):
        progress = float(np.clip(progress, 0.0, 1.0))
        x = self.start_x + progress * self.path_length_px
        y = self.center_y + float(lateral_offset_px)
        return np.array([x, y], dtype=np.float32)

    def progress_of(self, center):
        return float(np.clip((float(center[0]) - self.start_x) / self.path_length_px, 0.0, 1.0))

    def lateral_error(self, center):
        return float(center[1] - self.center_y)


class DynamicScenarioLibrary:
    """Scenario builder and updater for visual traffic only."""

    def __init__(self, road, config):
        self.road = road
        self.config = config
        self.track_points = road.track_points
        self.road_tangents = road.road_tangents
        self.road_normals = road.road_normals
        self.static_scene_obstacles = self.make_static_obstacles()
        self.dynamic_specs = []
        self.dynamic_obstacles = []
        self.start_time = 0.0

    def make_static_obstacles(self):
        if getattr(self.config, 'no_static_obstacles', False):
            return []
        return [
            self.make_obstacle(
                progress=0.72,
                lateral_offset_px=-self.road.half_width_px * 0.34,
                length_px=92.0,
                width_px=54.0,
                kind='static',
            )
        ]

    def reset(self, start_time, ego_center):
        self.start_time = float(start_time)
        rng = np.random.default_rng(int(self.config.dynamic_obstacle_seed))
        scenario = getattr(self.config, 'traffic_scenario', 'free_flow')
        if scenario == 'free_flow':
            self.dynamic_specs = self.make_free_flow_specs(rng)
        else:
            self.dynamic_specs = self.make_scenario_specs(scenario)
        self.place_specs_ahead_of_ego(ego_center)
        self.dynamic_obstacles = []

    def place_specs_ahead_of_ego(self, ego_center):
        if not self.dynamic_specs:
            return
        ego_progress = self.road.progress_of(ego_center)
        earliest_progress = min(float(spec.get('base_progress', 0.0)) for spec in self.dynamic_specs)
        min_start_progress = ego_progress + max(float(self.config.dynamic_obstacle_min_gap_progress), 0.18)
        shift = max(0.0, min_start_progress - earliest_progress)
        for spec in self.dynamic_specs:
            spec['base_progress'] = float(spec.get('base_progress', 0.0)) + shift

    def make_free_flow_specs(self, rng):
        count = max(0, int(self.config.dynamic_obstacle_count))
        if count <= 0:
            return []
        offsets = np.linspace(
            -self.road.half_width_px * 0.34,
            self.road.half_width_px * 0.34,
            max(2, count),
        )
        specs = []
        for index in range(count):
            progress = 0.22 + 0.16 * index
            speed_factor = FREE_FLOW_DYNAMIC_SPEED_FACTORS[index % len(FREE_FLOW_DYNAMIC_SPEED_FACTORS)]
            specs.append(
                {
                    'index': index,
                    'base_progress': progress,
                    'base_offset_px': float(offsets[index % len(offsets)]),
                    'progress_speed_pps': self.nominal_ego_progress_speed() * speed_factor * FREE_FLOW_DYNAMIC_DIRECTION,
                    'length_px': float(rng.uniform(74.0, 104.0)),
                    'width_px': float(rng.uniform(46.0, 62.0)),
                    'kind': 'dynamic',
                    'wrap_progress': True,
                }
            )
        return specs

    def make_scenario_specs(self, scenario):
        half_width = self.road.half_width_px
        ego_speed = self.nominal_ego_progress_speed()
        left = -half_width * 0.34
        right = half_width * 0.34
        center = 0.0

        def spec(index, progress, offset, speed, **extra):
            payload = {
                'index': index,
                'base_progress': float(progress),
                'base_offset_px': float(offset),
                'progress_speed_pps': float(speed),
                'length_px': float(extra.pop('length_px', 92.0)),
                'width_px': float(extra.pop('width_px', 54.0)),
                'kind': 'dynamic',
                'wrap_progress': True,
            }
            payload.update(extra)
            return payload

        if scenario == 'lead_slowdown':
            return [
                spec(0, 0.26, center, ego_speed * 0.12, slowdown_rate=0.75),
                spec(1, 0.55, right * 0.6, ego_speed * 0.10, length_px=78.0, width_px=48.0),
            ]
        if scenario == 'cut_in':
            return [
                spec(
                    0,
                    0.32,
                    left,
                    ego_speed * 0.12,
                    lateral_target_px=center,
                    lateral_start_delay_s=2.0,
                    lateral_target_duration_s=3.4,
                ),
                spec(
                    1,
                    0.56,
                    right,
                    ego_speed * 0.10,
                    lateral_target_px=center,
                    lateral_start_delay_s=5.2,
                    lateral_target_duration_s=3.2,
                    length_px=82.0,
                    width_px=50.0,
                ),
            ]
        if scenario == 'dense_flow':
            return [
                spec(0, 0.20, left * 0.8, ego_speed * 0.12),
                spec(1, 0.34, center, ego_speed * 0.09),
                spec(2, 0.48, right * 0.8, ego_speed * 0.12),
                spec(3, 0.62, left * 0.3, ego_speed * 0.10),
                spec(4, 0.76, right * 0.3, ego_speed * 0.11),
            ]
        if scenario == 'lane_weave':
            return [
                spec(0, 0.20, left * 0.8, ego_speed * 0.12, lateral_wave_amp=half_width * 0.16, lateral_wave_period_s=4.8),
                spec(1, 0.42, center, ego_speed * 0.10, lateral_wave_amp=half_width * 0.18, lateral_wave_period_s=5.4, phase_s=1.8),
                spec(2, 0.64, right * 0.8, ego_speed * 0.12, lateral_wave_amp=half_width * 0.16, lateral_wave_period_s=6.0, phase_s=3.6),
            ]
        return self.make_free_flow_specs(np.random.default_rng(int(self.config.dynamic_obstacle_seed)))

    def nominal_ego_progress_speed(self):
        return max(0.001, float(self.config.target_track_speed_pps) / self.road.path_length_px)

    def step(self, now):
        elapsed = max(0.0, float(now) - self.start_time)
        self.dynamic_obstacles = [self.obstacle_from_spec(spec, elapsed) for spec in self.dynamic_specs]
        return self.dynamic_obstacles

    def obstacle_from_spec(self, spec, elapsed):
        progress = float(spec.get('base_progress', 0.0))
        progress += float(spec.get('progress_speed_pps', 0.0)) * elapsed
        if spec.get('slowdown_rate'):
            progress -= 0.5 * float(spec['slowdown_rate']) * self.nominal_ego_progress_speed() * min(elapsed, 5.0) ** 2
        if spec.get('wrap_progress', False):
            progress = progress % 1.0
        else:
            progress = float(np.clip(progress, 0.0, 1.0))
        offset = self.lateral_offset(spec, elapsed)
        return self.make_obstacle(
            progress,
            offset,
            float(spec.get('length_px', 92.0)),
            float(spec.get('width_px', 54.0)),
            kind='dynamic',
            index=int(spec.get('index', 0)),
        )

    def lateral_offset(self, spec, elapsed):
        offset = float(spec.get('base_offset_px', 0.0))
        if spec.get('lateral_target_px') is not None:
            target = float(spec['lateral_target_px'])
            delay = float(spec.get('lateral_start_delay_s', 0.0))
            duration = max(0.1, float(spec.get('lateral_target_duration_s', 4.0)))
            if elapsed > delay:
                blend = np.clip((elapsed - delay) / duration, 0.0, 1.0)
                blend = 0.5 - 0.5 * math.cos(math.pi * blend)
                offset = (1.0 - blend) * offset + blend * target
        if spec.get('lateral_wave_amp'):
            amplitude = float(spec['lateral_wave_amp'])
            period = max(0.1, float(spec.get('lateral_wave_period_s', 5.0)))
            phase = float(spec.get('phase_s', 0.0))
            offset += amplitude * math.sin((2.0 * math.pi * elapsed / period) + phase)
        max_offset = max(0.0, self.road.half_width_px - 0.5 * float(spec.get('width_px', 54.0)))
        return float(np.clip(offset, -max_offset, max_offset))

    def make_obstacle(self, progress, lateral_offset_px, length_px, width_px, kind='dynamic', index=None):
        center = self.road.point_at_progress(progress, lateral_offset_px)
        half_length = float(length_px) * 0.5
        half_width = float(width_px) * 0.5
        polygon = np.array(
            [
                [center[0] - half_length, center[1] - half_width],
                [center[0] + half_length, center[1] - half_width],
                [center[0] + half_length, center[1] + half_width],
                [center[0] - half_length, center[1] + half_width],
            ],
            dtype=np.float32,
        )
        obstacle = {
            'center': center,
            'polygon': polygon,
            'progress': float(progress),
            'lateral_offset_px': float(lateral_offset_px),
            'length_px': float(length_px),
            'width_px': float(width_px),
            'kind': kind,
            'heading': 0.0,
        }
        if index is not None:
            obstacle['dynamic_index'] = int(index)
        return obstacle


class DriftStylePreview:
    """Matplotlib preview styled like safe_control drift_car/test_drift.py."""

    def __init__(self, simulation):
        import matplotlib.pyplot as plt
        from safe_control.envs.drifting_env import DriftingEnv

        self.simulation = simulation
        self.config = simulation.config
        self.plt = plt
        self.track_length_m = 300.0
        self.track_width_m = 20.0
        self.num_lanes = 5
        self.window_size = (60, 30)
        self.env = DriftingEnv(
            track_type='straight',
            track_width=self.track_width_m,
            track_length=self.track_length_m,
            num_lanes=self.num_lanes,
        )
        self.x_min_px = self.simulation.road.start_x
        self.x_max_px = self.simulation.road.end_x
        self.y_center_px = self.simulation.road.center_y
        self.y_scale = self.track_width_m / max(1.0, 2.0 * float(self.config.road_half_width_px))
        self.trajectory = []
        self.vehicle_patches = []
        self.obstacle_patches = []

        plt.ion()
        self.ax, self.fig = self.env.setup_plot()
        self.fig.canvas.manager.set_window_title('Standalone HIL Virtual Vehicle')
        self.setup_lines()

    def setup_lines(self):
        self.trajectory_line, = self.ax.plot([], [], 'b-', linewidth=2, alpha=0.7, zorder=5)
        self.cg_marker, = self.ax.plot([], [], 'ko', markersize=5, zorder=12)
        self.status_text = self.ax.text(
            0.015,
            0.97,
            '',
            transform=self.ax.transAxes,
            fontsize=9,
            va='top',
            bbox={'facecolor': 'white', 'alpha': 0.75, 'edgecolor': 'none'},
            zorder=100,
        )

    def pixel_to_world(self, point):
        point = np.asarray(point, dtype=np.float64)
        x = (float(point[0]) - self.x_min_px) / max(1.0, self.x_max_px - self.x_min_px)
        x = np.clip(x, 0.0, 1.0) * self.track_length_m
        y = (self.y_center_px - float(point[1])) * self.y_scale
        return np.array([x, y], dtype=np.float64)

    def heading_to_world(self, heading):
        return -float(heading)

    def speed_to_world(self, speed_pps):
        px_per_m = max(1e-6, (self.x_max_px - self.x_min_px) / self.track_length_m)
        return float(speed_pps) / px_per_m

    def obstacle_to_world(self, obstacle):
        center = self.pixel_to_world(obstacle['center'])
        x_scale = self.track_length_m / max(1.0, self.x_max_px - self.x_min_px)
        return {
            'x': float(center[0]),
            'y': float(center[1]),
            'theta': self.heading_to_world(float(obstacle.get('heading', 0.0))),
            'spec': {
                'a': 1.4,
                'b': 1.4,
                'body_length': max(2.8, float(obstacle.get('length_px', 90.0)) * x_scale),
                'body_width': max(1.2, float(obstacle.get('width_px', 54.0)) * self.y_scale),
                'radius': max(0.8, 0.5 * float(obstacle.get('width_px', 54.0)) * self.y_scale),
            },
        }

    def clear_patches(self, patches):
        for patch in patches:
            patch.remove()
        patches.clear()

    def update(self, result):
        self.clear_patches(self.vehicle_patches)
        self.clear_patches(self.obstacle_patches)

        ego = self.pixel_to_world(self.simulation.state.center)
        heading = self.heading_to_world(self.simulation.state.heading)
        self.trajectory.append(ego)
        if len(self.trajectory) > 400:
            self.trajectory.pop(0)

        for obstacle in self.simulation.visible_obstacles():
            patches = self.env._create_obstacle_car_patches(self.obstacle_to_world(obstacle), body_color=(0.7, 0.2, 0.2), zorder=8)
            for patch in patches:
                self.ax.add_patch(patch)
            self.obstacle_patches.extend(patches)

        self.vehicle_patches.extend(
            self.env._create_obstacle_car_patches(
                {
                    'x': float(ego[0]),
                    'y': float(ego[1]),
                    'theta': float(heading),
                    'spec': {'a': 1.4, 'b': 1.4, 'body_length': 4.5, 'body_width': 2.0, 'radius': 1.2},
                },
                body_color=(0.1, 0.35, 0.95),
                zorder=10,
            )
        )
        for patch in self.vehicle_patches:
            self.ax.add_patch(patch)

        self.cg_marker.set_data([ego[0]], [ego[1]])
        trajectory = np.asarray(self.trajectory)
        if len(trajectory) > 1:
            self.trajectory_line.set_data(trajectory[:, 0], trajectory[:, 1])
        self.status_text.set_text(
            f"scenario: {getattr(self.config, 'traffic_scenario', 'free_flow')}\n"
            f"t={self.simulation.now:.1f}s  V={self.speed_to_world(self.simulation.state.speed):.2f} m/s\n"
            f"cte={result.lateral_error_px:.1f}px  clearance={result.safety_clearance_px:.1f}px\n"
            "mode=open-loop visualization"
        )
        self.env.update_plot_frame(self.ax, ego, window_size=self.window_size)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        self.plt.pause(0.001)

    def close(self):
        self.plt.ioff()
        self.plt.close(self.fig)


class VirtualVehicleSimulation:
    """Standalone visual simulation."""

    def __init__(self, config):
        self.config = config
        self.stats = RuntimeStats()
        self.road = RoadModel(config)
        self.track_points = self.road.track_points
        self.road_boundaries = self.road.road_boundaries
        self.state = self.setup_vehicle()
        self.dynamic_scenarios = self.setup_dynamic_scenarios()
        self.latest_result = None
        self.now = 0.0

    def setup_vehicle(self):
        center = self.road.point_at_progress(
            float(self.config.virtual_start_progress),
            float(self.config.virtual_start_lateral_offset_px),
        )
        return VehicleState(
            x=float(center[0]),
            y=float(center[1]),
            heading=math.radians(float(self.config.virtual_start_heading_deg)),
            speed=max(0.0, float(self.config.virtual_start_speed_pps)),
        )

    def setup_dynamic_scenarios(self):
        scenarios = DynamicScenarioLibrary(self.road, self.config)
        scenarios.reset(0.0, self.state.center)
        return scenarios

    def run(self):
        if self.config.preview:
            return self.run_preview()
        return self.run_headless()

    def run_headless(self):
        steps = int(max(1, round(float(self.config.duration_s) / float(self.config.dt))))
        for step_index in range(steps):
            result = self.step(float(self.config.dt))
            if self.should_print(step_index):
                self.print_status(step_index, result)
            if self.stats.stopped_early:
                break
        self.print_summary()
        return self.latest_result

    def run_preview(self):
        preview = DriftStylePreview(self)
        steps = int(max(1, round(float(self.config.duration_s) / float(self.config.dt))))
        try:
            for step_index in range(steps):
                result = self.step(float(self.config.dt))
                preview.update(result)
                if self.should_print(step_index):
                    self.print_status(step_index, result)
                if self.stats.stopped_early:
                    break
            self.print_summary()
            return self.latest_result
        finally:
            preview.close()

    def step(self, dt):
        self.dynamic_scenarios.step(self.now)
        self.advance_vehicle(dt)
        self.now += dt
        self.latest_result = self.measure_state()
        self.update_stats(self.latest_result)
        return self.latest_result

    def advance_vehicle(self, dt):
        target_speed = max(0.0, float(self.config.target_track_speed_pps))
        speed_response = max(0.0, float(self.config.virtual_speed_response))
        accel = np.clip(
            (target_speed - self.state.speed) * speed_response,
            -float(self.config.virtual_max_brake_pps2),
            float(self.config.virtual_max_accel_pps2),
        )
        self.state.speed = max(0.0, self.state.speed + float(accel) * dt)

        heading_target = 0.0
        heading_error = wrap_angle(heading_target - self.state.heading)
        self.state.heading = wrap_angle(
            self.state.heading + heading_error * float(self.config.virtual_heading_response) * dt
        )
        self.state.x += math.cos(self.state.heading) * self.state.speed * dt
        self.state.y += math.sin(self.state.heading) * self.state.speed * dt
        if self.state.x > self.road.end_x:
            if getattr(self.config, 'virtual_unlimited_path', False):
                self.state.x = self.road.start_x
            else:
                self.stats.stopped_early = True
                self.stats.stop_reason = 'road_end'

    def measure_state(self):
        center = self.state.center
        return SimResult(
            lateral_error_px=self.road.lateral_error(center),
            heading_error_rad=wrap_angle(-self.state.heading),
            safety_clearance_px=self.nearest_clearance(center),
        )

    def nearest_clearance(self, center):
        clearances = []
        for obstacle in self.visible_obstacles():
            polygon = np.asarray(obstacle['polygon'], dtype=np.float32)
            min_xy = np.min(polygon, axis=0)
            max_xy = np.max(polygon, axis=0)
            dx = max(float(min_xy[0] - center[0]), 0.0, float(center[0] - max_xy[0]))
            dy = max(float(min_xy[1] - center[1]), 0.0, float(center[1] - max_xy[1]))
            outside = math.hypot(dx, dy)
            inside = min(max_xy[0] - center[0], center[0] - min_xy[0], max_xy[1] - center[1], center[1] - min_xy[1])
            clearances.append(-float(inside) if inside >= 0 else outside)
        return min(clearances) if clearances else float('inf')

    def visible_obstacles(self):
        obstacles = list(self.dynamic_scenarios.static_scene_obstacles)
        obstacles.extend(self.dynamic_scenarios.dynamic_obstacles)
        return obstacles

    def update_stats(self, result):
        self.stats.steps += 1
        self.stats.min_clearance_px = min(self.stats.min_clearance_px, float(result.safety_clearance_px))
        self.stats.max_abs_lateral_error_px = max(self.stats.max_abs_lateral_error_px, abs(float(result.lateral_error_px)))
        self.stats.max_speed_pps = max(self.stats.max_speed_pps, float(self.state.speed))

    def should_print(self, step_index):
        return int(self.config.print_every) > 0 and step_index % int(self.config.print_every) == 0

    def print_status(self, step_index, result):
        print(
            f'{step_index:04d} t={self.now:5.2f}s '
            f'pos=({self.state.x:6.1f},{self.state.y:6.1f}) '
            f'v={self.state.speed:5.1f} '
            f'cte={result.lateral_error_px:7.2f} '
            f'heading={result.heading_error_rad:6.3f} '
            f'clearance={result.safety_clearance_px:7.2f}'
        )

    def print_summary(self):
        status = self.stats.stop_reason if self.stats.stopped_early else 'completed'
        print(
            f'summary: steps={self.stats.steps} '
            f'max_speed={self.stats.max_speed_pps:.1f}pps '
            f'min_clearance={self.stats.min_clearance_px:.1f}px '
            f'max_abs_cte={self.stats.max_abs_lateral_error_px:.1f}px '
            f'status={status}'
        )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Run a standalone dynamic-straight virtual vehicle visualization.',
    )
    parser.add_argument('--duration-s', type=float, default=20.0)
    parser.add_argument('--dt', type=float, default=0.05)
    parser.add_argument('--preview', action='store_true')
    parser.add_argument('--print-every', type=int, default=20)
    parser.add_argument('--virtual-width', type=int, default=960)
    parser.add_argument('--virtual-height', type=int, default=540)
    parser.add_argument('--track-center-y', type=float, default=270.0)
    parser.add_argument('--road-half-width-px', type=float, default=250.0)
    parser.add_argument('--virtual-start-progress', type=float, default=0.04)
    parser.add_argument('--virtual-start-lateral-offset-px', type=float, default=70.0)
    parser.add_argument('--virtual-start-heading-deg', type=float, default=0.0)
    parser.add_argument('--virtual-start-speed-pps', type=float, default=0.0)
    parser.add_argument('--virtual-max-accel-pps2', type=float, default=120.0)
    parser.add_argument('--virtual-max-brake-pps2', type=float, default=220.0)
    parser.add_argument('--virtual-speed-response', type=float, default=2.0)
    parser.add_argument('--virtual-heading-response', type=float, default=2.4)
    parser.add_argument('--virtual-unlimited-path', action='store_true')
    parser.add_argument('--target-track-speed-pps', type=float, default=38.0)
    parser.add_argument(
        '--traffic-scenario',
        choices=('free_flow', 'lead_slowdown', 'cut_in', 'dense_flow', 'lane_weave'),
        default='free_flow',
    )
    parser.add_argument('--dynamic-obstacle-count', '--dynamic-count', dest='dynamic_obstacle_count', type=int, default=6)
    parser.add_argument('--dynamic-obstacle-seed', type=int, default=13)
    parser.add_argument('--dynamic-obstacle-min-gap-progress', type=float, default=0.08)
    parser.add_argument('--no-static-obstacles', action='store_true')
    config, unknown = parser.parse_known_args(argv)
    if unknown:
        print(f'ignoring unsupported args: {" ".join(unknown)}')
    return config


def wrap_angle(angle):
    """Wrap radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def main(argv=None):
    config = parse_args(argv)
    simulation = VirtualVehicleSimulation(config)
    simulation.run()


if __name__ == '__main__':
    main()
