"""ROS node for the straight-road ArUco follower with dynamic obstacles."""

import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from ..config.dynamic_straight import ARMING_SERVICE
from ..config.dynamic_straight import NEUTRAL_VALUE
from ..config.dynamic_straight import parse_args
from ..config.dynamic_straight import print_config
from ..config.dynamic_straight import SETTLE_DURATION
from ..config.dynamic_straight import UPDATE_RATE_HZ
from ..config.dynamic_straight import validate_config
from ..control import DynamicStraightController
from ..road import make_laneless_static_obstacle
from .straight_static_node import ArucoTrackFollower as StraightArucoTrackFollower


FREE_FLOW_DYNAMIC_DIRECTION = 1.0
FREE_FLOW_DYNAMIC_SPEED_FACTORS = (0.30, 0.34)


class ArucoTrackFollower(StraightArucoTrackFollower):
    """Straight-road follower that moves virtual obstacles over time."""

    def __init__(self, config):
        self.dynamic_obstacles = []
        self.detected_dynamic_obstacles = []
        self.dynamic_obstacle_specs = []
        self.dynamic_obstacle_previous_states = {}
        self.dynamic_obstacle_smoothed_states = {}
        self.retired_dynamic_obstacle_indices = set()
        self.dynamic_obstacle_start_time = None
        config.dynamic_obstacles = True
        if getattr(config, 'traffic_scenario', 'free_flow') == 'head_on':
            config.include_road_boundary_walls = False
        if not config.static_obstacles:
            config.static_obstacles = '[]'
        super().__init__(config)

    def make_controller(self, config):
        """Create the dynamic straight-road controller core."""
        return DynamicStraightController(config)

    def reset_virtual_vehicle(self):
        """Reset the synthetic vehicle and dynamic obstacle field."""
        super().reset_virtual_vehicle()
        self.remember_static_scene_obstacles()
        self.reset_dynamic_obstacles(time.monotonic())

    def image_callback(self, image_msg):
        """Process one real camera frame with the current dynamic obstacle map."""
        if self.config.virtual_vehicle_test:
            return
        if self.message_is_stale(image_msg):
            return
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]
        now = time.monotonic()
        image_size = (width, height)
        if self.last_image_size != image_size:
            self.controller.build_track_scene(width, height)
            self.remember_static_scene_obstacles()
            self.reset_dynamic_obstacles(now)
        detection = self.detect_marker(frame)
        if detection is None:
            self.stop_immediately()
            self.publish_tuning_telemetry(marker_seen=False)
            if self.config.preview:
                self.schedule_preview(frame, None, None, None, None)
            return

        center, heading, corners = detection
        self.update_marker_scale(corners)
        self.update_pid_from_panel()
        self.update_dynamic_obstacles(now, center, heading)
        result = self.controller.process_detection(
            center,
            heading,
            image_size=image_size,
            now=now,
        )
        self.publish_tuning_telemetry(result, marker_seen=True)
        if result.target_lap_reached:
            self.request_stop(
                f'target laps completed ({self.metrics.laps_completed}/'
                f'{self.target_laps})'
            )
            return

        if self.output_enabled and not self.config.dry_run:
            command = self.drive_message()
            self.command_pub.publish(command)

        if self.config.preview:
            self.schedule_preview(frame, corners, center, result.target, result.tangent)

    def virtual_vehicle_tick(self):
        """Run one controller/simulation step with moving obstacles."""
        if self.virtual_vehicle_state is None:
            self.reset_virtual_vehicle()
        now = time.monotonic()
        dt = min(0.1, max(1e-3, now - self.virtual_last_time))
        self.virtual_last_time = now

        self.update_pid_from_panel()
        state = self.virtual_vehicle_state
        center = np.array([state['x'], state['y']], dtype=np.float32)
        if self.config.random_static_obstacles:
            self.update_detected_random_obstacles(center)
        self.update_dynamic_obstacles(now, center, state['heading'])
        result = self.controller.process_detection(
            center,
            state['heading'],
            image_size=(self.config.virtual_width, self.config.virtual_height),
            now=now,
        )
        self.publish_tuning_telemetry(result, marker_seen=True)
        previous_center = center.copy()
        self.advance_virtual_vehicle(dt)
        preview_target = result.target
        if self.config.virtual_unlimited_path:
            scene_delta = self.advance_virtual_road_window(previous_center)
            preview_target = result.target + scene_delta
        elif self.config.virtual_stop_at_end and self.virtual_reached_road_end():
            self.request_stop('virtual vehicle reached road end')
        if self.config.preview:
            frame = self.make_virtual_frame()
            preview_center = np.array(
                [
                    self.virtual_vehicle_state['x'],
                    self.virtual_vehicle_state['y'],
                ],
                dtype=np.float32,
            )
            self.schedule_preview(
                frame,
                None,
                preview_center,
                preview_target,
                result.tangent,
            )

    def remember_static_scene_obstacles(self):
        """Keep CLI/static scene obstacles so dynamic obstacles can be added to them."""
        self.static_scene_obstacles = list(self.controller.static_obstacles)

    def reset_dynamic_obstacles(self, now):
        """Create repeatable moving obstacle specs for the current road scene."""
        self.dynamic_obstacle_start_time = now
        self.dynamic_obstacle_specs = []
        self.dynamic_obstacle_previous_states = {}
        self.dynamic_obstacle_smoothed_states = {}
        self.dynamic_obstacles = []
        self.detected_dynamic_obstacles = []
        self.retired_dynamic_obstacle_indices = set()
        self.dynamic_obstacles_need_ego_placement = True
        if self.track_points is None:
            self.sync_controller_obstacles()
            return

        scenario = getattr(self.config, 'traffic_scenario', 'free_flow')
        rng = np.random.default_rng(self.config.dynamic_obstacle_seed)
        if scenario != 'free_flow':
            self.dynamic_obstacle_specs = self.make_scenario_specs(scenario, rng)
        else:
            if self.config.dynamic_obstacle_count <= 0:
                self.sync_controller_obstacles()
                return
            self.dynamic_obstacle_specs = self.make_free_flow_specs(rng)

        self.place_dynamic_specs_ahead_of_ego()
        self.update_dynamic_obstacles(now)

    def place_dynamic_specs_ahead_of_ego(self, center=None):
        """Place newly-created dynamic specs ahead of ego without runtime reloads."""
        if not self.dynamic_obstacle_specs:
            return
        if center is None:
            if self.virtual_vehicle_state is None:
                return
            center = np.array(
                [
                    self.virtual_vehicle_state['x'],
                    self.virtual_vehicle_state['y'],
                ],
                dtype=np.float32,
            )
        else:
            center = np.asarray(center, dtype=np.float32)
        ego_progress = self.vehicle_progress(center)
        if ego_progress is None:
            return
        self.dynamic_obstacles_need_ego_placement = False
        min_start_progress = ego_progress + self.dynamic_obstacle_ahead_gap()
        earliest_progress = min(
            float(spec.get('base_progress', 0.0))
            for spec in self.dynamic_obstacle_specs
        )
        shift = max(0.0, min_start_progress - earliest_progress)
        if shift <= 0.0:
            return
        for spec in self.dynamic_obstacle_specs:
            spec['base_progress'] = float(spec.get('base_progress', 0.0)) + shift

    def make_scenario_specs(self, scenario, rng):
        """Build a reproducible traffic pattern for the selected scenario."""
        road_half_width = float(self.config.road_half_width_px)
        default_length = 96.0
        default_width = 58.0
        next_index = {'value': 0}

        def spec(
            base_progress,
            base_offset_px,
            progress_speed_pps,
            length_px=default_length,
            width_px=default_width,
            kind='dynamic',
            **extra,
        ):
            index = next_index['value']
            next_index['value'] += 1
            payload = {
                'index': index,
                'base_progress': float(base_progress),
                'base_offset_px': float(base_offset_px),
                'progress_speed_pps': float(progress_speed_pps),
                'length_px': float(length_px),
                'width_px': float(width_px),
                'kind': kind,
            }
            payload.update(extra)
            return payload

        offset_left = -road_half_width * 0.28
        offset_right = road_half_width * 0.28
        offset_center = 0.0
        ego_progress_speed = self.nominal_ego_progress_speed()

        if scenario == 'lead_slowdown':
            return [
                spec(0.28, offset_center, ego_progress_speed * 0.10, slowdown_rate=0.75),
                spec(0.62, offset_right * 0.45, ego_progress_speed * 0.08, length_px=78.0, width_px=50.0),
            ]
        if scenario == 'cut_in':
            return [
                spec(
                    0.34,
                    offset_left,
                    ego_progress_speed * 0.10,
                    lateral_target_px=offset_center,
                    lateral_start_delay_s=2.0,
                    lateral_target_duration_s=3.4,
                    hard_spacing=True,
                    hard_min_gap_progress=0.22,
                ),
                spec(
                    0.08,
                    offset_right,
                    ego_progress_speed * 0.08,
                    length_px=82.0,
                    width_px=52.0,
                    lateral_target_px=offset_center,
                    lateral_start_delay_s=6.0,
                    lateral_target_duration_s=3.4,
                    hard_spacing=True,
                    hard_min_gap_progress=0.22,
                ),
            ]
        if scenario == 'overtake_merge':
            return [
                spec(0.22, offset_center, ego_progress_speed * 0.10, length_px=104.0, width_px=62.0),
                spec(0.12, offset_left, ego_progress_speed * 0.08, lateral_target_px=offset_center, lateral_target_duration_s=4.5, progress_wave_amp=12.0, progress_wave_period_s=7.0),
            ]
        if scenario == 'stop_go_platoon':
            return [
                spec(0.24, offset_center, ego_progress_speed * 0.10, stop_go_period_s=5.5, stop_go_duty=0.45, phase_s=0.0),
                spec(0.33, offset_center, ego_progress_speed * 0.08, stop_go_period_s=5.5, stop_go_duty=0.45, phase_s=1.5),
                spec(0.42, offset_center, ego_progress_speed * 0.10, stop_go_period_s=5.5, stop_go_duty=0.45, phase_s=3.0),
                spec(0.51, offset_right * 0.18, ego_progress_speed * 0.08, stop_go_period_s=5.5, stop_go_duty=0.45, phase_s=4.5, width_px=50.0),
            ]
        if scenario == 'dense_flow':
            return [
                spec(0.18, offset_left * 0.75, ego_progress_speed * 0.10, length_px=84.0, width_px=52.0),
                spec(0.28, offset_center, ego_progress_speed * 0.08, length_px=92.0, width_px=58.0),
                spec(0.38, offset_right * 0.80, ego_progress_speed * 0.10, length_px=88.0, width_px=54.0),
                spec(0.49, offset_left * 0.25, ego_progress_speed * 0.08, length_px=96.0, width_px=56.0),
                spec(0.60, offset_right * 0.30, ego_progress_speed * 0.10, length_px=90.0, width_px=55.0),
            ]
        if scenario == 'bottleneck_merge':
            return [
                spec(0.18, offset_left, ego_progress_speed * 0.10, lateral_target_px=offset_center, lateral_target_duration_s=5.0),
                spec(0.22, offset_right, ego_progress_speed * 0.08, lateral_target_px=offset_center, lateral_target_duration_s=4.2),
                spec(0.34, offset_left * 0.55, ego_progress_speed * 0.10, lateral_target_px=offset_center, lateral_target_duration_s=3.5),
                spec(0.40, offset_right * 0.55, ego_progress_speed * 0.08, lateral_target_px=offset_center, lateral_target_duration_s=3.0),
            ]
        if scenario == 'lane_weave':
            return [
                spec(0.16, offset_left * 0.65, ego_progress_speed * 0.10, lateral_wave_amp=road_half_width * 0.14, lateral_wave_period_s=4.8, phase_s=0.0),
                spec(0.32, offset_center, ego_progress_speed * 0.08, lateral_wave_amp=road_half_width * 0.16, lateral_wave_period_s=5.4, phase_s=1.8),
                spec(0.49, offset_right * 0.60, ego_progress_speed * 0.10, lateral_wave_amp=road_half_width * 0.18, lateral_wave_period_s=6.0, phase_s=3.6),
            ]
        if scenario == 'crossing_conflict':
            return [
                spec(0.50, -road_half_width * 0.52, ego_progress_speed * 0.00, lateral_speed_px=road_half_width * 0.22, respect_vehicle_gap=False, length_px=70.0, width_px=46.0),
                spec(0.62, road_half_width * 0.48, ego_progress_speed * 0.08, length_px=82.0, width_px=50.0),
            ]
        if scenario == 'signal_phase':
            return [
                spec(0.36, offset_center, ego_progress_speed * 0.10, stop_go_period_s=6.0, stop_go_duty=0.35, phase_s=0.0, hold_point_progress=0.42),
                spec(0.55, offset_center, ego_progress_speed * 0.08, stop_go_period_s=6.0, stop_go_duty=0.35, phase_s=2.8, hold_point_progress=0.58),
            ]
        if scenario == 'looping_flow':
            return [
                spec(0.20, offset_left * 0.40, ego_progress_speed * 0.10, wrap_progress=True, length_px=88.0, width_px=52.0),
                spec(0.45, offset_center, ego_progress_speed * 0.08, wrap_progress=True, length_px=94.0, width_px=56.0),
                spec(0.70, offset_right * 0.42, ego_progress_speed * 0.10, wrap_progress=True, length_px=90.0, width_px=54.0),
            ]
        if scenario == 'three_sparse':
            return [
                spec(0.20, offset_left, ego_progress_speed * 0.10, wrap_progress=True, length_px=90.0, width_px=54.0),
                spec(0.50, offset_center, ego_progress_speed * 0.08, wrap_progress=True, length_px=90.0, width_px=54.0),
                spec(0.80, offset_right, ego_progress_speed * 0.10, wrap_progress=True, length_px=90.0, width_px=54.0),
            ]
        if scenario == 'head_on':
            return [
                spec(0.50, offset_center, 0.0, length_px=90.0, width_px=54.0),
            ]
        if scenario == 'endless_walls':
            return [
                spec(0.15, offset_right, ego_progress_speed * 0.10, wrap_progress=True, length_px=90.0, width_px=54.0),
                spec(0.35, offset_left, 0.0, wrap_progress=True, length_px=30.0, width_px=150.0),
                spec(0.55, offset_left, ego_progress_speed * 0.08, wrap_progress=True, length_px=90.0, width_px=54.0),
                spec(0.80, offset_right, 0.0, wrap_progress=True, length_px=30.0, width_px=150.0),
            ]

        return self.make_free_flow_specs(rng)

    def make_free_flow_specs(self, rng):
        """Create the original random moving obstacles used by free-flow mode."""
        min_progress = float(self.config.dynamic_obstacle_min_progress)
        max_progress = float(self.config.dynamic_obstacle_max_progress)
        road_half_width = float(self.config.road_half_width_px)
        ego_progress_speed = self.nominal_ego_progress_speed()
        specs = []
        # Double the dynamic obstacle count for free_flow
        count = self.config.dynamic_obstacle_count * 2
        for index in range(count):
            speed_factor = FREE_FLOW_DYNAMIC_SPEED_FACTORS[
                index % len(FREE_FLOW_DYNAMIC_SPEED_FACTORS)
            ]
            progress_speed_pps = (
                FREE_FLOW_DYNAMIC_DIRECTION
                * ego_progress_speed
                * speed_factor
            )
            specs.append(
                {
                    'index': index,
                    'base_progress': float(rng.uniform(min_progress, max_progress)),
                    'base_offset_px': float(rng.uniform(-road_half_width * 0.35, road_half_width * 0.35)),
                    'length_px': float(rng.uniform(70.0, 125.0)),
                    'width_px': float(rng.uniform(45.0, 76.0)),
                    'progress_speed_pps': progress_speed_pps,
                    'kind': 'dynamic',
                }
            )
        return specs

    def update_dynamic_obstacles(self, now, vehicle_center=None, vehicle_heading=None):
        """Update moving obstacle polygons and publish them to the controller."""
        if self.track_points is None:
            return
        if self.dynamic_obstacle_start_time is None:
            self.reset_dynamic_obstacles(now)
            return
        if (
            self.dynamic_obstacles_need_ego_placement
            and vehicle_center is not None
        ):
            self.place_dynamic_specs_ahead_of_ego(vehicle_center)
            self.dynamic_obstacle_start_time = now
            self.dynamic_obstacle_previous_states = {}
            self.dynamic_obstacle_smoothed_states = {}
            self.retired_dynamic_obstacle_indices = set()
            self.dynamic_obstacles_need_ego_placement = False

        elapsed = max(0.0, now - self.dynamic_obstacle_start_time)
        min_progress = float(self.config.dynamic_obstacle_min_progress)
        max_progress = float(self.config.dynamic_obstacle_max_progress)
        unlimited_path = (
            self.config.virtual_vehicle_test
            and self.config.virtual_unlimited_path
        )
        scene_track_points = self.dynamic_scene_track_points()
        candidates = []
        repeat_obstacles = self.dynamic_scenario_repeats_obstacles()
        for spec in self.dynamic_obstacle_specs:
            if spec['index'] in self.retired_dynamic_obstacle_indices:
                continue
            progress = self.evaluate_dynamic_progress(
                spec,
                elapsed,
                min_progress,
                max_progress,
                continuous=unlimited_path or not repeat_obstacles,
            )
            if unlimited_path and repeat_obstacles:
                ego_local_progress = 0.0
                if vehicle_center is not None:
                    local_p = self.vehicle_progress(vehicle_center)
                    if local_p is not None:
                        ego_local_progress = local_p
                else:
                    if self.virtual_vehicle_state is not None:
                        vc = np.array([self.virtual_vehicle_state['x'], self.virtual_vehicle_state['y']], dtype=np.float32)
                        local_p = self.vehicle_progress(vc)
                        if local_p is not None:
                            ego_local_progress = local_p
                ego_cont = float(getattr(self, 'virtual_road_tile_index', 0)) + ego_local_progress
                progress = ego_cont - 0.15 + ((progress - (ego_cont - 0.15)) % 1.0)
            if (
                not repeat_obstacles
                and (progress < min_progress or progress > max_progress)
            ):
                self.retired_dynamic_obstacle_indices.add(spec['index'])
                continue
            lateral_offset = self.evaluate_dynamic_lateral_offset(spec, elapsed)
            hard_min_gap = spec.get('hard_min_gap_progress')
            if not unlimited_path:
                progress = min(max(progress, min_progress), max_progress)
            candidates.append(
                {
                    'spec': spec,
                    'progress': progress,
                    'lateral_offset_px': lateral_offset,
                    'length_px': float(spec['length_px']),
                    'width_px': float(spec['width_px']),
                    'hard_spacing': bool(spec.get('hard_spacing', False)),
                    'hard_min_gap_progress': (
                        None if hard_min_gap is None else float(hard_min_gap)
                    ),
                }
            )

        if unlimited_path:
            candidates = self.resolve_continuous_dynamic_obstacle_layout(
                candidates,
                min_progress,
            )
        else:
            candidates = self.resolve_dynamic_obstacle_layout(
                candidates,
                min_progress,
                max_progress,
            )
        candidates = [
            self.smooth_dynamic_candidate(candidate, now)
            for candidate in candidates
        ]
        obstacles = []
        for candidate in candidates:
            spec = candidate['spec']
            progress = candidate['progress']
            tile_offset = np.zeros(2, dtype=np.float32)
            if unlimited_path:
                progress, tile_offset = self.unlimited_progress_to_tile(progress)
            obstacle = make_laneless_static_obstacle(
                {
                    'progress': progress,
                    'length_px': candidate['length_px'],
                    'width_px': candidate['width_px'],
                    'lateral_offset_px': candidate['lateral_offset_px'],
                },
                scene_track_points,
                self.road_tangents,
                self.road_normals,
                self.config,
            )
            if unlimited_path:
                obstacle['center'] = obstacle['center'] + tile_offset
                obstacle['polygon'] = obstacle['polygon'] + tile_offset
                obstacle['continuous_progress'] = candidate['progress']
            obstacle['dynamic_index'] = spec['index']
            obstacle['kind'] = spec.get('kind', 'dynamic')
            obstacle['detected'] = True
            previous = self.dynamic_obstacle_previous_states.get(spec['index'])
            if previous is not None:
                previous_center, previous_time = previous
                dt = max(1e-6, now - previous_time)
                displacement = np.linalg.norm(obstacle['center'] - previous_center)
                if displacement > 300.0:
                    obstacle['vx'] = 0.0
                    obstacle['vy'] = 0.0
                else:
                    velocity = (obstacle['center'] - previous_center) / dt
                    obstacle['vx'] = float(velocity[0])
                    obstacle['vy'] = float(velocity[1])
            else:
                obstacle['vx'] = 0.0
                obstacle['vy'] = 0.0
            self.dynamic_obstacle_previous_states[spec['index']] = (
                obstacle['center'].copy(),
                now,
            )
            obstacles.append(obstacle)
        self.dynamic_obstacles = obstacles
        self.update_detected_dynamic_obstacles(vehicle_center, vehicle_heading)
        self.sync_controller_obstacles()

    def dynamic_scene_track_points(self):
        """Return the currently displayed road geometry for obstacle placement."""
        if (
            self.config.virtual_vehicle_test
            and self.config.virtual_unlimited_path
            and getattr(self.controller, 'track_points', None) is not None
        ):
            return self.controller.track_points
        return self.track_points

    def translate_virtual_scene(self, delta):
        """Translate dynamic bookkeeping along with the scrolled virtual scene."""
        super().translate_virtual_scene(delta)
        delta = np.asarray(delta, dtype=np.float32)
        shifted_previous = {}
        for index, previous in self.dynamic_obstacle_previous_states.items():
            center, previous_time = previous
            shifted_previous[index] = (
                np.asarray(center, dtype=np.float32) + delta,
                previous_time,
            )
        self.dynamic_obstacle_previous_states = shifted_previous

    def smooth_dynamic_candidate(self, candidate, now):
        """Low-pass traffic progress/lateral states for smoother simulation."""
        spec_index = candidate['spec']['index']
        previous = self.dynamic_obstacle_smoothed_states.get(spec_index)
        if previous is None or abs(float(candidate['progress']) - float(previous['progress'])) > 0.5:
            self.dynamic_obstacle_smoothed_states[spec_index] = {
                'progress': float(candidate['progress']),
                'lateral_offset_px': float(candidate['lateral_offset_px']),
                'time': now,
            }
            return candidate

        dt = min(1.0 / 20.0, max(1e-3, now - float(previous['time'])))
        alpha = 1.0 - math.exp(-dt / 0.16)
        smoothed = dict(candidate)
        smoothed['progress'] = (
            (1.0 - alpha) * float(previous['progress'])
            + alpha * float(candidate['progress'])
        )
        smoothed['lateral_offset_px'] = (
            (1.0 - alpha) * float(previous['lateral_offset_px'])
            + alpha * float(candidate['lateral_offset_px'])
        )
        self.dynamic_obstacle_smoothed_states[spec_index] = {
            'progress': float(smoothed['progress']),
            'lateral_offset_px': float(smoothed['lateral_offset_px']),
            'time': now,
        }
        return smoothed

    def update_detected_dynamic_obstacles(self, center, heading):
        for obstacle in self.dynamic_obstacles:
            obstacle['detected'] = True
        self.detected_dynamic_obstacles = list(self.dynamic_obstacles)

    def dynamic_scenario_repeats_obstacles(self):
        """Return whether this scenario intentionally recycles traffic vehicles."""
        scenario = getattr(self.config, 'traffic_scenario', 'free_flow')
        return scenario in ('looping_flow', 'three_sparse', 'endless_walls')

    def resolve_dynamic_obstacle_layout(self, candidates, min_progress, max_progress):
        """Pack dynamic obstacle rectangles so their road-frame boxes do not overlap."""
        if not candidates:
            return []
        path_length_px = self.road_path_length_px()
        if path_length_px <= 1e-6:
            return candidates

        packed = sorted(
            (dict(candidate) for candidate in candidates),
            key=lambda item: (item['progress'], item['lateral_offset_px']),
        )
        fixed_candidates = self.fixed_obstacle_layout_candidates()
        self.forward_pack_obstacles(
            packed,
            fixed_candidates,
            min_progress,
            max_progress,
            path_length_px,
        )

        overflow = max(0.0, max(item['progress'] for item in packed) - max_progress)
        if overflow > 0.0:
            for item in packed:
                item['progress'] = max(min_progress, item['progress'] - overflow)
            self.forward_pack_obstacles(
                packed,
                fixed_candidates,
                min_progress,
                max_progress,
                path_length_px,
            )

        accepted = []
        for item in packed:
            if item['progress'] > max_progress:
                continue
            if self.candidate_overlaps_any(
                item,
                fixed_candidates + accepted,
                path_length_px,
            ):
                continue
            accepted.append(item)
        return accepted

    def resolve_continuous_dynamic_obstacle_layout(self, candidates, min_progress):
        """Pack dynamic traffic on an endless road without resetting progress."""
        if not candidates:
            return []
        path_length_px = self.road_path_length_px()
        if path_length_px <= 1e-6:
            return candidates

        packed = sorted(
            (dict(candidate) for candidate in candidates),
            key=lambda item: (item['progress'], item['lateral_offset_px']),
        )
        placed = self.continuous_fixed_obstacle_layout_candidates(min_progress)
        for candidate in packed:
            candidate['progress'] = max(float(candidate['progress']), min_progress)
            for previous in placed:
                if not self.candidates_need_spacing(candidate, previous):
                    continue
                min_delta = self.required_progress_delta(
                    candidate,
                    previous,
                    path_length_px,
                )
                candidate['progress'] = max(
                    candidate['progress'],
                    previous['progress'] + min_delta,
                )
            placed.append(candidate)
        return packed

    def continuous_fixed_obstacle_layout_candidates(self, min_progress):
        """Return active-tile fixed obstacles for continuous traffic packing."""
        fixed = []
        for obstacle in self.fixed_obstacle_layout_candidates():
            fixed.append(
                {
                    'progress': max(float(obstacle['progress']), min_progress),
                    'lateral_offset_px': float(obstacle['lateral_offset_px']),
                    'length_px': float(obstacle['length_px']),
                    'width_px': float(obstacle['width_px']),
                }
            )
        return fixed

    def fixed_obstacle_layout_candidates(self):
        """Return non-dynamic obstacle boxes that dynamic traffic must avoid."""
        fixed = []
        if self.config.random_static_obstacles:
            source_obstacles = self.detected_random_obstacles
        else:
            source_obstacles = self.static_scene_obstacles
        for obstacle in source_obstacles:
            if str(obstacle.get('kind', '')).startswith('road_boundary_wall'):
                continue
            fixed.append(
                {
                    'progress': float(obstacle.get('progress', 0.0)),
                    'lateral_offset_px': float(obstacle.get('lateral_offset_px', 0.0)),
                    'length_px': float(obstacle['half_length']) * 2.0,
                    'width_px': float(obstacle['half_width']) * 2.0,
                }
            )
        return sorted(
            fixed,
            key=lambda item: (item['progress'], item['lateral_offset_px']),
        )

    def forward_pack_obstacles(
        self,
        candidates,
        fixed_candidates,
        min_progress,
        max_progress,
        path_length_px,
    ):
        """Shift candidates forward until all lateral-overlap pairs have a gap."""
        placed = list(fixed_candidates)
        for candidate in candidates:
            candidate['progress'] = min(
                max(float(candidate['progress']), min_progress),
                max_progress,
            )
            for previous in placed:
                if not self.candidates_need_spacing(candidate, previous):
                    continue
                min_delta = self.required_progress_delta(
                    candidate,
                    previous,
                    path_length_px,
                )
                candidate['progress'] = max(
                    candidate['progress'],
                    previous['progress'] + min_delta,
                )
            placed.append(candidate)

    def candidate_overlaps_any(self, candidate, others, path_length_px):
        """Return whether candidate overlaps any already accepted road-frame box."""
        for other in others:
            if not self.candidates_need_spacing(candidate, other):
                continue
            min_delta = self.required_progress_delta(
                candidate,
                other,
                path_length_px,
            )
            if abs(candidate['progress'] - other['progress']) < min_delta:
                return True
        return False

    def candidates_overlap_laterally(self, first, second):
        """Return whether two road-frame rectangles overlap across road width."""
        lateral_gap = float(self.config.dynamic_obstacle_min_lateral_gap_px)
        min_separation = (
            0.5 * float(first['width_px'])
            + 0.5 * float(second['width_px'])
            + lateral_gap
        )
        return (
            abs(float(first['lateral_offset_px']) - float(second['lateral_offset_px']))
            < min_separation
        )

    def candidates_need_spacing(self, first, second):
        """Return whether two road-frame rectangles need longitudinal spacing."""
        if first.get('hard_spacing') or second.get('hard_spacing'):
            return True
        return self.candidates_overlap_laterally(first, second)

    def required_progress_delta(self, first, second, path_length_px):
        """Return normalized progress separation for non-overlapping lengths."""
        longitudinal_gap = float(self.config.dynamic_obstacle_min_longitudinal_gap_px)
        separation_px = (
            0.5 * float(first['length_px'])
            + 0.5 * float(second['length_px'])
            + longitudinal_gap
        )
        progress_delta = separation_px / max(path_length_px, 1e-6)
        hard_gap = max(
            float(first.get('hard_min_gap_progress') or 0.0),
            float(second.get('hard_min_gap_progress') or 0.0),
        )
        return max(progress_delta, hard_gap)

    def road_path_length_px(self):
        """Return the centerline arc length used to convert pixels to progress."""
        if self.track_points is None or len(self.track_points) < 2:
            return 0.0
        deltas = np.diff(self.track_points.astype(np.float64), axis=0)
        return float(np.sum(np.linalg.norm(deltas, axis=1)))

    def nominal_ego_progress_speed(self):
        """Return ego target speed as normalized road progress per second."""
        path_length = self.road_path_length_px()
        if path_length <= 1e-6:
            return 0.05
        return max(0.001, float(self.config.target_track_speed_pps) / path_length)

    def dynamic_obstacle_ahead_gap(self):
        """Return minimum progress gap for loading dynamic objects ahead of ego."""
        configured_gap = float(self.config.dynamic_obstacle_min_gap_progress)
        return max(configured_gap, 0.18)

    def unlimited_progress_to_tile(self, progress):
        """Map continuous progress onto the repeated straight-road tiles."""
        if self.track_points is None or len(self.track_points) < 2:
            return float(progress), np.zeros(2, dtype=np.float32)
        tile_index = math.floor(float(progress))
        local_progress = float(progress) - float(tile_index)
        tile_shift = self.track_points[-1] - self.track_points[0]
        active_tile_index = getattr(self, 'virtual_road_tile_index', 0)
        tile_offset = tile_shift * float(tile_index - active_tile_index)
        return local_progress, tile_offset.astype(np.float32)

    def evaluate_dynamic_progress(
        self,
        spec,
        elapsed,
        min_progress,
        max_progress,
        continuous=False,
    ):
        """Compute the current obstacle progress for one scenario spec."""
        base_progress = float(spec['base_progress'])
        speed_pps = float(spec.get('progress_speed_pps', self.config.dynamic_obstacle_speed_pps))
        if spec.get('stop_go_period_s'):
            period = max(0.1, float(spec['stop_go_period_s']))
            duty = min(0.95, max(0.05, float(spec.get('stop_go_duty', 0.5))))
            phase = float(spec.get('phase_s', 0.0))
            cycle = (elapsed + phase) % period
            moving = cycle < period * duty
            active_time = elapsed * duty if moving else elapsed * duty
            progress = base_progress + speed_pps * active_time
        elif spec.get('slowdown_rate') is not None:
            decay = max(0.0, float(spec['slowdown_rate']))
            progress = base_progress + speed_pps * (1.0 - math.exp(-decay * elapsed)) / max(1e-6, decay)
        else:
            progress = base_progress + speed_pps * elapsed

        if spec.get('progress_wave_amp'):
            amplitude = float(spec['progress_wave_amp'])
            period = max(0.1, float(spec.get('progress_wave_period_s', 6.0)))
            phase = float(spec.get('phase_s', 0.0))
            progress += amplitude * math.sin((2.0 * math.pi * elapsed / period) + phase)

        if spec.get('hold_point_progress') is not None:
            hold_point = float(spec['hold_point_progress'])
            progress = min(progress, hold_point)

        if continuous:
            return progress

        if spec.get('wrap_progress'):
            span = max(1e-3, max_progress - min_progress)
            progress = min_progress + ((progress - min_progress) % span)
        else:
            progress = min(max(progress, min_progress), max_progress)

        return progress

    def evaluate_dynamic_lateral_offset(self, spec, elapsed):
        """Compute the current lateral offset for one scenario spec."""
        offset = float(spec['base_offset_px'])
        if spec.get('lateral_target_px') is not None:
            target = float(spec['lateral_target_px'])
            duration = max(0.1, float(spec.get('lateral_target_duration_s', 4.0)))
            delay = max(0.0, float(spec.get('lateral_start_delay_s', 0.0)))
            lateral_elapsed = max(0.0, elapsed - delay)
            blend = min(1.0, max(0.0, lateral_elapsed / duration))
            blend = blend * blend * (3.0 - 2.0 * blend)
            offset = (1.0 - blend) * offset + blend * target
        if spec.get('lateral_speed_px') is not None:
            offset += float(spec['lateral_speed_px']) * elapsed
        if spec.get('lateral_wave_amp'):
            amplitude = float(spec['lateral_wave_amp'])
            period = max(0.1, float(spec.get('lateral_wave_period_s', 5.0)))
            phase = float(spec.get('phase_s', 0.0))
            offset += amplitude * math.sin((2.0 * math.pi * elapsed / period) + phase)

        road_half_width = float(self.config.road_half_width_px)
        max_offset = max(0.0, road_half_width - 0.5 * float(spec['width_px']))
        return float(np.clip(offset, -max_offset, max_offset))

    def vehicle_progress(self, center):
        """Return current ArUco progress as 0..1 along the straight road."""
        if center is None or self.track_points is None or len(self.track_points) < 2:
            return None
        distances = np.linalg.norm(self.track_points - center, axis=1)
        nearest_index = int(np.argmin(distances))
        return nearest_index / float(len(self.track_points) - 1)

    def sync_controller_obstacles(self):
        """Expose static, detected random, and dynamic obstacles to the controller."""
        self.controller.set_obstacle_layers(
            self.static_scene_obstacles,
            self.detected_random_obstacles,
            self.detected_dynamic_obstacles,
            use_random_static_obstacles=self.config.random_static_obstacles,
        )

def main(args=None):
    """Run guarded straight-road following with moving virtual obstacles."""
    config, ros_args = parse_args(args)
    config.dynamic_obstacles = True
    validate_config(config)
    if not config.dry_run and not config.confirm_propulsion_safe:
        raise SystemExit(
            'refusing ArUco track following output: securely restrain driven '
            'wheels, then pass --confirm-propulsion-safe'
        )

    rclpy.init(args=ros_args)
    node = ArucoTrackFollower(config)
    validate_config(config)
    print_config(config)
    executor = None
    if not config.single_thread:
        executor = MultiThreadedExecutor(num_threads=config.executor_threads)
        executor.add_node(node)
        import threading
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

    def spin_or_render_once():
        if executor is None:
            rclpy.spin_once(node, timeout_sec=0.02)
        else:
            time.sleep(0.02)
        if config.preview:
            node.update_pid_from_panel()
            node.render_pending_preview()
        if config.preview or node.tuning.enabled:
            node.poll_keyboard()

    try:
        if config.dry_run:
            if config.virtual_vehicle_test:
                print('Dry run: running dynamic straight virtual vehicle test.')
            else:
                print('Dry run: detecting marker without publishing RC output.')
            while rclpy.ok() and not node.stop_requested:
                spin_or_render_once()
            return

        time.sleep(0.2)
        if not getattr(config, 'ignore_subscribers', False) and node.count_subscribers(config.command_topic) == 0:
            raise RuntimeError(
                f'no subscriber on {config.command_topic}; run crsf_ros first'
            )
        print('Waiting for ArUco marker before arming.')
        deadline = time.monotonic() + 10.0
        while not node.command_is_fresh() and time.monotonic() < deadline:
            spin_or_render_once()
        if not node.command_is_fresh():
            raise RuntimeError(
                f'no marker ID {config.marker_id} seen on {config.image_topic}'
            )

        print('Sending stop/center command before arming.')
        node.publish_neutral_for(SETTLE_DURATION)
        node.set_armed(True)
        node.output_enabled = True
        if node.command_is_fresh():
            node.command_pub.publish(node.drive_message())
        print('Dynamic straight follower armed; Ctrl-C stops and disarms.')
        while rclpy.ok() and not node.stop_requested:
            spin_or_render_once()
    except KeyboardInterrupt:
        print('\nInterrupted; stopping dynamic straight follower.')
    finally:
        print('\nReturning to stop/center and disarming.')
        node.output_enabled = False
        try:
            if not config.dry_run:
                node.publish_neutral_for(SETTLE_DURATION)
            node.save_metrics_file()
        finally:
            try:
                if node.armed:
                    node.set_armed(False)
            finally:
                cv2.destroyAllWindows()
                if executor is not None:
                    executor.shutdown()
                    executor.remove_node(node)
                node.destroy_node()
                rclpy.try_shutdown()


if __name__ == '__main__':
    main()
