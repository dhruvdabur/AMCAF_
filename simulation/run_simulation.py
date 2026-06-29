#!/usr/bin/env python3
"""Master launcher and simulation loop for straight road follow-the-gap tracking."""

import sys
import math
import time
import json
from pathlib import Path
import numpy as np
import cv2
import rclpy
from rclpy.node import Node

# Import simulation modules
from .config import parse_args, validate_config, print_config, UPDATE_RATE_HZ, SETTLE_DURATION
from hardware.hil.control import DynamicStraightController
from .road import make_laneless_static_obstacle, make_straight_road_boundary_walls, make_straight_road_end_walls
from .tuning import StraightStaticTuning

PREVIEW_WINDOW = 'Aruco Track Follower'

def noop(val):
    pass

# Overlay Styles matching ui/preview.py
OVERLAY_TEXT_SCALE = 1.0
OVERLAY_TEXT_VISIBLE = True

def set_overlay_text_style(scale=None, visible=None):
    global OVERLAY_TEXT_SCALE, OVERLAY_TEXT_VISIBLE
    if scale is not None:
        OVERLAY_TEXT_SCALE = max(0.1, min(float(scale), 3.0))
    if visible is not None:
        OVERLAY_TEXT_VISIBLE = bool(visible)

def resize_for_preview(frame, target_width):
    if target_width <= 0:
        return frame
    height, width = frame.shape[:2]
    if width <= target_width:
        return frame
    scale = target_width / float(width)
    return cv2.resize(
        frame,
        (target_width, int(round(height * scale))),
        interpolation=cv2.INTER_AREA,
    )

def draw_label(frame, origin, text, color, scale=0.5):
    if not OVERLAY_TEXT_VISIBLE:
        return
    scaled = max(0.1, float(scale) * OVERLAY_TEXT_SCALE)
    halo_thickness = max(1, int(round(3 * OVERLAY_TEXT_SCALE)))
    text_thickness = max(1, int(round(OVERLAY_TEXT_SCALE)))
    cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scaled, (0, 0, 0), halo_thickness, cv2.LINE_AA)
    cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scaled, color, text_thickness, cv2.LINE_AA)

def put_status(frame, mode, throttle, roll, fresh, track_speed_pps, target_track_speed_pps, speed_error_pps, velocity_delta_pwm, cbf_scale, metrics, lap_limit_enabled, target_laps, nearest_static_clearance_px=float('inf'), cbf_qp_status='unused', cbf_qp_accel=0.0, cbf_qp_delta=0.0):
    if not OVERLAY_TEXT_VISIBLE:
        return
    lines = [
        f'{mode}  thr {throttle}  roll {roll}',
        f'speed {track_speed_pps:.1f}/{target_track_speed_pps:.1f} pps',
        f'cbf {cbf_scale:.2f}',
    ]
    if cbf_qp_status != 'unused':
        lines.append(f'qp {cbf_qp_status.lower()}  a {cbf_qp_accel:.2f}  d {cbf_qp_delta:.2f}')
    
    scale = 0.55 * OVERLAY_TEXT_SCALE
    line_step = max(12, int(round(22 * OVERLAY_TEXT_SCALE)))
    y_start = max(14, int(round(24 * OVERLAY_TEXT_SCALE)))
    halo_thickness = max(1, int(round(4 * OVERLAY_TEXT_SCALE)))
    text_thickness = max(1, int(round(OVERLAY_TEXT_SCALE)))
    for index, line in enumerate(lines):
        origin = (12, y_start + index * line_step)
        cv2.putText(frame, line, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), halo_thickness, cv2.LINE_AA)
        cv2.putText(frame, line, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), text_thickness, cv2.LINE_AA)

class ArucoTrackFollower(Node):
    """ROS 2 Node wrapper for Standalone visual tracking simulation."""

    def __init__(self, config):
        super().__init__('aruco_track_follower')
        self.config = config
        self.stop_requested = False
        
        # Load controller and tuning widgets
        self.controller = DynamicStraightController(config)
        self.tuning = StraightStaticTuning(config, self.controller, logger=self.get_logger(), enabled=config.preview)
        
        # Dynamic obstacles state bookkeeping
        self.dynamic_obstacles = []
        self.detected_dynamic_obstacles = []
        self.dynamic_obstacle_specs = []
        self.dynamic_obstacle_previous_states = {}
        self.dynamic_obstacle_smoothed_states = {}
        self.retired_dynamic_obstacle_indices = set()
        self.dynamic_obstacle_start_time = None
        self.dynamic_obstacles_need_ego_placement = True
        
        # Scenario options
        if getattr(config, 'traffic_scenario', 'free_flow') == 'head_on':
            config.include_road_boundary_walls = False
        if not config.static_obstacles:
            config.static_obstacles = '[]'
            
        # Virtual vehicle state variables
        self.virtual_vehicle_state = None
        self.virtual_last_time = None
        self.virtual_road_tile_index = 0
        self.random_static_obstacles = []
        self.detected_random_obstacles = []
        self.virtual_vehicle_trail = []
        self.latest_preview_frame = None
        
        self.preview_window_ready = False
        self.preview_text_controls_ready = False
        self.preview_window_size_initialized = False
        self.flash_timer = 0
        
        # Load tuning if requested
        if config.load_tuning:
            self.tuning.load_tuning_file()
            self.controller.refresh_cbf_qp_config()
            
        if self.tuning.enabled:
            self.tuning.create_pid_panel()
            
        # Initialize vehicle state
        self.reset_virtual_vehicle()
        
        # Create control loop timer at 50 Hz
        self.create_timer(1.0 / UPDATE_RATE_HZ, self.virtual_vehicle_tick)

    def __getattr__(self, name):
        """Delegate missing attributes to the controller core."""
        return getattr(self.controller, name)

    def reset_virtual_vehicle(self):
        """Position a virtual vehicle at the beginning of the track."""
        width = self.config.virtual_width
        height = self.config.virtual_height
        self.virtual_road_tile_index = 0
        self.controller.build_track_scene(width, height)
        
        if self.config.random_static_obstacles:
            self.random_static_obstacles = self.make_random_static_obstacles()
            self.detected_random_obstacles = []
            self.controller.static_obstacles = []
            
        point_count = len(self.track_points)
        start_index = int(round(self.config.virtual_start_progress * (point_count - 1)))
        start_index = max(0, min(start_index, point_count - 1))
        
        center = self.track_points[start_index].astype(np.float32)
        normal = self.road_normals[start_index]
        center = center + normal * self.config.virtual_start_lateral_offset_px
        
        self.virtual_vehicle_state = {
            'x': float(center[0]),
            'y': float(center[1]),
            'heading': math.radians(self.config.virtual_start_heading_deg),
            'speed': max(0.0, self.config.virtual_start_speed_pps),
        }
        self.controller.set_aruco_marker_size_px(float(getattr(self.config, 'virtual_marker_size_px', 10.0)))
        self.virtual_last_time = time.monotonic()
        self.controller.mark_marker_lost()
        
        self.remember_static_scene_obstacles()
        self.reset_dynamic_obstacles(time.monotonic())
        self.flash_timer = 5

    def remember_static_scene_obstacles(self):
        self.static_scene_obstacles = list(self.controller.static_obstacles)

    def reset_dynamic_obstacles(self, now):
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
        if not self.dynamic_obstacle_specs:
            return
        if center is None:
            if self.virtual_vehicle_state is None:
                return
            center = np.array([self.virtual_vehicle_state['x'], self.virtual_vehicle_state['y']], dtype=np.float32)
        else:
            center = np.asarray(center, dtype=np.float32)
            
        ego_progress = self.vehicle_progress(center)
        if ego_progress is None:
            return
        self.dynamic_obstacles_need_ego_placement = False
        min_start_progress = ego_progress + self.dynamic_obstacle_ahead_gap()
        earliest_progress = min(float(spec.get('base_progress', 0.0)) for spec in self.dynamic_obstacle_specs)
        shift = max(0.0, min_start_progress - earliest_progress)
        if shift <= 0.0:
            return
        for spec in self.dynamic_obstacle_specs:
            spec['base_progress'] = float(spec.get('base_progress', 0.0)) + shift

    def make_free_flow_specs(self, rng):
        min_progress = float(self.config.dynamic_obstacle_min_progress)
        max_progress = float(self.config.dynamic_obstacle_max_progress)
        road_half_width = float(self.config.road_half_width_px)
        ego_progress_speed = self.nominal_ego_progress_speed()
        specs = []
        count = self.config.dynamic_obstacle_count * 2
        
        # Free flow parameters
        factors = (0.30, 0.34)
        for index in range(count):
            speed_factor = factors[index % len(factors)]
            progress_speed_pps = 1.0 * ego_progress_speed * speed_factor
            
            # ponytail: Spawn smaller virtual obstacles (length 25-45px, width 15-30px)
            # Apply 0.55 zoom scaling
            length_px = float(rng.uniform(25.0, 45.0)) * 0.55
            width_px = float(rng.uniform(15.0, 30.0)) * 0.55
            
            specs.append({
                'index': index,
                'base_progress': float(rng.uniform(min_progress, max_progress)),
                'base_offset_px': float(rng.uniform(-road_half_width * 0.35, road_half_width * 0.35)),
                'length_px': length_px,
                'width_px': width_px,
                'progress_speed_pps': progress_speed_pps,
                'kind': 'dynamic',
            })
        return specs

    def make_scenario_specs(self, scenario, rng):
        road_half_width = float(self.config.road_half_width_px)
        # Apply 0.55 zoom scaling to obstacle sizes
        default_length = 96.0 * 0.55
        default_width = 58.0 * 0.55
        next_index = {'value': 0}

        def spec(base_progress, base_offset_px, progress_speed_pps, length_px=default_length, width_px=default_width, kind='dynamic', **extra):
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
                spec(0.62, offset_right * 0.45, ego_progress_speed * 0.08, length_px=78.0*0.55, width_px=50.0*0.55),
            ]
        if scenario == 'cut_in':
            return [
                spec(0.34, offset_left, ego_progress_speed * 0.10, lateral_target_px=offset_center, lateral_start_delay_s=2.0, lateral_target_duration_s=3.4, hard_spacing=True, hard_min_gap_progress=0.22),
                spec(0.08, offset_right, ego_progress_speed * 0.08, length_px=82.0*0.55, width_px=52.0*0.55, lateral_target_px=offset_center, lateral_start_delay_s=6.0, lateral_target_duration_s=3.4, hard_spacing=True, hard_min_gap_progress=0.22),
            ]
        if scenario == 'overtake_merge':
            return [
                spec(0.22, offset_center, ego_progress_speed * 0.10, length_px=104.0*0.55, width_px=62.0*0.55),
                spec(0.12, offset_left, ego_progress_speed * 0.08, lateral_target_px=offset_center, lateral_target_duration_s=4.5, progress_wave_amp=12.0, progress_wave_period_s=7.0),
            ]
        if scenario == 'stop_go_platoon':
            return [
                spec(0.24, offset_center, ego_progress_speed * 0.10, stop_go_period_s=5.5, stop_go_duty=0.45, phase_s=0.0),
                spec(0.33, offset_center, ego_progress_speed * 0.08, stop_go_period_s=5.5, stop_go_duty=0.45, phase_s=1.5),
                spec(0.42, offset_center, ego_progress_speed * 0.10, stop_go_period_s=5.5, stop_go_duty=0.45, phase_s=3.0),
                spec(0.51, offset_right * 0.18, ego_progress_speed * 0.08, stop_go_period_s=5.5, stop_go_duty=0.45, phase_s=4.5, width_px=50.0*0.55),
            ]
        if scenario == 'dense_flow':
            return [
                spec(0.18, offset_left * 0.75, ego_progress_speed * 0.10, length_px=84.0*0.55, width_px=52.0*0.55),
                spec(0.28, offset_center, ego_progress_speed * 0.08, length_px=92.0*0.55, width_px=58.0*0.55),
                spec(0.38, offset_right * 0.80, ego_progress_speed * 0.10, length_px=88.0*0.55, width_px=54.0*0.55),
                spec(0.49, offset_left * 0.25, ego_progress_speed * 0.08, length_px=96.0*0.55, width_px=56.0*0.55),
                spec(0.60, offset_right * 0.30, ego_progress_speed * 0.10, length_px=90.0*0.55, width_px=55.0*0.55),
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
                spec(0.50, -road_half_width * 0.52, ego_progress_speed * 0.00, lateral_speed_px=road_half_width * 0.22, respect_vehicle_gap=False, length_px=70.0*0.55, width_px=46.0*0.55),
                spec(0.62, road_half_width * 0.48, ego_progress_speed * 0.08, length_px=82.0*0.55, width_px=50.0*0.55),
            ]
        if scenario == 'signal_phase':
            return [
                spec(0.36, offset_center, ego_progress_speed * 0.10, stop_go_period_s=6.0, stop_go_duty=0.35, phase_s=0.0, hold_point_progress=0.42),
                spec(0.55, offset_center, ego_progress_speed * 0.08, stop_go_period_s=6.0, stop_go_duty=0.35, phase_s=2.8, hold_point_progress=0.58),
            ]
        if scenario == 'looping_flow':
            return [
                spec(0.20, offset_left * 0.40, ego_progress_speed * 0.10, wrap_progress=True, length_px=88.0*0.55, width_px=52.0*0.55),
                spec(0.45, offset_center, ego_progress_speed * 0.08, wrap_progress=True, length_px=94.0*0.55, width_px=56.0*0.55),
                spec(0.70, offset_right * 0.42, ego_progress_speed * 0.10, wrap_progress=True, length_px=90.0*0.55, width_px=54.0*0.55),
            ]
        if scenario == 'three_sparse':
            return [
                spec(0.20, offset_left, ego_progress_speed * 0.10, wrap_progress=True, length_px=90.0*0.55, width_px=54.0*0.55),
                spec(0.50, offset_center, ego_progress_speed * 0.08, wrap_progress=True, length_px=90.0*0.55, width_px=54.0*0.55),
                spec(0.80, offset_right, ego_progress_speed * 0.10, wrap_progress=True, length_px=90.0*0.55, width_px=54.0*0.55),
            ]
        if scenario == 'head_on':
            return [
                spec(0.50, offset_center, 0.0, length_px=90.0*0.55, width_px=54.0*0.55),
            ]
        if scenario == 'endless_walls':
            return [
                spec(0.15, offset_right, ego_progress_speed * 0.10, wrap_progress=True, length_px=90.0*0.55, width_px=54.0*0.55),
                spec(0.35, offset_left, 0.0, wrap_progress=True, length_px=30.0*0.55, width_px=150.0*0.55),
                spec(0.55, offset_left, ego_progress_speed * 0.08, wrap_progress=True, length_px=90.0*0.55, width_px=54.0*0.55),
                spec(0.80, offset_right, 0.0, wrap_progress=True, length_px=30.0*0.55, width_px=150.0*0.55),
            ]
        return self.make_free_flow_specs(rng)

    def make_random_static_obstacles(self):
        rng = np.random.default_rng(self.config.random_obstacle_seed)
        obstacles = []
        for index in range(self.config.random_obstacle_count):
            progress = float(rng.uniform(self.config.random_obstacle_min_progress, self.config.random_obstacle_max_progress))
            lateral_fraction = float(rng.uniform(-0.58, 0.58))
            
            # Spawn smaller virtual obstacles scaled by 0.55 zoom
            length_px = float(rng.uniform(25.0, 45.0)) * 0.55
            width_px = float(rng.uniform(15.0, 30.0)) * 0.55
            
            spec = {
                'progress': progress,
                'length_px': length_px,
                'width_px': width_px,
                'offset': lateral_fraction,
            }
            obstacle = make_laneless_static_obstacle(spec, self.track_points, self.road_tangents, self.road_normals, self.config)
            obstacle['random_index'] = index
            obstacle['detected'] = False
            obstacles.append(obstacle)
        return obstacles

    def update_detected_random_obstacles(self, center):
        detected = []
        heading = self.virtual_vehicle_state['heading']
        forward = np.array([math.cos(heading), math.sin(heading)], dtype=np.float32)
        for obstacle in self.random_static_obstacles:
            delta = obstacle['center'] - center
            ahead = float(np.dot(delta, forward))
            distance = float(np.linalg.norm(delta))
            visible = (ahead > -obstacle['half_length'] and distance <= self.config.random_obstacle_detection_range_px)
            if visible:
                obstacle['detected'] = True
            if obstacle.get('detected'):
                detected.append(obstacle)
        self.detected_random_obstacles = detected
        self.controller.static_obstacles = detected

    def update_dynamic_obstacles(self, now, vehicle_center=None, vehicle_heading=None):
        if self.track_points is None:
            return
        if self.dynamic_obstacle_start_time is None:
            self.reset_dynamic_obstacles(now)
            return
        if self.dynamic_obstacles_need_ego_placement and vehicle_center is not None:
            self.place_dynamic_specs_ahead_of_ego(vehicle_center)
            self.dynamic_obstacle_start_time = now
            self.dynamic_obstacle_previous_states = {}
            self.dynamic_obstacle_smoothed_states = {}
            self.retired_dynamic_obstacle_indices = set()
            self.dynamic_obstacles_need_ego_placement = False

        elapsed = max(0.0, now - self.dynamic_obstacle_start_time)
        min_progress = float(self.config.dynamic_obstacle_min_progress)
        max_progress = float(self.config.dynamic_obstacle_max_progress)
        unlimited_path = self.config.virtual_unlimited_path
        
        scene_track_points = self.track_points
        candidates = []
        repeat_obstacles = self.dynamic_scenario_repeats_obstacles()
        
        for spec in self.dynamic_obstacle_specs:
            if spec['index'] in self.retired_dynamic_obstacle_indices:
                continue
            progress = self.evaluate_dynamic_progress(
                spec, elapsed, min_progress, max_progress, continuous=unlimited_path or not repeat_obstacles
            )
            if unlimited_path and repeat_obstacles:
                ego_local_progress = 0.0
                if vehicle_center is not None:
                    local_p = self.vehicle_progress(vehicle_center)
                    if local_p is not None:
                        ego_local_progress = local_p
                ego_cont = float(self.virtual_road_tile_index) + ego_local_progress
                progress = ego_cont - 0.15 + ((progress - (ego_cont - 0.15)) % 1.0)
                
            if not repeat_obstacles and (progress < min_progress or progress > max_progress):
                self.retired_dynamic_obstacle_indices.add(spec['index'])
                continue
                
            lateral_offset = self.evaluate_dynamic_lateral_offset(spec, elapsed)
            hard_min_gap = spec.get('hard_min_gap_progress')
            if not unlimited_path:
                progress = min(max(progress, min_progress), max_progress)
                
            candidates.append({
                'spec': spec,
                'progress': progress,
                'lateral_offset_px': lateral_offset,
                'length_px': float(spec['length_px']),
                'width_px': float(spec['width_px']),
                'hard_spacing': bool(spec.get('hard_spacing', False)),
                'hard_min_gap_progress': None if hard_min_gap is None else float(hard_min_gap),
            })

        if unlimited_path:
            candidates = self.resolve_continuous_dynamic_obstacle_layout(candidates, min_progress)
        else:
            candidates = self.resolve_dynamic_obstacle_layout(candidates, min_progress, max_progress)
            
        candidates = [self.smooth_dynamic_candidate(candidate, now) for candidate in candidates]
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
                self.config
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
                
            self.dynamic_obstacle_previous_states[spec['index']] = (obstacle['center'].copy(), now)
            obstacles.append(obstacle)
            
        self.dynamic_obstacles = obstacles
        self.detected_dynamic_obstacles = list(self.dynamic_obstacles)
        self.sync_controller_obstacles()

    def update_detected_random_obstacles_sim(self, center):
        self.update_detected_random_obstacles(center)

    def sync_controller_obstacles(self):
        self.controller.set_obstacle_layers(
            self.static_scene_obstacles,
            self.detected_random_obstacles,
            self.detected_dynamic_obstacles,
            use_random_static_obstacles=self.config.random_static_obstacles
        )

    def virtual_vehicle_tick(self):
        """Timer callback triggered at 50Hz to run vehicle kinematics."""
        if self.stop_requested:
            return
        if self.virtual_vehicle_state is None:
            self.reset_virtual_vehicle()
            
        now = time.monotonic()
        dt = min(0.1, max(1e-3, now - self.virtual_last_time))
        self.virtual_last_time = now

        # Live parameters mapping from sliders
        if self.tuning.enabled:
            self.tuning.update_pid_from_panel()
            
        state = self.virtual_vehicle_state
        center = np.array([state['x'], state['y']], dtype=np.float32)
        
        if self.config.random_static_obstacles:
            self.update_detected_random_obstacles(center)
            
        # Update dynamic traffic
        self.update_dynamic_obstacles(now, center, state['heading'])
        
        # Calculate controls
        result = self.controller.process_detection(
            center, state['heading'], image_size=(self.config.virtual_width, self.config.virtual_height), now=now
        )
        
        previous_center = center.copy()
        
        # Kinematic update
        self.advance_virtual_vehicle(dt)
        
        preview_target = result.target
        if self.config.virtual_unlimited_path:
            scene_delta = self.advance_virtual_road_window(previous_center)
            preview_target = result.target + scene_delta
        elif self.config.virtual_stop_at_end and self.virtual_reached_road_end():
            self.request_stop('virtual vehicle reached road end')
            
        if self.config.preview:
            frame = self.make_virtual_frame()
            preview_center = np.array([self.virtual_vehicle_state['x'], self.virtual_vehicle_state['y']], dtype=np.float32)
            self.show_preview(frame, None, preview_center, preview_target, result.tangent)

    def advance_virtual_vehicle(self, dt):
        state = self.virtual_vehicle_state
        throttle_span = max(1.0, float(self.config.max_forward_pwm - self.config.neutral_throttle_pwm))
        throttle_ratio = (self.throttle - self.config.neutral_throttle_pwm) / throttle_span
        target_speed = max(0.0, throttle_ratio * self.config.target_track_speed_pps)
        speed_error = target_speed - state['speed']
        accel = speed_error * self.config.virtual_speed_response
        accel = max(-self.config.virtual_max_brake_pps2, min(self.config.virtual_max_accel_pps2, accel))
        state['speed'] = max(0.0, state['speed'] + accel * dt)
        
        # Actuation logic
        delta = self.controller.roll_pwm_to_qp_delta(self.roll)
        heading_rate = state['speed'] / max(1.0, self.config.qp_wheelbase_px) * delta
        state['heading'] = math.atan2(
            math.sin(state['heading'] + heading_rate * dt),
            math.cos(state['heading'] + heading_rate * dt)
        )
        state['x'] += math.cos(state['heading']) * state['speed'] * dt
        state['y'] += math.sin(state['heading']) * state['speed'] * dt
        
        # Track path trail
        self.virtual_vehicle_trail.append(np.array([state['x'], state['y']], dtype=np.float32))
        if len(self.virtual_vehicle_trail) > 120:
            self.virtual_vehicle_trail.pop(0)

    def virtual_reached_road_end(self):
        nearest_index = self.current_virtual_nearest_index()
        if nearest_index is None or self.track_points is None:
            return False
        return nearest_index >= len(self.track_points) - 3

    def current_virtual_nearest_index(self):
        if self.virtual_vehicle_state is None or self.track_points is None:
            return None
        center = np.array([self.virtual_vehicle_state['x'], self.virtual_vehicle_state['y']], dtype=np.float32)
        distances = np.linalg.norm(self.track_points - center, axis=1)
        return int(np.argmin(distances))

    def advance_virtual_road_window(self, previous_center):
        scene_delta = self.scroll_virtual_follow_frame(previous_center)
        if self.virtual_reached_road_end():
            scene_delta = scene_delta + self.recycle_virtual_road_window()
        return scene_delta

    def scroll_virtual_follow_frame(self, previous_center):
        zero_delta = np.zeros(2, dtype=np.float32)
        if self.virtual_vehicle_state is None or self.track_points is None or self.road_tangents is None or len(self.track_points) < 2:
            return zero_delta
        nearest_index = self.current_virtual_nearest_index()
        if nearest_index is None:
            return zero_delta
        state = self.virtual_vehicle_state
        center = np.array([state['x'], state['y']], dtype=np.float32)
        tangent = self.road_tangents[max(0, min(nearest_index, len(self.road_tangents) - 1))]
        forward_delta = float(np.dot(center - previous_center, tangent))
        if abs(forward_delta) <= 1e-6:
            return zero_delta
        shift = tangent * forward_delta
        state['x'] = float(center[0] - shift[0])
        state['y'] = float(center[1] - shift[1])
        scene_delta = -shift
        self.translate_virtual_scene(scene_delta)
        return scene_delta

    def recycle_virtual_road_window(self):
        if self.track_points is None or len(self.track_points) < 2:
            return np.zeros(2, dtype=np.float32)
        tile_shift = self.track_points[-1] - self.track_points[0]
        self.translate_virtual_scene(tile_shift)
        self.virtual_road_tile_index += 1
        self.controller.last_progress_index = None
        self.controller.metrics.previous_progress = None
        self.get_logger().info('advanced virtual road window')
        return tile_shift

    def translate_virtual_scene(self, delta):
        delta = np.asarray(delta, dtype=np.float32)
        if self.track_points is not None:
            self.track_points = self.track_points + delta
            self.controller.track_points = self.track_points
        if self.road_boundaries:
            self.road_boundaries = [boundary + delta for boundary in self.road_boundaries]
            self.controller.road_boundaries = self.road_boundaries
        for obstacle in self.virtual_scene_obstacles():
            obstacle['center'] = obstacle['center'] + delta
            obstacle['polygon'] = obstacle['polygon'] + delta
        if self.controller.latest_free_space_target is not None:
            self.controller.latest_free_space_target = self.controller.latest_free_space_target + delta
        if self.controller.latest_target_center is not None:
            self.controller.latest_target_center = self.controller.latest_target_center + delta
        self.virtual_vehicle_trail = [pt + delta for pt in self.virtual_vehicle_trail]
        
        ftg_debug = getattr(self.controller, 'latest_ftg_debug', None)
        if ftg_debug is not None:
            for key in ('target', 'nearest_point', 'origin', 'scaled_target'):
                if ftg_debug.get(key) is not None:
                    ftg_debug[key] = ftg_debug[key] + delta
            if ftg_debug.get('bubble_points'):
                ftg_debug['bubble_points'] = [p + delta for p in ftg_debug['bubble_points']]

    def virtual_scene_obstacles(self):
        obstacle_lists = [
            self.controller.static_obstacles,
            self.random_static_obstacles,
            self.detected_random_obstacles,
            self.dynamic_obstacles
        ]
        seen = set()
        for obstacles in obstacle_lists:
            for obstacle in obstacles:
                key = id(obstacle)
                if key in seen:
                    continue
                seen.add(key)
                yield obstacle

    def make_virtual_frame(self):
        height = self.config.virtual_height
        width = self.config.virtual_width
        y_gradient = np.linspace(248, 232, height, dtype=np.uint8)[:, None]
        frame = np.repeat(y_gradient, width, axis=1)
        frame = cv2.merge((frame, frame, frame))

        grid_color = (224, 224, 224)
        for x in range(0, width, 80):
            cv2.line(frame, (x, 0), (x, height), grid_color, 1, cv2.LINE_AA)
        for y in range(0, height, 80):
            cv2.line(frame, (0, y), (width, y), grid_color, 1, cv2.LINE_AA)

        cv2.rectangle(frame, (0, 0), (width - 1, height - 1), (205, 205, 205), 1)
        return frame

    def draw_road_scene(self, preview):
        if self.unlimited_virtual_road_enabled():
            self.draw_virtual_road_tiles(preview)
        elif self.road_boundaries and len(self.road_boundaries) >= 2:
            self.draw_road_band(preview)
            
        if self.road_boundaries and not self.unlimited_virtual_road_enabled():
            for boundary in self.road_boundaries:
                cv2.polylines(preview, [boundary.astype(np.int32)], False, (255, 255, 255), 3, cv2.LINE_AA)
                
        if self.track_points is not None and not self.unlimited_virtual_road_enabled():
            cv2.polylines(preview, [self.track_points.astype(np.int32)], False, (246, 246, 246), 2, cv2.LINE_AA)
            self.draw_progress_ticks(preview)
            
        if self.unlimited_virtual_road_enabled():
            return
            
        obstacles = list(self.random_static_obstacles or self.static_obstacles)
        if self.config.random_static_obstacles and getattr(self.config, 'include_road_boundary_walls', False):
            obstacles.extend(getattr(self.controller, 'road_boundary_obstacles', []))
            
        for obstacle in obstacles:
            if self.config.random_static_obstacles and not obstacle.get('detected'):
                fill_color = (174, 174, 174)
            elif str(obstacle.get('kind', '')).startswith('road_boundary_wall'):
                fill_color = (88, 92, 96)
            elif obstacle.get('kind') == 'dynamic' or 'dynamic_index' in obstacle:
                fill_color = (54, 100, 230)
            else:
                fill_color = (66, 66, 196)
            shadow = obstacle['polygon'].astype(np.int32) + np.array([3, 4])
            cv2.fillConvexPoly(preview, shadow, (188, 188, 188))
            cv2.fillConvexPoly(preview, obstacle['polygon'].astype(np.int32), fill_color)

    def unlimited_virtual_road_enabled(self):
        return (
            self.config.virtual_vehicle_test
            and self.config.virtual_unlimited_path
            and self.track_points is not None
            and self.road_boundaries
            and len(self.road_boundaries) >= 2
            and len(self.track_points) >= 2
        )

    def draw_virtual_road_tiles(self, preview):
        static_obstacles = self.virtual_tile_static_obstacles()
        for delta in self.virtual_road_tile_offsets(preview):
            shifted_boundaries = [boundary + delta for boundary in self.road_boundaries]
            shifted_track = self.track_points + delta
            self.draw_road_band_arrays(preview, shifted_boundaries, shifted_track)
            
            for boundary in shifted_boundaries:
                cv2.polylines(preview, [boundary.astype(np.int32)], False, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.polylines(preview, [shifted_track.astype(np.int32)], False, (246, 246, 246), 2, cv2.LINE_AA)
            self.draw_progress_ticks_for(preview, shifted_track)

            for obstacle in static_obstacles:
                if self.config.random_static_obstacles and not obstacle.get('detected'):
                    fill_color = (174, 174, 174)
                elif str(obstacle.get('kind', '')).startswith('road_boundary_wall'):
                    fill_color = (88, 92, 96)
                elif obstacle.get('kind') == 'dynamic' or 'dynamic_index' in obstacle:
                    fill_color = (54, 100, 230)
                else:
                    fill_color = (66, 66, 196)
                shifted_poly = obstacle['polygon'] + delta
                shadow = shifted_poly.astype(np.int32) + np.array([3, 4])
                cv2.fillConvexPoly(preview, shadow, (188, 188, 188))
                cv2.fillConvexPoly(preview, shifted_poly.astype(np.int32), fill_color)
                
        for obstacle in self.virtual_tile_dynamic_obstacles():
            fill_color = (54, 100, 230)
            polygon = obstacle['polygon'].astype(np.int32)
            shadow = polygon + np.array([3, 4])
            cv2.fillConvexPoly(preview, shadow, (188, 188, 188))
            cv2.fillConvexPoly(preview, polygon, fill_color)
            cv2.polylines(preview, [polygon], True, (30, 30, 30), 1, cv2.LINE_AA)

    def virtual_tile_static_obstacles(self):
        if self.config.random_static_obstacles:
            obstacles = list(self.random_static_obstacles)
            if getattr(self.config, 'include_road_boundary_walls', False):
                obstacles.extend(getattr(self.controller, 'road_boundary_obstacles', []))
            return obstacles
        obstacles = list(self.static_obstacles)
        if getattr(self.config, 'include_road_boundary_walls', False):
            obstacles.extend(getattr(self.controller, 'road_boundary_obstacles', []))
        return [obstacle for obstacle in obstacles if obstacle.get('kind') != 'dynamic' and 'dynamic_index' not in obstacle]

    def virtual_tile_dynamic_obstacles(self):
        return list(self.dynamic_obstacles)

    def virtual_road_tile_offsets(self, preview):
        tile_shift = self.track_points[-1] - self.track_points[0]
        if np.linalg.norm(tile_shift) <= 1e-6:
            return [np.zeros(2, dtype=np.float32)]
        height, width = preview.shape[:2]
        margin = max(width, height) * 0.25
        all_points = np.vstack(self.road_boundaries + [self.track_points])
        offsets = []
        for tile_index in range(-3, 5):
            delta = tile_shift * float(tile_index)
            shifted = all_points + delta
            min_xy = np.min(shifted, axis=0)
            max_xy = np.max(shifted, axis=0)
            visible = (max_xy[0] >= -margin and min_xy[0] <= width + margin and max_xy[1] >= -margin and min_xy[1] <= height + margin)
            if visible:
                offsets.append(delta.astype(np.float32))
        if not offsets:
            offsets.append(np.zeros(2, dtype=np.float32))
        return offsets

    def draw_road_band_arrays(self, preview, boundaries, track_points):
        left_boundary = boundaries[0].astype(np.int32)
        right_boundary = boundaries[1].astype(np.int32)
        road_polygon = np.vstack((left_boundary, right_boundary[::-1]))
        overlay = preview.copy()
        cv2.fillPoly(overlay, [road_polygon], (74, 84, 88))
        cv2.addWeighted(overlay, 0.74, preview, 0.26, 0.0, preview)
        inner_width = min(42.0, self.road_half_width_px * 0.35)
        for lane_edge in (track_points - self.road_normals * inner_width, track_points + self.road_normals * inner_width):
            cv2.polylines(preview, [lane_edge.astype(np.int32)], False, (112, 122, 126), 1, cv2.LINE_AA)

    def draw_progress_ticks_for(self, preview, track_points):
        if self.road_normals is None or len(track_points) < 2:
            return
        step = max(1, len(track_points) // 10)
        tick_half = 12.0
        for index in range(0, len(track_points), step):
            center = track_points[index]
            normal = self.road_normals[index]
            start = center - normal * tick_half
            end = center + normal * tick_half
            cv2.line(preview, tuple(start.astype(int)), tuple(end.astype(int)), (184, 190, 192), 1, cv2.LINE_AA)

    def draw_road_band(self, preview):
        left_boundary = self.road_boundaries[0].astype(np.int32)
        right_boundary = self.road_boundaries[1].astype(np.int32)
        road_polygon = np.vstack((left_boundary, right_boundary[::-1]))
        overlay = preview.copy()
        cv2.fillPoly(overlay, [road_polygon], (74, 84, 88))
        cv2.addWeighted(overlay, 0.74, preview, 0.26, 0.0, preview)
        inner_left = self.track_points - self.road_normals * min(42.0, self.road_half_width_px * 0.35)
        inner_right = self.track_points + self.road_normals * min(42.0, self.road_half_width_px * 0.35)
        for lane_edge in (inner_left, inner_right):
            cv2.polylines(preview, [lane_edge.astype(np.int32)], False, (112, 122, 126), 1, cv2.LINE_AA)

    def draw_progress_ticks(self, preview):
        if self.track_points is None or self.road_normals is None or len(self.track_points) < 2:
            return
        step = max(1, len(track_points) // 10)
        tick_half = 12.0
        for index in range(0, len(self.track_points), step):
            center = self.track_points[index]
            normal = self.road_normals[index]
            start = center - normal * tick_half
            end = center + normal * tick_half
            cv2.line(preview, tuple(start.astype(int)), tuple(end.astype(int)), (184, 190, 192), 1, cv2.LINE_AA)

    def draw_virtual_vehicle(self, preview):
        if self.virtual_vehicle_state is None:
            return
        state = self.virtual_vehicle_state
        center = np.array([state['x'], state['y']], dtype=np.float32)
        heading = state['heading']
        forward = np.array([math.cos(heading), math.sin(heading)], dtype=np.float32)
        right = np.array([-math.sin(heading), math.cos(heading)], dtype=np.float32)
        
        # Applying 0.55 zoom scaling
        half_length = 34.0 * 0.55
        half_width = 18.0 * 0.55
        cabin_scale = 0.55
        wheel_scale = 0.55
        wheel_radius = 2

        polygon = np.array([
            center + forward * half_length + right * half_width,
            center - forward * half_length + right * half_width,
            center - forward * half_length - right * half_width,
            center + forward * half_length - right * half_width,
        ], dtype=np.int32)
        
        shadow = polygon + np.array([2, 3], dtype=np.int32)
        cv2.fillConvexPoly(preview, shadow, (185, 185, 185))
        cv2.fillConvexPoly(preview, polygon, (188, 114, 0)) # BGR body color
        cv2.polylines(preview, [polygon], True, (18, 42, 74), 2, cv2.LINE_AA)
        
        cabin = np.array([
            center + forward * (10.0 * cabin_scale) + right * (11.0 * cabin_scale),
            center - forward * (14.0 * cabin_scale) + right * (10.0 * cabin_scale),
            center - forward * (14.0 * cabin_scale) - right * (10.0 * cabin_scale),
            center + forward * (10.0 * cabin_scale) - right * (11.0 * cabin_scale),
        ], dtype=np.int32)
        cv2.fillConvexPoly(preview, cabin, (255, 224, 166))
        cv2.polylines(preview, [cabin], True, (95, 85, 65), 1, cv2.LINE_AA)
        
        for wheel_offset in (
            forward * (19.0 * wheel_scale) + right * (19.0 * wheel_scale),
            forward * (19.0 * wheel_scale) - right * (19.0 * wheel_scale),
            -forward * (21.0 * wheel_scale) + right * (19.0 * wheel_scale),
            -forward * (21.0 * wheel_scale) - right * (19.0 * wheel_scale),
        ):
            wheel_center = center + wheel_offset
            cv2.circle(preview, tuple(wheel_center.astype(int)), wheel_radius, (32, 32, 32), -1)
            
        nose = center + forward * (half_length + 10.0 * wheel_scale)
        cv2.arrowedLine(preview, tuple(center.astype(int)), tuple(nose.astype(int)), (0, 58, 220), 2, cv2.LINE_AA, tipLength=0.26)

    def draw_debug_visuals(self, preview, center, target, tangent):
        self.draw_virtual_vehicle_trail(preview)
        self.draw_cbf_ellipse_debug(preview, center)
        self.draw_lidar_feedback(preview, center)
        self.draw_drift_style_hud(preview)
        self.draw_ftg_debug(preview, center)

    def draw_virtual_vehicle_trail(self, preview):
        if len(self.virtual_vehicle_trail) < 2:
            return
        points = [tuple(pt.astype(int)) for pt in self.virtual_vehicle_trail]
        for i in range(len(points) - 1):
            cv2.line(preview, points[i], points[i+1], (0, 165, 255), 1, cv2.LINE_AA)

    def draw_cbf_ellipse_debug(self, preview, center):
        if center is None:
            return
        color = (120, 170, 220)
        if self.cbf_active:
            color = (0, 165, 255)
        if self.cbf_qp_status == 'infeasible' or self.cbf_qp_h < 0.0:
            color = (0, 0, 255)
        a_ell_px, b_ell_px = self.controller.cbf_ellipse_axes_px()
        marker_scale_px = self.controller.cbf_ellipse_marker_scale_px()
        angle_deg = math.degrees(float(self.last_marker_heading))
        axes = (max(1, int(round(a_ell_px))), max(1, int(round(b_ell_px))))
        cv2.ellipse(preview, tuple(center.astype(int)), axes, angle_deg, 0.0, 360.0, color, 2, cv2.LINE_AA)
        
        label_origin = center + np.array([a_ell_px + 8.0, b_ell_px + 12.0])
        label_text = (
            f'QP a={self.config.cbf_a_ell:.2f} b={self.config.cbf_b_ell:.2f} '
            f'scale={self.format_distance_m(marker_scale_px)} -> '
            f'{self.format_distance_m(a_ell_px)}x{self.format_distance_m(b_ell_px)}'
        )
        draw_label(preview, tuple(label_origin.astype(int)), label_text, color, scale=0.42)

    def marker_scale_m_per_px(self):
        marker_size_px = self.controller.cbf_ellipse_marker_scale_px()
        if marker_size_px <= 1e-6:
            return None
        marker_size_m = float(self.config.aruco_marker_size_cm) / 100.0
        return marker_size_m / marker_size_px

    def distance_m(self, distance_px):
        scale = self.marker_scale_m_per_px()
        if scale is None or not math.isfinite(float(distance_px)):
            return None
        return float(distance_px) * scale

    def format_distance_m(self, distance_px):
        dist_m = self.distance_m(distance_px)
        if dist_m is None:
            return 'n/a m'
        return f'{dist_m:.3f}m'

    def draw_lidar_feedback(self, preview, center):
        if center is None:
            return
        critical_x = getattr(self.controller, 'cbf_qp_obstacle_x', None)
        critical_y = getattr(self.controller, 'cbf_qp_obstacle_y', None)
        if critical_x is not None and critical_y is not None:
            xobs = np.array([float(critical_x), float(critical_y)], dtype=np.float32)
            closest, distance_px = self.closest_point_on_cbf_ellipse(center, xobs)
            distance_color = (0, 255, 255) if distance_px >= 0.0 else (0, 0, 255)
            crit_hit = (int(round(critical_x)), int(round(critical_y)))
            closest_hit = tuple(closest.astype(int))
            cv2.line(preview, closest_hit, crit_hit, distance_color, 2, cv2.LINE_AA)
            cv2.circle(preview, closest_hit, 5, distance_color, -1, cv2.LINE_AA)
            cv2.circle(preview, crit_hit, 6, (0, 255, 255), -1)
            cv2.circle(preview, crit_hit, 12, (0, 0, 255), 2, cv2.LINE_AA)
            
            label_anchor = ((closest + xobs) * 0.5 + np.array([8.0, -8.0])).astype(int)
            draw_label(preview, tuple(label_anchor), f'ell-xobs {self.format_distance_m(distance_px)}', distance_color, scale=0.45)

    def closest_point_on_cbf_ellipse(self, center, point):
        center = np.asarray(center, dtype=np.float32)
        point = np.asarray(point, dtype=np.float32)
        a_ell_px, b_ell_px = self.controller.cbf_ellipse_axes_px()
        a_ell_px = max(1.0, float(a_ell_px))
        b_ell_px = max(1.0, float(b_ell_px))
        heading = float(self.last_marker_heading)
        forward = np.array([math.cos(heading), math.sin(heading)], dtype=np.float32)
        lateral = np.array([-math.sin(heading), math.cos(heading)], dtype=np.float32)
        delta = point - center
        local = np.array([float(np.dot(delta, forward)), float(np.dot(delta, lateral))], dtype=np.float64)
        closest_local = self.closest_point_on_axis_aligned_ellipse(local, a_ell_px, b_ell_px)
        closest_world = center + forward * float(closest_local[0]) + lateral * float(closest_local[1])
        distance_px = float(np.linalg.norm(point - closest_world))
        ellipse_value = (local[0] / a_ell_px) ** 2 + (local[1] / b_ell_px) ** 2
        if ellipse_value < 1.0:
            distance_px = -distance_px
        return closest_world.astype(np.float32), distance_px

    @staticmethod
    def closest_point_on_axis_aligned_ellipse(point, a_axis, b_axis):
        point = np.asarray(point, dtype=np.float64)
        if np.linalg.norm(point) <= 1e-9:
            if a_axis <= b_axis:
                return np.array([a_axis, 0.0], dtype=np.float64)
            return np.array([0.0, b_axis], dtype=np.float64)

        px, py = float(point[0]), float(point[1])
        theta = math.atan2(a_axis * py, b_axis * px)
        a_sq = a_axis * a_axis
        b_sq = b_axis * b_axis
        diff_sq = a_sq - b_sq
        for _step in range(8):
            sin_t = math.sin(theta)
            cos_t = math.cos(theta)
            gradient = diff_sq * sin_t * cos_t - a_axis * px * sin_t + b_axis * py * cos_t
            hessian = diff_sq * (cos_t * cos_t - sin_t * sin_t) - a_axis * px * cos_t - b_axis * py * sin_t
            if abs(hessian) <= 1e-9:
                break
            theta -= gradient / hessian
        return np.array([a_axis * math.cos(theta), b_axis * math.sin(theta)], dtype=np.float64)

    def draw_drift_style_hud(self, preview):
        height, width = preview.shape[:2]
        hud_width = 320
        hud_height = 95
        hx = max(12, width - hud_width - 12)
        hy = 12
        overlay = preview.copy()
        cv2.rectangle(overlay, (hx, hy), (hx + hud_width, hy + hud_height), (30, 30, 30), -1)
        cv2.addWeighted(overlay, 0.65, preview, 0.35, 0.0, preview)
        cv2.rectangle(preview, (hx, hy), (hx + hud_width, hy + hud_height), (100, 100, 100), 1, cv2.LINE_AA)

        speed = self.track_speed_pps
        target_speed = self.effective_target_track_speed_pps
        h_val = getattr(self, 'cbf_qp_h', 1.0)
        h_threshold = getattr(self.config, 'cbf_h_px', 42.0)
        h_ratio = np.clip(h_val / max(1.0, h_threshold), 0.0, 1.0)
        
        is_shield_active = getattr(self, 'cbf_active', False) or getattr(self, 'cbf_qp_brake_gate_active', False)
        mode_str = "SAFE SHIELD" if is_shield_active else "NOMINAL"
        mode_color = (0, 0, 255) if is_shield_active else (0, 255, 0)
        
        collisions = getattr(self.metrics, 'collision_samples', 0)
        coll_str = "COLLISION!" if collisions > 0 else "CLEAN"
        coll_color = (0, 0, 255) if collisions > 0 else (0, 255, 0)

        draw_label(preview, (hx + 10, hy + 20), "MODE: ", (255, 255, 255), scale=0.42)
        draw_label(preview, (hx + 60, hy + 20), mode_str, mode_color, scale=0.42)
        draw_label(preview, (hx + 180, hy + 20), "COLL: ", (255, 255, 255), scale=0.42)
        draw_label(preview, (hx + 230, hy + 20), coll_str, coll_color, scale=0.42)
        
        vel_ratio = np.clip(speed / max(1.0, target_speed), 0.0, 1.0)
        cv2.putText(preview, "V:", (hx + 10, hy + 45), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.rectangle(preview, (hx + 70, hy + 36), (hx + hud_width - 15, hy + 46), (60, 60, 60), -1)
        fill_w = int(vel_ratio * (hud_width - 85))
        if fill_w > 0:
            cv2.rectangle(preview, (hx + 70, hy + 36), (hx + 70 + fill_w, hy + 46), (188, 114, 0), -1)

        cv2.putText(preview, "Safety:", (hx + 10, hy + 75), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.rectangle(preview, (hx + 70, hy + 66), (hx + hud_width - 15, hy + 76), (60, 60, 60), -1)
        cbf_color = (0, int(255 * h_ratio), int(255 * (1.0 - h_ratio)))
        fill_h_w = int(h_ratio * (hud_width - 85))
        if fill_h_w > 0:
            cv2.rectangle(preview, (hx + 70, hy + 66), (hx + 70 + fill_h_w, hy + 76), cbf_color, -1)

    def draw_ftg_debug(self, preview, center):
        ftg_debug = getattr(self.controller, 'latest_ftg_debug', None)
        if not ftg_debug or center is None:
            return
        origin = ftg_debug.get('origin', center)
        origin = np.asarray(origin, dtype=np.float32)
        heading = float(ftg_debug.get('heading', self.last_marker_heading))

        costs = ftg_debug.get('costs')
        ranges = ftg_debug.get('ranges')
        angles = ftg_debug.get('angles')
        if costs is not None and ranges is not None and angles is not None:
            valid_costs = costs[np.isfinite(costs)]
            if len(valid_costs) > 0:
                min_cost = float(np.min(valid_costs))
                max_cost = float(np.max(valid_costs))
                cost_range = max_cost - min_cost if max_cost > min_cost else 1.0

                for i in range(len(ranges)):
                    angle = angles[i]
                    rng = ranges[i]
                    cost = costs[i]

                    if not np.isfinite(cost):
                        color = (0, 0, 200)
                    else:
                        t = (cost - min_cost) / cost_range
                        if t < 0.5:
                            u = t * 2.0
                            color = (0, 255, int(u * 255))
                        else:
                            u = (t - 0.5) * 2.0
                            color = (0, int((1.0 - u) * 255), 255)

                    global_angle = heading + angle
                    ray_end = origin + rng * np.array([math.cos(global_angle), math.sin(global_angle)], dtype=np.float32)
                    rx, ry = int(round(ray_end[0])), int(round(ray_end[1]))
                    ox, oy = int(round(origin[0])), int(round(origin[1]))

                    cv2.line(preview, (ox, oy), (rx, ry), color, 1, cv2.LINE_AA)
                    cv2.circle(preview, (rx, ry), 2, color, -1, cv2.LINE_AA)

        nearest_point = ftg_debug.get('nearest_point')
        if nearest_point is not None:
            nx, ny = int(round(nearest_point[0])), int(round(nearest_point[1]))
            cv2.line(preview, (nx - 8, ny - 8), (nx + 8, ny + 8), (0, 140, 255), 2, cv2.LINE_AA)
            cv2.line(preview, (nx - 8, ny + 8), (nx + 8, ny - 8), (0, 140, 255), 2, cv2.LINE_AA)
            cv2.circle(preview, (nx, ny), 5, (0, 140, 255), -1, cv2.LINE_AA)
            draw_label(preview, (nx + 10, ny - 10), "Threat", (0, 140, 255), scale=0.42)
            
            bubble_radius = ftg_debug.get('bubble_radius', 0.0)
            if bubble_radius > 0.0:
                cv2.circle(preview, (nx, ny), int(round(bubble_radius)), (0, 100, 255), 1, cv2.LINE_AA)

        target = ftg_debug.get('target')
        if target is not None:
            tx, ty = int(round(target[0])), int(round(target[1]))
            cv2.circle(preview, (tx, ty), 5, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.circle(preview, (tx, ty), 10, (0, 0, 255), 2, cv2.LINE_AA)
            draw_label(preview, (tx + 12, ty - 12), "Raw FTG", (0, 0, 255), scale=0.42)

        target_angle = math.degrees(float(ftg_debug.get('target_angle', 0.0)))
        target_dist = float(ftg_debug.get('target_dist', 0.0))
        max_range = float(ftg_debug.get('max_range_cap', 0.0))
        text_origin = np.array([16, preview.shape[0] - 18], dtype=np.int32)
        draw_label(preview, tuple(text_origin), f'ang={target_angle:.1f}deg dist={target_dist:.0f}px range={max_range:.0f}px', (255, 180, 60), scale=0.42)

    def show_preview(self, frame, corners, center, target, tangent):
        self.update_preview_text_controls()
        if self.flash_timer > 0:
            frame[:] = 255
            self.flash_timer -= 1
        preview = frame.copy()
        
        self.draw_road_scene(preview)
        if center is not None:
            cv2.circle(preview, tuple(center.astype(int)), 6, (255, 0, 0), -1)
        self.draw_virtual_vehicle(preview)
        
        if target is not None:
            cv2.circle(preview, tuple(target.astype(int)), 8, (0, 0, 255), -1)
        if center is not None and target is not None:
            cv2.line(preview, tuple(center.astype(int)), tuple(target.astype(int)), (0, 220, 255), 2, cv2.LINE_AA)
            
        self.draw_debug_visuals(preview, center, target, tangent)
        preview = resize_for_preview(preview, self.config.preview_width)
        self.latest_preview_frame = preview.copy()
        self.prepare_preview_window(preview)
        self.resize_preview_window(preview)
        cv2.imshow(PREVIEW_WINDOW, preview)

    def prepare_preview_window(self, preview):
        if self.preview_window_ready:
            return
        cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_NORMAL)
        cv2.moveWindow(PREVIEW_WINDOW, 50, 50)
        self.create_preview_text_controls()
        self.preview_window_ready = True

    def create_preview_text_controls(self):
        if self.preview_text_controls_ready:
            return
        cv2.createTrackbar('text x10', PREVIEW_WINDOW, 10, 30, noop)
        cv2.createTrackbar('text on', PREVIEW_WINDOW, 1, 1, noop)
        self.preview_text_controls_ready = True

    def update_preview_text_controls(self):
        if not self.preview_text_controls_ready:
            set_overlay_text_style(scale=1.0, visible=True)
            return
        text_scale_x10 = cv2.getTrackbarPos('text x10', PREVIEW_WINDOW)
        text_visible = cv2.getTrackbarPos('text on', PREVIEW_WINDOW) > 0
        set_overlay_text_style(scale=max(1, text_scale_x10) / 10.0, visible=text_visible)

    def resize_preview_window(self, preview):
        if self.preview_window_size_initialized:
            return
        height, width = preview.shape[:2]
        target_width = width
        if self.config.preview_width > 0:
            target_width = max(width, self.config.preview_width)
        target_height = int(round(height * target_width / max(1, width)))
        cv2.resizeWindow(PREVIEW_WINDOW, target_width, target_height)
        cv2.moveWindow(PREVIEW_WINDOW, 50, 50)
        self.preview_window_size_initialized = True

    def poll_keyboard(self):
        if self.config.ignore_preview_keys:
            return
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.request_stop('q pressed')
        elif key == ord('s'):
            self.tuning.save_tuning_file()
        elif key == ord('l'):
            self.tuning.load_tuning_file()
            self.controller.refresh_cbf_qp_config()

    def request_stop(self, reason):
        if self.stop_requested:
            return
        self.stop_requested = True
        self.get_logger().info(f'{reason}; stopping simulation')
        self.save_metrics_file()

    def save_metrics_file(self):
        if not self.controller.metrics_saved:
            summary = self.controller.metrics.summary()
            metrics_path = Path(self.controller.metrics_file_path)
            metrics_path.parent.mkdir(parents=True, exist_ok=True)
            metrics_path.write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
            self.get_logger().info(f'saved ranking metrics to {metrics_path}')
            self.controller.metrics_saved = True

    # Helper math mappings matching straight_static_node.py
    def resolve_dynamic_obstacle_layout(self, candidates, min_progress, max_progress):
        if not candidates:
            return []
        path_length_px = self.road_path_length_px()
        if path_length_px <= 1e-6:
            return candidates
        packed = sorted((dict(c) for c in candidates), key=lambda x: (x['progress'], x['lateral_offset_px']))
        fixed_candidates = self.fixed_obstacle_layout_candidates()
        self.forward_pack_obstacles(packed, fixed_candidates, min_progress, max_progress, path_length_px)
        
        overflow = max(0.0, max(item['progress'] for item in packed) - max_progress)
        if overflow > 0.0:
            for item in packed:
                item['progress'] = max(min_progress, item['progress'] - overflow)
            self.forward_pack_obstacles(packed, fixed_candidates, min_progress, max_progress, path_length_px)
            
        accepted = []
        for item in packed:
            if item['progress'] > max_progress:
                continue
            if self.candidate_overlaps_any(item, fixed_candidates + accepted, path_length_px):
                continue
            accepted.append(item)
        return accepted

    def resolve_continuous_dynamic_obstacle_layout(self, candidates, min_progress):
        if not candidates:
            return []
        path_length_px = self.road_path_length_px()
        if path_length_px <= 1e-6:
            return candidates
        packed = sorted((dict(c) for c in candidates), key=lambda x: (x['progress'], x['lateral_offset_px']))
        placed = self.continuous_fixed_obstacle_layout_candidates(min_progress)
        for candidate in packed:
            candidate['progress'] = max(float(candidate['progress']), min_progress)
            for previous in placed:
                if not self.candidates_need_spacing(candidate, previous):
                    continue
                min_delta = self.required_progress_delta(candidate, previous, path_length_px)
                candidate['progress'] = max(candidate['progress'], previous['progress'] + min_delta)
            placed.append(candidate)
        return packed

    def continuous_fixed_obstacle_layout_candidates(self, min_progress):
        fixed = []
        for obs in self.fixed_obstacle_layout_candidates():
            fixed.append({
                'progress': max(float(obs['progress']), min_progress),
                'lateral_offset_px': float(obs['lateral_offset_px']),
                'length_px': float(obs['length_px']),
                'width_px': float(obs['width_px']),
            })
        return fixed

    def fixed_obstacle_layout_candidates(self):
        fixed = []
        source_obstacles = self.detected_random_obstacles if self.config.random_static_obstacles else self.static_scene_obstacles
        for obs in source_obstacles:
            if str(obs.get('kind', '')).startswith('road_boundary_wall'):
                continue
            fixed.append({
                'progress': float(obs.get('progress', 0.0)),
                'lateral_offset_px': float(obs.get('lateral_offset_px', 0.0)),
                'length_px': float(obs['half_length']) * 2.0,
                'width_px': float(obs['half_width']) * 2.0,
            })
        return sorted(fixed, key=lambda x: (x['progress'], x['lateral_offset_px']))

    def forward_pack_obstacles(self, candidates, fixed_candidates, min_progress, max_progress, path_length_px):
        placed = list(fixed_candidates)
        for candidate in candidates:
            candidate['progress'] = min(max(float(candidate['progress']), min_progress), max_progress)
            for previous in placed:
                if not self.candidates_need_spacing(candidate, previous):
                    continue
                min_delta = self.required_progress_delta(candidate, previous, path_length_px)
                candidate['progress'] = max(candidate['progress'], previous['progress'] + min_delta)
            placed.append(candidate)

    def candidate_overlaps_any(self, candidate, others, path_length_px):
        for other in others:
            if not self.candidates_need_spacing(candidate, other):
                continue
            min_delta = self.required_progress_delta(candidate, other, path_length_px)
            if abs(candidate['progress'] - other['progress']) < min_delta:
                return True
        return False

    def candidates_overlap_laterally(self, first, second):
        lateral_gap = float(self.config.dynamic_obstacle_min_lateral_gap_px)
        min_separation = 0.5 * float(first['width_px']) + 0.5 * float(second['width_px']) + lateral_gap
        return abs(float(first['lateral_offset_px']) - float(second['lateral_offset_px'])) < min_separation

    def candidates_need_spacing(self, first, second):
        if first.get('hard_spacing') or second.get('hard_spacing'):
            return True
        return self.candidates_overlap_laterally(first, second)

    def required_progress_delta(self, first, second, path_length_px):
        longitudinal_gap = float(self.config.dynamic_obstacle_min_longitudinal_gap_px)
        separation_px = 0.5 * float(first['length_px']) + 0.5 * float(second['length_px']) + longitudinal_gap
        progress_delta = separation_px / max(path_length_px, 1e-6)
        hard_gap = max(float(first.get('hard_min_gap_progress') or 0.0), float(second.get('hard_min_gap_progress') or 0.0))
        return max(progress_delta, hard_gap)

    def road_path_length_px(self):
        if self.track_points is None or len(self.track_points) < 2:
            return 0.0
        deltas = np.diff(self.track_points.astype(np.float64), axis=0)
        return float(np.sum(np.linalg.norm(deltas, axis=1)))

    def nominal_ego_progress_speed(self):
        path_length = self.road_path_length_px()
        if path_length <= 1e-6:
            return 0.05
        return max(0.001, float(self.config.target_track_speed_pps) / path_length)

    def dynamic_obstacle_ahead_gap(self):
        configured_gap = float(self.config.dynamic_obstacle_min_gap_progress)
        return max(configured_gap, 0.18)

    def unlimited_progress_to_tile(self, progress):
        if self.track_points is None or len(self.track_points) < 2:
            return float(progress), np.zeros(2, dtype=np.float32)
        tile_index = math.floor(float(progress))
        local_progress = float(progress) - float(tile_index)
        tile_shift = self.track_points[-1] - self.track_points[0]
        tile_offset = tile_shift * float(tile_index - self.virtual_road_tile_index)
        return local_progress, tile_offset.astype(np.float32)

    def evaluate_dynamic_progress(self, spec, elapsed, min_progress, max_progress, continuous=False):
        base_progress = float(spec['base_progress'])
        speed_pps = float(spec.get('progress_speed_pps', self.config.dynamic_obstacle_speed_pps))
        if spec.get('stop_go_period_s'):
            period = max(0.1, float(spec['stop_go_period_s']))
            duty = min(0.95, max(0.05, float(spec.get('stop_go_duty', 0.5))))
            phase = float(spec.get('phase_s', 0.0))
            cycle = (elapsed + phase) % period
            moving = cycle < period * duty
            active_time = elapsed * duty
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
        if center is None or self.track_points is None or len(self.track_points) < 2:
            return None
        distances = np.linalg.norm(self.track_points - center, axis=1)
        nearest_index = int(np.argmin(distances))
        return nearest_index / float(len(self.track_points) - 1)

    def smooth_dynamic_candidate(self, candidate, now):
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
        smoothed['progress'] = (1.0 - alpha) * float(previous['progress']) + alpha * float(candidate['progress'])
        smoothed['lateral_offset_px'] = (1.0 - alpha) * float(previous['lateral_offset_px']) + alpha * float(candidate['lateral_offset_px'])
        
        self.dynamic_obstacle_smoothed_states[spec_index] = {
            'progress': float(smoothed['progress']),
            'lateral_offset_px': float(smoothed['lateral_offset_px']),
            'time': now,
        }
        return smoothed

    def dynamic_scenario_repeats_obstacles(self):
        scenario = getattr(self.config, 'traffic_scenario', 'free_flow')
        return scenario in ('looping_flow', 'three_sparse', 'endless_walls')

def main(args=None):
    # Parse CLI inputs
    config, ros_args = parse_args(sys.argv[1:] if args is None else args)
    validate_config(config)
    print_config(config)
    
    # Initialize ROS 2
    rclpy.init(args=ros_args)
    
    node = ArucoTrackFollower(config)
    print("Standalone simulation running. Press 'q' or 'ESC' on the preview frame to exit.")
    
    try:
        while rclpy.ok() and not node.stop_requested:
            # Spin ROS callbacks to handle timer events
            rclpy.spin_once(node, timeout_sec=0.01)
            
            # Poll keyboard inputs in OpenCV windows
            node.poll_keyboard()
            
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    finally:
        node.request_stop('cleanup')
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
