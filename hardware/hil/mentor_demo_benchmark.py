#!/usr/bin/env python3
"""Run PID versus CBF-QP ellipse benchmark scenarios for mentor demos."""

import argparse
import csv
import json
from pathlib import Path
import statistics

import numpy as np

from .config.dynamic_straight import parse_args as parse_follower_args
from .config.dynamic_straight import validate_config
from .control import DynamicStraightController
from .controllers import PID
from .controllers import PID_VELOCITY_CBF_QP_ELLIPSE
from .nodes.dynamic_straight_node import ArucoTrackFollower

DEFAULT_SCENARIOS = (
    'free_flow',
    'lead_slowdown',
    'cut_in',
    'stop_go_platoon',
    'dense_flow',
)
DEFAULT_CONTROLLERS = (
    PID,
    PID_VELOCITY_CBF_QP_ELLIPSE,
)


def parse_args(args=None):
    """Read benchmark options."""
    parser = argparse.ArgumentParser(
        description=(
            'Run deterministic virtual dynamic-straight scenarios and compare '
            'PID tracking against PID+CBF-QP ellipse safety filtering.'
        )
    )
    parser.add_argument('--scenarios', nargs='+', default=list(DEFAULT_SCENARIOS))
    parser.add_argument('--controllers', nargs='+', default=list(DEFAULT_CONTROLLERS))
    parser.add_argument('--duration-s', type=float, default=12.0)
    parser.add_argument('--rate-hz', type=float, default=20.0)
    parser.add_argument('--output-dir', type=Path, default=Path('mentor_demo_runs'))
    parser.add_argument('--dynamic-obstacle-count', type=int, default=5)
    parser.add_argument('--dynamic-obstacle-seed', type=int, default=13)
    parser.add_argument(
        '--qp-slack-weight',
        type=float,
        default=0.0,
        help='Optional QP slack penalty for CBF-QP runs. 0 disables slack.',
    )
    return parser.parse_args(args)


def make_config(scenario, controller, options):
    """Create a follower config for one benchmark run."""
    follower_args = [
        '--traffic-scenario',
        scenario,
        '--controller-mode',
        controller,
        '--virtual-vehicle-test',
        '--no-pid-panel',
        '--no-telemetry',
        '--dynamic-obstacle-count',
        str(options.dynamic_obstacle_count),
        '--dynamic-obstacle-seed',
        str(options.dynamic_obstacle_seed),
        '--qp-slack-weight',
        str(options.qp_slack_weight),
    ]
    config, _ros_args = parse_follower_args(follower_args)
    validate_config(config)
    return config


def make_headless_node(config):
    """Create the dynamic follower object without constructing a ROS node."""
    node = object.__new__(ArucoTrackFollower)
    node.config = config
    node.controller = DynamicStraightController(config)
    node.virtual_vehicle_state = None
    node.virtual_last_time = 0.0
    node.random_static_obstacles = []
    node.detected_random_obstacles = []
    node.dynamic_obstacles = []
    node.dynamic_obstacle_specs = []
    node.dynamic_obstacle_start_time = None
    node.static_scene_obstacles = []
    node.reset_virtual_vehicle()
    node.virtual_last_time = 0.0
    node.dynamic_obstacle_start_time = 0.0
    return node


def run_case(scenario, controller, options):
    """Run one deterministic virtual scenario and return metrics."""
    config = make_config(scenario, controller, options)
    node = make_headless_node(config)
    dt = 1.0 / options.rate_hz
    steps = max(1, int(round(options.duration_s * options.rate_hz)))

    solve_times = []
    slack_values = []
    h_values = []
    roll_values = []
    throttle_values = []
    roll_interventions = []
    throttle_interventions = []
    active_samples = 0

    for step in range(steps):
        now = step * dt
        state = node.virtual_vehicle_state
        center = np.array([state['x'], state['y']], dtype=np.float32)
        node.update_dynamic_obstacles(now, center)
        node.controller.process_detection(
            center,
            state['heading'],
            image_size=(config.virtual_width, config.virtual_height),
            now=now,
        )
        if node.controller.cbf_active:
            active_samples += 1
        if node.controller.cbf_qp_status != 'unused':
            solve_times.append(float(node.controller.cbf_qp_solve_time_ms))
            slack_values.append(float(node.controller.cbf_qp_slack))
            h_values.append(float(node.controller.cbf_qp_h))
        roll_values.append(float(node.controller.roll))
        throttle_values.append(float(node.controller.throttle))
        roll_interventions.append(
            abs(float(node.controller.roll) - float(node.controller.nominal_roll))
        )
        throttle_interventions.append(
            abs(
                float(node.controller.throttle)
                - float(node.controller.nominal_throttle)
            )
        )
        node.advance_virtual_vehicle(dt)

    summary = node.controller.metrics.summary()
    summary.update(
        {
            'scenario': scenario,
            'controller_mode': controller,
            'duration_s': options.duration_s,
            'rate_hz': options.rate_hz,
            'samples': summary.get('samples', steps),
            'cbf_active_ratio': active_samples / float(steps),
            'mean_qp_solve_time_ms': mean_or_zero(solve_times),
            'max_qp_solve_time_ms': max(solve_times) if solve_times else 0.0,
            'max_qp_slack': max(slack_values) if slack_values else 0.0,
            'mean_qp_slack': mean_or_zero(slack_values),
            'min_cbf_h': min(h_values) if h_values else None,
            'steering_smoothness_pwm': total_variation(roll_values),
            'throttle_smoothness_pwm': total_variation(throttle_values),
            'mean_roll_intervention_pwm': mean_or_zero(roll_interventions),
            'mean_throttle_intervention_pwm': mean_or_zero(throttle_interventions),
            'max_roll_intervention_pwm': max(roll_interventions)
            if roll_interventions
            else 0.0,
            'max_throttle_intervention_pwm': max(throttle_interventions)
            if throttle_interventions
            else 0.0,
        }
    )
    return summary


def mean_or_zero(values):
    """Return the arithmetic mean or 0 for an empty list."""
    if not values:
        return 0.0
    return float(statistics.fmean(values))


def total_variation(values):
    """Return sum of absolute sample-to-sample command changes."""
    if len(values) < 2:
        return 0.0
    return float(
        sum(abs(values[index] - values[index - 1]) for index in range(1, len(values)))
    )


def write_outputs(rows, output_dir):
    """Write machine-readable and mentor-friendly benchmark reports."""
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / 'mentor_demo_benchmark.json'
    csv_path = output_dir / 'mentor_demo_benchmark.csv'
    md_path = output_dir / 'mentor_demo_benchmark.md'

    json_path.write_text(json.dumps(rows, indent=2), encoding='utf-8')
    fieldnames = sorted({key for row in rows for key in row})
    with csv_path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    md_path.write_text(markdown_report(rows), encoding='utf-8')
    return json_path, csv_path, md_path


def markdown_report(rows):
    """Return a compact Markdown table for reports/slides."""
    headers = [
        'Scenario',
        'Controller',
        'Collisions',
        'Min clearance px',
        'CBF active',
        'Min h',
        'Mean QP ms',
        'Max QP ms',
        'Max slack',
        'Steer smooth',
    ]
    lines = ['# Mentor Demo Benchmark', '']
    lines.append('| ' + ' | '.join(headers) + ' |')
    lines.append('| ' + ' | '.join(['---'] * len(headers)) + ' |')
    for row in rows:
        min_h = row.get('min_cbf_h')
        min_h_text = 'n/a' if min_h is None else f'{min_h:.3f}'
        lines.append(
            '| '
            + ' | '.join(
                [
                    row['scenario'],
                    row['controller_mode'],
                    str(row['collision_samples']),
                    f'{row["min_obstacle_clearance_px"]:.1f}',
                    f'{100.0 * row["cbf_active_ratio"]:.1f}%',
                    min_h_text,
                    f'{row["mean_qp_solve_time_ms"]:.2f}',
                    f'{row["max_qp_solve_time_ms"]:.2f}',
                    f'{row["max_qp_slack"]:.3f}',
                    f'{row["steering_smoothness_pwm"]:.1f}',
                ]
            )
            + ' |'
        )
    lines.append('')
    lines.append(
        'Use the PID rows as baseline and the CBF-QP rows to show safety '
        'intervention, barrier value, solve time, and smoothness tradeoffs.'
    )
    return '\n'.join(lines) + '\n'


def main(args=None):
    """Run all requested benchmark cases."""
    options = parse_args(args)
    rows = []
    for scenario in options.scenarios:
        for controller in options.controllers:
            rows.append(run_case(scenario, controller, options))
            print(f'finished {scenario} / {controller}')
    json_path, csv_path, md_path = write_outputs(rows, options.output_dir)
    print(f'wrote {json_path}')
    print(f'wrote {csv_path}')
    print(f'wrote {md_path}')


if __name__ == '__main__':
    main()
