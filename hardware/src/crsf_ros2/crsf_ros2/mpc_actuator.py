#!/usr/bin/env python3
"""Map Ackermann MPC commands to guarded CRSF RC commands."""

import argparse
import math
import time

from rc_msgs.msg import AckermannCommand
from rc_msgs.msg import RCMessage
from rc_msgs.srv import CommandBool

import rclpy
from rclpy.node import Node


INPUT_TOPIC = '/mpc/ackermann_command'
COMMAND_TOPIC = '/drone/rc_command'
ARMING_SERVICE = '/drone/cmd/arming'
NEUTRAL_VALUE = 1500
TELEOP_FORWARD_PWM = 1589
TELEOP_MAX_FORWARD_PWM = 1600
TELEOP_MAX_REVERSE_PWM = 1415
TELEOP_LEFT_PWM = 1300
TELEOP_RIGHT_PWM = 1700
UPDATE_RATE_HZ = 50.0
SETTLE_DURATION = 0.5
STOP_SPEED_EPS_MPS = 1e-3


def bounded(value, lower, upper):
    """Return value restricted to inclusive lower and upper limits."""
    return max(lower, min(value, upper))


def steer_to_pwm(steer_rad, max_steer_rad, center, left, right):
    """Map signed steering radians to asymmetric servo endpoints."""
    steer_rad = bounded(steer_rad, -max_steer_rad, max_steer_rad)
    if steer_rad >= 0.0:
        return round(center + steer_rad / max_steer_rad * (left - center))
    return round(center + -steer_rad / max_steer_rad * (right - center))


def accel_to_pwm(accel_mps2, max_accel_mps2, center, forward, reverse):
    """Map signed model acceleration to neutral-centered throttle PWM."""
    accel_mps2 = bounded(accel_mps2, -max_accel_mps2, max_accel_mps2)
    if accel_mps2 >= 0.0:
        return round(
            center + accel_mps2 / max_accel_mps2 * (forward - center)
        )
    return round(
        center + -accel_mps2 / max_accel_mps2 * (reverse - center)
    )


def speed_to_pwm(speed_mps, max_speed_kph, center, forward):
    """Map requested forward speed to a calibrated throttle endpoint."""
    speed_kph = bounded(speed_mps * 3.6, 0.0, max_speed_kph)
    if speed_kph <= 1e-6:
        return center
    return round(center + speed_kph / max_speed_kph * (forward - center))


class MPCActuator(Node):
    """Convert fresh MPC command messages into bounded CRSF output."""

    def __init__(self, config):
        super().__init__('mpc_actuator')
        self.config = config
        self.command_pub = self.create_publisher(RCMessage, COMMAND_TOPIC, 10)
        self.control_sub = self.create_subscription(
            AckermannCommand,
            config.input_topic,
            self.control_callback,
            10,
        )
        self.arming_client = self.create_client(CommandBool, ARMING_SERVICE)
        self.last_control_time = None
        self.throttle = NEUTRAL_VALUE
        self.roll = NEUTRAL_VALUE
        self.output_enabled = False
        self.armed = False
        self.timeout_reported = False
        self.create_timer(1.0 / UPDATE_RATE_HZ, self.publish_command)

    def control_callback(self, command):
        """Store the latest finite MPC command after applying calibrations."""
        if not (
            math.isfinite(command.steering_angle_rad)
            and math.isfinite(command.acceleration_mps2)
            and math.isfinite(command.target_speed_mps)
        ):
            self.get_logger().error('ignoring non-finite MPC command')
            return
        self.roll = steer_to_pwm(
            command.steering_angle_rad,
            self.config.max_steer_rad,
            self.config.center_steering_pwm,
            self.config.left_pwm,
            self.config.right_pwm,
        )
        if command.target_speed_mps > STOP_SPEED_EPS_MPS:
            self.throttle = speed_to_pwm(
                command.target_speed_mps,
                self.config.max_speed_kph,
                self.config.neutral_throttle_pwm,
                self.config.forward_pwm,
            )
        elif command.target_speed_mps < -STOP_SPEED_EPS_MPS:
            self.throttle = accel_to_pwm(
                command.acceleration_mps2,
                self.config.max_accel_mps2,
                self.config.neutral_throttle_pwm,
                self.config.forward_pwm,
                self.config.reverse_pwm,
            )
        else:
            self.throttle = self.config.neutral_throttle_pwm
        self.last_control_time = time.monotonic()
        self.timeout_reported = False

    def command_is_fresh(self):
        """Return whether a recent controller message is available."""
        return (
            self.last_control_time is not None
            and time.monotonic() - self.last_control_time
            <= self.config.command_timeout
        )

    def neutral_message(self):
        """Create a centered and stopped RC command."""
        command = RCMessage()
        command.rc_throttle = NEUTRAL_VALUE
        command.rc_roll = self.config.center_steering_pwm
        command.rc_pitch = self.config.neutral_throttle_pwm
        command.rc_yaw = NEUTRAL_VALUE
        return command

    def publish_command(self):
        """Publish output, neutralizing and disarming after command loss."""
        if not self.output_enabled or not self.command_is_fresh():
            self.command_pub.publish(self.neutral_message())
            if self.output_enabled and not self.timeout_reported:
                self.timeout_reported = True
                self.output_enabled = False
                self.get_logger().error(
                    'MPC command timed out; neutralizing and disarming'
                )
                self.request_disarm()
            return

        command = RCMessage()
        command.rc_throttle = NEUTRAL_VALUE
        command.rc_roll = self.roll
        command.rc_pitch = self.throttle
        command.rc_yaw = NEUTRAL_VALUE
        self.command_pub.publish(command)

    def publish_neutral_for(self, duration):
        """Continuously publish neutral output for a fixed duration."""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.command_pub.publish(self.neutral_message())
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / UPDATE_RATE_HZ)

    def set_armed(self, armed):
        """Request the CRSF node arming state synchronously."""
        if not self.arming_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f'service unavailable: {ARMING_SERVICE}')
        request = CommandBool.Request()
        request.value = armed
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('arming service did not respond')
        self.armed = armed
        print(future.result().data)

    def request_disarm(self):
        """Request asynchronous disarming from a timer callback."""
        if not self.armed or not self.arming_client.service_is_ready():
            return
        request = CommandBool.Request()
        request.value = False
        self.arming_client.call_async(request)
        self.armed = False


def parse_args(args=None):
    """Read calibration and safety settings while retaining ROS arguments."""
    parser = argparse.ArgumentParser(
        description=(
            'Convert /mpc/ackermann_command output to guarded CRSF commands.'
        )
    )
    parser.add_argument('--input-topic', default=INPUT_TOPIC)
    parser.add_argument('--max-steer-rad', type=float, default=0.523)
    parser.add_argument('--max-accel-mps2', type=float, default=2.0)
    parser.add_argument('--max-speed-kph', type=float, default=10.0)
    parser.add_argument('--left-pwm', type=int, default=TELEOP_LEFT_PWM)
    parser.add_argument('--center-steering-pwm', type=int, default=1500)
    parser.add_argument('--right-pwm', type=int, default=TELEOP_RIGHT_PWM)
    parser.add_argument('--forward-pwm', type=int, default=TELEOP_FORWARD_PWM)
    parser.add_argument('--neutral-throttle-pwm', type=int, default=1500)
    parser.add_argument('--reverse-pwm', type=int, default=TELEOP_MAX_REVERSE_PWM)
    parser.add_argument('--command-timeout', type=float, default=0.3)
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print endpoint mappings without publishing or arming.',
    )
    parser.add_argument(
        '--confirm-propulsion-safe',
        action='store_true',
        help='Required for output; confirms driven wheels cannot injure.',
    )
    return parser.parse_known_args(args)


def validate_config(config):
    """Reject invalid controller calibration before hardware is armed."""
    for label in (
        'left_pwm',
        'center_steering_pwm',
        'right_pwm',
        'forward_pwm',
        'neutral_throttle_pwm',
        'reverse_pwm',
    ):
        value = getattr(config, label)
        if not 988 <= value <= 2012:
            raise SystemExit(f'{label} must be between 988 and 2012')
    if (
        config.max_steer_rad <= 0
        or config.max_accel_mps2 <= 0
        or config.max_speed_kph <= 0
    ):
        raise SystemExit(
            'maximum steering, acceleration, and speed must be positive'
        )
    if config.command_timeout <= 0:
        raise SystemExit('command timeout must be positive')


def print_mapping(config):
    """Display endpoint mapping before potentially enabling hardware."""
    print(
        f'input={config.input_topic} output={COMMAND_TOPIC} '
        f'service={ARMING_SERVICE}'
    )
    print(
        'steering: '
        f'+{config.max_steer_rad:.3f} rad -> {config.left_pwm}, '
        f'0 -> {config.center_steering_pwm}, '
        f'-{config.max_steer_rad:.3f} rad -> {config.right_pwm}'
    )
    print(
        'drive pitch: '
        f'{config.max_speed_kph:.1f} km/h -> {config.forward_pwm}, '
        f'0 km/h -> {config.neutral_throttle_pwm}'
    )
    print(
        'reverse target: '
        f'-{config.max_accel_mps2:.3f} m/s^2 -> {config.reverse_pwm}, '
        f'0 m/s^2 -> {config.neutral_throttle_pwm}'
    )


def main(args=None):
    """Run guarded real-vehicle actuation from MPC command messages."""
    config, ros_args = parse_args(args)
    validate_config(config)
    print_mapping(config)
    if config.dry_run:
        return
    if not config.confirm_propulsion_safe:
        raise SystemExit(
            'refusing MPC actuation: securely restrain driven wheels, '
            'then pass --confirm-propulsion-safe'
        )

    rclpy.init(args=ros_args)
    node = MPCActuator(config)
    try:
        time.sleep(0.2)
        if node.count_subscribers(COMMAND_TOPIC) == 0:
            raise RuntimeError(
                f'no subscriber on {COMMAND_TOPIC}; run crsf_ros first'
            )
        print(f'Waiting for MPC commands on {config.input_topic}.')
        deadline = time.monotonic() + 5.0
        while not node.command_is_fresh() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.command_is_fresh():
            raise RuntimeError(
                f'no command received on {config.input_topic} within 5 seconds'
            )

        print('Sending stop/center command before arming.')
        node.publish_neutral_for(SETTLE_DURATION)
        node.set_armed(True)
        node.output_enabled = True
        print('MPC actuation armed; Ctrl-C stops and disarms.')
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nInterrupted; stopping MPC actuation.')
    finally:
        print('\nReturning to stop/center and disarming.')
        node.output_enabled = False
        try:
            node.publish_neutral_for(SETTLE_DURATION)
        finally:
            try:
                if node.armed:
                    node.set_armed(False)
            finally:
                node.destroy_node()
                rclpy.try_shutdown()


if __name__ == '__main__':
    main()
