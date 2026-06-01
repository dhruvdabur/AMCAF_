#!/usr/bin/env python3
"""Exercise throttle around neutral through the running CRSF ROS node."""

import argparse
import time

from rc_msgs.msg import RCMessage
from rc_msgs.srv import CommandBool

import rclpy
from rclpy.node import Node


COMMAND_TOPIC = '/drone/rc_command'
ARMING_SERVICE = '/drone/cmd/arming'
DEFAULT_CENTER = 1500
NEUTRAL_THROTTLE_VALUE = 1500
DEFAULT_TARGET_THROTTLE_VALUE = 1510
MIN_TEST_THROTTLE_VALUE = 1400
MAX_TEST_THROTTLE_VALUE = 1600
UPDATE_RATE_HZ = 50.0


class ThrottleTopicTest(Node):
    """Publish neutral-centered RC commands and invoke the arming service."""

    def __init__(self):
        """Create the command publisher and arming client."""
        super().__init__('throttle_topic_test')
        self.command_pub = self.create_publisher(RCMessage, COMMAND_TOPIC, 10)
        self.arming_client = self.create_client(CommandBool, ARMING_SERVICE)

    def send_throttle(self, throttle_value, duration):
        """Publish throttle while holding steering axes centered."""
        command = RCMessage()
        command.rc_throttle = throttle_value
        command.rc_roll = DEFAULT_CENTER
        command.rc_pitch = DEFAULT_CENTER
        command.rc_yaw = DEFAULT_CENTER
        period = 1.0 / UPDATE_RATE_HZ
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.command_pub.publish(command)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def send_ramp(self, target_value, duration):
        """Ramp from neutral throttle to the requested endpoint."""
        period = 1.0 / UPDATE_RATE_HZ
        sample_count = max(1, int(duration * UPDATE_RATE_HZ))
        for sample in range(sample_count + 1):
            fraction = sample / sample_count
            throttle_value = round(
                NEUTRAL_THROTTLE_VALUE
                + fraction * (target_value - NEUTRAL_THROTTLE_VALUE)
            )
            self.send_throttle(throttle_value, period)

    def set_armed(self, armed):
        """Request the node's configured arming-switch state."""
        if not self.arming_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f'service unavailable: {ARMING_SERVICE}')
        request = CommandBool.Request()
        request.value = armed
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('arming service did not respond')
        print(future.result().data)


def parse_args(args=None):
    """Read safety acknowledgement and test timing arguments."""
    parser = argparse.ArgumentParser(
        description=(
            'Use the running crsf_ros node to arm at neutral throttle, ramp '
            'channel 3 to a bench-test PWM, then return neutral and disarm.'
        )
    )
    parser.add_argument('--idle-duration', type=float, default=1.0)
    parser.add_argument('--arm-settle-duration', type=float, default=1.0)
    parser.add_argument('--ramp-duration', type=float, default=2.0)
    parser.add_argument('--hold-duration', type=float, default=0.5)
    parser.add_argument('--shutdown-duration', type=float, default=2.0)
    parser.add_argument(
        '--target-pwm',
        type=int,
        default=DEFAULT_TARGET_THROTTLE_VALUE,
        help='Ramp endpoint; accepted range is 1400 to 1600 (default: 1510).',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Describe the command sequence without publishing or arming.',
    )
    parser.add_argument(
        '--confirm-propulsion-safe',
        action='store_true',
        help=(
            'Required for output; confirms propellers are removed or secured.'
        ),
    )
    return parser.parse_known_args(args)


def main(args=None):
    """Execute a guarded throttle bench test through the existing ROS node."""
    config, ros_args = parse_args(args)
    durations = (
        config.idle_duration,
        config.arm_settle_duration,
        config.ramp_duration,
        config.hold_duration,
        config.shutdown_duration,
    )
    if any(duration <= 0 for duration in durations):
        raise SystemExit('all durations must be positive')
    if not (
        MIN_TEST_THROTTLE_VALUE
        <= config.target_pwm
        <= MAX_TEST_THROTTLE_VALUE
    ):
        raise SystemExit('target PWM must be between 1400 and 1600')

    if config.dry_run:
        print(f'topic={COMMAND_TOPIC} service={ARMING_SERVICE}')
        print(
            f'sequence: neutral -> arm -> ramp to {config.target_pwm} '
            '-> neutral -> disarm'
        )
        return
    if not config.confirm_propulsion_safe:
        raise SystemExit(
            'refusing armed throttle output: remove propellers or securely '
            'restrain propulsion, then pass --confirm-propulsion-safe'
        )

    rclpy.init(args=ros_args)
    test_node = ThrottleTopicTest()
    arm_requested = False
    try:
        time.sleep(0.2)
        if test_node.count_subscribers(COMMAND_TOPIC) == 0:
            raise RuntimeError(
                f'no subscriber on {COMMAND_TOPIC}; run crsf_ros first'
            )
        print('Sending neutral throttle before arming.')
        test_node.send_throttle(NEUTRAL_THROTTLE_VALUE, config.idle_duration)
        arm_requested = True
        test_node.set_armed(True)
        test_node.send_throttle(
            NEUTRAL_THROTTLE_VALUE, config.arm_settle_duration
        )
        print(f'Armed at neutral throttle; ramping to {config.target_pwm}.')
        test_node.send_ramp(config.target_pwm, config.ramp_duration)
        test_node.send_throttle(config.target_pwm, config.hold_duration)
    except KeyboardInterrupt:
        print('Interrupted; stopping throttle test.')
    finally:
        print('Returning throttle to neutral and disarming.')
        try:
            test_node.send_throttle(
                NEUTRAL_THROTTLE_VALUE, config.shutdown_duration
            )
        finally:
            try:
                if arm_requested:
                    test_node.set_armed(False)
            finally:
                test_node.destroy_node()
                rclpy.try_shutdown()


if __name__ == '__main__':
    main()
