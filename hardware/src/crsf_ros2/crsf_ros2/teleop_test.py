#!/usr/bin/env python3
"""Drive a guarded CRSF bench test from a terminal using WASD keys."""

import argparse
import select
import sys
import termios
import time
import tty
from contextlib import contextmanager

from rc_msgs.msg import RCMessage
from rc_msgs.srv import CommandBool

import rclpy
from rclpy.node import Node


COMMAND_TOPIC = '/drone/rc_command'
ARMING_SERVICE = '/drone/cmd/arming'
NEUTRAL_VALUE = 1500
MAX_FORWARD_THROTTLE = 1600
MAX_REVERSE_THROTTLE = 1400
MAX_LEFT_ROLL = 1700
MAX_RIGHT_ROLL = 1300
THROTTLE_STEP = 5
ROLL_STEP = 25
UPDATE_RATE_HZ = 50.0
SETTLE_DURATION = 0.5


class TeleopTest(Node):
    """Publish bounded driving commands and operate the arming service."""

    def __init__(self):
        """Create topic and service endpoints with a neutral command state."""
        super().__init__('teleop_test')
        self.command_pub = self.create_publisher(RCMessage, COMMAND_TOPIC, 10)
        self.arming_client = self.create_client(CommandBool, ARMING_SERVICE)
        self.throttle = NEUTRAL_VALUE
        self.roll = NEUTRAL_VALUE

    def publish_command(self):
        """Publish the current bounded throttle and roll values."""
        command = RCMessage()
        command.rc_throttle = self.throttle
        command.rc_roll = self.roll
        command.rc_pitch = NEUTRAL_VALUE
        command.rc_yaw = NEUTRAL_VALUE
        self.command_pub.publish(command)

    def publish_for(self, duration):
        """Publish the current state continuously for a fixed duration."""
        period = 1.0 / UPDATE_RATE_HZ
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def set_neutral(self):
        """Center steering and command zero throttle."""
        self.throttle = NEUTRAL_VALUE
        self.roll = NEUTRAL_VALUE

    def set_armed(self, armed):
        """Request the running CRSF node's configured arming state."""
        if not self.arming_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f'service unavailable: {ARMING_SERVICE}')
        request = CommandBool.Request()
        request.value = armed
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('arming service did not respond')
        print(future.result().data)

    def handle_key(self, key):
        """Update the command from one terminal key; return false to quit."""
        if key == 'w':
            self.throttle = min(
                self.throttle + THROTTLE_STEP, MAX_FORWARD_THROTTLE
            )
        elif key == 's':
            self.throttle = max(
                self.throttle - THROTTLE_STEP, MAX_REVERSE_THROTTLE
            )
        elif key == 'a':
            self.roll = min(self.roll + ROLL_STEP, MAX_LEFT_ROLL)
        elif key == 'd':
            self.roll = max(self.roll - ROLL_STEP, MAX_RIGHT_ROLL)
        elif key == 'c':
            self.roll = NEUTRAL_VALUE
        elif key == ' ':
            self.set_neutral()
        elif key in ('q', '\x03'):
            return False
        else:
            return True

        print(
            f'\rthrottle={self.throttle} roll={self.roll}       ',
            end='',
            flush=True,
        )
        return True


@contextmanager
def keyboard_mode():
    """Yield while terminal input is available one character at a time."""
    if not sys.stdin.isatty():
        raise RuntimeError('teleop requires an interactive terminal')
    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)


def parse_args(args=None):
    """Read safety acknowledgement while preserving ROS-specific arguments."""
    parser = argparse.ArgumentParser(
        description=(
            'Keyboard test control: W/S adjust throttle within 1400..1600; '
            'A/D adjust roll steering within 1300..1700.'
        )
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print controls and limits without publishing or arming.',
    )
    parser.add_argument(
        '--confirm-propulsion-safe',
        action='store_true',
        help='Required for output; confirms wheels/propulsion cannot injure.',
    )
    return parser.parse_known_args(args)


def print_controls():
    """Display key bindings and all configured command bounds."""
    print(
        'W/S: throttle +/-5 '
        f'({MAX_REVERSE_THROTTLE}=max reverse, {NEUTRAL_VALUE}=stop, '
        f'{MAX_FORWARD_THROTTLE}=max forward)'
    )
    print(
        'A/D: steering left/right +/-25 '
        f'({MAX_RIGHT_ROLL}=max right, {NEUTRAL_VALUE}=center, '
        f'{MAX_LEFT_ROLL}=max left)'
    )
    print('C: center steering    SPACE: stop and center    Q: stop and quit')


def main(args=None):
    """Arm at neutral and run bounded terminal teleoperation until stopped."""
    config, ros_args = parse_args(args)
    print_controls()
    if config.dry_run:
        return
    if not config.confirm_propulsion_safe:
        raise SystemExit(
            'refusing armed teleop output: disconnect or securely restrain '
            'propulsion, then pass --confirm-propulsion-safe'
        )

    rclpy.init(args=ros_args)
    node = TeleopTest()
    arm_requested = False
    try:
        time.sleep(0.2)
        if node.count_subscribers(COMMAND_TOPIC) == 0:
            raise RuntimeError(
                f'no subscriber on {COMMAND_TOPIC}; run crsf_ros first'
            )
        print('Sending stop/center command before arming.')
        node.publish_for(SETTLE_DURATION)
        arm_requested = True
        node.set_armed(True)
        node.publish_for(SETTLE_DURATION)
        print('Teleop armed; press SPACE or Q to stop.')
        period = 1.0 / UPDATE_RATE_HZ
        with keyboard_mode():
            running = True
            while running:
                readable, _, _ = select.select([sys.stdin], [], [], period)
                if readable:
                    running = node.handle_key(sys.stdin.read(1).lower())
                node.publish_command()
                rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        print('\nInterrupted; stopping teleop.')
    finally:
        print('\nReturning to stop/center and disarming.')
        try:
            node.set_neutral()
            node.publish_for(SETTLE_DURATION)
        finally:
            try:
                if arm_requested:
                    node.set_armed(False)
            finally:
                node.destroy_node()
                rclpy.try_shutdown()


if __name__ == '__main__':
    main()
