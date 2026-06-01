#!/usr/bin/env python3
"""Ramp CRSF throttle from neutral to a test PWM, then restore neutral."""

import argparse
import time
from types import SimpleNamespace

import serial

from .ros2_crsf import BAUDRATE, RosCsrf, SERIAL_PORT
from .submodules.crsf import CRSF_TRANSMITTER, channelsCrsfToChannelsPacket


DEFAULT_CENTER = 1500
NEUTRAL_THROTTLE_VALUE = 1500
AUX_LOW_VALUE = 988
DEFAULT_TARGET_VALUE = 1510
MIN_TEST_VALUE = 1400
MAX_TEST_VALUE = 1600
UPDATE_RATE_HZ = 50.0


def throttle_channels(throttle_value):
    """Use the node conversion with throttle on channel 3."""
    command_state = SimpleNamespace(
        CMDS={
            'roll': DEFAULT_CENTER,
            'pitch': DEFAULT_CENTER,
            'throttle': throttle_value,
            'yaw': DEFAULT_CENTER,
            'aux1': AUX_LOW_VALUE,
            'aux2': AUX_LOW_VALUE,
            'aux3': AUX_LOW_VALUE,
            'aux4': AUX_LOW_VALUE,
        }
    )
    return RosCsrf.pwm_to_csrf(command_state)


def throttle_packet(throttle_value):
    """Create a handset-side CRSF frame addressed to the TX module."""
    return channelsCrsfToChannelsPacket(
        throttle_channels(throttle_value),
        address=CRSF_TRANSMITTER,
    )


def send_value(device, throttle_value, duration):
    """Continuously send one throttle value for the requested duration."""
    packet = throttle_packet(throttle_value)
    period = 1.0 / UPDATE_RATE_HZ
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        device.write(packet)
        time.sleep(period)


def send_ramp(device, target_value, duration):
    """Ramp throttle from neutral to the selected test endpoint."""
    period = 1.0 / UPDATE_RATE_HZ
    sample_count = max(1, int(duration * UPDATE_RATE_HZ))
    for sample in range(sample_count + 1):
        fraction = sample / sample_count
        throttle_value = round(
            NEUTRAL_THROTTLE_VALUE
            + fraction * (target_value - NEUTRAL_THROTTLE_VALUE)
        )
        device.write(throttle_packet(throttle_value))
        time.sleep(period)


def parse_args():
    """Read serial connection and test timing arguments."""
    parser = argparse.ArgumentParser(
        description=(
            'Ramp CRSF channel 3 throttle from neutral (1500) to a test '
            'PWM, then restore neutral throttle.'
        )
    )
    parser.add_argument('--port', default=SERIAL_PORT)
    parser.add_argument('--baudrate', type=int, default=BAUDRATE)
    parser.add_argument('--idle-duration', type=float, default=1.0)
    parser.add_argument('--ramp-duration', type=float, default=2.0)
    parser.add_argument('--hold-duration', type=float, default=1.0)
    parser.add_argument('--shutdown-duration', type=float, default=1.0)
    parser.add_argument(
        '--target-pwm',
        type=int,
        default=DEFAULT_TARGET_VALUE,
        help='Throttle endpoint from neutral; range 1400-1600, default 1510.',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print the low and target commands without opening serial.',
    )
    parser.add_argument(
        '--confirm-propulsion-safe',
        action='store_true',
        help='Required for output; confirms wheels/propellers cannot injure.',
    )
    return parser.parse_args()


def main():
    """Run the guarded throttle ramp and restore neutral afterward."""
    args = parse_args()
    durations = (
        args.idle_duration,
        args.ramp_duration,
        args.hold_duration,
        args.shutdown_duration,
    )
    if any(duration <= 0 for duration in durations):
        raise SystemExit('all durations must be positive')
    if not MIN_TEST_VALUE <= args.target_pwm <= MAX_TEST_VALUE:
        raise SystemExit('target PWM must be between 1400 and 1600')

    if args.dry_run:
        neutral_channels = throttle_channels(NEUTRAL_THROTTLE_VALUE)
        target_channels = throttle_channels(args.target_pwm)
        neutral_packet = throttle_packet(NEUTRAL_THROTTLE_VALUE).hex(' ')
        target_packet = throttle_packet(args.target_pwm).hex(' ')
        print(f'port={args.port} baudrate={args.baudrate}')
        print(
            f'neutral: channel 3={neutral_channels[2]}, '
            f'pwm={NEUTRAL_THROTTLE_VALUE}, packet={neutral_packet}'
        )
        print(
            f'target: channel 3={target_channels[2]}, '
            f'pwm={args.target_pwm}, '
            f'packet={target_packet}'
        )
        print(
            f'sequence: neutral -> ramp to {args.target_pwm} '
            '-> hold -> neutral'
        )
        return

    if not args.confirm_propulsion_safe:
        raise SystemExit(
            'refusing throttle output: disconnect propulsion or secure the '
            'vehicle, then pass --confirm-propulsion-safe'
        )

    print(
        f'Sending throttle test on {args.port}: neutral '
        f'({NEUTRAL_THROTTLE_VALUE}) '
        f'to target ({args.target_pwm}); steering remains centered.'
    )
    with serial.Serial(args.port, args.baudrate, timeout=2) as device:
        try:
            send_value(device, NEUTRAL_THROTTLE_VALUE, args.idle_duration)
            send_ramp(device, args.target_pwm, args.ramp_duration)
            send_value(device, args.target_pwm, args.hold_duration)
        finally:
            print('Returning throttle to neutral.')
            send_value(device, NEUTRAL_THROTTLE_VALUE, args.shutdown_duration)


if __name__ == '__main__':
    main()
