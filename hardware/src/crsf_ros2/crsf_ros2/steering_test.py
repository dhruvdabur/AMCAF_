#!/usr/bin/env python3
"""Send a short steering-only CRSF command while keeping throttle neutral."""

import argparse
import time

import serial

from .submodules.crsf import channelsCrsfToChannelsPacket, CRSF_TRANSMITTER


DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUDRATE = 400000
DEFAULT_CENTER = 1500
DEFAULT_LEFT = 1200
DEFAULT_RIGHT = 1800
MIN_CHANNEL_VALUE = 988
NEUTRAL_THROTTLE_VALUE = 1500
UPDATE_RATE_HZ = 50.0


def channel_to_crsf(value):
    """Convert a conventional RC microsecond value to a CRSF channel value."""
    return int(((820 / 512) * (value - 1500)) + 992)


def steering_channels(steering_value):
    """Create channels with steering on channel 1 and throttle neutral."""
    rc_values = [
        steering_value,
        DEFAULT_CENTER,
        NEUTRAL_THROTTLE_VALUE,
        DEFAULT_CENTER,
    ] + [MIN_CHANNEL_VALUE] * 12
    return [channel_to_crsf(value) for value in rc_values]


def steering_packet(steering_value):
    """Create a handset-side CRSF frame addressed to the TX module."""
    return channelsCrsfToChannelsPacket(
        steering_channels(steering_value),
        address=CRSF_TRANSMITTER,
    )


def send_position(device, steering_value, duration):
    """Continuously send one steering position for the requested duration."""
    packet = steering_packet(steering_value)
    period = 1.0 / UPDATE_RATE_HZ
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        device.write(packet)
        time.sleep(period)


def parse_args():
    """Read steering position, serial connection, and endpoint arguments."""
    parser = argparse.ArgumentParser(
        description=(
            'Check a steering servo using CRSF channel 1 only; throttle '
            'remains neutral and steering returns to center '
            'afterward.'
        )
    )
    parser.add_argument('position', choices=('left', 'center', 'right'))
    parser.add_argument('--port', default=DEFAULT_PORT)
    parser.add_argument('--baudrate', type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument('--duration', type=float, default=1.0)
    parser.add_argument('--recenter-duration', type=float, default=0.5)
    parser.add_argument('--left', type=int, default=DEFAULT_LEFT)
    parser.add_argument('--center', type=int, default=DEFAULT_CENTER)
    parser.add_argument('--right', type=int, default=DEFAULT_RIGHT)
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print the CRSF channel values without opening the serial port.',
    )
    return parser.parse_args()


def main():
    """Send a selected steering position and restore the centered position."""
    args = parse_args()
    if args.duration <= 0 or args.recenter_duration < 0:
        raise SystemExit('durations must be positive (recenter may be zero)')

    limits = {'left': args.left, 'center': args.center, 'right': args.right}
    for label, value in limits.items():
        if not MIN_CHANNEL_VALUE <= value <= 2012:
            raise SystemExit(f'{label} value must be between 988 and 2012')

    requested_value = limits[args.position]
    requested_channels = steering_channels(requested_value)
    center_channels = steering_channels(args.center)
    if args.dry_run:
        print(
            f'{args.position}: channel 1={requested_channels[0]}, '
            f'channels={requested_channels}'
        )
        packet_hex = steering_packet(requested_value).hex(' ')
        print(f'{args.position}: packet={packet_hex}')
        print(
            f'recenter: channel 1={center_channels[0]}, '
            f'channels={center_channels}'
        )
        return

    print(
        f'Sending {args.position} steering ({requested_value}) on '
        f'{args.port}; throttle is held neutral.'
    )
    with serial.Serial(args.port, args.baudrate, timeout=2) as device:
        try:
            send_position(device, requested_value, args.duration)
        finally:
            print('Returning steering to center.')
            send_position(device, args.center, args.recenter_duration)


if __name__ == '__main__':
    main()
