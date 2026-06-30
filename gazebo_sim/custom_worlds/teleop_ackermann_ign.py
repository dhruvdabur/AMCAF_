#!/usr/bin/env python3

import subprocess
import sys
import termios
import tty

TOPIC = "/model/prius/cmd_vel"

speed = 0.0
steer = 0.0

SPEED_STEP = 0.25
STEER_STEP = 0.15

MAX_SPEED = 2.0
MAX_STEER = 0.6


def clamp(value, low, high):
    return max(low, min(high, value))


def publish_cmd(speed, steer):
    msg = f"linear: {{x: {speed:.3f}}}, angular: {{z: {steer:.3f}}}"

    subprocess.run(
        [
            "ign",
            "topic",
            "-t",
            TOPIC,
            "-m",
            "ignition.msgs.Twist",
            "-p",
            msg,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return key


print("""
Ackermann Keyboard Teleop

w : increase speed
x : decrease speed / reverse
a : steer left
d : steer right
s : straighten steering
space : stop
q : quit
""")

publish_cmd(0.0, 0.0)

while True:
    key = get_key()

    if key == "w":
        speed += SPEED_STEP

    elif key == "x":
        speed -= SPEED_STEP

    elif key == "a":
        steer += STEER_STEP

    elif key == "d":
        steer -= STEER_STEP

    elif key == "s":
        steer = 0.0

    elif key == " ":
        speed = 0.0
        steer = 0.0

    elif key == "q":
        speed = 0.0
        steer = 0.0
        publish_cmd(speed, steer)
        print("\nStopped.")
        break

    speed = clamp(speed, -MAX_SPEED, MAX_SPEED)
    steer = clamp(steer, -MAX_STEER, MAX_STEER)

    publish_cmd(speed, steer)
    print(f"\rspeed: {speed:.2f} m/s | steer: {steer:.2f} rad", end="")
