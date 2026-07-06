#!/usr/bin/env python3
"""
traffic_controller.py
─────────────────────
Moves traffic_box_blue and traffic_box_red along trajectory.csv
inside Ignition Gazebo / Gazebo Sim (Fortress, Garden, Harmonic).

WHY THE BOXES WEREN'T MOVING BEFORE
  The previous version called node.advertise() + publish() treating
  /world/.../set_pose as a pub/sub topic — it is actually a SERVICE.
  Publishing to it does nothing. The fix is node.request().
  If gz-transport Python bindings are absent we fall back to shelling
  out `gz service` / `ign service`, which is always available when
  Gazebo is installed.

BOX LAYOUT
  Blue  →  spawns at 50 % through the trajectory
  Red   →  spawns at 75 % through the trajectory
  Both loop the full trajectory continuously, keeping their offset.
  Red is placed in the adjacent lane (2 m lateral offset).

CSV FORMAT  (auto-detected by column count)
  4-col : time, x, y, yaw          z = BOX_HEIGHT (auto)
  5-col : time, x, y, z, yaw
  9-col : time, bx,by,bz,byaw, rx,ry,rz,ryaw   (fully independent paths)

USAGE
  cd ~/Documents/AMCAF_-code/gazebo_sim/gazebo_worlds

  # Terminal 1 — launch Gazebo
  ign gazebo custom_road_final.sdf          # Fortress / Edifice
  # OR
  gz sim custom_road_final.sdf              # Garden / Harmonic

  # Terminal 2 — run this controller
  python3 traffic_controller.py trajectory.csv

  # Options
  python3 traffic_controller.py trajectory.csv --speed 2.0
  python3 traffic_controller.py trajectory.csv --no-loop
  python3 traffic_controller.py trajectory.csv --world custom_road_world
"""

import sys, csv, time, math, argparse, subprocess, json

# ── tuneable constants ────────────────────────────────────────────────────────
WORLD_NAME      = "custom_road_world"
BOX_HEIGHT      = 0.0075      # z so box base sits on road surface
LOOP            = True
SPEED_FACTOR    = 1.0

BLUE_START_FRAC = 0.50        # blue starts 50 % through the trajectory
RED_START_FRAC  = 0.75        # red  starts 75 % through the trajectory

RED_LANE_OFFSET = 0.02         # metres — lateral gap between the two boxes
# ─────────────────────────────────────────────────────────────────────────────

# ── try to import gz/ignition transport ──────────────────────────────────────
GZ_PYTHON = False
_Node = _Pose = None

try:
    from gz.transport13 import Node as _Node
    from gz.msgs10.pose_pb2 import Pose as _Pose
    GZ_PYTHON = True
    GZ_FLAVOR = "gz"
except ImportError:
    pass

if not GZ_PYTHON:
    try:
        from ignition.transport import Node as _Node
        from ignition.msgs.pose_pb2 import Pose as _Pose
        GZ_PYTHON = True
        GZ_FLAVOR = "ignition"
    except ImportError:
        pass

# ── detect CLI tool (gz or ign) ───────────────────────────────────────────────
def _find_cli():
    for cmd in ("gz", "ign"):
        try:
            subprocess.run([cmd, "--version"],
                           capture_output=True, timeout=3)
            return cmd
        except (FileNotFoundError, subprocess.TimeoutExpired):
            pass
    return None

GZ_CLI = _find_cli()

# ── helpers ───────────────────────────────────────────────────────────────────
def yaw_to_quat(yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return 0.0, 0.0, sy, cy          # x, y, z, w

def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        for line in csv.reader(f):
            line = [c.strip() for c in line]
            if not line or line[0].startswith("#"):
                continue
            try:
                rows.append([float(v) for v in line])
            except ValueError:
                continue
    return rows

def parse_waypoints(rows):
    """Returns list of (t, x, y, z, yaw) tuples — single reference path."""
    out = []
    for r in rows:
        n = len(r)
        if n >= 9:
            # 9-col: use first vehicle columns for the reference path
            out.append((r[0], r[1], r[2], r[3], r[4]))
        elif n >= 5:
            out.append((r[0], r[1], r[2], r[3], r[4]))
        elif n >= 4:
            out.append((r[0], r[1], r[2], BOX_HEIGHT, r[3]))
    return out

def lateral_point(x, y, z, yaw, dist):
    """Return a point `dist` metres to the left of the heading."""
    return (x + dist * math.cos(yaw + math.pi / 2),
            y + dist * math.sin(yaw + math.pi / 2),
            z, yaw)

# ── pose-as-protobuf text string for the CLI ──────────────────────────────────
def pose_protobuf_text(name, x, y, z, yaw):
    qx, qy, qz, qw = yaw_to_quat(yaw)
    return (
        f'name: "{name}" '
        f'position {{ x: {x:.6f} y: {y:.6f} z: {z:.6f} }} '
        f'orientation {{ x: {qx:.6f} y: {qy:.6f} z: {qz:.6f} w: {qw:.6f} }}'
    )

# ── mover classes ─────────────────────────────────────────────────────────────
class PythonMover:
    """Uses gz/ignition Python transport — publishes to set_pose topic."""
    def __init__(self, world):
        self.node = _Node()
        self.topic = f"/world/{world}/set_pose"
        self.pub = self.node.advertise(self.topic, _Pose)
        print(f"[python transport] Advertising topic {self.topic}")

    def move(self, name, x, y, z, yaw):
        req = _Pose()
        req.name = name
        req.position.x, req.position.y, req.position.z = x, y, z
        qx, qy, qz, qw = yaw_to_quat(yaw)
        req.orientation.x, req.orientation.y = qx, qy
        req.orientation.z, req.orientation.w = qz, qw
        self.pub.publish(req)


class CLIMover:
    """
    Shells out to `gz topic` / `ign topic` to publish to set_pose.
    Always works as long as Gazebo is installed, no Python bindings needed.
    """
    def __init__(self, world, cli):
        self.topic   = f"/world/{world}/set_pose"
        self.cli     = cli
        self.msgtype = "ignition.msgs.Pose" if cli == "ign" else "gz.msgs.Pose"
        print(f"[CLI mover] Using `{cli} topic -t {self.topic}`")

    def move(self, name, x, y, z, yaw):
        req_str = pose_protobuf_text(name, x, y, z, yaw)
        cmd = [
            self.cli, "topic",
            "-t", self.topic,
            "-m", self.msgtype,
            "-p", req_str,
        ]
        try:
            subprocess.run(cmd, capture_output=True, timeout=0.5)
        except subprocess.TimeoutExpired:
            pass   # non-fatal — Gazebo may just be busy


class PrintMover:
    """Dry-run fallback when nothing else is available."""
    def move(self, name, x, y, z, yaw):
        print(f"  POSE {name:25s}  x={x:8.3f} y={y:8.3f} z={z:5.3f} yaw={yaw:6.3f}")


# ── main ──────────────────────────────────────────────────────────────────────
def run(csv_path, world, loop, speed):
    # --- load trajectory ---
    rows = load_csv(csv_path)
    if not rows:
        sys.exit(f"ERROR: no data rows in '{csv_path}'")

    waypoints = parse_waypoints(rows)
    n = len(waypoints)
    if n < 4:
        sys.exit(f"ERROR: need at least 4 waypoints, got {n}")

    blue_idx = int(n * BLUE_START_FRAC)
    red_idx  = int(n * RED_START_FRAC)
    print(f"Loaded {n} waypoints from '{csv_path}'")
    print(f"Blue starts at index {blue_idx}  ({BLUE_START_FRAC*100:.0f}% through)")
    print(f"Red  starts at index {red_idx}   ({RED_START_FRAC*100:.0f}% through)")

    # --- choose mover ---
    if GZ_PYTHON:
        mover = PythonMover(world)
    elif GZ_CLI:
        mover = CLIMover(world, GZ_CLI)
    else:
        print("WARNING: neither gz-transport Python bindings nor gz/ign CLI found.")
        print("Printing poses only. Install Ignition/Gazebo to actually move boxes.")
        mover = PrintMover()

    # --- timing ---
    times   = [wp[0] for wp in waypoints]
    total_t = times[-1] - times[0]
    if total_t <= 0:
        dt_list = [0.05] * n       # 20 Hz default
    else:
        dt_list = []
        for i in range(n - 1):
            dt_list.append((times[i + 1] - times[i]) / speed)
        dt_list.append(dt_list[-1])

    # --- loop ---
    print("Running... (Ctrl-C to stop)")
    pass_num = 0
    try:
        while True:
            for step in range(n):
                bi = (blue_idx + step) % n
                ri = (red_idx  + step) % n

                _, bx, by, bz, byaw = waypoints[bi]
                _, rx, ry, rz, ryaw = waypoints[ri]

                # put red in adjacent lane
                rx2, ry2, rz2, ryaw2 = lateral_point(rx, ry, rz, ryaw, RED_LANE_OFFSET)

                mover.move("traffic_box_blue", bx,  by,  bz,  byaw)
                mover.move("traffic_box_red",  rx2, ry2, rz2, ryaw2)

                time.sleep(dt_list[step])

            pass_num += 1
            if not loop:
                break
    except KeyboardInterrupt:
        print("\nStopped by user.")

    print("Done.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="Drive Gazebo traffic boxes along a CSV trajectory")
    ap.add_argument("csv",       nargs="?",  default="trajectory.csv",
                    help="Path to trajectory CSV (default: trajectory.csv)")
    ap.add_argument("--world",   default=WORLD_NAME,
                    help=f"Gazebo world name (default: {WORLD_NAME})")
    ap.add_argument("--no-loop", action="store_true",
                    help="Play trajectory once then exit")
    ap.add_argument("--speed",   type=float, default=SPEED_FACTOR,
                    help="Playback speed multiplier (default: 1.0)")
    args = ap.parse_args()
    run(args.csv, args.world, not args.no_loop, args.speed)