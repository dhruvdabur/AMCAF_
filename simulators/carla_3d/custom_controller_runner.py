"""
Run a custom controller on a CARLA vehicle.

Start CARLA first from the repository root:
    cd third_party/CARLA_0.9.15
    ./CarlaUE4.sh -quality-level=Low

Then run from the repository root:
    python3 simulators/carla_3d/custom_controller_runner.py --road-only

The easiest place to write your own controller is CustomRoadController.update().
It receives a CarlaState and returns (steer_rad, accel_mps2).
"""

from __future__ import annotations

import argparse
import glob
import math
import random
import sys
import time
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
_CARLA_ROOT = REPO_ROOT / "third_party" / "CARLA_0.9.15"
_CARLA_EGG_GLOB = (
    _CARLA_ROOT
    / "PythonAPI"
    / "carla"
    / "dist"
    / f"carla-*{sys.version_info.major}.{sys.version_info.minor}-*.egg"
)
import numpy as np


carla = None


REPO_SRC = REPO_ROOT / "src" / "av_control_guide" / "src"
CARLA_MAPS = _CARLA_ROOT / "CarlaUE4" / "Content" / "Carla" / "Maps"
COMPONENTS = REPO_SRC / "components"
sys.path.append(str(COMPONENTS / "control" / "mpc"))
sys.path.append(str(COMPONENTS / "vehicle"))

from mpc_controller1 import MPCController1  # noqa: E402
from vehicle_specification import VehicleSpecification  # noqa: E402


@dataclass
class CarlaState:
    x_m: float
    y_m: float
    yaw_rad: float
    speed_mps: float

    @classmethod
    def from_vehicle(cls, vehicle: carla.Vehicle) -> "CarlaState":
        transform = vehicle.get_transform()
        velocity = vehicle.get_velocity()
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        return cls(
            x_m=transform.location.x,
            y_m=transform.location.y,
            yaw_rad=math.radians(transform.rotation.yaw),
            speed_mps=speed,
        )

    def get_x_m(self) -> float:
        return self.x_m

    def get_y_m(self) -> float:
        return self.y_m

    def get_yaw_rad(self) -> float:
        return self.yaw_rad

    def get_speed_mps(self) -> float:
        return self.speed_mps


class CarlaWaypointCourse:
    """Course adapter with the methods expected by MPCController1."""

    def __init__(self, waypoints: list[carla.Waypoint], target_speed_mps: float):
        if len(waypoints) < 2:
            raise ValueError("Need at least two waypoints for a course.")
        self._x = [wp.transform.location.x for wp in waypoints]
        self._y = [wp.transform.location.y for wp in waypoints]
        self._yaw = [math.radians(wp.transform.rotation.yaw) for wp in waypoints]
        self._speed = [target_speed_mps for _ in waypoints]

    def length(self) -> int:
        return len(self._x)

    def point_x_m(self, point_index: int) -> float:
        return self._x[point_index]

    def point_y_m(self, point_index: int) -> float:
        return self._y[point_index]

    def point_yaw_rad(self, point_index: int) -> float:
        return self._yaw[point_index]

    def point_speed_mps(self, point_index: int) -> float:
        return self._speed[point_index]

    def location(self, point_index: int, z_offset: float = 0.4) -> carla.Location:
        return carla.Location(
            x=self._x[point_index],
            y=self._y[point_index],
            z=z_offset,
        )


class CustomRoadController:
    """
    Example custom controller.

    Replace update() with your own control law. Keep the return contract:
        (steering angle in radians, acceleration in m/s^2)
    """

    def __init__(
        self,
        course: CarlaWaypointCourse,
        wheel_base_m: float = 2.8,
        target_speed_mps: float = 6.0,
        lookahead_m: float = 8.0,
    ):
        self.course = course
        self.wheel_base_m = wheel_base_m
        self.target_speed_mps = target_speed_mps
        self.lookahead_m = lookahead_m
        self._nearest_idx = 0

    def update(self, state: CarlaState, time_s: float) -> tuple[float, float]:
        target_idx = self._target_index(state)
        tx = self.course.point_x_m(target_idx)
        ty = self.course.point_y_m(target_idx)

        dx = tx - state.get_x_m()
        dy = ty - state.get_y_m()
        alpha = _wrap_to_pi(math.atan2(dy, dx) - state.get_yaw_rad())
        steer_rad = math.atan2(
            2.0 * self.wheel_base_m * math.sin(alpha),
            max(self.lookahead_m, 1e-3),
        )

        speed_error = self.target_speed_mps - state.get_speed_mps()
        accel_mps2 = float(np.clip(0.8 * speed_error, -3.0, 2.0))
        return steer_rad, accel_mps2

    def _target_index(self, state: CarlaState) -> int:
        search_end = min(self.course.length(), self._nearest_idx + 80)
        best_idx = self._nearest_idx
        best_dist = float("inf")

        for idx in range(self._nearest_idx, search_end):
            dist = math.hypot(
                self.course.point_x_m(idx) - state.get_x_m(),
                self.course.point_y_m(idx) - state.get_y_m(),
            )
            if dist < best_dist:
                best_dist = dist
                best_idx = idx

        self._nearest_idx = best_idx
        target_idx = best_idx
        while target_idx + 1 < self.course.length():
            dist = math.hypot(
                self.course.point_x_m(target_idx) - state.get_x_m(),
                self.course.point_y_m(target_idx) - state.get_y_m(),
            )
            if dist >= self.lookahead_m:
                break
            target_idx += 1
        return target_idx


class MPCControllerAdapter:
    """Adapter that runs your existing MPCController1 against CARLA state."""

    def __init__(self, course: CarlaWaypointCourse, target_speed_mps: float):
        spec = VehicleSpecification(f_len_m=2.8, r_len_m=0.0, max_accel_mps2=2.0)
        self._controller = MPCController1(
            spec,
            course,
            delta_t=0.1,
            horizon_step_T=10,
            max_steer_abs=0.523,
            max_accel_abs=2.0,
            v_min=0.0,
            v_max=max(target_speed_mps + 2.0, 8.0),
            ipopt_max_iter=40,
            visualize_optimal_traj=False,
        )

    def update(self, state: CarlaState, time_s: float) -> tuple[float, float]:
        self._controller.update(state, time_s)
        return (
            self._controller.get_target_steer_rad(),
            self._controller.get_target_accel_mps2(),
        )


def main() -> None:
    global carla

    args = _parse_args()
    carla = _import_carla()
    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()

    original_settings = world.get_settings()

    vehicle = None
    traffic_vehicles = []
    try:
        if args.road_only and args.road_only_mode == "opendrive":
            world = _load_road_only_opendrive_world(client, args.map)
        elif args.map and args.map not in world.get_map().name:
            world = client.load_world(args.map)

        if args.sync:
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = args.dt
            world.apply_settings(settings)

        if args.road_only and args.road_only_mode == "layers":
            _strip_world_to_roads(world)

        traffic_manager = client.get_trafficmanager(args.traffic_manager_port)
        traffic_manager.set_synchronous_mode(args.sync)
        traffic_manager.set_global_distance_to_leading_vehicle(
            args.traffic_follow_distance
        )
        traffic_manager.global_percentage_speed_difference(
            args.traffic_speed_difference
        )

        vehicle, course = _spawn_vehicle_and_course(
            world=world,
            spawn_index=args.spawn_index,
            route_distance_m=args.route_distance,
            waypoint_gap_m=args.waypoint_gap,
            target_speed_mps=args.target_speed,
        )
        traffic_vehicles = _spawn_traffic_vehicles(
            world=world,
            traffic_manager_port=args.traffic_manager_port,
            count=args.traffic_vehicles,
            seed=args.traffic_seed,
            skip_transform=vehicle.get_transform(),
        )
        if args.debug_draw_route:
            _draw_course_debug(world, course, life_time=max(args.duration, 10.0))

        if args.controller == "mpc":
            controller = MPCControllerAdapter(course, args.target_speed)
        else:
            controller = CustomRoadController(course, target_speed_mps=args.target_speed)

        print(
            f"Spawned {vehicle.type_id} on {world.get_map().name}. "
            f"Controller={args.controller}. Route points={course.length()}."
        )
        _control_loop(world, vehicle, controller, args)

    finally:
        traffic_manager = client.get_trafficmanager(args.traffic_manager_port)
        traffic_manager.set_synchronous_mode(False)
        _destroy_actors(client, [vehicle, *traffic_vehicles])
        if args.sync:
            world.apply_settings(original_settings)


def _strip_world_to_roads(world: carla.World) -> None:
    """Unload heavy visual map layers while keeping the driveable road network."""

    layers_to_unload = [
        ("Buildings", "Buildings"),
        ("Foliage", "Foliage"),
        ("ParkedVehicles", "ParkedVehicles"),
        ("Particles", "Particles"),
        ("Props", "Props"),
        ("StreetLights", "StreetLights"),
        ("Walls", "Walls"),
    ]

    unloaded = []
    unsupported = []
    for layer_name, label in layers_to_unload:
        layer = getattr(carla.MapLayer, layer_name, None)
        if layer is None:
            unsupported.append(label)
            continue
        try:
            world.unload_map_layer(layer)
            unloaded.append(label)
        except RuntimeError:
            unsupported.append(label)

    if unloaded:
        print("Road-only mode unloaded map layers: " + ", ".join(unloaded))
    if unsupported:
        print(
            "Road-only mode could not unload these layers on this map/API: "
            + ", ".join(unsupported)
        )


def _import_carla():
    """Import CARLA after argparse so --help still works without the egg installed."""

    for egg_path in glob.glob(str(_CARLA_EGG_GLOB)):
        sys.path.append(egg_path)
        break

    try:
        import carla as carla_module
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "Could not import the CARLA Python API. Install a CARLA 0.9.15 "
            "Python package matching your Python version, or run with a Python "
            "version supported by the CARLA distribution in third_party/."
        ) from exc
    return carla_module


def _load_road_only_opendrive_world(client: carla.Client, map_name: str) -> carla.World:
    """Generate a lightweight road-only CARLA world from a bundled OpenDRIVE map."""

    normalized_map = _normalized_map_name(map_name)
    xodr_path = CARLA_MAPS / "OpenDrive" / f"{normalized_map}.xodr"
    if not xodr_path.exists():
        raise FileNotFoundError(
            f"Could not find OpenDRIVE map {xodr_path}. "
            "Use --road-only-mode layers or choose a map that has a bundled .xodr file."
        )

    xodr_data = xodr_path.read_text(encoding="utf-8")
    params = carla.OpendriveGenerationParameters(
        vertex_distance=2.0,
        max_road_length=500.0,
        wall_height=0.0,
        additional_width=0.6,
        smooth_junctions=True,
        enable_mesh_visibility=True,
    )
    print(f"Road-only mode generating OpenDRIVE world from {xodr_path.name}.")
    return client.generate_opendrive_world(xodr_data, params)


def _normalized_map_name(map_name: str) -> str:
    if not map_name:
        return "Town01"
    name = map_name.rsplit("/", maxsplit=1)[-1]
    if name.endswith("_Opt"):
        return name[: -len("_Opt")]
    return name


def _spawn_vehicle_and_course(
    world: carla.World,
    spawn_index: int,
    route_distance_m: float,
    waypoint_gap_m: float,
    target_speed_mps: float,
) -> tuple[carla.Vehicle, CarlaWaypointCourse]:
    carla_map = world.get_map()
    spawn_points = _get_vehicle_spawn_transforms(carla_map)

    spawn_transform = spawn_points[spawn_index % len(spawn_points)]
    blueprint = world.get_blueprint_library().filter("vehicle.tesla.model3")[0]
    blueprint.set_attribute("role_name", "custom_controller")

    vehicle = world.try_spawn_actor(blueprint, spawn_transform)
    if vehicle is None:
        raise RuntimeError("Could not spawn vehicle. Try --spawn-index with another number.")

    start_wp = carla_map.get_waypoint(
        spawn_transform.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    waypoints = [start_wp]
    distance = 0.0
    current_wp = start_wp
    while distance < route_distance_m:
        next_wps = current_wp.next(waypoint_gap_m)
        if not next_wps:
            break
        current_wp = next_wps[0]
        waypoints.append(current_wp)
        distance += waypoint_gap_m

    course = CarlaWaypointCourse(waypoints, target_speed_mps=target_speed_mps)
    return vehicle, course


def _get_vehicle_spawn_transforms(carla_map: carla.Map) -> list[carla.Transform]:
    spawn_points = carla_map.get_spawn_points()
    if spawn_points:
        return spawn_points

    generated_wps = carla_map.generate_waypoints(10.0)
    if not generated_wps:
        raise RuntimeError("CARLA map has no spawn points or generated waypoints.")
    return [wp.transform for wp in generated_wps]


def _spawn_traffic_vehicles(
    world: carla.World,
    traffic_manager_port: int,
    count: int,
    seed: int,
    skip_transform: carla.Transform,
) -> list[carla.Vehicle]:
    if count <= 0:
        return []

    rng = random.Random(seed)
    blueprints = [
        bp
        for bp in world.get_blueprint_library().filter("vehicle.*")
        if int(bp.get_attribute("number_of_wheels")) == 4
    ]
    if not blueprints:
        print("No vehicle blueprints available for traffic.")
        return []

    spawn_points = _get_vehicle_spawn_transforms(world.get_map())
    rng.shuffle(spawn_points)

    traffic_vehicles = []
    for spawn_transform in spawn_points:
        if len(traffic_vehicles) >= count:
            break
        if _same_spawn_area(spawn_transform, skip_transform, min_distance_m=12.0):
            continue

        blueprint = rng.choice(blueprints)
        if blueprint.has_attribute("color"):
            color = rng.choice(blueprint.get_attribute("color").recommended_values)
            blueprint.set_attribute("color", color)
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", "autopilot")

        vehicle = world.try_spawn_actor(blueprint, spawn_transform)
        if vehicle is None:
            continue
        vehicle.set_autopilot(True, traffic_manager_port)
        traffic_vehicles.append(vehicle)

    print(f"Spawned {len(traffic_vehicles)} moving traffic vehicles.")
    return traffic_vehicles


def _same_spawn_area(
    first: carla.Transform,
    second: carla.Transform,
    min_distance_m: float,
) -> bool:
    dx = first.location.x - second.location.x
    dy = first.location.y - second.location.y
    return math.hypot(dx, dy) < min_distance_m


def _destroy_actors(client: carla.Client, actors: list[carla.Actor | None]) -> None:
    commands = [
        carla.command.DestroyActor(actor.id)
        for actor in actors
        if actor is not None
    ]
    if commands:
        client.apply_batch(commands)


def _control_loop(
    world: carla.World,
    vehicle: carla.Vehicle,
    controller: CustomRoadController | MPCControllerAdapter,
    args: argparse.Namespace,
) -> None:
    start = time.monotonic()
    end = start + args.duration
    max_steer_rad = 0.523

    while time.monotonic() < end:
        if args.sync:
            world.tick()
        else:
            world.wait_for_tick(timeout=2.0)

        state = CarlaState.from_vehicle(vehicle)
        if args.follow_spectator:
            _update_spectator(world, vehicle, args)
        if args.debug_draw_vehicle:
            world.debug.draw_point(
                vehicle.get_location() + carla.Location(z=1.0),
                size=0.12,
                color=carla.Color(255, 0, 0),
                life_time=0.2,
            )

        steer_rad, accel_mps2 = controller.update(state, time.monotonic() - start)
        control = _accel_to_vehicle_control(
            steer_rad=steer_rad,
            accel_mps2=accel_mps2,
            max_steer_rad=max_steer_rad,
            max_accel_mps2=2.0,
            max_decel_mps2=4.0,
        )
        vehicle.apply_control(control)

        print(
            "\r"
            f"t={time.monotonic() - start:5.1f}s "
            f"x={state.x_m:7.1f} y={state.y_m:7.1f} "
            f"v={state.speed_mps:4.1f}m/s "
            f"steer={control.steer:+.2f} throttle={control.throttle:.2f} brake={control.brake:.2f}",
            end="",
            flush=True,
        )

    print()


def _accel_to_vehicle_control(
    steer_rad: float,
    accel_mps2: float,
    max_steer_rad: float,
    max_accel_mps2: float,
    max_decel_mps2: float,
) -> carla.VehicleControl:
    steer = float(np.clip(steer_rad / max_steer_rad, -1.0, 1.0))
    if accel_mps2 >= 0.0:
        throttle = float(np.clip(accel_mps2 / max_accel_mps2, 0.0, 1.0))
        brake = 0.0
    else:
        throttle = 0.0
        brake = float(np.clip(-accel_mps2 / max_decel_mps2, 0.0, 1.0))
    return carla.VehicleControl(throttle=throttle, brake=brake, steer=steer)


def _draw_course_debug(
    world: carla.World,
    course: CarlaWaypointCourse,
    life_time: float,
) -> None:
    blue = carla.Color(0, 80, 255)
    green = carla.Color(0, 255, 0)
    for idx in range(course.length() - 1):
        world.debug.draw_line(
            course.location(idx),
            course.location(idx + 1),
            thickness=0.08,
            color=blue,
            life_time=life_time,
        )
        if idx % 5 == 0:
            world.debug.draw_point(
                course.location(idx, z_offset=0.7),
                size=0.08,
                color=green,
                life_time=life_time,
            )


def _update_spectator(
    world: carla.World,
    vehicle: carla.Vehicle,
    args: argparse.Namespace,
) -> None:
    transform = vehicle.get_transform()
    yaw_rad = math.radians(transform.rotation.yaw)
    back = carla.Location(
        x=-math.cos(yaw_rad) * args.spectator_distance,
        y=-math.sin(yaw_rad) * args.spectator_distance,
        z=args.spectator_height,
    )
    location = transform.location + back
    rotation = carla.Rotation(
        pitch=args.spectator_pitch,
        yaw=transform.rotation.yaw,
        roll=0.0,
    )
    world.get_spectator().set_transform(carla.Transform(location, rotation))


def _wrap_to_pi(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--map", default="Town01_Opt", help="Map to load, e.g. Town01_Opt.")
    parser.add_argument("--controller", choices=["custom", "mpc"], default="custom")
    parser.add_argument("--spawn-index", type=int, default=0)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--target-speed", type=float, default=6.0)
    parser.add_argument("--route-distance", type=float, default=160.0)
    parser.add_argument("--waypoint-gap", type=float, default=2.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--sync", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--follow-spectator", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--spectator-distance", type=float, default=10.0)
    parser.add_argument("--spectator-height", type=float, default=5.0)
    parser.add_argument("--spectator-pitch", type=float, default=-18.0)
    parser.add_argument("--debug-draw-route", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--debug-draw-vehicle", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--traffic-vehicles",
        type=int,
        default=8,
        help="Number of autopilot traffic vehicles to spawn. Use 0 to disable.",
    )
    parser.add_argument("--traffic-manager-port", type=int, default=8000)
    parser.add_argument("--traffic-seed", type=int, default=7)
    parser.add_argument(
        "--traffic-follow-distance",
        type=float,
        default=4.0,
        help="Minimum distance autopilot vehicles keep from the vehicle ahead.",
    )
    parser.add_argument(
        "--traffic-speed-difference",
        type=float,
        default=20.0,
        help="Positive values make traffic drive slower than speed limits.",
    )
    parser.add_argument(
        "--road-only",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Unload heavy visual map layers such as buildings, trees/foliage, props, and parked cars.",
    )
    parser.add_argument(
        "--road-only-mode",
        choices=["opendrive", "layers"],
        default="opendrive",
        help=(
            "opendrive generates a minimal road mesh from the map .xodr; "
            "layers loads the optimized town and unloads buildings/foliage/props."
        ),
    )
    return parser.parse_args()


if __name__ == "__main__":
    main()
