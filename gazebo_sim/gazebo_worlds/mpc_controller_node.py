#!/usr/bin/env python3
"""ROS 2 Node for controlling the Gazebo Prius vehicle using MPC with live OpenCV tuning, saving, and dedicated live plotting."""

import sys
import os
import json
import math
import warnings
from pathlib import Path
import numpy as np
import pandas as pd
import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from rc_msgs.msg import RCMessage
from rc_msgs.srv import CommandBool
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity

# Add workspace root to sys.path to resolve hardware package imports
REPO_ROOT = Path(__file__).resolve().parents[2]
GAZEBO_WORLDS_DIR = Path(__file__).resolve().parent
sys.path.append(str(REPO_ROOT))
from hardware.hil.controllers.mpc_cbf import MPCController, MPCConfig
from hardware.hil.controllers.cbf_qp_ellipse import EllipseCBFQPSafetyFilter, EllipseCBFQPConfig, PointObstacle, Car as CBFCar
from sensor_msgs.msg import LaserScan
import heapq

try:
    import casadi as ca
    import do_mpc
except ImportError:
    pass

def noop(val):
    pass

class TunedMPCController(MPCController):
    """Subclass of MPCController that allows dynamic re-initialization of CasADi weights."""

    def _setup_optimizer(self):
        """Set up the do_mpc model and controller using dynamic cost weights."""
        cfg = self.config
        L = cfg.wheelbase
        dt = self.delta_t

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            model = do_mpc.model.Model("discrete")

        model.set_variable("_x", "px")
        model.set_variable("_x", "py")
        model.set_variable("_x", "yaw")
        model.set_variable("_x", "vel")
        model.set_variable("_u", "steer")
        model.set_variable("_u", "accel")
        model.set_variable("_tvp", "ref_x")
        model.set_variable("_tvp", "ref_y")
        model.set_variable("_tvp", "ref_yaw")
        model.set_variable("_tvp", "ref_vel")

        # Use wrapped atan2 sin/cos to compute phase-independent yaw error
        yaw_err = ca.atan2(
            ca.sin(model.x["yaw"] - model.tvp["ref_yaw"]),
            ca.cos(model.x["yaw"] - model.tvp["ref_yaw"]),
        )
        model.set_expression("yaw_err", yaw_err)

        # Kinematic bicycle model transitions
        model.set_rhs("px", model.x["px"] + model.x["vel"] * ca.cos(model.x["yaw"]) * dt)
        model.set_rhs("py", model.x["py"] + model.x["vel"] * ca.sin(model.x["yaw"]) * dt)
        model.set_rhs("yaw", model.x["yaw"] + model.x["vel"] / L * ca.tan(model.u["steer"]) * dt)
        model.set_rhs("vel", model.x["vel"] + model.u["accel"] * dt)

        model.setup()
        self._model = model

        # Setup MPC
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            mpc = do_mpc.controller.MPC(model)

        ipopt_opts = {
            "ipopt.max_iter": cfg.ipopt_max_iter,
            "ipopt.print_level": cfg.ipopt_print_level,
            "ipopt.sb": "yes",
            "print_time": 0,
        }

        mpc.set_param(
            n_horizon=self.T,
            t_step=dt,
            n_robust=0,
            store_full_solution=True,
            nlpsol_opts=ipopt_opts,
        )

        tvp_template = mpc.get_tvp_template()
        self._current_ref = np.zeros((4, self.T + 1))

        def tvp_fun(t_now):
            for k in range(self.T + 1):
                tvp_template["_tvp", k, "ref_x"] = float(self._current_ref[0, k])
                tvp_template["_tvp", k, "ref_y"] = float(self._current_ref[1, k])
                tvp_template["_tvp", k, "ref_yaw"] = float(self._current_ref[2, k])
                tvp_template["_tvp", k, "ref_vel"] = float(self._current_ref[3, k])
            return tvp_template

        mpc.set_tvp_fun(tvp_fun)

        # Dynamic weights loaded from config
        w_xy = getattr(cfg, "w_xy", 2.0)
        w_yaw = getattr(cfg, "w_yaw", 500.0)
        w_vel = getattr(cfg, "w_vel", 1.0)
        w_steer = getattr(cfg, "w_steer", 10.0)
        w_accel = getattr(cfg, "w_accel", 0.5)

        # Stage weights: [px, py, yaw, vel]
        sw = np.array([w_xy, w_xy, w_yaw, w_vel])
        tw = sw * 2.0
        # Control weights: [steer, accel]
        cw = np.array([w_steer, w_accel])
        # Smoothness weights
        smw = np.array([10.0, 1.0])

        x = model.x
        tvp = model.tvp

        lterm = (
            sw[0] * (x["px"] - tvp["ref_x"]) ** 2
            + sw[1] * (x["py"] - tvp["ref_y"]) ** 2
            + sw[2] * model.aux["yaw_err"] ** 2
            + sw[3] * (x["vel"] - tvp["ref_vel"]) ** 2
        )
        mterm = (
            tw[0] * (x["px"] - tvp["ref_x"]) ** 2
            + tw[1] * (x["py"] - tvp["ref_y"]) ** 2
            + tw[2] * model.aux["yaw_err"] ** 2
            + tw[3] * (x["vel"] - tvp["ref_vel"]) ** 2
        )

        mpc.set_objective(lterm=lterm, mterm=mterm)
        mpc.set_rterm(
            steer=float(cw[0] + smw[0]),
            accel=float(cw[1] + smw[1]),
        )

        mpc.bounds["lower", "_u", "steer"] = cfg.min_steer
        mpc.bounds["upper", "_u", "steer"] = cfg.max_steer
        mpc.bounds["lower", "_u", "accel"] = cfg.min_accel
        mpc.bounds["upper", "_u", "accel"] = cfg.max_accel
        mpc.bounds["lower", "_x", "vel"] = cfg.v_min
        mpc.bounds["upper", "_x", "vel"] = cfg.v_max

        mpc.setup()
        self._mpc = mpc


class GazeboMpcControllerNode(Node):
    """ROS 2 Node wrapping the Model Predictive Controller (MPC) with live OpenCV tuning and saving."""

    def __init__(self):
        super().__init__('gazebo_mpc_controller')

        # Node parameters
        self.declare_parameter('trajectory_file', str(GAZEBO_WORLDS_DIR / 'trajectory.csv'))
        self.declare_parameter('tuning_file', str(GAZEBO_WORLDS_DIR / 'mpc_tuning.json'))
        self.declare_parameter('telemetry_file', str(GAZEBO_WORLDS_DIR / 'mpc_telemetry.csv'))
        self.declare_parameter('target_speed', 5.0)  # m/s
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('odom_topic', '/model/prius/odometry')
        self.declare_parameter('cmd_topic', '/model/prius/cmd_vel')
        self.declare_parameter('ackermann_topic', '/mpc/ackermann_command')
        self.declare_parameter('enable_tuning', True)

        self.trajectory_file = self.get_parameter('trajectory_file').value
        self.tuning_file = self.get_parameter('tuning_file').value
        self.telemetry_file = self.get_parameter('telemetry_file').value
        self.target_speed = self.get_parameter('target_speed').value
        self.control_rate = self.get_parameter('control_rate').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.ackermann_topic = self.get_parameter('ackermann_topic').value
        self.enable_tuning = self.get_parameter('enable_tuning').value

        # Setup ROS 2 Sim Time (declared by base Node class, set to True)
        try:
            self.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        except Exception:
            pass

        # Load waypoints
        self.get_logger().info(f"Loading trajectory waypoints from: {self.trajectory_file}")
        try:
            df = pd.read_csv(self.trajectory_file)
            self.track_points = df[['x', 'y']].to_numpy()
            self.get_logger().info(f"Loaded {len(self.track_points)} waypoints successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load trajectory file: {e}")
            sys.exit(1)

        # Coordinate frame transformation offsets (Odom spawn -> World Map frame)
        self.x_offset = float(self.track_points[0][0])
        self.y_offset = float(self.track_points[0][1])
        first_yaw_deg = float(df.iloc[0]['yaw_deg'])
        self.yaw_offset = math.radians(first_yaw_deg)
        self.get_logger().info(f"World Frame Transform Offset loaded: x_off={self.x_offset:.3f}, y_off={self.y_offset:.3f}, yaw_off={first_yaw_deg:.3f}°")

        # Create/Clear telemetry file
        try:
            with open(self.telemetry_file, 'w') as f:
                f.write("timestamp,x,y,yaw,speed,target_speed,lateral_error,steer_cmd,accel_cmd\n")
            self.get_logger().info(f"Created telemetry file at: {self.telemetry_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to create telemetry file: {e}")

        # Initial MPC configuration defaults
        self.dt = 1.0 / self.control_rate
        self.mpc_config = MPCConfig(
            wheelbase=2.86,
            delta_t=self.dt,
            horizon_T=12,
            max_steer=0.6,
            min_steer=-0.6,
            max_accel=2.0,
            min_accel=-5.0,
            v_min=0.0,
            v_max=12.0
        )
        self.mpc_config.w_xy = 2.0
        self.mpc_config.w_yaw = 500.0
        self.mpc_config.w_vel = 1.0
        self.mpc_config.w_steer = 10.0
        self.mpc_config.w_accel = 0.5

        # Scale factors to map real-world meters to layout units (where 1 layout unit = 1 Gazebo meter)
        self.scale_x = 0.029787
        self.scale_y = 0.033621
        self.x_offset_layout = None
        self.y_offset_layout = None
        self.yaw_offset_layout = None
        self.mpc_config.w_accel = 0.5

        # Try to load existing tuning values from file
        self.load_tuning_from_file()

        # Setup controller
        self.controller = TunedMPCController(self.mpc_config)
        self.controller.update_track(self.track_points)

        # CBF Safety Filter Setup
        self.enable_cbf = True
        self.cbf_config = EllipseCBFQPConfig(
            a_ell=2.5,
            b_ell=1.5,
            wheelbase=2.86,
            gamma1=10.0,
            gamma2=1.0,
            min_accel=self.mpc_config.min_accel,
            max_accel=self.mpc_config.max_accel,
            min_delta=self.mpc_config.min_steer,
            max_delta=self.mpc_config.max_steer,
        )
        self.cbf_filter = EllipseCBFQPSafetyFilter(self.cbf_config)
        self.latest_scan = None

        # ArUco Pose Feedback Setup
        self.use_aruco = os.environ.get('USE_ARUCO', '').lower() in ('true', '1', 'yes', 'on')
        self.aruco_pose_file = '/tmp/aruco_pose.json'
        if self.use_aruco:
            self.get_logger().info("ArUco pose feedback enabled. Reading from /tmp/aruco_pose.json")

        # State variables
        self.current_odom = None
        self.vel_cmd = 0.0
        self.last_accel = 0.0
        self.last_steer = 0.0
        
        # Save feedback status and error history
        self.save_status_msg = None
        self.save_status_time = None
        self.error_history = []
        self.latest_lateral_error = 0.0

        # ROS 2 Subscribers and Publishers
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/lidar2D/scan',
            self.lidar_callback,
            10
        )
        self.publisher = self.create_publisher(
            Twist,
            self.cmd_topic,
            10
        )

        self.rc_pub = self.create_publisher(
            RCMessage,
            "/drone/rc_command",
            10
        )

        # Call arming service for HIL real car
        self.arming_client = self.create_client(
            CommandBool,
            "/drone/cmd/arming"
        )
        self.get_logger().info("Checking for arming service /drone/cmd/arming...")
        if self.arming_client.wait_for_service(timeout_sec=1.0):
            req = CommandBool.Request()
            req.value = True
            self.get_logger().info("Arming RC Car...")
            self.arming_client.call_async(req)
        else:
            self.get_logger().info("Arming service /drone/cmd/arming not available. Skipping arming.")

        # Service client to teleport Prius model in Gazebo
        self.set_pose_client = self.create_client(
            SetEntityPose,
            "/world/custom_road_world/set_pose"
        )

        # Tuning GUI Setup
        if self.enable_tuning:
            self.window_name = "MPC Tuning Panel"
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 550, 460)
            
            self.plot_window_name = "Live Lateral Error Plot"
            cv2.namedWindow(self.plot_window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.plot_window_name, 600, 300)
            
            # Setup trackbars based on loaded values
            cv2.createTrackbar('Target Speed', self.window_name, int(self.target_speed * 10), 150, noop)
            cv2.createTrackbar('Horizon T', self.window_name, self.mpc_config.horizon_T, 25, noop)
            cv2.createTrackbar('XY Tracking W x10', self.window_name, int(self.mpc_config.w_xy * 10), 100, noop)
            cv2.createTrackbar('Yaw W /10', self.window_name, int(self.mpc_config.w_yaw / 10), 200, noop)
            cv2.createTrackbar('Steer Effort W x2', self.window_name, int(self.mpc_config.w_steer * 2), 100, noop)
            cv2.createTrackbar('Speed W x10', self.window_name, int(self.mpc_config.w_vel * 10), 100, noop)
            cv2.createTrackbar('Enable CBF', self.window_name, 1 if self.enable_cbf else 0, 1, noop)
            cv2.createTrackbar('CBF a_ell x10', self.window_name, int(self.cbf_config.a_ell * 10), 100, noop)
            cv2.createTrackbar('CBF b_ell x10', self.window_name, int(self.cbf_config.b_ell * 10), 100, noop)
            cv2.createTrackbar('CBF Gamma1', self.window_name, int(self.cbf_config.gamma1), 50, noop)
            
            self.get_logger().info("OpenCV Tuning Panel and Lateral Error Plot initialized.")

        # Periodic timer for the control loop
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(f"MPC Controller Node initialized at {self.control_rate}Hz.")

    def odom_callback(self, msg):
        self.current_odom = msg

    def lidar_callback(self, msg):
        self.latest_scan = msg

    def get_obstacles_from_scan(self, px, py, yaw):
        """Convert latest LaserScan ranges to PointObstacle objects in the world map frame."""
        if self.latest_scan is None:
            return []

        scan = self.latest_scan
        obstacles = []
        
        # Subsample scan points to avoid overloading the QP solver
        step = 6  # Process every 6th beam
        
        for i in range(0, len(scan.ranges), step):
            r = scan.ranges[i]
            beam_angle = scan.angle_min + i * scan.angle_increment
            
            # Focus only on nearby obstacles (e.g. within 12 meters) to optimize QP speed
            if np.isfinite(r) and scan.range_min < r < min(12.0, scan.range_max):
                x_local = 2.3 + r * math.cos(beam_angle)
                y_local = r * math.sin(beam_angle)
                
                # Transform to World Map frame
                x_world = px + x_local * math.cos(yaw) - y_local * math.sin(yaw)
                y_world = py + x_local * math.sin(yaw) + y_local * math.cos(yaw)
                
                obstacles.append(PointObstacle(x=x_world, y=y_world))
                
        return obstacles

    def load_tuning_from_file(self):
        """Loads tuning values from self.tuning_file if it exists."""
        tuning_path = Path(self.tuning_file)
        if not tuning_path.exists():
            self.get_logger().warning(f"No existing tuning file found at {self.tuning_file}. Using defaults.")
            return

        try:
            payload = json.loads(tuning_path.read_text(encoding='utf-8'))
            self.target_speed = payload.get('target_speed', self.target_speed)
            self.mpc_config.horizon_T = payload.get('horizon_T', self.mpc_config.horizon_T)
            self.mpc_config.w_xy = payload.get('w_xy', self.mpc_config.w_xy)
            self.mpc_config.w_yaw = payload.get('w_yaw', self.mpc_config.w_yaw)
            self.mpc_config.w_steer = payload.get('w_steer', self.mpc_config.w_steer)
            self.mpc_config.w_vel = payload.get('w_vel', self.mpc_config.w_vel)
            self.enable_cbf = payload.get('enable_cbf', self.enable_cbf)
            self.scale_x = payload.get('scale_x', self.scale_x)
            self.scale_y = payload.get('scale_y', self.scale_y)
            if hasattr(self, 'cbf_config'):
                self.cbf_config.a_ell = payload.get('cbf_a_ell', self.cbf_config.a_ell)
                self.cbf_config.b_ell = payload.get('cbf_b_ell', self.cbf_config.b_ell)
                self.cbf_config.gamma1 = payload.get('cbf_gamma1', self.cbf_config.gamma1)
            self.get_logger().info(f"Loaded tuning parameters successfully from {self.tuning_file} (scale_x={self.scale_x:.6f}, scale_y={self.scale_y:.6f})")
        except Exception as e:
            self.get_logger().error(f"Error reading tuning file: {e}")

    def save_tuning_to_file(self):
        """Saves current tuning trackbar parameters to self.tuning_file."""
        payload = {
            'target_speed': float(self.target_speed),
            'horizon_T': int(self.mpc_config.horizon_T),
            'w_xy': float(self.mpc_config.w_xy),
            'w_yaw': float(self.mpc_config.w_yaw),
            'w_steer': float(self.mpc_config.w_steer),
            'w_vel': float(self.mpc_config.w_vel),
            'enable_cbf': bool(self.enable_cbf),
            'cbf_a_ell': float(self.cbf_config.a_ell),
            'cbf_b_ell': float(self.cbf_config.b_ell),
            'cbf_gamma1': float(self.cbf_config.gamma1),
            'scale_x': float(self.scale_x),
            'scale_y': float(self.scale_y)
        }
        
        try:
            tuning_path = Path(self.tuning_file)
            tuning_path.parent.mkdir(parents=True, exist_ok=True)
            tuning_path.write_text(json.dumps(payload, indent=2) + '\n', encoding='utf-8')
            self.get_logger().info(f"Saved tuning values to: {self.tuning_file}")
            self.save_status_msg = "SAVED TO FILE SUCCESSFULLY!"
            self.save_status_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().error(f"Failed to save tuning: {e}")
            self.save_status_msg = "SAVE TO FILE FAILED!"
            self.save_status_time = self.get_clock().now()

    def read_tuning_panel(self):
        """Read values from trackbars and re-initialize optimizer if any weight changes."""
        if not self.enable_tuning:
            return

        # Target Speed can be changed without re-setup
        speed_val = cv2.getTrackbarPos('Target Speed', self.window_name) / 10.0
        self.target_speed = max(0.5, speed_val)

        # Check if structural parameters changed
        t_val = max(4, cv2.getTrackbarPos('Horizon T', self.window_name))
        w_xy_val = cv2.getTrackbarPos('XY Tracking W x10', self.window_name) / 10.0
        w_yaw_val = cv2.getTrackbarPos('Yaw W /10', self.window_name) * 10.0
        w_steer_val = cv2.getTrackbarPos('Steer Effort W x2', self.window_name) / 2.0
        w_vel_val = cv2.getTrackbarPos('Speed W x10', self.window_name) / 10.0

        changed = (
            t_val != self.mpc_config.horizon_T or
            abs(w_xy_val - self.mpc_config.w_xy) > 1e-3 or
            abs(w_yaw_val - self.mpc_config.w_yaw) > 1e-3 or
            abs(w_steer_val - self.mpc_config.w_steer) > 1e-3 or
            abs(w_vel_val - self.mpc_config.w_vel) > 1e-3
        )

        # Read CBF parameters
        self.enable_cbf = cv2.getTrackbarPos('Enable CBF', self.window_name) == 1
        self.cbf_config.a_ell = max(0.1, cv2.getTrackbarPos('CBF a_ell x10', self.window_name) / 10.0)
        self.cbf_config.b_ell = max(0.1, cv2.getTrackbarPos('CBF b_ell x10', self.window_name) / 10.0)
        self.cbf_config.gamma1 = float(max(1, cv2.getTrackbarPos('CBF Gamma1', self.window_name)))

        if changed:
            self.get_logger().info("Re-optimizing solver with new tuning panel parameters...")
            self.mpc_config.horizon_T = t_val
            self.mpc_config.w_xy = max(0.1, w_xy_val)
            self.mpc_config.w_yaw = max(1.0, w_yaw_val)
            self.mpc_config.w_steer = max(0.1, w_steer_val)
            self.mpc_config.w_vel = max(0.1, w_vel_val)
            
            # Preserve current waypoint progress index to prevent jumps/clamping issues
            prev_idx = self.controller._prev_waypoint_idx
            
            # Rebuild optimizer
            self.controller = TunedMPCController(self.mpc_config)
            self.controller.update_track(self.track_points)
            self.controller._prev_waypoint_idx = prev_idx
            
            self.get_logger().info("Solver re-built successfully.")

    def draw_status_display(self, vel, solve_time):
        """Draw a status panel with telemetry in OpenCV."""
        panel = np.zeros((350, 500, 3), dtype=np.uint8)
        
        # Header
        cv2.putText(panel, "MPC TUNING & TELEMETRY", (25, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.line(panel, (20, 48), (480, 48), (100, 100, 100), 1)

        # Status text rows
        solve_color = (0, 255, 0) if self.controller.solver_success else (0, 0, 255)
        cv2.putText(panel, f"Solver success: {self.controller.solver_success}", (25, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, solve_color, 1, cv2.LINE_AA)
        cv2.putText(panel, f"Solve time: {solve_time:.2f} ms", (25, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
        
        cv2.putText(panel, f"Current speed: {vel:.2f} m/s", (25, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(panel, f"Target speed: {self.target_speed:.2f} m/s", (25, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
        
        cv2.putText(panel, f"Steer Cmd: {math.degrees(self.last_steer):.1f} deg", (25, 225), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(panel, f"Accel Cmd: {self.last_accel:.3f} m/s2", (25, 255), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
        
        cbf_status_str = "ACTIVE" if (self.enable_cbf and self.latest_scan is not None) else ("DISABLED" if not self.enable_cbf else "WAITING FOR SCAN")
        cbf_color = (0, 255, 0) if self.enable_cbf else (128, 128, 128)
        cv2.putText(panel, f"CBF Safety Filter: {cbf_status_str}", (25, 285), cv2.FONT_HERSHEY_SIMPLEX, 0.55, cbf_color, 1, cv2.LINE_AA)

        # Save instructions and save status
        if self.save_status_msg:
            elapsed = (self.get_clock().now() - self.save_status_time).nanoseconds / 1e9
            if elapsed < 2.5:
                color = (0, 255, 0) if "SUCCESS" in self.save_status_msg else (0, 0, 255)
                cv2.putText(panel, self.save_status_msg, (25, 325), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
            else:
                self.save_status_msg = None
                cv2.putText(panel, "Press 'S' on this window to save values", (25, 325), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)
        else:
            cv2.putText(panel, "Press 'S' on this window to save values", (25, 325), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)

        cv2.imshow(self.window_name, panel)

        # Call key check (shared with plotting window render)
        cv2.waitKey(1)

    def draw_live_plot(self):
        """Draws a high-resolution live scrolling lateral error plot in its own dedicated window."""
        canvas_h, canvas_w = 300, 600
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

        # Draw dark grid lines and labels
        center_y = 150
        scale_y = 100.0  # 1.0m error = 100px vertical displacement

        grid_values = [1.0, 0.5, 0.0, -0.5, -1.0]
        for val in grid_values:
            y_pos = int(center_y - val * scale_y)
            color = (0, 60, 0) if val == 0.0 else (40, 40, 40)
            thickness = 2 if val == 0.0 else 1
            cv2.line(canvas, (65, y_pos), (canvas_w - 20, y_pos), color, thickness)
            
            # Label Y axis
            cv2.putText(canvas, f"{val:+.1f}m", (10, y_pos + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1, cv2.LINE_AA)

        # Draw vertical gridlines (time axis ticks)
        for x_pos in range(150, canvas_w - 20, 100):
            cv2.line(canvas, (x_pos, 10), (x_pos, canvas_h - 10), (30, 30, 30), 1)

        # Plot error history curve
        gx_start = 65
        g_width = canvas_w - gx_start - 20
        
        if len(self.error_history) > 1:
            points = []
            history_slice = self.error_history[-g_width:]
            
            for idx, err in enumerate(history_slice):
                # Scale data points horizontally to fit graph width
                x_pos = gx_start + int(idx * (g_width / len(history_slice)))
                y_pos = int(center_y - err * scale_y)
                # Keep points inside plot boundary
                y_pos = max(10, min(canvas_h - 10, y_pos))
                points.append((x_pos, y_pos))

            for k in range(len(points) - 1):
                # Yellow trace for error curve
                cv2.line(canvas, points[k], points[k+1], (0, 220, 255), 1, cv2.LINE_AA)

        # Stats Overlay
        if len(self.error_history) > 0:
            rmse = (np.array(self.error_history) ** 2).mean() ** 0.5
            max_err = np.abs(self.error_history).max()
            cv2.putText(canvas, f"Live Lateral Error: {self.latest_lateral_error:+.3f} m", (gx_start + 15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(canvas, f"RMSE: {rmse:.3f} m | Max Deviation: {max_err:.3f} m", (gx_start + 15, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1, cv2.LINE_AA)

        cv2.imshow(self.plot_window_name, canvas)
        
        # Check keyboard inputs
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') or key == ord('S'):
            self.save_tuning_to_file()

    def timer_callback(self):
        # Update weights and target speed from window
        if self.enable_tuning:
            self.read_tuning_panel()

        if self.current_odom is None:
            self.get_logger().warning("Waiting for odometry messages...", throttle_duration_sec=3.0)
            return

        # 1. Extract current state
        use_gazebo_odom = True
        if self.use_aruco:
            try:
                import json
                if os.path.exists(self.aruco_pose_file):
                    with open(self.aruco_pose_file, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                    target_in_origin = data.get('target_in_origin')
                    if target_in_origin is not None:
                        # Raw ArUco coordinates in meters
                        x_real = float(target_in_origin['translation_m'][0])
                        y_real = float(target_in_origin['translation_m'][1])
                        yaw_deg = float(target_in_origin['rpy_deg'][2])
                        
                        # Scale to layout units (1 layout unit = 1 Gazebo meter)
                        x_layout = x_real / self.scale_x
                        y_layout = -y_real / self.scale_y  # Mirror Y to match layout orientation
                        yaw_layout_deg = -yaw_deg          # Mirror yaw to match mirrored Y
                        
                        # Initialize offsets dynamically on the first frame to align starting waypoint
                        if self.x_offset_layout is None or self.y_offset_layout is None:
                            self.x_offset_layout = self.x_offset - x_layout
                            self.y_offset_layout = self.y_offset - y_layout
                            self.yaw_offset_layout = self.yaw_offset - math.radians(yaw_layout_deg)
                            self.get_logger().info(
                                f"Dynamic ArUco Alignment Offset Initialized: "
                                f"x_off={self.x_offset_layout:.3f}, y_off={self.y_offset_layout:.3f}, "
                                f"yaw_off={math.degrees(self.yaw_offset_layout):.1f}°"
                            )
                        
                        # Transform to Gazebo world meters
                        px = x_layout + self.x_offset_layout
                        py = y_layout + self.y_offset_layout
                        yaw = math.radians(yaw_layout_deg) + self.yaw_offset_layout
                        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                        use_gazebo_odom = False
                        
                        # Teleport the Prius model in Gazebo to match the real car
                        self.teleport_prius_gazebo(px, py, yaw)
            except Exception as e:
                self.get_logger().warning(f"Error reading ArUco pose: {e}. Falling back to Gazebo Odom.", throttle_duration_sec=2.0)

        if use_gazebo_odom:
            # Transform base Gazebo odometry to World Map frame
            px = self.current_odom.pose.pose.position.x + self.x_offset
            py = self.current_odom.pose.pose.position.y + self.y_offset

            # Quaternion to Euler Yaw
            q = self.current_odom.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw_odom = math.atan2(siny_cosp, cosy_cosp)
            
            # Shift yaw relative to initial world pose and wrap to [-pi, pi]
            yaw = yaw_odom + self.yaw_offset
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        # Current speed calculation (preserving direction)
        vx = self.current_odom.twist.twist.linear.x
        vy = self.current_odom.twist.twist.linear.y
        vel = math.hypot(vx, vy)
        if vx < 0:
            vel = -vel

        # 2. Run MPC optimization
        accel, steer = self.controller.step(px, py, yaw, vel, self.target_speed)

        # 2.5 Filter control commands through CBF-QP if enabled
        if self.enable_cbf:
            obstacles = self.get_obstacles_from_scan(px, py, yaw)
            if obstacles:
                # Limit the number of closest obstacles sent to the QP solver to max_obstacles (e.g. 4)
                closest_obstacles = heapq.nsmallest(
                    4,
                    obstacles,
                    key=lambda obs: (obs.x - px)**2 + (obs.y - py)**2
                )
                
                car_state = CBFCar(
                    x=float(px),
                    y=float(py),
                    psi=float(yaw),
                    v=max(0.0, float(vel)),
                    v_cmd=max(0.0, float(self.target_speed)),
                )
                
                try:
                    # Update min/max constraints to match current dynamic MPC parameters
                    self.cbf_config.min_accel = float(self.mpc_config.min_accel)
                    self.cbf_config.max_accel = float(self.mpc_config.max_accel)
                    self.cbf_config.min_delta = float(self.mpc_config.min_steer)
                    self.cbf_config.max_delta = float(self.mpc_config.max_steer)
                    
                    safe_accel, safe_steer = self.cbf_filter.solve(
                        car_state,
                        closest_obstacles,
                        accel,
                        steer
                    )
                    accel, steer = safe_accel, safe_steer
                except Exception as exc:
                    self.get_logger().error(f"CBF solver error: {exc}")

        self.last_accel = accel
        self.last_steer = steer

        # 3. Calculate signed lateral error (cross-track error)
        best_idx = self.controller._prev_waypoint_idx
        ref_x = self.controller._ext_x[best_idx]
        ref_y = self.controller._ext_y[best_idx]
        ref_yaw = self.controller._ext_yaw[best_idx]

        dx = px - ref_x
        dy = py - ref_y

        # Normal vector pointing left in World Map frame
        nx = -math.sin(ref_yaw)
        ny = math.cos(ref_yaw)
        
        # Signed lateral error is projection of distance vector onto the normal vector
        self.latest_lateral_error = dx * nx + dy * ny

        # Maintain lateral error history list (max 500 points to fill graph width)
        self.error_history.append(self.latest_lateral_error)
        if len(self.error_history) > 500:
            self.error_history.pop(0)

        # 4. Integrate acceleration to compute target velocity
        self.vel_cmd = self.vel_cmd + accel * self.dt
        self.vel_cmd = max(0.0, min(self.target_speed, self.vel_cmd))

        # 5. Construct and publish command message
        cmd_msg = Twist()
        cmd_msg.linear.x = float(self.vel_cmd)
        cmd_msg.angular.z = float(steer)
        self.publisher.publish(cmd_msg)

        # ackermann_msg = AckermannCommand()                               ##### That is just a dummy import generated by the AI 
        # ackermann_msg.steering_angle_rad = float(steer)
        # ackermann_msg.acceleration_mps2 = float(accel)
        # ackermann_msg.target_speed_mps = float(self.target_speed)
        # self.ackermann_publisher.publish(ackermann_msg)

        # 5b. Publish RCMessage for real-world car / HIL
        REAL_RC_SCALE_THROTTLE = 100
        REAL_RC_SCALE_STEER = 470
        max_steer_rad = float(self.mpc_config.max_steer)
        max_accel_mps2 = float(self.mpc_config.max_accel)

        rc_msg = RCMessage()
        
        # Steering -> ROLL
        steer_normalized = steer / max_steer_rad
        rc_msg.rc_roll = int(1500 + REAL_RC_SCALE_STEER * steer_normalized)
        rc_msg.rc_roll = max(1000, min(2000, rc_msg.rc_roll))
        
        # Throttle -> PITCH
        throttle_normalized = max(0.0, accel) / max_accel_mps2
        rc_msg.rc_pitch = int(1580 + REAL_RC_SCALE_THROTTLE * throttle_normalized)
        rc_msg.rc_pitch = max(1580, min(1590, rc_msg.rc_pitch))
        
        rc_msg.rc_throttle = 1500
        rc_msg.rc_yaw = 1500
        
        # Unused channels
        rc_msg.aux1 = 2000
        rc_msg.aux2 = 1000
        rc_msg.aux3 = 1000
        rc_msg.aux4 = 1000
        
        self.rc_pub.publish(rc_msg)

        # 6. Log telemetry to CSV file
        try:
            timestamp = self.get_clock().now().nanoseconds / 1e9
            with open(self.telemetry_file, 'a') as f:
                f.write(f"{timestamp:.3f},{px:.3f},{py:.3f},{yaw:.3f},{vel:.3f},{self.target_speed:.3f},{self.latest_lateral_error:.3f},{steer:.3f},{accel:.3f}\n")
        except Exception:
            pass

        # 7. Live display update
        if self.enable_tuning:
            self.draw_status_display(vel, self.controller.solve_time_ms)
            self.draw_live_plot()

        self.get_logger().info(
            f"Pose: ({px:.2f}, {py:.2f}) | Lateral Error: {self.latest_lateral_error:.3f}m | Target: {self.target_speed:.2f} m/s",
            throttle_duration_sec=0.5
        )

    def teleport_prius_gazebo(self, px, py, yaw):
        """Teleport the Gazebo Prius model to the ArUco camera pose."""
        try:
            if not self.set_pose_client.service_is_ready():
                self.get_logger().warning("Gazebo set_pose service not ready...", throttle_duration_sec=5.0)
                return

            req = SetEntityPose.Request()
            req.entity.name = "prius"
            req.entity.type = Entity.MODEL
            
            # Position
            req.pose.position.x = float(px)
            req.pose.position.y = float(py)
            req.pose.position.z = 0.35  # Keep it slightly above the ground plane
            
            # Orientation
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            req.pose.orientation.w = cy
            req.pose.orientation.x = 0.0
            req.pose.orientation.y = 0.0
            req.pose.orientation.z = sy
            
            self.set_pose_client.call_async(req)
        except Exception as e:
            self.get_logger().error(f"Failed to call set_pose service: {e}", throttle_duration_sec=3.0)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboMpcControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("MPC Controller Node stopped by user.")
    except ExternalShutdownException:
        pass
    finally:
        if node.enable_tuning:
            cv2.destroyAllWindows()
        if rclpy.ok():
            stop_msg = Twist()
            node.publisher.publish(stop_msg)
            
            
            rc_stop_msg = RCMessage()
            rc_stop_msg.rc_roll = 1500
            rc_stop_msg.rc_pitch = 1580
            rc_stop_msg.rc_throttle = 1500
            rc_stop_msg.rc_yaw = 1500
            rc_stop_msg.aux1 = 2000
            rc_stop_msg.aux2 = 1000
            rc_stop_msg.aux3 = 1000
            rc_stop_msg.aux4 = 1000
            node.rc_pub.publish(rc_stop_msg)

            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
