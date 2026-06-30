#!/usr/bin/env python3
"""ROS 2 Node for controlling the Gazebo Prius vehicle using MPC with live OpenCV tuning and saving."""

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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Add workspace root to sys.path to resolve hardware package imports
sys.path.append('/home/dhruv/amcaf')
from hardware.hil.controllers.mpc_cbf import MPCController, MPCConfig

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
        self.declare_parameter('trajectory_file', '/home/dhruv/amcaf/gazebo_sim/gazebo_worlds/trajectory.csv')
        self.declare_parameter('tuning_file', '/home/dhruv/amcaf/gazebo_sim/gazebo_worlds/mpc_tuning.json')
        self.declare_parameter('target_speed', 5.0)  # m/s
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('odom_topic', '/model/prius/odometry')
        self.declare_parameter('cmd_topic', '/model/prius/cmd_vel')
        self.declare_parameter('enable_tuning', True)
        # Set pre-declared use_sim_time parameter to True
        try:
            self.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        except Exception:
            pass

        self.trajectory_file = self.get_parameter('trajectory_file').value
        self.tuning_file = self.get_parameter('tuning_file').value
        self.target_speed = self.get_parameter('target_speed').value
        self.control_rate = self.get_parameter('control_rate').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.enable_tuning = self.get_parameter('enable_tuning').value

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

        # Try to load existing tuning values from file
        self.load_tuning_from_file()

        # Setup controller
        self.controller = TunedMPCController(self.mpc_config)
        self.controller.update_track(self.track_points)

        # State variables
        self.current_odom = None
        self.vel_cmd = 0.0
        self.last_accel = 0.0
        self.last_steer = 0.0
        
        # Save feedback status
        self.save_status_msg = None
        self.save_status_time = None

        # ROS 2 Subscribers and Publishers
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        self.publisher = self.create_publisher(
            Twist,
            self.cmd_topic,
            10
        )

        # Tuning GUI Setup
        if self.enable_tuning:
            self.window_name = "MPC Tuning Panel"
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 550, 420)
            
            # Setup trackbars based on loaded values
            cv2.createTrackbar('Target Speed', self.window_name, int(self.target_speed * 10), 150, noop)
            cv2.createTrackbar('Horizon T', self.window_name, self.mpc_config.horizon_T, 25, noop)
            cv2.createTrackbar('XY Tracking W x10', self.window_name, int(self.mpc_config.w_xy * 10), 100, noop)
            cv2.createTrackbar('Yaw W /10', self.window_name, int(self.mpc_config.w_yaw / 10), 200, noop)
            cv2.createTrackbar('Steer Effort W x2', self.window_name, int(self.mpc_config.w_steer * 2), 100, noop)
            cv2.createTrackbar('Speed W x10', self.window_name, int(self.mpc_config.w_vel * 10), 100, noop)
            
            self.get_logger().info("OpenCV Tuning Panel initialized.")

        # Periodic timer for the control loop
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(f"MPC Controller Node initialized at {self.control_rate}Hz.")

    def odom_callback(self, msg):
        self.current_odom = msg

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
            self.get_logger().info(f"Loaded tuning parameters successfully from {self.tuning_file}")
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
            'w_vel': float(self.mpc_config.w_vel)
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
        """Draw a status panel with telemetry in the OpenCV window."""
        panel = np.zeros((300, 500, 3), dtype=np.uint8)
        
        # Header
        cv2.putText(panel, "MPC TUNING & TELEMETRY", (25, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.line(panel, (20, 48), (480, 48), (100, 100, 100), 1)

        # Status text rows
        solve_color = (0, 255, 0) if self.controller.solver_success else (0, 0, 255)
        cv2.putText(panel, f"Solver success: {self.controller.solver_success}", (25, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, solve_color, 1, cv2.LINE_AA)
        cv2.putText(panel, f"Solve time: {solve_time:.2f} ms", (25, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
        
        cv2.putText(panel, f"Current speed: {vel:.2f} m/s", (25, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(panel, f"Target speed: {self.target_speed:.2f} m/s", (25, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
        
        cv2.putText(panel, f"Steer Cmd: {math.degrees(self.last_steer):.1f} deg", (25, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(panel, f"Accel Cmd: {self.last_accel:.3f} m/s2", (25, 265), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
        
        # Save instructions and save status
        if self.save_status_msg:
            elapsed = (self.get_clock().now() - self.save_status_time).nanoseconds / 1e9
            if elapsed < 2.5:
                color = (0, 255, 0) if "SUCCESS" in self.save_status_msg else (0, 0, 255)
                cv2.putText(panel, self.save_status_msg, (25, 285), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
            else:
                self.save_status_msg = None
                cv2.putText(panel, "Press 'S' on this window to save values", (25, 285), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)
        else:
            cv2.putText(panel, "Press 'S' on this window to save values", (25, 285), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)

        cv2.imshow(self.window_name, panel)
        
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

        # 1. Extract current state and transform to World Map frame
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
        self.last_accel = accel
        self.last_steer = steer

        # 3. Integrate acceleration to compute target velocity
        self.vel_cmd = self.vel_cmd + accel * self.dt
        self.vel_cmd = max(0.0, min(self.target_speed, self.vel_cmd))

        # 4. Construct and publish the command message
        cmd_msg = Twist()
        cmd_msg.linear.x = float(self.vel_cmd)
        cmd_msg.angular.z = float(steer)
        self.publisher.publish(cmd_msg)

        # 5. Live display update
        if self.enable_tuning:
            self.draw_status_display(vel, self.controller.solve_time_ms)

        self.get_logger().info(
            f"Pose: ({px:.2f}, {py:.2f}) | Speed: {vel:.2f} m/s | Target: {self.target_speed:.2f} m/s | "
            f"Steer: {math.degrees(steer):.1f}°",
            throttle_duration_sec=0.5
        )

def main(args=None):
    rclpy.init(args=args)
    node = GazeboMpcControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("MPC Controller Node stopped by user.")
    finally:
        if node.enable_tuning:
            cv2.destroyAllWindows()
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
