#!/usr/bin/env python3
"""ROS 2 Node for controlling the Gazebo Prius vehicle using PID with live OpenCV tuning.

Drop-in replacement for the MPC controller node. Only the controller changes
(MPC -> PID); the ROS2 node architecture, topics, trajectory loading, and
OpenCV tuning panel follow the same pattern as the original MPC node.
"""

import sys
import math
import numpy as np
import pandas as pd
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys

sys.path.append("/home/monukoru/Documents/AMCAF_-code/hardware/hil/controllers")

from pid import PIDController


def noop(val):
    pass


def normalize_angle(angle):
    """Wrap an angle to the range [-pi, +pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class GazeboPidControllerNode(Node):
    """ROS 2 Node wrapping two PIDController instances with a live OpenCV tuning panel."""

    def __init__(self):
        super().__init__('gazebo_pid_controller')

        # Node parameters
        self.declare_parameter('trajectory_file', '/home/monukoru/Digital_master_AMCAF-master/gazebo_worlds/trajectory.csv')
        self.declare_parameter('target_speed', 5.0)  # m/s
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('odom_topic', '/model/prius/odometry')
        self.declare_parameter('cmd_topic', '/model/prius/cmd_vel')
        self.declare_parameter('lookahead', 8)
        self.declare_parameter('enable_tuning', True)

        self.trajectory_file = self.get_parameter('trajectory_file').value
        self.target_speed = self.get_parameter('target_speed').value
        self.control_rate = self.get_parameter('control_rate').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.lookahead = self.get_parameter('lookahead').value
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

        self.dt = 1.0 / self.control_rate

        # --- PID setup ---
        # Steering PID: input heading_error (rad), output steering command (rad).
        self.steer_kp = 2.0
        self.steer_ki = 0.0
        self.steer_kd = 0.3
        self.steer_i_limit = 1.0
        self.steer_limits = (-0.6, 0.6)

        # Speed PID: input speed_error (m/s), output acceleration (m/s^2).
        self.speed_kp = 1.5
        self.speed_ki = 0.05
        self.speed_kd = 0.05
        self.speed_i_limit = 5.0
        self.accel_limits = (-5.0, 2.0)

        self.speed_pid = PIDController(
            self.speed_kp, self.speed_ki, self.speed_kd, self.speed_i_limit
        )
        self.steer_pid = PIDController(
            self.steer_kp, self.steer_ki, self.steer_kd, self.steer_i_limit
        )

        # State variables
        self.current_odom = None
        self.vel_cmd = 0.0
        self.last_accel = 0.0
        self.last_steer = 0.0

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
            self.window_name = "PID Tuning Panel"
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 550, 450)

            # Setup trackbars. Gains are scaled by an integer factor so they can
            # be represented on an OpenCV integer trackbar, then divided back
            # down when read in read_tuning_panel().
            cv2.createTrackbar('Target Speed x10', self.window_name, int(self.target_speed * 10), 150, noop)
            cv2.createTrackbar('Lookahead', self.window_name, int(self.lookahead), 50, noop)
            cv2.createTrackbar('Steer KP x100', self.window_name, int(self.steer_kp * 100), 1000, noop)
            cv2.createTrackbar('Steer KI x100', self.window_name, int(self.steer_ki * 100), 200, noop)
            cv2.createTrackbar('Steer KD x100', self.window_name, int(self.steer_kd * 100), 500, noop)
            cv2.createTrackbar('Speed KP x100', self.window_name, int(self.speed_kp * 100), 1000, noop)
            cv2.createTrackbar('Speed KI x100', self.window_name, int(self.speed_ki * 100), 200, noop)
            cv2.createTrackbar('Speed KD x100', self.window_name, int(self.speed_kd * 100), 200, noop)

            self.get_logger().info("OpenCV Tuning Panel initialized.")

        # Periodic timer for the control loop
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(f"PID Controller Node initialized at {self.control_rate}Hz.")

    def odom_callback(self, msg):
        self.current_odom = msg

    def read_tuning_panel(self):
        """Read values from trackbars and push any gain changes straight into the PIDs."""
        if not self.enable_tuning:
            return

        speed_val = cv2.getTrackbarPos('Target Speed x10', self.window_name) / 10.0
        self.target_speed = max(0.5, speed_val)

        self.lookahead = max(1, cv2.getTrackbarPos('Lookahead', self.window_name))

        steer_kp_val = cv2.getTrackbarPos('Steer KP x100', self.window_name) / 100.0
        steer_ki_val = cv2.getTrackbarPos('Steer KI x100', self.window_name) / 100.0
        steer_kd_val = cv2.getTrackbarPos('Steer KD x100', self.window_name) / 100.0

        speed_kp_val = cv2.getTrackbarPos('Speed KP x100', self.window_name) / 100.0
        speed_ki_val = cv2.getTrackbarPos('Speed KI x100', self.window_name) / 100.0
        speed_kd_val = cv2.getTrackbarPos('Speed KD x100', self.window_name) / 100.0

        steer_changed = (
            abs(steer_kp_val - self.steer_kp) > 1e-3
            or abs(steer_ki_val - self.steer_ki) > 1e-3
            or abs(steer_kd_val - self.steer_kd) > 1e-3
        )
        speed_changed = (
            abs(speed_kp_val - self.speed_kp) > 1e-3
            or abs(speed_ki_val - self.speed_ki) > 1e-3
            or abs(speed_kd_val - self.speed_kd) > 1e-3
        )

        if steer_changed:
            self.steer_kp, self.steer_ki, self.steer_kd = steer_kp_val, steer_ki_val, steer_kd_val
            self.steer_pid.update_gains(
                self.steer_kp, self.steer_ki, self.steer_kd, self.steer_i_limit
            )

        if speed_changed:
            self.speed_kp, self.speed_ki, self.speed_kd = speed_kp_val, speed_ki_val, speed_kd_val
            self.speed_pid.update_gains(
                self.speed_kp, self.speed_ki, self.speed_kd, self.speed_i_limit
            )

    def draw_status_display(self, vel, heading_error):
        """Draw a status panel with telemetry in the OpenCV window."""
        panel = np.zeros((300, 500, 3), dtype=np.uint8)

        # Header
        cv2.putText(panel, "PID TUNING & TELEMETRY", (25, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.line(panel, (20, 48), (480, 48), (100, 100, 100), 1)

        cv2.putText(panel, f"Current speed: {vel:.2f} m/s", (25, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(panel, f"Target speed: {self.target_speed:.2f} m/s", (25, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(panel, f"Lookahead: {self.lookahead}", (25, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)

        cv2.putText(panel, f"Heading error: {math.degrees(heading_error):.1f} deg", (25, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(panel, f"Steer Cmd: {math.degrees(self.last_steer):.1f} deg", (25, 225), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(panel, f"Accel Cmd: {self.last_accel:.3f} m/s2", (25, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)

        cv2.imshow(self.window_name, panel)
        cv2.waitKey(1)

    def find_target_waypoint(self, px, py):
        """Return the look-ahead waypoint (x, y) using nearest-point + lookahead index."""
        distances = np.hypot(self.track_points[:, 0] - px, self.track_points[:, 1] - py)
        nearest_index = int(np.argmin(distances))
        target_index = min(nearest_index + self.lookahead, len(self.track_points) - 1)
        return self.track_points[target_index]

    def timer_callback(self):
        # Update gains and target speed/lookahead from window
        if self.enable_tuning:
            self.read_tuning_panel()

        if self.current_odom is None:
            self.get_logger().warning("Waiting for odometry messages...", throttle_duration_sec=3.0)
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # 1. Extract current state (exactly like the MPC node)
        px = self.current_odom.pose.pose.position.x
        py = self.current_odom.pose.pose.position.y

        # Quaternion to Euler Yaw
        q = self.current_odom.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Current speed calculation (preserving direction)
        vx = self.current_odom.twist.twist.linear.x
        vy = self.current_odom.twist.twist.linear.y
        vel = math.hypot(vx, vy)
        if vx < 0:
            vel = -vel

        # 2 & 3. Nearest waypoint + look-ahead target waypoint
        target_x, target_y = self.find_target_waypoint(px, py)

        # 4. Vector towards target waypoint
        dx = target_x - px
        dy = target_y - py

        # 5. Target heading
        target_heading = math.atan2(dy, dx)

        # 6. Heading error, normalized to [-pi, +pi]
        heading_error = normalize_angle(target_heading - yaw)

        # 7. Steering PID
        steer = self.steer_pid.step(
            heading_error,
            now,
            output_limits=self.steer_limits,
        )
        steer = max(self.steer_limits[0], min(self.steer_limits[1], steer))
        self.last_steer = steer

        # 8. Speed PID -> acceleration
        speed_error = self.target_speed - vel
        accel = self.speed_pid.step(
            speed_error,
            now,
            output_limits=self.accel_limits,
        )
        self.last_accel = accel

        # 9. Integrate acceleration to compute target velocity (exactly like the MPC node)
        self.vel_cmd = self.vel_cmd + accel * self.dt
        self.vel_cmd = max(0.0, min(self.target_speed, self.vel_cmd))

        # 10. Construct and publish the command message
        cmd_msg = Twist()
        cmd_msg.linear.x = float(self.vel_cmd)
        cmd_msg.angular.z = float(steer)
        self.publisher.publish(cmd_msg)

        # Live display update
        if self.enable_tuning:
            self.draw_status_display(vel, heading_error)

        self.get_logger().info(
            f"Pose: ({px:.2f}, {py:.2f}) | Speed: {vel:.2f} m/s | Target: {self.target_speed:.2f} m/s | "
            f"Steer: {math.degrees(steer):.1f}°",
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPidControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("PID Controller Node stopped by user.")
    finally:
        if node.enable_tuning:
            cv2.destroyAllWindows()
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
