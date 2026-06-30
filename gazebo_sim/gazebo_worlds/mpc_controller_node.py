#!/usr/bin/env python3
"""ROS 2 Node for controlling the Gazebo Prius vehicle using MPC."""

import sys
import os
import math
import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Add workspace root to sys.path to resolve hardware package imports
sys.path.append('/home/dhruv/amcaf')
from hardware.hil.controllers.mpc_cbf import MPCController, MPCConfig

class GazeboMpcControllerNode(Node):
    """ROS 2 Node wrapping the Model Predictive Controller (MPC) for Prius tracking."""

    def __init__(self):
        super().__init__('gazebo_mpc_controller')

        # Node parameters
        self.declare_parameter('trajectory_file', '/home/dhruv/amcaf/gazebo_sim/gazebo_worlds/trajectory.csv')
        self.declare_parameter('target_speed', 5.0)  # m/s
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('odom_topic', '/model/prius/odometry')
        self.declare_parameter('cmd_topic', '/model/prius/cmd_vel')

        trajectory_file = self.get_parameter('trajectory_file').value
        self.target_speed = self.get_parameter('target_speed').value
        control_rate = self.get_parameter('control_rate').value
        odom_topic = self.get_parameter('odom_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value

        self.get_logger().info(f"Loading trajectory waypoints from: {trajectory_file}")
        
        # Load waypoints
        try:
            df = pd.read_csv(trajectory_file)
            track_points = df[['x', 'y']].to_numpy()
            self.get_logger().info(f"Loaded {len(track_points)} waypoints successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load trajectory file: {e}")
            sys.exit(1)

        # Initialize MPC Controller
        dt = 1.0 / control_rate
        mpc_config = MPCConfig(
            wheelbase=2.86,        # Prius wheelbase in meters
            delta_t=dt,            # Time step matching the node control rate
            horizon_T=12,          # Lookahead horizon steps
            max_steer=0.6,         # Prius steering limit in radians
            min_steer=-0.6,
            max_accel=2.0,         # Maximum acceleration in m/s^2
            min_accel=-5.0,        # Maximum deceleration
            v_min=0.0,
            v_max=12.0
        )
        self.controller = MPCController(mpc_config)
        self.controller.update_track(track_points)

        # State variables
        self.current_odom = None
        self.vel_cmd = 0.0

        # ROS 2 Subscribers and Publishers
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        self.publisher = self.create_publisher(
            Twist,
            cmd_topic,
            10
        )

        # Periodic timer for the control loop
        self.timer = self.create_timer(dt, self.timer_callback)
        self.get_logger().info(f"MPC Controller Node initialized at {control_rate}Hz.")

    def odom_callback(self, msg):
        """Cache the latest odometry message."""
        self.current_odom = msg

    def timer_callback(self):
        """Execute one step of the MPC solver and publish velocity and steering commands."""
        if self.current_odom is None:
            self.get_logger().warning("Waiting for odometry messages...", throttle_duration_sec=3.0)
            return

        # 1. Extract current state from Odometry
        px = self.current_odom.pose.pose.position.x
        py = self.current_odom.pose.pose.position.y

        # Quaternion to Euler Yaw (Heading)
        q = self.current_odom.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Current speed calculation (preserving direction direction)
        vx = self.current_odom.twist.twist.linear.x
        vy = self.current_odom.twist.twist.linear.y
        vel = math.hypot(vx, vy)
        if vx < 0:
            vel = -vel

        # 2. Run MPC optimization step
        dt = self.controller.delta_t
        accel, steer = self.controller.step(px, py, yaw, vel, self.target_speed)

        # 3. Integrate acceleration to compute the target velocity command
        self.vel_cmd = self.vel_cmd + accel * dt
        self.vel_cmd = max(0.0, min(self.target_speed, self.vel_cmd))

        # 4. Construct and publish the command message
        cmd_msg = Twist()
        cmd_msg.linear.x = float(self.vel_cmd)
        cmd_msg.angular.z = float(steer)
        self.publisher.publish(cmd_msg)

        # Detailed logging
        self.get_logger().info(
            f"Pose: ({px:.2f}, {py:.2f}) | Heading: {math.degrees(yaw):.1f}° | Speed: {vel:.2f} m/s | "
            f"Cmd: speed={self.vel_cmd:.2f} m/s, steer={math.degrees(steer):.1f}° (accel={accel:.3f} m/s²)",
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
        # Publish a final zero velocity command before shutting down
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
