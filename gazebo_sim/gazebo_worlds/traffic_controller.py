#!/usr/bin/env python3
"""
traffic_controller.py
─────────────────────
Moves traffic_box_blue and traffic_box_red along trajectory.csv
using ROS 2 Pose publishers and SetEntityPose service client.
"""

import sys
import math
import pandas as pd
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from geometry_msgs.msg import Pose

# ── constants ────────────────────────────────────────────────────────
WORLD_NAME      = "custom_road_world"
BOX_HEIGHT      = 0.0035      # sits on the road surface
BLUE_START_FRAC = 0.50        # blue starts 50% through the trajectory
RED_START_FRAC  = 0.75        # red starts 75% through the trajectory
RED_LANE_OFFSET = 0.02        # 2 cm lateral offset
SPEED_FACTOR    = 1.0         # speed multiplier
CONTROL_RATE    = 20.0        # Hz
# ────────────────────────────────────────────────────────────────────


class TrafficController(Node):
    def __init__(self, csv_path):
        super().__init__('traffic_controller')
        
        # Load trajectory
        self.get_logger().info(f"Loading trajectory from: {csv_path}")
        try:
            df = pd.read_csv(csv_path)
            # trajectory.csv columns: index,s,d_offset,x,y,yaw_deg
            self.waypoints = df[['x', 'y', 'yaw_deg']].to_numpy()
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints.")
        except Exception as e:
            self.get_logger().error(f"Failed to load CSV: {e}")
            sys.exit(1)
            
        self.n = len(self.waypoints)
        if self.n < 2:
            self.get_logger().error("Need at least 2 waypoints in CSV.")
            sys.exit(1)
            
        self.blue_idx = int(self.n * BLUE_START_FRAC)
        self.red_idx = int(self.n * RED_START_FRAC)
        
        # Calculate cumulative distance along track
        self.track_distances = [0.0]
        for i in range(1, self.n):
            d = math.hypot(self.waypoints[i][0] - self.waypoints[i-1][0],
                           self.waypoints[i][1] - self.waypoints[i-1][1])
            self.track_distances.append(self.track_distances[-1] + d)
        self.total_length = self.track_distances[-1]
        
        # Service client for set_pose
        self.set_pose_client = self.create_client(SetEntityPose, f"/world/{WORLD_NAME}/set_pose")
        
        # Topic publishers for model poses
        self.blue_pose_pub = self.create_publisher(Pose, "/model/traffic_box_blue/pose", 10)
        self.red_pose_pub = self.create_publisher(Pose, "/model/traffic_box_red/pose", 10)
        
        # Grid distance counters
        self.blue_s = self.track_distances[self.blue_idx]
        self.red_s = self.track_distances[self.red_idx]
        
        # Standard speed = 0.05 m/s (scaled)
        self.speed = 0.05 * SPEED_FACTOR
        self.dt = 1.0 / CONTROL_RATE
        
        # Create timer
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info("Traffic Controller initialized.")

    def teleport_model(self, name, x, y, z, yaw):
        if not self.set_pose_client.service_is_ready():
            return
        
        req = SetEntityPose.Request()
        req.entity.name = name
        req.entity.type = Entity.MODEL
        
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        req.pose.orientation.w = cy
        req.pose.orientation.x = 0.0
        req.pose.orientation.y = 0.0
        req.pose.orientation.z = sy
        
        self.set_pose_client.call_async(req)

    def publish_pose_topic(self, pub, x, y, z, yaw):
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        msg.orientation.w = cy
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = sy
        
        pub.publish(msg)

    def get_pose_at_s(self, s):
        # Find index
        import numpy as np
        idx = np.searchsorted(self.track_distances, s)
        if idx >= self.n:
            idx = self.n - 1
        wp = self.waypoints[idx]
        
        # x and y are scaled down by 0.01 to match 1/100 scale
        x = wp[0] * 0.01
        y = wp[1] * 0.01
        yaw = math.radians(wp[2])
        
        return x, y, yaw

    def timer_callback(self):
        # Update distances
        self.blue_s = (self.blue_s + self.speed * self.dt) % self.total_length
        self.red_s = (self.red_s + self.speed * self.dt) % self.total_length
        
        # Blue pose
        bx, by, byaw = self.get_pose_at_s(self.blue_s)
        self.teleport_model("traffic_box_blue", bx, by, BOX_HEIGHT, byaw)
        self.publish_pose_topic(self.blue_pose_pub, bx, by, BOX_HEIGHT, byaw)
        
        # Red pose (with lane offset of 0.02)
        rx, ry, ryaw = self.get_pose_at_s(self.red_s)
        # Apply lateral offset to place red box in adjacent lane
        rx_offset = rx + RED_LANE_OFFSET * math.cos(ryaw + math.pi / 2.0)
        ry_offset = ry + RED_LANE_OFFSET * math.sin(ryaw + math.pi / 2.0)
        self.teleport_model("traffic_box_red", rx_offset, ry_offset, BOX_HEIGHT, ryaw)
        self.publish_pose_topic(self.red_pose_pub, rx_offset, ry_offset, BOX_HEIGHT, ryaw)


def main(args=None):
    rclpy.init(args=args)
    csv_path = "trajectory.csv"
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        csv_path = sys.argv[1]
    
    node = TrafficController(csv_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()