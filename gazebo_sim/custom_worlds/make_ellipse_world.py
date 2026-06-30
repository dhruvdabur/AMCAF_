import math
from pathlib import Path

# --------- Road parameters ----------
A = 28.0          # ellipse radius in x direction
B = 16.0          # ellipse radius in y direction
ROAD_WIDTH = 5.0
ROAD_THICKNESS = 0.08
N = 160           # number of road segments; higher = smoother
OUTPUT = "ellipse_road.sdf"

# --------- World header ----------
sdf = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="ellipse_road_world">

    <plugin
      filename="ignition-gazebo-physics-system"
      name="ignition::gazebo::systems::Physics"/>

    <plugin
      filename="ignition-gazebo-user-commands-system"
      name="ignition::gazebo::systems::UserCommands"/>

    <plugin
      filename="ignition-gazebo-scene-broadcaster-system"
      name="ignition::gazebo::systems::SceneBroadcaster"/>

    <light name="sun" type="directional">
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.2 -1</direction>
    </light>

    <!-- Ground -->
    <model name="ground">
      <static>true</static>
      <pose>0 0 -0.03 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>80 60 0.04</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>80 60 0.04</size>
            </box>
          </geometry>
          <material>
            <ambient>0.25 0.45 0.25 1</ambient>
            <diffuse>0.25 0.45 0.25 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

# --------- Generate road segments ----------
for i in range(N):
    t1 = 2 * math.pi * i / N
    t2 = 2 * math.pi * (i + 1) / N

    x1 = A * math.cos(t1)
    y1 = B * math.sin(t1)

    x2 = A * math.cos(t2)
    y2 = B * math.sin(t2)

    mx = (x1 + x2) / 2
    my = (y1 + y2) / 2

    dx = x2 - x1
    dy = y2 - y1

    length = math.sqrt(dx * dx + dy * dy) * 1.25
    yaw = math.atan2(dy, dx)

    sdf += f"""
    <model name="road_segment_{i}">
      <static>true</static>
      <pose>{mx:.3f} {my:.3f} 0.02 0 0 {yaw:.6f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{length:.3f} {ROAD_WIDTH:.3f} {ROAD_THICKNESS:.3f}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{length:.3f} {ROAD_WIDTH:.3f} {ROAD_THICKNESS:.3f}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

# --------- Generate dashed center line ----------
for i in range(0, N, 8):
    t1 = 2 * math.pi * i / N
    t2 = 2 * math.pi * (i + 2) / N

    x1 = A * math.cos(t1)
    y1 = B * math.sin(t1)

    x2 = A * math.cos(t2)
    y2 = B * math.sin(t2)

    mx = (x1 + x2) / 2
    my = (y1 + y2) / 2

    dx = x2 - x1
    dy = y2 - y1

    length = math.sqrt(dx * dx + dy * dy) * 0.9
    yaw = math.atan2(dy, dx)

    sdf += f"""
    <model name="center_dash_{i}">
      <static>true</static>
      <pose>{mx:.3f} {my:.3f} 0.085 0 0 {yaw:.6f}</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>{length:.3f} 0.18 0.015</size>
            </box>
          </geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

# --------- Finish world ----------
sdf += """
  </world>
</sdf>
"""

Path(OUTPUT).write_text(sdf)
print(f"Generated {OUTPUT}")

