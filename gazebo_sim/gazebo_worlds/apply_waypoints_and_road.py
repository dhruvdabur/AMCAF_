import pandas as pd
import math
import re

# Load the trajectory
df = pd.read_csv('trajectory.csv')
waypoints = df.to_dict('records')

# 1. Generate new road segments XML for elliptical_road
road_links_xml = "    <model name=\"elliptical_road\">\n      <static>true</static>\n"
for i, row in enumerate(waypoints):
    x = row['x']
    y = row['y']
    yaw_rad = math.radians(row['yaw_deg'])
    
    # Each segment is 2.0m long, 6.0m wide (6.4m with yellow border), 0.02m thick
    road_links_xml += f"""
    <link name="seg_{i}">
      <pose>{x:.3f} {y:.3f} 0.01 0 0 {yaw_rad:.3f}</pose>
      
      <!-- Yellow Border Base -->
      <visual name="yellow_base_{i}">
        <pose>0 0 0.001 0 0 0</pose>
        <geometry>
          <box>
            <size>2.0 6.4 0.02</size>
          </box>
        </geometry>
        <material>
          <ambient>1.0 0.8 0.0 1.0</ambient>
          <diffuse>1.0 0.8 0.0 1.0</diffuse>
          <specular>0.1 0.1 0.1 1.0</specular>
        </material>
      </visual>
      
      <!-- Asphalt top -->
      <visual name="asphalt_top_{i}">
        <pose>0 0 0.002 0 0 0</pose>
        <geometry>
          <box>
            <size>2.0 6.0 0.02</size>
          </box>
        </geometry>
        <material>
          <ambient>0.15 0.15 0.15 1.0</ambient>
          <diffuse>0.15 0.15 0.15 1.0</diffuse>
          <specular>0.05 0.05 0.05 1.0</specular>
        </material>
      </visual>

      <!-- Center dashed line -->
      <visual name="center_line_{i}">
        <pose>0 0 0.003 0 0 0</pose>
        <geometry>
          <box>
            <size>1.0 0.15 0.001</size>
          </box>
        </geometry>
        <material>
          <ambient>0.9 0.9 0.9 1</ambient>
          <diffuse>0.9 0.9 0.9 1</diffuse>
        </material>
      </visual>
      
      <!-- Collision -->
      <collision name="collision_{i}">
        <pose>0 0 0.002 0 0 0</pose>
        <geometry>
          <box>
            <size>2.0 6.0 0.02</size>
          </box>
        </geometry>
      </collision>
    </link>"""

road_links_xml += "\n    </model>"

# Read custom_road.sdf content
with open('custom_road.sdf', 'r') as f:
    sdf_content = f.read()

# 2. Replace elliptical_road model
# Search from <model name="elliptical_road"> to the matching </model>
# Since there are nested tags, we find the first </model> after the start
model_start_re = r'<model name="elliptical_road">.*?</model>'
sdf_content, count = re.subn(model_start_re, road_links_xml, sdf_content, flags=re.DOTALL)
print(f"Replaced elliptical_road: {count} occurrence(s)")

# 3. Disable smooth_road_overlay (replace with empty/commented model)
smooth_road_re = r'<model name="smooth_road_overlay">.*?</model>'
sdf_content, count = re.subn(smooth_road_re, "<!-- smooth_road_overlay disabled -->", sdf_content, flags=re.DOTALL)
print(f"Disabled smooth_road_overlay: {count} occurrence(s)")

# 4. Generate trajectories for all 5 actors
actor_names = [
    "traffic_car_blue",
    "traffic_car_red",
    "traffic_car_green",
    "traffic_car_white",
    "traffic_car_yellow"
]

speed_m_s = 5.0
z_actor = 0.420

# Number of waypoints
N_wp = len(waypoints)

# Shift factor (spacing actors equally along the 125 waypoints)
shift_step = N_wp // len(actor_names) # 125 // 5 = 25

for idx, actor_name in enumerate(actor_names):
    # Circularly shift starting index
    start_idx = idx * shift_step
    
    # Shift waypoints and append the starting waypoint at the end to close the loop
    shifted_wps = [waypoints[(start_idx + i) % N_wp] for i in range(N_wp)]
    shifted_wps.append(waypoints[start_idx]) # Close loop
    
    # Calculate cumulative distance s_shifted
    s_shifted = [0.0]
    for i in range(1, len(shifted_wps)):
        prev = shifted_wps[i-1]
        curr = shifted_wps[i]
        d = math.hypot(curr['x'] - prev['x'], curr['y'] - prev['y'])
        s_shifted.append(s_shifted[-1] + d)
    
    # Generate waypoints XML block
    wp_xml = '        <trajectory id="0" type="traffic_loop" tension="0.8">\n'
    for i, wp in enumerate(shifted_wps):
        time = s_shifted[i] / speed_m_s
        yaw_rad = math.radians(wp['yaw_deg'])
        wp_xml += f"          <waypoint>\n"
        wp_xml += f"            <time>{time:.3f}</time>\n"
        wp_xml += f"            <pose>{wp['x']:.3f} {wp['y']:.3f} {z_actor} 0 0 {yaw_rad:.3f}</pose>\n"
        wp_xml += f"          </waypoint>\n"
    wp_xml += '        </trajectory>'
    
    # Locate the actor's block and replace its trajectory
    # We find '<actor name="ACTOR_NAME">...<trajectory ...>...</trajectory>'
    actor_pattern = re.escape(f'<actor name="{actor_name}">') + r'.*?(<trajectory id="0" type="traffic_loop" tension="0.8">.*?</trajectory>)'
    
    # Replace using re.sub with custom function to keep actor details and replace trajectory
    def repl_func(match, new_wp_xml=wp_xml):
        full_match = match.group(0)
        old_trajectory = match.group(1)
        return full_match.replace(old_trajectory, new_wp_xml)
        
    sdf_content, count = re.subn(actor_pattern, repl_func, sdf_content, flags=re.DOTALL)
    print(f"Updated actor {actor_name} trajectory: {count} occurrence(s)")

# Save modified content back to custom_road.sdf
with open('custom_road.sdf', 'w') as f:
    f.write(sdf_content)

print("custom_road.sdf updated successfully!")
