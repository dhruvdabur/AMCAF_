import pandas as pd
import math
import re

# Load the trajectory
df = pd.read_csv('trajectory.csv')
waypoints = df.to_dict('records')

# 1. Generate new road segments XML for elliptical_road
road_links_xml = "    <model name=\"elliptical_road\">\n      <static>true</static>\n"
N = len(waypoints)
for i in range(N):
    curr_wp = waypoints[i]
    next_wp = waypoints[(i + 1) % N]
    
    x1, y1 = curr_wp['x'], curr_wp['y']
    x2, y2 = next_wp['x'], next_wp['y']
    
    # Calculate midpoint coordinate for the segment center
    x_mid = (x1 + x2) / 2.0
    y_mid = (y1 + y2) / 2.0
    
    # Calculate exact distance length between waypoints
    L = math.hypot(x2 - x1, y2 - y1)
    
    # Calculate exact segment yaw heading
    yaw_rad = math.atan2(y2 - y1, x2 - x1)
    
    # Each segment is L long, 6.0m wide (6.4m with yellow border), 0.02m thick
    road_links_xml += f"""
    <link name="seg_{i}">
      <pose>{x_mid:.3f} {y_mid:.3f} 0.01 0 0 {yaw_rad:.3f}</pose>
      
      <!-- Yellow Border Base -->
      <visual name="yellow_base_{i}">
        <pose>0 0 0.001 0 0 0</pose>
        <geometry>
          <box>
            <size>{L:.3f} 6.4 0.02</size>
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
            <size>{L:.3f} 6.0 0.02</size>
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
            <size>{min(1.0, L/2.0):.3f} 0.15 0.001</size>
          </box>
        </geometry>
        <material>
          <ambient>0.9 0.9 0.9 1</ambient>
          <diffuse>0.9 0.9 0.9 1</diffuse>
        </material>
      </visual>
      
    </link>"""

road_links_xml += "\n    </model>"

# Read custom_road_backup.sdf content
with open('custom_road_backup.sdf', 'r') as f:
    sdf_content = f.read()

# Remove the green, white, and yellow actors
actor_remove_pattern = r'\s*<actor name="(traffic_car_green|traffic_car_white|traffic_car_yellow)">.*?</actor>\s*'
sdf_content, count = re.subn(actor_remove_pattern, '', sdf_content, flags=re.DOTALL)
print(f"Removed unused traffic actors: {count} occurrence(s)")

# 2. Replace elliptical_road model
# Search from <model name="elliptical_road"> to the matching </model>
# Since there are nested tags, we find the first </model> after the start
if '<model name="elliptical_road">' in sdf_content:
    model_start_re = r'<model name="elliptical_road">.*?</model>'
    sdf_content, count = re.subn(model_start_re, road_links_xml, sdf_content, flags=re.DOTALL)
    print(f"Replaced elliptical_road: {count} occurrence(s)")
else:
    # Insert before the prius include model
    include_re = r'(<include>\s*<name>prius</name>)'
    sdf_content, count = re.subn(include_re, lambda m: road_links_xml + "\n\n    " + m.group(1), sdf_content, flags=re.DOTALL)
    print(f"Inserted elliptical_road before prius include: {count} occurrence(s)")

# 3. Disable smooth_road_overlay (replace with empty/commented model)
smooth_road_re = r'<model name="smooth_road_overlay">.*?</model>'
sdf_content, count = re.subn(smooth_road_re, "<!-- smooth_road_overlay disabled -->", sdf_content, flags=re.DOTALL)
print(f"Disabled smooth_road_overlay: {count} occurrence(s)")

# 3.1. Update Prius spawn pose to match the first waypoint coordinate and yaw
first_wp = waypoints[0]
x_start = first_wp['x']
y_start = first_wp['y']
yaw_start_rad = math.radians(first_wp['yaw_deg'])

prius_pose_re = r'(<include>\s*<name>prius</name>\s*<uri>.*?</uri>\s*<pose>).*?(</pose>(?:\s*<scale>.*?</scale>)?\s*</include>)'
prius_pose_replacement = rf'\g<1>{x_start:.5f} {y_start:.5f} 0.0035 0 0 {yaw_start_rad:.5f}\g<2>'

sdf_content, count = re.subn(prius_pose_re, prius_pose_replacement, sdf_content, flags=re.DOTALL)
print(f"Updated Prius spawn pose to first waypoint: {count} occurrence(s)")

# 4. Generate trajectories for all 2 actors
actor_names = [
    "traffic_car_blue",
    "traffic_car_red"
]

speed_m_s = 0.05
z_actor = 0.0042

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
