import pandas as pd
import math

df = pd.read_csv('trajectory.csv')

# Speed in meters per second
speed_m_s = 5.0

with open('waypoints_output.txt', 'w') as f:
    for index, row in df.iterrows():
        # Time = cumulative distance / speed
        time = row['s'] / speed_m_s
        
        x = row['x']
        y = row['y']
        z = 0.01  # Slight elevation to avoid clipping
        
        # Convert yaw from degrees to radians
        yaw_rad = math.radians(row['yaw_deg'])
        
        waypoint_xml = f"      <waypoint>\n"
        waypoint_xml += f"        <time>{time:.2f}</time>\n"
        waypoint_xml += f"        <pose>{x:.3f} {y:.3f} {z} 0 0 {yaw_rad:.3f}</pose>\n"
        waypoint_xml += f"      </waypoint>\n"
        
        f.write(waypoint_xml)

print("Waypoints generation completed successfully.")
