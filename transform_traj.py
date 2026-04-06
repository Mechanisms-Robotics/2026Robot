import json
import math

# Load the original trajectory
with open('src/main/deploy/choreo/ShuttlingRight.traj', 'r') as f:
    data = json.load(f)

# Change name
data['name'] = 'ShuttlingRightMirrored'

# Function to mirror y and heading
def mirror_point(point):
    if isinstance(point['y'], dict):
        # params
        point['y']['val'] = 8 - point['y']['val']
        point['heading']['val'] = math.pi - point['heading']['val']
    else:
        # snapshot
        point['y'] = 8 - point['y']
        point['heading'] = math.pi - point['heading']
    return point

# Mirror waypoints in snapshot
for wp in data['snapshot']['waypoints']:
    mirror_point(wp)

# Mirror waypoints in params
for wp in data['params']['waypoints']:
    mirror_point(wp)
    # Update exp
    y_val = 8 - float(wp['y']['exp'].split()[0])
    wp['y']['exp'] = f"{y_val} m"
    # Heading
    h_deg = float(wp['heading']['exp'].split()[0])
    new_h_deg = 180 - h_deg
    wp['heading']['exp'] = f"{new_h_deg} deg"

# Mirror samples y and heading
for sample in data['trajectory']['samples']:
    sample['y'] = 8 - sample['y']
    sample['heading'] = math.pi - sample['heading']
    # Also flip velocities: vx same, vy negate, omega negate
    sample['vy'] = -sample['vy']
    sample['omega'] = -sample['omega']
    # ax same, ay negate
    sample['ay'] = -sample['ay']
    # alpha negate
    sample['alpha'] = -sample['alpha']
    # fy negate
    sample['fy'] = [-f for f in sample['fy']]

# Save the new file
with open('src/main/deploy/choreo/ShuttlingRightMirrored.traj', 'w') as f:
    json.dump(data, f, indent=2)
