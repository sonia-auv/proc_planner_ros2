import yaml
from pathlib import Path
from geometry_msgs.msg import Transform, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint as mdjTP
from builtin_interfaces.msg import Duration
from pprint import pprint


with open(f"{str(Path(__file__).parent)}/planner_test_data.yaml", "r") as file:
    data = yaml.safe_load(file)

traj = mdjTP()

pprint(data['transforms'])

transforms_data = data['transforms']

for trans in transforms_data:
    msg = Transform()
    msg.translation.x = trans['translation']['x']
    msg.translation.y = trans['translation']['y']
    msg.translation.z = trans['translation']['z']
    msg.rotation.x = trans['rotation']['x']
    msg.rotation.y = trans['rotation']['y']
    msg.rotation.z = trans['rotation']['z']
    msg.rotation.w = trans['rotation']['w']
    traj.transforms.append(msg)

velocity_data = data['velocities']

for vel in velocity_data:
    msg = Twist()
    msg.linear.x = vel['linear']['x']
    msg.linear.y = vel['linear']['y']
    msg.linear.z = vel['linear']['z']
    msg.angular.x = vel['angular']['x']
    msg.angular.y = vel['angular']['y']
    msg.angular.z = vel['angular']['z']
    traj.velocities.append(msg)

acceleration_data = data['accelerations']

for acc in acceleration_data:
    msg = Twist()
    msg.linear.x = acc['linear']['x']
    msg.linear.y = acc['linear']['y']
    msg.linear.z = acc['linear']['z']
    msg.angular.x = acc['angular']['x']
    msg.angular.y = acc['angular']['y']
    msg.angular.z = acc['angular']['z']
    traj.accelerations.append(msg)

traj.duration = Duration()
traj.duration.sec = data['time_from_start']['secs']
traj.duration.nanosec = data['time_from_start']['nsecs']

print(traj)