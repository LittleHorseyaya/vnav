#!/usr/bin/env python3
"""Record real ROS measurements; never synthesize flight results."""
import csv
import json
import math
import os
import time
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

rospy.init_node('lab3_tracking_metrics')
out = os.environ.get('LAB3_RESULTS', '/home/han/lab3_results')
os.makedirs(out, exist_ok=True)
desired = None
rows = []
start = time.monotonic()
actual_path = Path()
target_path = Path()
actual_path.header.frame_id = target_path.header.frame_id = 'world'
actual_pub = rospy.Publisher('/lab3/actual_path', Path, queue_size=1, latch=True)
target_pub = rospy.Publisher('/lab3/desired_path', Path, queue_size=1, latch=True)

def on_desired(msg):
    global desired
    if msg.transforms:
        desired = msg.transforms[0]

def on_current(msg):
    if desired is None:
        return
    p = msg.pose.pose.position
    d = desired.translation
    elapsed = time.monotonic() - start
    rows.append([elapsed, msg.header.stamp.to_sec(), p.x, p.y, p.z,
                 d.x, d.y, d.z,
                 math.sqrt((p.x-d.x)**2+(p.y-d.y)**2+(p.z-d.z)**2)])
    if len(rows) % 10 == 0:
        pose = PoseStamped(header=msg.header, pose=msg.pose.pose)
        pose.header.frame_id = 'world'
        target = PoseStamped()
        target.header = pose.header
        target.pose.position.x, target.pose.position.y, target.pose.position.z = d.x, d.y, d.z
        target.pose.orientation = desired.rotation
        actual_path.poses.append(pose)
        target_path.poses.append(target)
        actual_path.poses = actual_path.poses[-2000:]
        target_path.poses = target_path.poses[-2000:]
        actual_path.header.stamp = target_path.header.stamp = msg.header.stamp
        actual_pub.publish(actual_path)
        target_pub.publish(target_path)

rospy.Subscriber('/desired_state', MultiDOFJointTrajectoryPoint, on_desired, queue_size=1)
rospy.Subscriber('/current_state', Odometry, on_current, queue_size=1)
duration = float(os.environ.get('LAB3_DURATION', '65'))
while not rospy.is_shutdown() and time.monotonic()-start < duration:
    time.sleep(0.1)
with open(os.path.join(out, 'tracking.csv'), 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['wall_elapsed_s', 'ros_time_s', 'x','y','z','xd','yd','zd','error_m'])
    writer.writerows(rows)
report = {'samples':len(rows), 'measurement':'ROS current_state vs latest desired_state'}
steady = np.array([r for r in rows if r[0] >= 25.0])
if steady.size:
    report.update(rmse_m=float(np.sqrt(np.mean(steady[:,-1]**2))),
                  max_error_m=float(steady[:,-1].max()),
                  min_altitude_m=float(steady[:,4].min()),
                  max_altitude_m=float(steady[:,4].max()),
                  observed_ros_duration_s=float(steady[-1,1]-steady[0,1]))
with open(os.path.join(out, 'metrics.json'), 'w') as f:
    json.dump(report, f, indent=2)
print(json.dumps(report, indent=2), flush=True)
