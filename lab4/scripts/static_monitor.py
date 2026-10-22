#!/usr/bin/env python3
import json,time
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
rospy.init_node('lab4_static_monitor')
target=None
samples=[]
def gates(msg):
    global target
    if msg.poses:
        p=msg.poses[0].position;target=np.array([p.x,p.y,p.z])
def state(msg):
    if target is None:return
    p=msg.pose.pose.position;v=msg.twist.twist.linear
    samples.append([msg.header.stamp.to_sec(),p.x,p.y,p.z,
                    float(np.linalg.norm(np.array([p.x,p.y,p.z])-target)),
                    float(np.linalg.norm([v.x,v.y,v.z]))])
rospy.Subscriber('/desired_traj_vertices',PoseArray,gates,queue_size=1)
rospy.Subscriber('/current_state',Odometry,state,queue_size=20)
start=time.monotonic();stable_since=None;success=False
while not rospy.is_shutdown() and time.monotonic()-start<120:
    if samples:
        s=samples[-1]
        if s[4]<.4 and s[5]<.3:
            if stable_since is None:stable_since=s[0]
            if s[0]-stable_since>=2:success=True;break
        else:stable_since=None
    time.sleep(.1)
report=dict(success=success,samples=len(samples),target=None if target is None else target.tolist(),
            last_state=None if not samples else samples[-1])
with open('/home/han/lab4_results/static/result.json','w') as f:json.dump(report,f,indent=2)
if samples:np.savetxt('/home/han/lab4_results/static/flight.csv',samples,delimiter=',',
                      header='ros_time,x,y,z,error_m,speed_mps',comments='')
print(json.dumps(report,indent=2),flush=True)
