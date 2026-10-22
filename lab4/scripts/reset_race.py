#!/usr/bin/env python3
import csv,time
import rospy
from geometry_msgs.msg import Pose
from mav_msgs.msg import Actuators
from tesse_ros_bridge.srv import RepositionRequestService
rospy.init_node('lab4_reset')
pub=rospy.Publisher('/rotor_speed_cmds',Actuators,queue_size=1)
time.sleep(.3)
for _ in range(3):
    pub.publish(Actuators(angular_velocities=[0,0,0,0]));time.sleep(.05)
rospy.wait_for_service('/tesse/reposition_request',timeout=15)
with open('/home/han/vnav-builds/lab4/lab4_Data/StreamingAssets/vnav-2020-lab4_static_tfs.csv') as f:
    row=next(r for r in csv.reader(f) if r[0]=='QuadUAV')
values=[float(v) for v in row[1:]]
pose=Pose()
pose.position.x,pose.position.y,pose.position.z=values[:3]
pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=values[3:]
print(rospy.ServiceProxy('/tesse/reposition_request',RepositionRequestService)(pose))
# Unity retains the last rotor command. Hold level altitude while the planner
# starts, rather than letting the reset vehicle fall during node startup.
pub.publish(Actuators(angular_velocities=[(9.81/(4e-3))**.5]*4))
time.sleep(.1)
