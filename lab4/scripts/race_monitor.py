#!/usr/bin/env python3
"""Record actual ROS flight, ordered gate passages and aligned tracking errors."""
import csv
import json
import math
import os
from pathlib import Path
import time
import numpy as np
import rospy
import tf.transformations as tf
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from tesse_ros_bridge.msg import CollisionStats

rospy.init_node('lab4_race_monitor')
folder=Path(os.environ.get('LAB4_RUN_DIR','/home/han/lab4_results/race_attempt01'))
folder.mkdir(parents=True,exist_ok=True)
current, desired, collisions, events = [], [], [], []
gates=[]
normals=[]
minimum=[]
next_gate=0
previous=None
planned_start=None
duration=None
actual_path=RosPath()
target_path=RosPath()
actual_path.header.frame_id=target_path.header.frame_id='world'
actual_pub=rospy.Publisher('/lab4/actual_path',RosPath,queue_size=1,latch=True)
target_pub=rospy.Publisher('/lab4/desired_path',RosPath,queue_size=1,latch=True)
marker_pub=rospy.Publisher('/lab4/gates',MarkerArray,queue_size=1,latch=True)
began_wall=time.monotonic()

def waypoint_cb(msg):
    global gates,normals,minimum
    if gates: return
    for pose in msg.poses:
        gates.append(np.array([pose.position.x,pose.position.y,pose.position.z]))
        q=pose.orientation
        normals.append(tf.quaternion_matrix([q.x,q.y,q.z,q.w])[:3,0])
    minimum=[float('inf')]*len(gates)
    (folder/'gates.json').write_text(json.dumps([p.tolist() for p in gates],indent=2))

def markers(position,stamp):
    result=MarkerArray()
    for i,g in enumerate(gates):
        mark=Marker()
        mark.header.frame_id='world'; mark.header.stamp=stamp
        mark.ns='gate_centers'; mark.id=i; mark.type=Marker.SPHERE
        mark.pose.position.x,mark.pose.position.y,mark.pose.position.z=g
        mark.pose.orientation.w=1
        mark.scale.x=mark.scale.y=mark.scale.z=.25
        mark.color.r=.2 if i<next_gate else 1
        mark.color.g=1; mark.color.b=.5 if i<next_gate else .9; mark.color.a=1
        result.markers.append(mark)
        label=Marker()
        label.header=mark.header; label.ns='gate_labels'; label.id=i
        label.type=Marker.TEXT_VIEW_FACING
        label.pose.position.x,label.pose.position.y,label.pose.position.z=g+[0,0,.6]
        label.pose.orientation.w=1; label.scale.z=.45
        label.color.r=label.color.g=label.color.b=label.color.a=1
        label.text=str(i+1)
        result.markers.append(label)
    drone=Marker()
    drone.header.frame_id='world'; drone.header.stamp=stamp
    drone.ns='actual_drone'; drone.id=0; drone.type=Marker.SPHERE
    drone.pose.position.x,drone.pose.position.y,drone.pose.position.z=position
    drone.pose.orientation.w=1
    drone.scale.x=drone.scale.y=drone.scale.z=.4
    drone.color.r=1;drone.color.g=.45;drone.color.b=.05;drone.color.a=1
    result.markers.append(drone)
    marker_pub.publish(result)

def state_cb(msg):
    global previous,next_gate
    p=msg.pose.pose.position; q=msg.pose.pose.orientation
    v=msg.twist.twist.linear; o=msg.twist.twist.angular
    t=msg.header.stamp.to_sec()
    pos=np.array([p.x,p.y,p.z])
    current.append([t,p.x,p.y,p.z,v.x,v.y,v.z,q.x,q.y,q.z,q.w,o.x,o.y,o.z])
    if gates and planned_start is not None and t>=planned_start:
        distances=[float(np.linalg.norm(pos-g)) for g in gates]
        minimum[:]=[min(a,b) for a,b in zip(minimum,distances)]
        if next_gate<len(gates):
            method=None; crossing_error=None
            g,n=gates[next_gate],normals[next_gate]
            if previous is not None:
                last_t,last_pos=previous
                d0=float((last_pos-g)@n); d1=float((pos-g)@n)
                if d0*d1<=0 and abs(d0-d1)>1e-9:
                    cross=last_pos+d0/(d0-d1)*(pos-last_pos)
                    crossing_error=float(np.linalg.norm(cross-g))
                    if crossing_error<=.65: method='plane_crossing_within_0.65m'
            if method is None and distances[next_gate]<=.65:
                method='center_reached_within_0.65m'
            if method:
                events.append(dict(gate=next_gate+1,ros_time=t,elapsed_s=t-planned_start,
                                   method=method,distance_m=distances[next_gate],
                                   plane_offset_m=crossing_error))
                next_gate+=1
                print('Gate',next_gate,'/',len(gates),'reached at',round(t-planned_start,2),flush=True)
    previous=(t,pos)
    if len(current)%5==0:
        pose=PoseStamped(header=msg.header,pose=msg.pose.pose)
        pose.header.frame_id='world'
        actual_path.poses.append(pose)
        actual_path.poses=actual_path.poses[-5000:]
        actual_path.header.stamp=msg.header.stamp
        actual_pub.publish(actual_path)
        markers(pos,msg.header.stamp)

def desired_cb(msg):
    if not msg.transforms: return
    p=msg.transforms[0].translation
    v=msg.velocities[0].linear
    a=msg.accelerations[0].linear
    t=msg.time_from_start.to_sec()
    desired.append([t,p.x,p.y,p.z,v.x,v.y,v.z,a.x,a.y,a.z])
    if len(desired)%5==0:
        pose=PoseStamped()
        pose.header.frame_id='world'; pose.header.stamp=rospy.Time.now()
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=p.x,p.y,p.z
        pose.pose.orientation=msg.transforms[0].rotation
        target_path.poses.append(pose)
        target_path.poses=target_path.poses[-5000:]
        target_path.header.stamp=pose.header.stamp
        target_pub.publish(target_path)

def collision_cb(msg):
    if msg.is_collision:
        collisions.append(dict(ros_time=msg.header.stamp.to_sec(),object=msg.object_name))

rospy.Subscriber('/current_state',Odometry,state_cb,queue_size=20)
rospy.Subscriber('/desired_state',MultiDOFJointTrajectoryPoint,desired_cb,queue_size=50)
rospy.Subscriber('/desired_traj_vertices',PoseArray,waypoint_cb,queue_size=1)
rospy.Subscriber('/tesse/collision',CollisionStats,collision_cb,queue_size=20)
reason='wall_timeout'
while not rospy.is_shutdown() and time.monotonic()-began_wall<900:
    if planned_start is None and rospy.has_param('/trajectory_generation_node/trajectory_start_time'):
        planned_start=float(rospy.get_param('/trajectory_generation_node/trajectory_start_time'))
        duration=float(rospy.get_param('/trajectory_generation_node/trajectory_duration'))
    now=rospy.Time.now().to_sec()
    status=dict(gates_reached=next_gate,gate_count=len(gates),
                elapsed_s=None if planned_start is None else now-planned_start,
                planned_duration_s=duration,collision_messages=len(collisions))
    (folder/'status.json').write_text(json.dumps(status,indent=2))
    if planned_start is not None and now>planned_start+duration+8:
        reason='trajectory_finished'; break
    time.sleep(.5)

for name,header,rows in [('current_state.csv',['ros_time','x','y','z','vx','vy','vz','qx','qy','qz','qw','wx','wy','wz'],current),
                         ('desired_state.csv',['time_from_start','x','y','z','vx','vy','vz','ax','ay','az'],desired)]:
    with (folder/name).open('w',newline='') as f:
        w=csv.writer(f);w.writerow(header);w.writerows(rows)
report=dict(status,stop_reason=reason,success=bool(gates and next_gate==len(gates)),
            gate_tolerance_m=.65,gate_events=events,minimum_gate_distances_m=minimum,
            collision_events=collisions,current_samples=len(current),desired_samples=len(desired))
if planned_start is not None and current and desired:
    c=np.array(current);d=np.array(desired)
    _,indices=np.unique(d[:,0],return_index=True);d=d[indices]
    elapsed=c[:,0]-planned_start
    use=(elapsed>=0)&(elapsed<=duration)
    ref=np.column_stack([np.interp(elapsed[use],d[:,0],d[:,axis]) for axis in (1,2,3)])
    error=np.linalg.norm(c[use,1:4]-ref,axis=1)
    report.update(rmse_m=float(np.sqrt(np.mean(error**2))),max_error_m=float(error.max()))
    aligned=np.column_stack([elapsed[use],c[use,1:4],ref,error])
    np.savetxt(folder/'tracking_aligned.csv',aligned,delimiter=',',
               header='elapsed_ros_s,x,y,z,xd,yd,zd,error_m',comments='')
(folder/'race_result.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2),flush=True)
