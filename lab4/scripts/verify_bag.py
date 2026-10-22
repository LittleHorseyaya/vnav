#!/usr/bin/env python3
"""Independent offline checks against the saved ROS messages."""
import json,sys
from pathlib import Path
import numpy as np
import rosbag
from tf.transformations import quaternion_matrix,euler_from_quaternion

folder=Path(sys.argv[1])
current=[]; desired=[]; gates=None
with rosbag.Bag(str(folder/'race.bag')) as bag:
    for topic,msg,stamp in bag.read_messages(topics=['/current_state','/desired_state','/desired_traj_vertices']):
        if topic=='/current_state':
            p=msg.pose.pose.position
            current.append([msg.header.stamp.to_sec(),p.x,p.y,p.z])
        elif topic=='/desired_state':
            p=msg.transforms[0].translation;v=msg.velocities[0].linear;a=msg.accelerations[0].linear
            q=msg.transforms[0].rotation
            yaw=euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
            desired.append([msg.time_from_start.to_sec(),p.x,p.y,p.z,v.x,v.y,v.z,a.x,a.y,a.z,yaw])
        elif gates is None:gates=msg.poses
c=np.array(current);d=np.array(desired)
crossings=[];after=-float('inf');minima=[];desired_minima=[];reference_line_minima=[]
for i,pose in enumerate(gates):
    g=np.array([pose.position.x,pose.position.y,pose.position.z]);q=pose.orientation
    normal=quaternion_matrix([q.x,q.y,q.z,q.w])[:3,0]
    distances=np.linalg.norm(c[:,1:4]-g,axis=1)
    minima.append(float(distances.min()))
    desired_minima.append(float(np.linalg.norm(d[:,1:4]-g,axis=1).min()))
    delta=d[1:,1:4]-d[:-1,1:4]
    fraction=np.clip(np.sum((g-d[:-1,1:4])*delta,axis=1)/np.maximum(np.sum(delta*delta,axis=1),1e-20),0,1)
    reference_line_minima.append(float(np.linalg.norm(d[:-1,1:4]+fraction[:,None]*delta-g,axis=1).min()))
    side=(c[:,1:4]-g)@normal
    crossing=None
    for n in np.flatnonzero((side[:-1]*side[1:]<=0)&(np.abs(np.diff(side))>1e-8)):
        if c[n,0]<=after:continue
        alpha=side[n]/(side[n]-side[n+1])
        point=c[n,1:4]+alpha*(c[n+1,1:4]-c[n,1:4])
        offset=float(np.linalg.norm(point-g))
        if offset<.5:
            crossing=dict(gate=i+1,ros_time=float(c[n,0]),offset_m=offset)
            after=float(c[n,0]);break
    crossings.append(crossing)
end=gates[-1].position
terminal_error=float(np.linalg.norm(c[-1,1:4]-[end.x,end.y,end.z]))
report=dict(finite_states=bool(np.isfinite(c).all() and np.isfinite(d).all()),
    time_from_start_nondecreasing=bool((np.diff(d[:,0])>=-1e-8).all()),
    terminal_reference_speed_mps=float(np.linalg.norm(d[-1,4:7])),
    terminal_reference_acceleration_mps2=float(np.linalg.norm(d[-1,7:10])),
    largest_sampled_yaw_step_rad=float(np.abs(np.diff(np.unwrap(d[:,-1]))).max()),
    actual_minimum_gate_distances_m=minima,desired_minimum_gate_distances_m=desired_minima,
    interpolated_reference_gate_distances_m=reference_line_minima,
    largest_reference_sample_time_gap_s=float(np.diff(d[:,0]).max()),
    strict_plane_crossings=crossings,terminal_actual_error_m=terminal_error,
    first_11_gates_crossed_in_order=all(x is not None for x in crossings[:11]),
    final_gate_reached=terminal_error<.2,current_messages=len(c),desired_messages=len(d))
assert report['finite_states'] and report['time_from_start_nondecreasing']
assert report['first_11_gates_crossed_in_order'] and report['final_gate_reached']
assert report['terminal_reference_speed_mps']<1e-6
assert max(reference_line_minima)<.02
(folder/'offline_validation.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
