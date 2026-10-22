#!/bin/bash
exec > /home/han/lab4_results/bridge.log 2>&1
source /home/han/lab4_ws/devel/setup.bash
exec roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch publish_collisions:=true frame_rate:=60
