#!/bin/bash
exec > /tmp/lab4-sim-start.log 2>&1
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export XDG_RUNTIME_DIR=/run/user/1000
export PULSE_SERVER=unix:/run/user/1000/pulse/native
cd /home/han/vnav-builds/lab4
echo $$ > /home/han/lab4_results/simulator.pid
exec ./lab4.x86_64 -noaudio -disableManagedDebugger -screen-fullscreen 0 -screen-width 960 -screen-height 540 -logFile /home/han/lab4_results/unity.log
