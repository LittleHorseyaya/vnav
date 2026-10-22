#!/bin/bash
set -e
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export XDG_RUNTIME_DIR=/run/user/1000
out=/home/han/lab3_results/circle
mkdir -p "$out"
exec > "$out/recording.log" 2>&1
size=$(xdpyinfo | awk '/dimensions:/ {print $2; exit}')
exec ffmpeg -y -f x11grab -video_size "$size" -framerate 15 -i :0.0 \
    -t 50 -vf 'scale=1280:-2' -c:v libx264 -preset ultrafast -crf 23 \
    -pix_fmt yuv420p -movflags +faststart "$out/rviz_drone_flight.mp4"
