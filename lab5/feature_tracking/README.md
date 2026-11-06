# LAB5 feature tracking

The SIFT path now detects and describes keypoints, uses FLANN to find two
neighbors per descriptor, applies the lab's 0.8 distance-ratio filter, and
uses the provided fundamental-matrix RANSAC helper to classify inliers.
It handles empty descriptors and frames with too few matches. SURF reuses the
same FLANN ratio filter with floating-point descriptors. ORB uses Hamming
distance for binary descriptors. Harris corners with pyramidal LK optical
flow provide frame-to-frame tracks. FAST+BRIEF and dense optical flow remain
optional lab TODOs.

Prefer **Ubuntu 20.04 with ROS 1 Noetic** for this release. The package uses
ROS 1 APIs (`roslaunch`, `rosbag`, and catkin), so a ROS 2-only Ubuntu 22.04
installation cannot build it without porting. Check the VM first:

```bash
lsb_release -ds
echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
command -v roscore
command -v catkin
```

The repository builds OpenCV 3.4.2 through
`opencv3_catkin`. The build enables `OPENCV_ENABLE_NONFREE` for the historical
`cv::xfeatures2d::SIFT` API. Keep exactly one copy each of `catkin_simple`,
`gflags_catkin`, `glog_catkin`, and `opencv3_catkin` in the catkin workspace.
All four are included in this submission's `lab5/dependencies` directory.
If they are already present in your workspace, do not copy duplicates. For a
fresh catkin workspace, link the packages from this checkout (replace the
path on the first line):

```bash
LAB5_ROOT=/absolute/path/to/vnav/lab5
mkdir -p ~/vnav_ws/src
ln -s "$LAB5_ROOT/dependencies/catkin_simple" ~/vnav_ws/src/catkin_simple
ln -s "$LAB5_ROOT/dependencies/glog_catkin" ~/vnav_ws/src/glog_catkin
ln -s "$LAB5_ROOT/dependencies/gflags_catkin" ~/vnav_ws/src/gflags_catkin
ln -s "$LAB5_ROOT/dependencies/opencv3_catkin" ~/vnav_ws/src/opencv3_catkin
ln -s "$LAB5_ROOT/feature_tracking" ~/vnav_ws/src/feature_tracking
```

Source your ROS 1 installation first, then build and source the workspace:

```bash
source /opt/ros/noetic/setup.bash
cd ~/vnav_ws
catkin build lab_5
source devel/setup.bash
```

On the tested Ubuntu 20.04 VM, the build also needed
`sudo apt install libtool libgtk2.0-dev pkg-config`. The OpenCV 3.4.2 source
build is the slowest step. The VM cannot currently download GitHub archives
directly, so its successful build used verified source archives copied into
the catkin build cache. Keep the existing `~/vnav_ws/build` directory if you
rebuild offline.
The linker reports an OpenCV 3.4/4.2 warning because Noetic's `cv_bridge`
uses the system OpenCV 4.2 while this lab builds OpenCV 3.4. Both run modes
worked on this VM, but check this dependency mix if you later see image
conversion or GUI crashes.

## Run modes

**Two images (`mode=0`)** reads the bundled `box.png` and
`box_in_scene.png`, prints the SIFT counts, and saves five PNG files into
`feature_tracking/results`:

```bash
roslaunch lab_5 two_frames_tracking.launch descriptor:=SIFT
```

The files are `keypoints_1.png`, `keypoints_2.png`, `matches_raw.png`,
`matches_ratio.png`, and `matches_inliers.png`. In the last file, green lines
are RANSAC inliers and red lines are outliers among ratio-filtered matches.
You may pass `output_dir:=/existing/directory`; that directory must exist.
Pass `visualize:=true` to open GUI windows. The two-image process then waits
for a key press. The default is `false`, so the command works without a display.
`roslaunch` can remain open after the two-image node exits; press Ctrl-C after
the statistics appear.

The Ubuntu 20.04 VM produced these results for the bundled sample pair;
counts can vary slightly with dependencies:

| Method | Keypoints 1 | Keypoints 2 | Raw matches | Ratio matches | RANSAC inliers | Inlier ratio |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| SIFT | 603 | 969 | 603 | 99 | 79 | 79.8% |
| SURF | 997 | 1822 | 997 | 129 | 85 | 65.9% |
| ORB | 500 | 500 | 500 | 13 | 7 | 53.8% |

Choose `descriptor:=SURF` or `descriptor:=ORB` in the two-image command to
reproduce those rows. Set a separate `output_dir` per method if you want to
keep all result PNGs; otherwise the launch overwrites the previous PNGs.

**Video (`mode=1`)** plays a rosbag, processes consecutive frames, and reports
average keypoints, matches, inliers, and inlier ratio when the node stops:

```bash
roslaunch lab_5 video_tracking.launch descriptor:=SIFT path_to_dataset:=/absolute/path/to/file.bag
```

The first frame is used only as a reference. Press Ctrl-C after playback to
stop the subscriber and print the averages. The launch file remaps
`/tesse/left_cam/rgb/image_raw` to `/camera/color/image_raw`; check your
bag's image topic with `rosbag info` if no frames are processed. Video mode
does not save PNGs by default. Pass `visualize:=true` when a display is
available.
For a slower VM, add `playback_rate:=0.6` to avoid dropping frames. Check
`Num. of samples` against the image message count minus one. For runs that
automatically stop and save logs after the bag finishes, use:

```bash
bash $(rospack find lab_5)/run_bag_benchmark.sh SIFT \
  /absolute/path/to/file.bag 0.6 /absolute/path/to/log-directory \
  /camera/color/image_raw
```

The final argument is the image topic inside the bag. For the course's
`vnav-lab5-smooth-trajectory.bag`, pass
`/tesse/left_cam/rgb/image_raw`; for
`30fps_424x240_2018-10-01-18-35-06.bag`, pass
`/camera/color/image_raw`. The script maps either topic to the tracker's
input, plays only that image topic, and prints the averages when playback
ends. It stores the full tracker and rosbag logs in the chosen directory.
The public course datasets are linked from
https://github.com/MIT-SPARK/VNAV-lab-data.

The video path was also checked with a 12-frame synthetic rosbag made from
translated copies of the bundled sample image: all 11 adjacent frame pairs
were processed. Harris+LK can be run with `descriptor:=LK`. To save its
current track drawing as `lk_tracks.png`, pass `save_outputs:=true` and
`output_dir:=/existing/directory`. LK has no descriptor ratio test, so its
valid tracks are also printed as good matches. The synthetic bag checks the
subscriber and statistics path; use a real flight rosbag to evaluate tracking
performance. The rosbag player waits two seconds after advertising the image
topic so the subscriber receives the first frame.

Measured results and the interpretation of the bundled pair and real bag
experiments are recorded in `EXPERIMENTS.md`. The individual projection and
vanishing-point derivations are in `../INDIVIDUAL_SOLUTIONS.md`. Submission
PDFs are `../team_report.pdf` and `../individual_solutions.pdf`.
