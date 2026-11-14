# LAB6: Visual Relative Pose Estimation

Author: [LittleHorseyaya](https://github.com/LittleHorseyaya). Built on the MIT VNAV 2023 ROS1 starter code. Original attribution and copyright notices are retained for course templates and third-party code.

This directory contains the team implementation for D3-D5: **ROS1 C++ source code, an RViz demonstration video, error plots, raw results from five random seeds, and reproduction scripts**. **The experiment report is not included; it will be prepared separately. Individual deliverables D1/D2 are outside this upload.**

## Navigation

- [ROS pose-estimation node](src/pose_estimation.cpp) · [Offline shared-input evaluator](src/cache_eval.cpp)
- [Arun RViz demonstration video](media/Lab6_Arun_RViz.mp4)
- [2D rotation error](figures/error_curves/2d_rotation_seed12345.png) · [2D translation-direction error](figures/error_curves/2d_translation_seed12345.png)
- [Arun rotation error](figures/error_curves/arun_rotation_seed12345.png) · [Arun metric translation error](figures/error_curves/arun_translation_seed12345.png)
- [Across-seed summary plot](figures/robustness/across_seed_mean_std.png) · [Per-seed results](figures/robustness/per_seed_errors.png) · [Failure counts](figures/robustness/failure_counts.png)
- [Summary table](results/tables/summary.csv) · [Per-run statistics](results/tables/per_run_metrics.csv) · [20 per-frame result files](results/seed_runs)

## Implementation

- Reuses Lab5 SIFT/FLANN matching; calls `calibrateKeypoints` to undistort keypoints and produce unit bearing vectors.
- Implements Nister's five-point method, the Longuet-Higgins eight-point method, translation-only two-point estimation with known rotation, and Arun three-point alignment, all with OpenGV RANSAC.
- Refines two-point translation using SVD over all inliers while keeping the ground-truth relative rotation fixed. Translation sign follows RANSAC, not ground-truth translation.
- Buffers body-frame ground truth, interpolates position linearly and orientation with SLERP at image timestamps, then applies the camera extrinsic transform. Both bracketing samples are required; no extrapolation is performed.
- Checks RGB/depth timestamp differences, depth bounds, positive finite depth, and failed estimates. Exports RPE and matched-input caches.
- Evaluates all four methods on the same cached frame pairs. Isolated child processes enforce a two-second timeout per frame and method; timed-out estimates remain in the results as failures.

## Final shared-input experiment

All methods use the same **3,889 consecutive frame pairs and 1,400,564 SIFT correspondences**, with five prespecified seeds: `12345, 23456, 34567, 45678, 56789`. Five-point and eight-point estimation both use a bearing-residual threshold of `1e-6` followed by inlier refinement. The two-point threshold is `1e-6`; the Arun threshold is `0.05 m`. RANSAC is limited to 1,000 iterations.

| Method | Across-seed average of full-run mean translation error ± sample standard deviation | Failures / attempts |
| --- | ---: | ---: |
| Five-point | 0.495488 ± 0.007968 (direction error) | 4 / 19,445 |
| Eight-point | 0.366430 ± 0.003764 (direction error) | 0 / 19,445 |
| Two-point | 0.063565 ± 0.001146 (direction error) | 0 / 19,445 |
| Arun | 0.004084 ± 0.000015 m | 0 / 19,445 |

**Error means include successful estimates only; failures are reported separately.** All four failures are five-point timeouts. GDB located the excessive runtime for seed 23456 inside OpenGV's `Sturm::bracketRoots`. Seed 12345 reuses a completed non-isolated run whose numerical reproducibility was verified; the other seeds use process isolation. Timing is therefore not compared across these runs. Each frame initializes its sampler with the run's seed. These results characterize the fixed cached inputs and configuration, not a universal algorithm ranking.

Under this configuration, the eight-point method outperformed the five-point method for all five seeds. The two-point direction error was below 0.1 for 85.99%-86.73% of estimates. Its rotation comes from ground truth, so this is not evidence of superior independent visual rotation estimation. Arun's metric error and the 2D direction errors have different units and must not be compared directly.

Plots use the prespecified seed 12345 as a representative run. Raw peaks are retained without smoothing or outlier removal. One ground-truth interpolation bracket spans 35 ms; the others are approximately 5 ms. Both the [bracket record](results/tables/long_gt_brackets.csv) and the [sensitivity statistics with and without that pair](results/tables/gt_bracket_sensitivity.csv) are retained.

## Environment and build

Tested environment: Ubuntu 20.04.6, ROS Noetic, catkin_tools, and Eigen 3.3.7. The existing Lab5 workspace supplies `lab_5`, `catkin_simple`, `glog_catkin`, and `opencv3_catkin`. Additional dependencies include `opengv`, `eigen_catkin`, `cmake_modules`, `tf_conversions`, and the packages declared in the build configuration.

Create a separate Lab6 workspace. Place this repository's `lab6/` directory under `~/lab6_ws/src` as a single `lab_6` package, and prepare OpenGV and eigen_catkin source packages in the same workspace. Do not add duplicate packages with the same ROS package name. Then run:

```bash
source ~/lab5_ws/devel/setup.bash
cd ~/lab6_ws
catkin config --extend ~/lab5_ws/devel --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_SYSTEM_EIGEN=ON
catkin build lab_6 -j2 -p1
source ~/lab6_ws/devel/setup.bash
```

The Lab6 starter code comes from the official `VNAV2023` tag, commit `7d234d8b3618f0090b61fc6551f3e582db116f9c`, rather than the ROS2 `main` branch. `install/lab_6.rosinstall` retains the upstream repository URL, but its `master` reference is unpinned. Dependency source trees are not bundled here and must be prepared separately on another machine.

## Dataset

Use the [instructor's updated ROS2 office dataset](https://drive.google.com/file/d/1TohOnfjR_FKf_y9wYTEkmv-XCuFPFRPt/view). Install `rosbags==0.11.9` in a separate Python >=3.10 environment, then convert it:

```bash
rosbags-convert --src ./vnav-lab6-office --dst office_ros1.bag
```

The recording lasts 236.995 s, starting at timestamp 24 s. It contains 347,574 messages and 3,890 frames each for left RGB and depth. Images are 720x480 with `fx=fy=415.69219381653056, cx=360, cy=240` and zero distortion. Depth uses `32FC1` and is interpreted as camera-axis Z depth.

The approximately 19 GB source bag, full correspondence cache, build products, and experiment report are not committed. `results/shared_frame_index.csv` preserves pair timestamps, correspondence counts, relative ground truth, and interpolation-bracket durations. Do not substitute a Lab5 bag for the Lab6 dataset.

## ROS node and RViz

The following commands use the original experiment machine's `/mnt/lab6_data/office_ros1.bag`; adjust paths on another machine. Start `roscore` in terminal 1. In terminal 2:

```bash
source ~/lab6_ws/devel/setup.bash
rosparam set /use_sim_time true
rosrun lab_6 pose_estimation --pose_estimator=2 --use_ransac=true --fixed_seed=true --twopt_threshold=1e-6 --scale_translation=true --rpe_csv=/mnt/lab6_data/results/NEW_twopt.csv /ground_truth_pose:=/tesse/odom /rgb_images_topic:=/tesse/left_cam/rgb/image_raw /depth_images_topic:=/tesse/depth_cam/mono/image_raw _visualize:=false _save_outputs:=false
```

In terminal 3:

```bash
source ~/lab6_ws/devel/setup.bash
rosbag play /mnt/lab6_data/office_ros1.bag --clock -r 0.1 --delay=2
```

`pose_estimator=0/1/2/3` selects five-point, eight-point, two-point, or Arun, respectively. Use `--scale_translation=false` for Arun. **An existing RPE CSV with the same name will be overwritten; always select a new output filename.** Omit `--rpe_csv` for visualization only. Launch RViz in another terminal:

```bash
rviz -d "$(rospack find lab_6)/rviz/office.rviz"
```

The configuration displays RGB, depth, `/camera_pose`, and `/gt_camera_pose` in the `world` frame. The video records an earlier Arun ROS demonstration. Visualization anchors each estimate to the previous ground-truth pose; 2D visualization also recovers scale from ground truth. This is the local visualization requested by the handout, not independently accumulated visual odometry.

## Caching and offline controlled comparison

Capture common inputs with a **new output prefix**, without running pose solvers:

```bash
rosrun lab_6 pose_estimation --fixed_seed=true --cache_only=true --match_cache=/mnt/lab6_data/results/NEW_cache /ground_truth_pose:=/tesse/odom /rgb_images_topic:=/tesse/left_cam/rgb/image_raw /depth_images_topic:=/tesse/depth_cam/mono/image_raw _visualize:=false _save_outputs:=false
```

Play the bag in another terminal. This creates `NEW_cache_frames.csv` and `NEW_cache_matches.csv`. Caching refuses to overwrite existing files; the destination directory must already exist. The first image initializes the previous frame. The complete experiment cache was checked for all 3,889 consecutive pairs and their correspondence counts.

After stopping the ROS nodes, run the C++ evaluator directly; no ROS master is required:

```bash
time rosrun lab_6 cache_eval /mnt/lab6_data/results/NEW_cache /mnt/lab6_data/results/NEW_eval 1e-6 12345 2
```

Arguments are the cache prefix, new output prefix, shared 2D threshold, random seed, and timeout in seconds per method. Outputs end in `_nister.csv`, `_eightpt.csv`, `_twopt.csv`, and `_arun.csv`. Failed rows retain a failure `status` and NaN errors. Filter on `status=ok` when computing successful-error statistics and report failure rates separately; never treat failures as zero error.

Five-point and eight-point methods both refine on inliers; two-point estimation uses fixed-rotation SVD; Arun refits on valid 3D inliers. **This offline comparison differs from the eight-point branch in the ROS node**, which retains its earlier `1e-5` threshold without inlier refinement. Reproduce the final multi-seed tables using `cache_eval`. The five-point solver was not replaced with another algorithm to obtain a preferred ranking.

One complete four-method evaluation without process isolation took 45.642 s. The isolated evaluator adds process overhead; the problematic-seed test took 79.771 s. These timings exclude SIFT and full-system input processing and do not demonstrate real-time performance of the complete visual pipeline.

`scripts/experiment/` preserves the actual batch scripts, including original machine paths and reused runs, for provenance. They are not one-command installers for a new machine. A straightforward independent reproduction is to choose five seeds in advance and run the evaluator for each, using a new output prefix every time.

## Regenerate plots

With matplotlib installed, run from the repository root:

```bash
python3 lab6/scripts/plot_results.py
```

This regenerates seven PNG/SVG figures and the English summary table from committed CSVs; neither ROS nor the bag is required. Rotation error is `||Rgt-Rest||F`. For 2D methods, translation error is the Euclidean difference between unit translation directions; for Arun, it is metric Euclidean error. Successful-error statistics and failure rates are kept separate.

## Limitations and sources

- The environment mixes OpenCV 3.4 with the system cv_bridge dependency on OpenCV 4.2. Runs succeeded with feature pop-ups disabled, but the ABI compatibility risk has not been eliminated.
- Optional branches without RANSAC are not implemented; keep `use_ransac=true`.
- Process timeout protection is implemented in `cache_eval`, not the ROS online node. Certain random five-point samples can trigger excessive runtime in OpenGV root finding.
- [`results/provenance.json`](results/provenance.json) records input provenance, seed protocols, and qualifications. All run logs and seed results are retained.
- [MIT Lab6 handout](https://vnav.mit.edu/labs_2023/lab6/exercises.html) · [Official VNAV2023 starter code](https://github.com/MIT-SPARK/VNAV-labs/tree/VNAV2023/lab6) · [Original README](docs/UPSTREAM_README.md).

This directory archives experiment code and evidence; it does not confirm receipt by the course submission system. The formal experiment report will be written and uploaded separately according to the requested format.
