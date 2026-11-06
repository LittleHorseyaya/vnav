# LAB5 feature-tracking experiments

Environment: Ubuntu 20.04, ROS Noetic, the VNAV2021 starter code, and its
OpenCV 3.4.2 build. All methods use the same RANSAC fundamental-matrix test
with a 3-pixel threshold and 0.99 confidence. SIFT and SURF use FLANN and a
0.8 two-neighbor distance ratio. ORB uses Hamming-distance brute force with
the same ratio. Harris+LK uses valid optical-flow tracks instead of descriptor
matching; its "good matches" count is therefore the valid track count.
The video inlier ratio is averaged over frame pairs, not recomputed from
the averages of the inlier and good-match counts.

## Bundled image pair

Inputs: `images/box.png` and `images/box_in_scene.png`.

| Method | Descriptor | Keypoints 1 | Keypoints 2 | Raw matches | Good matches | RANSAC inliers | Inlier ratio |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| SIFT | 128 floats | 603 | 969 | 603 | 99 | 79 | 79.8% |
| SURF | 64 floats | 997 | 1822 | 997 | 129 | 85 | 65.9% |
| ORB | 32 bytes | 500 | 500 | 500 | 13 | 7 | 53.8% |

The raw-match count is the number of first nearest neighbors. Good matches
pass the 0.8 ratio filter. The inlier ratio is RANSAC inliers divided by good
matches. The PNGs in `results/`, `results/surf/`, and `results/orb/` show the
detected keypoints, raw matches, ratio-filtered matches, and geometric inliers.
On this sample pair, SURF found the most inliers; SIFT had the higher inlier
fraction. ORB found far fewer unambiguous matches at the current 500-feature,
single-pyramid-level setting. Those observations alone do not establish their
performance on flight video.

## Real rosbag: 30fps_424x240_2018-10-01-18-35-06.bag

The bag has 1584 color-image messages on `/camera/color/image_raw`, so a full
run has 1583 adjacent-frame pairs. At normal playback speed the VM processed
only 1393 pairs with SIFT. Playback at 0.6 speed processed all 1583 pairs.

| Method | Avg. keypoints 1 | Avg. keypoints 2 | Avg. raw/valid matches | Avg. good matches | Avg. inliers | Avg. inlier ratio | Pairs |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| SIFT | 403.547 | 403.606 | 403.547 | 258.428 | 248.952 | 95.699% | 1583 |
| SURF | 466.916 | 466.964 | 466.916 | 334.204 | 319.892 | 95.306% | 1583 |
| ORB | 315.681 | 315.730 | 315.681 | 230.166 | 225.968 | 97.868% | 1583 |
| Harris+LK | 102.201 | 102.209 | 100.877 | 100.877 | 100.793 | 99.919% | 1583 |

All four methods processed the same 1583 adjacent pairs at 0.6 playback
speed. SURF produced the most inliers on this relatively smooth sequence.
Harris+LK had the highest inlier fraction, but it started with fewer corners;
its match count is the number of tracks retained by optical flow, not a
descriptor nearest-neighbor count.
`results/lk_real/lk_tracks.png` shows Harris+LK tracks on color frames 300 and
305 extracted from this real bag; 75 of 78 previous-frame corners were
tracked. The five-frame gap makes the short track segments easier to see.

## Real rosbag: vnav-lab5-smooth-trajectory.bag

This bag has 1386 left-camera RGB messages on
`/tesse/left_cam/rgb/image_raw`, giving 1385 adjacent pairs. SIFT processed
only 1063 pairs at 0.6 playback speed; slowing to 0.35 covered all 1385.
Only complete runs are included in the table.

| Method | Avg. keypoints 1 | Avg. keypoints 2 | Avg. raw/valid matches | Avg. good matches | Avg. inliers | Avg. inlier ratio | Pairs |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| SIFT | 147.757 | 147.913 | 147.757 | 92.950 | 80.661 | 86.142% | 1385 |
| SURF | 1191.760 | 1192.910 | 1191.760 | 714.063 | 609.622 | 85.477% | 1385 |
| ORB | 278.775 | 279.090 | 278.775 | 166.765 | 151.237 | 90.792% | 1385 |
| Harris+LK | 70.618 | 70.677 | 68.907 | 68.907 | 64.206 | 93.921% | 1385 |

SURF produced the most average inliers on both real sequences (319.892 and
609.622). ORB performed poorly on the widely separated bundled pair, but it
exceeded SIFT's average inlier count on this smooth-trajectory sequence
(151.237 versus 80.661). Harris+LK had the highest average inlier fraction
on both sequences but tracked fewer corners. These tables do not measure
execution time, so they do not support a claim that one method is faster.
LK assumes approximately constant local brightness and small motion between
frames; large motion, occlusion, or appearance changes can break its tracks.

## Functional video check

A 12-frame synthetic bag was made from translated copies of the bundled
sample image. SIFT and Harris+LK each processed all 11 adjacent-frame pairs.
The LK track drawing is saved as `results/lk/lk_tracks.png`. The 100% LK
inlier ratio on that synthetic motion is only a functional check and should
not be used as a real-data comparison.
