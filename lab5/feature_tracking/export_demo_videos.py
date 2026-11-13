#!/usr/bin/env python3
"""Export synchronized LAB5 demo videos from one ROS image topic.

Run after sourcing ROS Noetic's setup.bash with Ubuntu's /usr/bin/python3.
The Farneback parameters and drawing conventions match self_flow.cpp, while
the Harris corner and pyramidal LK settings match lk_feature_tracker.cpp.
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge


VIDEO_NAMES = {
    "camera": "camera_original.mp4",
    "heatmap": "dense_flow_heatmap.mp4",
    "arrows": "dense_flow_arrows.mp4",
    "lk": "harris_lk_tracks.mp4",
    # Ubuntu 20.04's stock Videos player can decode these Motion JPEG files
    # without the extra MPEG-4 or H.264 GStreamer decoder packages.
    "heatmap_avi": "dense_flow_heatmap.avi",
    "lk_avi": "harris_lk_tracks.avi",
}


def harris_corners(gray):
    return cv2.goodFeaturesToTrack(
        gray, maxCorners=500, qualityLevel=0.01, minDistance=7.0,
        blockSize=3, useHarrisDetector=True, k=0.04,
    )


def flow_images(previous_gray, gray, frame, max_motion):
    flow = cv2.calcOpticalFlowFarneback(
        previous_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0,
    )
    magnitude, direction = cv2.cartToPolar(
        flow[:, :, 0], flow[:, :, 1], angleInDegrees=True,
    )
    hsv = np.empty((*gray.shape, 3), dtype=np.uint8)
    hsv[:, :, 0] = (direction * 0.5).astype(np.uint8)
    hsv[:, :, 1] = 255
    hsv[:, :, 2] = np.clip(magnitude * (255.0 / max_motion), 0, 255).astype(np.uint8)
    heatmap = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    arrows = frame.copy()
    for y in range(12, gray.shape[0], 20):
        for x in range(12, gray.shape[1], 20):
            dx, dy = flow[y, x]
            if np.hypot(dx, dy) < 0.2:
                continue
            end = (int(x + np.rint(4.0 * dx)), int(y + np.rint(4.0 * dy)))
            cv2.arrowedLine(arrows, (x, y), end, (0, 255, 0), 1,
                            cv2.LINE_AA, 0, 0.25)
    return heatmap, arrows


def lk_image(previous_gray, gray, previous_corners, frame):
    image = frame.copy()
    if previous_corners is None:
        return image
    tracked, status, _ = cv2.calcOpticalFlowPyrLK(
        previous_gray, gray, previous_corners, None,
    )
    if tracked is None or status is None:
        return image
    for start, end, valid in zip(
        previous_corners.reshape(-1, 2), tracked.reshape(-1, 2), status.reshape(-1)
    ):
        x, y = end
        if not valid or not (0 <= x < gray.shape[1] and 0 <= y < gray.shape[0]):
            continue
        p0 = tuple(np.rint(start).astype(int))
        p1 = tuple(np.rint(end).astype(int))
        cv2.line(image, p0, p1, (0, 255, 0), 1)
        cv2.circle(image, p1, 2, (0, 0, 255), -1)
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS bag with a color image topic")
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--max-pairs", type=int, default=0,
                        help="Stop after this many pairs; 0 exports the full bag")
    parser.add_argument("--max-motion", type=float, default=3.0,
                        help="Flow magnitude that reaches maximum heatmap brightness")
    args = parser.parse_args()
    if args.fps <= 0 or args.max_motion <= 0 or args.max_pairs < 0:
        parser.error("--fps and --max-motion must be positive; --max-pairs cannot be negative")
    if not args.bag.is_file():
        parser.error("bag file does not exist: {}".format(args.bag))
    args.output_dir.mkdir(parents=True, exist_ok=True)

    bridge = CvBridge()
    writers = {}
    previous_gray = None
    previous_corners = None
    pairs = 0
    try:
        with rosbag.Bag(str(args.bag), "r") as bag:
            if bag.get_message_count(topic_filters=[args.topic]) < 2:
                parser.error("topic has fewer than two images: {}".format(args.topic))
            for _, message, _ in bag.read_messages(topics=[args.topic]):
                frame = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners = harris_corners(gray)
                if previous_gray is None:
                    previous_gray, previous_corners = gray, corners
                    continue
                if gray.shape != previous_gray.shape:
                    raise RuntimeError("image dimensions changed within the bag")

                if not writers:
                    size = (frame.shape[1], frame.shape[0])
                    for kind, name in VIDEO_NAMES.items():
                        writer = cv2.VideoWriter(
                            str(args.output_dir / name),
                            cv2.VideoWriter_fourcc(
                                *("MJPG" if name.endswith(".avi") else "mp4v")
                            ), args.fps, size,
                        )
                        if not writer.isOpened():
                            raise RuntimeError("could not open video writer: " + name)
                        writers[kind] = writer

                heatmap, arrows = flow_images(
                    previous_gray, gray, frame, args.max_motion,
                )
                lk_tracks = lk_image(previous_gray, gray, previous_corners, frame)
                images = {
                    "camera": frame, "heatmap": heatmap,
                    "arrows": arrows, "lk": lk_tracks,
                }
                for kind, writer in writers.items():
                    image_kind = kind[:-4] if kind.endswith("_avi") else kind
                    writer.write(images[image_kind])
                pairs += 1
                if pairs % 200 == 0:
                    print("Exported {} adjacent pairs".format(pairs), flush=True)
                previous_gray, previous_corners = gray, corners
                if args.max_pairs and pairs >= args.max_pairs:
                    break
    finally:
        for writer in writers.values():
            writer.release()

    if pairs == 0:
        raise RuntimeError("no adjacent image pairs were exported")
    print("Exported {} synchronized frames at {} fps:".format(pairs, args.fps))
    for name in VIDEO_NAMES.values():
        path = args.output_dir / name
        print("  {} ({} bytes)".format(path, path.stat().st_size))


if __name__ == "__main__":
    main()
