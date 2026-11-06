#!/usr/bin/env python3
"""Copy selected image messages into a small ROS 1 bag for visual checks."""

import argparse

import rosbag


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source")
    parser.add_argument("destination")
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--start", type=int, default=300)
    parser.add_argument("--stride", type=int, default=5)
    args = parser.parse_args()
    if args.start < 0 or args.stride < 1:
        parser.error("start must be nonnegative and stride must be positive")

    indices = {args.start, args.start + args.stride}
    copied = 0
    with rosbag.Bag(args.source) as source, rosbag.Bag(args.destination, "w") as target:
        for index, (topic, message, stamp) in enumerate(
            source.read_messages(topics=[args.topic])
        ):
            if index in indices:
                target.write(topic, message, stamp)
                copied += 1
            if index >= max(indices):
                break

    if copied != 2:
        raise RuntimeError(f"expected two frames, found {copied}")
    print(f"Copied frames {args.start} and {args.start + args.stride}")


if __name__ == "__main__":
    main()
