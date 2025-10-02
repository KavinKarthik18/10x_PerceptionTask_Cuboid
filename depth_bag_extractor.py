#!/usr/bin/env python3
"""
extract_depth_from_bag.py
Extracts depth frames from a ROS2 rosbag2 file (.db3) and saves them as .npy arrays.

Usage (explicit):
    python3 extract_depth_from_bag.py depth.db3 [frames_folder] [depth_topic]

Usage (default):
    python3 extract_depth_from_bag.py
    -> assumes: bag file = ./depth.db3
                output   = ./frames
                topic    = /depth
"""

import sys
import os
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def save_depth_from_bag(bag_path, frames_dir, depth_topic_hint=None):
    if not os.path.exists(bag_path):
        print(f"Error: Bag file {bag_path} not found.")
        sys.exit(1)

    os.makedirs(frames_dir, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    bridge = CvBridge()
    idx = 0

    print("Reading bag:", bag_path)
    while reader.has_next():
        topic, data, t = reader.read_next()

        # Only process depth topic
        if depth_topic_hint and topic != depth_topic_hint:
            continue

        try:
            msg = deserialize_message(data, Image)
        except Exception as e:
            print("Failed to deserialize message:", e)
            continue

        try:
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            print("cv_bridge conversion failed:", e)
            continue

        # Convert depth to meters if in uint16
        if cv_img.dtype == np.uint16:
            depth_m = cv_img.astype(np.float32) / 1000.0
        else:
            depth_m = cv_img.astype(np.float32)

        fname = f"frame_{idx:06d}.npy"
        np.save(os.path.join(frames_dir, fname), depth_m)

        if idx % 10 == 0:
            print(f"Saved {fname} shape={depth_m.shape}")
        idx += 1

    print("Done. Extracted", idx, "frames to", frames_dir)


if __name__ == "__main__":
    # Default values
    bag_path = "depth/depth.db3"
    frames_dir = "./frames"
    depth_topic_hint = "/depth"

    if len(sys.argv) >= 2:
        bag_path = sys.argv[1]
    if len(sys.argv) >= 3:
        frames_dir = sys.argv[2]
    if len(sys.argv) >= 4:
        depth_topic_hint = sys.argv[3]

    save_depth_from_bag(bag_path, frames_dir, depth_topic_hint)
