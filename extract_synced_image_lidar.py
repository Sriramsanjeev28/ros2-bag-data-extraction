#!/usr/bin/env python3

import os
import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
import cv2
from cv_bridge import CvBridge
import numpy as np
import re


def pointcloud2_to_bin(pointcloud_msg, filename):
    """Convert PointCloud2 to KITTI-style .bin [x, y, z, intensity]."""
    points = []
    for p in point_cloud2.read_points(
        pointcloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
    ):
        points.append([p[0], p[1], p[2], p[3]])
    points = np.array(points, dtype=np.float32)

    points.tofile(filename)


def image_msg_to_png(image_msg, filename, bridge):
    """Convert ROS Image message to PNG using cv_bridge."""
    try:
        cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        cv2.imwrite(filename, cv_image)
    except Exception as e:
        print(f"[ERROR] Failed to convert and save image: {e}")


def get_next_start_index(camera_dir, lidar_dir):
    """Find the max existing index and return next index."""
    existing_nums = []

    for folder in [camera_dir, lidar_dir]:
        for f in os.listdir(folder):
            match = re.match(r"(\d{4})\.(png|bin)", f)
            if match:
                existing_nums.append(int(match.group(1)))

    if existing_nums:
        return max(existing_nums) + 1
    else:
        return 1


def main():
    rclpy.init()
    bridge = CvBridge()

    # ✅ Bag and output paths
    # Replace with your bag file path
    bag_path = "/path/to/your/rosbag"
    # Replace with your output directory path
    output_dir = "/path/to/your/output"

    # ✅ Create subfolders
    camera_dir = os.path.join(output_dir, "camera")
    lidar_dir = os.path.join(output_dir, "lidar")
    os.makedirs(camera_dir, exist_ok=True)
    os.makedirs(lidar_dir, exist_ok=True)

    # ✅ Figure out where to continue numbering
    start_index = get_next_start_index(camera_dir, lidar_dir)  # NEW
    print(f"[INFO] Continuing extraction from index {start_index:04d}")

    # ✅ Topics
    # Replace with your image topic
    image_topic = "/your_image_topic"
    # Replace with your point cloud topic
    pc_topic = "/your_pointcloud_topic"

    # ✅ Setup reader
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    print("[INFO] Reading messages from rosbag...")

    last_image_msg, last_image_ts = None, None
    last_pc_msg, last_pc_ts = None, None

    save_interval = 10.0  # ⏱ Save every 10 seconds
    last_saved_time = 0.0

    pair_count = 0
    max_pairs = 500

    while reader.has_next() and pair_count < max_pairs:
        topic, data, timestamp = reader.read_next()

        if topic == image_topic:
            last_image_msg = deserialize_message(data, Image)
            last_image_ts = timestamp
        elif topic == pc_topic:
            last_pc_msg = deserialize_message(data, PointCloud2)
            last_pc_ts = timestamp

        # ✅ Save when timestamps are close (<50 ms)
        if last_image_ts and last_pc_ts and abs(last_image_ts - last_pc_ts) < 5e7:
            current_time = timestamp / 1e9
            if current_time - last_saved_time >= save_interval:
                pair_count += 1
                file_index = start_index + pair_count - 1  # continue numbering
                last_saved_time = current_time

                filename_base = f"{file_index:04d}"
                image_filename = os.path.join(camera_dir, f"{filename_base}.png")
                lidar_filename = os.path.join(lidar_dir, f"{filename_base}.bin")

                image_msg_to_png(last_image_msg, image_filename, bridge)
                pointcloud2_to_bin(last_pc_msg, lidar_filename)

                print(f"[SAVED] Pair {file_index}: {filename_base}.png + {filename_base}.bin")

                last_image_msg, last_image_ts = None, None
                last_pc_msg, last_pc_ts = None, None

    print(f"[DONE] Extracted {pair_count} new pairs to: {output_dir}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
