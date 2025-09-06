#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
import open3d as o3d
import cv2
import datetime
import time
from message_filters import Subscriber, ApproximateTimeSynchronizer

SAVE_LIMIT = 10  # Number of unique synchronized frames to save
OUTPUT_DIR = os.path.expanduser("~/output_data")  # Change this path as needed
os.makedirs(OUTPUT_DIR, exist_ok=True)

class Extractor(Node):
    def __init__(self):
        super().__init__('data_extractor')
        self.bridge = CvBridge()
        self.counter = 0
        self.last_save_time = 0  # in seconds

        # Replace with your image topic
        self.image_sub = Subscriber(self, Image, '/your_image_topic')
        # Replace with your point cloud topic
        self.pc_sub = Subscriber(self, PointCloud2, '/your_pointcloud_topic')

        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.pc_sub],
            queue_size=100,
            slop=0.5  # tighter sync
        )
        self.ts.registerCallback(self.synced_callback)

    def synced_callback(self, image_msg, pc_msg):
        if self.counter >= SAVE_LIMIT:
            return

        current_time = time.time()
        # Only save if at least 1 second passed since last save
        if current_time - self.last_save_time < 1.0:
            return

        try:
            # Generate filename using ROS msg timestamp to avoid overwriting
            stamp = image_msg.header.stamp
            filename = f"{stamp.sec}_{stamp.nanosec}"

            # Convert and save image
            image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            cv2.imwrite(f"{OUTPUT_DIR}/{filename}.png", image)

            # Convert and save pointcloud
            points = []
            for p in point_cloud2.read_points(pc_msg, skip_nans=True):
                points.append([p[0], p[1], p[2]])
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(points)
            o3d.io.write_point_cloud(f"{OUTPUT_DIR}/{filename}.pcd", cloud)

            self.counter += 1
            self.last_save_time = current_time
            self.get_logger().info(f"✅ Saved synchronized pair #{self.counter} → {filename}")
        except Exception as e:
            self.get_logger().error(f"❌ Error during save: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Extractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
