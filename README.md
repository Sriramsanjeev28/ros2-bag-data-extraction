# ROS2 Bag Data Extraction

Scripts for extracting camera images and LiDAR point clouds from ROS 2 bag files. This repository includes two methods to extract and save multi-sensor data into standard formats (`.png` for images, `.pcd` or `.bin` for LiDAR), ideal for dataset creation, perception experiments, and robotics research.

---

## My Blogs

- [A Complete Guide to ROS 2 Bag: Record, Inspect, and Replay Multi-Sensor Data](https://medium.com/@sriram.sanjeev.d/a-complete-guide-to-ros-2-bag-record-inspect-and-replay-multi-sensor-data-d0e1e5b9c9e3)  
- [Extracting Camera Images and LiDAR Point Clouds from ROS 2 Bag Files](https://medium.com/@sriram.sanjeev.d/extracting-camera-images-and-lidar-point-clouds-from-ros-2-bag-files-7425fc0f012a)

---

## Scripts

### **Method 1: Time-Synchronized Extraction**
- **Script:** `extract_frames_ros2.py`  
- Subscribes to camera and LiDAR topics using ROS 2 node.  
- Saves **time-synchronized** camera–LiDAR pairs with filenames based on ROS message timestamps.  
- Recommended for **calibration and sensor fusion experiments**.


### **Method 2: Direct Bag Extraction**
- **Script:** `extract_synced_image_lidar.py`  
- Reads messages directly from a ROS 2 bag file.  
- Saves sequentially indexed camera images (`.png`) and LiDAR data (`.bin`).  
- Ideal for **dataset creation and offline processing**.

---

## Prerequisites

- ROS 2 (Humble/Foxy/Galactic, etc.)  
- Python 3  
- OpenCV (`cv2`)  
- Open3D (for `.pcd` handling)  
- CvBridge (`ros-$ROS_DISTRO-cv-bridge`)  
- sensor_msgs_py (`ros-$ROS_DISTRO-sensor-msgs-py`)

Install Python dependencies:
pip install opencv-python open3d
