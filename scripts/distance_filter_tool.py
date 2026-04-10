#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Features:
1) Automatically detect LiDAR point cloud types in rosbag:
   - sensor_msgs/PointCloud2  (e.g. /hesai/pandar)
   - livox_ros_driver/CustomMsg (e.g. /livox/lidar)
2) Parse and export the point cloud to a single PCD file with intensity (x y z intensity, ASCII)
3) Use Open3D for interactive point picking (at least 4 points), compute the bounding range
   from those 4 points, and save as a .txt file with the same base name.

Dependencies:
    - rosbag
    - sensor_msgs.point_cloud2
    - open3d, numpy

Usage examples:
    python FAST-Calib-tool.py
    python FAST-Calib-tool.py /path/to/data.bag /path/to/output_dir
"""

import os
import sys
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d

# ===================== General: Save PCD =====================

def save_pcd_with_intensity(points, intensities, output_path):
    """
    Save point cloud as a PCD file with intensity field (ASCII format).
    points: list/ndarray of [x, y, z]
    intensities: list/ndarray of intensity
    """
    N = len(points)
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {N}
HEIGHT 1
POINTS {N}
DATA ascii
"""
    with open(output_path, 'w') as f:
        f.write(header)
        for (x, y, z), inten in zip(points, intensities):
            f.write(f"{x} {y} {z} {inten}\n")
    print(f"[PCD] Saved point cloud with intensity field to: {output_path}")

# ===================== Case 1: PointCloud2 =====================

def find_intensity_field(msg):
    """Automatically detect the intensity field name in PointCloud2 fields."""
    candidates = ["intensity", "reflectivity", "i", "ref"]
    for field in msg.fields:
        if field.name.lower() in candidates:
            return field.name
    return None


def convert_pointcloud2_bag_to_pcd(
    bag_file,
    output_dir,
    topic_name="/hesai/pandar",                        # Change to your topic name if different
    pcd_name="sensor_PointCloud2_inten_ascii.pcd"
):
    """
    Merge and export PointCloud2-type point clouds from a rosbag into a single PCD file.
    Keeps original LiDAR coordinates without any coordinate transformation.
    """
    print(f"[Bag] Opening rosbag: {bag_file}")
    bag = rosbag.Bag(bag_file, "r")

    # 1) Detect the intensity field first
    intensity_field = None
    for topic, msg, t in bag.read_messages():
        if msg._type == "sensor_msgs/PointCloud2":
            intensity_field = find_intensity_field(msg)
            if intensity_field:
                print(f"[Bag] Detected intensity field: {intensity_field}")
            break

    if not intensity_field:
        print("[ERROR] Intensity field not found! Aborting PointCloud2 conversion.", file=sys.stderr)
        bag.close()
        return None

    # 2) Read all point clouds from the specified topic
    all_points = []
    all_intensities = []

    print(f"[Bag] Reading PointCloud2 point clouds from topic '{topic_name}'...")

    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        if msg._type == "sensor_msgs/PointCloud2":
            try:
                field_names = ["x", "y", "z", intensity_field]
                for point in pc2.read_points(msg, field_names=field_names, skip_nans=True):
                    all_points.append([point[0], point[1], point[2]])
                    all_intensities.append(point[3])  # Intensity is the fourth field
            except Exception as e:
                print(f"[ERROR] Read error: {str(e)}", file=sys.stderr)
                continue

    bag.close()

    if not all_points:
        print("[ERROR] No PointCloud2 point cloud data found!", file=sys.stderr)
        return None

    output_path = os.path.join(output_dir, pcd_name)
    save_pcd_with_intensity(all_points, all_intensities, output_path)
    return output_path

# ===================== Case 2: Livox CustomMsg =====================

def parse_livox_custom_msg(msg):
    """
    Parse x, y, z, reflectivity from a livox_ros_driver/CustomMsg.
    Assumes msg.points is a list of CustomPoint objects with fields: x, y, z, reflectivity.
    """
    points = []
    intensities = []

    for pt in msg.points:
        points.append([pt.x, pt.y, pt.z])
        intensities.append(pt.reflectivity)

    return points, intensities

def convert_livox_custom_bag_to_pcd(
    bag_file,
    output_dir,
    topic_name="/livox/lidar",                     # Change to your topic name if different
    pcd_name="livox_CustomMsg_inten_ascii.pcd"
):
    """
    Merge and export livox_ros_driver/CustomMsg-type point clouds from a rosbag into a single PCD file.
    Keeps original LiDAR coordinates without any coordinate transformation.
    """
    print(f"[Bag] Opening rosbag: {bag_file}")
    bag = rosbag.Bag(bag_file, "r")

    all_points = []
    all_intensities = []

    print(f"[Bag] Reading CustomMsg point clouds from topic '{topic_name}'...")

    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        if msg._type in ("livox_ros_driver/CustomMsg", "livox_ros_driver2/CustomMsg"): 
            pts, intens = parse_livox_custom_msg(msg)
            all_points.extend(pts)
            all_intensities.extend(intens)

    bag.close()

    if not all_points:
        print("[ERROR] No Livox CustomMsg point cloud data found!", file=sys.stderr)
        return None

    output_path = os.path.join(output_dir, pcd_name)
    intensities = np.array(all_intensities, dtype=np.float32)
    save_pcd_with_intensity(all_points, intensities, output_path)
    return output_path

# ===================== Auto-detect: which method to use for this bag =====================

def detect_lidar_msg_type(bag_file):
    """
    Scan the bag to detect whether it contains PointCloud2 or Livox CustomMsg.
    Returns:
        "PointCloud2", "CustomMsg", or None
    If both types are found, defaults to PointCloud2 with a printed notice.
    """
    has_pc2 = False
    has_livox = False

    print(f"[Detect] Scanning bag: {bag_file}")
    bag = rosbag.Bag(bag_file, "r")

    for topic, msg, t in bag.read_messages():
        if msg._type == "sensor_msgs/PointCloud2":
            has_pc2 = True
        elif msg._type in ("livox_ros_driver/CustomMsg", "livox_ros_driver2/CustomMsg"):  
            has_livox = True

        if has_pc2 and has_livox:
            break

    bag.close()

    if has_pc2 and has_livox:
        print("[Detect] Both PointCloud2 and Livox CustomMsg detected; defaulting to PointCloud2.")
        return "PointCloud2"
    elif has_pc2:
        print("[Detect] PointCloud2 point cloud detected.")
        return "PointCloud2"
    elif has_livox:
        print("[Detect] Livox CustomMsg point cloud detected.")
        return "CustomMsg"
    else:
        print("[Detect] No PointCloud2 or Livox CustomMsg point cloud detected.")
        return None

# ===================== Open3D interactive point picking & save range =====================

def select_and_save_points(pcd_folder, target_pcd_name):
    """
    Read the specified PCD file from the given directory, use Open3D interactive
    point picking, and save the bounding range.
    """
    pcd_path = os.path.join(pcd_folder, target_pcd_name)
    if not os.path.isfile(pcd_path):
        print(f"[ERROR] Specified PCD file does not exist: {pcd_path}", file=sys.stderr)
        return

    # Read point cloud
    pcd = o3d.io.read_point_cloud(pcd_path)
    if not pcd.has_points():
        print(f"[ERROR] {target_pcd_name} contains no point cloud data, skipping.", file=sys.stderr)
        return

    print(f"\nProcessing: {target_pcd_name}")
    print("Hold Shift and left-click to select points (at least 4) in the visualization window, then press Q to close.")

    # Create visualization window and add point cloud
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=f"Select Points - {target_pcd_name}")
    vis.add_geometry(pcd)

    # Wait for user interaction (Shift+left-click to pick, Q to exit)
    vis.run()
    vis.destroy_window()

    # Get indices of user-selected points
    selected_indices = vis.get_picked_points()

    if not selected_indices:
        print(f"[ERROR] No points selected; no file saved for {target_pcd_name}.", file=sys.stderr)
        return

    if len(selected_indices) < 4:
        print(f"[ERROR] Only {len(selected_indices)} point(s) selected, fewer than 4; skipping this file.", file=sys.stderr)
        return

    # Use only the first 4 points
    selected_indices = selected_indices[:4]

    all_points = np.asarray(pcd.points)
    selected_points = all_points[selected_indices, :]  # Shape (4, 3)

    # Compute min and max along each axis for the four points
    mins = selected_points.min(axis=0)  # [x_min_raw, y_min_raw, z_min_raw]
    maxs = selected_points.max(axis=0)  # [x_max_raw, y_max_raw, z_max_raw]

    # Expand by 0.2 m on each side
    x_min = mins[0] - 0.2
    x_max = maxs[0] + 0.2
    y_min = mins[1] - 0.2
    y_max = maxs[1] + 0.2
    z_min = mins[2] - 0.2
    z_max = maxs[2] + 0.2

    # Generate output filename (same base name as PCD, with .txt extension)
    base_name = os.path.splitext(target_pcd_name)[0]
    save_file = os.path.join(pcd_folder, f"{base_name}.txt")

    with open(save_file, 'w') as f:
        f.write("# 4 selected points (x y z)\n")
        for p in selected_points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

        f.write("# range values in order:\n")
        f.write(f"x_min: {x_min:.1f}\n")
        f.write(f"x_max: {x_max:.1f}\n")
        f.write(f"y_min: {y_min:.1f}\n")
        f.write(f"y_max: {y_max:.1f}\n")
        f.write(f"z_min: {z_min:.1f}\n")
        f.write(f"z_max: {z_max:.1f}\n")

    print(f"[Save] Selected points and range saved to: {save_file}")
    print("Point cloud processing complete.")

# ===================== main =====================

if __name__ == "__main__":
    # 1) Parse command-line arguments: bag path & output directory
    if len(sys.argv) > 1:
        bag_file = sys.argv[1]
    else:
        # Default to a bag in the current directory; modify as needed
        bag_file = os.path.join(os.getcwd(), "./src/FAST-Calib/calib_data/DataBag_2026-04-09-10-20-34/data.bag")
        print(f"No bag file specified, using default: {bag_file}")

    if len(sys.argv) > 2:
        output_dir = sys.argv[2]
    else:
        output_dir = os.getcwd()
        print(f"No output directory specified, using current directory: {output_dir}")

    if not os.path.isfile(bag_file):
        print(f"[ERROR] Bag file '{bag_file}' does not exist.", file=sys.stderr)
        sys.exit(1)

    if not os.path.isdir(output_dir):
        print(f"[ERROR] Output directory '{output_dir}' does not exist.", file=sys.stderr)
        sys.exit(1)

    # No need for rospy.init_node; this is a fully offline tool

    # 3) Auto-detect point cloud type in the bag
    msg_type = detect_lidar_msg_type(bag_file)
    if msg_type is None:
        print("[ERROR] No supported LiDAR message type detected; exiting.", file=sys.stderr)
        sys.exit(1)

    # 4) Perform the corresponding PCD conversion based on detected type
    if msg_type == "PointCloud2":
        pcd_path = convert_pointcloud2_bag_to_pcd(
            bag_file=bag_file,
            output_dir=output_dir,
            topic_name="/hesai/pandar",  # Change to your topic name if different
            pcd_name="sensor_PointCloud2_inten_ascii.pcd"
        )
    else:  # "CustomMsg"
        pcd_path = convert_livox_custom_bag_to_pcd(
            bag_file=bag_file,
            output_dir=output_dir,
            topic_name="/livox/lidar",  # Change to your topic name if different
            pcd_name="livox_CustomMsg_inten_ascii.pcd"
        )

    if pcd_path is None:
        print("[ERROR] PCD generation failed; exiting.", file=sys.stderr)
        sys.exit(1)

    # 5) Run interactive point picking + range saving on the generated PCD
    select_and_save_points(
        pcd_folder=output_dir,
        target_pcd_name=os.path.basename(pcd_path)
    )
