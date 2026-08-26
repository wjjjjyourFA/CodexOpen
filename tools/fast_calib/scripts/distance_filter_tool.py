#!/usr/bin/env python3
"""FAST-Calib offline distance-filter helper.

The workflow is preserved from the staged FAST-Calib utility:
1. merge a supported LiDAR topic from a rosbag into an ASCII PCD;
2. select at least four points with Open3D;
3. save the first four points and their axis-aligned bounds expanded by 0.2 m.
"""

import argparse
import os
import sys

import numpy as np


SUPPORTED_CUSTOM_TYPES = {
    "livox_ros_driver/CustomMsg",
    "livox_ros_driver2/CustomMsg",
}


def save_pcd_with_intensity(points, intensities, output_path):
    """Save x/y/z/intensity as an ASCII PCD file."""
    point_count = len(points)
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {point_count}
HEIGHT 1
POINTS {point_count}
DATA ascii
"""
    with open(output_path, "w", encoding="utf-8") as output:
        output.write(header)
        for (x, y, z), intensity in zip(points, intensities):
            output.write(f"{x} {y} {z} {intensity}\n")
    print(f"[PCD] 保存带 intensity 字段的点云到: {output_path}")


def find_intensity_field(message):
    candidates = {"intensity", "reflectivity", "i", "ref"}
    for field in message.fields:
        if field.name.lower() in candidates:
            return field.name
    return None


def detect_lidar_source(bag_file, requested_topic):
    import rosbag

    pointcloud_sources = []
    custom_sources = []
    print(f"[Detect] 扫描 bag: {bag_file}")
    with rosbag.Bag(bag_file, "r") as bag:
        for topic, message, _ in bag.read_messages(
                topics=[requested_topic] if requested_topic else None):
            if message._type == "sensor_msgs/PointCloud2":
                source = ("PointCloud2", topic, message._type)
                if source not in pointcloud_sources:
                    pointcloud_sources.append(source)
            elif message._type in SUPPORTED_CUSTOM_TYPES:
                source = ("CustomMsg", topic, message._type)
                if source not in custom_sources:
                    custom_sources.append(source)

    sources = pointcloud_sources + custom_sources
    if not sources:
        requested = f" topic '{requested_topic}'" if requested_topic else ""
        raise RuntimeError(f"未在{requested}检测到受支持的雷达消息")
    if len(sources) > 1:
        print("[Detect] 检测到多个受支持的点云源；按原工具规则优先使用 PointCloud2。")
    selected = sources[0]
    print(f"[Detect] 使用 {selected[2]} topic: {selected[1]}")
    return selected


def convert_pointcloud2(bag_file, output_dir, topic_name):
    import rosbag
    import sensor_msgs.point_cloud2 as point_cloud2

    all_points = []
    all_intensities = []
    intensity_field = None
    print(f"[Bag] 从 topic '{topic_name}' 合并 PointCloud2 点云...")
    with rosbag.Bag(bag_file, "r") as bag:
        for _, message, _ in bag.read_messages(topics=[topic_name]):
            if message._type != "sensor_msgs/PointCloud2":
                continue
            if intensity_field is None:
                intensity_field = find_intensity_field(message)
                if intensity_field is None:
                    raise RuntimeError("PointCloud2 中未找到 intensity/reflectivity 字段")
                print(f"[Bag] 检测到 intensity 字段: {intensity_field}")
            fields = ["x", "y", "z", intensity_field]
            for point in point_cloud2.read_points(
                    message, field_names=fields, skip_nans=True):
                all_points.append([point[0], point[1], point[2]])
                all_intensities.append(point[3])
    if not all_points:
        raise RuntimeError("指定 topic 中没有可用 PointCloud2 点")

    output_path = os.path.join(
        output_dir, "sensor_PointCloud2_inten_ascii.pcd")
    save_pcd_with_intensity(all_points, all_intensities, output_path)
    return output_path


def convert_custom_msg(bag_file, output_dir, topic_name, message_type):
    import rosbag

    all_points = []
    all_intensities = []
    print(f"[Bag] 从 topic '{topic_name}' 合并 {message_type} 点云...")
    with rosbag.Bag(bag_file, "r") as bag:
        for _, message, _ in bag.read_messages(topics=[topic_name]):
            if message._type != message_type:
                continue
            for point in message.points:
                all_points.append([point.x, point.y, point.z])
                all_intensities.append(point.reflectivity)
    if not all_points:
        raise RuntimeError(f"指定 topic 中没有可用 {message_type} 点")

    output_path = os.path.join(
        output_dir, "livox_CustomMsg_inten_ascii.pcd")
    save_pcd_with_intensity(
        all_points, np.asarray(all_intensities, dtype=np.float32), output_path)
    return output_path


def select_and_save_points(pcd_path):
    try:
        import open3d as o3d
    except ImportError as error:
        raise RuntimeError(
            "缺少 Python open3d；安装后才能执行交互选点，或使用 --no-pick 仅导出 PCD"
        ) from error

    cloud = o3d.io.read_point_cloud(pcd_path)
    if not cloud.has_points():
        raise RuntimeError(f"PCD 中没有点: {pcd_path}")
    print("请在窗口中按住 Shift 并单击至少 4 个点，然后按 Q 结束选点。")
    visualizer = o3d.visualization.VisualizerWithEditing()
    visualizer.create_window(window_name=f"选择点 - {os.path.basename(pcd_path)}")
    visualizer.add_geometry(cloud)
    visualizer.run()
    visualizer.destroy_window()
    selected_indices = visualizer.get_picked_points()
    if len(selected_indices) < 4:
        raise RuntimeError(f"只选择了 {len(selected_indices)} 个点，至少需要 4 个")

    selected_points = np.asarray(cloud.points)[selected_indices[:4], :]
    minimum = selected_points.min(axis=0) - 0.2
    maximum = selected_points.max(axis=0) + 0.2
    output_path = os.path.splitext(pcd_path)[0] + ".txt"
    with open(output_path, "w", encoding="utf-8") as output:
        output.write("# 4 selected points (x y z)\n")
        for point in selected_points:
            output.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
        output.write("# range values in order:\n")
        output.write(f"x_min: {minimum[0]:.1f}\n")
        output.write(f"x_max: {maximum[0]:.1f}\n")
        output.write(f"y_min: {minimum[1]:.1f}\n")
        output.write(f"y_max: {maximum[1]:.1f}\n")
        output.write(f"z_min: {minimum[2]:.1f}\n")
        output.write(f"z_max: {maximum[2]:.1f}\n")
    print(f"[Save] 已保存选点与范围到: {output_path}")


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="从 rosbag 生成 PCD，并交互选取 FAST-Calib 距离滤波范围")
    parser.add_argument("bag", help="输入 rosbag 路径")
    parser.add_argument("output_dir", help="PCD 和范围文件输出目录")
    parser.add_argument(
        "--topic", default="",
        help="LiDAR topic；省略时自动选择，PointCloud2 优先")
    parser.add_argument(
        "--no-pick", action="store_true",
        help="仅生成 PCD，不启动 Open3D 交互选点")
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    if not os.path.isfile(arguments.bag):
        raise RuntimeError(f"bag 文件不存在: {arguments.bag}")
    os.makedirs(arguments.output_dir, exist_ok=True)
    source_kind, topic, message_type = detect_lidar_source(
        arguments.bag, arguments.topic)
    if source_kind == "PointCloud2":
        pcd_path = convert_pointcloud2(arguments.bag, arguments.output_dir, topic)
    else:
        pcd_path = convert_custom_msg(
            arguments.bag, arguments.output_dir, topic, message_type)
    if not arguments.no_pick:
        select_and_save_points(pcd_path)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        sys.exit(1)
