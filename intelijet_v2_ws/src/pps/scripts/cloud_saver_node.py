#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import datetime
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
from shared.config_loader import CONFIG as cfg
from pps.helpers import convert_pointcloud2_to_o3d

# Các topic cần theo dõi (load từ config)
PRE_SCAN_RAW_TOPIC = cfg.PRE_SCAN_TOPIC       # "/pre_scan_0"
POST_SCAN_RAW_TOPIC = cfg.POST_SCAN_TOPIC     # "/post_scan_0"
PRE_SCAN_PROCESSED_TOPIC = cfg.PRE_SCAN_CLOUD_TOPIC   # "/pre_scan_cloud"
POST_SCAN_PROCESSED_TOPIC = cfg.POST_SCAN_CLOUD_TOPIC # "/post_scan_cloud"

# Thư mục gốc để lưu point cloud
SAVE_DIR = os.path.join(cfg.BASE_DIR, "data")

if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)

def cloud_callback(msg, topic_name):
    """Subscriber callback: convert and save .ply into SAVE_DIR/yyyyMMdd/"""
    pcd = convert_pointcloud2_to_o3d(msg)
    if pcd is None or len(pcd.points) == 0:
        rospy.logwarn("cloud_callback: empty or invalid cloud from %s", topic_name)
        return

    # Prefer message timestamp; fallback to now

    dt = datetime.datetime.now()
    day_str = dt.strftime("%Y%m%d")
    day_folder = os.path.join(SAVE_DIR, day_str)
    os.makedirs(day_folder, exist_ok=True)

    timestamp = dt.strftime("%Y%m%d_%H%M%S")
    topic_short = topic_name.strip("/").replace("/", "_")
    filename = "{}_{}.ply".format(timestamp, topic_short)
    filepath = os.path.join(day_folder, filename)

    try:
        ok = o3d.io.write_point_cloud(filepath, pcd)
        if ok:
            rospy.loginfo("Saved cloud from %s -> %s", topic_name, filepath)
        else:
            rospy.logerr("Failed to write pointcloud to %s", filepath)
    except Exception as e:
        rospy.logerr("Exception while saving pointcloud from %s: %s", topic_name, e)


def main():
    rospy.init_node("cloud_saver", anonymous=True)

    # Đăng ký subscriber cho từng topic
    rospy.Subscriber(PRE_SCAN_RAW_TOPIC, PointCloud2, cloud_callback, PRE_SCAN_RAW_TOPIC)
    rospy.Subscriber(POST_SCAN_RAW_TOPIC, PointCloud2, cloud_callback, POST_SCAN_RAW_TOPIC)
    rospy.Subscriber(PRE_SCAN_PROCESSED_TOPIC, PointCloud2, cloud_callback, PRE_SCAN_PROCESSED_TOPIC)
    rospy.Subscriber(POST_SCAN_PROCESSED_TOPIC, PointCloud2, cloud_callback, POST_SCAN_PROCESSED_TOPIC)

    rospy.loginfo("Cloud saver node started. Saving PLY files under %s/<yyyyMMdd>/", SAVE_DIR)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception occurred. Shutting down node %s.", rospy.get_name())
