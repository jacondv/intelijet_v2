#!/usr/bin/env python3

import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from pps.data_converter import cloudconverter


def publish_pcd(pre_scan_path=None, post_scan_path=None):
    """
    Publish 1 hoặc 2 PCD files lên topic và kết thúc.
    """
    rospy.init_node("pcd_publisher_once", anonymous=True)

    publishers = {}
    if pre_scan_path:
        publishers['pre'] = rospy.Publisher("/pre_scan_cloud", PointCloud2, queue_size=1, latch=True)
    if post_scan_path:
        publishers['post'] = rospy.Publisher("/post_scan_cloud", PointCloud2, queue_size=1, latch=True)

    # ROS cần sleep 1s để publisher register
    rospy.sleep(1)

    if pre_scan_path:
        cloud = o3d.t.io.read_point_cloud(pre_scan_path)
        msg = cloudconverter.o3d_tensor_to_pointcloud2(cloud, frame_id="base_link")
        publishers['pre'].publish(msg)
        rospy.loginfo("Published pre-scan: %s", pre_scan_path)

    rospy.sleep(10)

    if post_scan_path:
        cloud = o3d.t.io.read_point_cloud(post_scan_path)
        msg = cloudconverter.o3d_tensor_to_pointcloud2(cloud, frame_id="base_link")
        publishers['post'].publish(msg)
        rospy.loginfo("Published post-scan: %s", post_scan_path)
        
    rospy.sleep(10)
    # Không spin, node sẽ kết thúc ngay sau khi publish


if __name__ == "__main__":
    pre_path = "/mnt/c/work/projects/intelijet_v2/data/Projects/TEST4/ARM5DegFront/ARM5DegFront#20251121_084455#pre_scan_cloud_02.ply"
    
    post_path = "/mnt/c/work/projects/intelijet_v2/data/Projects/TEST4/ARM5DegFront/ARM5DegFront#20251121_094034#post_scan_cloud_09.ply"

    publish_pcd(pre_scan_path=pre_path, post_scan_path=post_path)
