#!/usr/bin/env python3
import rospy
# import ros_numpy
# import numpy as np
# import open3d as o3d
from sensor_msgs.msg import PointCloud2
from pps.helper import compute_heatmap_to_plane, assign_colors_by_threshold, color_voxel_majority, \
    convert_open3d_to_pointcloud2, convert_open3d_to_pointcloud2_with_diff, convert_pointcloud2_to_o3d, convert_open3d_to_pointcloud2_v2
from pps.data_converter import CloudConverter

cloudconverter = CloudConverter()

from shared.config_loader import CONFIG as cfg

CLOUD_COMPARED  = cfg.CLOUD_COMPARED_TOPIC
PRE_SCAN_CLOUD  = cfg.PRE_SCAN_CLOUD_TOPIC
POST_SCAN_CLOUD = cfg.POST_SCAN_CLOUD_ALIGNED_TOPIC

THICKNESS_TARGET = cfg.thickness.target/1000  # Target thickness in meter -> convert mm to m
THICKNESS_TOLERANCE = cfg.thickness.tolerance/1000  # Allowable tolerance in meter of thickness

class CloudComparer:
    def __init__(self, pubpish_topic,
                 pres_topic,
                 post_topic):
        
        self.publish_topic = pubpish_topic
        self.pres_topic = pres_topic
        self.post_topic = post_topic

        self.pres_cloud = None
        self.post_cloud = None
        self.compared_cloud = None

        self.got_pres = False
        self.got_post = False
        self.frame_id = "base_link"

        self.target_thickness = THICKNESS_TARGET # Target thickness in meter
        self.tolerance_thickness = THICKNESS_TOLERANCE # Allowable tolerance in meter of thickness

        self.pub = rospy.Publisher(self.publish_topic, PointCloud2, queue_size=1)
        rospy.Subscriber(self.pres_topic, PointCloud2, self.callback_pres)
        rospy.Subscriber(self.post_topic, PointCloud2, self.callback_post)
       
    def callback_pres(self, msg):
        self.frame_id = msg.header.frame_id
        rospy.loginfo(f"Received from {self.pres_topic}")
        if not isinstance(msg, PointCloud2):
            rospy.logerr("Received message is not of type PointCloud2.")
            return
        # self.pres_cloud = convert_pointcloud2_to_o3d(msg)
        self.pres_cloud = cloudconverter.pointcloud2_to_o3d_tensor(msg)
        # self.pub.publish(convert_open3d_to_pointcloud2(self.pres_cloud, frame_id=msg.header.frame_id))
        self.got_pres = True

    def callback_post(self, msg):
        rospy.loginfo(f"Received from {self.post_topic}")
        if not isinstance(msg, PointCloud2):
            rospy.logerr("Received message is not of type PointCloud2.")
            return
        # self.post_cloud = convert_pointcloud2_to_o3d(msg)
        self.post_cloud = cloudconverter.pointcloud2_to_o3d_tensor(msg)
        self.got_post = True

        if self.got_pres and self.got_post:
            self.compared_cloud = self.compare(self.pres_cloud, self.post_cloud)

        if self.compared_cloud is None:
            rospy.logwarn("No comparison result available yet.")
            return
        rospy.loginfo("Publishing compared cloud...")
        # self.compared_cloud = convert_open3d_to_pointcloud2_v2(self.compared_cloud, frame_id=self.frame_id)
        self.compared_cloud = cloudconverter.o3d_tensor_to_pointcloud2(self.compared_cloud, frame_id=self.frame_id)
        self.pub.publish(self.compared_cloud)
        self.got_post = False


    def compare(self, pres, post):
        # Thực hiện xử lý màu hóa theo khoảng cách
        result, dists = compute_heatmap_to_plane(post, pres, k=6, 
                                                 target_thickness=self.target_thickness, 
                                                 tolerance_thickness=self.tolerance_thickness)
        # result = assign_colors_by_threshold(result, dists, threshold=[THICKNESS_MIN, THICKNESS_MAX])
        # result = color_voxel_majority(result, voxel_size=0.05)
        return result
    
    

def main():
    rospy.init_node("dv_cloud_compare", anonymous=False)
    rospy.loginfo("Cloud comparison node started.")
    
    CloudComparer(pubpish_topic=CLOUD_COMPARED,
                             pres_topic=PRE_SCAN_CLOUD,
                             post_topic=POST_SCAN_CLOUD)
    
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception occurred. Shutting down the node %s.", rospy.get_name())