#!/usr/bin/env python3
import rospy
import ros_numpy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from pps.helper import compute_heatmap_to_plane, assign_colors_by_threshold, color_voxel_majority, \
    convert_open3d_to_pointcloud2, convert_open3d_to_pointcloud2_with_diff, convert_pointcloud2_to_o3d

CLOUD_COMPARED  = "/cloud_compared"
PRE_SCAN_CLOUD  = "/pre_scan_cloud"
POST_SCAN_CLOUD = "/post_scan_cloud_aligned"

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

        self.pub = rospy.Publisher(self.publish_topic, PointCloud2, queue_size=1)
        rospy.Subscriber(self.pres_topic, PointCloud2, self.callback_pres)
        rospy.Subscriber(self.post_topic, PointCloud2, self.callback_post)
       
    def callback_pres(self, msg):
        self.frame_id = msg.header.frame_id
        rospy.loginfo(f"Received from {self.pres_topic}")
        if not isinstance(msg, PointCloud2):
            rospy.logerr("Received message is not of type PointCloud2.")
            return
        self.pres_cloud = convert_pointcloud2_to_o3d(msg)
        self.pub.publish(convert_open3d_to_pointcloud2(self.pres_cloud, frame_id=msg.header.frame_id))
        self.got_pres = True

    def callback_post(self, msg):
        rospy.loginfo(f"Received from {self.post_topic}")
        if not isinstance(msg, PointCloud2):
            rospy.logerr("Received message is not of type PointCloud2.")
            return
        self.post_cloud = convert_pointcloud2_to_o3d(msg)
        self.got_post = True

        if self.got_pres and self.got_post:
            self.compared_cloud, diff = self.compare(self.pres_cloud, self.post_cloud)

        if self.compared_cloud is None:
            rospy.logwarn("No comparison result available yet.")
            return
        rospy.loginfo("Publishing compared cloud...")
        self.compared_cloud = convert_open3d_to_pointcloud2_with_diff(self.compared_cloud, diff_array=diff, frame_id=self.frame_id)
        self.pub.publish(self.compared_cloud)
        self.got_post = False


    def compare(self, pres, post):
        # Thực hiện xử lý màu hóa theo khoảng cách
        colored_source, dists = compute_heatmap_to_plane(post, pres, k=6)
        result = assign_colors_by_threshold(colored_source, dists, threshold=[0.03, 0.045])
        result = color_voxel_majority(result, voxel_size=0.03)
        return result, dists
    
    
    # def convert_pointcloud2_to_o3d(self, msg):
    #     """Convert PointCloud2 message to Open3D PointCloud."""
    #     if not isinstance(msg, PointCloud2):
    #         rospy.logerr("Input message is not of type PointCloud2.")
    #         return None
    #     cloud_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
    #     cloud_o3d = o3d.geometry.PointCloud()
    #     cloud_o3d.points = o3d.utility.Vector3dVector(cloud_np)
    #     return cloud_o3d

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