#!/usr/bin/env python3

# This is the first step after the system receives cloud data from the lidar device

from pps.helper import convert_open3d_to_pointcloud2, crop_pointcloud_by_box, notify_one, detect_boundary_pca, remove_boundary_region
from std_msgs.msg import Empty
import rospy
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import ros_numpy
import numpy as np
from datetime import datetime
from shared.config_loader import CONFIG as cfg

PRE_SCAN_RAW_TOPIC = "/pre_scan_0"
POST_SCAN_RAW_TOPIC = "/post_scan_0"
PRE_SCAN_PROCESSED_TOPIC = "/pre_scan_cloud"
POST_SCAN_PROCESSED_TOPIC = "/post_scan_cloud"

class CloudProcessorNode:
    def __init__(self, pub_pre_topic,
                 pub_post_topic,
                 sub_pre_topic,
                 sub_post_topic):
        
        self.pub_pre_topic = pub_pre_topic
        self.pub_post_topic = pub_post_topic
        self.sub_pre_topic = sub_pre_topic
        self.sub_post_topic = sub_post_topic
        
        rospy.init_node("dv_cloud_process")
        
        # Publishers
        self.pub_pre = rospy.Publisher(self.pub_pre_topic, PointCloud2, queue_size=1)
        self.pub_post = rospy.Publisher(self.pub_post_topic, PointCloud2, queue_size=1)

        # Subscribers
        rospy.Subscriber(self.sub_pre_topic, PointCloud2, self.callback_pres)
        rospy.Subscriber(self.sub_post_topic, PointCloud2, self.callback_post)

        rospy.loginfo("CloudProcessorNode initialized.")
        rospy.spin()

    def process_cloud(self, msg: PointCloud2, rgb=[255,255,255]) -> PointCloud2:

        # Chuyển sang numpy
        cloud = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
        cloud_o3d = o3d.geometry.PointCloud()
        cloud_o3d.points = o3d.utility.Vector3dVector(cloud)

        # aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
        # cloud_cropped = cloud_o3d.crop(aabb)

        result = crop_pointcloud_by_box(pcd=cloud_o3d, box_type='aabb', 
                                               min_bound=[cfg.crop_box.min.x, cfg.crop_box.min.y, cfg.crop_box.min.z], 
                                               max_bound=[cfg.crop_box.max.x, cfg.crop_box.max.y, cfg.crop_box.max.z])
        

        # Lọc nhiễu
        # cloud_cropped, _ = cloud_cropped.remove_statistical_outlier(nb_neighbors=5, std_ratio=1)

        # mask, boder = detect_boundary_pca(result.voxel_down_sample(0.1), k=30, angle_threshold=np.pi/2)
        # result = remove_boundary_region(result, boder, radius=0.1)

        # Downsample
        if not isinstance(result, o3d.geometry.PointCloud):
            rospy.logerr("[%s] cloud_cropped is not an Open3D PointCloud." % rospy.get_name())
        else:
            # cloud_cropped = cloud_cropped.voxel_down_sample(voxel_size=0.01)
            pass # giữ nguyên độ phân giải gốc

        return convert_open3d_to_pointcloud2(result, frame_id=msg.header.frame_id,rgb=rgb)


    def callback_pres(self, msg):
        rospy.loginfo("Received /pre_scan_0")
        processed = self.process_cloud(msg, rgb=[255,255,255])
        if isinstance(processed, PointCloud2):
            rospy.loginfo("processed is a PointCloud2 message.")
        else:
            rospy.logerr("processed is NOT a PointCloud2 message.")

        self.pub_pre.publish(processed)


    def callback_post(self, msg):
        rospy.loginfo("Received /post_scan_0")
        processed = self.process_cloud(msg, rgb=[255,255,0])
        if isinstance(processed, PointCloud2):
            rospy.loginfo("processed is a PointCloud2 message.")
        else:
            rospy.logerr("processed is NOT a PointCloud2 message.")
            
        self.pub_post.publish(processed)

def main():
    rospy.init_node("dv_cloud_process", anonymous=False)
    rospy.loginfo("Cloud processing node started.")
    
    CloudProcessorNode(pub_pre_topic=PRE_SCAN_PROCESSED_TOPIC,
                        pub_post_topic=POST_SCAN_PROCESSED_TOPIC,
                        sub_pre_topic=PRE_SCAN_RAW_TOPIC,
                        sub_post_topic=POST_SCAN_RAW_TOPIC)

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception occurred. Shutting down the node %s.", rospy.get_name())
