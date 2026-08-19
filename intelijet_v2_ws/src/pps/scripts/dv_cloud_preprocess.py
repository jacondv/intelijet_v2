#!/usr/bin/env python3

# This is the first step after the system receives cloud data from the lidar device

from pps.helper import crop_pointcloud_by_box
# from pps.tunnel_processing import TunnelProcessing
# from std_msgs.msg import Empty
import traceback
import rospy
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import ros_numpy
from shared.config_loader import CONFIG as cfg
from shared.notify import notify

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
        
        # rospy.init_node("dv_cloud_process")
        
        # Publishers
        self.pub_pre = rospy.Publisher(self.pub_pre_topic, PointCloud2, queue_size=1)
        self.pub_post = rospy.Publisher(self.pub_post_topic, PointCloud2, queue_size=1)

        # Subscribers
        rospy.Subscriber(self.sub_pre_topic, PointCloud2, self.callback_pres)
        rospy.Subscriber(self.sub_post_topic, PointCloud2, self.callback_post)

        rospy.loginfo("CloudProcessorNode initialized.")
        # rospy.spin()

    def process_cloud(self, msg: PointCloud2, rgb=[255,255,255]) -> PointCloud2:
        from pps.data_converter import cloudconverter
        # Chuyển sang numpy
        cloud = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
        cloud_o3d = o3d.geometry.PointCloud()
        cloud_o3d.points = o3d.utility.Vector3dVector(cloud)
        # aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
        # cloud_cropped = cloud_o3d.crop(aabb)

        # Remove non finite points
        cloud_o3d = cloud_o3d.remove_non_finite_points()
        # Crop by box 
        cloud_o3d = crop_pointcloud_by_box(pcd=cloud_o3d, box_type='aabb', 
                                        min_bound=[cfg.crop_box.min.x, cfg.crop_box.min.y, cfg.crop_box.min.z], 
                                        max_bound=[cfg.crop_box.max.x, cfg.crop_box.max.y, cfg.crop_box.max.z])
        
        #Downsample        
        cloud_o3d = cloudconverter.voxel_down_sample_spatial(cloud_o3d, voxel_size=0.015)
        # Auto crop boundary
        # tunnel = TunnelProcessing(cloud_o3d)
        # result = tunnel.run_processing_pipeline()
        # Cloud after process
        return cloudconverter.legacy_o3d_to_pointcloud2(cloud_o3d, frame_id=msg.header.frame_id, rgb=rgb)

    def callback_pres(self, msg):
        rospy.loginfo("Received /pre_scan_0")
        try:
            processed = self.process_cloud(msg, rgb=[255,255,255])
        except Exception as e:
            rospy.logerr(f"{type(e).__name__}: {e}\n{traceback.format_exc()}")
            notify(message=f"[ERROR] Pre-scan cloud preprocessing failed: {e}", level="error")
            return
        if isinstance(processed, PointCloud2):
            rospy.loginfo("processed is a PointCloud2 message and sent --> pre_scan_0")
        else:
            rospy.logerr("processed is NOT a PointCloud2 message.")
            notify(message="[ERROR] Pre-scan cloud preprocessing returned invalid data", level="error")
            return

        self.pub_pre.publish(processed)


    def callback_post(self, msg):
        rospy.loginfo("Received /post_scan_0")
        try:
            processed = self.process_cloud(msg, rgb=[255,255,0])
        except Exception as e:
            rospy.logerr(f"{type(e).__name__}: {e}\n{traceback.format_exc()}")
            notify(message=f"[ERROR] Post-scan cloud preprocessing failed: {e}", level="error")
            return
        if isinstance(processed, PointCloud2):
            rospy.loginfo("processed is a PointCloud2 message and sent --> post_scan_0")
        else:
            rospy.logerr("processed is NOT a PointCloud2 message.")
            notify(message="[ERROR] Post-scan cloud preprocessing returned invalid data", level="error")
            return

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
