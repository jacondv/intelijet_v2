#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Image
from datetime import datetime
import open3d as o3d
import copy

import numpy as np
import cv2

from pps.helper import convert_open3d_to_pointcloud2, convert_pointcloud2_to_o3d, convert_msg_to_image
from pps.cloud_processing.base_aligner import CloudAligner
from pps.cloud_processing.icp_aligner import ICPConfig
from pps.cloud_processing.align_manager import PointCloudAlignerManager
from pps.image_processing.keypoint_processing import KeypointCloudAlignManager
# from cloud_processing.ransac_aligner import RANSACAligner

# Cloud topics
POST_SCAN_CLOUD= "/post_scan_cloud"
PRE_SCAN_CLOUD = "/pre_scan_cloud"
ALIGNED_CLOUD = "/post_scan_cloud_aligned"

# Image topics
PRE_SCAN_IMAGE = "/pre_scan_0/image"
POST_SCAN_IMAGE = "/post_scan_0/image"

# Flags
ENABLE_ALIGNMENT = True

#IPC Param
ICP_THRESHOLDS = [1.0, 0.1, 0.05]      # coarse → fine
ICP_MAX_ITERS = [30, 40, 50]           # coarse → fine
ICP_ALIGN_AREA = None                  # hoặc [[xmin, xmax], [ymin, ymax], [zmin, zmax]]

# === Keypoint Matching Parameters ===
# CAMERA_INTRINSICS = [653.76474515, 655.82709085, 756.55566752, 541.23742774]
CAMERA_INTRINSICS = np.array([
    [653.76474515,     0.0,         756.55566752],
    [0.0,              655.82709085, 541.23742774],
    [0.0,              0.0,           1.0]
], dtype=np.float64)

# LIDAR_TO_CAM_EXTRINSIC = np.array([
#     [1,  0,  0, 0],
#     [0,  0, -1, 0],
#     [0,  1,  0, 0],
#     [0,  0,  0, 1]
# ], dtype=np.float64)

# từ base → camera front
LIDAR_TO_CAM_EXTRINSIC_ =  np.array([
    [-0.98383629,  0.00314181, -0.17904252, -0.00723122],
    [ 0.17903614, -0.0022086,  -0.98383999, -0.04821113],
    [-0.00348647, -0.99999261,  0.00161041, -0.10187813],
    [ 0.        ,  0.        ,  0.        ,  1.        ]
])

LIDAR_TO_CAM_EXTRINSIC = np.linalg.inv(LIDAR_TO_CAM_EXTRINSIC_)

# DIST_COEFFS = [-0.06065661 , 0.04854407 ,-0.0606453  , 0.02428142]
DIST_COEFFS = np.array([-0.06065661 , 0.04854407 ,-0.0606453  , 0.02428142])

FEATURE_METHOD = "SIFT"
PIXEL_RADIUS = 100
CLOUD_RADIUS = 0.5
MATCH_RATIO = 0.6

class CloudAlignNode:
    def __init__(self, source_topic, 
                 target_topic, 
                 aligned_topic, 
                 image_prescan_topic,
                 image_postscan_topic,
                 enable_align=True):
        
        self.node_name = "dv_cloud_align"
        self._enable_align = enable_align
        self.source = None
        self.target = None
        self.__aligner = None
        self.__keypoint_manager = None

        rospy.Subscriber(source_topic, PointCloud2, self.source_callback)
        rospy.Subscriber(target_topic, PointCloud2, self.target_callback)
        rospy.Subscriber(image_prescan_topic, Image, self.image_prescan_callback)
        rospy.Subscriber(image_postscan_topic, Image, self.image_postscan_callback)

        self.pub = rospy.Publisher(aligned_topic, PointCloud2, queue_size=1)
        
        self.align_srv = rospy.Service("start_align", Trigger, self.handle_start_align)

    def __try_align(self):
        timeout = 10
        start_time = rospy.Time.now().to_sec()

        while self.__keypoint_manager.process_status !=2:
             rospy.sleep(0.1)
             if rospy.Time.now().to_sec() - start_time > timeout:
                rospy.logwarn("Timeout: keypoint_manager is not ready after 10 seconds.")
                break
             
        if self.__keypoint_manager.is_ready():    
            # cloud1_target, cloud2_source, T = self.__keypoint_manager.get_result()
            _, source_patch, T = self.__keypoint_manager.get_result()
            rospy.logwarn("T is:\n%s", T)
            __timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")  
            # o3d.io.write_point_cloud(f"/mnt/c/work/projects/intelijet_v2/data/cloud1_target{__timestamp}.ply", cloud1_target)
            # o3d.io.write_point_cloud(f"/mnt/c/work/projects/intelijet_v2/data/cloud2_source{__timestamp}.ply", cloud2_source)  
                    
            img = self.__keypoint_manager.draw_result()
            cv2.imwrite(f"/mnt/c/work/projects/intelijet_v2/data/keypoint_postcan_{__timestamp}.png", img)
            rospy.logwarn("[Aligner by keypoint]")
        else:
            # cloud1_target = self.target
            # cloud2_source = self.source
            rospy.logwarn("[Aligner by original cloud]")

        # source_temp = copy.deepcopy(self.source)        

        source_temp = copy.deepcopy(self.source)     
        rospy.logwarn("source_temp T matrix 0 : \n%s", source_temp.transformation)
        print("source_temp T matrix 0 : \n%s", source_temp.transformation)
        source_temp.transform(T)
        rospy.logwarn("source_temp T matrix 1 : \n%s", source_temp.transformation)
        print("source_temp T matrix 1 : \n%s", source_temp.transformation)
        _ = self.__aligner.align(source_temp, self.target)
        rospy.logwarn("source_temp T matrix 2 : \n%s", source_temp.transformation)
        print("source_temp T matrix 2 : \n%s", source_temp.transformation)
        T_2 = self.__aligner.get_transformation_matrix()

        source_patch.transform(T_2@T)
        _ = self.__aligner.align(source_patch, self.target)
        T_3 = self.__aligner.get_transformation_matrix()

        transformation_matrix = T_3@T_2@T
        rospy.logwarn("transformation_matrix is : \n%s", transformation_matrix)
        print("transformation_matrix is : \n%s", transformation_matrix)

        # o3d.io.write_point_cloud(f"/mnt/c/work/projects/intelijet_v2/data/pcd1{__timestamp}.ply", pcd1)
  
        result = copy.deepcopy(self.source)
        if transformation_matrix is not None:
            # Transform the source (postscan) cloud using the calculated transformation matrix
            result.transform(transformation_matrix)
        else:
            rospy.logwarn("Transformation matrix is None, returning source cloud without alignment.")
            result = self.source

        # o3d.io.write_point_cloud(f"/mnt/c/work/projects/intelijet_v2/data/pcd2{__timestamp}.ply", result)
        return result
            

    def source_callback(self, msg):
        #Postscan msg is the source cloud
        if not self._enable_align:
            self.pub.publish(msg)
            return

        self.source = convert_pointcloud2_to_o3d(msg)
        self.__keypoint_manager.set_cloud2(self.source)
        # aligned = self.__try_align()
        # self.pub.publish(convert_open3d_to_pointcloud2(aligned))

    def target_callback(self, msg):
        #Prescan msg is the target cloud
        if not self._enable_align:
            return
        self.target = convert_pointcloud2_to_o3d(msg)
        self.__keypoint_manager.set_cloud1(self.target)
        
    def image_prescan_callback(self, msg):
        #Prescan image is the target image
        rospy.loginfo("Received prescan image callback")
        img = convert_msg_to_image(msg)
        if not self._enable_align:
            return
        self.__keypoint_manager.set_image1(img)

    def image_postscan_callback(self, msg):
        rospy.loginfo("Received postscan image callback")
        #Postscan image is the source image
        if not self._enable_align:
            return
        img = convert_msg_to_image(msg)
        self.__keypoint_manager.set_image2(img)

    def handle_start_align(self, req):
        if not self._enable_align:
            return TriggerResponse(success=False, message="Alignment disabled.")
        
        try:
            rospy.loginfo("Starting cloud alignment...")
            aligned = self.__try_align()
            self.pub.publish(convert_open3d_to_pointcloud2(aligned))
            return TriggerResponse(success=True, message="Alignment completed and published.")
        except Exception as e:
            rospy.logerr(f"[Align Error] {e}")
            return TriggerResponse(success=False, message=str(e))
    

    def get_transformation_matrix(self):
        return self.__aligner.get_transformation_matrix()

    def build_keypoint_extracter(self, camera_intrinsics, 
                                       lidar_to_cam_extrinsic, 
                                       dist_coeffs=None,
                                       feature_method="SIFT",
                                       pixel_radius=5,
                                       cloud_radius=0.3,
                                       match_ratio=0.5):
        
        self.__keypoint_manager = KeypointCloudAlignManager(camera_intrinsics=camera_intrinsics,
                                                            lidar_to_cam_extrinsic=lidar_to_cam_extrinsic,
                                                            dist_coeffs=dist_coeffs,
                                                            feature_method=feature_method,
                                                            pixel_radius=pixel_radius,
                                                            cloud_radius=cloud_radius,
                                                            match_ratio=match_ratio)
    
    def build_cloud_aligner(self,aligner:PointCloudAlignerManager):
        """
        Set a custom aligner for the node.
        """
        if not isinstance(aligner, PointCloudAlignerManager):
            rospy.logerr("Aligner must be an instance of PointCloudAlignerManager.")
            return
        self.__aligner = aligner

    

def main():
    rospy.init_node("dv_cloud_align", anonymous=False)
    try:
        node = CloudAlignNode(source_topic=POST_SCAN_CLOUD,
                       target_topic=PRE_SCAN_CLOUD,
                       aligned_topic=ALIGNED_CLOUD,
                       image_prescan_topic=PRE_SCAN_IMAGE,
                       image_postscan_topic=POST_SCAN_IMAGE,
                       enable_align=ENABLE_ALIGNMENT)
        
        node.build_keypoint_extracter(
            camera_intrinsics=CAMERA_INTRINSICS,  # Example values
            lidar_to_cam_extrinsic=LIDAR_TO_CAM_EXTRINSIC ,
            dist_coeffs=DIST_COEFFS ,
            feature_method=FEATURE_METHOD ,
            pixel_radius=PIXEL_RADIUS ,
            cloud_radius=CLOUD_RADIUS,
            match_ratio=MATCH_RATIO 
        )

        node.build_cloud_aligner(
            PointCloudAlignerManager(
                strategy="icp",
                config=ICPConfig(
                    threshold=ICP_THRESHOLDS,
                    max_iters=ICP_MAX_ITERS,
                    align_area=ICP_ALIGN_AREA
                )
            )
        )

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr(f"ROS Interrupt Exception occurred. Shutting down CloudAlignNode.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass