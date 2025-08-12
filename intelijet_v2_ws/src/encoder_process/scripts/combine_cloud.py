#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from laser_assembler.srv import AssembleScans2

def combine_cloud(start_time, end_time):
    rospy.wait_for_service('assemble_scans2')
    try:
        assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
        resp = assemble_scans(start_time, end_time)
        rospy.logwarn("Got combined cloud with %d points", len(resp.cloud.data))
        return resp.cloud
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None

if __name__ == "__main__":
    rospy.init_node("cloud_combiner_client")

    from rospy import Time, Duration
    from pps.helper import convert_pointcloud2_to_o3d
    import open3d as o3d
    # Ví dụ: ghép cloud từ 5 giây trước tới hiện tại
    now = rospy.Time.now()
    start = now - rospy.Duration(5.0)
    end = now

    cloud = combine_cloud(start, end)
    pcd = convert_pointcloud2_to_o3d(cloud)
    o3d.io.write_point_cloud('/mnt/c/work/projects/intelijet_v2/output.ply', pcd)
    if cloud:
        pub = rospy.Publisher("combined_cloud", PointCloud2, queue_size=1)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            pub.publish(cloud)
            rate.sleep()
