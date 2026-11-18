#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from laser_assembler.srv import AssembleScans2

def call_assemble_scans():
    rospy.init_node('assemble_scans_client')

    pub = rospy.Publisher('/assembled_cloud', PointCloud2, queue_size=10)

    rospy.wait_for_service('/assemble_scans2')
    try:
        assemble_scans = rospy.ServiceProxy('/assemble_scans2', AssembleScans2)

        start_time = rospy.Time.now() - rospy.Duration(40.0)  
        end_time = rospy.Time.now()

        rospy.loginfo("Calling assemble_scans2 from %.2f to %.2f" % (start_time.to_sec(), end_time.to_sec()))
        resp = assemble_scans(start_time, end_time)

        if isinstance(resp.cloud, PointCloud2):
            rospy.loginfo("Assembled cloud has width %d" % resp.cloud.width)
            rate = rospy.Rate(1)  # publish 1 lần/giây

            # Publish pointcloud trong vài giây để RViz nhận kịp
            for _ in range(5):
                pub.publish(resp.cloud)
                rate.sleep()
        else:
            rospy.logwarn("No point cloud returned.")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    call_assemble_scans()
