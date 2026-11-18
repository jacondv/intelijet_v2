# pps/cloud_utils/io_utils.py
"""Point-cloud file I/O and small ROS notification helpers.

Split out of the old pps/helper.py god-module (see docs/plan/phase_06_pps_cleanup.md).
"""
import rospy
from std_msgs.msg import Empty


def load_ply(filepath):
    import open3d as o3d
    try:
        pcd = o3d.io.read_point_cloud(filepath)
        if len(pcd.points) == 0:
            rospy.logwarn(f"[cloud_utils.load_ply] File has no data: {filepath}")
            return None
        return pcd
    except Exception as e:
        rospy.logerr(f"[cloud_utils.load_ply] Can't load file {filepath}: {e}")
        return None


def notify_one(topic_name: str):
    pub = rospy.Publisher(topic_name, Empty, queue_size=1, latch=True)
    rospy.sleep(0.5)  # Chờ publisher được đăng ký với master
    pub.publish(Empty())
    pub.unregister()  # Giải phóng sau khi publish nếu không cần giữ lại
