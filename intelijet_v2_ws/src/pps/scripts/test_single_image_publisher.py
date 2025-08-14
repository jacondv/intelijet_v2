#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def publish_single_image(image_path, topic="/pre_scan_image"):
    """
    Publish một ảnh duy nhất lên topic ROS dưới dạng sensor_msgs/Image.
    """
    rospy.init_node("single_image_publisher", anonymous=True)
    pub = rospy.Publisher(topic, Image, queue_size=1, latch=True)  # latch: giữ lại message mới nhất
    bridge = CvBridge()

    # Đọc ảnh từ file
    img = cv2.imread(image_path)
    if img is None:
        rospy.logerr(f"Not found : {image_path}")
        return

    # Convert sang message
    img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    rospy.loginfo(f"Publish image to topic: {topic}")
    pub.publish(img_msg)

    rospy.sleep(1.0)  # Chờ gửi xong


if __name__ == "__main__":
    try:
        image_path = "intelijet_v2_ws/src/pps/img1.png"  # <<<--- Thay đổi đường dẫn ảnh tại đây
        publish_single_image(image_path)
    except rospy.ROSInterruptException:
        pass
