#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ai_core_pkg.srv import Matcher, MatcherRequest

import cv2
import sys

def call_matcher_service(img_path0, img_path1):
    rospy.wait_for_service("match_images")
    try:
        match_images = rospy.ServiceProxy("match_images", Matcher)
        bridge = CvBridge()

        # Đọc ảnh bằng OpenCV
        img0_cv = cv2.imread(img_path0)
        img1_cv = cv2.imread(img_path1)
        img0_cv = cv2.resize(img0_cv, (500, 500), interpolation=cv2.INTER_AREA)
        img1_cv = cv2.resize(img1_cv, (500, 500), interpolation=cv2.INTER_AREA)
        if img0_cv is None or img1_cv is None:
            rospy.logerr("Không đọc được ảnh từ đường dẫn!")
            return

        # Convert OpenCV -> ROS Image
        img0_msg = bridge.cv2_to_imgmsg(img0_cv, encoding="bgr8")
        img1_msg = bridge.cv2_to_imgmsg(img1_cv, encoding="bgr8")

        # Tạo request
        req = MatcherRequest()
        req.img0 = img0_msg
        req.img1 = img1_msg

        # Gọi service
        resp = match_images(req)

        # In kết quả
        print("Số lượng keypoints0:", len(resp.keypoints0))
        print("Số lượng keypoints1:", len(resp.keypoints1))
        print("Confidence:", resp.confidence)  # in thử 10 giá trị đầu

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: rosrun ai_core_pkg test_matcher_client.py <img0_path> <img1_path>")
        sys.exit(1)

    rospy.init_node("test_matcher_client")
    call_matcher_service(sys.argv[1], sys.argv[2])