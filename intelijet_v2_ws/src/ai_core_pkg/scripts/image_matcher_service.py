#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ai_core_pkg.srv import Matcher, MatcherResponse
# from ai_core_pkg.msg import MatcherMsg
from ai_core_pkg.matchers.factory import MatcherFactory  # file chứa class bạn đã viết
import time

class MatcherServiceNode:
    def __init__(self):
        self.bridge = CvBridge()
        # Khởi tạo matcher từ factory
        # matcher_name = rospy.get_param("~matcher_name", "LoFTR")
        # device = rospy.get_param("~device", "cpu")
        matcher_name = "LoFTR"
        device = "cpu"
        self.matcher = MatcherFactory.create(matcher_name, device)

        # Khởi tạo service
        self.service = rospy.Service("match_images", Matcher, self.handle_match)

    def handle_match(self, req):
        # Convert ROS Image -> OpenCV
        t0 = time.perf_counter()
        print(f"Start match: {t0*1000:.2f} ms")
        img0 = self.bridge.imgmsg_to_cv2(req.img0, desired_encoding="bgr8")
        img1 = self.bridge.imgmsg_to_cv2(req.img1, desired_encoding="bgr8")

        # Gọi matcher
        matches = self.matcher.match(img0, img1)
        # Giả sử matches trả về dict {"keypoints0":..., "keypoints1":..., "confidence":...}

        # print("Matches obtained from matcher:", matches)

        mkpts0 = matches["keypoints0"].cpu().numpy()
        kpts0 = mkpts0.astype("float32").reshape(-1).tolist()

        mkpts1 = matches["keypoints1"].cpu().numpy()
        kpts1 = mkpts1.astype("float32").reshape(-1).tolist()

        mconf = matches["confidence"].cpu().numpy()
        conf = mconf.astype("float32").reshape(-1).tolist()

        # rospy.loginfo(f"type(matches) = {type(matches)}")
        # print(matches)
        # Tạo response
        resp = MatcherResponse()
        # resp.matcher = MatcherMsg()
        resp.keypoints0 = kpts0
        resp.keypoints1 = kpts1
        resp.confidence = conf if matches.get("confidence") is not None else []
        t1 = time.perf_counter()
        print(f"Inference time: {(t1 - t0)*1000:.2f} ms")
        rospy.logwarn(f"Inference time: {(t1 - t0)*1000:.2f} ms")
        return resp

if __name__ == "__main__":
    rospy.init_node("matcher_service_node")
    node = MatcherServiceNode()
    rospy.loginfo("Matcher service ready.")
    rospy.spin()