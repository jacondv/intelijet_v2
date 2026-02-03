import rospy
from ai_core_pkg.srv import Matcher, MatcherRequest
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

_bridge = CvBridge()
_srv = None
SERVICE_NAME = 'match_images'
def call_matcher_service(img0_cv, img1_cv, method="loftr",conf_th=0.7):
    """
    This client calls the image matcher service to get matched keypoints between two images.
    
    :param img0_cv: Input image 0 in OpenCV format (numpy array)
    :param img1_cv: Input image 1 in OpenCV format (numpy array)
    :param method: Matching method to use (e.g., "loftr")
    :return: kpts0, kpts1, conf
        kpts0: 2N numpy array of keypoints in image 0
        kpts1: 2N numpy array of keypoints in image 1
        conf:  N numpy array of confidence scores
        Note: we should reshape keypoints from flat list to Nx2 array
    """
    global _srv
    
    try:
        rospy.wait_for_service(SERVICE_NAME, timeout=1.0)
    except rospy.ROSException:
        rospy.logwarn(f"{SERVICE_NAME} service not available")
        return None
    
    if _srv is None:
        rospy.wait_for_service(SERVICE_NAME)
        _srv = rospy.ServiceProxy(SERVICE_NAME, Matcher)

    # req = Matcher.Request()
    req = MatcherRequest()
    req.method = method
    req.img0 = _bridge.cv2_to_imgmsg(img0_cv, encoding="bgr8")
    req.img1 = _bridge.cv2_to_imgmsg(img1_cv, encoding="bgr8")

    resp = _srv(req)

    # reshape lại keypoints
    kpts0 = np.array(resp.keypoints0, dtype=np.float32).reshape(-1, 2)
    kpts1 = np.array(resp.keypoints1, dtype=np.float32).reshape(-1, 2)
    conf  = np.array(resp.confidence, dtype=np.float32)


    mask = conf >= conf_th

    kpts0_m = kpts0[mask]   # (M, 2)
    kpts1_m = kpts1[mask]   # (M, 2)
    conf_m  = conf[mask]

    good_matches = [
        cv2.DMatch(_queryIdx=i, _trainIdx=i, _distance=1.0 - conf_m[i])
        for i in range(len(conf_m))
    ]


    return kpts0_m, kpts1_m, good_matches
