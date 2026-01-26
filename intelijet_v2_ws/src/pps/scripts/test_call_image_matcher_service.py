#!/usr/bin/env python3
import rospy
import cv2

from ai_core_pkg.image_matcher_client import call_matcher_service

def main():
    rospy.init_node("test_matcher_client", anonymous=True)


    # Đọc ảnh
    img0 = cv2.imread("/mnt/c/WORK/projects/intelijet_v2/intelijet_v2_ws/src/ai_core_pkg/images/matches_1769067378842_pos.png")
    img1 = cv2.imread("/mnt/c/WORK/projects/intelijet_v2/intelijet_v2_ws/src/ai_core_pkg/images/matches_1769067378842_pre.png")

    if img0 is None or img1 is None:
        rospy.logerr("Cannot load images")
        return

    # Resize (ví dụ 400x400)
    # img0_cv = cv2.resize(img0, (400, 400))
    # img1_cv = cv2.resize(img1, (400, 400))
    img0_cv=img0
    img1_cv=img1

    # Gọi service
    kp0, kp1, good_matches = call_matcher_service(img0_cv, img1_cv)
    # kp0=kp0*2
    # kp1=kp1*2

    print(f"Number of keypoints in image 0: {len(kp0)}")
    print(f"Number of keypoints in image 1: {len(kp1)}")
    # print(f"Match : {good_matches}")

    kp0_cv = [cv2.KeyPoint(float(x), float(y), 1) for x, y in kp0]
    kp1_cv = [cv2.KeyPoint(float(x), float(y), 1) for x, y in kp1]

    # Vẽ matches
    vis = cv2.drawMatches(
        img0, kp0_cv,
        img1, kp1_cv,
        good_matches,
        None,
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
    )


    out_path = "/mnt/c/WORK/projects/intelijet_v2/intelijet_v2_ws/src/ai_core_pkg/images/matches_vis.png"
    cv2.imwrite(out_path, vis)
    print(f"Saved matches visualization to: {out_path}")


if __name__ == "__main__":
    main()
