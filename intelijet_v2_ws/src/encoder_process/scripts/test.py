#!/usr/bin/env python3
import rospy
from can_msgs.msg import Frame

def test_encoder_publisher():
    rospy.init_node("test_encoder_publisher", anonymous=False)
    pub = rospy.Publisher("/encoder01/can_msg", Frame, queue_size=10)
    rate = rospy.Rate(50)  # 10 Hz

    rospy.loginfo("Test publisher started. Sending fake CAN frames...")

    position = 1      # Giá trị ban đầu (giả lập raw data)
    step = 1            # Mỗi lần tăng thêm 50 (giống encoder di chuyển)
    max_position = 5000
    direction = 1         # 1: tăng, -1: giảm

    while not rospy.is_shutdown():
        msg = Frame()
        msg.id = 0x123
        msg.is_extended = False
        msg.dlc = 4

        # Chuyển position sang 4 byte
        msg.data = [
            position & 0xFF,
            (position >> 8) & 0xFF,
            (position >> 16) & 0xFF,
            (position >> 24) & 0xFF,
            0, 0, 0, 0  # thêm 4 byte padding
        ]
        msg.dlc = 4  # chỉ định là 4 byte dữ liệu hợp lệ
        rospy.loginfo(f"Publishing CAN frame with position: {position}")
        pub.publish(msg)

        # Cập nhật position
        position += step 
        if position > 1200:
            break
            position= 1  # Reset về giá trị ban đầu nếu vượt quá 1000

        rate.sleep()

if __name__ == "__main__":
    try:
        test_encoder_publisher()
    except rospy.ROSInterruptException:
        pass
