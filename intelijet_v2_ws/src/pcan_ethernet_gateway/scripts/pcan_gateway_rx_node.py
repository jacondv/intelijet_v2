#!/usr/bin/env python3

# This script receives CAN frames from a PCAN Ethernet Gateway and convert UDP data to CAN msgs Frame publishes them to a ROS topic.

import rospy
from can_msgs.msg import Frame
from pcan_ethernet_gateway.pcan_udp_driver import PcanUdpReceiver

# Cấu hình IP và port
PC_LOCAL_IP = "0.0.0.0"           # Lắng nghe tất cả interface mạng
PCAN_GATEWAY_RECV_PORT  = 6000       # Port Gateway gửi về PC, Port này sẽ được cấu hình trong PCAN Gateway
PCAN_GATEWAY_RECV_TOPIC = "/pcan_received_messanges"  # Topic to publish CAN frames

def main():
    rospy.init_node("pcan_gateway_rx_node")

    receiver = PcanUdpReceiver(listen_ip=PC_LOCAL_IP, listen_port=PCAN_GATEWAY_RECV_PORT )
    pub = rospy.Publisher(PCAN_GATEWAY_RECV_TOPIC, Frame, queue_size=10)

    rospy.loginfo(f"PCAN Gateway RX Node started, listening on {PC_LOCAL_IP}:{PCAN_GATEWAY_RECV_PORT }")

    rate = rospy.Rate(50)  # 200 Hz
    while not rospy.is_shutdown():
        frame = receiver.receive_frame()
        if frame:
            pub.publish(frame)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
