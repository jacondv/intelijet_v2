#!/usr/bin/env python3

# This script receives CAN frames from a PCAN Ethernet Gateway and converts TCP data to CAN msgs Frame, publishes them to a ROS topic.

import rospy
from can_msgs.msg import Frame
# from pcan_ethernet_gateway.pcan_udp_driver import PcanUdpReceiver as PcanReceiver
from pcan_ethernet_gateway.pcan_tcp_driver import PcanTcpReceiver as PcanReceiver

# Cấu hình IP và port
PC_LOCAL_IP = "0.0.0.0"              # Lắng nghe tất cả interface mạng
PCAN_GATEWAY_RECV_PORT  = 55000      # Port Gateway kết nối vào PC, cấu hình trong PCAN Gateway
PCAN_GATEWAY_RECV_TOPIC = "/pcan_received_messanges"  # Topic to publish CAN frames

def main():
    rospy.init_node("pcan_gateway_rx_node")

    receiver = PcanReceiver(listen_ip=PC_LOCAL_IP, listen_port=PCAN_GATEWAY_RECV_PORT)

    pub = rospy.Publisher(PCAN_GATEWAY_RECV_TOPIC, Frame, queue_size=100)

    rospy.loginfo(f"PCAN Gateway RX Node started, listening on {PC_LOCAL_IP}:{PCAN_GATEWAY_RECV_PORT}")

    rate = rospy.Rate(1000)  # 1000 Hz
    while not rospy.is_shutdown():
        # Drain every complete frame currently buffered, not just one, so a
        # burst faster than 1000 Hz can't pile up in receiver._recv_buf.
        frame = receiver.receive_frame()
        while frame:
            pub.publish(frame)
            frame = receiver.receive_frame()
        rate.sleep()

    receiver.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
