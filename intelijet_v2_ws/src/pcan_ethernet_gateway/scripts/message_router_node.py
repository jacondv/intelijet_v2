#!/usr/bin/env python3
import rospy
from can_msgs.msg import Frame
from shared.config_loader import CONFIG
cfg = CONFIG
# Mapping COB-ID → topic
COB_ID_MAPPING = {
    "encoder": [0x285],  
    "plc": [0x200, 0x201, 0x210]   
}

class MessageRouter:
    def __init__(self):
        rospy.init_node("message_router_node")  

        # Subscribe CAN frames từ PCAN Gateway
        self.sub = rospy.Subscriber("/pcan_received_messanges", Frame, self.callback)

        # Publishers cho từng nhóm
        self.pub_encoder = rospy.Publisher(cfg.ENCODER01_CAN_MSG, Frame, queue_size=10)
        self.pub_other  = rospy.Publisher("/can/other", Frame, queue_size=10)

        rospy.loginfo("Message Router Node started")

    def callback(self, msg: Frame):
        cob_id = msg.id

        # Phân loại dựa trên COB-ID
        if cob_id in COB_ID_MAPPING["encoder"]:
            self.pub_encoder.publish(msg)
        else:
            self.pub_other.publish(msg)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = MessageRouter()
        node.spin()
    except rospy.ROSInterruptException:
        pass
