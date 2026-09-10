#!/usr/bin/env python3
import rospy
from can_msgs.msg import Frame
from shared.config_loader import CONFIG
cfg = CONFIG
# Mapping COB-ID → topic
COB_ID_MAPPING = {
    "encoder": [0x285],  
    "plc": [0x285]   
}
PCAN_GATEWAY_RECV_TOPIC = "/pcan_received_messanges"

class MessageRouter:
    def __init__(self):
        rospy.init_node("message_router_node")  

        # Subscribe CAN frames từ PCAN Gateway
        self.sub = rospy.Subscriber(PCAN_GATEWAY_RECV_TOPIC, Frame, self.callback)

        # Publishers cho từng nhóm
        self.pub_encoder = rospy.Publisher(cfg.ENCODER01_CAN_MSG, Frame, queue_size=1)
        self.pub_plc = rospy.Publisher(cfg.PLC_HEARTBEAT_MSG, Frame, queue_size=1)
        # self.pub_other  = rospy.Publisher("/can/other", Frame, queue_size=1)

        rospy.loginfo("Message Router Node started")

    def callback(self, msg: Frame):
        cob_id = msg.id

        # Phân loại dựa trên COB-ID - độc lập, không elif, vì một ID có thể
        # thuộc nhiều nhóm (vd: encoder và plc tạm dùng chung 0x285).
        if cob_id in COB_ID_MAPPING["encoder"]:
            self.pub_encoder.publish(msg)
        if cob_id in COB_ID_MAPPING["plc"]:
            self.pub_plc.publish(msg)
        # pub_other chưa được dùng ở đâu - tạm không publish.

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = MessageRouter()
        node.spin()
    except rospy.ROSInterruptException:
        pass
