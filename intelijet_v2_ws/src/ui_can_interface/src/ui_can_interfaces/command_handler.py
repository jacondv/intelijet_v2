# command_handler.py
import rospy
from can_msgs.msg import Frame

class CommandHandler:
    def __init__(self, pdos, can_topic_to_send="sent_messages"):
        self.pdos = pdos
        self.can_pub = rospy.Publisher(can_topic_to_send, Frame, queue_size=10)

    def __set_value(self, pdo_name, field_name, value):
        if pdo_name not in self.pdos:
            raise ValueError(f"PDO '{pdo_name}' not found.")
        if field_name not in self.pdos[pdo_name].fields:
            raise ValueError(f"Field '{field_name}' not found in PDO '{pdo_name}'.")
        self.pdos[pdo_name].set_field(field_name, value)

    def send_pdo(self, pdo_name, field_name, value):
        self.__set_value(pdo_name, field_name, value)
        can_frame = self.pdos[pdo_name].to_frame()
        self.can_pub.publish(can_frame)
        # rospy.loginfo(f"Published CAN frame for PDO '{pdo_name}'.")

    def execute_command(self, command):
        cmd = command.strip().lower()
        
        if cmd == "start_prescan":
            pdo_name= "RxScannerCommandRos"
            field_name="bPerformPreScan"
            self.send_pdo(pdo_name, field_name, value=1)
            rospy.sleep(0.5)
            self.send_pdo(pdo_name, field_name, value=0)
            return True

        elif cmd == "start_postscan":
            pdo_name= "RxScannerCommandRos"
            field_name="bPerformPostScan"
            self.send_pdo(pdo_name, field_name, value=1)
            rospy.sleep(0.5)
            self.send_pdo(pdo_name, field_name, value=0)
            return True


        elif cmd == "set_zero_position":
            pdo_name= "RxZeroCommand"
            field_name="bPerformPostScan"
            self.send_pdo(pdo_name, field_name, value=2)
            rospy.sleep(0.5)
            self.send_pdo(pdo_name, field_name, value=0)
            return True
        else:
            return False
