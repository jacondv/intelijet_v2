# command_handler.py
import rospy
from can_msgs.msg import Frame
from shared.pps_command import PPSCommand


class CommandHandler:
    def __init__(self, pdos, can_topic_to_send):
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
        rospy.logwarn(f"Published CAN frame for PDO '{pdo_name}'.")

    def execute_command(self, command):
        # cmd = command.strip().lower()
        cmd = command
        
        if cmd == PPSCommand.PLC_START_PRESCAN.value:
            pdo_name= "RxScannerCommandRos"
            field_name="bPerformPreScan"
            self.send_pdo(pdo_name, field_name, value=1)
            rospy.sleep(0.5)
            self.send_pdo(pdo_name, field_name, value=0)
            return True

        elif cmd == PPSCommand.PCL_START_POSTSCAN.value:
            pdo_name= "RxScannerCommandRos"
            field_name="bPerformPostScan"
            self.send_pdo(pdo_name, field_name, value=1)
            rospy.sleep(0.5)
            self.send_pdo(pdo_name, field_name, value=0)
            return True


        elif cmd == "set_zero_position":
            pdo_name= "RxZeroCommand"
            field_name="bPerformPostScan"
            self.send_pdo(pdo_name, field_name, value=1)
            rospy.sleep(0.5)
            self.send_pdo(pdo_name, field_name, value=0)
            return True
        
        elif cmd == PPSCommand.PLC_OPEN_HOUSING.value:
            pdo_name= "RxScannerCommandRos"
            field_name="bServiceOpenScanner"
            self.send_pdo(pdo_name, field_name, value=1)
            return True
        
        elif cmd == PPSCommand.PLC_CLOSE_HOUSING.value:
            pdo_name= "RxScannerCommandRos"
            field_name="bServiceCloseScanner"
            self.send_pdo(pdo_name, field_name, value=1)
            return True
        
        elif cmd == PPSCommand.PLC_PAUSE_HOUSING.value:
            pdo_name= "RxScannerCommandRos"
            self.send_pdo(pdo_name, "bServiceOpenScanner", value=0)
            self.send_pdo(pdo_name, "bServiceCloseScanner", value=0)
            return True
        
        elif cmd == "plc_on_cancel":
            pdo_name= "RxScannerCommandRos"
            self.send_pdo(pdo_name, "bServiceOpenScanner", value=0)
            self.send_pdo(pdo_name, "bServiceCloseScanner", value=0)
            return True
                
        else:
            return False
