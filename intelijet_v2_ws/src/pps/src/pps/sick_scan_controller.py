
#!/usr/bin/env python3
# scripts/sick_scan_controller.py
import rospy
from pps.generic_scan_controller import GenericScanController
from std_msgs.msg import String, Int32
from shared.config_loader import CONFIG as cfg
from sensor_msgs.msg import PointCloud2
from laser_assembler.srv import AssembleScans2
from shared.pps_command import PPSCommand

def assemble_cloud_client(start_time, end_time):
    rospy.wait_for_service('assemble_scans2')
    try:
        assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
        resp = assemble_scans(start_time, end_time)
        rospy.logwarn("Got combined cloud with %d points", len(resp.cloud.data))
        return resp.cloud
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None

class SickScanController(GenericScanController):
    def __init__(self):
        super().__init__()

        self.open_housing_cmd = Int32()
        self.open_housing_cmd.data = PPSCommand.PLC_OPEN_HOUSING.value

        self.close_housing_cmd = Int32()
        self.close_housing_cmd.data = PPSCommand.PLC_CLOSE_HOUSING.value

        self.stop_housing_cmd = Int32()
        self.stop_housing_cmd.data = PPSCommand.PLC_PAUSE_HOUSING.value     

      
    def run_workflow(self)->PointCloud2:
        # Send run commant to PLC via ROS Topic. Detail in command_handler.py

        self.cmd_pub.publish(self.open_housing_cmd)

        #  Waiting Scaner housing open around 10 degree to start collect data point from sickscan
        if not self.wait_until_target(target_position_in_degree=cfg.HOUSING_START_POSITION):
            rospy.logwarn("[SickScanController] Encoder not reaching target value on time")
            return None
        
        # Start collect data. 
        self.start_time = rospy.Time.now()


        #  Wait Scaner hosing open to target value
        if not self.wait_until_target(target_position_in_degree=cfg.HOUSING_END_POSITION):
            return None


        # Stop move housing
        self.end_time = rospy.Time.now()


        # Send run commant to PLC via ROS Topic. Detail in command_handler.py
        self.cmd_pub.publish(self.stop_housing_cmd)

        # Wait some second before go back and call assemble cloud service.
        point_cloud = assemble_cloud_client(start_time=self.start_time, end_time=self.end_time)
        rospy.sleep(2)


        # Send back command
        self.cmd_pub.publish(self.close_housing_cmd)


        #  Wait Scaner hosing clouse to target value
        if not self.wait_until_target(target_position_in_degree=cfg.HOUSING_START_POSITION, direction=False):
            return None        
        rospy.sleep(2)
        self.cmd_pub.publish(self.stop_housing_cmd)

        # Call service to assembler pointcloud and publish result to Prescan or PostScan topic...
        
        return point_cloud
    
    
    def reset(self):
        self.cmd_pub.publish(self.stop_housing_cmd)



#  How to use

# controller = SickScanController()

# # Run workflow in separate thread
# import threading
# threading.Thread(target=controller.run_workflow).start()

# # When need to stop:
# controller.stop()