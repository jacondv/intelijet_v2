
#!/usr/bin/env python3
# scripts/sick_scan_controller.py
import rospy
from pps.generic_scan_controller import GenericScanController
from std_msgs.msg import String, Int32
from sensor_msgs.msg import PointCloud2
from laser_assembler.srv import AssembleScans2
from shared.config_loader import CONFIG as cfg
from shared.log_status import log_status

def assemble_cloud_client(start_time, end_time):
    #rospy.wait_for_service('assemble_scans2')
    try:
        assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
        resp = assemble_scans(start_time, end_time)
        return resp.cloud
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None

class SickScanController(GenericScanController):
    def __init__(self):
        super().__init__()

    def run_workflow(self,publisher=None)->PointCloud2:
        rospy.loginfo("run_workflow Open")
        # Send run commant to PLC via ROS Topic. Detail in command_handler.py
        self.housing.open('fast')
        log_status(
                name=cfg.NOTIFICATION, 
                status=None, 
                value=None, 
                message="[INFO] Scanning...", 
                node=None
            )
        #  Waiting Scaner housing open around 10 degree to start collect data point from sickscan
        if not self.wait_until_target(target_position_in_degree=cfg.housing_start_position,timeout=5.0, direction=True):
            log_status(
                name=cfg.NOTIFICATION, 
                status=None, 
                value=None, 
                message="[WARN] Encoder not reaching target value on time", 
                node=None
            )    
            self.housing.stop()
            return None
        
        self.housing.open('medium')

        # Start collect data. 
        self.start_time = rospy.Time.now()

        #  Wait Scaner hosing open to target value
        if not self.wait_until_target(target_position_in_degree=30.0, direction=True):
            log_status(
                name=cfg.NOTIFICATION, 
                status=None, 
                value=None, 
                message="[WARN] Encoder not reaching target value on time", 
                node=None
            )            
            self.housing.stop()
            return None

        self.housing.open('slow')

        # Start collect data. 
        self.start_time = rospy.Time.now()

        #  Wait Scaner hosing open to target value
        if not self.wait_until_target(target_position_in_degree=cfg.housing_end_position, direction=True):
            log_status(
                name=cfg.NOTIFICATION, 
                status=None, 
                value=None, 
                message="[WARN] Encoder not reaching target value on time", 
                node=None
            )            
            return None
        

        # Stop move housing
        self.end_time = rospy.Time.now()

        # Send run commant to PLC via ROS Topic.
        self.housing.stop()

        # Wait some second before go back and call assemble cloud service.
        
        point_cloud = assemble_cloud_client(start_time=self.start_time, end_time=self.end_time)
        rospy.sleep(2)

        if point_cloud is not None:
            publisher.publish(point_cloud)
            log_status(
                name=cfg.NOTIFICATION, 
                status=None, 
                value=None, 
                message="[INFO] Scan completed", 
                node=None
            )

        # Send back command
        self.housing.close('fast')

        #  Wait Scaner hosing clouse to target value
        if not self.wait_until_target(target_position_in_degree=cfg.housing_start_position, direction=False):
            log_status(
                name=cfg.NOTIFICATION, 
                status=None, 
                value=None, 
                message="[WARN] Encoder not reaching target value on time", 
                node=None
            )
            self.housing.stop()
            return None        
        
        rospy.sleep(2)
        self.housing.stop()
        log_status(
                name=cfg.NOTIFICATION, 
                status=None, 
                value=None, 
                message="[INFO] Done", 
                node=None
            )

        # Call service to assembler pointcloud and publish result to Prescan or PostScan topic...
        
        return point_cloud
    
    
    def reset(self):
        self.housing.stop()



#  How to use

# controller = SickScanController()

# # Run workflow in separate thread
# import threading
# threading.Thread(target=controller.run_workflow).start()

# # When need to stop:
# controller.stop()
