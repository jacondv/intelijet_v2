
#!/usr/bin/env python3
# scripts/sick_scan_controller.py
import rospy
from pps.generic_scan_controller import GenericScanController
from std_msgs.msg import String, Int32
from sensor_msgs.msg import PointCloud2
from laser_assembler.srv import AssembleScans2
from shared.config_loader import CONFIG as cfg
from shared.log_status import log_status
from shared.msg import DeviceStatus

def assemble_cloud_client(start_time, end_time):
    #rospy.wait_for_service('assemble_scans2')
    try:
        assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
        resp = assemble_scans(start_time, end_time)
        return resp.cloud
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None

class SickScanErobController(GenericScanController):
    def __init__(self, status_callback=None):
        super().__init__(status_callback=status_callback)

    def run_workflow(self, publisher=None) -> PointCloud2:
        #This function run the workflow of scanning process and return cloud data
        topic_name = publisher.name if publisher else "Unknown"
        log_status(name=cfg.NOTIFICATION,
                   message=f"[INFO] Starting {'Pre-Scan' if topic_name==cfg.PRE_SCAN_TOPIC else 'Post-Scan'}")

        # ---- Start collect data ----
        self.start_time = rospy.Time.now()
        self.end_time = rospy.Time.now()  # will update at the end

        # ---- Open housing in steps ----
        if not self._open_housing_sequence():
            if self.status_callback:
                if topic_name == cfg.PRE_SCAN_TOPIC:
                    self.status_callback(DeviceStatus.PRESCAN_ERROR)
                else:
                    self.status_callback(DeviceStatus.POSTSCAN_ERROR)
            return None
        
        # ---- Stop housing movement before assemble ----
        self.end_time = rospy.Time.now()
        self.housing.stop()

        # ---- Assemble point cloud ----
        point_cloud = assemble_cloud_client(start_time=self.start_time, end_time=self.end_time)
        rospy.sleep(2)

        if point_cloud and publisher:
            publisher.publish(point_cloud)
            point_cloud = None
            log_status(name=cfg.NOTIFICATION, message="[INFO] Scan completed")

        # ---- Close housing back ----
        if not self._close_housing_sequence():
            if self.status_callback:
                if topic_name == cfg.PRE_SCAN_TOPIC:
                    self.status_callback(DeviceStatus.PRESCAN_ERROR)
                else:
                    self.status_callback(DeviceStatus.POSTSCAN_ERROR)

            return point_cloud  # vẫn trả về cloud nếu có
        
        if self.status_callback:
            self.status_callback(DeviceStatus.IDLE)

        return point_cloud

    def reset(self):
        """Stop housing in case of emergency or end"""
        self.housing.stop()

    # ----------------- Helper methods -----------------
    def _open_housing_sequence(self):
        """Open housing gradually: fast -> medium -> slow, check encoder each step"""
        # speed, target, timeout (giây)
        speeds_targets = [
            # ('fast', cfg.housing_start_position, 10.0),
            # ('medium', cfg.housing_start_position+1, 10.0),
            ('slow', cfg.housing_end_position, 150.0)
        ]

        for speed, target, timeout in speeds_targets:
            self.housing.open(speed)
            if not self.wait_until_target(target, direction=True, timeout=timeout):
                log_status(name=cfg.NOTIFICATION,
                        message=f"[WARN] Encoder did not reach target {target}° for speed {speed} after {timeout}s")
                self.housing.stop()
                return False
        return True


    def _close_housing_sequence(self):
        """Close housing gradually back to start position with step-specific speed and timeout"""
        # speed, target, timeout (giây)
        speeds_targets = [
                        # ('fast', cfg.housing_start_position+50.0, 35.0), 
                        #   ('medium', cfg.housing_start_position+30, 35.0),
                          ('slow', cfg.housing_start_position-0.0, 30)
                          ]

        for speed, target, timeout in speeds_targets:
            self.housing.close(speed)
            if not self.wait_until_target(target, direction=False, timeout=timeout):
                if speed == 'slow':
                    return True  # cho phép không đạt chính xác vị trí start khi đóng chậm
                
                log_status(name=cfg.NOTIFICATION,
                        message=f"[WARN] Encoder did not reach target {target}° while closing at speed {speed}")
                self.housing.stop()     
                return False

        # rospy.sleep(2)
        self.housing.stop()
        return True

