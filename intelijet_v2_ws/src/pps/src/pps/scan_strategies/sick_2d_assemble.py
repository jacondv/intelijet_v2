# pps/scan_strategies/sick_2d_assemble.py
"""Current SickScan acquisition method: rotate the housing through a
step sequence while the 2D LiDAR streams continuously, then ask
laser_assembler to stitch everything seen during that time window into
one 3D cloud. Moved verbatim out of sick_scan_eRob_controller.py -
see docs/plan/phase_07_scanner_abstraction.md for the acquire()<->
run_workflow() correspondence.
"""
import rospy
from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2

from shared.config_loader import CONFIG as cfg
from shared.notify import notify
from shared.msg import DeviceStatus

from pps.scan_strategies.base import ScanStrategy


def assemble_cloud_client(start_time, end_time):
    #rospy.wait_for_service('assemble_scans2')
    try:
        assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
        resp = assemble_scans(start_time, end_time)
        return resp.cloud
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None


class Sick2DAssembleStrategy(ScanStrategy):

    def acquire(self, controller, publisher=None) -> PointCloud2:
        #This function run the workflow of scanning process and return cloud data
        topic_name = publisher.name if publisher else "Unknown"
        notify(message=f"[INFO] Starting {'Pre-Scan' if topic_name==cfg.PRE_SCAN_TOPIC else 'Post-Scan'}")

        # ---- Start collect data ----
        start_time = rospy.Time.now()
        end_time = rospy.Time.now()  # will update at the end

        # ---- Open housing in steps ----
        if not self._open_housing_sequence(controller):
            if controller.status_callback:
                if topic_name == cfg.PRE_SCAN_TOPIC:
                    controller.status_callback(DeviceStatus.PRESCAN_ERROR)
                else:
                    controller.status_callback(DeviceStatus.POSTSCAN_ERROR)
            return None

        # ---- Stop housing movement before assemble ----
        end_time = rospy.Time.now()
        controller.housing.stop()

        # ---- Assemble point cloud ----
        point_cloud = assemble_cloud_client(start_time=start_time, end_time=end_time)
        rospy.sleep(2)

        assemble_failed = point_cloud is None
        if assemble_failed:
            notify(
                message="[ERROR] Failed to assemble point cloud (assemble_scans2 "
                        "service call failed or returned no data)",
                level="error",
            )
            if controller.status_callback:
                if topic_name == cfg.PRE_SCAN_TOPIC:
                    controller.status_callback(DeviceStatus.PRESCAN_ERROR)
                else:
                    controller.status_callback(DeviceStatus.POSTSCAN_ERROR)
        elif publisher:
            publisher.publish(point_cloud)
            point_cloud = None
            notify(message="[INFO] Scan completed")

        # ---- Close housing back ----
        if not self._close_housing_sequence(controller):
            if controller.status_callback:
                if topic_name == cfg.PRE_SCAN_TOPIC:
                    controller.status_callback(DeviceStatus.PRESCAN_ERROR)
                else:
                    controller.status_callback(DeviceStatus.POSTSCAN_ERROR)

            return point_cloud  # vẫn trả về cloud nếu có

        # Don't let a successful housing close overwrite the *_ERROR status
        # just set above when assemble failed - otherwise the error
        # disappears from the UI as soon as the housing finishes closing.
        if controller.status_callback and not assemble_failed:
            controller.status_callback(DeviceStatus.IDLE)

        return point_cloud

    # ----------------- Helper methods -----------------
    def _open_housing_sequence(self, controller):
        """Open housing gradually: fast -> medium -> slow, check encoder each step"""
        # speed, target, timeout (giây)
        speeds_targets = [
            # ('fast', cfg.housing_start_position, 10.0),
            # ('medium', cfg.housing_start_position+1, 10.0),
            ('slow', cfg.housing_end_position, 150.0)
        ]

        for speed, target, timeout in speeds_targets:
            controller.housing.open(speed)
            if not controller.wait_until_target(target, direction=True, timeout=timeout):
                notify(message=f"[WARN] Encoder did not reach target {target}° for speed {speed} after {timeout}s")
                controller.housing.stop()
                return False
        return True

    def _close_housing_sequence(self, controller):
        """Close housing gradually back to start position with step-specific speed and timeout"""
        # speed, target, timeout (giây)
        speeds_targets = [
                        # ('fast', cfg.housing_start_position+50.0, 35.0),
                        #   ('medium', cfg.housing_start_position+30, 35.0),
                          ('slow', max(cfg.housing_start_position, 1), 30)
                          ]

        for speed, target, timeout in speeds_targets:
            controller.housing.close(speed)
            if not controller.wait_until_target(target, direction=False, timeout=timeout):
                if speed == 'slow':
                    return True  # cho phép không đạt chính xác vị trí start khi đóng chậm

                notify(message=f"[WARN] Encoder did not reach target {target}° while closing at speed {speed}")
                controller.housing.stop()
                return False

        # rospy.sleep(2)
        controller.housing.stop()
        return True
