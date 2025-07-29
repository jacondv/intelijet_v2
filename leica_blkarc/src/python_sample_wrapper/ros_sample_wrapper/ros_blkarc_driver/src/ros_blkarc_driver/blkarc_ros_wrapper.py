import threading
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

import logging
import sys
from dataclasses import dataclass
from typing import Generator, List


import time
import grpc
import numpy as np
import open3d as o3d
import quaternion
from blk_arc_grpc import capture_pb2 as capture_message
from sensor_msgs.msg import PointCloud2
from ros_blkarc_driver.helper import convert_open3d_to_pointcloud2
from blk_arc_grpc import imaging_pb2 as imaging_message
from blk_arc_sample_wrapper.blk_arc_pano_image_decoder import BLK_ARC_PanoImageDecoder as ImageDecoder
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


import actionlib
import rospy
from blk_arc_grpc import device_pb2 as device_message
from ros_blkarc_msgs.msg import (TimedScanAction, TimedScanFeedback, TimedScanGoal, TimedScanResult)
from ros_blkarc_msgs.srv import (Capture, CaptureRequest, CaptureResponse, DownloadScan, DownloadScanRequest,
                                 DownloadScanResponse, GetDeviceState, GetDeviceStateRequest, GetDeviceStateResponse)
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from blk_arc_sample_wrapper.blk_arc import BLK_ARC
from blk_arc_sample_wrapper.blk_arc_config import ConnectionType


def publish_once(topic: str, msg: PointCloud2):
    pub = rospy.Publisher(topic, PointCloud2, queue_size=1, latch=True)
    rospy.sleep(0.5)
    pub.publish(msg)
    pub.unregister()

def publish_image_once(topic: str, msg: Image):
    pub = rospy.Publisher(topic, Image, queue_size=1, latch=True)
    rospy.sleep(0.5)  # Đợi để ROS đảm bảo kết nối với subscriber
    pub.publish(msg)
    pub.unregister()

class ScanDataStreamHandler:

    @dataclass
    class BaserPoint:
        timestamp_ns: int
        # The position is extended by a fourth element 1.0, this allows to directly multiply it with
        # the 4x4 transformation-matrix
        position: np.ndarray    # [x, y, z, 1.0]

        @staticmethod
        def from_proto(point_pb: capture_message.CloudPoint) -> 'ScanDataStreamHandler.BaserPoint':
            timestamp_ns = point_pb.timestamp.seconds * 10**9 + point_pb.timestamp.nanos
            position = np.array([point_pb.u, point_pb.v, point_pb.w, 1.0])
            return ScanDataStreamHandler.BaserPoint(timestamp_ns=timestamp_ns, position=position)

    @dataclass
    class TrajectoryPose:
        timestamp_ns: int
        transformation_matrix: np.ndarray

        @staticmethod
        def from_proto(pose_pb: capture_message.TrajectoryPoint) -> 'ScanDataStreamHandler.TrajectoryPose':
            timestamp_ns = pose_pb.timestamp.seconds * 10**9 + pose_pb.timestamp.nanos
            transformation_matrix = ScanDataStreamHandler.TrajectoryPose._pose_to_transformation_matrix(
                pose_pb.x, pose_pb.y, pose_pb.z, pose_pb.q1, pose_pb.q2, pose_pb.q3, pose_pb.q0)
            return ScanDataStreamHandler.TrajectoryPose(timestamp_ns=timestamp_ns,
                                                        transformation_matrix=transformation_matrix)

        @staticmethod
        def _pose_to_transformation_matrix(x: float, y: float, z: float, qx: float, qy: float, qz: float,
                                           qw: float) -> np.ndarray:
            q = np.quaternion(qw, qx, qy, qz)

            transformation_matrix = np.empty([4, 4], dtype=float)
            transformation_matrix[:3, :3] = quaternion.as_rotation_matrix(q)
            transformation_matrix[:3, -1] = [x, y, z]
            transformation_matrix[-1, :4] = [0.0, 0.0, 0.0, 1.0]

            return transformation_matrix

    def __init__(self):
        self._scan_data_stream: Optional[Generator[capture_message.CaptureStreamMessage, None, None]] = None
        self._low_confidence = False
        self._baser_points: List[ScanDataStreamHandler.BaserPoint] = []
        self._last_trajectory_pose = ScanDataStreamHandler.TrajectoryPose(timestamp_ns=sys.maxsize,
                                                                          transformation_matrix=np.identity(4))
        self._pointcloud = o3d.geometry.PointCloud()
        self._trajectory = o3d.geometry.PointCloud()
        self._stream_handler_thread: Optional[threading.Thread] = None

    
    @property
    def world_points(self) -> o3d.geometry.PointCloud:

        return self._pointcloud
    # DANH-----------------------------------------------------


    @property
    def baser_point(self)-> o3d.geometry.PointCloud:
        _pointcloud = o3d.geometry.PointCloud()
        for i, baser_point in enumerate(self._baser_points):
            # # Ignore points which are older than the last trajectory-pose. During normal operation this should not happen.
            # if baser_point.timestamp_ns < self._last_trajectory_pose.timestamp_ns:
            #     continue
            
            world_point = baser_point.position
            _pointcloud.points.append(world_point[:3])

        return _pointcloud
    # DANH End--------------------------------------------------

    @property
    def trajectory(self) -> o3d.geometry.PointCloud:
        return self._trajectory

    def reset(self) -> None:
        self._low_confidence = False
        self._baser_points.clear()
        self._pointcloud.clear()
        self._trajectory.clear()
        self._last_trajectory_pose = ScanDataStreamHandler.TrajectoryPose(timestamp_ns=sys.maxsize,
                                                                          transformation_matrix=np.identity(4))

    def start(self, scan_data_stream: Generator[capture_message.CaptureStreamMessage, None, None]) -> bool:
        if self._stream_handler_thread is not None and self._stream_handler_thread.is_alive():
            logging.warning("Stream-Handler already started. Stop it first before starting again.")
            return False

        self._scan_data_stream = scan_data_stream
        self.reset()

        self._stream_handler_thread = threading.Thread(target=self._stream_handler,
                                                       name="Stream Handler Thread",
                                                       daemon=False)
        self._stream_handler_thread.start()
        return True

    def stop(self) -> None:
        if self._scan_data_stream is not None:
            self._scan_data_stream.cancel()
        if self._stream_handler_thread is not None and self._stream_handler_thread is not threading.current_thread():
            self._stream_handler_thread.join()

    @property
    def is_running(self) -> bool:
        return self._stream_handler_thread is not None and self._stream_handler_thread.is_alive()

    def _stream_handler(self) -> None:
        assert self._scan_data_stream is not None
        try:
            for scan_data in self._scan_data_stream:
                if scan_data.trajectory:
                    self._process_trajectory_batch(trajectory_batch=scan_data.trajectory.points)

                if scan_data.baser:
                    self._process_baser_batch(baser_batch=scan_data.baser.points)

                # The SLAM-confidence is a metric which can be used to determine the quality of the current SLAM-estimate
                if scan_data.slam_metric and scan_data.slam_metric.key == "confidence":
                    self._check_slam_confidence(float(scan_data.slam_metric.value))
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.CANCELLED:
                logging.info("Scan data stream has been cancelled.")
            else:
                logging.error("GRPC error in scan data stream.")

    def _process_trajectory_batch(self, trajectory_batch: List[capture_message.TrajectoryPointList]) -> None:
        # Don't process any points while the confidence is low
        if self._low_confidence:
            return

        # Go through all trajectory poses and process the baser-points in between the last and current trajectory pose
        for pose_pb in trajectory_batch:
            current_trajectory_pose = ScanDataStreamHandler.TrajectoryPose.from_proto(pose_pb=pose_pb)

            # Always use the closest trajectory-point for the transformation of the pointcloud-points
            # Trajectory poses are streamed with 200Hz. For applications requiring higher accuracy,
            # you might want to consider using nlerp or slerp to accurately interpolate the trajectory
            # at the exact timestamp of the pointcloud-point
            cutoff_timestamp_ns = (current_trajectory_pose.timestamp_ns + self._last_trajectory_pose.timestamp_ns) / 2

            for i, baser_point in enumerate(self._baser_points):
                # Ignore points which are older than the last trajectory-pose. During normal operation this should not happen.
                if baser_point.timestamp_ns < self._last_trajectory_pose.timestamp_ns:
                    continue
                elif baser_point.timestamp_ns < cutoff_timestamp_ns:
                    transformation_matrix = self._last_trajectory_pose.transformation_matrix
                elif baser_point.timestamp_ns <= current_trajectory_pose.timestamp_ns:
                    transformation_matrix = current_trajectory_pose.transformation_matrix
                else:
                    # Remove all processed points from the buffer of baser-points
                    del self._baser_points[:i]
                    break

                world_point = np.dot(transformation_matrix, baser_point.position)
                self._pointcloud.points.append(world_point[:3])

            self._trajectory.points.append(current_trajectory_pose.transformation_matrix[:3, -1])
            self._trajectory.colors.append([1, 0, 0])
            self._last_trajectory_pose = current_trajectory_pose

    def _process_baser_batch(self, baser_batch: List[capture_message.CloudPoint]) -> None:
        # Don't add any points while the confidence is low
        if self._low_confidence:
            return

        for point_pb in baser_batch:
            self._baser_points.append(ScanDataStreamHandler.BaserPoint.from_proto(point_pb))

    def _check_slam_confidence(self, slam_confidence: float) -> None:
        if slam_confidence < 0.25:
            logging.error("Very low SLAM-confidence. Accuracy will likely be degraded. Aborting.")
            self.stop()
        elif slam_confidence < 0.75:
            if not self._low_confidence:
                logging.warning("Low SLAM-confidence detected. Not adding points anymore.")
                self._low_confidence = True
        else:
            if self._low_confidence:
                logging.info("High SLAM-confidence detected. Resuming to add points.")
                self._low_confidence = False

class BLKARCROSWrapper():

    def __init__(self) -> None:

        # Acquire connection type
        connection_type_int = rospy.get_param(param_name="~connection_type", default=0)    # Default is wired connection

        # Map input int to a ConnectionType enum class entry, raise error if an invalid int has been provided
        if connection_type_int == 0:
            self._blk_arc_connection_type = ConnectionType.WIRED
        elif connection_type_int == 1:
            self._blk_arc_connection_type = ConnectionType.WIRELESS
        else:
            connection_type_mapping_msg = ""
            for connection_type_entry in ConnectionType:
                connection_type_mapping_msg += f" {connection_type_entry.name}: {connection_type_entry.value},"
            connection_type_mapping_msg = connection_type_mapping_msg[:-1]

            rospy.logerr(
                f"[BLKARCROSWrapper] The provided ROS param connection_type int is invalid. Possible values are {connection_type_mapping_msg}."
            )
            raise ValueError("Invalid connection_type int provided")

        # Acquire directory where downloaded files will be saved
        scan_save_directory = rospy.get_param(param_name="~scan_save_directory",
                                              default=Path.home())    # Default is the home directory
        self._scan_save_path = Path(scan_save_directory).expanduser(
        )    # Note: expanduser() takes into account that user could use ~ to signify home directory

        # Acquire the name of the sensor (Used to generate namespace for service names)
        self._sensor_name = rospy.get_param(param_name="~sensor_name", default="blkarc")

        # Use a lock to prevent BLK_ARC api from accessed by multiple services/actions at once. Only one service should be performed at any one time.
        self._lock = threading.Lock()

        # With required variables acquired, initialise BLK_ARC API and attempt connection to BLK ARC
        self._blk_arc = BLK_ARC()
        self.connected_successfully = self._attempt_connection()
# DANH ------------------------------------------------------
        self._terminate = False  # dùng để dừng thread monitor connection khi shutdown
        self._connection_monitor_thread = threading.Thread(target=self._monitor_connection, daemon=True)
        self._connection_monitor_thread.start()
# -----------------------------------------------------------
        # If connection is not successful, raise error.
        # Enforce that ROS node should be connected on sensor upon startup
        # This is so user of node is guaranteed that sensor starts up in a connected state
        # And does not need to check this with an additional service call
        connection_type_msg = ConnectionType(self._blk_arc_connection_type).name
        if not self.connected_successfully:
            rospy.logerr(
                f"[BLKARCROSWrapper] Failed to connect to sensor during ROS node initialisation over {connection_type_msg} connection."
            )
            raise RuntimeError("Connection to Device failed during initialisation")
        else:
            # Print status message
            connection_status_msg = f"[BLKARCROSWrapper] Successfully connected to BLK over {connection_type_msg} connection."
            rospy.loginfo(connection_status_msg)

        # Setup ROS Services
        rospy.Service(name=f"{self._sensor_name}/connect", service_class=Trigger, handler=self.handle_connect)
        rospy.Service(name=f"{self._sensor_name}/disconnect", service_class=Trigger, handler=self.handle_disconnect)
        rospy.Service(name=f"{self._sensor_name}/start_capture",
                      service_class=Capture,
                      handler=self.handle_start_capture)
        rospy.Service(name=f"{self._sensor_name}/stop_capture", service_class=Capture, handler=self.handle_stop_capture)
        rospy.Service(name=f"{self._sensor_name}/begin_static_pose",
                      service_class=Trigger,
                      handler=self.handle_begin_static_pose)
        rospy.Service(name=f"{self._sensor_name}/end_static_pose",
                      service_class=Trigger,
                      handler=self.handle_end_static_pose)
        rospy.Service(name=f"{self._sensor_name}/trigger_detail_image",
                      service_class=Trigger,
                      handler=self.handle_trigger_detail_image)
        rospy.Service(name=f"{self._sensor_name}/download_scan",
                      service_class=DownloadScan,
                      handler=self.handle_download_scan)
        rospy.Service(name=f"{self._sensor_name}/get_device_state",
                      service_class=GetDeviceState,
                      handler=self.handle_get_device_state)
        rospy.Service(name=f"{self._sensor_name}/reboot", service_class=Trigger, handler=self.handle_reboot)

        # Setup ROS actions
        self._timed_scan_action_server = actionlib.SimpleActionServer(name=f"{self._sensor_name}/timed_scan",
                                                                      ActionSpec=TimedScanAction,
                                                                      execute_cb=self.timed_scan_cb_v2,
                                                                      auto_start=False)
        self._timed_scan_action_server.start()


# DANH TODO ----------------------------------------
        # self.image_pub = rospy.Publisher(f'{self._sensor_name}/image', Image, queue_size=1)
        self.bridge = CvBridge()
# --------------------------------------------------

    @staticmethod
    def grpc_device_state_to_ros_device_state(grpc_state=Optional[int]) -> int:
        if grpc_state is None:
            return GetDeviceStateResponse.STATE_UNKNOWN
        elif grpc_state == device_message.DeviceStateResponse.State.BOOTING:
            return GetDeviceStateResponse.STATE_BOOTING
        elif grpc_state == device_message.DeviceStateResponse.State.OTA_IN_PROGRESS:
            return GetDeviceStateResponse.STATE_OTA_IN_PROGRESS
        elif grpc_state == device_message.DeviceStateResponse.State.FLASHING_HW_MAIN:
            return GetDeviceStateResponse.STATE_FLASHING_HW_MAIN
        elif grpc_state == device_message.DeviceStateResponse.State.FLASHING_HW_HEAD:
            return GetDeviceStateResponse.STATE_FLASHING_HW_HEAD
        elif grpc_state == device_message.DeviceStateResponse.State.IDLE:
            return GetDeviceStateResponse.STATE_IDLE
        elif grpc_state == device_message.DeviceStateResponse.State.CAPTURE_STARTING:
            return GetDeviceStateResponse.STATE_CAPTURE_STARTING
        elif grpc_state == device_message.DeviceStateResponse.State.CAPTURE_RUNNING:
            return GetDeviceStateResponse.STATE_CAPTURE_RUNNING
        elif grpc_state == device_message.DeviceStateResponse.State.CAPTURE_RUNNING_STATIC:
            return GetDeviceStateResponse.STATE_CAPTURE_RUNNING_STATIC
        elif grpc_state == device_message.DeviceStateResponse.State.CAPTURE_STOPPING:
            return GetDeviceStateResponse.STATE_CAPTURE_STOPPING
        elif grpc_state == device_message.DeviceStateResponse.State.FAULT_BROKEN:
            return GetDeviceStateResponse.STATE_FAULT_BROKEN
        elif grpc_state == device_message.DeviceStateResponse.State.FAULT_DEGRADED:
            return GetDeviceStateResponse.STATE_FAULT_DEGRADED
        elif grpc_state == device_message.DeviceStateResponse.State.FAULT_RECOVERABLE_DEGRADED:
            return GetDeviceStateResponse.STATE_FAULT_RECOVERABLE_DEGRADED
        elif grpc_state == device_message.DeviceStateResponse.State.SHUTDOWN:
            return GetDeviceStateResponse.STATE_SHUTDOWN
        else:
            return GetDeviceStateResponse.STATE_UNKNOWN

    @staticmethod
    def ros_device_state_to_string(ros_device_state: int) -> str:
        if ros_device_state == GetDeviceStateResponse.STATE_UNKNOWN:
            return "UNKNOWN"
        elif ros_device_state == GetDeviceStateResponse.STATE_BOOTING:
            return "BOOTING"
        elif ros_device_state == GetDeviceStateResponse.STATE_OTA_IN_PROGRESS:
            return "OTA_IN_PROGRESS"
        elif ros_device_state == GetDeviceStateResponse.STATE_FLASHING_HW_MAIN:
            return "FLASHING_HW_MAIN"
        elif ros_device_state == GetDeviceStateResponse.STATE_FLASHING_HW_HEAD:
            return "FLASHING_HW_HEAD"
        elif ros_device_state == GetDeviceStateResponse.STATE_IDLE:
            return "IDLE"
        elif ros_device_state == GetDeviceStateResponse.STATE_CAPTURE_STARTING:
            return "CAPTURE_STARTING"
        elif ros_device_state == GetDeviceStateResponse.STATE_CAPTURE_RUNNING:
            return "CAPTURE_RUNNING"
        elif ros_device_state == GetDeviceStateResponse.STATE_CAPTURE_RUNNING_STATIC:
            return "CAPTURE_RUNNING_STATIC"
        elif ros_device_state == GetDeviceStateResponse.STATE_CAPTURE_STOPPING:
            return "CAPTURE_STOPPING"
        elif ros_device_state == GetDeviceStateResponse.STATE_FAULT_BROKEN:
            return "FAULT_BROKEN"
        elif ros_device_state == GetDeviceStateResponse.STATE_FAULT_DEGRADED:
            return "FAULT_DEGRADED"
        elif ros_device_state == GetDeviceStateResponse.STATE_FAULT_RECOVERABLE_DEGRADED:
            return "FAULT_RECOVERABLE_DEGRADED"
        elif ros_device_state == GetDeviceStateResponse.STATE_SHUTDOWN:
            return "SHUTDOWN"
        else:
            return "UNKNOWN"

    def _attempt_connection(self) -> bool:
        connected_successfully = False

        try:
            connected_successfully = self._blk_arc.connect(connection_type=self._blk_arc_connection_type)
        except Exception as error:
            rospy.logerr(f"Failed to connect to BLK sensor {self._sensor_name}. Error: {error}")

        return connected_successfully

    def _scanning_in_progress(self) -> bool:
        device_status = self._blk_arc.get_device_status()
        if device_status:
            return device_status.state in (device_message.DeviceStateResponse.State.CAPTURE_RUNNING,
                                           device_message.DeviceStateResponse.State.CAPTURE_RUNNING_STATIC,
                                           device_message.DeviceStateResponse.State.CAPTURE_STARTING,
                                           device_message.DeviceStateResponse.State.CAPTURE_STOPPING)
        return False
# DANH ---- add monitor function to auto reconnect lidar device

    def _monitor_connection(self):
        while not self._terminate and not rospy.is_shutdown():
            if not self.connected_successfully:
                rospy.logwarn("[BLKARCROSWrapper] Sensor not connected. Attempting reconnection...")
                try:
                    self.connected_successfully = self._attempt_connection()
                    if self.connected_successfully:
                        rospy.loginfo("[BLKARCROSWrapper] Reconnected to sensor successfully.")
                except Exception as e:
                    rospy.logwarn(f"[BLKARCROSWrapper] Reconnection failed: {e}")
            time.sleep(3)

# -------------------------------------------------------------
    def _ready_to_scan(self) -> bool:
        device_status = self._blk_arc.get_device_status()
        if device_status:
            return device_status.state == device_message.DeviceStateResponse.State.IDLE
        return False

    def close(self) -> None:
        rospy.loginfo("[BLKARCROSWrapper] Closing BLK ARC ROS Wrapper.")
# DANH --------------------------------------------------
        self._terminate = True
        self._connection_monitor_thread.join()
# -------------------------------------------------------------
        # If we are scanning, stop the scan
        if self._scanning_in_progress():
            self._blk_arc.stop_capture()

        # If we are connected, disconnect
        if self._blk_arc.is_connected():
            self._blk_arc.disconnect()

    @staticmethod
    def _handle_trigger_task(task_name: str,
                             task: Callable,
                             device_lock: threading.Lock,
                             failure_msg: str = "") -> TriggerResponse:
        # Print acknowledgement that service call received
        rospy.loginfo(f"[BLKARCROSWrapper] {task_name} service call received.")

        # Attempt connection
        with device_lock:
            success = task()

        # Print outcome
        outcome_message = f"BLK executed {task_name} {'successfully' if success else 'unsuccessfully'}."
        outcome_message += "" if success else f" {failure_msg}"
        rospy.loginfo(f"[BLKARCROSWrapper] {outcome_message}")

        return TriggerResponse(success=success, message=outcome_message)

    def handle_connect(self, request: TriggerRequest) -> TriggerResponse:
        return self._handle_trigger_task(task_name="Connect",
                                         task=lambda: self._attempt_connection(),
                                         device_lock=self._lock)

    def handle_disconnect(self, request: TriggerRequest) -> TriggerResponse:
        return self._handle_trigger_task(task_name="Disconnect",
                                         task=lambda: self._blk_arc.disconnect(),
                                         device_lock=self._lock)

    def handle_reboot(self, request: TriggerRequest) -> TriggerResponse:
        return self._handle_trigger_task(task_name="Reboot",
                                         task=lambda: self._blk_arc.reboot(),
                                         device_lock=self._lock)

    def handle_begin_static_pose(self, request: TriggerRequest) -> TriggerResponse:
        return self._handle_trigger_task(task_name="Begin Static Pose",
                                         task=lambda: self._blk_arc.begin_static_pose(),
                                         device_lock=self._lock,
                                         failure_msg="Device is either already in a static pose or not scanning.")

    def handle_end_static_pose(self, request: TriggerRequest) -> TriggerResponse:
        return self._handle_trigger_task(task_name="End Static Pose",
                                         task=lambda: self._blk_arc.end_static_pose(),
                                         device_lock=self._lock,
                                         failure_msg="Device is currently not scanning in a static pose.")

    def handle_trigger_detail_image(self, request: TriggerRequest) -> TriggerResponse:
        return self._handle_trigger_task(
            task_name="Trigger Detail Image",
            task=lambda: self._blk_arc.trigger_detail_image(),
            device_lock=self._lock,
            failure_msg="Ensure that the device is scanning before triggering a detailed image.")

    def handle_start_capture(self, request: CaptureRequest) -> CaptureResponse:
        with self._lock:

            success = False

            # Print acknowledgement that service call received
            rospy.loginfo("[BLKARCROSWrapper] Start capture service call received.")

            # Check if scanner is currently scanning
            ready_to_scan = self._ready_to_scan()

            # If scanner is currently scanning, we do not call start scan again
            response_scan_id = -1    # If start capture fails, scan_id in response is set to -1
            if ready_to_scan:
                start_scan_info = self._blk_arc.start_capture()
                if start_scan_info:
                    success = True
                    response_scan_id = int(start_scan_info.scan_id)

            # Print outcome
            outcome_message = str()

            if not ready_to_scan:
                outcome_message = "Device is in a state where it is not ready to start scan (must be in IDLE state), ignoring start capture service call."
            elif success:
                outcome_message = f"Start capture succeeded. Scan ID: {response_scan_id}"
            else:
                outcome_message = "Start capture failed even though it is ready, please try again."

            rospy.loginfo(f"[BLKARCROSWrapper] {outcome_message}")
            return CaptureResponse(success=success, scan_id=response_scan_id, message=outcome_message)

    def handle_stop_capture(self, request: Capture) -> CaptureResponse:
        with self._lock:

            success = False

            # Print acknowledgement that service call received
            rospy.loginfo("[BLKARCROSWrapper] Stop capture service call received.")

            # Check if scanner is currently scanning
            scanning_in_progress = self._scanning_in_progress()

            # Only call stop scanning if scanner is scanning (or else there is nothing to stop)
            scan_id = -1    # If stop capture fails, scan_id in response is set to -1
            if scanning_in_progress:
                stop_capture_response = self._blk_arc.stop_capture()
                if stop_capture_response:
                    scan_id = int(stop_capture_response.scan_id)
                    success = True

            # Print outcome
            outcome_message = str()
            if not scanning_in_progress:
                outcome_message = "Device is not scanning, ignoring stop capture service call."
            elif success:
                outcome_message = f"Stop capture succeeded. Scan ID: {scan_id}"
            else:
                outcome_message = "Stop capture failed."
            rospy.loginfo(f"[BLKARCROSWrapper] {outcome_message}")

            return CaptureResponse(success=success, scan_id=scan_id, message=outcome_message)

    def handle_download_scan(self, request: DownloadScanRequest) -> DownloadScanResponse:
        with self._lock:

            # Print acknowledgement that service call received
            rospy.loginfo("[BLKARCROSWrapper] Download scan service call received.")

            # If negative scan_id, we look for the scan_id of the latest scan
            download_scan_id = request.scan_id
            if download_scan_id < 0:
                scans = self._blk_arc.list_scans()

                # If there exists at least one scan
                if scans:
                    # Obtain latest last scan (scan with the most recent end time)
                    latest_scan = max(scans,
                                      key=lambda scan: datetime.strptime(scan.metadata.properties['scan/time-finished'],
                                                                         "%Y-%m-%d %H:%M:%S"))
                    download_scan_id = int(latest_scan.metadata.properties['item/id'])

            # If scan_id is valid (non negative), attempt to download the scan and if successful, store its information
            success = False
            if download_scan_id >= 0:
                rospy.loginfo(
                    f"[BLKARCROSWrapper] Attempting to download scan {download_scan_id}. This will take a moment, please wait. Do not disconnect the sensor."
                )
                success = self._blk_arc.download_scan(scan_id=download_scan_id, path=self._scan_save_path)
                if success:
                    downloaded_scan = self._blk_arc.get_scan_info(scan_id=download_scan_id)

            # Process response message
            response_scan_id = download_scan_id if success else -1
            save_directory = str(self._scan_save_path) if success else str()
            filename = f"{download_scan_id}.b2g" if success else str()
            size_bytes = downloaded_scan.size_bytes if success else 0

            # Print outcome
            outcome_message = str()
            if success:
                outcome_message = str(f"Downloaded scan with id {download_scan_id} successfully to: "
                                      f"{save_directory}/{filename}.\n"
                                      f"Scan info:\n"
                                      f"  Size:         {(size_bytes / 1e6):.3f}MB\n"
                                      f"  ID:           {download_scan_id}\n"
                                      f"  Time Started: {downloaded_scan.metadata.properties['scan/time-started']}\n"
                                      f"  Time Stopped: {downloaded_scan.metadata.properties['scan/time-finished']}")
            else:
                outcome_message = f"Download failed for scan {download_scan_id}. Note: If scan session is too short (<10 seconds), scan may not be saved."
            rospy.loginfo(f"[BLKARCROSWrapper] {outcome_message}")

            return DownloadScanResponse(success=success,
                                        scan_id=response_scan_id,
                                        save_directory=save_directory,
                                        filename=filename,
                                        size_bytes=size_bytes,
                                        message=outcome_message)

    def handle_get_device_state(self, request: GetDeviceStateRequest) -> GetDeviceStateResponse:
        # Print acknowledgement that service call received
        rospy.loginfo("[BLKARCROSWrapper] Get device status service call received.")

        # Attempt to obtain device status, process results
        with self._lock:
            grpc_status = self._blk_arc.get_device_status()
        status_received = grpc_status is not None
        grpc_state = grpc_status.state if status_received else None
        ros_device_state = BLKARCROSWrapper.grpc_device_state_to_ros_device_state(grpc_state=grpc_state)
        device_message = f"Additional message from device: {grpc_status.state_details}" if (
            status_received and len(grpc_status.state_details) > 0) else ""

        # Outcome message
        state_string = BLKARCROSWrapper.ros_device_state_to_string(ros_device_state=ros_device_state)
        outcome_message = f"Current device state: {state_string}{f' | {device_message}' if len(device_message) > 0 else ''}"
        rospy.loginfo(f"[BLKARCROSWrapper] {outcome_message}")

        return GetDeviceStateResponse(state=ros_device_state, message=outcome_message)

    def timed_scan_cb(self, goal: TimedScanGoal) -> None:
        with self._lock:

            # Print acknowledgement that action goal received
            rospy.loginfo(f"[BLKARCROSWrapper] Timed scan action called to scan for {goal.scan_time_seconds} seconds")

            # Make sure BLK is connected and not scanning before starting, or else abort
            is_connected = self._blk_arc.is_connected()
            scanning_before_action_called = self._scanning_in_progress()
            proceed_with_action = is_connected and not scanning_before_action_called

            # Start scanning
            start_scan_info = None
            if proceed_with_action:
                start_scan_info = self._blk_arc.start_capture()

                if not start_scan_info:
                    rospy.loginfo("[BLKARCROSWrapper] Start capture for timed scan action failed.")

            else:
                rospy.loginfo(
                    "[BLKARCROSWrapper] BLK either not connected or in the process of scanning, cannot start timed scan action."
                )

            # Wait for scanning if start scan successful
            scan_duration = 0
            preempted = False
            if start_scan_info:
                r = rospy.Rate(1)

                # At every second,
                for t in range(1, goal.scan_time_seconds + 1):

                    # Check if preempt requested (action cancelled), if so break loop
                    if self._timed_scan_action_server.is_preempt_requested():
                        rospy.loginfo("[BLKARCROSWrapper] Timed scan action preempted.")
                        preempted = True
                        break

                    # Sleep for a second
                    r.sleep()
                    scan_duration += 1

                    # Publish feedback
                    self._timed_scan_action_server.publish_feedback(feedback=TimedScanFeedback(
                        time_elapsed_seconds=scan_duration))

                    # Log current time
                    rospy.loginfo(f"[BLKARCROSWrapper] Time elapsed since start scan: {scan_duration} seconds")

                # If duration of scan is less than 10, warn that the final scan may not be stored in device
                if scan_duration < 10:
                    rospy.logwarn(
                        f"[BLKARCROSWrapper] Scanned time duration is less than 10 seconds ({scan_duration} seconds). The final scan may not be stored in the device"
                    )

                # Stop scan
                stop_capture_response = self._blk_arc.stop_capture()
                stop_scan_id = stop_capture_response.scan_id if stop_capture_response else None
                if not stop_capture_response:
                    rospy.loginfo("[BLKARCROSWrapper] Stop capture for timed scan action failed.")

            # Publish results
            success = start_scan_info and stop_scan_id and start_scan_info.scan_id == stop_scan_id
            result = TimedScanResult(success=success, scan_id=int(
                start_scan_info.scan_id)) if success else TimedScanResult(success=False, scan_id=-1)

            # Set state of goal
            if success and not preempted:
                self._timed_scan_action_server.set_succeeded(result=result)
                rospy.loginfo(f"[BLKARCROSWrapper] Timed scan capture succeeded. Scan id: {start_scan_info.scan_id}.")
            elif preempted:
                self._timed_scan_action_server.set_preempted(result=result)
                trailing_message = f" Scan id of interupted capture: {stop_scan_id}" if success else ""
                rospy.loginfo(f"[BLKARCROSWrapper] Timed scan capture preempted/interrupted. {trailing_message}")
            else:
                self._timed_scan_action_server.set_aborted(result=result)
                rospy.loginfo("[BLKARCROSWrapper] Timed scan capture aborted.")

    def timed_scan_cb_v2(self, goal: TimedScanGoal) -> None:
        with self._lock:

            # Print acknowledgement that action goal received
            rospy.loginfo(f"[BLKARCROSWrapper] Timed scan action called to scan for {goal.scan_time_seconds} seconds")

            # Make sure BLK is connected and not scanning before starting, or else abort
            is_connected = self._blk_arc.is_connected()
            scanning_before_action_called = self._scanning_in_progress()
            proceed_with_action = is_connected and not scanning_before_action_called

            # Start scanning
            start_scan_info = None
            if proceed_with_action:
                start_scan_info = self._blk_arc.start_capture()

                if not start_scan_info:
                    rospy.loginfo("[BLKARCROSWrapper] Start capture for timed scan action failed.")

            else:
                rospy.loginfo(
                    "[BLKARCROSWrapper] BLK either not connected or in the process of scanning, cannot start timed scan action."
                )
# ---------------------------------------
            __count = 0
            while not self._blk_arc.is_scanning():
                __count += 1
                rospy.loginfo(f"[BLKARCROSWrapper] Waiting({__count}) for BLK to start scanning...")
                rospy.sleep(1)
            # Start scan data stream
            scan_data_stream = self._blk_arc.start_scan_data_stream(max_point_frequency=1000000, enable_low_latency_trajectory=False)
            # Start handler
            stream_handler = ScanDataStreamHandler()
            stream_handler.start(scan_data_stream=scan_data_stream)

            pano_image_stream = self._blk_arc.stream_pano_images_scanning(count=1)

            for img_id, pano_image in enumerate(pano_image_stream):
                # Decode the received Image either to BGR (decodeToColor) or Grayscale (decodeToGrayscale)

                # Publish to ROS
                if img_id==1:
                    bgr_image = ImageDecoder.decodeToColor(pano_image.data)
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite(f"/mnt/c/work/projects/intelijet_v2/pano_rest_{img_id}_{timestamp}.png", bgr_image)
                    ros_image = self.bridge.cv2_to_imgmsg(bgr_image, encoding="bgr8")
                    ros_image.header.frame_id = "base_link"
                    # self.image_pub.publish(ros_image)``

                    publish_image_once(topic=f"{goal.output_topic}/image",msg=ros_image)
               
# ---------------------------------------

            # Wait for scanning if start scan successful
            scan_duration = 0
            preempted = False
            
            if start_scan_info:
                r = rospy.Rate(1)

                # At every second,
                for t in range(1, goal.scan_time_seconds + 1):

                    # Check if preempt requested (action cancelled), if so break loop
                    if self._timed_scan_action_server.is_preempt_requested():
                        rospy.loginfo("[BLKARCROSWrapper] Timed scan action preempted.")
                        preempted = True
                        break

                    # Sleep for a second
                    r.sleep()
                    scan_duration += 1

                    # Publish feedback
                    self._timed_scan_action_server.publish_feedback(feedback=TimedScanFeedback(
                        time_elapsed_seconds=scan_duration))

                    # Log current time
                    rospy.loginfo(f"[BLKARCROSWrapper] Time elapsed since start scan: {scan_duration} seconds")

                # If duration of scan is less than 10, warn that the final scan may not be stored in device
                if scan_duration < 10:
                    rospy.logwarn(
                        f"[BLKARCROSWrapper] Scanned time duration is less than 10 seconds ({scan_duration} seconds). The final scan may not be stored in the device"
                    )
                            

                # Stop scan
                stop_capture_response = self._blk_arc.stop_capture()
                stop_scan_id = stop_capture_response.scan_id if stop_capture_response else None
                if not stop_capture_response:
                    rospy.loginfo("[BLKARCROSWrapper] Stop capture for timed scan action failed.")

           
            # Publish results
            success = start_scan_info and stop_scan_id and start_scan_info.scan_id == stop_scan_id
            result = TimedScanResult(success=success, scan_id=int(
                start_scan_info.scan_id)) if success else TimedScanResult(success=False, scan_id=-1)

            # Set state of goal
            if success and not preempted:
# TODO DANH---------------------------------------self._baser_points
                # stream_handler.stop()
                # # Lưu point cloud ra file
                # dir = f"/mnt/c/work/projects/intelijet_v2/leica_blkarc/"
                # filename = dir + f"output_{datetime.now().strftime('%Y%m%d_%H%M%S')}.ply"
                # print("Num points:", len(stream_handler.world_points.points))
                # o3d.io.write_point_cloud(filename, stream_handler.world_points)
                # print(filename, " saved successfully.")
                # __msg = convert_open3d_to_pointcloud2(stream_handler.world_points)
                # publish_once(goal.output_topic, __msg)

                # self._timed_scan_action_server.set_succeeded(result=result)
                # rospy.loginfo(f"[BLKARCROSWrapper] Timed scan capture succeeded. Scan id: {start_scan_info.scan_id}.")

                stream_handler.stop()
                # Lưu point cloud ra file
                dir = f"/mnt/c/work/projects/intelijet_v2/leica_blkarc/"
                filename = dir + f"output_{datetime.now().strftime('%Y%m%d_%H%M%S')}.ply"
                # print("Num points:", len(stream_handler.baser_point.points))
                o3d.io.write_point_cloud(filename, stream_handler.baser_point)
                print(filename, " saved successfully.")
                __msg = convert_open3d_to_pointcloud2(stream_handler.baser_point)
                publish_once(goal.output_topic, __msg)

                self._timed_scan_action_server.set_succeeded(result=result)
                rospy.loginfo(f"[BLKARCROSWrapper] Timed scan capture succeeded. Scan id: {start_scan_info.scan_id}.")
# TODO---------------------------------------
            elif preempted:
                stream_handler.stop()
                self._timed_scan_action_server.set_preempted(result=result)
                trailing_message = f" Scan id of interupted capture: {stop_scan_id}" if success else ""
                rospy.loginfo(f"[BLKARCROSWrapper] Timed scan capture preempted/interrupted. {trailing_message}")
            else:
                self._timed_scan_action_server.set_aborted(result=result)
                rospy.loginfo("[BLKARCROSWrapper] Timed scan capture aborted.")

