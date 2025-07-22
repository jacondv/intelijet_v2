from __future__ import annotations

import logging
import sys
import threading
import time
from datetime import datetime
from dataclasses import dataclass
from typing import Generator, List, Optional

import grpc
import numpy as np
import open3d as o3d
import quaternion
from blk_arc_grpc import capture_pb2 as capture_message

from blk_arc_sample_wrapper.blk_arc import BLK_ARC
from blk_arc_sample_wrapper.blk_arc_config import ConnectionType

CONNECTION_TYPE = ConnectionType.WIRELESS

def scan(duration: int = 30):
    # Create BLK ARC instance:
    blk_arc = BLK_ARC()

    # Connect:
    if not blk_arc.connect(CONNECTION_TYPE):
        logging.warning("Could not connect, make sure you selected the right connection-type.")
        return

    # Start Capture:
    logging.info("Starting a scan.")
    blk_arc.start_capture()

    # Wait for the scan to be initialized:
    while not blk_arc.is_scanning():
        time.sleep(5)  # Chờ đợi cho đến khi scan được khởi tạo

    # Start scan data stream
    scan_data_stream = blk_arc.start_scan_data_stream(max_point_frequency=1000000, enable_low_latency_trajectory=False)

    # Start handler
    stream_handler = ScanDataStreamHandler()
    stream_handler.start(scan_data_stream=scan_data_stream)

    logging.info(f"Stream started. Running for {duration} seconds...")

    start_time = time.time()
    RUN_DURATION = duration  # seconds

    while time.time() - start_time < RUN_DURATION:
        time.sleep(1)  # Nhẹ CPU, vẫn thu thập dữ liệu từ stream

    # Dừng stream
    stream_handler.stop()

    # Lưu point cloud ra file
    filename = f"output_{datetime.now().strftime('%Y%m%d_%H%M%S')}.ply"
    print("Num points:", len(stream_handler.world_points.points))

    o3d.io.write_point_cloud(filename, stream_handler.world_points)
    logging.info(f"Saved point cloud to: {filename}")

    # Dừng scan và ngắt kết nối
    blk_arc.stop_capture()
    blk_arc.disconnect()

    logging.info("Scan stopped and device disconnected.")

def run_example():
    # Create BLK ARC instance:
    blk_arc = BLK_ARC()

    # Connect:
    if not blk_arc.connect(CONNECTION_TYPE):
        logging.warning("Could not connect, make sure you selected the right connection-type.")
        return

    # Start Capture:
    logging.info("Starting a scan.")
    blk_arc.start_capture()

    # Wait for the scan to be initialized:
    while not blk_arc.is_scanning():
        time.sleep(0.1)

    # Prepare the visualization window
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # Start scan data stream
    scan_data_stream = blk_arc.start_scan_data_stream(max_point_frequency=1000000, enable_low_latency_trajectory=False)

    # Start handler
    stream_handler = ScanDataStreamHandler()
    stream_handler.start(scan_data_stream=scan_data_stream)

    logging.info("Stream and visualization started. Close the Open3D-window to stop.")

    num_visualized_points = 0
    MIN_POINTS_PER_UPDATE = 1000000

    while True:
        num_world_points = len(stream_handler.world_points.points)
        if num_world_points > num_visualized_points + MIN_POINTS_PER_UPDATE:
            if num_visualized_points == 0:
                vis.add_geometry(stream_handler.world_points)
                vis.add_geometry(stream_handler.trajectory)
            vis.update_geometry(stream_handler.world_points)
            vis.update_geometry(stream_handler.trajectory)
            num_visualized_points = num_world_points

        if not vis.poll_events():
            break
        vis.update_renderer()

        if not stream_handler.is_running and blk_arc.is_scanning():
            blk_arc.stop_capture()
        time.sleep(0.1)

    # Stop stream
    stream_handler.stop()

    # 🟡 Save the point cloud to file here
    filename = f"output_{datetime.now().strftime('%Y%m%d_%H%M%S')}.ply"
    print("Num points:", len(stream_handler.world_points.points))

    o3d.io.write_point_cloud(filename, stream_handler.world_points)
    logging.info(f"Saved point cloud to: {filename}")

    # Stop scanner
    blk_arc.stop_capture()
    blk_arc.disconnect()

    vis.destroy_window()


class ScanDataStreamHandler:

    @dataclass
    class BaserPoint:
        timestamp_ns: int
        # The position is extended by a fourth element 1.0, this allows to directly multiply it with
        # the 4x4 transformation-matrix
        position: np.ndarray    # [x, y, z, 1.0]

        @staticmethod
        def from_proto(point_pb: capture_message.CloudPoint) -> ScanDataStreamHandler.BaserPoint:
            timestamp_ns = point_pb.timestamp.seconds * 10**9 + point_pb.timestamp.nanos
            position = np.array([point_pb.u, point_pb.v, point_pb.w, 1.0])
            return ScanDataStreamHandler.BaserPoint(timestamp_ns=timestamp_ns, position=position)

    @dataclass
    class TrajectoryPose:
        timestamp_ns: int
        transformation_matrix: np.ndarray

        @staticmethod
        def from_proto(pose_pb: capture_message.TrajectoryPoint) -> ScanDataStreamHandler.TrajectoryPose:
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


if __name__ == '__main__':
    FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
    logging.basicConfig(level=logging.INFO, format=FORMAT)
    # run_example()
    scan(45)
