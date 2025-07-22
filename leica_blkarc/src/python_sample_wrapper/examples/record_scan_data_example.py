import csv
import logging
import threading
import time
from pathlib import Path
from typing import Generator, List, Optional

import grpc
from blk_arc_grpc import capture_pb2 as capture_message

from blk_arc_sample_wrapper.blk_arc import BLK_ARC
from blk_arc_sample_wrapper.blk_arc_config import ConnectionType

CONNECTION_TYPE = ConnectionType.WIRELESS


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
        time.sleep(0.5)

    # Start the scan-data-stream (only possible with a running scan)
    # params:
    #   - max_point_frequency = max number of points streamed per second
    #   - enable_low_latency_trajectory = set to True to use the low latency trajectory
    scan_data_stream = blk_arc.start_scan_data_stream(max_point_frequency=10000, enable_low_latency_trajectory=False)

    # Process the stream with the ScanDataStreamHandler, as implemented below
    stream_recorder = ScanDataStreamRecorder()
    stream_recorder.start(scan_data_stream=scan_data_stream)

    input("Hit <Enter> to stop the recording.")

    stream_recorder.stop()

    # Stop the scan:
    blk_arc.stop_capture()
    blk_arc.disconnect()


class ScanDataStreamRecorder:

    def __init__(self):
        self._scan_data_stream: Optional[Generator[capture_message.CaptureStreamMessage, None, None]] = None
        self._stream_recorder_thread: Optional[threading.Thread] = None
        self._local_points_csv_file = Path()
        self._trajectory_csv_file = Path()

    def start(self, scan_data_stream: Generator[capture_message.CaptureStreamMessage, None, None]) -> bool:
        if self._stream_recorder_thread is not None and self._stream_recorder_thread.is_alive():
            logging.warning("Stream-Recorder already started. Stop it first before starting again.")
            return False

        self._scan_data_stream = scan_data_stream

        start_time = time.strftime("%Y%m%d-%H%M%S")
        self._local_points_csv_file = Path(f"scan_data/{start_time}_local_points.csv")
        self._trajectory_csv_file = Path(f"scan_data/{start_time}_trajectory_poses.csv")

        self._local_points_csv_file.parent.mkdir(parents=True, exist_ok=True)
        self._trajectory_csv_file.parent.mkdir(parents=True, exist_ok=True)

        self._write_headers()

        self._stream_recorder_thread = threading.Thread(target=self._stream_recorder,
                                                        name="Stream Handler Thread",
                                                        daemon=False)
        self._stream_recorder_thread.start()
        return True

    def stop(self) -> None:
        if self._scan_data_stream is not None:
            self._scan_data_stream.cancel()
        if self._stream_recorder_thread is not None and self._stream_recorder_thread is not threading.current_thread():
            self._stream_recorder_thread.join()

    def _stream_recorder(self) -> None:
        if self._scan_data_stream is None:
            logging.error("Scan-data-stream not available. Can't start to record, aborting.")
            return
        try:
            for scan_data in self._scan_data_stream:
                if scan_data.trajectory:
                    self._record_trajectory_batch(trajectory_batch=scan_data.trajectory.points)
                if scan_data.baser:
                    self._record_baser_batch(baser_batch=scan_data.baser.points)
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.CANCELLED:
                logging.info("Scan data stream has been cancelled.")
            else:
                logging.error("GRPC error in scan data stream.")

    def _record_trajectory_batch(self, trajectory_batch: List[capture_message.TrajectoryPointList]) -> None:
        # Go through all trajectory poses and process the baser-points in between the last and current trajectory pose
        for pose_pb in trajectory_batch:
            timestamp_captured_ns = pose_pb.timestamp.seconds * 10**9 + pose_pb.timestamp.nanos
            self._write_line(file_path=self._trajectory_csv_file,
                             entries=[
                                 time.time_ns(), timestamp_captured_ns, pose_pb.x, pose_pb.y, pose_pb.z, pose_pb.q1,
                                 pose_pb.q2, pose_pb.q3, pose_pb.q0
                             ])

    def _record_baser_batch(self, baser_batch: List[capture_message.CloudPoint]) -> None:
        for point_pb in baser_batch:
            timestamp_captured_ns = point_pb.timestamp.seconds * 10**9 + point_pb.timestamp.nanos
            self._write_line(
                file_path=self._local_points_csv_file,
                entries=[time.time_ns(), timestamp_captured_ns, point_pb.u, point_pb.v, point_pb.w, point_pb.intensity])

    def _write_headers(self) -> None:
        self._write_line(
            file_path=self._local_points_csv_file,
            entries=["timestamp_received_ns", "timestamp_captured_ns", "x_local", "y_local", "z_local", "intensity"],
            overwrite_file=True)
        self._write_line(file_path=self._trajectory_csv_file,
                         entries=[
                             "timestamp_received_ns", "timestamp_captured_ns", "x_global", "y_global", "z_global",
                             "q_x", "q_y", "q_z", "q_w"
                         ],
                         overwrite_file=True)

    @staticmethod
    def _write_line(file_path: Path, entries: List[str], overwrite_file: bool = False) -> None:
        write_mode = "w" if overwrite_file else "a"
        with file_path.open(write_mode) as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(entries)


if __name__ == '__main__':
    FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
    logging.basicConfig(level=logging.INFO, format=FORMAT)
    run_example()
