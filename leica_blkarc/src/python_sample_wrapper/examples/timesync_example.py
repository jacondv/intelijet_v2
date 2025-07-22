import logging
import time
from datetime import datetime
from typing import Optional

import blk_arc_grpc.clock_pb2 as clock_message
from google.protobuf import duration_pb2 as duration_message

from blk_arc_sample_wrapper.blk_arc import BLK_ARC
from blk_arc_sample_wrapper.blk_arc_config import ConnectionType

CONNECTION_TYPE = ConnectionType.WIRED


def run_example():
    # Create BLK ARC instance:
    blk_arc = BLK_ARC()
    timesync = BLKARC_Timesync(blk_arc)

    # Connect:
    if not blk_arc.connect(CONNECTION_TYPE):
        logging.warning("Could not connect, make sure you selected the right connection-type.")
        return
    logging.info("Established a connection.")

    # Start Capture:
    logging.info("Starting a scan and scanning for 10 seconds.")
    start_scan_info = blk_arc.start_capture()
    if not start_scan_info:
        logging.error("Could not start a scan.")
        return

    # Get the start time of the scan relative to the head assembly time
    starting_time = start_scan_info.starting_time
    on_device_capture_start_timestamp_ns = int(starting_time.seconds * 1e9) + starting_time.nanos
    on_device_capture_start_time = datetime.fromtimestamp(on_device_capture_start_timestamp_ns /
                                                          1e9).strftime('%H:%M:%S')

    # Transform the start time to the local computer time
    computer_capture_start_timestamp_ns = timesync.get_computer_time_ns_from_scanner_system_time_ns(
        on_device_capture_start_timestamp_ns)
    computer_capture_start_time = datetime.fromtimestamp(computer_capture_start_timestamp_ns / 1e9).strftime('%H:%M:%S')

    # Scan for a while
    time.sleep(10)

    # Get the current scanner system time
    scanner_system_timestamp_ns = timesync.get_scanner_system_time_ns()
    scanner_system_time = datetime.fromtimestamp(scanner_system_timestamp_ns / 1e9).strftime('%H:%M:%S')

    logging.info("-" * 60)
    logging.info(f"Current scanner system time: {scanner_system_time}")
    logging.info(f"The scan was started at:     {on_device_capture_start_time} (in scanner system time)")
    logging.info(f"Current computer time:       {datetime.now().strftime('%H:%M:%S')}")
    logging.info(f"The scan was started at:     {computer_capture_start_time} (in local computer time)")

    blk_arc.stop_capture()
    blk_arc.disconnect()


class BLKARC_Timesync:

    def __init__(self, blk_arc: BLK_ARC):
        self._blk_arc = blk_arc

        self._synced: bool = False
        self._time_of_last_sync_ns: int = 0

        self._scanner_system_clock_offset_ns: int = 0
        self._scanner_main_fpga_clock_offset_ns: int = 0
        self._scanner_head_assembly_clock_offset_ns: int = 0

    def perform_timesync(self) -> bool:
        MAX_TRIES = 5
        MAX_NETWORKING_DELAY = 0.1e9

        for _ in range(MAX_TRIES):
            computer_time_ns = time.time_ns()

            grpc_call_start_time_ns = time.perf_counter_ns()
            times_proto: clock_message.TimestampsResponse = self._blk_arc.get_time_from_clock_sources()
            grpc_call_end_time_ns = time.perf_counter_ns()

            networking_delay_ns = grpc_call_end_time_ns - grpc_call_start_time_ns
            if times_proto is None:
                logging.warning("Could not get the times-proto for time-sync. Trying again.")
                continue
            if networking_delay_ns > MAX_NETWORKING_DELAY:
                logging.warning(f"Networking delay of {networking_delay_ns}ns above threshold "
                                f"of {MAX_NETWORKING_DELAY}ns. Trying again.")
                continue
            computer_time_ns += networking_delay_ns / 2

            self._scanner_system_clock_offset_ns = \
                computer_time_ns - self._duration_proto_to_timestamp_ns(times_proto.system_time)
            self._scanner_main_fpga_clock_offset_ns = \
                computer_time_ns - self._duration_proto_to_timestamp_ns(times_proto.main_fpga)
            self._scanner_head_assembly_clock_offset_ns = \
                computer_time_ns - self._duration_proto_to_timestamp_ns(times_proto.head_assembly_clock)

            self._time_of_last_sync_ns = computer_time_ns
            self._synced = True
            logging.info("Synchronized the times.")
            return True
        logging.error(f"Could not perform time-sync {MAX_TRIES} times, aborting.")
        return False

    def get_scanner_system_time_ns(self) -> Optional[int]:
        if not self._synced:
            if not self.perform_timesync():
                logging.error("Could not get scanner system time as time-sync could not be established.")
                return None
        return time.time_ns() - self._scanner_system_clock_offset_ns

    def get_computer_time_ns_from_scanner_system_time_ns(self, scanner_system_time_ns: int) -> Optional[int]:
        if not self._synced:
            if not self.perform_timesync():
                logging.error("Could not get scanner system time as time-sync could not be established.")
                return None
        return scanner_system_time_ns + self._scanner_system_clock_offset_ns

    def get_computer_time_ns_from_scanner_main_fpga_clock(self, main_fpga_clock_time_ns: int) -> Optional[int]:
        if not self._synced:
            if not self.perform_timesync():
                logging.error("Could not get scanner system time as time-sync could not be established.")
                return None
        return main_fpga_clock_time_ns + self._scanner_main_fpga_clock_offset_ns

    def get_computer_time_ns_from_scanner_head_assembly_clock(self, head_assembly_clock_time_ns: int) -> Optional[int]:
        if not self._synced:
            if not self.perform_timesync():
                logging.error("Could not get scanner system time as time-sync could not be established.")
                return None
        return head_assembly_clock_time_ns + self._scanner_head_assembly_clock_offset_ns

    def get_computer_time_ns_of_last_sync(self) -> int:
        if not self._synced:
            logging.warning("Times are not yet synced. Returning 0.")
        return self._time_of_last_sync_ns

    @staticmethod
    def _duration_proto_to_timestamp_ns(duration_proto: duration_message.Duration) -> int:
        return int(duration_proto.seconds * 1e9) + duration_proto.nanos


if __name__ == '__main__':
    FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
    logging.basicConfig(level=logging.INFO, format=FORMAT)
    run_example()
