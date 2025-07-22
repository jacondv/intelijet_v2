import logging
import time

from blk_arc_sample_wrapper.blk_arc import BLK_ARC
from blk_arc_sample_wrapper.blk_arc_config import ConnectionType

CONNECTION_TYPE = ConnectionType.WIRED


def run_example():
    # Create BLK ARC instance:
    blk_arc = BLK_ARC()

    # Connect:
    if not blk_arc.connect(CONNECTION_TYPE):
        logging.warning("Could not connect, make sure you selected the right connection-type.")
        return

    # Start Capture:
    logging.info("Starting a scan.")
    start_scan_info = blk_arc.start_capture()
    if start_scan_info:
        # Wait for the scan to be initialized:
        while not blk_arc.is_scanning():
            time.sleep(0.5)
        logging.info(f"Initialized scan with ID: {start_scan_info.scan_id}.")

        # Mark a static pose:
        input("Hit <Enter> to mark the beginning of a static pose. Only use this when the scanner is stationary.")
        logging.info("Marking the beginning of a static pose.")
        blk_arc.begin_static_pose()

        input("Hit <Enter> to mark the end of the static pose.")
        logging.info("Marking the end of a static pose.")
        blk_arc.end_static_pose()

        # Stop the scan:
        input("Hit <Enter> to stop the scan.")
        logging.info("Stopping the scan.")
        blk_arc.stop_capture()
        logging.info('Stopped the scan.')
    else:
        logging.warning("Could not start a scan.")

    blk_arc.disconnect()


if __name__ == '__main__':
    FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
    logging.basicConfig(level=logging.INFO, format=FORMAT)
    run_example()
