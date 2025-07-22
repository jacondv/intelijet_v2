import logging
import time

from blk_arc_sample_wrapper.blk_arc import BLK_ARC
from blk_arc_sample_wrapper.blk_arc_config import ConnectionType

# Modify desired connection type here: WIRED or WIRELESS
CONNECTION_TYPE = ConnectionType.WIRELESS


def run_example():
    # Create BLK ARC instance:
    blk_arc = BLK_ARC()

    # Connect:
    if not blk_arc.connect(CONNECTION_TYPE):
        logging.warning("Could not connect, make sure you selected the right connection-type.")
        return

    logging.info("Starting a scan.")
    start_scan_info = blk_arc.start_capture()
    if start_scan_info is None:
        logging.warning("Could not start a scan.")
        return

    # Wait for the scan to be initialized:
    while not blk_arc.is_scanning():
        time.sleep(0.5)
    logging.info(f"Initialized scan with ID: {start_scan_info.scan_id}.")

    time.sleep(1)

    # Trigger a single detail image
    logging.info("Triggering Detail Image.")
    blk_arc.trigger_detail_image()

    # Triggering a detail image requires a few seconds.
    # Caution: If the scan is stopped before the image is triggered, no image will be saved.
    time.sleep(3)

    logging.info("Stopping the scan.")
    blk_arc.stop_capture()
    logging.info('Stopped the scan.')

    blk_arc.disconnect()


if __name__ == '__main__':
    FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
    logging.basicConfig(level=logging.INFO, format=FORMAT)
    run_example()
