import logging
import time
from pathlib import Path

from blk_arc_grpc import masking_pb2 as masking_message
from blk_arc_sample_wrapper.blk_arc import BLK_ARC
from blk_arc_sample_wrapper.blk_arc_config import ConnectionType

# Modify desired connection type here: WIRED or WIRELESS
CONNECTION_TYPE = ConnectionType.WIRED


def capture_scan(blk_arc: BLK_ARC) -> bool:
    logging.info("Starting a scan.")
    start_scan_info = blk_arc.start_capture()
    if start_scan_info:
        # Wait for the scan to be initialized:
        while not blk_arc.is_scanning():
            time.sleep(0.5)
        logging.info(f"Initialized scan with ID: {start_scan_info.scan_id}.")

        # Stop the scan:
        input("Hit <Enter> to stop the scan.")
        logging.info("Stopping the scan.")
        blk_arc.stop_capture()
        time.sleep(2)
        logging.info('Stopped the scan.')
        return True
    else:
        logging.warning("Could not start a scan.")
        return False


def get_lidar_mask(blk_arc: BLK_ARC) -> bool:
    logging.info("Get the active lidar mask.")

    lidar_mask = blk_arc.get_mask(mask_type=masking_message.MaskIdentifier.LIDAR)

    if not lidar_mask:
        logging.warning("Failed to get the lidar mask.")
        return False

    if lidar_mask.id != masking_message.MaskIdentifier.LIDAR:
        logging.warning("Got the wrong mask.")
        return False

    lidar_byte_mask = lidar_mask.lidar_mask.byte_mask
    logging.info(f"Got the lidar mask with width: {lidar_byte_mask.width} and height: {lidar_byte_mask.height}.")
    return True


def reset_lidar_mask(blk_arc: BLK_ARC) -> bool:
    logging.info("Reset the mask.")
    if not blk_arc.reset_mask(masking_message.MaskIdentifier.LIDAR):
        logging.warning("Failed to reset the lidar mask.")
        return False
    return True


def run_example():
    # Create BLK ARC instance:
    blk_arc = BLK_ARC()

    # Connect:
    if not blk_arc.connect(CONNECTION_TYPE):
        logging.warning("Could not connect, make sure you selected the right connection-type.")
        return

    # Set the lidar mask to crop the upper half of the scan
    # The mask is active until a reboot is done or a capture application crash happened.
    # When rebooting, the capture application always starts up with an empty mask.
    logging.info("Setting the lidar mask.")
    if not blk_arc.set_lidar_mask(Path('src/python_sample_wrapper/examples/masks/lidar_mask.png')):
        return

    # Capture a scan with the applied lidar mask
    if not capture_scan(blk_arc=blk_arc):
        return

    # Get the lidar mask
    if not get_lidar_mask(blk_arc=blk_arc):
        return

    # If desired, save the lidar mask as an image
    if "y" in input("Would you like to save the currently active lidar mask as an image? (y/n) ").lower():
        path_mask = Path("active_lidar_mask.png")
        blk_arc.download_mask(mask_type=masking_message.MaskIdentifier.LIDAR, path=path_mask)

    # Reset the lidar mask
    if not reset_lidar_mask(blk_arc=blk_arc):
        return

    blk_arc.disconnect()


if __name__ == '__main__':
    FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
    logging.basicConfig(level=logging.INFO, format=FORMAT)
    run_example()
