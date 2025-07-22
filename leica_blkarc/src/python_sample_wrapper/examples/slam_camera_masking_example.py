import logging
import time
from pathlib import Path
from typing import List

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


def get_left_slam_cam_mask(blk_arc: BLK_ARC) -> bool:
    logging.info("Get the active mask for the left SLAM camera.")

    slam_mask = blk_arc.get_mask(mask_type=masking_message.MaskIdentifier.SLAM_CAMERA_LEFT)

    if not slam_mask:
        logging.warning("Failed to get the slam camera mask.")
        return False

    if slam_mask.id != masking_message.MaskIdentifier.SLAM_CAMERA_LEFT:
        logging.warning("Got the wrong mask.")
        return False

    slam_bit_mask = slam_mask.slam_mask.bit_mask
    logging.info(
        f"Got the left SLAM camera mask with width: {slam_bit_mask.image_width_pixels} and height: {slam_bit_mask.image_height_pixels}."
    )
    return True


def reset_all_slam_cam_masks(blk_arc: BLK_ARC, ids: List) -> bool:
    logging.info("Reset all SLAM camera masks.")
    for id in ids:
        if not blk_arc.reset_mask(id):
            logging.warning(f"Failed to reset the mask for camera: {id}")
            return False
    return True


def run_example():
    # Create BLK ARC instance:
    blk_arc = BLK_ARC()

    # Connect:
    if not blk_arc.connect(CONNECTION_TYPE):
        logging.warning("Could not connect, make sure you selected the right connection-type.")
        return

    image_masks = [
        Path('src/python_sample_wrapper/examples/masks/slam_camera_mask_left.png'),
        Path('src/python_sample_wrapper/examples/masks/slam_camera_mask_center.png'),
        Path('src/python_sample_wrapper/examples/masks/slam_camera_mask_right.png')
    ]

    slam_camera_ids = [
        masking_message.MaskIdentifier.SLAM_CAMERA_LEFT, masking_message.MaskIdentifier.SLAM_CAMERA_CENTER,
        masking_message.MaskIdentifier.SLAM_CAMERA_RIGHT
    ]

    # Set the SLAM camera masks to crop the ceiling
    # The masks are active until a reboot is done or a capture application crash happened.
    # When rebooting, the capture application always starts up with empty masks.
    for i in range(len(image_masks)):
        logging.info(f"Setting the mask for camera: {slam_camera_ids[i]}.")
        if not blk_arc.set_slam_mask(image_masks[i], slam_camera_ids[i]):
            return

    # Capture a scan with the applied lidar mask (e.g. crop the ceiling)
    if not capture_scan(blk_arc=blk_arc):
        return

    # Get the left SLAM camera mask
    if not get_left_slam_cam_mask(blk_arc=blk_arc):
        return

    # If desired, save the mask of the left slam camera as an image
    if "y" in input("Would you like to save the currently active left slam camera mask as an image? (y/n) ").lower():
        mask_type = masking_message.MaskIdentifier.SLAM_CAMERA_LEFT
        mask_filename = Path(f"active_{masking_message.MaskIdentifier.Name(mask_type).lower()}_mask.png")
        blk_arc.download_mask(mask_type=mask_type, path=mask_filename)

    # Reset all SLAM camera masks
    if not reset_all_slam_cam_masks(blk_arc=blk_arc, ids=slam_camera_ids):
        return

    blk_arc.disconnect()


if __name__ == '__main__':
    FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
    logging.basicConfig(level=logging.INFO, format=FORMAT)
    run_example()
