import logging

import blk_arc_grpc.device_pb2 as device_message

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

    # Get Device Info:
    calib_info = blk_arc.get_calibration_info()
    if calib_info:
        print(calib_info)
            

    else:
        logging.warning('Could not get device-info.')

    # Get Firmware Version:
    firmware_version = blk_arc.get_firmware_version()
    if firmware_version:
        logging.info(f"Firmware version: {firmware_version}")
    else:
        logging.warning('Could not get firmware-version.')

    # Get Device Status:
    device_status = blk_arc.get_device_status()
    if device_status:
        device_states = device_message.DeviceStateResponse.State
        logging.info(f"Device is in state '{device_states.Name(device_status.state)}'.")
    else:
        logging.warning('Could not get device-status.')

    # Get Disk Information
    free_disk_space = blk_arc.get_free_disk_space_percentage()
    if free_disk_space:
        logging.info(f"Free disk space: {free_disk_space:.2f}%")
    else:
        logging.warning('Could not get free disk space information.')

    blk_arc.disconnect()


if __name__ == '__main__':
    FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
    logging.basicConfig(level=logging.INFO, format=FORMAT)
    run_example()
