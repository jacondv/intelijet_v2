import logging

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
    logging.info("Established a connection. Checking if the device was started in data-only mode.")

    data_only_mode = blk_arc.data_only_mode_active()
    if data_only_mode:
        logging.info("Data-only mode detected. Rebooting the device.")
        if not blk_arc.reboot():
            logging.warning("The device could not be rebooted.")
            return
        logging.info("Please wait for the device to be restarted and connect again. "
                     "If the power-cable is connected, the device will restart with full functionality.")
    else:
        logging.info("The device is not in data-only mode and hence no restart is required.")

    blk_arc.disconnect()


if __name__ == '__main__':
    FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
    logging.basicConfig(level=logging.INFO, format=FORMAT)
    run_example()
