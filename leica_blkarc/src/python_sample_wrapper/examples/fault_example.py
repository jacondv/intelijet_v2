import logging

from blk_arc_grpc import device_pb2 as device_message

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

    # Get Device Status:
    device_status = blk_arc.get_device_status()
    if device_status:
        device_states = device_message.DeviceStateResponse.State
        logging.info(f"Device is in state '{device_states.Name(device_status.state)}'.")
    else:
        logging.warning('Could not get device-status.')

    # Check device status for faults
    if device_status.state == device_message.DeviceStateResponse.State.FAULT_BROKEN:
        logging.warning('The device has experienced a serious fault, likely caused by mechanical issues. ' +
                        'Please create a service report. (may be recovered with a reboot in some cases)')
    elif device_status.state == device_message.DeviceStateResponse.State.FAULT_DEGRADED:
        logging.warning(
            'The device has experienced a fault, likely caused by components being in an unexpected state. ' +
            'Please create a service report and reboot the device to fix the fault.')
    elif device_status.state == device_message.DeviceStateResponse.State.FAULT_RECOVERABLE_DEGRADED:
        logging.warning('The device has experienced a fault that can be recovered without a reboot. ' +
                        'The fault must be acknowledged in order to continue operation.')
        logging.info("Trying to acknowledge the recoverable faults.")
        if blk_arc.acknowledge_faults():
            logging.info("Recoverable faults successfully acknowledged.")
        else:
            logging.warning("Failed to acknowledge recoverable faults. Please reboot the device.")

    blk_arc.disconnect()


if __name__ == '__main__':
    FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
    logging.basicConfig(level=logging.INFO, format=FORMAT)
    run_example()
