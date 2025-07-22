import logging

import blk_arc_grpc.network_pb2 as network_message

from blk_arc_sample_wrapper.blk_arc import BLK_ARC
from blk_arc_sample_wrapper.blk_arc_config import ConnectionType

CONNECTION_TYPE = ConnectionType.WIRED


def print_network_information(network_information: network_message.NetworkInformationResponse) -> None:
    for counter, interface in enumerate(network_information.interfaces):
        logging.info(f"\nNetwork Interface {counter}:\n"
                     f"  Network Interface:     {interface.network_interface}\n"
                     f"  IPv4:                  {interface.ipv4}\n"
                     f"  IPv4:                  {interface.ipv6}\n"
                     f"  MAC address:           {interface.mac_address}")


def run_example() -> None:
    # Create BLK ARC instance:
    blk_arc = BLK_ARC()

    # Connect:
    if not blk_arc.connect(CONNECTION_TYPE):
        logging.warning("Could not connect, make sure you selected the right connection-type.")
        return

    network_information = blk_arc.get_network_information()
    if network_information:
        print_network_information(network_information)
    else:
        logging.warning('Failed to get the network information.')

    blk_arc.disconnect()


if __name__ == '__main__':
    FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
    logging.basicConfig(level=logging.INFO, format=FORMAT)
    run_example()
