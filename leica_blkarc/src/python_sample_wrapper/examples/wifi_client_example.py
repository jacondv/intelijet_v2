import logging

import blk_arc_grpc.wifi_client_pb2 as wifi_client_message

from blk_arc_sample_wrapper.blk_arc import BLK_ARC
from blk_arc_sample_wrapper.blk_arc_config import ConnectionType

CONNECTION_TYPE = ConnectionType.WIRED


def print_visible_networks(visible_networks: wifi_client_message.ScanWifiNetworksResponse) -> None:
    if len(visible_networks.networks) == 0:
        logging.warning("No Wifi network visible.")
    else:
        for counter, network in enumerate(visible_networks.networks):
            logging.info(f"\nWifi {counter}:\n"
                         f"  Wifi SSID:            {network.ssid}\n"
                         f"  Wifi Signal Strength: {network.signal_strength}")


def print_wifi_client_status(wifi_client_status: wifi_client_message.WifiClientStatusResponse) -> None:
    if wifi_client_status.connection_status == wifi_client_message.WifiClientConnectionState.NOT_CONNECTED:
        logging.info("The BLK ARC is not connected to any Wifi network.")
    else:
        logging.info(f"\nWifi Client status:\n"
                     f"  Connection State:     {wifi_client_status.connection_status}\n"
                     f"  Wifi Network SSID:    {wifi_client_status.ssid}\n"
                     f"  Wifi Internet Access: {wifi_client_status.internet_access}")


def print_known_wifi_networks(list_known_wifi_networks: wifi_client_message.KnownWifiListResponse) -> None:
    if len(list_known_wifi_networks.networks) == 0:
        logging.warning('No known Wifi networks.')
    else:
        for counter, network in enumerate(list_known_wifi_networks.networks):
            logging.info(f"\nWifi network {counter}:"
                         f"   Wifi SSID:           {network.ssid}\n"
                         f"   Encryption:          {network.encryption}\n"
                         f"   Autoconnect enabled: {network.autoconnect}")


def run_example() -> None:
    # Create BLK ARC instance:
    blk_arc = BLK_ARC()

    # Connect:
    if not blk_arc.connect(CONNECTION_TYPE):
        logging.warning("Could not connect, make sure you selected the right connection-type.")
        return

    # Scan for visible Wifi Networks:
    visible_networks = blk_arc.scan_wifi_networks()
    if visible_networks is not None:
        print_visible_networks(visible_networks)
    else:
        logging.warning("Failed to scan the wifi networks.")

    # Connect to a known Wifi network
    if "y" in input("\nWould you like to connect to a Wifi network? (y/n) ").lower():
        ssid = input("Enter the SSID of the Wifi network you would like to connect: ")
        pwd = input("Enter the Password of the Wifi network you would like to connect: ")
        wifi_status = blk_arc.connect_wifi_network(ssid=ssid, pwd=pwd, encryption=wifi_client_message.Encryption.WPA)
        if wifi_status:
            print_wifi_client_status(wifi_status)
        else:
            logging.warning("Failed to connect to the Wifi network.")

    # Get the status of the Wifi client
    wifi_client_status = blk_arc.get_wifi_client_status()
    if wifi_client_status:
        print_wifi_client_status(wifi_client_status)
    else:
        logging.warning("Failed to get the Wifi client status.")

    # Get a list of all known Wifi Networks
    logging.info("List all known Wifi networks:")
    list_known_wifi_networks = blk_arc.get_known_wifi_networks()
    if list_known_wifi_networks:
        print_known_wifi_networks(list_known_wifi_networks)
    else:
        logging.warning("Failed to get a list of all known Wifi networks.")

    # Delete a known Wifi Network
    if "y" in input("\nWould you like to delete a known Wifi Network? (y/n)").lower():
        ssid_delete = input("Enter the SSID of the Wifi network you would like to delete:")
        if not blk_arc.delete_known_wifi_network(ssid=ssid_delete):
            logging.info(f"Failed to delete Wifi network with SSID: {ssid_delete}.")

    # Clear all known Wifi Networks
    if "y" in input("\nWould you like to clear all known Wifi Networks? (y/n)").lower():
        if not blk_arc.clear_known_wifi_networks():
            logging.info("Failed to clear all knwon Wifi networks.")

    blk_arc.disconnect()


if __name__ == '__main__':
    FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
    logging.basicConfig(level=logging.INFO, format=FORMAT)
    run_example()
