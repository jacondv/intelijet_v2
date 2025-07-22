import logging
from pathlib import Path

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

    # List last five scans on the scanner:
    scans = blk_arc.list_scans()
    if scans:
        # The ordering of the scans is arbitrary, therefore we need to sort them:
        scans.sort(key=lambda x: int(x.metadata.properties['item/id']))
        logging.info("Last five scans on the scanner:")
        for scan in scans[-5:]:
            logging.info(f"\n{scan.path}\n"
                         f"  Size:         {(scan.size_bytes / 1e6):.3f}MB\n"
                         f"  ID:           {scan.metadata.properties['item/id']}\n"
                         f"  Time Started: {scan.metadata.properties['scan/time-started']}\n"
                         f"  Time Stopped: {scan.metadata.properties['scan/time-finished']}")
    else:
        logging.warning('Could not get scans.')

    # Download a scan
    if "y" in input("Would you like to download a scan? (y/n) ").lower():
        scan_id = input("Enter the ID of the scan you would like to download: ")
        scan_info = blk_arc.get_scan_info(int(scan_id))
        if scan_info:
            # Download the captured scan:
            datasets_path = Path.cwd() / "datasets"
            logging.info(f"Downloading scan to '{datasets_path}', this might take a while...")
            blk_arc.download_scan(int(scan_id), datasets_path)
            logging.info("Scan downloaded.")
        else:
            logging.error(f"Scan '{scan_id}' could not be found on the device.")

    # Delete a scan
    if "y" in input("Would you like to delete a scan? (y/n) ").lower():
        scan_id = input("Enter the ID of the scan you would like to delete: ")
        scan_info = blk_arc.get_scan_info(int(scan_id))
        if scan_info:
            # Delete the captured scan:
            logging.info("Deleting scan.")
            blk_arc.delete_scan(int(scan_id))
            logging.info("Scan deleted.")
        else:
            logging.error(f"Scan '{scan_id}' could not be found on the device.")

    blk_arc.disconnect()


if __name__ == '__main__':
    FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
    logging.basicConfig(level=logging.INFO, format=FORMAT)
    run_example()
