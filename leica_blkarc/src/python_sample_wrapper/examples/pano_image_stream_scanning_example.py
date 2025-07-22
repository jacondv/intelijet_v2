import cv2
import logging
import time

from blk_arc_sample_wrapper.blk_arc import BLK_ARC
from blk_arc_sample_wrapper.blk_arc_config import ConnectionType
from blk_arc_sample_wrapper.blk_arc_pano_image_decoder import BLK_ARC_PanoImageDecoder as ImageDecoder

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

    # Start Pano Image Stream during an on-going scan
    logging.info("Start Pano Image Stream during an on-going scan.")
    pano_image_stream = blk_arc.stream_pano_images_scanning(count=2)

    img_id = 0
    for pano_image in pano_image_stream:
        # Decode the received Image either to BGR (decodeToColor) or Grayscale (decodeToGrayscale)
        bgr_image = ImageDecoder.decodeToColor(pano_image.data)

        # Display the image
        cv2.namedWindow("Streamed Pano Image", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Streamed Pano Image", 800, 600)  # đặt size cửa sổ
        cv2.imshow("Streamed Pano Image", bgr_image)
        cv2.waitKey(500)
        

        # Save the image to your disk
        # if "y" in input("Would you like to save the image? (y/n) ").lower():
        #     cv2.imwrite(f"pano_{img_id}.png", bgr_image)
        img_id += 1

    cv2.destroyAllWindows()
    # close the pano image stream
    pano_image_stream.cancel()

    time.sleep(3)

    logging.info("Stopping the scan.")
    blk_arc.stop_capture()
    logging.info('Stopped the scan.')

    blk_arc.disconnect()


if __name__ == '__main__':
    FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
    logging.basicConfig(level=logging.INFO, format=FORMAT)
    run_example()
