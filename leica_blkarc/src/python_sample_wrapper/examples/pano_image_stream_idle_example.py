import cv2
import logging

from blk_arc_sample_wrapper.blk_arc import BLK_ARC
from blk_arc_sample_wrapper.blk_arc_config import ConnectionType
from blk_arc_sample_wrapper.blk_arc_pano_image_decoder import BLK_ARC_PanoImageDecoder as ImageDecoder

from blk_arc_grpc import imaging_pb2 as imaging_message

# Modify desired connection type here: WIRED or WIRELESS
CONNECTION_TYPE = ConnectionType.WIRELESS


def run_example():
    # Create BLK ARC instance:
    blk_arc = BLK_ARC()

    # Connect:
    if not blk_arc.connect(CONNECTION_TYPE):
        logging.warning("Could not connect, make sure you selected the right connection-type.")
        return

    # Start Pano Camera Stream while no scan is on-going.
    logging.info("Start Pano Camera Stream.")
    # if the exposure time and the gain is given by the integrator, then the exposure mode is set to MANUAL, else it is set to AUTO
    pano_camera_stream = blk_arc.stream_pano_images_idle(
        camera_id=imaging_message.PanoramaCameraStreamRequest.CameraID.CAM_FRONT, count=3, exposure_in_ms=10, gain=25)

    img_id = 0
    for pano_image in pano_camera_stream:
        # Decode the received Image either to BGR (decodeToColor) or Grayscale (decodeToGrayscale)
        bgr_image = ImageDecoder.decodeToColor(pano_image.data)

        # Display the image
        cv2.imshow("Streamed Pano Camera", bgr_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Save the image to your disk
        if "y" in input("Would you like to save the image? (y/n) ").lower():
            cv2.imwrite(f"pano_{img_id}.png", bgr_image)
        img_id += 1

    # close the pano camera stream and disconnect
    pano_camera_stream.cancel()
    blk_arc.disconnect()


if __name__ == '__main__':
    FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
    logging.basicConfig(level=logging.INFO, format=FORMAT)
    run_example()
