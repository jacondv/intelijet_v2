import logging

import numpy as np

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

    # Get the extrinsics of all frames
    logging.info("Loading the extrinsics.")
    extrinsics = blk_arc.get_coordinate_frames()
    baser_tf_slam_cam_front = np.array(extrinsics.slam_cam_front_transformation.elements).reshape(4, 4)
    logging.info(f"\nBaser-frame to slam camera front: \n{baser_tf_slam_cam_front}")
    baser_tf_slam_cam_left = np.array(extrinsics.slam_cam_left_transformation.elements).reshape(4, 4)
    logging.info(f"\nBaser-frame to slam camera left: \n{baser_tf_slam_cam_left}")
    baser_tf_slam_cam_right = np.array(extrinsics.slam_cam_right_transformation.elements).reshape(4, 4)
    logging.info(f"\nBaser-frame to slam camera right: \n{baser_tf_slam_cam_right}")
    baser_tf_user_camera = np.array(extrinsics.user_cam_transformation.elements).reshape(4, 4)
    logging.info(f"\nBaser-frame to user camera: \n{baser_tf_user_camera}")
    baser_tf_imu = np.array(extrinsics.imu_transformation.elements).reshape(4, 4)
    logging.info(f"\nBaser-frame to IMU: \n{baser_tf_imu}")
    baser_tf_housing_reference = np.array(extrinsics.housing_ref_mark_transformation.elements).reshape(4, 4)
    logging.info(f"\nBaser-frame to housing-reference: \n{baser_tf_housing_reference}")

    blk_arc.disconnect()


if __name__ == '__main__':
    FORMAT = '%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s'
    logging.basicConfig(level=logging.INFO, format=FORMAT)
    run_example()
