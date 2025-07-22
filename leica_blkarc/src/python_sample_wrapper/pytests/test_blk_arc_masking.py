from pathlib import Path

import numpy as np
import pytest
from blk_arc_grpc import masking_pb2 as masking_message
from PIL import Image

from blk_arc_sample_wrapper.blk_arc import BLK_ARC

# LIDAR mask dimension
LIDAR_MASK_WIDTH = 2048
LIDAR_MASK_HEIGHT = 1024
# SLAM camera mask dimension
SLAM_CAM_MASK_WIDTH = 1456
SLAM_CAM_MASK_HEIGHT = 1088


def test_lidar_get_mask(connected_blk_arc: BLK_ARC):
    mask_description = connected_blk_arc.get_mask(masking_message.MaskIdentifier.LIDAR)

    assert mask_description.id == masking_message.MaskIdentifier.LIDAR

    byte_mask = mask_description.lidar_mask.byte_mask

    assert byte_mask.width == LIDAR_MASK_WIDTH
    assert byte_mask.height == LIDAR_MASK_HEIGHT
    assert len(byte_mask.data) == LIDAR_MASK_WIDTH * LIDAR_MASK_HEIGHT


def test_lidar_set_mask(connected_blk_arc: BLK_ARC):
    byte_indices = {10, 2357, 8888, 98662, 828861, 1112232, 2063151}

    byte_mask = bytearray(int(LIDAR_MASK_HEIGHT * LIDAR_MASK_WIDTH))
    num_bytes = len(byte_mask)

    SET_BYTE_MASK = 255
    for index in byte_indices:
        byte_mask[index] = SET_BYTE_MASK
        assert byte_mask[index] == SET_BYTE_MASK

    image = Image.frombytes('L', (LIDAR_MASK_WIDTH, LIDAR_MASK_HEIGHT), bytes(byte_mask))
    mask_filename = Path("test_lidar_mask.png")
    image.save(mask_filename)

    assert connected_blk_arc.set_lidar_mask(mask_filename=mask_filename)

    # delete the saved mask image
    mask_filename.unlink()

    mask_description = connected_blk_arc.get_mask(masking_message.MaskIdentifier.LIDAR)

    assert mask_description.id == masking_message.MaskIdentifier.LIDAR

    lidar_byte_mask = mask_description.lidar_mask.byte_mask

    assert lidar_byte_mask.width == LIDAR_MASK_WIDTH
    assert lidar_byte_mask.height == LIDAR_MASK_HEIGHT
    assert len(lidar_byte_mask.data) == num_bytes

    lidar_byte_mask_data = lidar_byte_mask.data
    for index in range(num_bytes):
        assert byte_mask[index] == lidar_byte_mask_data[index]


def test_lidar_set_mask_not_png(connected_blk_arc: BLK_ARC):
    byte_indices = {10, 2357, 8888, 98662, 828861, 1112232, 2063151}

    byte_mask = bytearray(int(LIDAR_MASK_HEIGHT * LIDAR_MASK_WIDTH))

    SET_BYTE_MASK = 255
    for index in byte_indices:
        byte_mask[index] = SET_BYTE_MASK
        assert byte_mask[index] == SET_BYTE_MASK

    image = Image.frombytes('L', (LIDAR_MASK_WIDTH, LIDAR_MASK_HEIGHT), bytes(byte_mask))
    mask_filename = Path("test_lidar_mask.jpg")
    image.save(mask_filename)

    assert not connected_blk_arc.set_lidar_mask(mask_filename=mask_filename)

    # delete the saved mask image
    mask_filename.unlink()


def test_lidar_reset_mask(connected_blk_arc: BLK_ARC):
    assert connected_blk_arc.reset_mask(masking_message.MaskIdentifier.LIDAR)


@pytest.mark.parametrize("slam_camera_id", [
    masking_message.MaskIdentifier.SLAM_CAMERA_LEFT, masking_message.MaskIdentifier.SLAM_CAMERA_RIGHT,
    masking_message.MaskIdentifier.SLAM_CAMERA_CENTER
])
def test_slam_camera_get_mask(connected_blk_arc: BLK_ARC, slam_camera_id: masking_message.MaskIdentifier):
    mask_description = connected_blk_arc.get_mask(slam_camera_id)

    assert mask_description.id == slam_camera_id

    byte_mask = mask_description.slam_mask.bit_mask

    bytes_per_row = SLAM_CAM_MASK_WIDTH // 8
    assert byte_mask.image_width_pixels == SLAM_CAM_MASK_WIDTH
    assert byte_mask.image_height_pixels == SLAM_CAM_MASK_HEIGHT
    assert byte_mask.bytes_per_row == bytes_per_row
    assert len(byte_mask.data) == bytes_per_row * SLAM_CAM_MASK_HEIGHT


@pytest.mark.parametrize("slam_camera_id", [
    masking_message.MaskIdentifier.SLAM_CAMERA_LEFT, masking_message.MaskIdentifier.SLAM_CAMERA_RIGHT,
    masking_message.MaskIdentifier.SLAM_CAMERA_CENTER
])
def test_slam_camera_set_mask(connected_blk_arc: BLK_ARC, slam_camera_id: masking_message.MaskIdentifier):
    pixel_locations = [[0, 0], [0, 300], [444, 0], [555, 555], [900, 0], [0, 999],
                       [SLAM_CAM_MASK_HEIGHT - 1, SLAM_CAM_MASK_WIDTH - 1]]

    mask_array = np.zeros([SLAM_CAM_MASK_HEIGHT, SLAM_CAM_MASK_WIDTH], dtype=np.uint8)

    SET_BYTE_MASK = 255
    for location in pixel_locations:
        mask_array[location[0], location[1]] = SET_BYTE_MASK
        assert mask_array[location[0], location[1]] == SET_BYTE_MASK

    bool_mask = mask_array > 128
    bit_array = np.packbits(bool_mask, axis=1).flatten()

    bytes_per_row = SLAM_CAM_MASK_WIDTH // 8

    image = Image.frombytes('L', (SLAM_CAM_MASK_WIDTH, SLAM_CAM_MASK_HEIGHT), bytes(mask_array))

    mask_filename = Path("test_slam_mask.png")
    image.save(mask_filename)

    assert connected_blk_arc.set_slam_mask(mask_filename=mask_filename, camera_id=slam_camera_id)

    # delete the saved mask image
    mask_filename.unlink()

    mask_description = connected_blk_arc.get_mask(slam_camera_id)

    assert mask_description.id == slam_camera_id

    slam_bit_mask = mask_description.slam_mask.bit_mask

    assert slam_bit_mask.image_width_pixels == SLAM_CAM_MASK_WIDTH
    assert slam_bit_mask.image_height_pixels == SLAM_CAM_MASK_HEIGHT
    assert slam_bit_mask.bytes_per_row == bytes_per_row
    assert len(slam_bit_mask.data) == bytes_per_row * SLAM_CAM_MASK_HEIGHT

    # Unpack the bit array to get the boolean mask
    bool_mask = np.unpackbits(np.frombuffer(bit_array, dtype=np.uint8)).reshape(SLAM_CAM_MASK_HEIGHT,
                                                                                SLAM_CAM_MASK_WIDTH).astype(bool)

    # Convert the boolean mask to the mask array
    slam_bit_mask_data = np.zeros([SLAM_CAM_MASK_HEIGHT, SLAM_CAM_MASK_WIDTH], dtype=np.uint8)
    slam_bit_mask_data[bool_mask] = 255

    assert np.array_equal(mask_array, slam_bit_mask_data)


def test_slam_camera_set_mask_not_png(connected_blk_arc: BLK_ARC):
    pixel_locations = [[0, 0], [0, 300], [444, 0], [555, 555], [900, 0], [0, 999],
                       [SLAM_CAM_MASK_HEIGHT - 1, SLAM_CAM_MASK_WIDTH - 1]]

    mask_array = np.zeros([SLAM_CAM_MASK_HEIGHT, SLAM_CAM_MASK_WIDTH], dtype=np.uint8)

    SET_BYTE_MASK = 255
    for location in pixel_locations:
        mask_array[location[0], location[1]] = SET_BYTE_MASK
        assert mask_array[location[0], location[1]] == SET_BYTE_MASK

    image = Image.frombytes('L', (SLAM_CAM_MASK_WIDTH, SLAM_CAM_MASK_HEIGHT), bytes(mask_array))

    mask_filename = Path("test_slam_mask.jpg")
    image.save(mask_filename)

    assert not connected_blk_arc.set_slam_mask(mask_filename=mask_filename,
                                               camera_id=masking_message.MaskIdentifier.SLAM_CAMERA_LEFT)

    # delete the saved mask image
    mask_filename.unlink()


@pytest.mark.parametrize("slam_camera_id", [
    masking_message.MaskIdentifier.SLAM_CAMERA_LEFT, masking_message.MaskIdentifier.SLAM_CAMERA_RIGHT,
    masking_message.MaskIdentifier.SLAM_CAMERA_CENTER
])
def test_slam_camera_reset_mask(connected_blk_arc: BLK_ARC, slam_camera_id: masking_message.MaskIdentifier):
    assert connected_blk_arc.reset_mask(slam_camera_id)


def test_reset_mask_invalid_type(connected_blk_arc: BLK_ARC):
    assert not connected_blk_arc.reset_mask(masking_message.MaskIdentifier.TYPE_UNSET)


def test_get_mask_invalid_type(connected_blk_arc: BLK_ARC):
    assert not connected_blk_arc.get_mask(masking_message.MaskIdentifier.TYPE_UNSET)
