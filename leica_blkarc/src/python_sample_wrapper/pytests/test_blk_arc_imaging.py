import time

from blk_arc_sample_wrapper.blk_arc import BLK_ARC

from blk_arc_grpc import imaging_pb2 as imaging_message

PANO_IMAGE_HEIGHT = 1098
PANO_IMAGE_WIDTH = 1456
PANO_IMAGE_ROWSTRIDE = 2912


# helper function
def assert_pano_image_properties(pano_image):
    assert pano_image.height == PANO_IMAGE_HEIGHT
    assert pano_image.width == PANO_IMAGE_WIDTH
    assert pano_image.row_stride == PANO_IMAGE_ROWSTRIDE
    assert pano_image.data_size == PANO_IMAGE_HEIGHT * PANO_IMAGE_ROWSTRIDE + 16


def test_trigger_detail_image(blkarc_scanning: BLK_ARC):
    assert blkarc_scanning.trigger_detail_image()
    time.sleep(3)


def test_trigger_detail_image_not_scanning(connected_blk_arc: BLK_ARC):
    assert connected_blk_arc.is_connected()
    assert not connected_blk_arc.trigger_detail_image()


def test_pano_image_stream(blkarc_scanning: BLK_ARC):
    pano_image_stream = blkarc_scanning.stream_pano_images_scanning(count=1)

    img_count = 0
    for pano_image in pano_image_stream:
        img_count += 1
        assert_pano_image_properties(pano_image)

    pano_image_stream.cancel()
    assert img_count == 3


def test_pano_image_stream_not_scanning(connected_blk_arc: BLK_ARC):
    assert connected_blk_arc.is_connected()
    assert not connected_blk_arc.stream_pano_images_scanning(count=1)


def test_pano_camera_stream(connected_blk_arc: BLK_ARC):
    assert connected_blk_arc.is_connected()
    total_imgs = 3
    pano_camera_stream = connected_blk_arc.stream_pano_images_idle(
        camera_id=imaging_message.PanoramaCameraStreamRequest.CameraID.CAM_LEFT,
        count=total_imgs,
        exposure_in_ms=20,
        gain=25)

    img_count = 0
    for pano_image in pano_camera_stream:
        img_count += 1
        assert_pano_image_properties(pano_image)

    pano_camera_stream.cancel()
    assert img_count == total_imgs


def test_pano_camera_stream_not_all_params(connected_blk_arc: BLK_ARC):
    assert connected_blk_arc.is_connected()
    total_imgs = 3
    pano_camera_stream = connected_blk_arc.stream_pano_images_idle(
        camera_id=imaging_message.PanoramaCameraStreamRequest.CameraID.CAM_LEFT, count=total_imgs)

    img_count = 0
    for pano_image in pano_camera_stream:
        img_count += 1
        assert_pano_image_properties(pano_image)

    pano_camera_stream.cancel()
    assert img_count == total_imgs


def test_pano_camera_stream_while_scanning(blkarc_scanning: BLK_ARC):
    assert not blkarc_scanning.stream_pano_images_idle(
        camera_id=imaging_message.PanoramaCameraStreamRequest.CameraID.CAM_LEFT, count=1)
