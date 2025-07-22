from blk_arc_sample_wrapper.blk_arc import BLK_ARC


def test_get_coordinate_frames(connected_blk_arc: BLK_ARC):
    extrinsics = connected_blk_arc.get_coordinate_frames()
    message_fields = extrinsics.DESCRIPTOR.fields_by_name.keys()

    expected_fields = [
        'slam_cam_front_transformation', 'slam_cam_left_transformation', 'slam_cam_right_transformation',
        'user_cam_transformation', 'imu_transformation', 'housing_ref_mark_transformation'
    ]

    assert len(message_fields) == 6
    assert message_fields == expected_fields

    for _, val in extrinsics.ListFields():
        assert len(val.elements) == 16
