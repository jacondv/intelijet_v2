from blk_arc_sample_wrapper.blk_arc import BLK_ARC


def test_acknowledge_faults(connected_blk_arc: BLK_ARC):
    assert connected_blk_arc.acknowledge_faults()
