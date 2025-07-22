from pathlib import Path
from typing import Tuple

import blk_arc_grpc.scan_library_pb2 as scan_library_message

from blk_arc_sample_wrapper.blk_arc import BLK_ARC


def test_list_scans(blk_arc_with_finished_scan: Tuple[BLK_ARC, int]):
    blk_arc, scan_id = blk_arc_with_finished_scan
    assert blk_arc.is_connected()
    scans = blk_arc.list_scans()
    assert scans
    assert len(scans) >= 1
    assert str(type(scans[-1])) == str(scan_library_message.ItemInfo)
    assert sum(scan.metadata.properties['item/id'] == str(scan_id) for scan in scans) == 1


def test_list_scan_info(blk_arc_with_finished_scan: Tuple[BLK_ARC, int]):
    blk_arc, scan_id = blk_arc_with_finished_scan
    assert blk_arc.is_connected()
    scan_info = blk_arc.get_scan_info(scan_id)
    assert scan_info
    assert str(type(scan_info)) == str(scan_library_message.ItemInfo)
    assert scan_info.metadata.properties['item/id'] == str(scan_id)


def test_delete_scan(blk_arc_with_finished_scan: Tuple[BLK_ARC, int]):
    blk_arc, scan_id = blk_arc_with_finished_scan
    assert blk_arc.is_connected()
    assert blk_arc.get_scan_info(scan_id)
    assert blk_arc.delete_scan(scan_id)
    assert not blk_arc.get_scan_info(scan_id)


def test_download_scan(blk_arc_with_finished_scan: Tuple[BLK_ARC, int]):
    blk_arc, scan_id = blk_arc_with_finished_scan
    assert blk_arc.is_connected()
    assert blk_arc.download_scan(scan_id, Path.cwd())
    file_path = Path.cwd() / f"{scan_id}.b2g"
    assert file_path.exists()

    # It is a know issue that the size of the actual scan and the one state in the scan info slightly differs, we allow test to pass if the state size is within range of the actual one
    SCAN_SIZE_TOLERANCE = 0.2
    info_scan_size = blk_arc.get_scan_info(scan_id).size_bytes
    size_min_bound = int(info_scan_size * (1 - SCAN_SIZE_TOLERANCE))
    size_max_bound = int(info_scan_size * (1 + SCAN_SIZE_TOLERANCE))
    assert size_min_bound <= file_path.stat().st_size <= size_max_bound

    file_path.unlink()
