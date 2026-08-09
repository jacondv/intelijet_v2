#!/usr/bin/env python3
from pps.generic_scan_controller import GenericScanController
from pps.scan_strategies.sick_2d_assemble import Sick2DAssembleStrategy


class SickScanErobController(GenericScanController):
    """SickScan (2D LiDAR + housing rotation + assemble_scans2) scan
    controller. All of the actual acquisition logic now lives in
    Sick2DAssembleStrategy - this class only wires that strategy into
    GenericScanController and keeps its name/import path unchanged so
    hmi_scan_command_handler.py doesn't need to change."""

    def __init__(self, status_callback=None):
        super().__init__(strategy=Sick2DAssembleStrategy(), status_callback=status_callback)
