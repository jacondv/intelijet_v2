# pps/scan_strategies/direct_3d.py
"""Skeleton for a scanner that returns a complete 3D point cloud from a
single fixed housing position (e.g. Leica BLK360G2 - SDK/ROS scaffold
already exists at blk360g2_ws/, not integrated into pps/ui yet).

NOT wired up anywhere yet (hmi_scan_command_handler.py::get_scanner_controller()
still only returns SickScanErobController) and intentionally raises
NotImplementedError - this file only documents the intended shape so a
future implementer knows exactly what to fill in and where, without having
to touch GenericScanController or anything in pps/ui.

Intended acquire() flow, once implemented:
  1. Open the housing to a single fixed angle (NOT a step sequence like
     Sick2DAssembleStrategy) - e.g. a new `cfg.housing_fixed_open_angle`
     config value, via `controller.housing.open(...)` +
     `controller.wait_until_target(cfg.housing_fixed_open_angle, direction=True)`.
  2. Trigger a capture on the BLK360G2 (topic/action exposed by
     blk360g2_ws/src/blk360g2_ros - TODO: figure out the actual
     action/topic name once that package is integrated into this
     workspace) and wait for the resulting point cloud.
  3. Convert/publish the result as a sensor_msgs/PointCloud2 on
     `publisher`, same as Sick2DAssembleStrategy.acquire() does.
  4. Close the housing back (a single close command is enough here -
     no graduated step sequence needed since, unlike SickScan, housing
     motion here isn't part of the measurement).
  5. Drive controller.status_callback(...) with PRESCAN_ERROR/
     POSTSCAN_ERROR/IDLE the same way Sick2DAssembleStrategy does, so the
     UI's existing status handling keeps working unchanged.
"""
from pps.scan_strategies.base import ScanStrategy


class Direct3DScanStrategy(ScanStrategy):
    def acquire(self, controller, publisher=None):
        raise NotImplementedError(
            "Direct3DScanStrategy is a skeleton for a future direct-3D "
            "scanner (e.g. BLK360G2) - see this file's module docstring "
            "for the intended implementation. Not wired into "
            "get_scanner_controller() yet."
        )
