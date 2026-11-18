# pps/scan_strategies/base.py
"""Pluggable scan-acquisition interface.

Today SickScan is a 2D LiDAR: the housing rotates through a sequence of
angles WHILE the scanner continuously streams 2D slices, and a separate
service (laser_assembler) stitches everything seen during that time window
into one 3D cloud. So for SickScan, "3D" is a product of
(housing motion x time window x 2D data) - the housing motion is not
incidental, it IS part of the measurement.

A future scanner (e.g. Leica BLK360G2, see blk360g2_ws/ - not integrated
yet) instead returns a complete 3D point cloud from a single fixed housing
position: open to a fixed angle, ask the scanner to capture, get the cloud
back, done. No time-window stitching, no relationship between housing
motion and the resulting data.

Because housing motion and cloud acquisition are coupled differently per
scanner, ScanStrategy owns BOTH end-to-end (not split into a "move housing"
interface + a separate "get cloud" interface, which would force an
unnatural mapping between the two for SickScan). What's genuinely
scanner-agnostic (publishing HousingControl commands, waiting for the
encoder to reach a target, checking cancellation) lives on
GenericScanController and is called by the strategy via the `controller`
argument.

To add a new scanner: write a new ScanStrategy subclass and pass an
instance of it to GenericScanController.__init__() - see
pps/hmi_scan_command_handler.py::get_scanner_controller() for where a new
scanner type gets wired in. No changes to GenericScanController itself.
"""
from abc import ABC, abstractmethod


class ScanStrategy(ABC):
    """One scan acquisition method (e.g. "SickScan 2D + assemble",
    "BLK360G2 direct 3D capture")."""

    @abstractmethod
    def acquire(self, controller, publisher):
        """Run one full prescan/postscan acquisition: move the housing,
        obtain the cloud, publish it on `publisher` itself, and move the
        housing back - all of it, including the publish call. (Matches the
        original run_workflow(), which published mid-function and kept
        going to close the housing afterward; GenericScanController.
        run_workflow() does not publish or touch housing on the strategy's
        behalf, to avoid reordering that sequence.)

        Fully owns housing motion (open/close) for this acquisition -
        GenericScanController.run_workflow() does not do any housing
        motion of its own. Must leave the housing in a safe state before
        returning, matching whatever the concrete strategy considers safe
        (see Sick2DAssembleStrategy for the current close-sequence
        behavior, including what it does when a step fails).

        Returns the resulting sensor_msgs/PointCloud2, or None - note the
        return value is currently unused by any caller (the publish call
        is the actual side effect callers rely on), kept only so a future
        caller can use it without another interface change.

        Args:
            controller: the owning GenericScanController instance - use
                controller.housing (HousingControl), controller.wait_until_target(...),
                controller.status_callback(...), controller.cancel_job.
            publisher: the rospy.Publisher the resulting cloud should be
                published on (cfg.PRE_SCAN_TOPIC or cfg.POST_SCAN_TOPIC).
        """
        raise NotImplementedError
