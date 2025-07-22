#!/usr/bin/env python3

PKG = 'ros_blkarc_driver'

import rospy
import test_helpers
import time
import unittest

from ros_blkarc_msgs.srv import CaptureRequest, DownloadScanRequest
from std_srvs.srv import TriggerRequest


class TestDownloadScan(unittest.TestCase):

    # Test if download scan service call behaves correctly
    def test_download_scan(self) -> None:

        # Initialise services and ensure device is ready
        sensor_name = rospy.get_param(param_name="sensor_name", default=None)
        device_is_ready, service_handles = test_helpers.get_all_service_handles_and_ensure_device_is_idle(
            sensor_name=sensor_name)
        self.assertTrue(device_is_ready,
                        "Device is not in IDLE state at the start of test, this may cause unexpected test results")

        # Start scan
        test_helpers.wait_till_blk_arc_is_idle(get_device_state_handle=service_handles["get_device_state"])
        start_response = service_handles["start_capture"].call(CaptureRequest())
        start_scan_id = start_response.scan_id
        test_helpers.assert_capture_service(test_case=self, response=start_response, success_expected=True)

        # Wait for a bit (>= 10 seconds to ensure that scan session is saved)
        time.sleep(20)

        # Stop scan, should be successful
        stop_response = service_handles["stop_capture"].call(CaptureRequest())
        stop_scan_id = stop_response.scan_id
        test_helpers.assert_capture_service(test_case=self, response=stop_response, success_expected=True)

        # Make sure that scan_id matches the one provided by the start scan response
        self.assertEqual(start_scan_id, stop_scan_id,
                         f"Start scan_id {start_scan_id} is different from stop scan_id {stop_scan_id}")

        # Download using scan_id, should be successful and scan id matches the one obtained by the start/stop service calls
        numbered_download_response_1 = service_handles["download_scan"].call(DownloadScanRequest(scan_id=start_scan_id))
        test_helpers.assert_download_scan_service(test_case=self,
                                                  response=numbered_download_response_1,
                                                  success_expected=True)
        self.assertEqual(
            numbered_download_response_1.scan_id, start_scan_id,
            f"Expected download scan_id to be {start_scan_id}, got {numbered_download_response_1.scan_id} instead.")

        # Check that file exists and delete file.
        test_helpers.delete_file(
            test_case=self,
            file_location=f"{numbered_download_response_1.save_directory}/{numbered_download_response_1.filename}")

        # Download using latest scan mode (scan_id set to -1), should be successful and scan id matches the one obtained by the start/stop service calls
        numbered_download_response_2 = service_handles["download_scan"].call(DownloadScanRequest(scan_id=-1))
        test_helpers.assert_download_scan_service(test_case=self,
                                                  response=numbered_download_response_2,
                                                  success_expected=True)
        self.assertEqual(
            numbered_download_response_2.scan_id, start_scan_id,
            f"Expected download scan_id to be {start_scan_id}, got {numbered_download_response_2.scan_id} instead.")

        # Check that file exists, and the size matches up before deleting
        test_helpers.delete_file(
            test_case=self,
            file_location=f"{numbered_download_response_2.save_directory}/{numbered_download_response_2.filename}")

        # Disconnect
        disconnect_response = service_handles["disconnect"].call(TriggerRequest())
        self.assertTrue(disconnect_response.success, "Sensor failed to disconnect")

        # Download using scan_id, should be unsuccessful
        numbered_download_response_3 = service_handles["download_scan"].call(DownloadScanRequest(scan_id=start_scan_id))
        test_helpers.assert_download_scan_service(test_case=self,
                                                  response=numbered_download_response_3,
                                                  success_expected=False)

        # Download using latest scan mode, should be unsuccessful
        numbered_download_response_4 = service_handles["download_scan"].call(DownloadScanRequest(scan_id=-1))
        test_helpers.assert_download_scan_service(test_case=self,
                                                  response=numbered_download_response_4,
                                                  success_expected=False)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_stop_capture', TestDownloadScan)
