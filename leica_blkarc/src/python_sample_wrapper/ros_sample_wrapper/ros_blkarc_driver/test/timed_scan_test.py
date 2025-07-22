#!/usr/bin/env python3

PKG = 'ros_blkarc_driver'

import actionlib
import rospy
import test_helpers
import time
import unittest

from actionlib_msgs.msg import GoalStatus
from ros_blkarc_msgs.srv import CaptureRequest
from ros_blkarc_msgs.msg import TimedScanAction, TimedScanFeedback, TimedScanGoal
from std_srvs.srv import TriggerRequest


class TestTimedScan(unittest.TestCase):

    def __init__(self, methodName="runTest") -> None:
        super(TestTimedScan, self).__init__(methodName=methodName)

    def setUp(self):
        self.scan_time = 10
        self.counter = 0
        self.expected_feedback_values = [i + 1 for i in range(self.scan_time)]

    def feedback_callback(self, feedback: TimedScanFeedback):
        # Check that the feedback messages sent by the timed scan action contains the correct time elapsed value
        self.assertEqual(
            feedback.time_elapsed_seconds, self.expected_feedback_values[self.counter],
            f"Feedback message's time_elapsed_seconds is {feedback.time_elapsed_seconds}, expected f{self.expected_feedback_values[self.counter]} instead"
        )
        self.counter += 1

    # Test if timed scan action behaves correctly
    def test_timed_scan(self) -> None:
        # Initialise services and ensure device is ready
        sensor_name = rospy.get_param(param_name="sensor_name", default=None)
        device_is_ready, service_handles = test_helpers.get_all_service_handles_and_ensure_device_is_idle(
            sensor_name=sensor_name)
        self.assertTrue(device_is_ready,
                        "Device is not in IDLE state at the start of test, this may cause unexpected test results")

        # Initialise action client
        timed_scan_action_client = actionlib.SimpleActionClient(ns=f"{sensor_name}/timed_scan",
                                                                ActionSpec=TimedScanAction)
        timed_scan_action_client.wait_for_server()

        # Disconnect
        disconnect_response = service_handles["disconnect"].call(TriggerRequest())
        self.assertTrue(disconnect_response.success, "Sensor failed to disconnect")

        # Try to call action, should be unsuccessful
        timed_scan_action_client.send_goal(goal=TimedScanGoal(scan_time_seconds=10))
        timed_scan_action_client.wait_for_result()
        result_1 = timed_scan_action_client.get_result()
        state_1 = timed_scan_action_client.get_state()
        self.assertFalse(result_1.success, "Expected action to fail, got success instead")
        self.assertEqual(result_1.scan_id, -1, f"Expected scan_id to be -1, got {result_1.scan_id} instead")
        self.assertEqual(state_1, GoalStatus.ABORTED,
                         f"Expected action state to be {GoalStatus.ABORTED} (ABORTED), got {state_1} instead")

        # Connect
        connect_response = service_handles["connect"].call(TriggerRequest())
        self.assertTrue(connect_response.success, "Sensor failed to connect")

        # Start capture
        start_response = service_handles["start_capture"].call(CaptureRequest())
        start_scan_id = start_response.scan_id
        test_helpers.assert_capture_service(test_case=self, response=start_response, success_expected=True)

        # Wait for a bit
        time.sleep(1.0)

        # Try to call action, should be unsuccessful
        timed_scan_action_client.send_goal(goal=TimedScanGoal(scan_time_seconds=10))
        timed_scan_action_client.wait_for_result()
        result_2 = timed_scan_action_client.get_result()
        state_2 = timed_scan_action_client.get_state()
        self.assertFalse(result_2.success, "Expected action to fail, got success instead")
        self.assertEqual(result_2.scan_id, -1, f"Expected scan_id to be -1, got {result_2.scan_id} instead")
        self.assertEqual(state_2, GoalStatus.ABORTED,
                         f"Expected action state to be {GoalStatus.ABORTED} (ABORTED), got {state_1} instead")

        # Stop capture
        stop_response = service_handles["stop_capture"].call(CaptureRequest())
        stop_scan_id = stop_response.scan_id
        test_helpers.assert_capture_service(test_case=self, response=stop_response, success_expected=True)

        # Make sure that scan_id matches the one provided by the start scan response (start/stop capture works as expected)
        self.assertEqual(start_scan_id, stop_scan_id,
                         f"Start scan_id {start_scan_id} is different from stop scan_id {stop_scan_id}")

        # Wait for device to be ready to scan again
        test_helpers.wait_till_blk_arc_is_idle(get_device_state_handle=service_handles["get_device_state"])

        # Try to call action, should be successful. We also verify that the feedback messages are correct and are of the correct number
        # using the function feedback_callback
        timed_scan_action_client.send_goal(goal=TimedScanGoal(scan_time_seconds=self.scan_time),
                                           done_cb=None,
                                           active_cb=None,
                                           feedback_cb=self.feedback_callback)
        timed_scan_action_client.wait_for_result()
        result_3 = timed_scan_action_client.get_result()
        state_3 = timed_scan_action_client.get_state()
        self.assertTrue(result_3.success, "Expected action to succeed, failed instead")
        self.assertNotEqual(result_3.scan_id, -1, "Expected scan_id not to be -1")
        self.assertEqual(state_3, GoalStatus.SUCCEEDED,
                         f"Expected action state to be {GoalStatus.SUCCEEDED} (SUCCEEDED), got {state_3} instead")
        self.assertEqual(self.counter, self.scan_time,
                         f"Expected counter ({self.counter}) to equal scan_time ({self.scan_time})")

        # Wait for device to be ready to scan again
        test_helpers.wait_till_blk_arc_is_idle(get_device_state_handle=service_handles["get_device_state"])

        # Try to call action and then cancel, check for correct results
        timed_scan_action_client.send_goal(goal=TimedScanGoal(scan_time_seconds=self.scan_time))
        time.sleep(10)    # Need to cater a bit of time for the sensor to start scanning
        timed_scan_action_client.cancel_goal()
        timed_scan_action_client.wait_for_result()
        result_4 = timed_scan_action_client.get_result()
        state_4 = timed_scan_action_client.get_state()
        self.assertTrue(result_4.success, "Expected action to succeed, failed instead")
        self.assertNotEqual(result_4.scan_id, -1, "Expected scan_id not to be -1")
        self.assertEqual(state_4, GoalStatus.PREEMPTED,
                         f"Expected action state to be {GoalStatus.PREEMPTED} (PREEMPTED), got {state_4} instead")


if __name__ == '__main__':
    import rostest
    rospy.init_node("test_timed_scan_node")
    rostest.rosrun(PKG, 'test_timed_scan', TestTimedScan)
