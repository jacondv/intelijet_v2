from types import SimpleNamespace
from unittest.mock import patch

import pytest

from shared.device_monitor import Monitor


@pytest.fixture(autouse=True)
def _no_ros_runtime():
    """Monitor.__init__/update_status touch rospy.Timer/rospy.Time.now(),
    which need a live node (rospy.init_node()) connected to a roscore.
    These tests only exercise the on_transition bookkeeping logic, so stub
    those two calls out instead of standing up a real ROS runtime."""
    with patch("shared.device_monitor.rospy.Timer"), \
         patch("shared.device_monitor.rospy.Time") as mock_time:
        mock_time.now.return_value = 0.0
        yield


class _FakeMonitor(Monitor):
    """Skips the real subscriber/thread wiring _setup() would normally do -
    tests drive update_status() directly instead of via a live ROS topic."""

    def _setup(self, cfg):
        pass

    def check_status(self, event):
        pass


def _make_cfg(name="dev", timeout=3.0, mode="continuous"):
    return SimpleNamespace(name=name, timeout=timeout, mode=mode)


def test_on_transition_not_called_on_first_update():
    transitions = []
    m = _FakeMonitor(_make_cfg(), on_transition=lambda *args: transitions.append(args))

    # Constructor default is DISCONNECTED - the first real observation
    # (even if it differs) must not fire a transition.
    m.update_status("Connected")

    assert transitions == []


def test_on_transition_called_only_on_actual_change():
    transitions = []
    m = _FakeMonitor(_make_cfg(), on_transition=lambda *args: transitions.append(args))

    m.update_status("Connected")   # first observation, suppressed
    m.update_status("Connected")   # no change, must not fire
    m.update_status("Disconnected")  # real transition
    m.update_status("Disconnected")  # no change again

    assert transitions == [("dev", "Connected", "Disconnected")]


def test_no_on_transition_callback_is_a_safe_default():
    m = _FakeMonitor(_make_cfg())  # on_transition=None
    m.update_status("Connected")
    m.update_status("Disconnected")  # must not raise
