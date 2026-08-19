# ui/tests/test_notification_center.py
"""Standalone tests for NotificationCenter - no ROS/rospy needed.

Run with:  python3 -m pytest ui/src/ui/tests/test_notification_center.py
       or:  python3 ui/src/ui/tests/test_notification_center.py
"""
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from PyQt5.QtWidgets import QApplication  # noqa: E402

_app = QApplication.instance() or QApplication(sys.argv)

import ui.notification_center as notification_center  # noqa: E402
from ui.notification_center import NotificationCenter  # noqa: E402


def _fresh_log_dir():
    # NotificationCenter.__init__ restores today's already-persisted log on
    # startup (reads notification_center.LOG_DIR). Point it at a brand new
    # throwaway directory before each test so: (a) real data/logs/<today>
    # entries from actually running the app don't leak in, and (b) one
    # test's push()es (written to today's file) don't leak into the next
    # test in this module.
    notification_center.LOG_DIR = tempfile.mkdtemp(prefix="intelijet_test_logs_")


def test_max_history_cap():
    _fresh_log_dir()
    nc = NotificationCenter(max_history=50, dedup_window=0)
    for i in range(100):
        nc.push("src", f"message {i}", "info")
    assert len(nc.history()) == 50
    # Oldest should have been evicted - last item in history is the newest.
    assert nc.history()[-1]["message"] == "message 99"


def test_dedup_within_window():
    _fresh_log_dir()
    nc = NotificationCenter(max_history=50, dedup_window=5)
    nc.push("src", "same message", "info")
    nc.push("src", "same message", "info")
    nc.push("src", "same message", "info")
    assert len(nc.history()) == 1


def test_dedup_expires_after_window():
    _fresh_log_dir()
    nc = NotificationCenter(max_history=50, dedup_window=0.05)
    nc.push("src", "same message", "info")
    time.sleep(0.1)
    nc.push("src", "same message", "info")
    assert len(nc.history()) == 2


def test_error_pins_label_against_info():
    _fresh_log_dir()
    nc = NotificationCenter(pin_seconds=10)
    seen = []
    nc.label_changed.connect(lambda text, level: seen.append((text, level)))

    nc.push("device", "lidar disconnected", "error")
    assert seen[-1] == ("lidar disconnected", "error")

    # An info push arriving while pinned must NOT change the label.
    nc.push_transient("routine status tick", "info")
    assert seen[-1] == ("lidar disconnected", "error")

    # A warning/error can still override even while pinned.
    nc.push("device", "encoder disconnected", "error")
    assert seen[-1] == ("encoder disconnected", "error")


def test_transient_not_added_to_history():
    _fresh_log_dir()
    nc = NotificationCenter()
    nc.push_transient("COMPARE [====      ] 40%", "info")
    assert len(nc.history()) == 0


def run_all():
    tests = [v for k, v in globals().items() if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print(f"OK: {t.__name__}")
    print(f"\n{len(tests)} tests passed.")


if __name__ == "__main__":
    run_all()
