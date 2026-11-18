"""User-facing notification channel - publishes a `Notification` message on
`cfg.NOTIFICATION_TOPIC` for the UI to display (see ui/scripts/ros_thread.py).

Deliberately independent of log_status.py: that module is for rosout debug
logging (rqt_console, `roslaunch` output), this one is for operator-facing
messages. Don't merge them - mixing "debug log line" and "user notification"
semantics back together is exactly what this replaces
(the old log_status(name=cfg.NOTIFICATION, ...) JSON-over-/rosout hack).
"""
import rospy
from shared.config_loader import CONFIG as cfg
from shared.error_codes import lookup as lookup_error_code
from shared.msg import Notification

_pub = None


def _get_publisher():
    global _pub
    if _pub is None:
        _pub = rospy.Publisher(cfg.NOTIFICATION_TOPIC, Notification, queue_size=10)
    return _pub


def notify(message=None, level=None, source=None, code=None):
    """
    level: "info" | "warning" | "error" (optional). If omitted, inferred from
    "[WARN]"/"[ERROR]" markers in `message`, same convention as the old
    log_status() helper - existing callers that don't pass `level` keep
    working unchanged.
    source: human-readable label for where this came from (e.g. "Scan",
    "Compare", "Housing"). Defaults to the publishing node's ROS name.
    code: stable error/warning code (see shared/error_codes.py), e.g.
    "SCAN-004". When given, `message`/`level` default from the registry
    entry if not passed explicitly - pass `message` too only when you want
    to add exception-specific detail on top of the registry's generic text.
    An unknown code is logged and otherwise ignored (message/level still
    come from the caller).
    """
    entry = lookup_error_code(code) if code else None
    if code and entry is None:
        rospy.logwarn(f"[notify] Unknown error code: {code}")

    if entry is not None:
        if message is None:
            message = entry["message"]
        if level is None:
            level = entry["level"]

    if level is None:
        if message and "[ERROR]" in message:
            level = "error"
        elif message and "[WARN]" in message:
            level = "warning"
        else:
            level = "info"

    _get_publisher().publish(Notification(
        source=source or rospy.get_name(),
        level=level,
        message=message,
        node=rospy.get_name(),
        stamp=rospy.Time.now(),
        code=code or "",
    ))
