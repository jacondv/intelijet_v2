import rospy
import json
from datetime import datetime

_LEVEL_TO_TYPE = {"info": 0, "warning": 1, "error": 2}


def log_status(name=None, status=None, value=None, message=None, node=None, level=None):
    """
    Tạo dict chuẩn và log lên /rosout dưới dạng JSON.

    level: "info" | "warning" | "error" (optional). If omitted, severity is
    inferred from "[WARN]"/"[ERROR]" markers in `message`, same as before -
    existing callers that don't pass `level` keep working unchanged.
    """
    if level is None:
        if message and "[ERROR]" in message:
            level = "error"
        elif message and "[WARN]" in message:
            level = "warning"
        else:
            level = "info"

    msg_dict = {
        "name": name, # Class msg to show in UI
        "type": _LEVEL_TO_TYPE.get(level, 0),
        "level": level,
        "status": status,
        "value": value,
        "message": message,
        "node": node if node else rospy.get_name(),
        "timestamp": datetime.now().isoformat(),
    }

    if level == "error":
        rospy.logerr(json.dumps(msg_dict))
    elif level == "warning":
        rospy.logwarn(json.dumps(msg_dict))
    else:
        rospy.loginfo(json.dumps(msg_dict))


def unpack_log_status(msg):
    try:
        # Thử parse JSON từ msg.msg
        data = json.loads(msg.msg)
        if not isinstance(data, dict):
            return None
    except json.JSONDecodeError:
        return None

    # Old payloads (before `level` was added) only have "type": 0/1/2.
    if "level" not in data:
        data["level"] = {0: "info", 1: "warning", 2: "error"}.get(data.get("type"), "info")

    return data

# log_status(
#     name=None, 
#     status=None, 
#     value=None, 
#     message=None, 
#     node=None
# )