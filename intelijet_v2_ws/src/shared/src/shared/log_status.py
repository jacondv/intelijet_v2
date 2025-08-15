import rospy
import json
from datetime import datetime

def log_status(name=None, status=None, value=None, message=None, node=None):
    """
    Tạo dict chuẩn và log lên /rosout dưới dạng JSON.
    """
    msg_dict = {
        "name": name, # Class msg to show in UI
        "type": 0,
        "status": status,
        "value": value,
        "message": message,
        "node": node if node else rospy.get_name(),
        "timestamp": datetime.now().isoformat(),
    }
    if "[WARN]" in message:
        msg_dict["type"] = 1
        rospy.logwarn(json.dumps(msg_dict))
    elif "[ERROR]" in message:
        msg_dict["type"] = 2
        rospy.logerr(json.dumps(msg_dict))
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

    # Luôn thêm tên node và timestamp từ msg

    return data

# log_status(
#     name=None, 
#     status=None, 
#     value=None, 
#     message=None, 
#     node=None
# )