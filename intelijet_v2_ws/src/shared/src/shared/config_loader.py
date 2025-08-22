import yaml
from types import SimpleNamespace
import os
import rospy
import rospkg

def _guess_calling_package():
    """
    Tries to guess the calling package by inspecting __file__ variable
    of the caller. Only works if caller is in a ROS package.
    """
    import inspect
    try:
        # Lấy đường dẫn file gọi hàm này
        frame = inspect.stack()[2]
        module_path = os.path.abspath(frame.filename)

        # Dò ngược từ module_path để tìm gói ROS
        while module_path != "/" and module_path:
            if os.path.exists(os.path.join(module_path, "package.xml")):
                return os.path.basename(module_path)
            module_path = os.path.dirname(module_path)

    except Exception:
        pass

    return None

def dict_to_namespace(d):
    if isinstance(d, dict):
        return SimpleNamespace(**{k: dict_to_namespace(v) for k, v in d.items()})
    elif isinstance(d, list):
        return [dict_to_namespace(v) for v in d]
    return d

def deep_merge(dict1, dict2):
    """Gộp dict2 vào dict1 (deep merge)."""
    for k, v in dict2.items():
        if k in dict1 and isinstance(dict1[k], dict) and isinstance(v, dict):
            deep_merge(dict1[k], v)
        else:
            dict1[k] = v
    return dict1

def load_config(*paths):
    merged = {}
    caller_package = _guess_calling_package()

    if caller_package:
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(caller_package)

    for path in paths:

        default_path = os.path.join(pkg_path, path)
        if os.path.isfile(default_path):
            rospy.logwarn(f"Loading config from default path: {default_path}")
            
        with open(default_path, "r") as f:
            data = yaml.safe_load(f)
            merged = deep_merge(merged, data)

    return dict_to_namespace(merged)


CONFIG = load_config("../config/commond.yaml", 
                     "../config/lidar.yaml")
