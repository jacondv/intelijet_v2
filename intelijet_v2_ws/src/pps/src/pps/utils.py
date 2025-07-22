import os
import yaml
import rospy
import rospkg

def load_config(default_filename="config/lidar.yaml", param_name="~lidar_config_file"):
    """
    Load configuration file in order of priority:
    1. ROS param ~config_file (absolute path or relative to package)
    2. Environment variable LIDAR_CONFIG
    3. Default file inside the current package (using rospkg)
    """
    try:
        # Check ROS param
        param_path = rospy.get_param(param_name, None)
        if param_path and os.path.isfile(param_path):
            rospy.loginfo(f"Loading config from ROS param: {param_path}")
            return yaml.safe_load(open(param_path, 'r'))


        # Default: use rospkg to locate caller's package
        caller_package = _guess_calling_package()
        if caller_package:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path(caller_package)
            default_path = os.path.join(pkg_path, default_filename)
            if os.path.isfile(default_path):
                rospy.loginfo(f"Loading config from default path: {default_path}")
                return yaml.safe_load(open(default_path, 'r'))

        raise FileNotFoundError("Config file not found via any method.")

    except Exception as e:
        rospy.logerr(f"[ConfigLoader] Failed to load config: {e}")
        return {}

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
