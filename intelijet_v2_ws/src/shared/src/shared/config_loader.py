import yaml
from types import SimpleNamespace
import os
import rospy
import rospkg

def get_config_dir():
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("pps") 
    pkg_path = os.path.dirname(pkg_path)
    config_dir = os.path.join(pkg_path, "config")
    return config_dir

CONFIG = None
CONFIG_FILE_NAME = "last_used.yaml"
CONFIG_PATH = os.path.join(get_config_dir(),CONFIG_FILE_NAME )

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


def namespace_to_dict(ns):
    if isinstance(ns, SimpleNamespace):
        return {k: namespace_to_dict(v) for k, v in vars(ns).items()}
    elif isinstance(ns, list):
        return [namespace_to_dict(v) for v in ns]
    return ns


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
    # caller_package = _guess_calling_package()

    # if caller_package:
    #     rospack = rospkg.RosPack()
    #     pkg_path = rospack.get_path(caller_package)

    config_dir = get_config_dir()

    for path in paths:

        config_path = os.path.join(config_dir, path)
        if os.path.isfile(config_path):
            rospy.loginfo(f"Loading config from default path: {config_path}")
            
        with open(config_path, "r") as f:
            data = yaml.safe_load(f)
            merged = deep_merge(merged, data)

    return dict_to_namespace(merged)


def save_config(config_obj, filename=CONFIG_FILE_NAME):
    """Lưu config object (SimpleNamespace) thành YAML."""

    data = namespace_to_dict(config_obj)

    config_dir = get_config_dir()
    config_path = os.path.join(config_dir, filename)
    with open(config_path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


def reload_config():
    global CONFIG
    config_dir = get_config_dir()
    config_path = os.path.join(config_dir, CONFIG_FILE_NAME)

    if os.path.isfile(config_path):
        new_config = load_config(CONFIG_FILE_NAME)
    else:
        new_config = load_config("commond.yaml", "lidar.yaml", "runtime.yaml")
        save_config(new_config, filename=CONFIG_FILE_NAME)

    if CONFIG is None:
        CONFIG = new_config
    else:
        # giữ reference cũ, update __dict__
        CONFIG.__dict__.clear()
        CONFIG.__dict__.update(new_config.__dict__)
    
    return CONFIG


reload_config()
if __name__ == "__main__":
    # Test loading and saving config
    # save_config(CONFIG)
    # reload_config()
    # print(CONFIG)
    pass