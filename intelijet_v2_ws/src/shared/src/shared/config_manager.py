class ConfigNode:
    """Namespace-like object có thể chứa nested dict"""
    def __init__(self, data=None):
        if data:
            for k, v in data.items():
                if isinstance(v, dict):
                    v = ConfigNode(v)
                setattr(self, k, v)

    def update(self, data):
        """Update recursively"""
        for k, v in data.items():
            if hasattr(self, k):
                attr = getattr(self, k)
                if isinstance(attr, ConfigNode) and isinstance(v, dict):
                    attr.update(v)
                else:
                    setattr(self, k, v)
            else:
                if isinstance(v, dict):
                    v = ConfigNode(v)
                setattr(self, k, v)

    def to_dict(self):
        """Convert back to dict recursively"""
        result = {}
        for k, v in self.__dict__.items():
            if isinstance(v, ConfigNode):
                result[k] = v.to_dict()
            else:
                result[k] = v
        return result

import yaml, os

CONFIG = ConfigNode()  # singleton

def load_config(*files):
    merged = {}
    for f in files:
        with open(f, 'r') as fh:
            data = yaml.safe_load(fh)
            merged.update(data)  # merge dict bình thường
    return merged

def reload_config():
    global CONFIG
    data = load_config("commond.yaml", "lidar.yaml", "runtime.yaml")
    CONFIG.update(data)  # giữ reference cũ
    print("Config reloaded:", CONFIG.to_dict())

if __name__ == "__main__":
    reload_config()
    print(CONFIG.to_dict())