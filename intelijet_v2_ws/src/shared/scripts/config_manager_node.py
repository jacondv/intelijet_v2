#!/usr/bin/env python3

import rospy, os, yaml
from shared.config_loader import reload_config, namespace_to_dict, CONFIG_PATH

def sanitize_for_ros(data):
    if isinstance(data, dict):
        return {k: sanitize_for_ros(v) for k, v in data.items()}
    elif isinstance(data, list):
        return [sanitize_for_ros(v) for v in data]
    elif data is None:
        return ""  # hoặc 0 tùy kiểu dữ liệu
    else:
        return data

import rospy
from std_msgs.msg import Empty  # hoặc String nếu muốn gửi path config
from shared.config_loader import reload_config, namespace_to_dict

def update_config_callback(msg):
    """Callback khi có message publish lên /update_config"""
    rospy.loginfo("Received /update_config signal → Reloading config...")
    cfg = sanitize_for_ros(reload_config())
    rospy.set_param("/system/config", namespace_to_dict(cfg))
    rospy.loginfo("System config updated on /system/config")

def main():
    rospy.init_node("config_manager")

    # Load lần đầu
    cfg = sanitize_for_ros(reload_config())
    rospy.set_param("/system/config", namespace_to_dict(cfg))
    rospy.loginfo("Initial config loaded to /system/config")

    # Subscriber để nhận tín hiệu reload
    rospy.Subscriber("/system/config/update", Empty, update_config_callback)

    rospy.loginfo("Config manager node is ready. Waiting for /update_config...")
    rospy.spin()  # giữ node chạy

if __name__ == "__main__":
    main()
