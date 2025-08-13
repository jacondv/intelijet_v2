#!/usr/bin/env python3

import sys
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Int32

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout,
    QPushButton, QLabel, QHBoxLayout
)
from PyQt5.QtCore import Qt

import open3d as o3d
import numpy as np

class HMIWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot HMI - BLK360 + Open3D")
        self.resize(600, 200)

        # ROS init
        rospy.init_node("hmi_gui_node", anonymous=True)
        self.cmd_pub = rospy.Publisher("/hmi/cmd", Int32, queue_size=1)

        # Subcribe point cloud (prescan và postscan cùng lắng nghe 1 topic hoặc tách riêng)
        rospy.Subscriber("/pre_scan_0", PointCloud2, self.cloud_callback)
        rospy.Subscriber("/pos_scan_0", PointCloud2, self.cloud_callback)

        # Layout
        layout = QVBoxLayout()
        btn_layout = QHBoxLayout()

        self.btn_prescan = QPushButton("Start PreScan")
        self.btn_postscan = QPushButton("Start PostScan")
        self.btn_compare = QPushButton("Compare")

        self.status_label = QLabel("Waiting for commands...")
        self.status_label.setAlignment(Qt.AlignCenter)

        self.btn_prescan.clicked.connect(self.start_prescan)
        self.btn_postscan.clicked.connect(self.start_postscan)
        self.btn_compare.clicked.connect(self.start_compare)

        btn_layout.addWidget(self.btn_prescan)
        btn_layout.addWidget(self.btn_postscan)
        btn_layout.addWidget(self.btn_compare)

        layout.addLayout(btn_layout)
        layout.addWidget(self.status_label)
        self.setLayout(layout)

    def start_prescan(self):
        self.cmd_pub.publish(String("start_prescan"))
        self.status_label.setText("Requested PreScan...")

    def start_postscan(self):
        self.cmd_pub.publish(String("start_postscan"))
        self.status_label.setText("Requested PostScan...")

    def start_compare(self):
        self.cmd_pub.publish(String("start_compare"))
        self.status_label.setText("Requested Compare...")        
    
    def cloud_callback(self, msg):
        rospy.loginfo("Received pointcloud")
        self.status_label.setText("Pointcloud received. Opening viewer...")
    

def main():
    app = QApplication(sys.argv)
    window = HMIWidget()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
