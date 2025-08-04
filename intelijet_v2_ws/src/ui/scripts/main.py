#!/usr/bin/env python3
import sys
import vtk
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QFrame, QSizePolicy, QWidget
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from ui.pps_ui import Ui_Frame  # Import class từ file pps_ui.py
from ui.utils import ros_pointcloud2_to_o3d_to_vtk_polydata_voxel
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from PyQt5.QtCore import pyqtSignal, pyqtSlot
import threading
import subprocess
import os

# from pps.utils import load_config
from shared.config_loader import CONFIG as cfg


class RosThread(threading.Thread):
    def __init__(self, cloud_received_signal, ui_send_cmd_signal):
        super(RosThread, self).__init__()
        self.daemon = True  
        self.cloud_received_signal = cloud_received_signal
        self.ui_send_cmd_signal = ui_send_cmd_signal

    def run(self):
        rospy.init_node("gui_node", anonymous=True, disable_signals=True)
        self.cmd_pub = rospy.Publisher(HMI_CMD_TOPIC, String, queue_size=1)
        rospy.Subscriber(PRE_SCAN_CLOUD_TOPIC, PointCloud2, self.cloud_received_signal_callback)
        rospy.Subscriber(POST_SCAN_CLOUD_TOPIC, PointCloud2, self.cloud_received_signal_callback)
        rospy.Subscriber(CLOUD_COMPARED_TOPIC, PointCloud2, self.cloud_received_signal_callback)
        rospy.spin()

    def cloud_received_signal_callback(self, msg):
        # Đẩy msg về Qt bằng signal
        self.cloud_received_signal.emit(msg)

    def send_command(self, text):
        if hasattr(self, 'cmd_pub'):
            self.cmd_pub.publish(String(data=text))    

class App(QWidget):

    cloud_received_signal = pyqtSignal(object)
    ui_send_cmd_signale = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.ui = Ui_Frame()
        self.ui.setupUi(self)  # Gán các widget đã thiết kế vào self
	
        self.ros_thread = RosThread(self.cloud_received_signal, self.ui_send_cmd_signale)
        self.cloud_received_signal.connect(self.update_pointcloud)
        self.ui_send_cmd_signale.connect(self.ros_thread.send_command)
        self.ros_thread.start()
        
        self.current_actor = None
        self.vl = self.ui.cloudFrame.layout()#QVBoxLayout(self.ui.cloudFrame)
        self.vl.setContentsMargins(0, 0, 0, 0)
        self.vl.setSpacing(0)

        self.vtkWidget = QVTKRenderWindowInteractor(self.ui.cloudFrame)
        
        self.vl.addWidget(self.vtkWidget)

	
        # Create a VTK renderer
        self.renderer = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.renderer)

        self.vtkWidget.Initialize()
        self.vtkWidget.Start()

        self.showFullScreen()
        self.vtkWidget.resize(self.ui.cloudFrame.size())  # Ép nó tràn ra

        self.ui.btnPreScan.clicked.connect(self.start_prescan)
        self.ui.btnPostScan.clicked.connect(self.start_postscan)
        self.ui.btnCompare.clicked.connect(self.start_compare)

        self.ui.btnCancel.clicked.connect(self.on_cancel)

        self.ui.btnOpenScanner.clicked.connect(self.open_scanner)
        self.ui.btnCloseScanner.clicked.connect(self.close_scanner)
        self.ui.btnStop.clicked.connect(self.stop_scanner)
    
    def start_prescan(self):
        # self.cmd_pub.publish(String("start_prescan"))
        # self.status_label.setText("Requested PreScan...")
        self.ui_send_cmd_signale.emit("start_prescan")

    def start_postscan(self):
        self.ui_send_cmd_signale.emit("start_postscan")
        # self.status_label.setText("Requested PostScan...")

    def start_compare(self):
        self.ui_send_cmd_signale.emit("start_compare")
        # self.status_label.setText("Requested Compare...")        
    
    def cloud_callback(self, msg):
        rospy.logwarn("Received pointcloud")
        self.cloud_received.emit(msg)

    def open_scanner(self):
        # self.cmd_pub.publish(String("open_scanner"))
        # self.status_label.setText("Opening Scanner...")
        self.ui_send_cmd_signale.emit("open_scanner")

    def close_scanner(self):
        self.ui_send_cmd_signale.emit("close_scanner")

    def stop_scanner(self): 
        self.ui_send_cmd_signale.emit("stop_scanner")
    
    

    @pyqtSlot(object)
    def update_pointcloud(self, msg):
        rospy.logwarn("Updating pointcloud in VTK widget")
        polydata = ros_pointcloud2_to_o3d_to_vtk_polydata_voxel(msg)

        vertex_filter = vtk.vtkVertexGlyphFilter()
        vertex_filter.SetInputData(polydata)
        vertex_filter.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(vertex_filter.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetPointSize(1)

        if self.current_actor is not None:
            self.renderer.RemoveActor(self.current_actor)

        self.current_actor = actor
        self.renderer.AddActor(actor)
        self.renderer.ResetCamera()
        self.vtkWidget.GetRenderWindow().Render()


    def on_cancel(self):
        # Example point cloud

        import open3d as o3d
        import numpy as np
        
        #     # Hardcoded path
        self.vtkWidget.resize(self.ui.cloudFrame.size())
        path = "/mnt/c/work/projects/pointcloud/post_scan_0_20250530_130626_afterporcess.ply"

        reader = vtk.vtkPLYReader()
        reader.SetFileName(path)
        reader.Update()

        polydata = reader.GetOutput()

        vertex_filter = vtk.vtkVertexGlyphFilter()
        vertex_filter.SetInputData(polydata)
        vertex_filter.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(vertex_filter.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetPointSize(1)

        self.renderer.AddActor(actor)
        self.renderer.ResetCamera()

if __name__ == "__main__":

    from PyQt5.QtWidgets import QWidget

    # Load parameters befor work.
    
    # cfg = load_config(default_filename="../config/commond.yaml") # intelijet_v2_ws/src/config/commond.yaml
    # HMI_CMD_TOPIC = cfg.HMI_CMD_TOPIC # config.get('HMI_CMD_TOPIC')
    # PRE_SCAN_CLOUD_TOPIC = cfg.PRE_SCAN_CLOUD_TOPIC
    # POST_SCAN_CLOUD_TOPIC = cfg.PRE_SCAN_CLOUD_TOPIC
    # CLOUD_COMPARED_TOPIC = cfg.PRE_SCAN_CLOUD_TOPIC

    HMI_CMD_TOPIC = cfg.HMI_CMD_TOPIC # config.get('HMI_CMD_TOPIC')
    PRE_SCAN_CLOUD_TOPIC = cfg.PRE_SCAN_CLOUD_TOPIC
    POST_SCAN_CLOUD_TOPIC = cfg.PRE_SCAN_CLOUD_TOPIC
    CLOUD_COMPARED_TOPIC = cfg.PRE_SCAN_CLOUD_TOPIC
    print(cfg)
    app = QApplication(sys.argv)
    viewer = App()  # App kế thừa QWidget
    sys.exit(app.exec_())
