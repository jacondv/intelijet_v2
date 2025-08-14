#!/usr/bin/env python3
import sys
import vtk
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QFrame, QSizePolicy, QWidget
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from ui.pps_ui import Ui_Frame  # Import class từ file pps_ui.py
from ui.utils import ros_pointcloud2_to_o3d_to_vtk_polydata_voxel
import rospy
from sensor_msgs.msg import PointCloud2, JointState
from std_msgs.msg import String, Int32
from PyQt5.QtCore import pyqtSignal, pyqtSlot
import threading
import subprocess
import os
from shared.pps_command import PPSCommand
import random


# from pps.utils import load_config
from shared.config_loader import CONFIG as cfg


class RosThread(threading.Thread):
    def __init__(self, cloud_received_signal, ui_send_cmd_signal, ui_data_update):
        super(RosThread, self).__init__()
        self.daemon = True  
        self.cloud_received_signal = cloud_received_signal
        self.ui_send_cmd_signal = ui_send_cmd_signal

        self.ui_data_update = ui_data_update # Data update to UI
        self.data_store = {}

    def run(self):
        rospy.init_node("gui_node", anonymous=True, disable_signals=True)
        self.cmd_pub = rospy.Publisher(HMI_CMD_TOPIC, Int32, queue_size=1)
        rospy.Subscriber(PRE_SCAN_CLOUD_TOPIC, PointCloud2, self.cloud_received_signal_callback)
        rospy.Subscriber(POST_SCAN_CLOUD_TOPIC, PointCloud2, self.cloud_received_signal_callback)
        rospy.Subscriber(CLOUD_COMPARED_TOPIC, PointCloud2, self.cloud_received_signal_callback)

        # listening topic update infomation for UI.
        rospy.Subscriber("/joint_states", JointState, self.update_joint_states_status)

        rospy.Timer(rospy.Duration(1.0), self.emit_ui_data_update) # Update data 1Hz
        # rospy.Subscriber(HMI_CMD_TOPIC,Int32, self.update_hmi_cmd)

        rospy.spin()

    def cloud_received_signal_callback(self, msg):
        # Đẩy msg về Qt bằng signal
        self.cloud_received_signal.emit(msg)

    def send_command(self, cmd: PPSCommand):
        if hasattr(self, 'cmd_pub'):
            self.cmd_pub.publish(Int32(data=cmd))    
    
    def update_joint_states_status(self, msg):
        try:
            idx = msg.name.index(cfg.ENCODER_JOINT_NAME)
            current_encoder_value_in_degree = msg.position[idx] * 180 / 3.14

            self.data_store["encoder_value_in_deg"] = current_encoder_value_in_degree

        except ValueError:

            rospy.logwarn(f"Joint {cfg.ENCODER_JOINT_NAME} not found in JointState")

    def emit_ui_data_update(self,msg):
        self.data_store["encoder_value_in_deg"] = random.random()
        self.ui_data_update.emit(self.data_store)



        



class App(QWidget):

    cloud_received_signal = pyqtSignal(object)
    ui_send_cmd_signale = pyqtSignal(int)
    ui_data_update = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.ui = Ui_Frame()
        self.ui.setupUi(self)  # Gán các widget đã thiết kế vào self
	
        self.ros_thread = RosThread(self.cloud_received_signal, self.ui_send_cmd_signale, self.ui_data_update)

        self.cloud_received_signal.connect(self.update_pointcloud)
        self.ui_data_update.connect(self.update_data)

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
        self.ui_send_cmd_signale.emit(PPSCommand.START_PRESCAN.value)

    def start_postscan(self):
        self.ui_send_cmd_signale.emit(PPSCommand.START_POSTSCAN.value)
        # self.status_label.setText("Requested PostScan...")

    def start_compare(self):
        self.ui_send_cmd_signale.emit(PPSCommand.START_COMPARE.value)
        # self.status_label.setText("Requested Compare...")        
    
    def cloud_callback(self, msg):
        rospy.logwarn("Received pointcloud")
        self.cloud_received.emit(msg)

    def open_scanner(self):
        # self.cmd_pub.publish(String("open_scanner"))
        # self.status_label.setText("Opening Scanner...")
        self.ui_send_cmd_signale.emit(PPSCommand.OPEN_HOUSING.value)

    def close_scanner(self):
        self.ui_send_cmd_signale.emit(PPSCommand.CLOSE_HOUSING.value)

    def stop_scanner(self): 
        self.ui_send_cmd_signale.emit(PPSCommand.PAUSE_HOUSING.value)
    
    def on_cancel(self):
        self.ui_send_cmd_signale.emit(PPSCommand.CANCEL_JOB.value)
        
        # Example point cloud

        # import open3d as o3d
        # import numpy as np
        
        # #     # Hardcoded path
        # self.vtkWidget.resize(self.ui.cloudFrame.size())
        # path = "/mnt/c/work/projects/pointcloud/post_scan_0_20250530_130626_afterporcess.ply"

        # reader = vtk.vtkPLYReader()
        # reader.SetFileName(path)
        # reader.Update()

        # polydata = reader.GetOutput()

        # vertex_filter = vtk.vtkVertexGlyphFilter()
        # vertex_filter.SetInputData(polydata)
        # vertex_filter.Update()

        # mapper = vtk.vtkPolyDataMapper()
        # mapper.SetInputConnection(vertex_filter.GetOutputPort())

        # actor = vtk.vtkActor()
        # actor.SetMapper(mapper)
        # actor.GetProperty().SetPointSize(1)

        # self.renderer.AddActor(actor)
        # self.renderer.ResetCamera()    

    @pyqtSlot(dict)
    def update_data(self, data):
        try:
            encoder_value_in_deg = data["encoder_value_in_deg"]
            self.ui.lblEncoder.setText(f"{encoder_value_in_deg:.2f}")  # 2 chữ số thập phân
        except Exception as e:
            rospy.logwarn(f"update_data error: {e}")

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



if __name__ == "__main__":

    from PyQt5.QtWidgets import QWidget

    # Load parameters befor work.
    
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