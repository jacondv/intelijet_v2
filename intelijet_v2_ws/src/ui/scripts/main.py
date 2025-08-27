#!/usr/bin/env python3
import sys, subprocess
import vtk
from PyQt5.QtWidgets import QApplication, QWidget, QMessageBox, QVBoxLayout, QLabel
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from vtkmodules.vtkInteractionStyle import vtkInteractorStyleTrackballCamera

from vtk.util import numpy_support
from PyQt5.QtCore import Qt
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow

# from ui.pps_ui import Ui_Frame  # Import class từ file pps_ui.py
from ui.setting_page_ui import Ui_setting_page
from ui.intelijet_ui import Ui_MainWindow as Ui_Frame

from ui.utils import ros_pointcloud2_to_o3d_to_vtk_polydata_voxel
from ui.update_data_utils import upload_data_to_ui, download_data_from_ui
from ui.handlers import *
import rospy
from sensor_msgs.msg import PointCloud2, JointState
import sensor_msgs.point_cloud2 as pc2

from std_msgs.msg import Int32
from PyQt5.QtCore import pyqtSignal, pyqtSlot

import threading
import numpy as np


from shared.pps_command import PPSCommand
from shared.log_status import unpack_log_status
from rosgraph_msgs.msg import Log


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

        rospy.Subscriber('/rosout', Log, self.rosout_callback)


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


    def rosout_callback(self,msg):
        # Lọc theo mức INFO
        data = unpack_log_status(msg)
        if data is not None:
            name = data.get("name")
            self.data_store[name] = data.get("message")
        

    def emit_ui_data_update(self, msg):
        # self.data_store["encoder_value_in_deg"] = random.random()

        self.ui_data_update.emit(self.data_store)



class App(QMainWindow):

    cloud_received_signal = pyqtSignal(object)
    ui_send_cmd_signale = pyqtSignal(int)
    ui_data_update = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.ui = Ui_Frame()
        self.ui.setupUi(self)  # Gán các widget đã thiết kế vào self

         # === Load setting UI ===
        self.setting_page_ui = Ui_setting_page()
        self.setting_page_widget = QWidget() 
        self.setting_page_ui.setupUi(self.setting_page_widget)
        # === Gán setting_page_widget vào tab_setting ===
        # Nếu tab_setting chưa có layout → tạo VBoxLayout

        self.ui.tab_mainview.setCurrentIndex(0)

        if self.ui.tab_setting.layout() is None:
            self.ui.tab_setting.setLayout(QVBoxLayout())
        self.ui.tab_setting.layout().addWidget(self.setting_page_widget)
	
        self.ros_thread = RosThread(self.cloud_received_signal, self.ui_send_cmd_signale, self.ui_data_update)

        self.cloud_received_signal.connect(self.update_pointcloud)
        self.ui_data_update.connect(self.update_data)

        self.ui_send_cmd_signale.connect(self.ros_thread.send_command)
        
        self.ros_thread.start()
        
        self.current_actor = None
        self.vl = self.ui.cloudFrame.layout()
        self.vl = QVBoxLayout(self.ui.cloudFrame)
        self.vl.setContentsMargins(0, 0, 0, 0)
        self.vl.setSpacing(0)

        self.vtkWidget = QVTKRenderWindowInteractor(self.ui.cloudFrame)
        
        self.vl.addWidget(self.vtkWidget)
        
	
        # Create a VTK renderer
        self.renderer = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.renderer)

        iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        self.style = vtkInteractorStyleTrackballCamera()
        iren.SetInteractorStyle(self.style)
        iren.AddObserver("EndInteractionEvent", self._stop_rotation)

        # === HIỂN THỊ TRỤC TỌA ĐỘ ===
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(1.0, 1.0, 1.0)  # độ dài X,Y,Z
        axes.AxisLabelsOn()
        axes.SetCylinderRadius(0.05)
        transform = vtk.vtkTransform()
        # transform.RotateY(90) 
        # transform.RotateX(-90)
        axes.SetUserTransform(transform)

        self.orientation_widget = vtk.vtkOrientationMarkerWidget()
        self.orientation_widget.SetOrientationMarker(axes)
        self.orientation_widget.SetInteractor(self.vtkWidget)
        self.orientation_widget.SetViewport(0.0, 0.0, 0.2, 0.2)  # góc trái dưới
        self.orientation_widget.EnabledOn()
        self.orientation_widget.InteractiveOn()

        self.vtkWidget.Initialize()
        self.vtkWidget.Start()

        # self.showFullScreen()
        self.showMaximized()
        self.vtkWidget.resize(self.ui.cloudFrame.size())  # Ép nó tràn ra

        self.ui.btnPreScan.released.connect(self.start_prescan)
        self.ui.btnPostScan.released.connect(self.start_postscan)
        self.ui.btnCompare.released.connect(self.start_compare)

        self.ui.btnCancel.released.connect(self.on_cancel)

        self.ui.btnOpenScanner.released.connect(self.open_scanner)
        self.ui.btnCloseScanner.released.connect(self.close_scanner)

        self.ui.btnShutdown.released.connect(self.on_shutdown)
        self.setting_page_ui.btnUpdateHousingParam.released.connect(lambda: btnUpdateHousingParam_handler(self))
        self.lblNotification = QLabel("Ready")
        self.lblNotification.setStyleSheet("margin-left: 5px;")  
        self.ui.statusbar.addWidget(self.lblNotification)

        upload_data_to_ui(self.ui.tab_setting)
        # self.__load_sample()

    def _stop_rotation(self,obj, ev):
        # self.style.StopState()
        # obj.GetRenderWindow().Render()
        pass

    def closeEvent(self, event):
        subprocess.call(["rosnode", "kill", "-a"])
        subprocess.call("pkill -f ros", shell=True)
        subprocess.call(["rosclean", "purge", "-y"])
        event.accept()  

    def on_shutdown(self):
        # Tạo QMessageBox không truyền parent → trở thành top-level
        msg = QMessageBox()
        msg.setWindowTitle("Confirmation")
        msg.setText("Are you sure you want to exit?")
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        msg.setDefaultButton(QMessageBox.No)

        # Chặn toàn bộ ứng dụng và luôn nổi trên top
        msg.setWindowModality(Qt.ApplicationModal)
        msg.setWindowFlags(msg.windowFlags() | Qt.WindowStaysOnTopHint)

        # Đẩy popup lên và kích hoạt
        msg.raise_()
        msg.activateWindow()

        # Hiển thị modal → block code tới khi user bấm
        reply = msg.exec_()

        if reply == QMessageBox.Yes:
            self.close()

    
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
        # Cancel job and stop scaner
        self.ui_send_cmd_signale.emit(PPSCommand.CLOSE_HOUSING.value)
   
    def on_cancel(self):
        self.ui_send_cmd_signale.emit(PPSCommand.CANCEL_JOB.value)


    def __load_sample(self):
        try:
                                
            # Example point cloud

            import open3d as o3d
            import numpy as np
            
            #     # Hardcoded path
            self.vtkWidget.resize(self.ui.cloudFrame.size())
            # path = "/mnt/c/work/projects/pointcloud/post_scan_0_20250530_130626_afterporcess.ply"
            path = "/root/intelijet_v2/post_scan_0_20250530_130626_afterporcess.ply"

            #Đọc PLY bằng Open3D
            pcd = o3d.io.read_point_cloud(path)
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors) 

            #  Chuyển sang vtkPolyData
            polydata = vtk.vtkPolyData()
            vtk_points = vtk.vtkPoints()
            vtk_points.SetData(numpy_support.numpy_to_vtk(points))
            polydata.SetPoints(vtk_points)

            # reader = vtk.vtkPLYReader()
            # reader.SetFileName(path)
            # reader.Update()

            # polydata = reader.GetOutput()

            # Chuyển màu sang vtkUnsignedCharArray
            # Open3D màu float [0,1], VTK cần uint8 [0,255]
            colors_uint8 = (colors * 255).astype(np.uint8)
            vtk_colors = numpy_support.numpy_to_vtk(colors_uint8)
            vtk_colors.SetNumberOfComponents(3)
            vtk_colors.SetName("Colors")
            polydata.GetPointData().SetScalars(vtk_colors)

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

        except Exception as e:
            rospy.logwarn(f"Load sample error: {e}")
              

    @pyqtSlot(dict)
    def update_data(self, data):
        try:
            if "encoder_value_in_deg" in data:
                encoder_value_in_deg = data["encoder_value_in_deg"]
                self.ui.lblEncoder.setText(f"{encoder_value_in_deg:.2f}")  # 2 chữ số thập phân
                
            if cfg.NOTIFICATION in data:            
                self.lblNotification.setText(data[cfg.NOTIFICATION])
                # self.ui.statusbar.showMessage(data[cfg.NOTIFICATION], 3000)
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

        if hasattr(self, 'current_actor') and self.current_actor is not None:
            self.renderer.RemoveActor(self.current_actor)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetPointSize(1)

        self.current_actor = actor
        self.renderer.AddActor(actor)
        self.renderer.ResetCamera()
        self.vtkWidget.GetRenderWindow().Render()







if __name__ == "__main__":

    from PyQt5.QtWidgets import QWidget

    # Load parameters befor work.
    
    HMI_CMD_TOPIC = "/hmi/cmd" # config.get('HMI_CMD_TOPIC')
    PRE_SCAN_CLOUD_TOPIC = cfg.PRE_SCAN_CLOUD_TOPIC
    POST_SCAN_CLOUD_TOPIC = cfg.POST_SCAN_CLOUD_TOPIC
    CLOUD_COMPARED_TOPIC = cfg.CLOUD_COMPARED_TOPIC

    # HMI_CMD_TOPIC = cfg.HMI_CMD_TOPIC # config.get('HMI_CMD_TOPIC')
    # PRE_SCAN_CLOUD_TOPIC = cfg.PRE_SCAN_TOPIC
    # POST_SCAN_CLOUD_TOPIC = cfg.POST_SCAN_TOPIC
    # CLOUD_COMPARED_TOPIC = cfg.CLOUD_COMPARED_TOPIC
    print(cfg)

    app = QApplication(sys.argv)
    viewer = App()  # App kế thừa QWidget
    sys.exit(app.exec_())
