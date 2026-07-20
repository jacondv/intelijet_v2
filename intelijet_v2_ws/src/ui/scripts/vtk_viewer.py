# vtk_viewer.py
import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QGestureEvent, QPinchGesture, QPanGesture

import rospy


class QVTKWidget(QVTKRenderWindowInteractor):
    def __init__(self, parent=None, on_resize=None, vtk_viewer=None):
        super().__init__(parent)
        self._on_resize = on_resize
        self._vtk_viewer = vtk_viewer

        # 🔥 bắt buộc để nhận touch thật
        self.setAttribute(Qt.WA_AcceptTouchEvents, True)
        self.grabGesture(Qt.PinchGesture)
        self.grabGesture(Qt.PanGesture)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._on_resize:
            self._on_resize()

    def event(self, event):
        if event.type() == QGestureEvent.Gesture:
            return self._gesture_event(event)
        return super().event(event)


    def _gesture_event(self, event: QGestureEvent):

        pinch = event.gesture(Qt.PinchGesture)
        if pinch:
            self._handle_pinch(pinch)
            return True

        pan = event.gesture(Qt.PanGesture)
        if pan:
            self._handle_pan(pan)
            return True

        return False

    def _handle_pinch(self, pinch: QPinchGesture):
        if pinch.state() == Qt.GestureUpdated:
            factor = pinch.scaleFactor()

            if abs(factor - 1.0) < 0.02:   # ignore tiny pinch noise
                return
            
            # clamp cho mượt, tránh zoom điên
            factor = max(0.9, min(1.1, factor))

            if self._vtk_viewer:
                self._vtk_viewer.on_pinch_zoom(factor)

    def _handle_pan(self, pan: QPanGesture):

        if pan.state() == Qt.GestureUpdated:
            delta = pan.delta()

            dx = delta.x()
            dy = delta.y()

            # ignore noise nhỏ
            if abs(dx) < 1 and abs(dy) < 1:
                return

            # scale để control tốc độ drag
            dx *= 0.5
            dy *= 0.5

            if self._vtk_viewer:
                self._vtk_viewer.on_pan(dx, dy)

class VTKViewer:
    def __init__(self, parent_widget: QWidget):
        self.parent_widget = parent_widget
        self.renderer = vtk.vtkRenderer()
        self.current_actor = None
        self.box_widget = None
        self.initial_camera_state = None

        # ----- VTK widget -----
        self.vtkWidget = QVTKWidget(parent_widget, on_resize=None, vtk_viewer=self)

        layout = parent_widget.layout()
        if layout is None:
            layout = QVBoxLayout(parent_widget)
            parent_widget.setLayout(layout)
        layout.addWidget(self.vtkWidget)

        self.vtkWidget.GetRenderWindow().AddRenderer(self.renderer)
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()

        # ----- Camera -----
        cam = self.renderer.GetActiveCamera()
        cam.SetPosition(-1, 0, 0)
        cam.SetFocalPoint(0, 0, 0)
        cam.SetViewUp(0, 0, 1)
        self.renderer.ResetCameraClippingRange()

        # Trackball camera
        style = vtk.vtkInteractorStyleTrackballCamera()
        self.iren.SetInteractorStyle(style)


        # ----- Axes orientation -----
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(1.0, 1.0, 1.0)
        axes.AxisLabelsOn()

        self.orientation_widget = vtk.vtkOrientationMarkerWidget()
        self.orientation_widget.SetOrientationMarker(axes)
        self.orientation_widget.SetInteractor(self.vtkWidget)
        self.orientation_widget.SetViewport(0.0, 0.0, 0.2, 0.2)
        self.orientation_widget.EnabledOn()
        self.orientation_widget.InteractiveOff()

        self.vtkWidget.Initialize()
        self.vtkWidget.Start()

    #Zoom function
    def on_pinch_zoom(self, scale_factor):
        cam = self.renderer.GetActiveCamera()

        if cam.GetParallelProjection():
            # orthographic
            scale = cam.GetParallelScale()
            cam.SetParallelScale(scale / scale_factor)
        else:
            # perspective
            cam.Dolly(scale_factor)
            self.renderer.ResetCameraClippingRange()

        self.vtkWidget.GetRenderWindow().Render()

    def on_pan(self, dx, dy):
        cam = self.renderer.GetActiveCamera()

        # hệ số điều khiển tốc độ drag
        sensitivity = 0.2

        if cam.GetParallelProjection():
            # 🟢 Orthographic: pan = translate view bằng rotate nhẹ
            cam.Azimuth(-dx * sensitivity)
            cam.Elevation(dy * sensitivity)

            self.renderer.ResetCameraClippingRange()

        else:
            # 🔵 Perspective: orbit camera quanh target
            cam.Azimuth(-dx * sensitivity)
            cam.Elevation(dy * sensitivity)

            self.renderer.ResetCameraClippingRange()

        self.vtkWidget.GetRenderWindow().Render()

    # ------------------ Camera ------------------
    def _save_camera_state(self):
        cam = self.renderer.GetActiveCamera()
        self.initial_camera_state = {
            "position": cam.GetPosition(),
            "focal_point": cam.GetFocalPoint(),
            "view_up": cam.GetViewUp(),
            "view_angle": cam.GetViewAngle(),
            "parallel_scale": cam.GetParallelScale(),
        }

        # Box / Actor transform
        if self.current_actor:
            t = vtk.vtkTransform()
            if self.current_actor.GetUserTransform():
                t.DeepCopy(self.current_actor.GetUserTransform())
            else:
                t.Identity()
            self.initial_actor_transform = vtk.vtkTransform()
            self.initial_actor_transform.DeepCopy(t)

    def restore_initial_view(self):
        if not self.initial_camera_state:
            return
        cam = self.renderer.GetActiveCamera()
        s = self.initial_camera_state
        cam.SetPosition(*s["position"])
        cam.SetFocalPoint(*s["focal_point"])
        cam.SetViewUp(*s["view_up"])
        cam.SetViewAngle(s["view_angle"])
        cam.SetParallelScale(s["parallel_scale"])
        self.renderer.ResetCameraClippingRange()

            # ---- Actor / Box transform ----
        if self.current_actor and hasattr(self, "initial_actor_transform"):
            self.current_actor.SetUserTransform(self.initial_actor_transform)
            # Cập nhật box widget
            if self.box_widget:
                self.box_widget.PlaceWidget()

        self.vtkWidget.GetRenderWindow().Render()


    # ------------------ Box Widget ------------------
    def _enable_box_widget(self):
        if self.box_widget:
            self.box_widget.Off()
            self.box_widget = None

        box = vtk.vtkBoxWidget()
        box.SetInteractor(self.iren)
        box.SetPlaceFactor(3)
        box.SetProp3D(self.current_actor)
        box.PlaceWidget()
        box.ScalingEnabledOff()
        box.GetOutlineProperty().SetOpacity(0)
        box.OutlineCursorWiresOff()
        box.GetHandleProperty().SetPointSize(1)


        def on_interact(caller, event):
            t = vtk.vtkTransform()
            box.GetTransform(t)
            self.current_actor.SetUserTransform(t)
            self.vtkWidget.GetRenderWindow().Render()

        box.AddObserver("InteractionEvent", on_interact)
        box.On()
        self.box_widget = box

    # ------------------ Update cloud ------------------
    def update(self, polydata):
        if not polydata:
            return

        vertex_filter = vtk.vtkVertexGlyphFilter()
        vertex_filter.SetInputData(polydata)
        vertex_filter.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(vertex_filter.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetPointSize(3)

        if self.current_actor:
            self.renderer.RemoveActor(self.current_actor)

        self.current_actor = actor
        self.renderer.AddActor(actor)
        self.renderer.ResetCamera()

        # lưu camera LẦN ĐẦU
        if self.initial_camera_state is None:
            self._save_camera_state()

        self.vtkWidget.GetRenderWindow().Render()
        self._enable_box_widget()



# How to use touch screen for zoom
