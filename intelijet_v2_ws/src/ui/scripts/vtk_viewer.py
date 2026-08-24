# vtk_viewer.py
import numpy as np
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
    # "Zoom Center" - recomputed fresh against whichever cloud is
    # CURRENTLY loaded (previously this replayed a camera state captured
    # once from the very FIRST cloud ever shown in the app session, which
    # no longer matched a later, differently-sized/positioned cloud).
    # Camera-only: this has no effect on compare or its results - compare
    # works off the raw cloud data, never off this viewport's
    # camera/actor state.
    ZOOM_CENTER_DOLLY_METERS = 2.0

    def restore_initial_view(self):
        if not self.current_actor:
            return
        cam = self.renderer.GetActiveCamera()

        # Undo any box-widget drag back to identity, then re-fit the
        # camera to the actor's OWN bounds specifically (ResetCamera()
        # with no args scans every visible prop in the renderer, which
        # includes the box widget's outline - invisible at opacity 0, but
        # still a real prop with real bounds set 3x larger than the cloud
        # via SetPlaceFactor(3) in _enable_box_widget - so it was fitting
        # the camera to a box 3x too big, making the cloud look small and
        # far instead of properly framed).
        self.current_actor.SetUserTransform(vtk.vtkTransform())
        self.renderer.ResetCamera(self.current_actor.GetBounds())

        # Move the camera ZOOM_CENTER_DOLLY_METERS closer along its own
        # view direction (camera -> focal point) so the cloud reads
        # closer/larger than a plain bounding-box fit, not just centered.
        pos = np.array(cam.GetPosition())
        focal = np.array(cam.GetFocalPoint())
        direction = focal - pos
        distance = np.linalg.norm(direction)
        if distance > self.ZOOM_CENTER_DOLLY_METERS:
            cam.SetPosition(*(pos + direction / distance * self.ZOOM_CENTER_DOLLY_METERS))

        self.renderer.ResetCameraClippingRange()

        if self.box_widget:
            self.box_widget.PlaceWidget()

        self.vtkWidget.GetRenderWindow().Render()


    # ------------------ Box Widget ------------------
    def _enable_box_widget(self):
        # Box widget itself is created once and reused; only SetProp3D()
        # (current_actor is a new vtkActor every update()) and PlaceWidget()
        # need to run each time. Previously this tore down and rebuilt the
        # whole widget + observer on every single cloud update.
        if self.box_widget is None:
            box = vtk.vtkBoxWidget()
            box.SetInteractor(self.iren)
            box.SetPlaceFactor(3)
            box.ScalingEnabledOff()
            box.GetOutlineProperty().SetOpacity(0)
            box.OutlineCursorWiresOff()
            box.GetHandleProperty().SetPointSize(1)

            def on_interact(caller, event):
                t = vtk.vtkTransform()
                box.GetTransform(t)
                if self.current_actor:
                    self.current_actor.SetUserTransform(t)
                self.vtkWidget.GetRenderWindow().Render()

            box.AddObserver("InteractionEvent", on_interact)
            self.box_widget = box

        self.box_widget.SetProp3D(self.current_actor)
        self.box_widget.PlaceWidget()
        self.box_widget.On()

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
        # Explicit bounds, not a bare ResetCamera(): from the 2nd cloud
        # onward, the box widget from the PREVIOUS cloud is still On()
        # and visible (opacity 0, but still a real prop) with bounds 3x
        # the old cloud's size (SetPlaceFactor(3) in _enable_box_widget,
        # called below AFTER this) - a bare ResetCamera() would fit
        # against that stale, oversized box instead of the new cloud.
        self.renderer.ResetCamera(actor.GetBounds())

        self.vtkWidget.GetRenderWindow().Render()
        self._enable_box_widget()



# How to use touch screen for zoom
