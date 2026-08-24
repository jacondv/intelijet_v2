# vtk_viewer.py
import numpy as np
import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from PyQt5.QtCore import Qt, QEvent
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QGestureEvent, QPinchGesture, QPanGesture

import rospy

# Touch events this widget consumes itself (see event() below) instead of
# letting them fall through to Qt's default mouse-synthesis-from-touch
# fallback.
_TOUCH_EVENT_TYPES = (
    QEvent.TouchBegin, QEvent.TouchUpdate, QEvent.TouchEnd, QEvent.TouchCancel,
)


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
        if event.type() in _TOUCH_EVENT_TYPES:
            # Without this, an un-accepted touch event falls through to
            # Qt's default handling, which (since nothing here overrides
            # touchEvent()) synthesizes a left-button mouse press/move/
            # release from the SAME touch points. Those synthesized mouse
            # events reach QVTKRenderWindowInteractor's own native
            # handling underneath us, which feeds
            # vtkInteractorStyleTrackballCamera's own (different) rotation
            # algorithm - running at the same time as, and fighting
            # against, the gesture-driven on_pan()/on_pinch_zoom() above.
            # That fight is exactly what produced the reported symptom:
            # the cloud snapping to an unpredictable orientation right as
            # a touch-drag begins, before settling into normal gesture-
            # only rotation once the synthesized mouse drag stops. Accept
            # (consume) touch here so only the gesture recognizer ever
            # drives the camera.
            event.accept()
            return True
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
        self._apply_canonical_orientation()
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
    # The app's one fixed camera convention, matching the original
    # __init__ setup below: standing at -X looking toward +X (X axis
    # points into the screen, away from the viewer), Z up. With that
    # direction/up pair, the camera's screen-right vector works out to
    # (0,-1,0) - i.e. +Y is screen-LEFT. Both a freshly loaded cloud
    # (update()) and "Zoom Center" (restore_initial_view()) reset back to
    # this orientation, regardless of whatever the user last rotated to
    # via mouse drag - only the fit distance is recomputed per cloud.
    CANONICAL_VIEW_DIRECTION = (-1, 0, 0)
    CANONICAL_VIEW_UP = (0, 0, 1)

    # "Zoom Center" - recomputed fresh against whichever cloud is
    # CURRENTLY loaded (previously this replayed a camera state captured
    # once from the very FIRST cloud ever shown in the app session, which
    # no longer matched a later, differently-sized/positioned cloud).
    # Camera-only: this has no effect on compare or its results - compare
    # works off the raw cloud data, never off this viewport's
    # camera/actor state.
    ZOOM_CENTER_DOLLY_METERS = 12.0
    # Floor so the dolly above can never cross into/past the cloud's own
    # center when ZOOM_CENTER_DOLLY_METERS is bigger than the fit distance.
    MIN_DISTANCE_METERS = 0.1

    def _apply_canonical_orientation(self):
        cam = self.renderer.GetActiveCamera()
        cam.SetPosition(*self.CANONICAL_VIEW_DIRECTION)
        cam.SetFocalPoint(0, 0, 0)
        cam.SetViewUp(*self.CANONICAL_VIEW_UP)

    def restore_initial_view(self):
        if not self.current_actor:
            return
        cam = self.renderer.GetActiveCamera()

        # Undo any box-widget drag back to identity, reset to the
        # canonical orientation, then re-fit the camera to the actor's
        # OWN bounds specifically (ResetCamera() with no args scans every
        # visible prop in the renderer, which includes the box widget's
        # outline - invisible at opacity 0, but still a real prop with
        # real bounds set 3x larger than the cloud via SetPlaceFactor(3)
        # in _enable_box_widget - so it was fitting the camera to a box
        # 3x too big, making the cloud look small and far instead of
        # properly framed).
        self.current_actor.SetUserTransform(vtk.vtkTransform())
        self._apply_canonical_orientation()
        self.renderer.ResetCamera(self.current_actor.GetBounds())

        # Move the camera ZOOM_CENTER_DOLLY_METERS closer along its own
        # view direction (camera -> focal point) so the cloud reads
        # closer/larger than a plain bounding-box fit, not just centered.
        # Clamped to MIN_DISTANCE_METERS instead of skipped outright when
        # the requested dolly is bigger than the fit distance itself (the
        # previous `if distance > ZOOM_CENTER_DOLLY_METERS` guard silently
        # did nothing whenever the cloud's fit distance was smaller than
        # the configured dolly - e.g. dolly=5m on a cloud that fits at
        # 3m - instead of moving in as far as it safely can).
        pos = np.array(cam.GetPosition())
        focal = np.array(cam.GetFocalPoint())
        direction = focal - pos
        distance = np.linalg.norm(direction)
        if distance > 0:
            new_distance = max(distance - self.ZOOM_CENTER_DOLLY_METERS, self.MIN_DISTANCE_METERS)
            cam.SetPosition(*(focal - direction / distance * new_distance))

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
        # Reset to the canonical orientation (see CANONICAL_VIEW_DIRECTION
        # above) rather than keeping whatever direction the user last
        # rotated to - a freshly loaded cloud always starts from the same
        # fixed viewing angle.
        self._apply_canonical_orientation()
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
