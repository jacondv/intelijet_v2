# vtk_viewer.py
import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from PyQt5.QtWidgets import QWidget, QVBoxLayout

class VTKViewer:
    def __init__(self, parent_widget: QWidget):
        self.parent_widget = parent_widget
        self.renderer = vtk.vtkRenderer()
        self.current_actor = None
        self.box_widget = None

        # ----- VTK widget -----
        self.vtkWidget = QVTKRenderWindowInteractor(parent_widget)
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
        self.style = vtk.vtkInteractorStyleTrackballCamera()
        self.iren.SetInteractorStyle(self.style)

        # Axes orientation
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

    # ----------------------------------------------------
    #  Tạo vtkBoxWidget sau khi actor được add vào scene ()
    # ----------------------------------------------------
    def _enable_box_widget(self):
        if self.box_widget:
            self.box_widget.Off()
            self.box_widget = None

        box = vtk.vtkBoxWidget()
        box.SetInteractor(self.iren)
        box.SetPlaceFactor(1.0)
        box.SetProp3D(self.current_actor)
        box.PlaceWidget()

        def on_interact(caller, event):
            t = vtk.vtkTransform()
            box.GetTransform(t)
            self.current_actor.SetUserTransform(t)
            self.vtkWidget.GetRenderWindow().Render()

        box.AddObserver("InteractionEvent", on_interact)
        box.On()

        self.box_widget = box

    # ----------------------------------------------------
    #  Update cloud
    # ----------------------------------------------------
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
        actor.GetProperty().SetPointSize(2)

        if self.current_actor:
            self.renderer.RemoveActor(self.current_actor)

        self.current_actor = actor
        self.renderer.AddActor(actor)
        self.renderer.ResetCamera()

        self.vtkWidget.GetRenderWindow().Render()

        # ---- bật box widget ----
        self._enable_box_widget()
