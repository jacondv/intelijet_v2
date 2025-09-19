# vtk_viewer.py
import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from PyQt5.QtWidgets import QWidget

class VTKViewer:
    def __init__(self, parent_widget: QWidget):
        self.parent_widget = parent_widget
        self.renderer = vtk.vtkRenderer()
        self.current_actor = None

        self.vtkWidget = QVTKRenderWindowInteractor(parent_widget)
        layout = parent_widget.layout()
        if layout is None:
            from PyQt5.QtWidgets import QVBoxLayout
            layout = QVBoxLayout(parent_widget)
            parent_widget.setLayout(layout)
        layout.addWidget(self.vtkWidget)

        self.vtkWidget.GetRenderWindow().AddRenderer(self.renderer)

        # --- Đặt camera ngay lúc khởi tạo ---
        cam = self.renderer.GetActiveCamera()
        cam.SetPosition(-1, 0, 0)        # camera phía trước gốc (X vào màn hình)
        cam.SetFocalPoint(0, 0, 0)       # nhìn vào gốc
        cam.SetViewUp(0, 0, 1)        # hướng up
        self.renderer.ResetCameraClippingRange()

        iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        self.style = vtk.vtkInteractorStyleTrackballCamera()
        iren.SetInteractorStyle(self.style)

        # Axes
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(1.0, 1.0, 1.0)
        axes.AxisLabelsOn()
        axes.SetCylinderRadius(0.05)

        self.orientation_widget = vtk.vtkOrientationMarkerWidget()
        self.orientation_widget.SetOrientationMarker(axes)
        self.orientation_widget.SetInteractor(self.vtkWidget)
        self.orientation_widget.SetViewport(0.0, 0.0, 0.2, 0.2)
        self.orientation_widget.EnabledOn()
        self.orientation_widget.InteractiveOff()

        self.vtkWidget.Initialize()
        self.vtkWidget.Start()

    def update(self, polydata):
        import vtk

        if not polydata:
            return None

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
