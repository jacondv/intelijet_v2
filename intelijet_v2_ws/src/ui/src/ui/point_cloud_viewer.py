import sys
import vtk
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QFrame
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from pps_ui import Ui_Frame  # Import class từ file pps_ui.py

class App(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_Frame()
        self.ui.setupUi(self)  # Gán các widget đã thiết kế vào self

        self.vl = QVBoxLayout()
        self.vtkWidget = QVTKRenderWindowInteractor(self.ui.couldFrame)
        self.vl.addWidget(self.vtkWidget)

        # Create a VTK renderer
        self.renderer = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.renderer)

        # Example point cloud
        points = vtk.vtkPoints()
        for x in range(-10, 10):
            for y in range(-10, 10):
                z = (x**2 + y**2) * 0.01
                points.InsertNextPoint(x, y, z)

        polydata = vtk.vtkPolyData()
        polydata.SetPoints(points)

        vertex_filter = vtk.vtkVertexGlyphFilter()
        vertex_filter.SetInputData(polydata)
        vertex_filter.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(vertex_filter.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetPointSize(3)

        self.renderer.AddActor(actor)
        self.renderer.ResetCamera()

        self.vtkWidget.Initialize()
        self.vtkWidget.Start()

if __name__ == "__main__":
    from PyQt5.QtWidgets import QWidget, QHBoxLayout

    app = QApplication(sys.argv)
    main_widget = QWidget()
    layout = QHBoxLayout(main_widget)

    viewer = App()
    layout.addWidget(viewer)

    main_widget.show()
    sys.exit(app.exec_())
