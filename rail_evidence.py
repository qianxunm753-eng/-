"""VTK-only diagnostic geometry. All calls stay on the RSView UI thread."""
from rail_centerline import _unit, _cross


class RailEvidenceOverlay(object):
    def __init__(self, smp, vtk, view):
        self.smp, self.vtk, self.view = smp, vtk, view
        self.layers = {}
        active = smp.GetActiveSource()
        for name, color, size in (('left', [1., .65, .05], 4.),
                                  ('right', [.1, 1., 1.], 4.),
                                  ('markers', [1., 1., 1.], 11.),
                                  ('section', [.85, .85, 1.], 1.)):
            source = smp.PVTrivialProducer(guiName='Rail evidence ' + name)
            source.GetClientSideObject().SetOutput(vtk.vtkPolyData())
            source.UpdatePipeline()
            rep = smp.Show(source, view)
            rep.ColorArrayName = ''
            rep.DiffuseColor = rep.AmbientColor = color
            rep.Representation = 'Surface With Edges' if name == 'section' else 'Points'
            rep.PointSize = size
            rep.Visibility = 0
            rep.Pickable = 0
            if name == 'section':
                rep.Opacity = .15
                rep.EdgeColor = color
                rep.LineWidth = 2.
            self.layers[name] = (source, rep)
        smp.SetActiveSource(active)

    def _set_points(self, name, values, polygon=False):
        points, cells = self.vtk.vtkPoints(), self.vtk.vtkCellArray()
        for p in values:
            points.InsertNextPoint(p)
        if polygon:
            cells.InsertNextCell(len(values))
            for i in range(len(values)):
                cells.InsertCellPoint(i)
        else:
            for i in range(len(values)):
                cells.InsertNextCell(1)
                cells.InsertCellPoint(i)
        data = self.vtk.vtkPolyData()
        data.SetPoints(points)
        if polygon:
            data.SetPolys(cells)
        else:
            data.SetVerts(cells)
        source, rep = self.layers[name]
        source.GetClientSideObject().SetOutput(data)
        source.UpdatePipeline()
        rep.Visibility = int(bool(values))

    def display(self, result):
        if result is None:
            for source, rep in self.layers.values():
                rep.Visibility = 0
            return
        for name in ('left', 'right'):
            self._set_points(name, result[name + '_support_points'])
        left, center, right = [result[k] for k in ('left_point', 'center_point', 'right_point')]
        self._set_points('markers', [left, center, right])
        start, end = result['endpoints']
        direction = _unit(tuple(end[i] - start[i] for i in range(3)))
        across = _unit(tuple(right[i] - left[i] for i in range(3)))
        vertical = _unit(_cross(direction, across))
        # Illustrative section rectangle, not a detected physical surface.
        corners = [tuple(p[i] + margin * across[i] + height * vertical[i] for i in range(3))
                   for p, margin, height in ((left, -.1, -.2), (right, .1, -.2),
                                              (right, .1, .2), (left, -.1, .2))]
        self._set_points('section', corners, polygon=True)

    def close(self):
        for source, rep in self.layers.values():
            self.smp.Delete(source)
        self.layers.clear()
