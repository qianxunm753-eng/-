# -*- coding: utf-8 -*-
"""Python2-compatible appearance only: view depth, point size and manual ROI.

No acquisition, registration, point-coordinate edits or equipment labels.
"""
import array
import math


def _finite(values):
    return all(not math.isnan(v) and not math.isinf(v) for v in values)


def camera_projection(position,focal):
    position=tuple(float(v) for v in position);focal=tuple(float(v) for v in focal)
    if len(position)!=3 or len(focal)!=3 or not _finite(position+focal):
        raise ValueError('finite camera position/focal required')
    direction=tuple(f-p for f,p in zip(focal,position));length=math.sqrt(sum(v*v for v in direction))
    if length<1e-12:raise ValueError('camera needs a nonzero viewing direction')
    axis=tuple(v/length for v in direction)
    return axis,-sum(a*p for a,p in zip(axis,position))


def layer_projection(position,focal):
    """Signed view-axis offset from the focal plane, NOT a LiDAR measurement."""
    axis,unused=camera_projection(position,focal)
    return axis,-sum(a*float(v) for a,v in zip(axis,focal))


def surface_slice_planes(position,focal,center,width):
    center=float(center);width=float(width)
    if not _finite((center,width)) or not .002<=width<=20. or abs(center)>20.:
        raise ValueError('finite camera-layer center and 0.002..20m width required')
    axis,unused=layer_projection(position,focal)
    return tuple((tuple(a*(1 if side==0 else -1) for a in axis),
        tuple(float(f)+a*(center+(-.5 if side==0 else .5)*width) for f,a in zip(focal,axis))) for side in (0,1))


def union_bounds(bounds):
    valid=[tuple(float(v) for v in b) for b in bounds if b is not None]
    valid=[b for b in valid if len(b)==6 and _finite(b) and all(b[i]<=b[i+1] for i in (0,2,4))]
    if not valid:return None
    return tuple((min if i%2==0 else max)(b[i] for b in valid) for i in range(6))


def projected_range(bounds,axis,offset):
    low=offset;high=offset
    for i,value in enumerate(axis):
        a,b=bounds[2*i:2*i+2]
        low+=value*(a if value>=0. else b);high+=value*(b if value>=0. else a)
    if high-low<.01:
        center=(low+high)*.5;low=center-.005;high=center+.005
    return low,high


def display_settings(depth,point_size,roi):
    point_size=float(point_size)
    if not _finite((point_size,)) or not .5<=point_size<=5.:
        raise ValueError('display point size must be finite and within 0.5..5 pixels')
    if roi is not None:
        roi=tuple(float(v) for v in roi)
        if (len(roi)!=6 or not _finite(roi) or
            not all(roi[i]<roi[i+1] for i in (0,2,4)) or max(abs(v) for v in roi)>10.):
            raise ValueError('display ROI needs ordered finite XYZ bounds within -10..10m')
    return dict(depth=bool(depth),point_size=point_size,roi=roi)


def _kind(vtk,name):
    value=getattr(vtk,name,None)
    if value is not None:return value
    for module in ('vtkCommonCorePython','vtkCommonDataModelPython','vtkFiltersCorePython',
                   'vtkRenderingCorePython','vtkRenderingAnnotationPython'):
        value=getattr(__import__(module),name,None)
        if value is not None:return value
    raise AttributeError(name)


class _BoundCloud(object):
    """Bounded C++ bounds query; own storage without creating another actor."""
    def __init__(self,vtk):
        self.points=_kind(vtk,'vtkPoints')();self.coords=_kind(vtk,'vtkFloatArray')()
        self.coords.SetNumberOfComponents(3);self.points.SetData(self.coords)
        self.values=array.array('f')

    def update(self,values):
        if values:self.coords.SetVoidArray(values,len(values),1)
        else:self.coords.Initialize();self.coords.SetNumberOfComponents(3)
        self.values=values;self.coords.Modified();self.points.Modified()
        return tuple(self.points.GetBounds()) if values else None


class ImageAppearance(object):
    def __init__(self,overlay):
        self.overlay=overlay;self.vtk=overlay.vtk;self.renderer=overlay.renderer
        self.data,self.actor=overlay.cloud_actors['object'];self.mapper=self.actor.GetMapper()
        self.settings=display_settings(True,2.,None);self.pair=None;self.bounds=None
        self.bounds_clouds=[_BoundCloud(self.vtk),_BoundCloud(self.vtk)]
        self.closed=False;self.range=None;self.last_projection=None;self.observer=None
        self.surface_mode=None;self.surface_center=0.;self.surface_width=.08
        self.frame_values=array.array('f');self.frame_entry=None
        self.frame_array=_kind(self.vtk,'vtkFloatArray')();self.frame_array.SetName('image_scan_ordinal')
        self.frame_array.SetNumberOfComponents(1)
        self.calculator=_kind(self.vtk,'vtkArrayCalculator')()
        self.calculator.SetInputData(self.data);self.calculator.SetAttributeModeToUsePointData()
        self.calculator.CoordinateResultsOff();self.calculator.SetResultArrayName('image_view_depth')
        for i,name in enumerate(('x','y','z')):self.calculator.AddCoordinateScalarVariable(name,i)
        self.calculator.SetFunction('z')
        self.threshold=_kind(self.vtk,'vtkThresholdPoints')()
        self.threshold.SetInputConnection(self.calculator.GetOutputPort())
        self.threshold.SetInputArrayToProcess(0,0,0,0,'image_view_depth')
        self.frame_table=_kind(self.vtk,'vtkLookupTable')();self.frame_table.SetNumberOfTableValues(20)
        colors=((31,119,180),(174,199,232),(255,127,14),(255,187,120),(44,160,44),
            (152,223,138),(214,39,40),(255,152,150),(148,103,189),(197,176,213),
            (140,86,75),(196,156,148),(227,119,194),(247,182,210),(127,127,127),
            (199,199,199),(188,189,34),(219,219,141),(23,190,207),(158,218,229))
        for i,rgb in enumerate(colors):self.frame_table.SetTableValue(i,*(tuple(v/255. for v in rgb)+(1.,)))
        self.frame_table.SetRange(-.5,19.5)
        self.support_table=_kind(self.vtk,'vtkLookupTable')();self.support_table.SetNumberOfTableValues(21)
        anchors=((68,1,84),(59,82,139),(33,145,140),(94,201,98),(253,231,37))
        for i in range(21):
            index=min(i//5,3);weight=(i-index*5)/5.
            rgb=tuple((a*(1.-weight)+b*weight)/255. for a,b in zip(anchors[index],anchors[index+1]))
            self.support_table.SetTableValue(i,*(rgb+(1.,)))
        self.support_table.SetRange(0.,20.)
        self.table=_kind(self.vtk,'vtkLookupTable')()
        self.table.SetHueRange(.60,.03);self.table.SetSaturationRange(.80,.80)
        self.table.SetValueRange(.95,.95);self.table.SetNumberOfTableValues(256);self.table.Build()
        self.legend=_kind(self.vtk,'vtkScalarBarActor')();self.legend.SetLookupTable(self.table)
        self.legend.SetTitle('View layers (m)\nrelative to focus');self.legend.SetNumberOfLabels(3)
        self.legend.SetLabelFormat('%.2f');self.legend.SetPosition(.89,.10)
        self.legend.SetWidth(.09);self.legend.SetHeight(.32);self.legend.SetVisibility(0)
        self.legend.SetMaximumWidthInPixels(150);self.legend.SetMaximumHeightInPixels(280)
        self.legend.GetTitleTextProperty().SetFontSize(12);self.legend.GetLabelTextProperty().SetFontSize(11)
        self.planes=[]
        try:
            self.configure(True,2.,None)
            self.renderer.AddActor2D(self.legend)
            self.observer=self.renderer.AddObserver('StartEvent',self._start)
        except Exception:
            self.close()
            raise

    def prepare(self,packet):
        # Remove the old tuple field BEFORE base point upload/render. Its old
        # length can differ from the new cloud; never expose a stale C++ buffer.
        self.data.GetPointData().RemoveArray('image_scan_ordinal')
        self.data.GetPointData().RemoveArray('image_scan_support')
        self.frame_array.Initialize();self.frame_array.SetNumberOfComponents(1)
        self.frame_values=array.array('f')
        mode=self.surface_mode
        self.frame_entry=packet.get('_surface_views',{}).get(mode) if mode in ('frames','support') else None
        pair=packet.get('_image_pairs',{})
        if pair is self.pair:return
        self.pair=pair;bounds=[]
        for cloud,key in zip(self.bounds_clouds,('single','fused')):
            entry=pair.get(key,{}).get('object')
            values=self.overlay.cache.cloud(entry) if entry is not None else array.array('f')
            bounds.append(cloud.update(values))
        self.bounds=union_bounds(bounds);self.last_projection=None

    def finish(self,packet):
        if self.surface_mode not in ('frames','support') or self.frame_entry is None:return
        support=self.surface_mode=='support'
        entry=self.frame_entry;count=entry.get('count',-1);data=entry.get('support_data' if support else 'frame_data',b'')
        if not 0<=count<=150000 or len(data)!=count*4 or self.data.GetNumberOfPoints()!=count:
            raise ValueError('surface frame IDs must match the real displayed points')
        values=array.array('f')
        if hasattr(values,'frombytes'):values.frombytes(data)
        else:values.fromstring(data)
        maximum=20 if support else 19
        if not _finite(values) or any(v<0 or v>maximum or v!=int(v) for v in values):
            raise ValueError('surface scalar needs bounded integer scan evidence')
        self.frame_array.SetName('image_scan_support' if support else 'image_scan_ordinal')
        if values:self.frame_array.SetVoidArray(values,len(values),1)
        else:self.frame_array.Initialize()
        self.frame_values=values;self.frame_array.Modified()
        self.data.GetPointData().AddArray(self.frame_array);self.data.Modified()

    def surface_style(self,mode=None,center=0.,width=.08):
        if mode not in (None,'frames','support','layer'):raise ValueError('surface appearance mode invalid')
        surface_slice_planes((0,0,-1),(0,0,0),center,width)
        self.surface_mode=mode;self.surface_center=float(center);self.surface_width=float(width)
        self.configure(self.settings['depth'],self.settings['point_size'],self.settings['roi'])

    def configure(self,depth,point_size,roi):
        settings=display_settings(depth,point_size,roi)
        self.settings=settings;self.last_projection=None
        self.mapper.RemoveAllClippingPlanes();self.planes=[]
        if settings['roi'] is not None:
            for dimension in range(3):
                for side in (0,1):
                    plane=_kind(self.vtk,'vtkPlane')();normal=[0.,0.,0.];origin=[0.,0.,0.]
                    normal[dimension]=1. if side==0 else -1.
                    origin[dimension]=settings['roi'][2*dimension+side]
                    plane.SetNormal(*normal);plane.SetOrigin(*origin)
                    self.mapper.AddClippingPlane(plane);self.planes.append(plane)
        for unused,actor in self.overlay.cloud_actors.values():actor.GetProperty().SetPointSize(point_size)
        if self.surface_mode in ('frames','support'):
            support=self.surface_mode=='support';table=self.support_table if support else self.frame_table
            self.mapper.SetInputData(self.data);self.mapper.SetLookupTable(table)
            self.mapper.SetScalarModeToUsePointFieldData();self.mapper.SelectColorArray('image_scan_support' if support else 'image_scan_ordinal')
            self.mapper.SetColorModeToMapScalars();self.mapper.UseLookupTableScalarRangeOn();self.mapper.ScalarVisibilityOn()
            self.legend.SetLookupTable(table)
            self.legend.SetTitle('Nearby scan support\n20mm; NOT accuracy' if support else 'Window scan ordinal\n0..19; real IDs in report')
        elif depth or self.surface_mode=='layer':
            self.mapper.SetInputConnection((self.threshold if self.surface_mode=='layer' else self.calculator).GetOutputPort())
            self.mapper.SetLookupTable(self.table);self.mapper.SetScalarModeToUsePointFieldData()
            self.mapper.SelectColorArray('image_view_depth');self.mapper.SetColorModeToMapScalars()
            self.mapper.UseLookupTableScalarRangeOn();self.mapper.ScalarVisibilityOn()
            self.legend.SetLookupTable(self.table);self.legend.SetTitle('View layers (m)\nrelative to focus')
        else:
            self.mapper.SetInputData(self.data);self.mapper.ScalarVisibilityOff()
            self.actor.GetProperty().SetColor(.45,.84,.96);self.legend.SetVisibility(0)

    def _start(self,*unused):
        if self.closed:return
        if not self.actor.GetVisibility():self.legend.SetVisibility(0);return
        if self.surface_mode in ('frames','support'):self.legend.SetVisibility(1);return
        if not self.settings['depth'] and self.surface_mode!='layer':return
        bounds=self.settings['roi'] or self.bounds
        if bounds is None:self.legend.SetVisibility(0);return
        camera=self.renderer.GetActiveCamera()
        axis,offset=layer_projection(camera.GetPosition(),camera.GetFocalPoint())
        depth_range=projected_range(bounds,axis,offset)
        if self.surface_mode=='layer':
            depth_range=(self.surface_center-self.surface_width/2.,self.surface_center+self.surface_width/2.)
            self.threshold.ThresholdBetween(*depth_range)
        signature=(axis,offset,depth_range)
        if signature!=self.last_projection:
            self.calculator.SetFunction('(%0.17g)*x+(%0.17g)*y+(%0.17g)*z+(%0.17g)'%tuple(axis+(offset,)))
            self.table.SetRange(*depth_range);self.mapper.SetScalarRange(*depth_range)
            self.range=depth_range;self.last_projection=signature
        self.legend.SetVisibility(1)

    def snapshot(self):
        return dict(self.settings,depth_range=self.range,paired_bounds=self.bounds,
            surface_mode=self.surface_mode,surface_center=self.surface_center,surface_width=self.surface_width,
            color_meaning=('distinct nearby scans; NOT noise truth or accuracy' if self.surface_mode=='support' else
                'view-axis offset from focus; NOT LiDAR range, lamp color or equipment class'))

    def close(self):
        if self.closed:return
        self.closed=True
        if self.observer is not None:self.renderer.RemoveObserver(self.observer)
        self.renderer.RemoveActor2D(self.legend)
        self.mapper.RemoveAllClippingPlanes();self.mapper.SetInputData(self.data);self.mapper.ScalarVisibilityOff()
        self.data.GetPointData().RemoveArray('image_scan_ordinal');self.data.GetPointData().RemoveArray('image_scan_support')
        self.frame_array.Initialize();self.frame_values=array.array('f')
        self.pair=None
