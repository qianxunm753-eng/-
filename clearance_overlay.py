# -*- coding: utf-8 -*-
"""Python 2.7/3 compatible RSView renderer for exact offline playback frames.

No scipy/modern numpy in the embedded interpreter. Precomputed packets are
explicitly labelled OFFLINE; reading them is not a live algorithm benchmark.
"""
import array
import hashlib
import json
import os
import struct
import sys
import time

_clock = getattr(time, 'perf_counter', None) or (time.clock if sys.platform == 'win32' else time.time)


def is_target_file(filename):
    return os.path.basename(filename).lower() == '1212121.pcap'


def label_positions(anchors, widths, view_size):
    """Place short screen-space labels without changing measured geometry.

    Prefer each line's midpoint; move text vertically when labels collide.
    Clamp positions to the viewport, including after camera/size changes.
    """
    view_width, view_height = view_size
    height, gap = 24, 4
    placed, positions = [], []
    for (anchor_x, anchor_y), width in zip(anchors, widths):
        width = max(1, int(width))
        max_x, max_y = max(0, view_width-width), max(0, view_height-height)
        x = int(max(0, min(max_x, anchor_x+8)))
        y = int(max(0, min(max_y, anchor_y+8)))
        offsets = [0]
        for step in range(1, int(view_height//(height+gap))+2):
            offsets.extend((step*(height+gap), -step*(height+gap)))
        chosen = None
        for offset in offsets:
            candidate_y = int(max(0, min(max_y, y+offset)))
            if not any(x < px+pw+gap and px < x+width+gap and
                       candidate_y < py+height+gap and py < candidate_y+height+gap
                       for px, py, pw in placed):
                chosen = (x, candidate_y)
                break
        # A viewport shorter than the labels cannot fit every rectangle.
        if chosen is None:
            chosen = (x, y)
        positions.append(chosen)
        placed.append((chosen[0], chosen[1], width))
    return positions


class InstanceLabelLayout(object):
    """Remember relative text rows only, never old geometry or distance.

    Current visible IDs retain placement priority when selection order changes.
    State is bounded by the current eight-caption budget and reset per source.
    """
    def __init__(self):
        self.context=None
        self.view_size=None
        self.order=[]
        self.offsets={}

    def positions(self,keys,anchors,widths,view_size,context):
        if self.context!=context:
            self.order=[]
            self.offsets={}
        if self.view_size!=tuple(view_size):
            self.offsets={}
        self.context=context
        self.view_size=tuple(view_size)
        # Occurrence suffixes tolerate malformed duplicate display IDs without
        # letting a renderer exception erase the other current dimensions.
        counts={}
        unique=[]
        for key in keys:
            count=counts.get(key,0)
            counts[key]=count+1
            unique.append((key,count))
        rows=dict((key,i) for i,key in enumerate(unique))
        self.order=[key for key in self.order if key in rows]
        self.order.extend(key for key in unique if key not in self.order)
        height,gap=24,4
        view_width,view_height=view_size
        placed=[]
        output=[None]*len(keys)
        current_offsets={}
        for key in self.order:
            i=rows[key]
            ax,ay=anchors[i]
            width=max(1,int(widths[i]))
            max_x,max_y=max(0,view_width-width),max(0,view_height-height)
            x=int(max(0,min(max_x,ax+8)))
            base_y=int(max(0,min(max_y,ay+8)))
            shifts=[self.offsets.get(key,0),0]
            for step in range(1,int(view_height//(height+gap))+2):
                shifts.extend((step*(height+gap),-step*(height+gap)))
            chosen=None
            for shift in shifts:
                y=int(max(0,min(max_y,base_y+shift)))
                if not any(x<px+pw+gap and px<x+width+gap and
                           y<py+height+gap and py<y+height+gap for px,py,pw in placed):
                    chosen=(x,y)
                    break
            if chosen is None:
                chosen=(x,base_y)
            output[i]=chosen
            placed.append((chosen[0],chosen[1],width))
            current_offsets[key]=chosen[1]-base_y
        self.offsets=current_offsets
        return output


def measurement_priority(item):
    """Bounded identity support affects text selection, never measured distance.

    Old untracked packets retain their historical ordering. A long history
    cannot monopolize labels indefinitely, nor certify a fragment as equipment.
    """
    quality=item.get('selection_quality')
    if quality is not None:
        # Current evidence first. Dense far returns/old reservations do not
        # outweigh an adequately observed nearer surface. No value is smoothed.
        weak=not quality.get('boundary_supported',not item['uncertain'])
        ambiguous=quality.get('ambiguity_fraction',
                              1. if 'nearest_ambiguous_surface' in item.get('flags',()) else 0.)
        weak=bool(weak or ambiguous>.3 or
                  quality.get('boundary_gap_m',0.)>quality.get('neighbor_radius_m',float('inf'))+1e-9)
        coverage=min(12,max(0,int(quality.get('occupied_cells',0))))
        return (weak,-coverage,item['distance_m'],
                -min(5,int(item.get('display_observations',1))),
                not item.get('display_slot_reserved',False))
    if item.get('display_id'):
        # A repeatedly seen two-return fragment cannot displace a currently
        # visible complete region. This is text decluttering, not rejection.
        reserved=item.get('display_slot_reserved',False)
        return ('nearest_ambiguous_surface' in item.get('flags',()) and not reserved,
                -item['point_count']*(2 if reserved else 1),item['uncertain'],
                -min(5,int(item.get('display_observations',1))),item['distance_m'])
    if item.get('surface_id'):
        return (item['uncertain'], -min(5,int(item.get('surface_observations',1))),
                -item['point_count'], item['distance_m'])
    return (item['uncertain'], 0, -item['point_count'], item['distance_m'])


def measurement_label(item):
    identity = item.get('display_id') or item.get('surface_id')
    if item.get('measurement_basis')=='rolling_20_window':
        prefix=('20F Window raw ?' if item.get('window_raw_preview') else
            '20F Window pending '+(identity or item.get('object_id') or ''))
    elif item.get('measurement_basis')=='stop_window':
        prefix=('Window pending ' if item['uncertain'] else 'Window ')+(identity or item.get('object_id') or '')
    else:
        prefix = ('Region ' if item.get('display_id') else 'Surface ')+identity if identity else 'Candidate'
    if item['uncertain'] and item.get('measurement_basis') not in ('stop_window','rolling_20_window'):
        prefix = 'Uncertain '+(identity or '')
    text = '%s %.3f m' % (prefix, item['distance_m'])
    if item.get('raw_near_warning'):
        text += ' | raw %.3f ?' % item['observed_min_distance_m']
    return text


def select_display_measurements(items, limit, balance_sides=False):
    """Choose current observations only; never change distance or membership.

    Round-robin available side buckets prevents a dense side taking every slot.
    An exhausted side relinquishes slots; no fixed equipment count is imposed.
    Legacy packets retain the old priority and four-line presentation.
    """
    if limit < 0:
        raise ValueError('nonnegative display selection limit required')
    eligible=[item for item in items if item.get('selection_quality') is None or not measurement_priority(item)[0]]
    ranked=sorted(eligible,key=measurement_priority)
    if not balance_sides or limit <= 1:
        return ranked[:limit]
    buckets={}
    order=[]
    for measurement in ranked:
        side=measurement.get('side')
        if side not in buckets:
            buckets[side]=[]
            order.append(side)
        buckets[side].append(measurement)
    selected=[]
    positions=dict((side,0) for side in order)
    while len(selected)<limit:
        added=False
        for side in order:
            index=positions[side]
            if index<len(buckets[side]):
                selected.append(buckets[side][index])
                positions[side]+=1
                added=True
                if len(selected)==limit:
                    break
        if not added:
            break
    return selected


def dimension_endpoints(item):
    """Display annotation, explicitly separate from actual measurement points."""
    return (item.get('dimension_projected_L',item['projected_L']),
            item.get('dimension_foot_L',item['foot_L']))


def boundary_markers(items):
    """The selected displayed real return and foot, not invented geometry."""
    return tuple(array.array('f',[v for item in items for v in item[key]])
                 for key in ('point_L','foot_L'))


def raw_boundary_markers(items):
    """A nearer unconfirmed return is orange, never silently called reliable."""
    return array.array('f',[v for item in items if item.get('raw_near_warning') and
        item.get('observed_source_index') != item.get('source_index')
        for v in item['observed_point_L']])


class MemoryPacketCache(object):
    """Read only this worker's owned packed cloud; no file-result lookup."""
    def __init__(self,max_points=20000):
        if not isinstance(max_points,int) or not 1<=max_points<=150000:
            raise ValueError('bounded packed display budget required')
        self.max_points=max_points

    def cloud(self, entry):
        count = int(entry['count'])
        if (not 0 <= count <= self.max_points or entry.get('dtype') != 'little-float32-xyz' or
                len(entry['data']) != count*12):
            raise ValueError('Invalid online display buffer')
        values = array.array('f')
        if sys.version_info[0] == 2:
            values.fromstring(entry['data'])
        else:
            values.frombytes(entry['data'])
        if sys.byteorder != 'little':
            values.byteswap()
        return values


class PacketCache(object):
    def __init__(self, folder, source_pcap=None):
        self.folder = os.path.abspath(folder)
        self.last_sequence = None
        self.last_packet = None
        if source_pcap is not None:
            with open(os.path.join(self.folder, 'manifest.json'), 'rb') as stream:
                manifest = json.load(stream)
            digest = hashlib.sha256()
            with open(source_pcap, 'rb') as stream:
                while True:
                    block = stream.read(1024*1024)
                    if not block:
                        break
                    digest.update(block)
            if digest.hexdigest() != manifest['source_sha256']:
                raise ValueError('Playback cache belongs to a different PCAP')

    def get(self, sequence):
        sequence = int(sequence)
        if sequence == self.last_sequence:
            return self.last_packet
        self.last_sequence, self.last_packet = sequence, None
        path = os.path.join(self.folder, 'frame-%04d.json' % sequence)
        if not os.path.isfile(path):
            return None
        if os.path.getsize(path) > 256*1024:
            raise ValueError('Oversized playback metadata')
        with open(path, 'rb') as stream:
            packet = json.load(stream)
        if (packet.get('format') != 'rsview-clearance-playback-v1' or
                packet.get('sequence') != sequence):
            raise ValueError('Frame/cache identity mismatch')
        # The v1 producer may flag an ambiguous *nearest* point even when the
        # larger component has clear surface support. Do not call that point
        # confirmed or let dense ground remnants take the primary label slots.
        for item in packet['measurements']:
            item['uncertain'] = bool(item['uncertain'] or
                'nearest_ambiguous_surface' in item['flags'])
        packet['measurements'].sort(key=measurement_priority)
        self.last_packet = packet
        return packet

    def cloud(self, entry):
        path = os.path.abspath(os.path.join(self.folder, entry['file']))
        if os.path.dirname(path) != self.folder:
            raise ValueError('Display buffer outside playback cache')
        count = int(entry['count'])
        if not 0 <= count <= 20000 or os.path.getsize(path) != count*12:
            raise ValueError('Invalid display buffer length')
        values = array.array('f')
        with open(path, 'rb') as stream:
            values.fromfile(stream, count*3)
        if sys.byteorder != 'little':
            values.byteswap()
        return values


def _vtk_type(vtk, name):
    value = getattr(vtk, name, None)
    if value is not None:
        return value
    for module_name in ('vtkCommonCorePython', 'vtkCommonDataModelPython',
                        'vtkFiltersSourcesPython', 'vtkRenderingCorePython'):
        module = __import__(module_name)
        value = getattr(module, name, None)
        if value is not None:
            return value
    raise AttributeError(name)


class VtkPointBuffer(object):
    """Own bounded native buffers and reuse the VTK6 geometry across frames.

    save=1 tells VTK not to free Python's storage. Keep BOTH owners alive for
    as long as their arrays are attached. Python 2 on Windows has no array('q')
    even though this VTK build has 64-bit IDs; struct supplies native-size IDs.
    """
    def __init__(self, vtk, data, max_points=20000):
        self.data, self.max_points = data, int(max_points)
        self.points = _vtk_type(vtk, 'vtkPoints')()
        self.coordinates = _vtk_type(vtk, 'vtkFloatArray')()
        self.coordinates.SetNumberOfComponents(3)
        self.points.SetData(self.coordinates)
        self.cells = _vtk_type(vtk, 'vtkCellArray')()
        self.id_type = _vtk_type(vtk, 'vtkIdTypeArray')
        self.ids = self.id_type()
        self.last_count = None
        id_code = 'q' if self.ids.GetDataTypeSize() == 8 else 'i'
        flat = [value for i in range(self.max_points) for value in (1, i)]
        self.connectivity = struct.pack('=%d%s' % (len(flat), id_code), *flat)
        self.values = array.array('f')
        data.SetPoints(self.points)
        data.SetVerts(self.cells)

    def update(self, values):
        if not isinstance(values, array.array) or values.typecode != 'f' or len(values)%3:
            raise ValueError('Display coordinates require complete float32 XYZ tuples')
        count = len(values)//3
        if count > self.max_points:
            raise ValueError('Display point buffer exceeds its bound')
        if count != self.last_count:
            # VTK6 SetCells does not update NumberOfCells when given the same
            # vtkIdTypeArray again after a resize. Replace only its tiny array
            # wrapper; the native connectivity storage and points are reused.
            self.ids = self.id_type()
        # Attach the new owner before releasing the old buffer. The immutable
        # vertex buffer is reused; only its active length changes on a resize.
        if count:
            self.coordinates.SetVoidArray(values, len(values), 1)
            self.ids.SetVoidArray(self.connectivity, count*2, 1)
        else:
            self.coordinates.Initialize()
            self.coordinates.SetNumberOfComponents(3)
            self.ids.Initialize()
        self.values = values
        self.cells.SetCells(count, self.ids)
        self.last_count = count
        # Picking/GetCell builds a separate topology lookup in vtkPolyData.
        # Modified alone does not invalidate that VTK6 lookup after a resize.
        self.data.DeleteCells()
        self.coordinates.Modified()
        self.points.Modified()
        self.cells.Modified()
        self.data.Modified()


class VtkClearanceOverlay(object):
    def __init__(self, smp, vtk, view, status_label, filename, reader, cache_folder=None,
                 packet_cache=None,cloud_point_limit=20000):
        if not isinstance(cloud_point_limit,int) or not 1<=cloud_point_limit<=150000:
            raise ValueError('bounded VTK display budget required')
        if packet_cache is None and cache_folder is None:
            path = os.path.join(os.path.dirname(__file__), 'clearance_overlay_config.json')
            with open(path, 'rb') as stream:
                cache_folder = json.load(stream)['cache_folder']
        self.cache = packet_cache if packet_cache is not None else PacketCache(cache_folder, filename)
        self.smp, self.vtk, self.view = smp, vtk, view
        self.status_label = status_label
        self.renderer = view.GetRenderer()
        self.actors, self.labels, self.cloud_actors, self.center_lines, self.measure_lines = [], [], {}, [], []
        self.point_buffers = {}
        self.boundary_actors = []
        self.last_display_timings = None
        self.current = None
        self.last_processed_frame = None
        self.last_result = None
        self.raw_rep = smp.GetDisplayProperties(reader, view)
        self.raw_visibility = self.raw_rep.Visibility
        for name, color in (('object', (0.2, 1., .5)), ('ambiguous', (.65, .65, .65)),
                            ('rails', (.1, .65, 1.)), ('ground', (.25, .3, .38)),
                            ('background', (.32, .32, .42))):
            data = _vtk_type(vtk, 'vtkPolyData')()
            mapper = _vtk_type(vtk, 'vtkPolyDataMapper')()
            mapper.SetInputData(data)
            actor = _vtk_type(vtk, 'vtkActor')()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(*color)
            actor.GetProperty().SetPointSize(3. if name == 'object' else 2.)
            actor.SetVisibility(0)
            self.renderer.AddActor(actor)
            self.actors.append(actor)
            self.cloud_actors[name] = (data, actor)
            self.point_buffers[name] = VtkPointBuffer(vtk, data,max_points=cloud_point_limit)
        for color in ((1.,1.,1.),(.1,1.,1.),(1.,.65,.1)):
            data = _vtk_type(vtk, 'vtkPolyData')()
            mapper = _vtk_type(vtk, 'vtkPolyDataMapper')()
            mapper.SetInputData(data)
            actor = _vtk_type(vtk, 'vtkActor')()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(*color)
            actor.GetProperty().SetPointSize(8.)
            actor.SetVisibility(0)
            self.renderer.AddActor(actor)
            self.actors.append(actor)
            self.boundary_actors.append((VtkPointBuffer(vtk,data,max_points=8),actor))
        self.raw_boundary_actor = self.boundary_actors.pop()
        for unused in range(32):
            self.center_lines.append(self._line((1., 0., 1.), 5.))
        for unused in range(8):
            self.measure_lines.append(self._line((.1, 1., .2), 3.))
            label = _vtk_type(vtk, 'vtkTextActor')()
            label.GetTextProperty().SetColor(1., 1., 1.)
            label.GetTextProperty().SetFontSize(17)
            label.GetTextProperty().SetBold(1)
            label.SetVisibility(0)
            self.renderer.AddActor2D(label)
            self.labels.append(label)
        self.render_observer = self.renderer.AddObserver('StartEvent', self._position_labels)
        status_label.setText('  OFFLINE | Trackside distance playback')

    def _line(self, color, width):
        source = _vtk_type(self.vtk, 'vtkLineSource')()
        mapper = _vtk_type(self.vtk, 'vtkPolyDataMapper')()
        mapper.SetInputConnection(source.GetOutputPort())
        actor = _vtk_type(self.vtk, 'vtkActor')()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*color)
        actor.GetProperty().SetLineWidth(width)
        actor.SetVisibility(0)
        self.renderer.AddActor(actor)
        self.actors.append(actor)
        return source, actor

    def _position_labels(self, *unused):
        if self.current is None:
            return
        anchors, widths, labels = [], [], []
        for label, item in zip(self.labels, self.current['measurements']):
            point = [(a+b)*.5 for a, b in zip(*dimension_endpoints(item))]
            self.renderer.SetWorldPoint(point[0], point[1], point[2], 1.)
            self.renderer.WorldToDisplay()
            x, y, unused_z = self.renderer.GetDisplayPoint()
            anchors.append((x, y))
            bounds = [0., 0., 0., 0.]
            label.GetBoundingBox(self.renderer, bounds)
            widths.append(max(1, int(bounds[1]-bounds[0]+4)))
            labels.append(label)
        if self.current.get('stable_instance_labels',False):
            layout=getattr(self,'instance_label_layout',None)
            if layout is None:
                layout=self.instance_label_layout=InstanceLabelLayout()
            keys=[item.get('display_id') or item.get('candidate_id') for item in self.current['measurements']]
            context=tuple(self.current.get(key) for key in ('session_id','clock_id','epoch','config_id'))
            positions=layout.positions(keys,anchors,widths,self.renderer.GetSize(),context)
        else:
            positions = label_positions(anchors, widths, self.renderer.GetSize())
        for label, (x, y) in zip(labels, positions):
            label.SetDisplayPosition(x, y)

    def _hide(self):
        for actor in self.actors+self.labels:
            actor.SetVisibility(0)

    def update(self, unused_polydata, frame):
        frame = int(frame)
        if frame == self.last_processed_frame:
            self._position_labels()
            return self.last_result
        self.last_processed_frame = frame
        packet = self.cache.get(frame)
        return self.display_packet(packet, frame)

    def display_packet(self, packet, frame=None):
        started = _clock()
        frame = int(packet['sequence']) if packet is not None else int(frame or 0)
        limit=packet.get('display_limit',4) if packet is not None else 4
        if not isinstance(limit,int) or not 0<=limit<=8:
            raise ValueError('native display budget must be an integer within 0..8')
        self.current, self.last_result = (dict(packet, measurements=packet['measurements'][:limit])
                                         if packet is not None else None), packet
        if packet is None:
            self._hide()
            self.raw_rep.Visibility = self.raw_visibility
            self.status_label.setText('  OFFLINE | Frame %d has no prepared clearance result' % frame)
            return None
        self.raw_rep.Visibility = 0
        cloud_load, cloud_upload = 0., 0.
        for name, (data, actor) in self.cloud_actors.items():
            loaded = _clock()
            entry = packet['clouds'].get(name)
            values = self.cache.cloud(entry) if entry is not None else array.array('f')
            uploading = _clock()
            self.point_buffers[name].update(values)
            cloud_load += uploading-loaded
            cloud_upload += _clock()-uploading
            # Context visibility is independent of measurement membership.
            # Ground and uncertain returns remain dim, not target measurements.
            actor.SetVisibility(bool(len(values)))
        for (upload,actor),values in zip(self.boundary_actors,boundary_markers(self.current['measurements'])):
            upload.update(values)
            actor.SetVisibility(bool(len(values)))
        upload,actor = self.raw_boundary_actor
        values = raw_boundary_markers(self.current['measurements'])
        upload.update(values)
        actor.SetVisibility(bool(len(values)))
        geometry_done = _clock()
        for i, (source, actor) in enumerate(self.center_lines):
            if i < len(packet['centerline']):
                a, b = packet['centerline'][i]
                source.SetPoint1(*a)
                source.SetPoint2(*b)
                source.Update()
                actor.SetVisibility(1)
            else:
                actor.SetVisibility(0)
        for i, ((source, actor), label) in enumerate(zip(self.measure_lines, self.labels)):
            if i < len(self.current['measurements']):
                item = self.current['measurements'][i]
                endpoint,foot = dimension_endpoints(item)
                source.SetPoint1(*endpoint)
                source.SetPoint2(*foot)
                source.Update()
                uncertain = item['uncertain'] or 'nearest_ambiguous_surface' in item['flags']
                actor.GetProperty().SetColor(*((1., .65, .1) if uncertain else (.1, 1., .2)))
                actor.SetVisibility(1)
                label.SetInput(measurement_label(item))
                label.SetVisibility(1)
            else:
                actor.SetVisibility(0)
                label.SetVisibility(0)
        self._position_labels()
        self.status_label.setText(self.packet_status(packet))
        rendering = _clock()
        self.smp.Render(self.view)
        finished = _clock()
        self.last_display_timings = dict(cloud_load_ms=cloud_load*1000., cloud_upload_ms=cloud_upload*1000.,
            lines_labels_ms=(rendering-geometry_done)*1000., render_ms=(finished-rendering)*1000.,
            total_ms=(finished-started)*1000.)
        return packet

    def packet_status(self, packet):
        return '  OFFLINE PRECOMPUTED | Frame %d | %s | %d surface candidates' % (
            packet['sequence'], packet['mode'], packet['candidate_count'])

    def close(self):
        self.renderer.RemoveObserver(self.render_observer)
        for actor in self.actors:
            self.renderer.RemoveActor(actor)
        for label in self.labels:
            self.renderer.RemoveActor2D(label)
        self.raw_rep.Visibility = self.raw_visibility
        self.actors, self.labels = [], []


def make_overlay(smp, vtk, view, status_label, filename, reader):
    if os.environ.get('RSVIEW_CLEARANCE_COMPUTE') == 'online':
        try:
            from .clearance_native import VtkOnlineClearanceOverlay
        except (ImportError, ValueError):
            from clearance_native import VtkOnlineClearanceOverlay
        return VtkOnlineClearanceOverlay(smp, vtk, view, status_label, filename, reader)
    return VtkClearanceOverlay(smp, vtk, view, status_label, filename, reader)
