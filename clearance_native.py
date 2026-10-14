# -*- coding: utf-8 -*-
"""Native Python2 RSView online adapter; never load cached measurement results.

Each result carries its own current-source cloud snapshot. Slower-than-input
results stay visible but are labelled delayed; never put old geometry on a new
raw cloud and call it current. Live GPS/INS interpolation is not yet supplied.
"""
import array
import hashlib
import json
import os
import sys
import time
import uuid

try:
    from .clearance_overlay import VtkClearanceOverlay, MemoryPacketCache
    from .clearance_process import ClearanceProcess
except (ImportError, ValueError):
    from clearance_overlay import VtkClearanceOverlay, MemoryPacketCache
    from clearance_process import ClearanceProcess


def _clock():
    return getattr(time, 'monotonic', time.time)()


def _live_span(data):
    if data is None or data.GetPoints() is None:
        return None
    if not data.GetPoints().GetData().GetNumberOfTuples():
        return None
    stamps = data.GetPointData().GetArray('timestamp')
    return tuple(float(v) for v in stamps.GetRange()) if stamps is not None else None


def live_measurement_config():
    """Frozen native live/baseline preset; recorded experiments use a copy."""
    return dict(config_id='rsview-current-boundary-beta-v20',sensor_max_range_m=5.,
        measurement_height_range_m=(-.3,3.5),confirmed_background_path=None,
        confirmed_measurement_scope_path=None,live_motion_estimation=True,
        display_identity_seconds=1.2,display_balance_sides=True,
        rail_profile_roles=True,spatial_object_ownership=True,
        geometry_label_priority=True,track_structure_context=True,
        temporal_object_proposals=True,surface_object_proposals=True,
        local_terrain_validation=True,qualified_object_cores=True,qualified_instance_ownership=True,
        current_instance_reconciliation=True,adaptive_motion_compensation=True)


def recorded_measurement_config(variant=None):
    """Replay-only experiment, with an explicit baseline switch for A/B runs."""
    config = live_measurement_config()
    variant = (os.environ.get('RSVIEW_CLEARANCE_REPLAY_EXPERIMENT', '1') if variant is None else variant).strip().lower()
    if variant in ('0', 'false', 'off'):
        return config
    config.update(config_id='rsview-replay-scope-scanline-beta-v21',
                  measurement_height_range_m=None,
                  above_rail_measurement_only=True, scanline_object_proposals=True)
    # Keep the existing v21 scope as the replay default. The body
    # rule remains an explicit experiment until field identity improves.
    if variant == 'v22':
        config.update(config_id='rsview-replay-body-fragment-beta-v22',body_fragment_proposals=True)
    elif variant == 'v23':
        config.update(config_id='rsview-replay-qualified-history-beta-v23',qualified_history_reconciliation=True)
    elif variant == 'v24':
        config.update(config_id='rsview-replay-raw-identity-beta-v24',qualified_history_reconciliation=True,
                      raw_history_identity_support=True)
    elif variant == 'v25':
        config.update(config_id='rsview-replay-instance-owner-beta-v25',qualified_history_reconciliation=True,
                      raw_history_identity_support=True,qualified_history_owner_keys=True)
    elif variant in ('v26','v27','v28','v29','v30','v31','v32','final'):
        config.update(config_id='rsview-replay-integrated-beta-v26',qualified_history_reconciliation=True,
                      raw_history_identity_support=True,qualified_history_owner_keys=True,
                      observable_instance_seeding=True,stable_instance_labels=True)
        if variant in ('v27','v28','v29','v30','v31','v32','final'):
            config.update(config_id='rsview-replay-current-observation-beta-v27',current_observation_gating=True)
        if variant in ('v28','v29','v30','v31','v32','final'):
            config.update(config_id='rsview-replay-complete-current-closure-beta-v28',complete_current_reconciliation=True)
        if variant in ('v29','v30','v31','v32','final'):
            config.update(config_id='rsview-replay-exact-surface-work-beta-v29')
        if variant in ('v30','v31','v32','final'):
            config.update(config_id='rsview-replay-native-shared-edge-beta-v30',native_shared_edge_evidence=True)
        if variant in ('v31','v32','final'):
            config.update(config_id='rsview-replay-above-scope-uncertainty-beta-v31',above_rail_uncertainty=True)
        if variant in ('v32','final'):
            config.update(config_id='rsview-replay-bounded-fast-continuity-beta-v32',recorded_playback_continuity=True)
    return config


def stop_measurement_config():
    config=recorded_measurement_config('v32')
    config.update(config_id='rsview-stop-window-beta-v33',stop_window_fusion=True,
        sensor_max_range_m=10.,stop_window_stages=(20,40,80,160),stop_window_budget_s=60.)
    return config


class PlaybackScanGate(object):
    """Adjacent scans, or explicitly known bounded forward-play steps only."""
    def __init__(self, allow_fast_play=False, max_gap_s=.6):
        if not 0.<float(max_gap_s)<float('inf'):
            raise ValueError('positive finite replay history gap required')
        self.allow_fast_play,self.max_gap_s=bool(allow_fast_play),float(max_gap_s)
        self.frame = self.span = None
        self.discontinuous = False

    def observe(self, frame, span, playback_step=0):
        frame, span = int(frame), tuple(span)
        if self.frame == frame and self.span == span:
            return False
        if self.frame is not None:
            monotonic=span[0]>=self.span[0] and span[1]>self.span[1]
            known_skip=(self.allow_fast_play and playback_step>1 and frame-self.frame==playback_step
                and monotonic and max(span[1]-self.span[1],
                    (sum(span)-sum(self.span))*.5)<=self.max_gap_s+1e-9)
            self.discontinuous=not monotonic or (frame!=self.frame+1 and not known_skip)
        else:
            self.discontinuous=False
        self.frame, self.span = frame, span
        return True


def patch_recording_decoder(logic):
    """Scope the verified 1212121.pcap's decoder to one open operation.

    Native dialog getters can otherwise be empty/default RS16 after launch.
    RSAIRY=13 is verified from this installed build's SensorComboBox data;
    these are offline decoder inputs, not device/WEB or persistent GUI writes.
    The caller MUST restore the returned factories in its finally block.
    """
    values = {
        'vvCalibrationDialog': dict(GetSensorType=13,GetSensorTypeName='RSAIRY',
            GetJumboFrameNum=0,GetCalibrationPath='',GetPosition=''),
        'vvNetworkConfigPcapDialog': dict(GetUseVlan=0,GetUserLayerBytes=0,
            GetTailLayerBytes=0,GetMsopPort=6699,GetDifopPort=7788,GetInputTypeStr='2 PCAP'),
        'vvLaserSelectionDialog': dict(getLaserSelection='1'*480,getLaserDefault480='1'*480),
        'vvColumnSelectionDialog': dict(getColumnSelection='1'),
        'vvPointsSelectionDialog': dict(getPointsSelection=''),
        'vvCropReturnsDialog': dict(GetCropReturns='0 0 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000'),
        'vvCropDistReturnsDialog': dict(GetCropDistReturns='0 0 0.000000 5.000000')}
    class DialogAdapter:
        def __init__(self,native,overrides):
            self.native,self.overrides=native,overrides
        def __getattr__(self,name):
            if name in self.overrides:
                return lambda:self.overrides[name]
            return getattr(self.native,name)
    def wrap(factory,overrides):
        return lambda *args,**kwargs:DialogAdapter(factory(*args,**kwargs),overrides)
    originals = dict((name,getattr(logic,name)) for name in values)
    for name,overrides in values.items():
        setattr(logic,name,wrap(originals[name],overrides))
    return originals


def prepare_pcap_reader(reader):
    """Prime RSView's native decoder before enumerating its lazy time steps.

    UpdatePipelineInformation alone can leave TimestepValues empty. This is
    the same initial native decode used by the verified full-frame exporter,
    not a retry that silently substitutes another recording or frame.
    """
    reader.GetClientSideObject().Modified()
    reader.UpdatePipeline(1.)
    reader.UpdatePipelineInformation()
    return list(reader.TimestepValues)


def _packed(data, columns, count):
    types = {'float':('f','<f4'), 'double':('d','<f8'), 'int':('i','<i4'),
             'unsigned short':('H','<u2'), 'short':('h','<i2'),
             'unsigned char':('B','<u1')}
    code, dtype = types[data.GetDataTypeAsString()]
    if data.GetNumberOfComponents() != columns or data.GetNumberOfTuples() != count:
        raise ValueError('Native point attribute length/components mismatch')
    packing='vtk_buffer'
    try:
        raw = (str(buffer(data)) if sys.version_info[0] == 2 else memoryview(data).tobytes())
    except (TypeError, AttributeError, BufferError):
        packing='tuple_fallback'
        values = array.array(code)
        for i in range(count):
            row = data.GetTuple(i)
            values.extend(row if code in ('f','d') else (int(v) for v in row))
        raw = values.tostring() if sys.version_info[0] == 2 else values.tobytes()
    if sys.byteorder != 'little':
        values = array.array(code)
        (values.fromstring if sys.version_info[0] == 2 else values.frombytes)(raw)
        values.byteswap()
        raw = values.tostring() if sys.version_info[0] == 2 else values.tobytes()
    return dict(data=raw, count=count, dtype=dtype,packing=packing)


def native_request(polydata, sequence, session, clock):
    data = polydata.GetPoints().GetData()
    count = data.GetNumberOfTuples()
    stamps = polydata.GetPointData().GetArray('timestamp')
    if stamps is None or not count:
        raise ValueError('Online clearance requires a current scan with device timestamps')
    span = tuple(float(t) for t in stamps.GetRange())
    if not all(abs(t) < float('inf') for t in span) or span[0] > span[1]:
        raise ValueError('Invalid native device time range')
    attrs = polydata.GetPointData()
    rings = next((attrs.GetArray(name) for name in ('Ring','ring','laser_id')
                  if attrs.GetArray(name) is not None), None)
    return dict(sequence=int(sequence), device_time_s=sum(span)/2.,
        session_id=session, clock_id=clock, scan_span_s=span,
        xyz=_packed(data,3,count), point_times=_packed(stamps,1,count),
        rings=_packed(rings,1,count) if rings is not None else None)


def recorded_request(polydata, frame, session, clock, vtk):
    """Verify the native decoded frame; do not infer device time from UI time."""
    info, key = polydata.GetInformation(), vtk.vtkDataObject.DATA_TIME_STEP()
    if not info.Has(key) or info.Get(key) != float(frame):
        raise ValueError('Requested frame %s is still decoding; previous scan is not accepted' % frame)
    return native_request(polydata, frame, session, clock)


def display_state(packet, identity, now):
    keys = ('sequence','device_time_s','session_id','clock_id','epoch')
    current = identity is not None and all(packet.get(k) == identity.get(k) for k in keys)
    return dict(current_source=bool(current),
        fresh_distance=bool(current and packet.get('status') == 'completed' and packet.get('measurements')
            and packet.get('measurement_basis')!='rolling_20_window'),
        result_age_s=max(0.,now-packet.get('received_at',now)))


def validate_scan_identity(request, expected_span):
    if any(abs(a-b) > 1e-6 for a,b in zip(request['scan_span_s'],expected_span)):
        raise ValueError('Requested frame %s is still decoding; previous scan is not accepted' % request['sequence'])


class LiveScanGate(object):
    """Only new native scans refresh live liveness; a UI tick is not a scan."""
    def __init__(self, timeout_s=1.5):
        self.timeout_s = timeout_s
        self.running = False
        self.span = self.last_seen = self.started_at = None

    def resume(self, now, baseline=None):
        self.running = True
        self.span, self.last_seen, self.started_at = baseline, None, now

    def pause(self):
        self.running = False
        self.span = self.last_seen = self.started_at = None

    def observe(self, span, now):
        if not self.running or span is None or span == self.span:
            return False
        self.span, self.last_seen = span, now
        return True

    def stale(self, now):
        since = self.last_seen if self.last_seen is not None else self.started_at
        return self.running and since is not None and now-since >= self.timeout_s-1e-9


class VtkOnlineClearanceOverlay(VtkClearanceOverlay):
    def __init__(self, smp, vtk, view, status_label, filename, reader,
                 source_mode='simulated_pcap',pose_provider=None,worker_config=None):
        if worker_config is not None and source_mode!='recorded_pcap':
            raise ValueError('custom replay config cannot enter live/simulated overlay')
        # Only a verified recording can use its explicit experimental map/prior.
        self.source_mode, self.pose_provider = source_mode,pose_provider
        self.source_sha256 = None
        self.scan_spans = {}
        if source_mode == 'simulated_pcap':
            with open(os.path.join(os.path.dirname(__file__),'clearance_online_config.json'),'rb') as stream:
                manifest = json.load(stream)
            if manifest.get('source_mode') != 'simulated_pcap':
                raise ValueError('Recorded poses require explicit online simulation settings')
            digest = hashlib.sha256()
            with open(filename,'rb') as stream:
                for block in iter(lambda:stream.read(1024*1024),b''):
                    digest.update(block)
            if digest.hexdigest() != manifest['source_sha256']:
                raise ValueError('Online simulation settings belong to a different recording')
            self.source_sha256 = digest.hexdigest()
            self.session, self.clock = manifest['session_id'],manifest['clock_id']
            with open(manifest['native_metadata'],'rb') as stream:
                native_metadata = json.load(stream)
            self.scan_spans = dict((row['frame'],tuple(row['timestamp_range'])) for row in native_metadata['frames'])
        elif source_mode in ('live', 'recorded_pcap'):
            self.session = ('live:' if source_mode == 'live' else 'pcap:')+uuid.uuid4().hex
            self.clock = 'lidar-device:'+self.session
            manifest = dict(config=(worker_config if worker_config is not None else
                                    recorded_measurement_config() if source_mode == 'recorded_pcap'
                                    else live_measurement_config()),poses_source=None)
        else:
            raise ValueError('invalid native online source mode')
        self.reader = reader
        self.live_span = None
        self.live_sequence = 0
        self.live_gate = LiveScanGate()
        self.live_stale = False
        self.playback_gate = PlaybackScanGate()
        self.worker_config = manifest['config']
        if source_mode == 'live':
            self.live_gate.resume(_clock(), _live_span(reader.GetClientSideObject().GetOutput()))
        self.host = ClearanceProcess(config=manifest['config'], source_mode=source_mode,
            experimental_pose_path=manifest['poses_source'],
            period_s=.05 if manifest['config'].get('image_fusion_only') else .2,
            worker_timeout_s=70. if (manifest['config'].get('stop_window_fusion') or
                manifest['config'].get('image_contour_mode','baseline')!='baseline') else 30.)
        self.closed = False
        self.last_input = None
        self.requested_frame = None
        self.input_error = None
        self.last_input_at = None
        self.idle_flush_frame = None
        self.recorded_result_floor = None
        self.image_single_view=False;self.image_clean_view=False
        try:
            limit=int(manifest['config'].get('image_display_limit',150000)) if manifest['config'].get('image_fusion_only') else 20000
            VtkClearanceOverlay.__init__(self,smp,vtk,view,status_label,filename,reader,
                packet_cache=MemoryPacketCache(max_points=limit),cloud_point_limit=limit)
            if manifest['config'].get('image_fusion_only'):
                for unused,actor in self.cloud_actors.values():
                    actor.GetProperty().SetPointSize(2.)
                    actor.GetProperty().SetColor(.45,.84,.96)
                try:from .image_display import ImageAppearance
                except (ImportError,ValueError):from image_display import ImageAppearance
                self.image_appearance=ImageAppearance(self)
            if source_mode != 'live':
                reader.GetClientSideObject().SetNumberOfTrailingFrames(0)
            from PythonQt import QtCore
            self.timer = QtCore.QTimer()
            self.timer.connect('timeout()', self._timer_tick)
            self.timer.start(40)
            status_label.setText('  %s | device trigger 0.2s | awaiting current scan' %
                ('LIVE BETA / UNCOMPENSATED' if source_mode == 'live' else
                 'PCAP REPLAY / UNCOMPENSATED' if source_mode == 'recorded_pcap' else 'ONLINE SIMULATION'))
        except Exception:
            try:
                timer=getattr(self,'timer',None)
                if timer is not None:
                    timer.stop();timer.disconnect('timeout()',self._timer_tick)
                appearance=getattr(self,'image_appearance',None)
                if appearance is not None:appearance.close()
            finally:
                try:
                    if hasattr(self,'render_observer'):VtkClearanceOverlay.close(self)
                finally:self.host.close()
            raise

    def _timer_tick(self):
        if self.closed:
            return
        if self.source_mode == 'live':
            if not self.live_gate.running:
                return
            try:
                # Copy only a newly arrived native scan. This reads the sensor
                # output; it does not connect/configure GNSS or invent a pose.
                self.reader.UpdatePipelineInformation()
                steps = list(self.reader.TimestepValues)
                if steps:
                    self.reader.UpdatePipeline(float(steps[-1]))
                else:
                    self.reader.UpdatePipeline()
                new_scan = self._accept_live(self.reader.GetClientSideObject().GetOutput(), _clock())
            except Exception as error:
                self._clear_live('LIVE INPUT ERROR | '+str(error)[:180])
                self.input_error = str(error)
                return
            # Native buffer copying may exceed the timeout on a loaded PC.
            # Do not let a NEW scan expire its own freshly submitted work.
            if not new_scan and self.live_gate.stale(_clock()):
                if not self.live_stale:
                    self.live_stale = True
                    self._clear_live('LIVE BETA | NO NEW SCAN | distances cleared')
                return
        self.poll()

    def _restart_live_worker(self):
        self.session = 'live:'+uuid.uuid4().hex
        self.clock = 'lidar-device:'+self.session
        self.host = ClearanceProcess(config=self.worker_config, source_mode='live',
            experimental_pose_path=None, period_s=.2)
        self.live_sequence = 0

    def _clear_live(self, reason):
        host, self.host = self.host, None
        try:
            if host is not None:
                host.close()
        except Exception as error:
            reason += ' | cleanup warning: '+str(error)[:180]
        self.current = self.last_result = None
        self.last_input = None
        self._hide()
        self.raw_rep.Visibility = self.raw_visibility
        self.status_label.setText('  '+reason)
        self.smp.Render(self.view)

    def set_stream_running(self, running):
        if self.closed or self.source_mode != 'live':
            return
        if not running:
            self.live_gate.pause()
            self._clear_live('LIVE BETA | PAUSED | distances cleared')
        elif not self.live_gate.running:
            baseline = _live_span(self.reader.GetClientSideObject().GetOutput())
            self._restart_live_worker()
            self.live_gate.resume(_clock(), baseline)
            self.live_stale = False
            self.input_error = None
            self.status_label.setText('  LIVE BETA / UNCOMPENSATED | awaiting NEW native scan')

    def _accept_live(self, data, now):
        span = _live_span(data)
        if not self.live_gate.running or span is None or span == self.live_gate.span:
            return False
        if not all(abs(t)<float('inf') for t in span) or span[0]>span[1]:
            raise ValueError('Invalid native device time range')
        if self.host is None:
            self._restart_live_worker()
        identity = dict(sequence=self.live_sequence,device_time_s=sum(span)/2.,
                        session_id=self.session,clock_id=self.clock)
        def pack_current_scan():
            request = native_request(data,self.live_sequence,self.session,self.clock)
            if self.pose_provider is not None:
                request['pose'] = self.pose_provider(request)
            return request
        old_epoch = self.host.epoch
        self.host.submit(identity,payload_factory=pack_current_scan)
        self.live_gate.observe(span, now)
        self.live_stale = False
        self.last_input = self.live_sequence
        self.live_sequence += 1
        self.live_span = span
        self.input_error = None
        if old_epoch != self.host.epoch:
            self.current = self.last_result = None
            self._hide()
            self.raw_rep.Visibility = self.raw_visibility
        return True

    def update(self, polydata, frame):
        if self.closed:
            return None
        if self.source_mode == 'live':
            self._accept_live(polydata, _clock())
            return self.poll()
        if self.source_mode == 'recorded_pcap':
            return self._update_recorded(frame)
        if frame != self.last_input:
            self.requested_frame = frame
            try:
                self.reader.UpdatePipeline(float(frame))
                polydata = self.reader.GetClientSideObject().GetOutput()
                request = native_request(polydata,frame,self.session,self.clock)
                validate_scan_identity(request,self.scan_spans[int(frame)])
                request['source_sha256'] = self.source_sha256
                old_epoch = self.host.epoch
                self.host.submit(request)
                self.last_input = frame
                self.last_input_at, self.idle_flush_frame = time.time(), None
                self.input_error = None
                if self.host.epoch != old_epoch:
                    self.current = self.last_result = None
                    self._hide()
                    self.raw_rep.Visibility = self.raw_visibility
            except Exception as error:
                self.input_error = str(error)
                self.current = self.last_result = None
                self._hide()
                self.raw_rep.Visibility = self.raw_visibility
        return self.poll()

    def _update_recorded(self, frame, poll=True):
        p3=getattr(self,'p3_driver',None)
        if p3 is not None and p3.blocks_inputs:
            if int(frame)==getattr(self,'p3_ui_frame',None):return self.poll() if poll else None
            p3.cancel('replay_advanced');self.set_image_detail_view(False)
            controller=getattr(getattr(getattr(self,'logic',None),'app',None),'imageFusionController',None)
            if controller is not None and controller.overlay is self:controller.cancel_p3_detail('replay_advanced')
            if p3.awaiting_release:
                self.p3_pending_frame=int(frame);return self.poll() if poll else None
        parking=getattr(self,'worker_config',{}).get('image_contour_mode')=='parking'
        driver=getattr(self,'parking_driver',None)
        if parking:
            if driver is None:return None
            driver.seeked(int(frame));driver.tick()
            if not driver.active:return None
        self.requested_frame = int(frame)
        try:
            self.reader.UpdatePipeline(float(frame))
            data = self.reader.GetClientSideObject().GetOutput()
            # Validate before changing history or accepting any previous scan.
            info, key = data.GetInformation(), self.vtk.vtkDataObject.DATA_TIME_STEP()
            if not info.Has(key) or info.Get(key) != float(frame):
                raise ValueError('Requested frame %s is still decoding; previous scan is not accepted' % frame)
            span = _live_span(data)
            if span is None or not all(abs(t)<float('inf') for t in span) or span[0]>span[1]:
                raise ValueError('PCAP current frame has no valid device timestamp range')
            config=getattr(self,'worker_config',{})
            proposed = PlaybackScanGate(config.get('recorded_playback_continuity',False),
                min(.6,float(config.get('history_seconds',.6))))
            proposed.frame, proposed.span = self.playback_gate.frame, self.playback_gate.span
            # Use this installed SDK's actual player state/step map, not wall
            # time or inferred playback. Manual/seek-button/reverse steps keep
            # the previous epoch-reset policy; decoded/current guards precede it.
            app=getattr(getattr(self,'logic',None),'app',None)
            step=0
            if getattr(app,'playing',False) and not getattr(app,'seekPlay',False):
                direction=int(getattr(app,'playDirection',0))
                step={4:1,5:1,-4:-1,-5:-1}.get(direction,direction)
            if parking and driver.previous_frame==proposed.frame and driver.pending_frame==int(frame):
                step=int(frame)-driver.previous_frame if driver.previous_frame is not None else 0
            if proposed.observe(frame, span,step):
                session, clock = self.session, self.clock
                if proposed.discontinuous:
                    if parking:
                        driver.cancel('device_time_discontinuity');return None
                    # New identity makes the host discard old pending/results,
                    # and the worker resets all rail/object/relative-pose history.
                    session = 'pcap:'+uuid.uuid4().hex
                    clock = 'lidar-device:'+session
                    self.current = self.last_result = None
                    self.recorded_result_floor = int(frame)
                    self._hide()
                    self.raw_rep.Visibility = self.raw_visibility
                identity = dict(sequence=int(frame), device_time_s=sum(span)/2.,
                                session_id=session, clock_id=clock)
                if parking:identity.update(self.parking_request_metadata())
                def pack():
                    payload=recorded_request(data, frame, session, clock, self.vtk)
                    if parking:payload.update(self.parking_request_metadata())
                    return payload
                self.host.submit(identity, force=parking or proposed.discontinuous, payload_factory=pack)
                if getattr(self.host, 'failed', False) or getattr(self.host, 'closed', False):
                    raise ValueError('Calculation worker unavailable; disable and re-enable measurement')
                # A scheduling skip still records the current identity. A
                # packing/submit exception must not commit this frame as seen.
                self.playback_gate = proposed
                self.session, self.clock = session, clock
                self.last_input = int(frame)
                self.last_input_at, self.idle_flush_frame = time.time(), None
            self.input_error = None
        except Exception as error:
            self.input_error = str(error)
            if parking:return self.poll() if poll else None
            self.recorded_result_floor = int(frame)
            self.current = self.last_result = None
            self._hide()
            self.raw_rep.Visibility = self.raw_visibility
        return self.poll() if poll else None

    def poll(self):
        if self.closed:
            return None
        if self.source_mode == 'live' and (not self.live_gate.running or self.host is None):
            return None
        p3=getattr(self,'p3_driver',None)
        if p3 is not None:
            if (p3.valid and getattr(p3,'command','enhance')=='measure' and
                p3.source_key!=(self.session,self.clock,getattr(self.host,'epoch',0))):
                p3.cancel('source_changed')
            control=p3.poll()
            if not p3.valid and getattr(self,'p3_accept_packet',None) is not None:
                controller=getattr(getattr(getattr(self,'logic',None),'app',None),'imageFusionController',None)
                if controller is not None and controller.overlay is self:controller.cancel_p3_detail(p3.reason)
                else:
                    self.set_image_detail_view(False);self.p3_accept_packet=None;self.p3_on_result=None
                if self.last_result is not None and self.last_result.get('p3') is not None:
                    self.last_result['p3'].update(state='cancelled',reason=p3.reason)
            accept=getattr(self,'p3_accept_packet',None)
            source=(self.session,self.clock,getattr(self.host,'epoch',0))
            if (control is not None and accept is not None and not self.input_error and source==tuple(control.get('source_key',()))
                and accept(control)):
                self._display_p3_packet(control)
                callback=getattr(self,'p3_on_result',None)
                if callback is not None:callback(control)
            if p3.blocks_inputs:
                self.host.poll()  # Drain already completed ordinary result without replacing the frozen pair.
                if self.last_result is not None:
                    if getattr(p3,'command','enhance')=='measure':
                        self.last_result.update(display_state(self.last_result,self.host.current,time.time()))
                    if (p3.phase in ('failed','cancelled') and getattr(p3,'command','enhance')!='measure'
                        and not getattr(self,'p3_start_failure',None)):
                        self.status_label.setText('  P3 DETAIL | baseline image held | '+p3.reason+' | NO MEASUREMENT')
                    else:self.status_label.setText(self.packet_status(self.last_result))
                return self.last_result
            pending=getattr(self,'p3_pending_frame',None)
            if pending is not None:
                self.p3_pending_frame=None;self._update_recorded(pending,poll=False)
        parking=getattr(self,'worker_config',{}).get('image_contour_mode')=='parking'
        driver=getattr(self,'parking_driver',None)
        if parking and driver is not None:
            snapshot=driver.tick()
            if not driver.active:
                # Drain owned late work without popping display buffers or
                # decoding another scan. Keep the last validated A/B pair.
                if self.host is not None and not getattr(self.host,'closed',False):self.host.poll()
                if self.last_result is not None and self.last_result.get('parking') is not None:
                    self.last_result['parking'].update(snapshot)
                    self.status_label.setText(self.packet_status(self.last_result))
                else:self.status_label.setText('  PARK IMAGE | %s | %s | NO MEASUREMENT'%(snapshot['state'],snapshot['reason']))
                return None
        if parking and getattr(self,'parking_accept_packet',None) is None:return None
        if self.source_mode == 'recorded_pcap' and self.input_error:
            self._update_recorded(self.requested_frame, poll=False)
            if self.input_error:
                # A late old packet cannot restore geometry after a decode error.
                try:
                    self.host.poll()
                except Exception as error:
                    self.input_error = str(error)
                self.status_label.setText(self._p3_start_failure_status() or '  PCAP INPUT ERROR | '+self.input_error[:180])
                return None
        # The decoder/timekeeper can emit its first callback before the newly
        # requested native output is ready. Retry without accepting an old scan.
        if self.source_mode == 'simulated_pcap' and self.input_error and self.requested_frame != self.last_input:
            frame = self.requested_frame
            try:
                self.reader.UpdatePipeline(float(frame))
                request = native_request(self.reader.GetClientSideObject().GetOutput(),frame,self.session,self.clock)
                validate_scan_identity(request,self.scan_spans[int(frame)])
                request['source_sha256'] = self.source_sha256
                self.host.submit(request)
                self.last_input, self.input_error = frame, None
                self.last_input_at, self.idle_flush_frame = time.time(), None
            except Exception as error:
                self.input_error = str(error)
        if (self.source_mode == 'simulated_pcap' and not self.input_error and
                self.last_input_at is not None and time.time()-self.last_input_at >= .2 and
                self.idle_flush_frame != self.last_input):
            # Paused or end-of-file: the last arrived scan may be between the
            # scheduled device ticks. Compute it once, using its REAL timestamp.
            # Never apply this PCAP-only idle refresh to a disconnected live
            # sensor and pretend that the old scan is a new arrival.
            try:
                self.reader.UpdatePipeline(float(self.last_input))
                request = native_request(self.reader.GetClientSideObject().GetOutput(),
                                         self.last_input,self.session,self.clock)
                validate_scan_identity(request,self.scan_spans[int(self.last_input)])
                request['source_sha256'] = self.source_sha256
                self.host.submit(request,force=True)
                self.idle_flush_frame = self.last_input
            except Exception as error:
                self.input_error = str(error)
        if (self.source_mode == 'recorded_pcap' and not parking and self.last_input_at is not None and
                time.time()-self.last_input_at >= .2 and self.idle_flush_frame != self.last_input):
            # Paused, single-step or EOF: compute the selected real frame once,
            # including a frame skipped by regular device-time scheduling.
            try:
                self.reader.UpdatePipeline(float(self.last_input))
                request = recorded_request(self.reader.GetClientSideObject().GetOutput(),
                    self.last_input, self.session, self.clock, self.vtk)
                self.host.submit(request, force=True)
                self.idle_flush_frame = self.last_input
            except Exception as error:
                self.input_error = str(error)
                self.recorded_result_floor = self.requested_frame
                self.current = self.last_result = None
                self._hide()
                self.raw_rep.Visibility = self.raw_visibility
                self.status_label.setText('  PCAP INPUT ERROR | '+self.input_error[:180])
                return None
        try:
            packet = self.host.poll()
        except Exception as error:
            if self.source_mode != 'recorded_pcap':
                raise
            self.input_error = str(error)
            self.recorded_result_floor = self.requested_frame
            self.current = self.last_result = None
            self._hide()
            self.raw_rep.Visibility = self.raw_visibility
            self.status_label.setText('  PCAP INPUT ERROR | '+self.input_error[:180])
            return None
        if packet is not None:
            if parking and not self.parking_accept_packet(packet):return None
            if packet.get('status') == 'failed':
                if parking:
                    self.parking_on_result(packet)
                    self.status_label.setText('  PARK IMAGE | incomplete | worker failed | previous pair held')
                    return packet
                self._hide()
                self.current = None
                self.last_result = packet
                self.raw_rep.Visibility = self.raw_visibility
                self.status_label.setText('  ONLINE FAILED | '+packet.get('error','unknown error')[:180])
                self.smp.Render(self.view)
                return packet
            floor = getattr(self, 'recorded_result_floor', None)
            if self.source_mode == 'recorded_pcap' and floor is not None:
                identity = self.host.current
                same_source = identity is not None and all(packet.get(key) == identity.get(key)
                    for key in ('session_id', 'clock_id', 'epoch'))
                if not same_source or packet.get('sequence', -1) < floor:
                    self.status_label.setText('  PCAP REPLAY | awaiting selected frame %s' % self.last_input)
                    return None
                # New-segment snapshots are valid even if playback has already
                # advanced. Their own cloud stays paired and explicitly delayed.
                self.recorded_result_floor = None
            fused=packet.pop('_display')
            if packet.get('measurement_basis')=='image_only':
                packet['_image_pairs']=dict(fused=fused,single=packet.pop('_single_display'))
                if '_clean_display' in packet and '_clean_single_display' in packet:
                    packet['_image_pairs'].update(clean_fused=packet.pop('_clean_display'),
                        clean_single=packet.pop('_clean_single_display'))
                packet['clouds']=self._image_clouds(packet)
            else:packet['clouds']=fused
            self._advance_stop_status(packet)
            self.display_packet(packet)
            if parking:self.parking_on_result(packet)
        if self.input_error:
            if self.last_result is not None:
                self.last_result.update(current_source=False,fresh_distance=False)
            self.status_label.setText('  ONLINE INPUT ERROR | '+self.input_error[:180])
        elif self.last_result is not None and self.last_result.get('status') != 'failed':
            self.last_result.update(display_state(self.last_result,self.host.current,time.time()))
            if self._advance_stop_status(self.last_result) and self.last_result.get('measurements'):
                # Refresh colors/labels once on expiry. No decode, submission,
                # synthetic timestamp or new scan is involved in this tick.
                self.display_packet(self.last_result)
            self.status_label.setText(self.packet_status(self.last_result))
        return self.last_result

    def set_parking_receiver(self,accept_packet,on_result,request_metadata,driver=None):
        if (self.source_mode!='recorded_pcap' or not getattr(self,'worker_config',{}).get('image_fusion_only')
            or getattr(self,'worker_config',{}).get('image_contour_mode')!='parking'):
            if any(x is not None for x in (accept_packet,on_result,request_metadata,driver)):
                raise ValueError('parking receiver is recorded image-only')
        self.parking_accept_packet=accept_packet;self.parking_on_result=on_result
        self.parking_request_metadata=request_metadata;self.parking_driver=driver

    def set_p3_receiver(self,accept_packet,on_result,suspend_submissions):
        if (self.source_mode!='recorded_pcap' or not getattr(self,'worker_config',{}).get('image_fusion_only')
            or getattr(self,'worker_config',{}).get('image_contour_mode','baseline')!='baseline'):
            if accept_packet is not None or on_result is not None or suspend_submissions:
                raise ValueError('P3 receiver is baseline recorded image-only')
        self.p3_accept_packet=accept_packet;self.p3_on_result=on_result
        if suspend_submissions:self.p3_ui_frame=self.requested_frame if self.requested_frame is not None else self.last_input

    def _display_p3_packet(self,control):
        if self.closed:return
        key=control.get('snapshot_key');pair=control.get('pair')
        if key is None or pair is None:
            if control.get('command')=='measure':
                self.set_image_detail_view(False)
                if self.last_result is not None:
                    self.last_result['window_measurement_control']=dict(state='failed',reason=control.get('reason','failed'))
            elif self.last_result is not None:self.last_result['p3']=dict(state='failed',reason=control.get('reason','failed'))
            return
        if control['state']=='frozen' or not hasattr(self,'p3_baseline_packet'):
            packet=dict(self.last_result or {})
            packet.pop('pending_window_measurements',None);packet.pop('_qualified_window_measurements',None)
            self.window_pending_view=False
            packet.pop('surface',None);packet.pop('_surface_views',None);self.surface_view=None
            packet.update(sequence=key['reference_sequence'],device_time_s=key['reference_time_s'],session_id=key['session_id'],
                clock_id=key['clock_id'],epoch=key['epoch'],status='completed',measurement_basis='image_only',
                measurements=[],secondary_measurements=[],centerline=[],current_source=False,fresh_distance=False,
                fusion=dict((self.last_result or {}).get('fusion',{})),_image_pairs={})
            packet['fusion'].update(reference_sequence=key['reference_sequence'],reference_time_s=key['reference_time_s'],
                source_frames=list(key['source_frames']),scan_count=len(key['source_frames']),state='window_ready',age_s=0.,
                snapshot_key=key,clean_preview_counts=pair.get('clean_counts',{}))
            for name in ('fused','single','clean_fused','clean_single'):
                if name in pair:packet['_image_pairs'][name]={'object':pair[name]}
            self.p3_baseline_packet=packet;self.image_detail_view=False
        else:
            packet=dict(self.p3_baseline_packet);packet['_image_pairs']=dict(packet['_image_pairs'])
        if control.get('command')=='surface':
            packet.pop('p3',None)
            packet['surface']=dict(control.get('report',{}),state=control['state'],reason=control.get('reason',''))
            if control['state']=='accepted':
                packet['_surface_views']=pair['surface_views'];self.surface_view='local'
        elif control.get('command')=='measure':
            measured=control.get('report',{}).get('measurement_packet')
            expected=dict(sequence=key['reference_sequence'],device_time_s=key['reference_time_s'],
                session_id=key['session_id'],clock_id=key['clock_id'],epoch=key['epoch'])
            config=getattr(self,'worker_config',{})
            capable=config.get('image_window_measurement_capable') or config.get('image_window_measurement')
            valid=(capable and isinstance(measured,dict)
                and measured.get('measurement_basis')=='rolling_20_window'
                and all(measured.get(name)==value for name,value in expected.items()))
            window=measured.get('window_measurement',{}) if isinstance(measured,dict) else {}
            valid=(valid and len(key['source_frames'])==20 and window.get('scan_count')==20
                and tuple(window.get('source_frames',()))==tuple(key['source_frames'])
                and window.get('reference_sequence')==key['reference_sequence']
                and window.get('reference_time_s')==key['reference_time_s'])
            packet.pop('p3',None);self.image_detail_view=False
            packet['window_measurement_control']=dict(state=control['state'],reason=control.get('reason',''))
            if control['state']=='accepted' and valid:
                protected=('clouds','fusion','_image_pairs','p3','surface')
                packet.update(dict((name,value) for name,value in measured.items()
                    if not name.startswith('_') and name not in protected))
                for name in ('measurements','secondary_measurements'):
                    packet[name]=[dict(item,uncertain=True,measurement_basis='rolling_20_window')
                        for item in measured.get(name,[])]
                packet['_qualified_window_measurements']=list(packet['measurements'])
                packet['pending_window_measurements']=[dict(item,uncertain=True,window_raw_preview=True,
                    supported_distance_m=None,measurement_basis='rolling_20_window')
                    for item in measured.get('pending_window_measurements',[])[:8]]
                self.window_pending_view=False
                packet.update(current_scan_measurement=False,absolute_accuracy_verified=False,
                    semantic_one_object_one_label_verified=False,current_source=False,fresh_distance=False,
                    received_at=time.time(),result_age_s=0.)
            elif control['state']=='accepted':
                packet['window_measurement_control']=dict(state='baseline_retained',reason='invalid_measurement_packet')
        elif control['state']=='accepted':
            packet['_image_pairs']['detail_fused']={'object':pair['fused']}
            if 'clean_fused' in pair:packet['_image_pairs']['clean_detail_fused']={'object':pair['clean_fused']}
            self.image_detail_view=True
        if control.get('command') not in ('surface','measure'):
            packet['p3']=dict(state=control['state'],reason=control.get('reason',''),request_id=control['request_id'])
        packet['clouds']=self._image_clouds(packet);self.display_packet(packet)

    def set_image_surface_view(self,mode,center=0.,width=.08):
        if self.closed or self.source_mode!='recorded_pcap':return False
        packet=self.last_result
        if packet is None or packet.get('measurement_basis')!='image_only':return False
        key='local' if mode=='layer' else mode
        if mode is not None and key not in packet.get('_surface_views',{}):return False
        appearance=getattr(self,'image_appearance',None)
        if appearance is not None:appearance.surface_style(mode if mode in ('frames','support','layer') else None,center,width)
        self.surface_view=mode;packet['clouds']=self._image_clouds(packet);self.display_packet(packet)
        return True

    def set_image_detail_view(self,checked):
        if self.closed:return False
        packet=self.last_result
        if packet is not None and packet.get('measurement_basis')=='rolling_20_window':
            if checked:return False
            baseline=getattr(self,'p3_baseline_packet',None)
            if baseline is None:return False
            packet=dict(baseline);packet.pop('p3',None)
            packet.pop('window_measurement',None);packet.pop('window_measurement_control',None)
            packet.pop('pending_window_measurements',None);packet.pop('_qualified_window_measurements',None)
            self.window_pending_view=False
            packet.update(measurement_basis='image_only',measurements=[],secondary_measurements=[],centerline=[],
                current_scan_measurement=False,current_source=False,fresh_distance=False)
        if packet is None or packet.get('measurement_basis')!='image_only':return False
        if checked and 'detail_fused' not in packet.get('_image_pairs',{}):return False
        self.image_detail_view=bool(checked);packet['clouds']=self._image_clouds(packet);self.display_packet(packet)
        return True

    def set_window_pending_view(self,checked):
        packet=self.last_result
        if (self.closed or self.source_mode!='recorded_pcap' or packet is None
            or packet.get('measurement_basis')!='rolling_20_window'):return False
        pending=packet.get('pending_window_measurements',[])
        if checked and not pending:return False
        self.window_pending_view=bool(checked)
        qualified=packet['_qualified_window_measurements']
        packet['measurements']=list(qualified)+(list(pending) if checked else [])
        packet.update(fresh_distance=False,current_scan_measurement=False)
        self.display_packet(packet)
        return True

    def set_image_single_view(self,single):
        if self.source_mode!='recorded_pcap':return False
        packet=self.last_result
        if packet is not None and packet.get('measurement_basis')!='image_only':return False
        self.image_single_view=bool(single)
        if packet is not None and packet.get('_image_pairs') is not None:
            packet['clouds']=self._image_clouds(packet)
            # Same paired reference and persistent actors; no reader update,
            # worker submission, camera reset or reconstruction is involved.
            self.display_packet(packet)
        return True

    def display_packet(self,packet,frame=None):
        appearance=getattr(self,'image_appearance',None)
        if appearance is not None and packet is not None and packet.get('measurement_basis') in ('image_only','rolling_20_window'):
            appearance.prepare(packet)
        result=VtkClearanceOverlay.display_packet(self,packet,frame)
        finish=getattr(appearance,'finish',None)
        if finish is not None and packet is not None and packet.get('measurement_basis') in ('image_only','rolling_20_window'):
            finish(packet)
            if getattr(appearance,'surface_mode',None) in ('frames','support'):self.smp.Render(self.view)
        return result

    def set_image_display_settings(self,depth,point_size,roi=None):
        appearance=getattr(self,'image_appearance',None)
        if self.source_mode!='recorded_pcap' or appearance is None:return False
        driver=getattr(self,'p3_driver',None)
        measuring=driver is not None and driver.valid and getattr(driver,'command','enhance')=='measure'
        previous_roi=appearance.snapshot().get('roi') if measuring else None
        appearance.configure(depth,point_size,roi)
        packet=self.last_result or {};driver=getattr(self,'p3_driver',None)
        new_roi=tuple(float(v) for v in roi) if roi is not None else None
        if measuring and previous_roi!=new_roi:
            controller=getattr(getattr(getattr(self,'logic',None),'app',None),'imageFusionController',None)
            if controller is not None and controller.overlay is self:
                controller.cancel_p3_detail('roi_changed');controller.source_ready()
            else:
                driver.cancel('roi_changed');self.set_image_detail_view(False);self.set_p3_receiver(None,None,False)
            packet=self.last_result or {}
        surface=packet.get('surface',{})
        old_roi=surface.get('roi') if packet.get('_surface_views') else None
        if driver is not None and driver.valid and getattr(driver,'command','enhance')=='surface':
            old_roi=(driver.options or {}).get('roi')
        if old_roi is not None and tuple(old_roi)!=new_roi:
            # Diagnostic buffers were physically cropped at the frozen ROI.
            # A larger clipping box cannot recover points removed at that stage.
            controller=getattr(getattr(getattr(self,'logic',None),'app',None),'imageFusionController',None)
            if controller is not None and controller.overlay is self:
                controller.cancel_p3_detail('roi_changed',restore=False)
            else:
                if driver is not None:driver.cancel('roi_changed')
                self.set_p3_receiver(None,None,False)
            self.surface_view=None;appearance.surface_style(None)
            packet.pop('_surface_views',None)
            packet['surface']=dict(state='cancelled',reason='roi_changed')
            if packet.get('_image_pairs') is not None:packet['clouds']=self._image_clouds(packet)
            if controller is not None and controller.overlay is self:controller.source_ready()
        if self.last_result is not None:self.display_packet(self.last_result)
        return True

    def get_image_display_settings(self):
        appearance=getattr(self,'image_appearance',None)
        return appearance.snapshot() if appearance is not None else None

    def _image_clouds(self,packet):
        if packet.get('measurement_basis')=='rolling_20_window':return packet['_image_pairs']['fused']
        surface=getattr(self,'surface_view',None)
        if surface is not None:
            key='local' if surface=='layer' else surface
            entry=packet.get('_surface_views',{}).get(key)
            if entry is not None:return {'object':entry}
        pairs=packet['_image_pairs'];key='single' if self.image_single_view else 'fused'
        if key=='fused' and getattr(self,'image_detail_view',False) and 'detail_fused' in pairs:key='detail_fused'
        clean='clean_'+key
        return pairs[clean] if getattr(self,'image_clean_view',False) and clean in pairs else pairs[key]

    def set_image_clean_view(self,clean):
        if self.source_mode!='recorded_pcap':return False
        packet=self.last_result
        if packet is not None and packet.get('measurement_basis')!='image_only':return False
        if clean and packet is not None and 'clean_fused' not in packet.get('_image_pairs',{}):return False
        self.image_clean_view=bool(clean)
        if packet is not None and packet.get('_image_pairs') is not None:
            packet['clouds']=self._image_clouds(packet);self.display_packet(packet)
        return True

    def _advance_stop_status(self,packet):
        """Wall-budget status on pause/EOF, independent of duplicate scan gates."""
        if packet.get('measurement_basis')!='stop_window':return False
        window=packet['stop_window'];now=_clock()
        identity=tuple(packet.get(k) for k in ('session_id','clock_id','epoch'))+(window.get('operation_token'),)
        elapsed=max(0.,float(window.get('elapsed_s',0.)))
        # Python2 uses a wall clock, while the worker uses monotonic time.
        # Carry a relative duration rather than comparing their clock origins.
        transit=max(0.,time.time()-packet.get('worker_finished_at',time.time()))
        # Freeze this packet's receipt anchor. Subsequent UI ticks must not
        # subtract the already-advanced elapsed value or transit age again.
        proposed=window.setdefault('_native_budget_anchor_s',now-elapsed-transit)
        if identity!=getattr(self,'_stop_status_identity',None):
            self._stop_status_identity=identity;self._stop_status_started=proposed
        else:
            self._stop_status_started=min(self._stop_status_started,proposed)
        window['elapsed_s']=max(elapsed,now-self._stop_status_started)
        if window['elapsed_s']>float(window.get('budget_s',60.)) and window.get('state')!='complete':
            changed=window.get('state')!='time_budget_exceeded'
            window.update(state='time_budget_exceeded',confirmed=False,status_only_expiry=True)
            packet['fresh_distance']=False
            for item in packet.get('measurements',[]):item['uncertain']=True
            for item in packet.get('secondary_measurements',[]):item['uncertain']=True
            return changed
        return False

    def _p3_start_failure_status(self):
        if not getattr(self,'worker_config',{}).get('image_fusion_only'):return None
        source=(self.session,self.clock,getattr(self.host,'epoch',0))
        failure=getattr(self,'p3_start_failure',None)
        if (failure is None or failure.get('source')!=source
            or failure.get('frame')!=getattr(self,'requested_frame',None)):return None
        text='  P3 DETAIL | start failed | baseline held | '+failure['reason']+' | NO MEASUREMENT'
        if self.input_error:text+=' | PCAP INPUT ERROR | '+self.input_error[:180]
        return text

    def packet_status(self, packet):
        if (packet.get('measurement_basis')=='rolling_20_window' or
            (packet.get('measurement_basis')=='image_only' and getattr(self,'window_measurement_view',
                getattr(self,'worker_config',{}).get('image_window_measurement',False)))):
            failure=getattr(self,'p3_start_failure',None)
            source=(self.session,self.clock,getattr(self.host,'epoch',0))
            if (failure is not None and failure.get('command')=='measure' and failure.get('source')==source
                and failure.get('frame')==getattr(self,'requested_frame',None)):
                return '  20F WINDOW DISTANCE | start failed | baseline held | '+failure['reason']+' | NO DISTANCE | EXPERIMENTAL | NOT CURRENT SCAN'
            window=packet.get('window_measurement',{});fusion=packet.get('fusion',{})
            driver=getattr(self,'p3_driver',None);control=packet.get('window_measurement_control',{})
            state=window.get('state',control.get('state','image ready; calculate explicitly'))
            if driver is not None and driver.processing:
                state='freezing same window' if driver.phase=='freeze' else 'calculating window distance'
            elif driver is not None and getattr(driver,'command','enhance')=='measure' and not window:
                state='baseline held: '+driver.reason
            if control.get('reason') and not window:state+='; '+control['reason']
            reference=window.get('reference_sequence',fusion.get('reference_sequence','-'))
            reference_time=window.get('reference_time_s',fusion.get('reference_time_s',packet.get('device_time_s')))
            source_time=(getattr(self.host,'current',None) or {}).get('device_time_s',reference_time)
            lag=max(0.,source_time-reference_time) if source_time is not None and reference_time is not None else 0.
            count=window.get('scan_count',len(window.get('source_frames',fusion.get('source_frames',[]))))
            if not count:count=fusion.get('scan_count',0)
            distances=str(len(packet.get('measurements',[])))+' distances' if packet.get('measurements') else 'NO DISTANCE'
            if packet.get('measurement_basis')=='rolling_20_window':
                qualified=packet.get('_qualified_window_measurements',packet.get('measurements',[]))
                visible=packet.get('measurements',[])[:packet.get('display_limit',4)]
                pending_shown=sum(1 for item in visible if item.get('window_raw_preview'))
                distances='qualified %s | pending shown %s/%s'%(len(qualified),pending_shown,
                    len(packet.get('pending_window_measurements',[])))
            return '  20F WINDOW DISTANCE | %s | %s/20 real scans | ref %s | source age %.2fs | result age %.2fs | %s | EXPERIMENTAL | NOT CURRENT SCAN'%(
                state,count,reference,lag,packet.get('result_age_s',0.),distances)
        if packet.get('measurement_basis')=='image_only':
            f=packet['fusion'];mode='SINGLE' if getattr(self,'image_single_view',False) else 'FUSED'
            source=(self.session,self.clock,getattr(self.host,'epoch',0))
            failure=self._p3_start_failure_status()
            if failure is not None:return failure
            driver=getattr(self,'p3_driver',None)
            if driver is not None and driver.processing and driver.source_key==source:
                progress='freezing completed baseline' if driver.phase=='freeze' else 'computing refinement'
                prefix='SURFACE DIAG' if getattr(driver,'command','enhance')=='surface' else 'P3 DETAIL'
                return '  '+prefix+' | '+progress+' | baseline held | %s/20 | NO MEASUREMENT'%f['scan_count']
            surface=packet.get('surface')
            if surface is not None:
                if surface.get('reason')=='roi_changed':
                    return '  SURFACE DIAG | ROI changed; baseline restored; enable diagnosis again | NO MEASUREMENT'
                if getattr(self,'surface_view',None) in ('support','stable'):
                    return '  SURFACE DIAG | %s | %s/20 | ref %s | stable %s / all %s | 20mm; >=3 scans | weak != noise | unchanged XYZ | NO MEASUREMENT'%(
                        self.surface_view,f['scan_count'],f['reference_sequence'],surface.get('stable_point_count','-'),surface.get('local_unique_count','-'))
                return '  SURFACE DIAG | %s | %s/20 | ref %s | local %s / base %s | unchanged XYZ | %s | NO MEASUREMENT'%(
                    getattr(self,'surface_view',None) or 'baseline',f['scan_count'],f['reference_sequence'],
                    surface.get('local_unique_count','-'),surface.get('baseline_roi_count','-'),surface.get('reason',''))
            p3=packet.get('p3')
            if p3 is not None:
                if f['scan_count']<20:return '  P3 DETAIL | base window incomplete %s/20 | NO MEASUREMENT'%f['scan_count']
                state=('enhanced' if getattr(self,'image_detail_view',False) else 'baseline') if p3['state']=='accepted' else 'baseline held'
                return '  P3 DETAIL | %s | %s | %s/20 | ref %s | %s | NO MEASUREMENT'%(
                    mode,state,f['scan_count'],f['reference_sequence'],p3.get('reason',''))
            if packet.get('parking') is not None:
                p=packet['parking']
                return '  PARK IMAGE | %s | %s | g%s/p%s | w/a/t %s/%s/%s | ref %s | age %.2fs | NO MEASUREMENT'%(
                    mode,p.get('state','idle'),p.get('capture_generation',0),p.get('pair_generation','-'),
                    p.get('retained_count',0),p.get('actual_count',0),p.get('trusted_count',0),
                    f.get('reference_sequence','-'),f.get('age_s',0.))
            imu='IMU + LIDAR' if packet.get('relative_pose_diagnostics',{}).get('imu_active') else 'LIDAR ONLY'
            current=getattr(self.host,'current',None) or {}
            reference=f.get('reference_time_s',packet.get('device_time_s'))
            input_time=current.get('device_time_s',reference)
            lag=max(0.,input_time-reference) if input_time is not None and reference is not None else f.get('age_s',0.)
            freshness='PAIRED INPUT' if packet.get('current_source') else 'DELAYED PAIR'
            filtered=getattr(self,'image_clean_view',False) and 'clean_fused' in packet.get('_image_pairs',{})
            display='LIGHT FILTER' if filtered else 'RAW VIEW'
            if filtered:
                counts=f.get('clean_preview_counts',{});prefix='single' if mode=='SINGLE' else 'fused'
                display+=' %s/%s'%(counts.get(prefix+'_kept','?'),counts.get(prefix+'_raw','?'))
            if 'effective_scan_count' in f:
                layer=f.get('refinement_state','baseline')
                if f['state']=='held_pair':layer='held:'+layer
                return '  IMAGE | %s | %s | %s | window %s contributed %s | ref %s/input %s | lag %.2fs age %.2fs | 10m %s | NO MEASUREMENT'%(
                    mode,layer,'CURRENT' if packet.get('current_source') else 'DELAYED',f.get('window_scan_count',0),
                    f['effective_scan_count'],f['reference_sequence'],self.last_input,lag,packet.get('result_age_s',0.),display)
            prefix='IMAGE ONLY / MOTION DETAIL' if f.get('motion_detail_enabled',False) else 'IMAGE ONLY'
            return '  '+prefix+' | %s | %s | %s | %s | %s/20 real scans | input %s / ref %s | lag %.2fs | result age %.2fs | 10m | %s | NO MEASUREMENT' % (
                mode,f['state'],imu,freshness,f['scan_count'],self.last_input,f['reference_sequence'],
                lag,packet.get('result_age_s',0.),display)
        if packet.get('measurement_basis')=='stop_window':
            window=packet.get('stop_window',{})
            return '  STOP WINDOW REPLAY | %s | 10m | scans %s / checkpoint %s | reference %s | %.1fs / 60s | %s distances' % (
                window.get('state','waiting'),window.get('scan_count',0),window.get('stage_scans',0),
                window.get('reference_sequence','-'),window.get('elapsed_s',0.),len(packet.get('measurements',[])))
        prefix = ('ONLINE SIM / '+packet.get('motion_status','raw') if self.source_mode == 'simulated_pcap'
                  else ('PCAP REPLAY / ' if self.source_mode == 'recorded_pcap' else 'LIVE BETA / ')+('LIDAR RELATIVE / NO GPS' if
                    packet.get('pose_scope') == 'live_lidar_relative_experimental' and
                    packet.get('relative_pose_diagnostics',{}).get('usable') and
                    packet.get('motion_status') == 'deskewed_experimental' else 'UNCOMPENSATED'))
        if self.source_mode == 'recorded_pcap' and getattr(self, 'worker_config', {}).get('above_rail_measurement_only'):
            prefix = prefix.replace('PCAP REPLAY / ', 'PCAP REPLAY EXPERIMENT / ')
        count_text=str(len(packet.get('measurements',[])))
        if packet.get('observable_instance_seeding',False):
            count_text+=' (%d unconfirmed)' % len(packet.get('secondary_measurements',[]))
        return '  %s | input %s / result %s | %s | %s, %s distances | compute %.0fms | age %.2fs | trigger 0.2s' % (
            prefix,self.last_input,packet['sequence'],
            'CURRENT SCAN' if packet.get('current_source') else 'DELAYED SNAPSHOT',
            packet.get('mode','unknown'),count_text,
            packet.get('timing_ms',{}).get('packet_total',0.), packet.get('result_age_s',0.))

    def close(self):
        if self.closed:
            return
        self.window_pending_view=False
        p3=getattr(self,'p3_driver',None)
        if p3 is not None:p3.cancel('overlay_closed')
        self.p3_driver=None;self.p3_accept_packet=None;self.p3_on_result=None
        driver=getattr(self,'parking_driver',None)
        if driver is not None:driver.cancel('overlay_closed')
        self.parking_driver=None;self.parking_accept_packet=None;self.parking_on_result=None
        self.closed = True
        self.timer.stop()
        appearance=getattr(self,'image_appearance',None)
        if appearance is not None:appearance.close()
        host, self.host = self.host, None
        try:
            if host is not None:
                host.close()
        finally:
            VtkClearanceOverlay.close(self)


def enable_live(logic,pose_provider=None):
    """Explicit beta entry for an already connected RSView sensor.

    No recorded map/prior/pose can enter this path. The optional supplied pose
    still needs independent GPS/INS calibration/time coverage before deskew.
    """
    sensor=logic.getSensor()
    if sensor is None:
        raise ValueError('Connect a live RSView lidar first; PCAP playback is not a live sensor')
    if logic.app.railOverlay is not None:
        logic.app.railOverlay.close()
        logic.app.railOverlay = None
    logic.app.railOverlay=VtkOnlineClearanceOverlay(logic.smp,logic.vtk,logic.app.mainView,
        logic.app.statusLabel,None,sensor,source_mode='live',pose_provider=pose_provider)
    return logic.app.railOverlay


def enable_recorded(logic,worker_config=None):
    """Opt-in ordinary Airy PCAP calculation with the live preset, no priors."""
    reader = logic.getReader()
    filename = logic.getReaderFileName() if reader is not None else ''
    if reader is None or logic.app.curLidarType != 'RSAIRY' or not filename.lower().endswith('.pcap'):
        raise ValueError('Open an Airy PCAP recording first')
    if logic.app.railOverlay is not None:
        logic.app.railOverlay.close()
        logic.app.railOverlay = None
    kwargs=dict(source_mode='recorded_pcap')
    if worker_config is not None:kwargs['worker_config']=worker_config
    overlay = VtkOnlineClearanceOverlay(logic.smp, logic.vtk, logic.app.mainView,
        logic.app.statusLabel, filename, reader, **kwargs)
    # Only the recorded adapter consults this SDK state; absent state keeps
    # the conservative seek policy. Connect it before the first menu update.
    overlay.logic = logic
    logic.app.railOverlay = overlay
    return overlay


def enable_recorded_stop(logic):
    return enable_recorded(logic,stop_measurement_config())
