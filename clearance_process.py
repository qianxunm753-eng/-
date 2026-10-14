"""Python-2-compatible, nonblocking native host for current-cloud clearance.

One request is in flight; only the newest eligible pending request is retained.
Private fixed IPC files contain primitives and packed buffers (never NumPy or
VTK objects). Protocol-2 pickle is read only from this owned temporary directory.
"""
import json
import math
import os
try:
    import cPickle as pickle
except ImportError:
    import pickle
import shutil
import subprocess
import sys
import tempfile
import time
from collections import deque


def _clock():
    return getattr(time, 'monotonic', time.time)()


def _parking_metadata(frame):
    token=frame.get('capture_request_id')
    try:string_types=(basestring,)
    except NameError:string_types=(str,)
    if token is not None:
        if not isinstance(token,string_types) or not 1<=len(token)<=128:
            raise ValueError('bounded ASCII capture token required')
        try:token.encode('ascii')
        except UnicodeError:raise ValueError('ASCII capture token required')
    try:remaining=float(frame.get('capture_remaining_s',0.))
    except (TypeError,ValueError):raise ValueError('finite remaining capture duration required')
    if math.isnan(remaining) or math.isinf(remaining) or not 0.<=remaining<=60.:
        raise ValueError('capture duration must be within 0--60 seconds')
    return dict(capture_request_id=token,capture_remaining_s=remaining)


def _parking_failure(metadata):
    return dict(request_id=metadata.get('capture_request_id'),state='incomplete',reason='worker_failed',
        capture_generation=0,remaining_s=0.)


def _remove(path):
    try:
        os.unlink(path)
    except OSError:
        if os.path.exists(path):
            raise


def _write(path, value):
    with open(path+'.tmp', 'wb') as stream:
        pickle.dump(value, stream, 2)
    os.rename(path+'.tmp', path)


def _read(path):
    with open(path, 'rb') as stream:
        if sys.version_info[0] >= 3:
            return pickle.load(stream, encoding='latin1')
        return pickle.load(stream)


def _consume_result(path):
    """Nonblocking read+unlink; leave the request in flight on Windows file locks."""
    if not os.path.isfile(path):return None
    try:
        packet=_read(path)
        if not isinstance(packet,dict):raise ValueError('private result must be a primitive packet')
        _remove(path)
        return packet
    except (IOError,OSError) as error:
        if getattr(error,'errno',None)==13 or getattr(error,'winerror',None) in (5,32,33):return None
        raise


def _buffer(value, dtype, columns, count=None):
    if value is None:
        return None
    widths = {'<f8':8,'<f4':4,'<i4':4,'<u2':2,'<i2':2,'<u1':1,'<i8':8}
    if isinstance(value, dict):
        data, actual_dtype = value['data'], value.get('dtype', dtype)
        width = widths.get(actual_dtype)
        if width is None:
            raise ValueError('unsupported packed buffer dtype')
        length = int(value['count'])
        if length < 0 or len(data) != length*columns*width:
            raise ValueError('packed buffer length mismatch')
    else:
        # NumPy belongs to the caller's environment; embedded RSView can supply
        # packed buffers directly. Retain float64 XYZ to preserve native minima.
        actual_dtype = dtype
        array = value.astype(dtype, copy=False)
        shape = tuple(array.shape)
        if columns == 3 and (len(shape) != 2 or shape[1] != 3):
            raise ValueError('XYZ must have shape N,3')
        if columns == 1 and len(shape) != 1:
            raise ValueError('point times/rings must be vectors')
        length = len(array)
        data = array.tobytes() if hasattr(array, 'tobytes') else array.tostring()
    if count is not None and length != count:
        raise ValueError('current point buffer count mismatch')
    return dict(data=data,count=length,dtype=actual_dtype)


def _runtime(python):
    """Resolve Windows venv launchers to the actual owned interpreter."""
    prefix = os.path.dirname(os.path.dirname(os.path.abspath(python)))
    cfg = os.path.join(prefix,'pyvenv.cfg')
    libraries = []
    if os.path.isfile(cfg):
        with open(cfg) as stream:
            entries = dict((parts[0].strip(),parts[1].strip())
                           for parts in (line.split('=',1) for line in stream if '=' in line))
        base = os.path.join(entries.get('home','').strip(),'python.exe' if os.name == 'nt' else 'python')
        if not os.path.isfile(base):
            raise RuntimeError('venv base interpreter not found: '+base)
        python = base
        site = os.path.join(prefix,'Lib','site-packages')
        if os.path.isdir(site):
            libraries.append(site)
    return python,libraries


class ClearanceProcess(object):
    def __init__(self, python=None, config=None, period_s=.2, source_mode='live',
                 experimental_pose_path=None, max_points=2000000, worker_timeout_s=30.):
        if period_s <= 0 or max_points <= 0 or worker_timeout_s <= 0:
            raise ValueError('positive period, point limit and timeout required')
        if source_mode not in ('live','recorded_pcap','simulated_pcap'):
            raise ValueError('invalid source mode')
        if (config or {}).get('image_fusion_only') and source_mode!='recorded_pcap':
            raise ValueError('image fusion is replay-only')
        if experimental_pose_path and source_mode != 'simulated_pcap':
            raise ValueError('recorded poses require explicit simulated_pcap mode')
        if python is None:
            if sys.version_info[0] >= 3:
                python = sys.executable
            else:
                with open(os.path.join(os.path.dirname(__file__),'worker_config.json')) as stream:
                    python = json.load(stream)['python']
                # The original rail worker used a dependency-free base Python.
                # Clearance needs the already present scientific environment.
                candidate = os.path.join(os.environ.get('LOCALAPPDATA',''),
                    'uv','cache','archive-v0','YrB8poOFTUorDkrQ','Scripts','python.exe')
                installed = os.path.join(os.path.dirname(python),'Lib','site-packages','numpy')
                if not os.path.isdir(installed) and os.path.isfile(candidate):
                    python = candidate
        if not os.path.isfile(python):
            raise RuntimeError('Calculation Python not found: '+str(python))
        python,libraries = _runtime(python)
        self.directory = tempfile.mkdtemp(prefix='rsview-clearance-')
        self.period_s,self.max_points,self.timeout = float(period_s),int(max_points),float(worker_timeout_s)
        self.source_mode = source_mode
        self.parking_mode=bool((config or {}).get('image_fusion_only') and
            (config or {}).get('image_contour_mode')=='parking')
        self.parking_metadata=None
        self.running = self.pending = self.result = self.current = None
        self.last_queued = None
        self.next_time = None
        self.epoch = 0
        self.pending_replacements = self.stale_results = 0
        self.closed = self.failed = False
        self.events = deque(maxlen=128)
        self.child = None
        try:from .image_control import ImageControlChannel
        except (ImportError,ValueError):from image_control import ImageControlChannel
        self.image_control=ImageControlChannel(self,bool((config or {}).get('image_fusion_only') and
            (config or {}).get('image_contour_mode','baseline')=='baseline' and source_mode=='recorded_pcap'),
            _clock,_read,_write,_remove,_consume_result)
        self.error_stream = open(os.path.join(self.directory,'worker-error.log'),'w+b')
        env = os.environ.copy()
        for name in ('PYTHONHOME','PYTHONPATH','PYTHONSTARTUP'):
            env.pop(name,None)
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)),'clearance_process.py')
        args = [python,script,self.directory,str(os.getpid())]
        if sys.version_info[0] == 2 and os.name == 'nt':
            args = [a.encode('mbcs') if isinstance(a,unicode) else a for a in args]
        try:
            _write(self._path('settings'),dict(config=config,source_mode=source_mode,
                experimental_pose_path=experimental_pose_path,libraries=libraries))
            self.child = subprocess.Popen(args,stdout=self.error_stream,stderr=self.error_stream,
                env=env,creationflags=0x08000000 if os.name == 'nt' else 0)
        except Exception:
            self.error_stream.close()
            shutil.rmtree(self.directory)
            raise

    def _path(self, name):
        return os.path.join(self.directory,name+'.pkl')

    @staticmethod
    def _identity(request):
        return dict(sequence=request['sequence'],device_time_s=request['device_time_s'],
                    session_id=request['session_id'],clock_id=request['clock_id'],epoch=request['epoch'])

    def submit(self, frame, force=False, payload_factory=None):
        """Queue a period-eligible scan, or one explicit paused-PCAP refresh.

        A forced refresh uses the actual last scan's identity/time, never a
        fabricated device tick. It does not move the regular device cadence
        and cannot queue the same identity twice. Live callers use force=False.
        """
        if self.closed or self.failed:
            return False
        if getattr(self,'image_control',None) is not None and self.image_control.suspended:return False
        parking=_parking_metadata(frame) if getattr(self,'parking_mode',False) else None
        timestamp,sequence = float(frame['device_time_s']),int(frame['sequence'])
        if math.isnan(timestamp) or math.isinf(timestamp):
            raise ValueError('finite device timestamp required')
        session,clock = frame.get('session_id','live:default'),frame.get('clock_id','device:default')
        if not session or not clock:
            raise ValueError('source session/clock required')
        reset = self.current is not None and (session != self.current['session_id'] or
            clock != self.current['clock_id'] or timestamp < self.current['device_time_s'] or
            sequence < self.current['sequence'] or
            (sequence == self.current['sequence'] and timestamp != self.current['device_time_s']) or
            (timestamp == self.current['device_time_s'] and sequence != self.current['sequence']))
        if reset:
            self.epoch += 1
            self.next_time = None
            self.pending = self.result = None
            self.last_queued = None
            self.events.append(dict(event='source_reset',epoch=self.epoch))
        same = self.current is not None and (sequence == self.current['sequence'] and
            timestamp == self.current['device_time_s'] and clock == self.current['clock_id'] and
            session == self.current['session_id'])
        self.current = dict(sequence=sequence,device_time_s=timestamp,session_id=session,
                            clock_id=clock,epoch=self.epoch)
        self._pump()
        if self.failed or (same and not force and self.current == self.last_queued) or (force and self.current == self.last_queued):
            return False
        if not force and self.next_time is not None and timestamp+1e-5 < self.next_time:
            return False
        received_at = time.time()
        packing_started = _clock()
        if payload_factory is not None:
            payload = payload_factory()
            if any(payload.get(key) != frame.get(key) for key in
                   ('sequence','device_time_s','session_id','clock_id')):
                raise ValueError('lazy native payload identity changed while copying')
            if parking is not None:
                copied=_parking_metadata(payload)
                if (copied['capture_request_id']!=parking['capture_request_id']
                    or copied['capture_remaining_s']>parking['capture_remaining_s']):
                    raise ValueError('lazy capture identity or deadline changed while copying')
                parking=copied
            frame = payload
        native_pack_ms = (_clock()-packing_started)*1000.
        xyz = _buffer(frame['xyz'],'<f8',3)
        if xyz['count'] > self.max_points:
            raise ValueError('current scan exceeds bounded input point limit; no measurement subsampling')
        point_times = _buffer(frame.get('point_times'),'<f8',1,xyz['count'])
        rings = _buffer(frame.get('rings'),'<i4',1,xyz['count'])
        request = dict(self.current,xyz=xyz,point_times=point_times,rings=rings,
            source_sha256=frame.get('source_sha256'),
            scan_span_s=tuple(frame.get('scan_span_s',(timestamp,timestamp))),
            pose=frame.get('pose'),received_at=received_at,native_pack_ms=native_pack_ms)
        request['native_pack_routes'] = dict((name,frame[name].get('packing','array'))
            for name in ('xyz','point_times','rings') if isinstance(frame.get(name),dict))
        if parking is not None:
            request.update(parking);self.parking_metadata=dict(parking)
        if not force:
            anchor = timestamp if self.next_time is None else self.next_time
            self.next_time = anchor+(max(1,int((timestamp-anchor+1e-5)/self.period_s)+1))*self.period_s
        else:
            self.events.append(dict(event='paused_refresh',**self.current))
        if self.pending is not None:
            self.pending_replacements += 1
            self.events.append(dict(event='pending_replaced',**self._identity(self.pending)))
        self.pending = request
        self.last_queued = dict(self.current)
        self._dispatch()
        return True

    def _dispatch(self):
        if getattr(self,'image_control',None) is not None and self.image_control.running is not None:return
        if self.running is None and self.pending is not None and not self.failed:
            request = self.pending
            _write(self._path('request'),request)
            # Keep the accepted payload retryable if an IPC write raises.
            self.pending = None
            self.running = self._identity(request)
            self.running['received_at'] = request['received_at']
            self.running['sent_monotonic'] = _clock()
        if getattr(self,'image_control',None) is not None:self.image_control.dispatch()

    def request_image_control(self,command,request_id,source_key,snapshot_key=None,budget_s=60.,options=None):
        self._pump()
        return self.image_control.request(command,request_id,source_key,snapshot_key,budget_s,options)

    def poll_image_control(self):
        self._pump()
        return self.image_control.poll()

    def cancel_image_control(self,request_id):
        return self.image_control.cancel(request_id)

    def resume_image_submissions(self):
        self._pump()
        return self.image_control.resume()

    def _failure(self, error):
        if getattr(self,'image_control',None) is not None:self.image_control.worker_failed(error)
        self.failed = True
        self.pending = None
        identity = self.current or self.running
        self.result = dict(identity or {},status='failed',error=error,measurements=[],
            _display={},calculation='online_current_cloud',source_mode=self.source_mode,
            current_source=False,fresh_distance=False)
        if getattr(self,'parking_mode',False) and self.parking_metadata is not None:
            self.result['parking']=_parking_failure(self.parking_metadata)
        self.running = None
        self.events.append(dict(event='worker_failed',error=error))

    def _pump(self):
        if self.closed or self.failed:
            return
        if getattr(self,'image_control',None) is not None:self.image_control.pump()
        path = self._path('result')
        packet=_consume_result(path)
        if packet is not None:
            self.running = None
            if packet.get('epoch') == self.epoch and (self.current is None or
                (packet.get('session_id') == self.current['session_id'] and
                 packet.get('clock_id') == self.current['clock_id'])):
                self.result = packet
            else:
                self.stale_results += 1
        if self.child.poll() is not None:
            self.error_stream.seek(0)
            detail = self.error_stream.read(2000).decode('utf-8','replace')
            self._failure('Calculation process exited (%s): %s' % (self.child.returncode,detail))
            return
        if self.running is not None and _clock()-self.running['sent_monotonic'] > self.timeout:
            self.child.terminate()
            self._failure('Calculation process timeout after %.3fs' % self.timeout)
            return
        self._dispatch()

    def poll(self):
        self._pump()
        packet,self.result = self.result,None
        if packet is not None:
            identity = dict((k,packet.get(k)) for k in ('sequence','device_time_s','session_id','clock_id','epoch'))
            packet['current_source'] = identity == self.current
            packet['fresh_distance'] = bool(packet.get('status') != 'failed' and
                                            packet['current_source'] and packet.get('measurements'))
            packet['result_age_s'] = max(0.,time.time()-packet.get('received_at',time.time()))
        return packet

    def tick(self):
        return self.poll()

    def diagnostics(self):
        self._pump()
        return dict(running=self.running,pending=self._identity(self.pending) if self.pending else None,
            current=self.current,epoch=self.epoch,pending_replacements=self.pending_replacements,
            stale_results=self.stale_results,failed=self.failed,
            alive=not self.closed and self.child.poll() is None,events=list(self.events),
            period_s=self.period_s,max_points=self.max_points)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.pending = self.result = None
        if self.child is not None and self.child.poll() is None:
            self.child.terminate()
        if self.child is not None:
            self.child.wait()
        self.error_stream.close()
        target = os.path.realpath(self.directory)
        base = os.path.realpath(tempfile.gettempdir())
        if os.path.dirname(target) != base or not os.path.basename(target).startswith('rsview-clearance-'):
            raise RuntimeError('refusing cleanup outside owned temporary directory')
        # Windows may briefly retain the terminated worker's log handle.
        # Retry only sharing/lock violations, only in this validated owned
        # directory, and report a persistent error rather than hiding it.
        for attempt in range(10):
            try:
                shutil.rmtree(target)
                break
            except OSError as error:
                if getattr(error, 'winerror', None) not in (32, 33) or attempt == 9:
                    raise
                time.sleep(.05)


def run(directory, parent_pid):
    # Add the repository parent; executing this file directly must retain package
    # imports without inheriting embedded Python's PYTHONPATH or PYTHONHOME.
    sys.path.insert(0,os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    settings = _read(os.path.join(directory,'settings.pkl'))
    _remove(os.path.join(directory,'settings.pkl'))
    for path in settings.pop('libraries',[]):
        import site
        site.addsitedir(path)
    if (settings.get('config') or {}).get('image_fusion_only',False):
        from rsview_centerline.fusion.engine import ImageFusionEngine
        engine_type=ImageFusionEngine
    elif (settings.get('config') or {}).get('stop_window_fusion',False):
        from rsview_centerline.clearance.stop_fusion import StopFusionEngine
        engine_type=StopFusionEngine
    else:
        from rsview_centerline.clearance.online import OnlineEngine
        engine_type=OnlineEngine
    engine = engine_type(**settings)
    parent_handle = None
    if os.name == 'nt':
        import ctypes
        kernel = ctypes.windll.kernel32
        kernel.OpenProcess.restype = ctypes.c_void_p
        parent_handle = kernel.OpenProcess(0x00100000,False,parent_pid)
        if not parent_handle:
            return
        kernel.WaitForSingleObject.argtypes = [ctypes.c_void_p,ctypes.c_ulong]
        kernel.CloseHandle.argtypes = [ctypes.c_void_p]
    try:
        while os.path.isdir(directory):
            if parent_handle:
                if kernel.WaitForSingleObject(parent_handle,0) != 258:
                    return
            elif os.getppid() != parent_pid:
                return
            path = os.path.join(directory,'request.pkl')
            control_path=os.path.join(directory,'image-control-request.pkl')
            if os.path.isfile(control_path):
                request=_read(control_path);_remove(control_path)
                from rsview_centerline.image_control import run_control,control_reply
                try:packet=run_control(engine,request,directory,_read)
                except Exception as exc:
                    packet=control_reply(request,'failed','control_failed',request.get('snapshot_key'),report={'error':str(exc)[:2000]})
                packet['worker_pid']=os.getpid()
                _write(os.path.join(directory,'image-control-result.pkl'),packet)
                continue  # Optional failure never resets the baseline engine.
            if not os.path.isfile(path):
                time.sleep(.005)
                continue
            request = _read(path)
            _remove(path)
            started = time.time()
            try:
                packet = engine.process(request,request['epoch'])
            except Exception as exc:
                packet = dict(ClearanceProcess._identity(request),status='failed',
                    error=str(exc)[:2000],measurements=[],_display={},
                    calculation='online_current_cloud',source_mode=settings['source_mode'])
                if ((settings.get('config') or {}).get('image_fusion_only')
                    and (settings.get('config') or {}).get('image_contour_mode')=='parking'):
                    packet['parking']=_parking_failure(_parking_metadata(request))
                engine = engine_type(**settings)  # A partly processed failed scan cannot seed history.
            packet.update(received_at=request['received_at'],worker_pid=os.getpid(),
                          worker_finished_at=time.time())
            packet.setdefault('timing_ms',{})['queue'] = max(0.,(started-request['received_at'])*1000.)
            _write(os.path.join(directory,'result.pkl'),packet)
    finally:
        if parent_handle:
            kernel.CloseHandle(parent_handle)


if __name__ == '__main__':
    run(sys.argv[1],int(sys.argv[2]))
