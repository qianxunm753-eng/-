"""Python2-safe serial native replay, separate monotonic capture deadline."""
import math
import os
import time


def capture_clock():
    monotonic=getattr(time,'monotonic',None)
    if monotonic is not None:return monotonic()
    if os.name=='nt':
        try:
            import ctypes
            get_tick=getattr(ctypes.windll.kernel32,'GetTickCount64',None)
            if get_tick is not None:
                get_tick.restype=ctypes.c_ulonglong;get_tick.argtypes=[]
                return get_tick()/1000.
        except ImportError:pass
        # CPython 2.7 on Windows implements time.clock with the native
        # QueryPerformanceCounter elapsed-time clock (not Unix CPU time).
        # This stripped RSView build has time.clock but no _ctypes extension.
        performance=getattr(time,'clock',None)
        if performance is not None:return performance()
    raise RuntimeError('No monotonic parking capture clock available')


def _finite(value):
    try:return not math.isnan(value) and not math.isinf(value)
    except (TypeError,ValueError):return False


def _token(value):
    try:string_types=(basestring,)
    except NameError:string_types=(str,)
    if not isinstance(value,string_types) or not 1<=len(value)<=128:
        raise ValueError('bounded ASCII capture token required')
    try:value.encode('ascii')
    except UnicodeError:raise ValueError('ASCII capture token required')
    return value


class ParkingReplayDriver(object):
    def __init__(self,frame_ids,select_frame,clock=capture_clock):
        values=list(frame_ids)
        if (not values or any(not _finite(v) or v<0 or int(v)!=v for v in values)
            or any(b<=a for a,b in zip(values,values[1:]))):
            raise ValueError('nonempty increasing actual frame IDs required')
        self.frame_ids=tuple(int(v) for v in values);self.select_frame=select_frame;self.clock=clock
        self.state='idle';self.reason='not_started';self.request_id=None;self.source_key=None
        self.pending_frame=None;self.previous_frame=None;self.generation_floor=0;self.deadline_s=0.
        self.last_parking={}

    @property
    def active(self):return self.state in ('awaiting_stability','collecting')

    def _now(self):
        now=float(self.clock())
        if not _finite(now):raise ValueError('finite monotonic clock required')
        return now

    def start(self,request_id,frame,source_key):
        token=_token(request_id);source=tuple(source_key)
        if frame not in self.frame_ids:raise ValueError('selected real frame unavailable')
        if len(source)!=3 or not source[0] or not source[1]:raise ValueError('source identity required')
        now=self._now();self.request_id=token;self.source_key=source
        self.deadline_s=now+60.;self.generation_floor=0;self.previous_frame=None
        self.state='awaiting_stability';self.reason='manual_start';self.last_parking={}
        self._select(int(frame));return self.tick()

    def _select(self,frame):
        if not self.active or self._now()>=self.deadline_s:return
        self.pending_frame=frame
        try:self.select_frame(frame)
        except Exception:
            self.state='incomplete';self.reason='frame_unavailable';self.pending_frame=None

    def tick(self):
        now=self._now()
        if self.active and now>=self.deadline_s:
            self.state='incomplete';self.reason='deadline';self.pending_frame=None
        return dict(self.last_parking,state=self.state,reason=self.reason,request_id=self.request_id,
            capture_generation=self.generation_floor,remaining_s=max(0.,self.deadline_s-now),
            pending_frame=self.pending_frame,previous_frame=self.previous_frame)

    def request_metadata(self):
        snapshot=self.tick()
        return dict(capture_request_id=self.request_id if self.active else None,
            capture_remaining_s=snapshot['remaining_s'] if self.active else 0.)

    def accepts(self,packet):
        self.tick()
        if not self.active or packet.get('sequence')!=self.pending_frame:return False
        if tuple(packet.get(k) for k in ('session_id','clock_id','epoch'))!=self.source_key:return False
        parking=packet.get('parking')
        if not isinstance(parking,dict) or parking.get('request_id')!=self.request_id:return False
        # Failed work cannot contribute geometry. The exact command/source/
        # input identity suffices to terminate promptly even if its worker
        # lost the current generation while constructing the error packet.
        if packet.get('status')=='failed':return True
        generation=parking.get('capture_generation')
        if isinstance(generation,bool) or not _finite(generation) or int(generation)!=generation or generation<self.generation_floor:
            return False
        return (parking.get('state') in ('idle','awaiting_stability','collecting','capture_complete','incomplete')
            and _finite(parking.get('remaining_s')) and 0.<=parking['remaining_s']<=60.)

    def ack(self,packet):
        if not self.accepts(packet):return False
        if packet.get('status')=='failed':
            self.state='incomplete';self.reason='worker_failed';self.pending_frame=None;return True
        parking=dict(packet['parking']);self.last_parking=parking
        self.generation_floor=max(self.generation_floor,int(parking['capture_generation']))
        self.deadline_s=min(self.deadline_s,self._now()+parking['remaining_s'])
        frame=self.pending_frame;self.previous_frame=frame;self.pending_frame=None
        self.state=parking['state'];self.reason=parking.get('reason',self.state)
        if not self.active:return True
        position=self.frame_ids.index(frame)+1
        if position==len(self.frame_ids):self.state='incomplete';self.reason='eof';return True
        self._select(self.frame_ids[position]);return True

    def seeked(self,frame):
        if self.active and frame!=self.pending_frame:
            self.cancel('manual_seek');return True
        return False

    def cancel(self,reason):
        self.state='idle';self.reason=str(reason);self.pending_frame=None
