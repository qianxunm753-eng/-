"""Python-2-compatible bounded image controls. No NumPy, VTK or SDK imports."""
import math
import os
from collections import deque


def validate_control(command,request_id,source_key,snapshot_key=None,budget_s=60.,options=None):
    try:string_types=(basestring,)
    except NameError:string_types=(str,)
    try:integer_types=(int,long)
    except NameError:integer_types=(int,)
    if command not in ('freeze','enhance','surface','measure'):raise ValueError('invalid image control command')
    if not isinstance(request_id,string_types) or not 1<=len(request_id)<=64:raise ValueError('bounded ASCII control token required')
    try:request_id.encode('ascii')
    except UnicodeError:raise ValueError('ASCII control token required')
    if (not isinstance(source_key,(tuple,list)) or len(source_key)!=3
        or not all(isinstance(v,string_types) and v for v in source_key[:2])
        or isinstance(source_key[2],bool) or not isinstance(source_key[2],integer_types) or source_key[2]<0):
        raise ValueError('source session/clock/epoch required')
    source_key=tuple(source_key)
    seconds=float(budget_s)
    if math.isnan(seconds) or math.isinf(seconds) or not 0.<seconds<=60.:raise ValueError('finite 0--60 image budget required')
    if command=='freeze' and snapshot_key is not None:raise ValueError('freeze determines completed snapshot, not UI key')
    if command in ('enhance','surface','measure'):
        names=('session_id','clock_id','epoch','generation','revision','reference_sequence','reference_time_s','source_frames')
        if not isinstance(snapshot_key,dict) or set(snapshot_key)!=set(names):raise ValueError('complete image snapshot identity required')
        snapshot_key=dict(snapshot_key);snapshot_key['source_frames']=tuple(snapshot_key['source_frames'])
        if any(isinstance(snapshot_key[k],bool) or not isinstance(snapshot_key[k],integer_types) or snapshot_key[k]<0
               for k in ('generation','revision','reference_sequence')):
            raise ValueError('integer snapshot identity required')
        if (not all(isinstance(v,integer_types) and not isinstance(v,bool) and v>=0 for v in snapshot_key['source_frames'])
            or any(a>=b for a,b in zip(snapshot_key['source_frames'],snapshot_key['source_frames'][1:]))):
            raise ValueError('increasing integer accepted scan IDs required')
        if tuple(snapshot_key[k] for k in ('session_id','clock_id','epoch'))!=source_key:raise ValueError('snapshot source differs')
        if (not 1<=len(snapshot_key['source_frames'])<=20 or len(set(snapshot_key['source_frames']))!=len(snapshot_key['source_frames'])
            or snapshot_key['reference_sequence']!=snapshot_key['source_frames'][-1]):raise ValueError('actual accepted scan IDs required')
        if not isinstance(snapshot_key['reference_time_s'],integer_types+(float,)):
            raise ValueError('numeric actual reference required')
        stamp=float(snapshot_key['reference_time_s'])
        if math.isnan(stamp) or math.isinf(stamp):raise ValueError('finite actual reference required')
    result=dict(command=command,request_id=request_id,source_key=source_key,snapshot_key=snapshot_key,budget_s=seconds)
    if command=='surface':
        if not isinstance(options,dict) or set(options)!=set(('roi',)):raise ValueError('surface ROI options required')
        try:from .image_display import display_settings
        except (ImportError,ValueError):from image_display import display_settings
        roi=display_settings(False,1.,options['roi'])['roi']
        if roi is None:raise ValueError('surface ROI required')
        result['options']={'roi':roi}
    elif options is not None:raise ValueError('options are surface-only')
    return result


def control_reply(request,state,reason,key=None,pair=None,report=None):
    return dict(format='rsview-image-control-v1',command=request['command'],request_id=request['request_id'],
        source_key=tuple(request['source_key']),snapshot_key=key,state=state,reason=reason,report=report or {},pair=pair)


def wire_pair(pair):
    """Whitelist primitive buffers; snapshot ndarray/origin sidecars never cross IPC."""
    return dict((name,pair[name]) for name in ('fused','single','clean_fused','clean_single','clean_counts',
        'sequence','time','source_frames','raw_points','bytes','surface_views') if name in pair)


class ImageControlChannel(object):
    def __init__(self,host,enabled,clock,read,write,remove,consume):
        self.host=host;self.enabled=enabled;self.clock=clock;self.read=read;self.write=write;self.remove=remove
        self.consume=consume
        self.pending=self.running=self.result=None;self.suspended=False;self.deadline=None
        self.active_id=None;self.frozen_key=None;self.cancelled=False;self.seen=deque(maxlen=128)

    def path(self,name):return self.host._path('image-control-'+name)

    def request(self,command,request_id,source_key,snapshot_key,budget_s,options=None):
        if not self.enabled:raise ValueError('P3 controls are baseline recorded image-only')
        request=validate_control(command,request_id,source_key,snapshot_key,budget_s,options)
        if self.host.closed or self.host.failed:return False
        current=self.host.current
        if current is None or tuple(current[k] for k in ('session_id','clock_id','epoch'))!=request['source_key']:
            raise ValueError('control source is not current scan source')
        if (command,request_id) in self.seen:raise ValueError('duplicate image control command token')
        if self.running is not None or self.pending is not None or self.result is not None:return False
        if command=='freeze':
            self.deadline=self.clock()+request['budget_s'];self.active_id=request_id
            self.frozen_key=None;self.cancelled=False;self.remove(self.path('cancel'))
        elif self.active_id!=request_id or self.frozen_key!=request['snapshot_key'] or not self.suspended:
            raise ValueError('enhance requires its own completed freeze receipt')
        self.suspended=True;self.pending=request;self.seen.append((command,request_id))
        self.dispatch();return True

    def dispatch(self):
        if self.pending is None or self.running is not None or self.host.running is not None or self.host.pending is not None:return
        request=self.pending;remaining=min(request['budget_s'],60.,max(0.,self.deadline-self.clock()))
        if remaining<=0.:
            self.pending=None;self.result=control_reply(request,'cancelled','deadline',self.frozen_key);self.cancelled=True;return
        request=dict(request,budget_s=remaining)
        self.write(self.path('request'),request)
        self.pending=None;self.running=request

    def cancel(self,request_id):
        if request_id!=self.active_id:return False
        if self.cancelled:return True  # Do not recreate an already consumed cancellation receipt.
        self.cancelled=True
        request=self.running or self.pending or dict(command='enhance',request_id=request_id,source_key=tuple(
            (self.host.current or {}).get(k) for k in ('session_id','clock_id','epoch')))
        self.pending=None
        if self.running is not None and not os.path.isfile(self.path('cancel')):
            self.write(self.path('cancel'),dict(request_id=request_id))
        self.result=control_reply(request,'cancelled','cancelled',self.frozen_key)
        return True

    def pump(self):
        path=self.path('result')
        packet=self.consume(path)
        if packet is not None:
            running=self.running;self.running=None
            if running is not None and packet.get('request_id')==running['request_id'] and packet.get('command')==running['command']:
                current=self.host.current or {}
                same_source=packet.get('source_key')==tuple(current.get(k) for k in ('session_id','clock_id','epoch'))
                expired=self.deadline is not None and self.clock()>=self.deadline
                if not same_source or self.cancelled or expired:
                    if self.result is None:self.result=control_reply(running,'cancelled','stale_or_expired',self.frozen_key)
                else:
                    self.result=packet
                    if packet.get('state')=='frozen':self.frozen_key=packet.get('snapshot_key')
        if self.deadline is not None and self.clock()>=self.deadline and not self.cancelled and (self.pending or self.running):
            self.cancel(self.active_id)

    def poll(self):
        packet,self.result=self.result,None
        return packet

    def resume(self):
        if self.running is not None or self.pending is not None or self.host.failed or self.host.closed:return False
        reply=self.result;current=self.host.current or {}
        if (self.cancelled and reply is not None and reply.get('format')=='rsview-image-control-v1'
            and reply.get('state')=='cancelled' and reply.get('request_id')==self.active_id
            and tuple(reply.get('source_key',()))==tuple(current.get(k) for k in ('session_id','clock_id','epoch'))):
            self.result=None  # Only this terminal old cancellation, after its worker request has settled.
        self.suspended=False;return True

    def worker_failed(self,error):
        request=self.running or self.pending
        if request is not None:self.result=control_reply(request,'failed','worker_failed',self.frozen_key,report={'error':error})
        self.pending=self.running=None


def run_control(engine,request,directory,read):
    """Called only in the existing serial scientific worker, never in embedded Python."""
    import time
    from .fusion.state import RefinementBudget
    cancel_path=os.path.join(directory,'image-control-cancel.pkl')
    def cancelled():
        return (not os.path.isdir(directory) or os.path.isfile(cancel_path)
            and read(cancel_path).get('request_id')==request['request_id'])
    budget=RefinementBudget(temporary_limit_bytes=(128 if request['command']=='measure' else 64)*1024**2,
        deadline_s=time.monotonic()+min(60.,request['budget_s']),cancelled=cancelled)
    if cancelled():return control_reply(request,'cancelled','cancelled',request.get('snapshot_key'))
    return engine.process_image_control(request,budget)
