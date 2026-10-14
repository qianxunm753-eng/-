"""Python-2-compatible one-shot P3 freeze/enhance lifecycle, no SDK reads."""
import uuid
import math


class P3ReplayEnhancement(object):
    def __init__(self,host,clock,command='enhance',options=None):
        if command not in ('enhance','surface','measure'):raise ValueError('bounded frozen-window command required')
        self.command=command;self.options=options
        self.host=host;self.clock=clock;self.request_id=None;self.source_key=None;self.snapshot_key=None
        self.phase='idle';self.reason='idle';self.valid=False;self.deadline=None;self.awaiting_release=False

    @property
    def processing(self):return self.valid and self.phase in ('freeze','enhance')

    @property
    def blocks_inputs(self):return self.valid or self.awaiting_release

    def _now(self):
        now=float(self.clock())
        if math.isnan(now) or math.isinf(now):raise ValueError('finite monotonic image clock required')
        return now

    def begin(self,source_key):
        if self.processing:return False
        now=self._now()
        if not self.host.resume_image_submissions():self.awaiting_release=True;return False
        self.request_id=uuid.uuid4().hex;self.source_key=tuple(source_key);self.snapshot_key=None
        self.deadline=now+60.;self.phase='freeze';self.valid=True;self.reason='freezing'
        try:
            accepted=self.host.request_image_control('freeze',self.request_id,self.source_key,budget_s=60.)
        except Exception:
            self.valid=False;self.phase='failed';self.reason='freeze_unavailable';raise
        if not accepted:self.valid=False;self.phase='failed';self.reason='worker_busy'
        return accepted

    def accepts(self,packet):
        if (not self.valid or packet.get('format')!='rsview-image-control-v1' or packet.get('request_id')!=self.request_id
            or tuple(packet.get('source_key',()))!=self.source_key or self._now()>=self.deadline):return False
        key=packet.get('snapshot_key')
        if key is not None:
            key=dict(key);key['source_frames']=tuple(key.get('source_frames',()))
            if tuple(key.get(k) for k in ('session_id','clock_id','epoch'))!=self.source_key:return False
        if packet.get('command')==self.command:return self.snapshot_key is not None and key==self.snapshot_key
        return packet.get('command')=='freeze' and (self.snapshot_key is None or key==self.snapshot_key)

    def poll(self):
        packet=self.host.poll_image_control()
        if not self.valid:
            if self.awaiting_release:self.awaiting_release=not self.host.resume_image_submissions()
            return None
        if self.processing and self._now()>=self.deadline:self.cancel('deadline');return None
        if packet is None or not self.accepts(packet):return None
        state=packet.get('state')
        if state=='frozen':
            self.snapshot_key=dict(packet['snapshot_key']);self.snapshot_key['source_frames']=tuple(self.snapshot_key['source_frames'])
            if len(self.snapshot_key['source_frames'])<20:
                self.phase='ready';self.reason='base_window_incomplete';packet['reason']=self.reason
            else:
                self.phase='enhance';self.reason='computing'
                remaining=min(60.,max(0.,self.deadline-self._now()))
                try:
                    extra={'options':self.options} if self.command=='surface' else {}
                    accepted=remaining>0. and self.host.request_image_control(self.command,self.request_id,self.source_key,self.snapshot_key,remaining,**extra)
                except Exception:accepted=False
                if not accepted:self.phase='failed';self.reason='enhance_unavailable';packet['reason']=self.reason
        elif state in ('accepted','baseline_retained'):
            self.phase='ready';self.reason=packet.get('reason',state)
        elif state=='failed':self.phase='failed';self.reason=packet.get('reason','worker_failed')
        elif state=='cancelled':self.cancel(packet.get('reason','cancelled'));return None
        return packet

    def cancel(self,reason):
        self.valid=False;self.phase='cancelled';self.reason=reason
        if getattr(self.host,'closed',False) or getattr(self.host,'failed',False):self.awaiting_release=False;return
        if self.request_id is not None:self.host.cancel_image_control(self.request_id)
        self.awaiting_release=not self.host.resume_image_submissions()
