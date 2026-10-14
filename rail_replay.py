"""Wall-clock paced decoded-frame input; never waits for a fitting result."""
import bisect
import math


def lock_simulated_source(logic, notify):
    """This dedicated window must not mix cached frames with a newly opened source."""
    def blocked(*args, **kwargs):
        notify('SIMULATION ONLY | Restart RSView to open another file or connect a sensor')
        return False
    logic.openPCAP = logic.openSensor = blocked
    for name in ('actionClose', 'actionChoose_Calibration_File',
                 'actionNetworkConfigSensor', 'actionNetworkConfigPcap',
                 'actionRecord', 'actionSeek_Forward', 'actionSeek_Backward',
                 'actionGo_To_Start', 'actionGo_To_End'):
        if name in logic.app.actions:
            logic.app.actions[name].setEnabled(False)


class ReplayClock(object):
    def __init__(self, timestamps, start, loops=0):
        if (len(timestamps) < 2 or
                any(math.isnan(t) or math.isinf(t) for t in timestamps) or
                any(b <= a for a, b in zip(timestamps, timestamps[1:]))):
            raise ValueError('Need increasing, finite frame timestamps')
        steps = sorted(b - a for a, b in zip(timestamps, timestamps[1:]))
        self.offsets = [t - timestamps[0] for t in timestamps]
        self.period = self.offsets[-1] + steps[len(steps) // 2]
        self.start, self.loops = start, loops
        self.previous = -1
        self.skipped = 0
        self.finished = False

    def update(self, now):
        if self.finished:
            return None
        elapsed = max(0., now - self.start)
        cycle = int(elapsed / self.period)
        if self.loops and cycle >= self.loops:
            self.skipped += max(0, self.loops * len(self.offsets) - self.previous - 1)
            self.finished = True
            return None
        index = bisect.bisect_right(self.offsets, elapsed - cycle * self.period + 1e-9) - 1
        sequence = cycle * len(self.offsets) + index
        if sequence <= self.previous:
            return None
        self.skipped += max(0, sequence - self.previous - 1)
        self.previous = sequence
        return cycle, index
