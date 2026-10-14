"""Live Airy rail prototype. Native +Z points along the vehicle, no PCAP input."""
import math
import random
import time
import threading
import csv
import os
import sys
import datetime
import json
import array
from collections import deque

from rail_centerline import (fit_centerline, measurement_text, _dot, _cross, _unit,
                             _heading_basis, _quantile, _track_point, VtkRailOverlay)
from rail_evidence import RailEvidenceOverlay
from rail_process import ProcessWorker


class LatestWorker(object):
    """One running job plus one replaceable pending frame; no VTK off-thread."""
    def __init__(self, process=None):
        self.process = process or analyze_live
        self.condition = threading.Condition()
        self.pending = self.result = None
        self.running = None
        self.events = deque(maxlen=256)
        self.lost_events = 0
        self.closed = False
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.thread.start()

    def submit(self, job):
        with self.condition:
            if not self.closed:
                if self.pending is not None:
                    self._record('pending_replaced', **self._job_info(self.pending))
                self.pending = job
                self._record('submitted', **self._job_info(job))
                self.condition.notify()

    @staticmethod
    def _job_info(job):
        return dict(sequence=job[0], received=job[1],
                    point_count=len(job[2]) if hasattr(job[2], '__len__') else None)

    def _record(self, event, **fields):
        # Caller holds condition. No disk I/O or VTK on the worker thread.
        if len(self.events) == self.events.maxlen:
            self.lost_events += 1
        fields.update(event=event, time=time.time())
        self.events.append(fields)

    def diagnostics(self):
        with self.condition:
            state = dict(running=dict(self.running) if self.running else None,
                         pending=self._job_info(self.pending) if self.pending else None,
                         lost_events=self.lost_events, alive=self.thread.is_alive())
            events = list(self.events)
            self.events.clear()
        return state, events

    def poll(self):
        with self.condition:
            result, self.result = self.result, None
            return result

    def close(self):
        with self.condition:
            self.closed = True
            self.pending = None
            self.condition.notify()

    def cancel_pending(self):
        with self.condition:
            if self.pending is not None:
                self._record('pending_cancelled', **self._job_info(self.pending))
            if self.result is not None:
                self._record('result_cancelled', sequence=self.result[0])
            self.pending = self.result = None

    def _run(self):
        while True:
            with self.condition:
                while self.pending is None and not self.closed:
                    self.condition.wait()
                if self.closed:
                    return
                sequence, received, points = self.pending
                start = time.time()
                self.running = self._job_info(self.pending)
                self.running['started'] = start
                self._record('started', **self.running)
                self.pending = None
            error = ''
            try:
                result = self.process(points)
            except Exception as exc:
                result, error = None, str(exc)
            with self.condition:
                finished = time.time()
                elapsed = (finished - start) * 1000.
                info = dict(self.running)
                info.update(finished=finished, compute_ms=elapsed,
                            queue_ms=(start - received) * 1000., error=error)
                self._record('completed', **info)
                if self.result is not None:
                    self._record('result_replaced', sequence=self.result[0])
                self.result = (sequence, received, result, elapsed, error)
                self.running = None


def cloud_key(polydata, timestep):
    """Prefer packet-derived point timestamps over render/update counters."""
    if polydata is None or polydata.GetNumberOfPoints() == 0:
        return None
    data = polydata.GetPointData()
    for i in range(data.GetNumberOfArrays()):
        name = data.GetArrayName(i) or ''
        if 'time' in name.lower():
            array = data.GetArray(i)
            n = array.GetNumberOfTuples()
            if n:
                first, last = array.GetTuple1(0), array.GetTuple1(n - 1)
                if all(not math.isnan(v) and not math.isinf(v) for v in (first, last)):
                    return (name, first, last)
    return timestep


def measurement_points(vtk_points, max_points=30000):
    """Copy the measurement ROI first; periodic strides can erase scan rings."""
    return _measurement_roi((vtk_points.GetPoint(i) for i in range(vtk_points.GetNumberOfPoints())),
                            max_points)


def _measurement_roi(source, max_points=30000, legacy=False):
    points = []
    for p in source:
        # Avoid an extra Python function call for each raw return.
        if .3 < p[2] < 6.2 and p[0]*p[0] + p[1]*p[1] + p[2]*p[2] < 49:
            points.append(p)
    if len(points) > max_points:
        rng = random.Random(17)
        if legacy and sys.version_info[0] >= 3:
            # Match the embedded Python 2 sample(), including selection order.
            # Python 3's randbelow-based implementation selects other returns.
            n, k = len(points), max_points
            setsize = 21 + (4 ** int(math.ceil(math.log(k * 3, 4))) if k > 5 else 0)
            if n <= setsize:
                pool, selected = list(points), []
                for i in range(k):
                    j = int(rng.random() * (n - i))
                    selected.append(pool[j])
                    pool[j] = pool[n - i - 1]
            else:
                used, selected = set(), []
                while len(selected) < k:
                    j = int(rng.random() * n)
                    if j not in used:
                        used.add(j)
                        selected.append(points[j])
            points = selected
        else:
            points = rng.sample(points, max_points)
    return points


def prepare_measurement(vtk_points):
    """Own a bulk XYZ copy on the UI thread; never pass VTK to the worker."""
    try:
        data = vtk_points.GetData()
        dtype = {'float': 'f', 'double': 'd'}[data.GetDataTypeAsString()]
        if data.GetNumberOfComponents() != 3:
            raise ValueError('XYZ must have three components')
        raw = bytearray(buffer(data) if sys.version_info[0] == 2 else memoryview(data))
        return dict(format='vtk_xyz_v1', dtype=dtype, raw=raw,
                    count=vtk_points.GetNumberOfPoints(), sampler_major=sys.version_info[0])
    except (AttributeError, TypeError, KeyError, BufferError):
        # Older/custom VTK wrappers may not expose a numeric buffer.
        return measurement_points(vtk_points)


def unpack_measurement(payload):
    if not isinstance(payload, dict):
        return payload
    if payload.get('format') != 'vtk_xyz_v1' or payload.get('dtype') not in ('f', 'd'):
        raise ValueError('Unsupported XYZ buffer format')
    values = array.array(payload['dtype'])
    raw, count = payload['raw'], payload['count']
    if count < 0 or len(raw) != count * 3 * values.itemsize:
        raise ValueError('XYZ byte length does not match point count')
    if sys.version_info[0] == 2:
        values.fromstring(str(raw))
    else:
        values.frombytes(raw)
    points = ((values[i], values[i+1], values[i+2]) for i in range(0, len(values), 3))
    return _measurement_roi(points, legacy=payload.get('sampler_major') == 2)


class LiveRailOverlay(VtkRailOverlay):
    def __init__(self, smp, vtk, view, status_label, history_dir, interval=2):
        VtkRailOverlay.__init__(self, smp, vtk, view, status_label, None)
        self.evidence = RailEvidenceOverlay(smp, vtk, view)
        self.labels[1].GetTextProperty().SetColor(1., .65, .05)
        self.labels[2].GetTextProperty().SetColor(.1, 1., 1.)
        self.gate = FrameGate(interval=interval)
        self.worker = ProcessWorker()
        self.sequence = 0
        self.accept_after = float('-inf')
        self.result_time = None
        self.valid_result_time = None
        self.holding = False
        self.hold_reason = ''
        self.hold_failed_sequence = None
        self.last_result_sequence = None
        self.last_drop = ''
        self.last_heartbeat = float('-inf')
        self.diagnostic_log = None
        self.recent = []
        try:
            self.message = vtk.vtkTextActor()
        except AttributeError:
            from vtkRenderingCorePython import vtkTextActor
            self.message = vtkTextActor()
        self.message.GetTextProperty().SetColor(1., 1., 1.)
        self.message.GetTextProperty().SetFontSize(15)
        self.message.SetDisplayPosition(16, 16)
        self.message.SetInput('LIVE | Waiting for new frames (measure every %d new frames)' % self.gate.interval)
        self.renderer.AddActor2D(self.message)
        self.camera_observer = None
        if hasattr(self.renderer, 'GetActiveCamera'):
            self.camera = self.renderer.GetActiveCamera()
            self.camera_observer = self.camera.AddObserver('ModifiedEvent', self._camera_changed)
        self.log = None
        self.log_error = ''
        try:
            if not os.path.isdir(history_dir):
                os.makedirs(history_dir)
            name = 'live-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.csv'
            self.log_path = os.path.join(history_dir, name)
            self.log = open(self.log_path, 'wb') if sys.version_info[0] == 2 else open(self.log_path, 'w', newline='')
            self.writer = csv.writer(self.log)
            self.writer.writerow(['received_local_unix_s', 'observed_frame', 'status',
                                  'gauge_mm', 'left_mm', 'right_mm', 'delta_1435_mm', 'compute_ms',
                                  'fit_residual_p95_mm', 'paired_sections', 'measurement_kind',
                                  'all_section_residual_p95_mm', 'section_outlier_fraction',
                                  'reason_code', 'reason_details', 'left_support_points', 'right_support_points'])
            self.log.flush()
            self.diagnostic_path = self.log_path[:-4] + '-diagnostics.jsonl'
            self.diagnostic_log = open(self.diagnostic_path, 'w')
        except (IOError, OSError) as exc:
            self.log_error = ' | History save error: ' + str(exc)

    def _camera_changed(self, unused=None, event=None):
        if self.last_result is not None:
            self._show_labels(self.last_result)

    def display(self, result):
        if result is None:
            self.last_result_sequence = None
        VtkRailOverlay.display(self, result)
        self.evidence.display(result)

    def _show_labels(self, result):
        VtkRailOverlay._show_labels(self, result)
        self.labels[0].GetTextProperty().SetColor(*((1., .7, .1) if self.holding else (1., 1., 1.)))
        if self.holding:
            self.labels[0].SetInput('HOLD | ' + measurement_text(result)['gauge'])
        center = self._display_point(result['center_point'])
        for index, side in ((1, 'left'), (2, 'right')):
            point = self._display_point(result[side + '_point'])
            dx, dy = point[0] - center[0], point[1] - center[1]
            if abs(dx) >= abs(dy):
                offset = (12 if dx >= 0 else -125, -10)
            else:
                offset = (-55, 16 if dy >= 0 else -36)
            self.labels[index].SetDisplayPosition(int(point[0] + offset[0]),
                                                   int(point[1] + offset[1]))

    def invalidate(self, message):
        self.display(None)
        self.valid_result_time = None
        self.holding = False
        self.hold_reason = ''
        history = ' | History: ' + ' / '.join(self.recent) if self.recent else ''
        self.message.SetInput('LIVE | ' + message + history + self.log_error)

    def pause(self, now=None):
        now = time.time() if now is None else now
        self.accept_after = now
        self.result_time = None
        self.last_drop = ''
        self.worker.cancel_pending()
        self.gate = FrameGate(interval=self.gate.interval, timeout=self.gate.timeout)
        self.invalidate('Paused')

    def _diagnostic(self, event, **fields):
        if self.diagnostic_log is None:
            return
        fields.setdefault('time', time.time())
        fields['event'] = event
        try:
            self.diagnostic_log.write(json.dumps(fields, ensure_ascii=True) + '\n')
            self.diagnostic_log.flush()
        except (IOError, OSError) as exc:
            self.log_error = ' | Diagnostic save error: ' + str(exc)

    def _progress(self, now):
        state, events = self.worker.diagnostics()
        for event in events:
            self._diagnostic(**event)
        if now - self.last_heartbeat >= 1.:
            self.last_heartbeat = now
            thread = getattr(self.worker, 'thread', None)
            frame = sys._current_frames().get(thread.ident) if thread else None
            location = state.pop('worker_location', '')
            if frame is not None:
                location = '%s:%d:%s' % (os.path.basename(frame.f_code.co_filename),
                                         frame.f_lineno, frame.f_code.co_name)
            del frame
            self._diagnostic('heartbeat', time=now, observed_frame=self.sequence,
                             worker_location=location, **state)
        running, pending = state['running'], state['pending']
        if not state['alive']:
            return 'Worker stopped'
        if running:
            text = 'Computing #%d | elapsed %.2f s | input age %.2f s' % (
                running['sequence'], now - running['started'], now - running['received'])
            if pending:
                text += ' | queued #%d' % pending['sequence']
            return text
        if pending:
            return 'Queued #%d | waiting %.2f s' % (pending['sequence'], now - pending['received'])
        return 'Received %d new frames | next calculation every %d new frames' % (self.sequence, self.gate.interval)

    def tick(self, polydata, timestep, now=None):
        live_clock = now is None
        now = time.time() if now is None else now
        key = cloud_key(polydata, timestep)
        previous_key = self.gate.key
        due = self.gate.observe(key, now)
        if key is not None and key != previous_key:
            self.sequence += 1
        if due:
            vtk_points = polydata.GetPoints()
            points = prepare_measurement(vtk_points)
            self.worker.submit((self.sequence, now, points))
        completed = self.worker.poll()
        # Point copying / waiting for the worker lock can consume real time.
        # Judge freshness when the result is received, not at tick entry.
        if live_clock:
            now = time.time()
        if completed is not None:
            sequence, received, analysis, elapsed, error = completed
            if received > self.accept_after and now - received < self.gate.timeout:
                self.last_drop = ''
                self._diagnostic('accepted', sequence=sequence, received=received,
                                 age_ms=(now - received) * 1000., compute_ms=elapsed,
                                 code='processing_error' if error else analysis['code'])
                result = analysis['measurement'] if analysis else None
                self.result_time = received
                status = 'Fit error: ' + error if error else analysis['message']
                if result:
                    self.holding = False
                    self.display(result)
                    self.last_result_sequence = sequence
                    self.valid_result_time = received
                    self.hold_reason = ''
                elif self.last_result is not None and self.valid_result_time is not None:
                    self.holding = True
                    self.hold_reason = status
                    self.hold_failed_sequence = sequence
                    self._show_labels(self.last_result)
                else:
                    self.display(None)
                    self.valid_result_time = None
                    self.holding = False
                code = 'processing_error' if error else analysis['code']
                details = error if error else analysis['message']
                values = [result[k] * 1000. for k in ('gauge', 'left_distance', 'right_distance')] if result else ['', '', '']
                delta = values[0] - 1435. if result else ''
                if self.log:
                    try:
                        quality = ([result['fit_residual_p95_m'] * 1000.,
                                    result['paired_sections'], result['measurement_kind'],
                                    result['all_section_residual_p95_m'] * 1000.,
                                    result['section_outlier_fraction']]
                                   if result else ['', '', '', '', ''])
                        support = [len(result[k + '_support_points']) for k in ('left', 'right')] if result else ['', '']
                        self.writer.writerow([received, sequence, status] + values + [delta, elapsed] + quality +
                                             [code, details] + support)
                        self.log.flush()
                    except (IOError, OSError) as exc:
                        self.log_error = ' | History save error: ' + str(exc)
                if result:
                    self.recent.append('#%d %.1f mm' % (sequence, values[0]))
                    self.recent = self.recent[-3:]
                quality_text = (' | Fit P95 %.1f mm | %d sections | dropped %.0f%% (not accuracy)' %
                                (result['fit_residual_p95_m'] * 1000., result['paired_sections'],
                                 result['section_outlier_fraction'] * 100.)
                                if result else '')
                legend = ('\nL orange / R cyan: contributing returns | white: fitted intersections'
                          '\nVisible upper 25 mm band below estimated P95 top; not certified gauge'
                          '\nPale rectangle: measurement section | HOLD retains last measured frame'
                          if result else '')
                self.message.SetInput('LIVE | %s | measured frame %d | %.0f ms%s\nHistory: %s%s%s' %
                                      (status, sequence, elapsed, quality_text,
                                       ' / '.join(self.recent), self.log_error, legend))
            else:
                reason = 'invalidated' if received <= self.accept_after else 'timeout'
                self._diagnostic('discarded', sequence=sequence, received=received,
                                 reason=reason, age_ms=(now - received) * 1000.,
                                 compute_ms=elapsed, error=error)
                self.last_drop = ('Result expired #%d | age %.2f s >= %.2f s | compute %.0f ms' %
                                  (sequence, now - received, self.gate.timeout, elapsed)
                                  if reason == 'timeout' else 'Result discarded after pause / disconnect')
        progress = self._progress(now)
        if self.gate.stale(now):
            self.accept_after = now
            self.invalidate('Waiting for new frames / stream stopped')
        elif self.last_result is not None and (self.holding or
              (self.result_time is not None and now - self.result_time >= self.gate.timeout)):
            if not self.holding:
                self.holding = True
                self.hold_reason = 'No newer completed measurement'
                self.hold_failed_sequence = self.last_result_sequence
                self._show_labels(self.last_result)
            self.message.SetInput(
                'LIVE | HOLD last valid frame #%d (age %.2f s; no new measurement)'
                ' | latest check #%d: %s\nHistory: %s%s' %
                (self.last_result_sequence, now - self.valid_result_time,
                 self.hold_failed_sequence, self.hold_reason,
                 ' / '.join(self.recent), self.log_error))
        elif self.result_time is not None and now - self.result_time >= self.gate.timeout:
            self.invalidate('Measurement expired | ' + progress +
                            ('\n' + self.last_drop if self.last_drop else ''))
        elif self.result_time is None:
            self.invalidate(progress + ('\n' + self.last_drop if self.last_drop else ''))

    def close(self):
        self.worker.close()
        self._progress(time.time())
        self._diagnostic('closed')
        if self.diagnostic_log:
            self.diagnostic_log.close()
        if self.log:
            self.log.close()
        if self.camera_observer is not None:
            self.camera.RemoveObserver(self.camera_observer)
        self.renderer.RemoveActor2D(self.message)
        self.evidence.close()
        VtkRailOverlay.close(self)


class FrameGate(object):
    def __init__(self, interval=2, timeout=1.5):
        if interval < 1 or int(interval) != interval:
            raise ValueError('Frame interval must be a positive integer')
        self.interval, self.timeout = interval, timeout
        self.key = None
        self.count = 0
        self.last_seen = None

    def stale(self, now):
        return self.last_seen is None or now - self.last_seen >= self.timeout

    def observe(self, key, now):
        if key is None or key == self.key:
            return False
        if self.stale(now):
            self.count = 0
        self.key, self.last_seen = key, now
        self.count += 1
        return self.count % self.interval == 0


def _ground_basis(points):
    """Estimate a broad local ground plane, oriented towards the sensor."""
    sample = random.Random(11).sample(points, 1600) if len(points) > 1600 else points
    rng = random.Random(7)
    best = None
    best_count = 0
    for unused in range(80):
        a, b, c = rng.sample(sample, 3)
        normal = _unit(_cross(tuple(b[i] - a[i] for i in range(3)),
                              tuple(c[i] - a[i] for i in range(3))))
        if normal is None:
            continue
        offset = _dot(a, normal)
        # Sideways mounting: the ground normal is mostly transverse to +Z.
        if abs(normal[2]) > .5 or not .12 < abs(offset) < 1.5:
            continue
        support = [p for p in sample if abs(_dot(p, normal) - offset) < .025]
        count = len(support)
        if count > best_count:
            best, best_count = (support, normal), count
    if best is None or best_count < max(120, len(sample) * .2):
        return None
    support, normal = best
    center = tuple(sum(p[i] for p in support) / len(support) for i in range(3))
    k = max(range(3), key=lambda i: abs(normal[i]))
    i, j = [a for a in range(3) if a != k]
    centered = [tuple(p[a] - center[a] for a in range(3)) for p in support]
    xx = sum(p[i] * p[i] for p in centered)
    yy = sum(p[j] * p[j] for p in centered)
    xy = sum(p[i] * p[j] for p in centered)
    xz = sum(p[i] * p[k] for p in centered)
    yz = sum(p[j] * p[k] for p in centered)
    det = xx * yy - xy * xy
    if det < .001 * len(support) ** 2:
        return None
    normal = [0., 0., 0.]
    normal[i], normal[j], normal[k] = -(xz * yy - yz * xy) / det, -(yz * xx - xz * xy) / det, 1.
    up = _unit(normal)
    if _dot(center, up) > 0:
        up = tuple(-v for v in up)
    if abs(up[2]) > .5:
        return None
    forward = _unit(tuple(float(a == 2) - up[2] * up[a] for a in range(3)))
    return (forward, _cross(up, forward), up), _dot(center, up)


def _transverse_measurement(result, basis):
    """Intersect both fitted rails with a perpendicular in their mean plane."""
    forward, lateral, up = basis
    end0, end1 = result['endpoints']
    direction = _unit(tuple(end1[i] - end0[i] for i in range(3)))
    center = result['center_point']
    s, y = _dot(center, forward), _dot(center, lateral)
    # Use both observed rail heights; the ground plane need not equal the
    # rail plane (cant, uneven ballast). This includes measured cross-level.
    heights = result['rail_heights']
    width = ((result['rail_lines'][1][0] - result['rail_lines'][0][0]) * s +
             result['rail_lines'][1][1] - result['rail_lines'][0][1])
    rise = (heights[1][0] - heights[0][0]) * s + heights[1][1] - heights[0][1]
    across = tuple(width * lateral[i] + rise * up[i] for i in range(3))
    transverse = _unit(tuple(across[i] - _dot(across, direction) * direction[i]
                             for i in range(3)))
    if transverse is None:
        return None
    ds, dy = _dot(transverse, forward), _dot(transverse, lateral)
    distances = []
    points = []
    for slope, offset in result['rail_lines']:
        denominator = dy - slope * ds
        if abs(denominator) < 1e-6:
            return None
        t = (slope * s + offset - y) / denominator
        distances.append(t)
        points.append(tuple(center[i] + t * transverse[i] for i in range(3)))
    result['left_point'], result['right_point'] = points
    result['gauge'] = abs(distances[1] - distances[0])
    result['left_distance'], result['right_distance'] = map(abs, distances)
    return result


def _reject(diagnostics, code, message, **details):
    if diagnostics is not None:
        diagnostics.update(details)
        diagnostics['code'], diagnostics['message'] = code, message
    return None


def _validated_section(result, basis, diagnostics=None):
    """Compute the observed section; report test-stage quality warnings."""
    rails = result['rail_fits']
    errors = []
    all_errors = []
    outliers = []
    warnings = []
    for side, rail in zip(('L', 'R'), rails):
        samples = rail['all_samples']
        if len(rail['samples']) < .75 * len(samples):
            warnings.append('%s rail: %d/%d sections retained; below 75%%' %
                            (side, len(rail['samples']), len(samples)))
        slope, offset = rail['line']
        def residual(sample):
            s, y = sample
            return abs(y - slope * s - offset) / math.sqrt(1 + slope * slope)
        all_errors.append(_quantile([residual(p) for p in samples], .95))
        errors.append(_quantile([residual(p) for p in rail['samples']], .95))
        outliers.append(1. - len(rail['samples']) / float(len(samples)))
        # Isolated contaminated sections may be trimmed. A continuous bad
        # run must not be hidden by reporting only the surviving residuals.
        run = 0
        for p in sorted(samples):
            run = run + 1 if p not in rail['samples'] else 0
            if run >= 3:
                warnings.append('%s rail: 3 consecutive rejected sections' % side)
                break
    if max(errors) > .015:
        side = ('L', 'R')[errors.index(max(errors))]
        warnings.append('%s rail residual P95 %.1f > 15.0 mm' %
                        (side, max(errors) * 1000.))
    left, right = rails
    pairs = []
    for sample in left['samples']:
        other = min(right['samples'], key=lambda p: abs(p[0] - sample[0]))
        if abs(sample[0] - other[0]) <= .12:
            pairs.append((sample[0] + other[0]) * .5)
    if len(pairs) < 8:
        warnings.append('Only %d paired sections; below 8' % len(pairs))
    middle = _dot(result['center_point'], basis[0])
    station = min(pairs, key=lambda s: abs(s - middle)) if pairs else middle
    y = sum(m * station + b for m, b in result['rail_lines']) * .5
    h = sum(m * station + b for m, b in result['rail_heights']) * .5
    result['center_point'] = _track_point(station, y, h, basis)
    result = _transverse_measurement(result, basis)
    if result is None:
        return _reject(diagnostics, 'degenerate_section', 'Cannot construct perpendicular measurement section')
    # Intersections move longitudinally with yaw; verify actual local support.
    for side, rail, point in zip(('L', 'R'), rails, (result['left_point'], result['right_point'])):
        s = _dot(point, basis[0])
        gap = min(abs(p[0] - s) for p in rail['samples'])
        if gap > .18:
            warnings.append('%s intersection support gap %.0f > 180 mm' %
                            (side, gap * 1000.))
    result['fit_residual_p95_m'] = max(errors)
    result['all_section_residual_p95_m'] = max(all_errors)
    result['section_outlier_fraction'] = max(outliers)
    result['paired_sections'] = len(pairs)
    result['measurement_kind'] = rails[0].get('measurement_kind', 'observed_rail_features')
    result['left_support_points'], result['right_support_points'] = [r['support_points'] for r in rails]
    result['quality_warnings'] = warnings
    if diagnostics is not None:
        diagnostics['quality_warnings'] = warnings
    return result


def fit_live(points, diagnostics=None):
    diagnostics = {} if diagnostics is None else diagnostics
    # The bounded squared radius also rejects NaN/Inf coordinates. Keep this
    # hot path scalar: two nested generators per return dominated worker time.
    points = [(float(p[0]), float(p[1]), float(p[2])) for p in points
              if .3 < p[2] < 6.2 and p[0]*p[0] + p[1]*p[1] + p[2]*p[2] < 49]
    if len(points) < 250:
        return _reject(diagnostics, 'too_few_points',
                       'Too few valid ROI points: %d; need 250' % len(points), roi_points=len(points))
    ground = _ground_basis(points)
    if ground is None:
        return _reject(diagnostics, 'ground_not_found', 'No stable ground plane found')
    basis, floor = ground
    # Keep rail heads above the estimated ground, not arbitrary scene planes.
    if not .12 < -floor < 1.5:
        return _reject(diagnostics, 'ground_height_out_of_range',
                       'Ground offset %.3f m outside mounting range' % -floor)
    candidates = [p for p in points if floor + .035 < _dot(p, basis[2]) < floor + .35]
    if len(candidates) < 100:
        return _reject(diagnostics, 'too_few_rail_points',
                       'Too few points above ground: %d; need 100' % len(candidates))
    result = fit_centerline(candidates, basis, diagnostics, live=True)
    if result is None:
        count = diagnostics.get('max_fitted_rails', 0)
        if count < 2:
            return _reject(diagnostics, 'insufficient_rails',
                           'Fewer than 2 usable rail candidates (max %d)' % count)
        return _reject(diagnostics, 'no_valid_pair',
                       'Candidates found; no pair satisfies direction, height and overlap checks')
    if result['fit_mode'] != 'dense':
        return _reject(diagnostics, 'sparse_rails', 'Rail candidates lack longitudinal support')
    # The vehicle is between the rails; reject two ridges on the same side.
    if result['rail_lines'][0][1] * result['rail_lines'][1][1] >= 0:
        return _reject(diagnostics, 'same_side_pair', 'Both rail candidates are on the same side of the sensor')
    fitted_basis = _heading_basis(basis, result['heading_correction_deg'])
    return _validated_section(result, fitted_basis, diagnostics)


def analyze_live(points):
    """Keep diagnostics with their frame; no shared/global failure state."""
    diagnostics = {}
    result = fit_live(unpack_measurement(points), diagnostics)
    warnings = diagnostics.get('quality_warnings', [])
    return {'measurement': result,
            'code': ('quality_warning' if warnings else 'ok') if result is not None else diagnostics['code'],
            'message': ('TEST estimate | ' + '; '.join(warnings) if warnings else 'OK')
                       if result is not None else diagnostics['message'],
            'diagnostics': diagnostics}
