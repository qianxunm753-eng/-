import math
import os
import struct


_REFERENCE_GAUGE = 1.435


def is_target_file(filename):
    return os.path.basename(filename).lower() == '22222.pcap'


def measurement_text(result, reference=_REFERENCE_GAUGE):
    """Format display labels after fitting; reference never enters fitting."""
    delta = (result['gauge'] - reference) * 1000.0
    return {
        'gauge': 'Gauge %.1f mm' % (result['gauge'] * 1000.0),
        'left': 'L %.1f mm' % (result['left_distance'] * 1000.0),
        'right': 'R %.1f mm' % (result['right_distance'] * 1000.0),
        'reference': 'Ref %.1f | Delta %+.1f mm' %
                     (reference * 1000.0, delta)
    }


class CenterlineProcessor(object):
    def __init__(self, pose_index, max_points=60000):
        self.pose_index = pose_index
        self.max_points = max_points

    def process(self, polydata, frame):
        basis = self.pose_index.basis(frame)
        vtk_points = polydata.GetPoints() if polydata is not None else None
        if basis is None or vtk_points is None:
            return None
        count = vtk_points.GetNumberOfPoints()
        if count < 100:
            return None
        stride = max(1, int(math.ceil(count / float(self.max_points))))
        points = [vtk_points.GetPoint(i) for i in range(0, count, stride)]
        return fit_centerline(points, basis)


class VtkRailOverlay(object):
    def __init__(self, smp, vtk, view, status_label, pose_index):
        self.smp = smp
        self.vtk = vtk
        self.view = view
        self.status_label = status_label
        self.processor = CenterlineProcessor(pose_index)
        active = smp.GetActiveSource()
        self.source, self.representation = self._line(
            [1.0, 0.0, 1.0], 8.0)
        self.measurement_source, self.measurement_representation = self._line(
            [0.1, 1.0, 0.2], 5.0)
        self.renderer = view.GetRenderer()
        self.labels = []
        for unused in range(4):
            try:
                actor = vtk.vtkTextActor()
            except AttributeError:
                from vtkRenderingCorePython import vtkTextActor
                actor = vtkTextActor()
            prop = actor.GetTextProperty()
            prop.SetColor(1.0, 1.0, 1.0)
            prop.SetFontSize(18)
            prop.SetBold(1)
            actor.SetVisibility(0)
            self.renderer.AddActor2D(actor)
            self.labels.append(actor)
        self.last_processed_frame = None
        self.last_result = None
        status_label.setText('')
        smp.SetActiveSource(active)

    def _line(self, color, width):
        source = self.smp.Line()
        source.Point1 = [0.0, 0.0, 0.0]
        source.Point2 = [0.0, 0.0, 0.0]
        source.UpdatePipeline()
        representation = self.smp.Show(source, self.view)
        representation.ColorArrayName = ''
        representation.DiffuseColor = color
        representation.AmbientColor = color
        representation.LineWidth = width
        representation.Visibility = 0
        return source, representation

    def _display_point(self, point):
        self.renderer.SetWorldPoint(point[0], point[1], point[2], 1.0)
        self.renderer.WorldToDisplay()
        return self.renderer.GetDisplayPoint()

    def _hide(self):
        self.representation.Visibility = 0
        self.measurement_representation.Visibility = 0
        for actor in self.labels:
            actor.SetVisibility(0)

    def _show_labels(self, result):
        text = measurement_text(result)
        left = result['left_point']
        center = result['center_point']
        right = result['right_point']
        anchors = (center,
                   tuple((left[i] + center[i]) * 0.5 for i in range(3)),
                   tuple((center[i] + right[i]) * 0.5 for i in range(3)),
                   center)
        offsets = ((-65, 32), (-110, -18), (24, -18), (-95, -48))
        values = (text['gauge'], text['left'], text['right'],
                  text['reference'])
        for actor, value, anchor, offset in zip(
                self.labels, values, anchors, offsets):
            display = self._display_point(anchor)
            actor.SetInput(value)
            actor.SetDisplayPosition(int(display[0] + offset[0]),
                                     int(display[1] + offset[1]))
            actor.SetVisibility(1)

    def update(self, polydata, frame):
        frame = int(frame)
        if (self.last_processed_frame is not None and
                abs(frame - self.last_processed_frame) < 2):
            return self.last_result
        result = self.processor.process(polydata, frame)
        self.last_processed_frame = frame
        self.display(result)
        self.smp.Render(self.view)
        return result

    def display(self, result):
        """Update VTK actors on the UI thread, also used by live processing."""
        self.last_result = result
        if result is None:
            self._hide()
        else:
            self.source.Point1 = list(result['endpoints'][0])
            self.source.Point2 = list(result['endpoints'][1])
            self.source.UpdatePipeline()
            self.measurement_source.Point1 = list(result['left_point'])
            self.measurement_source.Point2 = list(result['right_point'])
            self.measurement_source.UpdatePipeline()
            self.representation.Visibility = 1
            self.measurement_representation.Visibility = 1
            self._show_labels(result)

    def close(self):
        if self.source is not None:
            self.smp.Delete(self.source)
            self.source = None
        if self.measurement_source is not None:
            self.smp.Delete(self.measurement_source)
            self.measurement_source = None
        for actor in self.labels:
            self.renderer.RemoveActor2D(actor)
        self.labels = []


def _byte_value(value):
    return value if isinstance(value, int) else ord(value)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _unit(v):
    length = math.sqrt(_dot(v, v))
    if length < 1e-9:
        return None
    return tuple(x / length for x in v)


def _rotate_quaternion(q, v):
    x, y, z, w = q
    return (
        (1 - 2 * (y * y + z * z)) * v[0] +
        2 * (x * y - z * w) * v[1] + 2 * (x * z + y * w) * v[2],
        2 * (x * y + z * w) * v[0] +
        (1 - 2 * (x * x + z * z)) * v[1] + 2 * (y * z - x * w) * v[2],
        2 * (x * z - y * w) * v[0] + 2 * (y * z + x * w) * v[1] +
        (1 - 2 * (x * x + y * y)) * v[2])


class PcapPoseIndex(object):
    def __init__(self, frames, imu, quaternion):
        self.frames = frames
        self.imu = imu
        self.quaternion = quaternion

    @classmethod
    def from_file(cls, filename):
        frames = {}
        imu = []
        quaternion = None
        frame = 0
        previous_azimuth = None
        for timestamp, port, payload in cls._udp_packets(filename):
            if port == 7788 and len(payload) == 1248 and quaternion is None:
                quaternion = struct.unpack('>4f', payload[1092:1108])
                quaternion = _unit(quaternion)
            elif port == 6688 and len(payload) == 51 and payload[:4] == b'\xaa\x55\x5a\x05':
                unit = (1 << (_byte_value(payload[44]) + 1)) / 32768.0
                raw = struct.unpack('>iii', payload[14:26])
                imu.append((timestamp, tuple(x * unit for x in raw)))
            elif port == 6699 and len(payload) == 1248 and payload[:4] == b'\x55\xaa\x05\x5a':
                azimuth = struct.unpack('>H', payload[44:46])[0]
                if previous_azimuth is not None and azimuth < previous_azimuth:
                    frame += 1
                previous_azimuth = azimuth
                if frame not in frames:
                    frames[frame] = [timestamp, timestamp]
                frames[frame][1] = timestamp
        if quaternion is None:
            raise ValueError('DIFOP quaternion not found')
        return cls(frames, imu, quaternion)

    @staticmethod
    def _udp_packets(filename):
        with open(filename, 'rb') as stream:
            header = stream.read(24)
            formats = {b'\xd4\xc3\xb2\xa1': ('<', 1e-6),
                       b'\xa1\xb2\xc3\xd4': ('>', 1e-6),
                       b'\x4d\x3c\xb2\xa1': ('<', 1e-9),
                       b'\xa1\xb2\x3c\x4d': ('>', 1e-9)}
            if header[:4] not in formats:
                raise ValueError('unsupported PCAP format')
            endian, scale = formats[header[:4]]
            while True:
                packet_header = stream.read(16)
                if len(packet_header) < 16:
                    return
                sec, subsec, captured, unused = struct.unpack(
                    endian + 'IIII', packet_header)
                packet = stream.read(captured)
                if len(packet) < captured or len(packet) < 42:
                    continue
                offset = 14
                ether_type = struct.unpack('>H', packet[12:14])[0]
                if ether_type == 0x8100:
                    ether_type = struct.unpack('>H', packet[16:18])[0]
                    offset = 18
                if ether_type != 0x0800:
                    continue
                ihl = (_byte_value(packet[offset]) & 15) * 4
                if _byte_value(packet[offset + 9]) != 17:
                    continue
                udp = offset + ihl
                if len(packet) < udp + 8:
                    continue
                port = struct.unpack('>H', packet[udp + 2:udp + 4])[0]
                length = struct.unpack('>H', packet[udp + 4:udp + 6])[0]
                payload = packet[udp + 8:udp + length]
                yield sec + subsec * scale, port, payload

    def basis(self, frame):
        times = self.frames.get(int(frame))
        if times is None:
            return None
        middle = (times[0] + times[1]) * 0.5
        samples = [a for t, a in self.imu if abs(t - middle) < 0.06]
        if not samples:
            return None
        count = float(len(samples))
        acceleration = tuple(sum(a[i] for a in samples) / count for i in range(3))
        gravity = _rotate_quaternion(self.quaternion, acceleration)
        up = _unit(tuple(-x for x in gravity))
        native_forward = (0.0, 0.0, 1.0)
        projection = _dot(native_forward, up)
        forward = _unit(tuple(native_forward[i] - projection * up[i]
                              for i in range(3)))
        lateral = _unit(_cross(up, forward))
        if forward is None or lateral is None:
            return None
        return forward, lateral, up


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _quantile(values, q):
    data = sorted(values)
    if not data:
        raise ValueError('empty values')
    pos = (len(data) - 1) * q
    lo = int(pos)
    hi = min(lo + 1, len(data) - 1)
    return data[lo] + (data[hi] - data[lo]) * (pos - lo)


def _line_fit(samples, min_sections=8):
    if len(samples) < min_sections:
        return None
    # Robust initialization prevents high-leverage returns rotating the fit.
    seed = sorted(samples)[::max(1, int(math.ceil(len(samples) / 64.0)))]
    slopes = [(b[1] - a[1]) / (b[0] - a[0])
              for i, a in enumerate(seed) for b in seed[i + 1:]
              if abs(b[0] - a[0]) > 1e-6]
    if not slopes:
        return None
    slope = _quantile(slopes, .5)
    offset = _quantile([p[1] - slope * p[0] for p in samples], .5)
    active = []
    for _ in range(6):
        residuals = [(p, p[1] - (slope * p[0] + offset)) for p in samples]
        median = _quantile([r[1] for r in residuals], 0.5)
        mad = 1.4826 * _quantile([abs(r[1] - median) for r in residuals], 0.5)
        limit = max(0.008, 2.5 * mad)
        selected = [p for p, error in residuals if abs(error - median) < limit]
        unchanged = selected == active
        active = selected
        if len(active) < min_sections:
            return None
        n = float(len(active))
        sx = sum(p[0] for p in active)
        sy = sum(p[1] for p in active)
        sxx = sum(p[0] * p[0] for p in active)
        sxy = sum(p[0] * p[1] for p in active)
        den = n * sxx - sx * sx
        if abs(den) < 1e-9:
            return None
        slope = (n * sxy - sx * sy) / den
        offset = (sy - slope * sx) / n
        if unchanged:
            break
    errors = [abs(p[1] - (slope * p[0] + offset)) for p in active]
    return slope, offset, active, _quantile(errors, 0.5)


def _track_point(s, lateral, height, basis):
    forward, left, up = basis
    return tuple(s * forward[i] + lateral * left[i] + height * up[i]
                 for i in range(3))


def _heading_basis(basis, angle_deg):
    angle = math.radians(angle_deg)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    forward, lateral, up = basis
    return (
        tuple(cosine * forward[i] + sine * lateral[i] for i in range(3)),
        tuple(-sine * forward[i] + cosine * lateral[i] for i in range(3)),
        up)


def _rail_candidates(projected):
    """Find narrow vertical ridges from this frame, without a gauge prior."""
    width = 0.025
    buckets = {}
    for point in projected:
        if not (0.3 <= point[0] <= 6.2 and abs(point[1]) <= 3.0 and
                -1.0 <= point[2] <= 2.0):
            continue
        index = int(math.floor(point[1] / width))
        buckets.setdefault(index, []).append(point)
    peaks = sorted(buckets, key=lambda key: len(buckets[key]), reverse=True)
    chosen = []
    for index in peaks:
        center = (index + 0.5) * width
        if len(buckets[index]) < 24:
            break
        if any(abs(center - item['seed']) < 0.18 for item in chosen):
            continue
        region = [p for p in projected if 0.3 <= p[0] <= 6.2 and
                  abs(p[1] - center) < 0.075]
        heights = [p[2] for p in region]
        if len(region) < 35:
            continue
        vertical_span = _quantile(heights, 0.95) - _quantile(heights, 0.05)
        if not 0.06 <= vertical_span <= 0.65:
            continue
        laterals = [p[1] for p in region]
        observed = _quantile(laterals, 0.5)
        spread = 1.4826 * _quantile(
            [abs(value - observed) for value in laterals], 0.5)
        s_bins = set(int((p[0] - 0.3) / 0.15) for p in region)
        chosen.append({
            'seed': center,
            'lateral': observed,
            'points': region,
            'support': len(region),
            's_bins': len(s_bins),
            's_span': _quantile([p[0] for p in region], 0.95) -
                      _quantile([p[0] for p in region], 0.05),
            'spread': spread,
            'top': _quantile(heights, 0.90),
            'peak': len(buckets[index])
        })
        if len(chosen) >= 10:
            break
    return chosen


def _fit_rail(candidate, head_band=False):
    bins = {}
    for point in candidate['points']:
        bins.setdefault(int((point[0] - 0.3) / 0.15), []).append(point)
    lateral_samples = []
    height_samples = []
    sample_regions = {}
    for region in bins.values():
        if len(region) < 5:
            continue
        if head_band:
            # A visible upper-surface band, not a certified gauge face.
            # The reference height is estimated independently per section;
            # no nominal gauge or nominal rail-head width enters this step.
            top = _quantile([p[2] for p in region], .95)
            region = [p for p in region if top - .025 <= p[2] <= top]
            if len(region) < 2:
                continue
        s = _quantile([p[0] for p in region], 0.5)
        lateral_samples.append((s, _quantile([p[1] for p in region], 0.5)))
        height_samples.append((s, _quantile([p[2] for p in region], 0.90)))
        sample_regions[s] = region
    min_sections = 4 if head_band else 8
    lateral_fit = _line_fit(lateral_samples, min_sections)
    height_fit = _line_fit(height_samples, min_sections)
    # Four observed 15 cm sections suffice for a test-stage local fit.
    # Older offline fitting keeps its eight-section requirement.
    if (lateral_fit is not None and height_fit is not None and
            len(lateral_fit[2]) >= min_sections and (head_band or candidate['s_span'] >= 1.2)):
        return {
            'mode': 'dense',
            'measurement_kind': 'observed_head_band' if head_band else 'observed_rail_features',
            'line': lateral_fit[:2],
            'height': height_fit[:2],
            'samples': lateral_fit[2],
            'all_samples': lateral_samples,
            'height_samples': height_samples,
            # Exact source returns for retained lateral-section statistics.
            # These are not individually classified as rail-surface inliers.
            'support_points': [p[3] for s, unused in lateral_fit[2]
                               for p in sample_regions[s] if len(p) > 3],
            'residual': lateral_fit[3],
            'support': candidate['support'],
            'spread': candidate['spread'],
            'top': candidate['top']
        }
    if head_band or candidate['support'] < 50 or candidate['spread'] > 0.055:
        return None
    return {
        'mode': 'sparse',
        'line': (0.0, candidate['lateral']),
        'height': (0.0, candidate['top']),
        'samples': [(p[0], p[1]) for p in candidate['points']],
        'residual': candidate['spread'],
        'support': candidate['support'],
        'spread': candidate['spread'],
        'top': candidate['top']
    }


def _pair_score(left, right):
    separation = right['line'][1] - left['line'][1]
    if not 0.30 <= separation <= 3.20 or left['mode'] != right['mode']:
        return None
    parallel = abs(left['line'][0] - right['line'][0])
    height_difference = abs(left['top'] - right['top'])
    if parallel > 0.10 or height_difference > 0.35:
        return None
    if left['mode'] == 'dense':
        return (min(len(left['samples']), len(right['samples'])) * 8.0 +
                min(left['support'], right['support']) * 0.02 -
                parallel * 100.0 - height_difference * 5.0)
    return (min(left['support'], right['support']) * 0.10 -
            max(left['spread'], right['spread']) * 100.0 -
            height_difference * 5.0)


def _fit_centerline_at_heading(points, basis, diagnostics=None, live=False):
    projected = [(_dot(p, basis[0]), _dot(p, basis[1]), _dot(p, basis[2]), p)
                 for p in points]
    candidates = _rail_candidates(projected)
    rails = [rail for rail in
             (_fit_rail(candidate, head_band=live) for candidate in candidates)
             if rail is not None]
    if diagnostics is not None:
        diagnostics['max_candidates'] = max(diagnostics.get('max_candidates', 0), len(candidates))
        diagnostics['max_fitted_rails'] = max(diagnostics.get('max_fitted_rails', 0), len(rails))
    best_pair = None
    best_score = None
    for i, first in enumerate(rails):
        for second in rails[i + 1:]:
            left, right = sorted((first, second), key=lambda rail: rail['line'][1])
            if live and left['line'][1] * right['line'][1] >= 0:
                continue
            score = _pair_score(left, right)
            if score is not None and (best_score is None or score > best_score):
                best_pair = (left, right)
                best_score = score
    if best_pair is None:
        return None
    left, right = best_pair
    mode = left['mode']
    if mode == 'dense':
        low = max(min(p[0] for p in rail['samples']) for rail in best_pair)
        high = min(max(p[0] for p in rail['samples']) for rail in best_pair)
        if high <= low or (not live and high - low < 1.2):
            return None
        station = (low + high) * 0.5
    else:
        low, high = 0.6, 5.8
        station = sum(_quantile([p[0] for p in rail['samples']], 0.5)
                      for rail in best_pair) * 0.5

    def value(line, s):
        return line[0] * s + line[1]

    left_l = value(left['line'], station)
    right_l = value(right['line'], station)
    gauge = right_l - left_l
    if gauge <= 0.0:
        return None
    center_l = (left_l + right_l) * 0.5
    plane_height = (value(left['height'], station) +
                    value(right['height'], station)) * 0.5
    left_point = _track_point(station, left_l, plane_height, basis)
    center_point = _track_point(station, center_l, plane_height, basis)
    right_point = _track_point(station, right_l, plane_height, basis)
    center_slope = (left['line'][0] + right['line'][0]) * 0.5
    center_offset = (left['line'][1] + right['line'][1]) * 0.5
    height_slope = (left['height'][0] + right['height'][0]) * 0.5
    height_offset = (left['height'][1] + right['height'][1]) * 0.5
    endpoints = [_track_point(
        s, center_slope * s + center_offset,
        height_slope * s + height_offset, basis) for s in (low, high)]
    residual = max(left['residual'], right['residual'])
    parallel = abs(left['line'][0] - right['line'][0])
    if mode == 'dense':
        coverage = min(1.0, min(len(left['samples']),
                                len(right['samples'])) / 24.0)
        confidence = (0.45 + 0.35 * coverage +
                      0.10 * max(0.0, 1.0 - residual / 0.035) +
                      0.10 * max(0.0, 1.0 - parallel / 0.05))
    else:
        support = min(1.0, min(left['support'], right['support']) / 100.0)
        confidence = (0.42 + 0.20 * support +
                      0.18 * max(0.0, 1.0 - residual / 0.055) +
                      0.10 * max(0.0, 1.0 - parallel / 0.05))
    return {
        'endpoints': endpoints,
        'span': high - low,
        'gauge': gauge,
        'left_distance': gauge * 0.5,
        'right_distance': gauge * 0.5,
        'left_point': left_point,
        'center_point': center_point,
        'right_point': right_point,
        'confidence': min(1.0, confidence),
        'fit_mode': mode,
        'rail_lines': (left['line'], right['line']),
        'rail_heights': (left['height'], right['height']),
        'rail_fits': best_pair,
        '_fit_score': best_score
    }


def fit_centerline(points, basis, diagnostics=None, live=False):
    """Fit rails from current points; 1435 mm is intentionally not used."""
    direct = _fit_centerline_at_heading(points, basis, diagnostics, live=live)
    if direct is not None:
        direct.pop('_fit_score', None)
        direct['heading_correction_deg'] = 0.0
        return direct
    best = None
    best_angle = None
    best_score = None
    broad_angles = list(range(-90, 0, 5)) + list(range(5, 91, 5))
    scans = [(-5, 5), (-10, 10), broad_angles] if live else [broad_angles]
    for angles in scans:
        for angle in angles:
            result = _fit_centerline_at_heading(
                points, _heading_basis(basis, angle), diagnostics, live=live)
            if result is None:
                continue
            score = result['_fit_score'] * result['confidence']
            if best is None or score > best_score:
                best, best_angle, best_score = result, angle, score
        if best is not None:
            break
    if best is None:
        return None
    for angle in range(best_angle - 4, best_angle + 5):
        if angle % 5 == 0:
            continue
        result = _fit_centerline_at_heading(
            points, _heading_basis(basis, angle), diagnostics, live=live)
        if result is None:
            continue
        score = result['_fit_score'] * result['confidence']
        if score > best_score:
            best, best_angle, best_score = result, angle, score
    best.pop('_fit_score', None)
    best['heading_correction_deg'] = float(best_angle)
    return best
