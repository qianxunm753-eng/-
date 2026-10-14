"""RSView startup script: real PCAP frames, paced input, shared live callbacks.

No UDP sockets, no precomputed fits, no changes to the source PCAP.
"""
import datetime
import json
import os
import tempfile
import time
import traceback
import shutil
from PythonQt import QtCore, QtGui
import rsview.applogic as logic
from rail_replay import ReplayClock, lock_simulated_source

root = os.path.dirname(logic._rail_centerline_path)
session = os.path.join(root, 'simulation', datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
os.makedirs(session)
pace_time = getattr(time, 'perf_counter', time.clock)
auto_exit = os.environ.get('RSVIEW_SIM_AUTO_EXIT') == '1'
loops = int(os.environ.get('RSVIEW_SIM_LOOPS', '0'))
yield_ms = float(os.environ.get('RSVIEW_SIM_YIELD_MS', '0'))
input_path = os.environ.get('RSVIEW_SIM_PCAP', os.path.join(os.path.expanduser('~'), 'Desktop', '22222.pcap'))
frame_limit = int(os.environ.get('RSVIEW_SIM_FRAMES', '31'))
input_label = os.path.basename(input_path)
report = {'input': input_path.decode('mbcs') if isinstance(input_path, str) else input_path,
          'frame_limit': frame_limit, 'mode': 'simulated decoded live input',
          'network_tested': False, 'frames': [], 'measurements': [], 'error': None,
          'experimental_yield_ms': yield_ms, 'timing_ms': {}}
clouds, frames_by_sequence = [], {}
overlay = None
cycle, source_frame = -1, 0
seen_result = None
playing = True
ended = False
resources_closed = False
source_log = open(os.path.join(session, 'source-frames.jsonl'), 'w')


def timed(name, function):
    if not auto_exit:
        return function
    def invoke(*args, **kwargs):
        started = pace_time()
        try:
            return function(*args, **kwargs)
        finally:
            samples = report['timing_ms'].setdefault(name, [])
            samples.append((pace_time() - started) * 1000.)
            del samples[:-4096]
    return invoke


def save_report():
    with open(os.path.join(session, 'report.json'), 'w') as stream:
        json.dump(report, stream, indent=2)


def snapshot():
    fd, path = tempfile.mkstemp(suffix='.png')
    os.close(fd)
    logic.smp.WriteImage(path, view=logic.app.mainView)
    shutil.move(path, os.path.join(session, 'preview.png'))


def end():
    global ended
    if ended:
        return
    ended = True
    logic.app.playing = False
    logic.app.playTimer.stop()
    report['observed_frames'] = overlay.sequence if overlay else 0
    report['skipped_input_frames'] = clock.skipped if 'clock' in globals() else 0
    report['finished_unix_s'] = time.time()
    save_report()
    source_log.close()
    if overlay:
        overlay.pause()
    if auto_exit:
        exit_timer.start(100)


def shutdown():
    global resources_closed
    end()
    if overlay and not resources_closed:
        resources_closed = True
        overlay.close()
        thread = getattr(overlay.worker, 'thread', None)
        if thread:
            thread.join(3.)


def exit_probe():
    shutdown()
    QtGui.QApplication.instance().quit()


class SimulatedSensor(object):
    def UpdatePipeline(self):
        pass

    def GetClientSideObject(self):
        return self

    def GetOutput(self):
        return clouds[source_frame]

    def Start(self):
        global playing
        if not playing:
            clock.start += pace_time() - self.pause_time
        playing = True

    def Stop(self):
        global playing
        self.pause_time = pace_time()
        playing = False


class SimulatedScene(object):
    AnimationTime = -1
    EndTime = 30
    StartTime = 0

    def GoToLast(self):
        pass


def timesteps():
    global cycle, source_frame
    if not playing or ended:
        return []
    entry = clock.update(pace_time())
    if clock.finished:
        end()
        return []
    if entry is not None:
        next_cycle, source_frame = entry
        if next_cycle != cycle:
            overlay.pause()
            cycle = next_cycle
        producer.GetClientSideObject().SetOutput(clouds[source_frame])
        producer.GetClientSideObject().Modified()
        producer.SMProxy.MarkModified(producer.SMProxy)
        producer.UpdatePipeline()
        sequence = overlay.sequence + 1
        context = dict(sequence=sequence, cycle=cycle + 1, source_frame=source_frame,
                       source_timestamp=report['frames'][source_frame]['timestamp'],
                       received=time.time(), skipped=clock.skipped)
        frames_by_sequence[sequence] = context
        if len(frames_by_sequence) > 256:
            del frames_by_sequence[min(frames_by_sequence)]
        source_log.write(json.dumps(context) + '\n')
        source_log.flush()
        if source_frame >= 30 and 'rendered_bounds_check' not in report:
            report['rendered_bounds_check'] = dict(frame=source_frame,
                expected=list(clouds[source_frame].GetBounds()),
                actual=list(producer.GetDataInformation().GetBounds()))
        logic.app.timeLabel.setText('  SIM source frame: %d | cycle: %d' % (source_frame, cycle + 1))
    return [cycle * len(clouds) + source_frame]


def update_banner():
    measured = frames_by_sequence.get(overlay.last_result_sequence)
    label = ('cycle %d / source frame %d' % (measured['cycle'], measured['source_frame'])
             if measured else 'none')
    banner.SetDisplayPosition(16, max(150, logic.app.mainView.ViewSize[1] - 90))
    banner.SetInput('SIMULATED LIVE | %s | source 0-%d | cycle %d / frame %d\n'
                    'Measurement: %s | skipped inputs: %d\n'
                    'Held annotations belong to their measurement frame; no motion compensation' %
                    (input_label, len(clouds) - 1, cycle + 1, source_frame, label, clock.skipped))


def playback():
    global seen_result
    try:
        original_playback()
        if yield_ms:
            time.sleep(yield_ms / 1000.)
        measured = frames_by_sequence.get(overlay.last_result_sequence)
        if measured and seen_result != overlay.last_result_sequence:
            seen_result = overlay.last_result_sequence
            row = dict(measured)
            row.update(gauge_mm=overlay.last_result['gauge'] * 1000.,
                       age_ms=(time.time() - overlay.result_time) * 1000.)
            report['measurements'].append(row)
            report['measurements'] = report['measurements'][-300:]
            save_report()
            if auto_exit and not os.path.exists(os.path.join(session, 'preview.png')):
                snapshot()
        if auto_exit and cycle >= 1 and source_frame >= 23 and not os.path.exists(os.path.join(session, 'preview.png')):
            snapshot()
    except Exception:
        report['error'] = traceback.format_exc().decode('mbcs', 'replace')
        print(report['error'].encode('ascii', 'backslashreplace'))
        end()


def start():
    global overlay, producer, clock, banner, original_playback
    pointer = None
    try:
        logic.chooseCalibrationFile = lambda unused: True
        logic.networkConfigPcap = lambda unused: True
        pointer = tempfile.NamedTemporaryFile(mode='w', delete=False)
        pointer.write(input_path)
        pointer.close()
        logic.openPCAP(pointer.name)
        logic.stop()
        logic.app.tempTimer.stop()
        if logic.app.railOverlay:
            logic.app.railOverlay.close()
            logic.app.railOverlay = None
        reader = logic.app.reader
        client = reader.GetClientSideObject()
        client.SetNumberOfTrailingFrames(0)
        logic.app.numberOfTrailingFrames = 0
        logic.app.trailingFramesSpinBox.setValue(0)
        client.Modified()
        reader.UpdatePipeline(1.)
        available = list(reader.TimestepValues)
        selected = available if frame_limit == 0 else available[:frame_limit]
        for frame, timestep in enumerate(selected):
            reader.UpdatePipeline(float(timestep))
            cloud = logic.vtk.vtkPolyData()
            cloud.DeepCopy(client.GetOutput())
            stamps = cloud.GetPointData().GetArray('timestamp')
            if stamps is None or not stamps.GetNumberOfTuples():
                raise ValueError('Frame %d has no timestamps' % frame)
            timestamp = stamps.GetRange()[0]
            report['frames'].append(dict(frame=frame, timestamp=timestamp,
                                         points=cloud.GetNumberOfPoints()))
            clouds.append(cloud)
        view = logic.app.mainView
        reader_rep = logic.smp.GetRepresentation(reader, view)
        producer = logic.smp.PVTrivialProducer()
        producer.GetClientSideObject().SetOutput(clouds[0])
        producer.UpdatePipeline()
        rep = logic.smp.Show(producer, view)
        rep.Representation = 'Points'
        rep.PointSize = 2
        rep.ColorArrayName = ['POINTS', 'intensity']
        rep.LookupTable = reader_rep.LookupTable
        logic.smp.Hide(reader, view)
        logic.smp.SetActiveSource(producer)
        overlay = logic.rail_live.LiveRailOverlay(logic.smp, logic.vtk, view, logic.app.statusLabel, session)
        logic.rail_live.prepare_measurement = timed('copy', logic.rail_live.prepare_measurement)
        overlay.display = timed('display', overlay.display)
        overlay.tick = timed('tick', overlay.tick)
        overlay.worker.submit = timed('submit', overlay.worker.submit)
        overlay.worker.poll = timed('poll', overlay.worker.poll)
        logic.app.railOverlay = overlay
        report['worker_backend'] = type(overlay.worker).__name__
        report['measurement_interval'] = overlay.gate.interval
        from vtkRenderingCorePython import vtkTextActor
        banner = vtkTextActor()
        banner.GetTextProperty().SetColor(1., .85, .25)
        banner.GetTextProperty().SetFontSize(17)
        view.GetRenderer().AddActor2D(banner)
        fixture = SimulatedSensor()
        logic.getSensor = lambda: fixture
        logic.getReader = lambda: None
        logic.getCurrentTimesteps = timesteps
        logic.app.scene = SimulatedScene()
        logic.app.scene.EndTime = len(clouds) - 1
        logic.app.colorByInitialized = True
        # Match the existing online playback timer; input still follows PCAP time.
        logic.app.targetFps = 10
        logic.app.filenameLabel.setText('SIMULATED LIVE | %s | no sensor connection' % input_label)
        for widget in (logic.app.timeSlider, logic.app.timeSpinBox, logic.app.trailingFramesSpinBox):
            widget.setEnabled(False)
        lock_simulated_source(logic, overlay.message.SetInput)
        report['source_switch_locked'] = not logic.app.actions['actionClose'].isEnabled()
        view.CameraPosition = ([-5., 0., 2.5] if os.environ.get('RSVIEW_SIM_VIEW_AXIS') == '-X'
                               else [0., -5., 2.5])
        view.CameraFocalPoint = [0., 0., 2.5]
        view.CameraViewUp = [0., 0., 1.]
        view.CameraParallelProjection = 1
        view.CameraParallelScale = 3.4
        original_playback = timed('playback', logic.playbackTick)
        logic.playbackTick = playback
        original_render = timed('render', logic.smp.Render)
        def render_with_context(*args, **kwargs):
            update_banner()
            return original_render(*args, **kwargs)
        logic.smp.Render = render_with_context
        clock = ReplayClock([f['timestamp'] for f in report['frames']], pace_time(), loops)
        report['period_s'] = clock.period
        report['started_unix_s'] = time.time()
        report['loops_requested'] = loops
        save_report()
        logic.app.playing = True
        logic.setActionIcon('actionPlay', logic.IconPaths.pause)
        logic.app.playTimer.start(1)
        QtGui.QApplication.instance().connect('aboutToQuit()', shutdown)
    except Exception:
        report['error'] = traceback.format_exc().decode('mbcs', 'replace')
        print(report['error'].encode('ascii', 'backslashreplace'))
        end()
    finally:
        if pointer:
            os.unlink(pointer.name)


exit_timer = QtCore.QTimer()
exit_timer.setSingleShot(True)
exit_timer.connect('timeout()', exit_probe)
start_timer = QtCore.QTimer()
start_timer.setSingleShot(True)
start_timer.connect('timeout()', start)
start_timer.start(1000)
