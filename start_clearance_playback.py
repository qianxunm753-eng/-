# -*- coding: utf-8 -*-
"""Open the real 1212121.pcap in RSView with its offline measurement overlay.

Run with --script=start_clearance_playback.py. Validation mode only exits its
own RSView process; normal mode leaves the window open for user playback.
"""
import json
import math
import os
import tempfile
import time
import traceback
import shutil
from PythonQt import QtCore, QtGui
import rsview.applogic as logic

root = os.path.dirname(logic._rail_centerline_path)
validate = os.environ.get('RSVIEW_CLEARANCE_VALIDATE') == '1'
online = os.environ.get('RSVIEW_CLEARANCE_COMPUTE') == 'online'
report_path = os.environ.get('RSVIEW_CLEARANCE_REPORT')
if report_path is not None:
    report_path = report_path.decode('mbcs')
frame_number = int(os.environ.get('RSVIEW_CLEARANCE_FRAME', '260'))
input_path = os.path.join(os.path.expanduser('~'), 'Desktop', '1212121.pcap')
pointer_folder = None
report = {'input': input_path, 'frame': frame_number, 'error': None,
          'mode': ('RSView current-cloud online simulation with recorded experimental poses' if online
                   else 'RSView offline precomputed playback, not live compute')}
online_active = False
online_timer = None
online_state = {}


def unit(v):
    norm = math.sqrt(sum(x*x for x in v))
    return [x/norm for x in v] if norm > 1e-8 else [0., 0., 1.]


def camera_for(packet):
    if not packet or not packet['centerline'] or not packet['measurements']:
        return
    a, b = packet['centerline'][0]
    tangent = unit([y-x for x, y in zip(a, b)])
    item = packet['measurements'][0]
    across = unit([x-y for x, y in zip(item['projected_L'], item['foot_L'])])
    normal = unit([tangent[1]*across[2]-tangent[2]*across[1],
                   tangent[2]*across[0]-tangent[0]*across[2],
                   tangent[0]*across[1]-tangent[1]*across[0]])
    height = sum(n*(p-q) for n, p, q in zip(normal, item['point_L'], item['projected_L']))
    if height < 0:
        normal = [-x for x in normal]
    camera = logic.app.mainView.GetRenderer().GetActiveCamera()
    focal = [(x+y)*.5 for x, y in zip(a, b)]
    camera.SetFocalPoint(*focal)
    camera.SetPosition(*[c+n*6-t*4-s*3 for c, n, t, s in zip(focal, normal, tangent, across)])
    camera.SetViewUp(*normal)
    camera.ParallelProjectionOn()
    camera.SetParallelScale(3.4)
    logic.smp.Render(logic.app.mainView)


def finish_report():
    if validate and report.get('error') is None and logic.app.railOverlay is not None:
        overlay = logic.app.railOverlay
        report['display_timing_ms'] = getattr(overlay,'last_display_timings',None)
        if hasattr(overlay,'boundary_actors') and overlay.current is not None:
            report['native_boundary_points'] = [list(upload.data.GetPoint(i))
                for upload,actor in overlay.boundary_actors[:1]
                for i in range(upload.data.GetNumberOfPoints())]
            report['centerline_feet'] = [list(upload.data.GetPoint(i))
                for upload,actor in overlay.boundary_actors[1:]
                for i in range(upload.data.GetNumberOfPoints())]
            selected = overlay.current['measurements']
            from clearance_overlay import dimension_endpoints
            report['native_dimension_lines'] = []
            for item,(source,actor) in zip(selected,overlay.measure_lines):
                endpoint,foot=dimension_endpoints(item)
                actual=[list(source.GetPoint1()),list(source.GetPoint2())]
                report['native_dimension_lines'].append(dict(display_id=item.get('display_id'),endpoints=actual,
                    dimension_model=item.get('dimension_model','raw_boundary'),visible=bool(actor.GetVisibility())))
                if not actor.GetVisibility() or any(abs(a-b)>1e-8 for point,expected in
                        zip(actual,(endpoint,foot)) for a,b in zip(point,expected)):
                    report['error']='Native dimension actor geometry mismatch'
                if abs(math.sqrt(sum((a-b)**2 for a,b in zip(*actual)))-item['distance_m'])>1e-8:
                    report['error']='Native dimension actor length mismatch'
            for points,key in ((report['native_boundary_points'],'point_L'),
                               (report['centerline_feet'],'foot_L')):
                if len(points) != len(selected):
                    report['error'] = 'Native boundary marker count mismatch'
                if any(abs(a-b)>1e-6 for point,item in zip(points,selected) for a,b in zip(point,item[key])):
                    report['error'] = 'Native boundary marker provenance mismatch'
            if hasattr(overlay,'raw_boundary_actor'):
                upload,actor = overlay.raw_boundary_actor
                report['unconfirmed_raw_points'] = [list(upload.data.GetPoint(i))
                    for i in range(upload.data.GetNumberOfPoints())]
                expected = [item['observed_point_L'] for item in selected if item.get('raw_near_warning')
                    and item.get('observed_source_index') != item.get('source_index')]
                actual = report['unconfirmed_raw_points']
                if len(actual) != len(expected) or bool(actor.GetVisibility()) != bool(expected):
                    report['error'] = 'Native raw marker count or visibility mismatch'
                if any(abs(a-b)>1e-6 for point,expected_point in zip(actual,expected)
                       for a,b in zip(point,expected_point)):
                    report['error'] = 'Native raw marker provenance mismatch'
    if report_path:
        with open(report_path,'wb') as stream:
            stream.write(json.dumps(report,indent=2).encode('utf-8'))
    if validate:
        if online and logic.app.railOverlay is not None:
            logic.app.railOverlay.close()
        QtGui.QApplication.instance().quit()


def begin_online(reader, steps):
    global online_active, online_timer
    online_active = True
    online_state.update(reader=reader,steps=steps,started=time.time(),next=max(6,frame_number-6),
                        phase='warmup_first',first_sent=False,records=[],input_frames=[],poll_ms=[],due=time.time())
    # A paused, exact-source result proves actual embedded-to-worker execution.
    # Feed a short chronological prefix through the same actual native callback.
    # An isolated scan has no observed velocity; don't invent a static pose or
    # make the first paused picture look like a compensated/background-filtered
    # result before the worker has a real prior motion interval.
    online_timer = QtCore.QTimer()
    online_timer.connect('timeout()', online_tick)
    online_timer.start(40)


def online_tick():
    try:
        state, overlay = online_state, logic.app.railOverlay
        started = time.time()
        packet = overlay.poll()
        state['poll_ms'].append((time.time()-started)*1000.)
        state['poll_ms'] = state['poll_ms'][-500:]
        if packet and packet.get('status') == 'failed':
            raise RuntimeError(packet.get('error','worker failure'))
        if state['phase'] == 'warmup_first':
            if not state['first_sent']:
                number=state['next']
                state['first_frame']=number
                state['reader'].UpdatePipeline(float(state['steps'][number]))
                logic.app.scene.AnimationTime=float(state['steps'][number])
                logic.onTimeChanged()
                state['next']+=1
                state['first_sent']=True
            elif packet and packet['sequence'] == state['first_frame'] and packet['current_source']:
                # The booting worker may otherwise replace EVERY prefix scan
                # with the final scan. Observe one real past input completing
                # before the paced prefix, without blocking the Qt event loop.
                report['startup_processed_prefix_frame']=packet['sequence']
                state['phase']='warmup'
                state['due']=time.time()
            elif time.time()-state['started'] > 25:
                raise RuntimeError('Online first causal prefix scan timed out')
            return
        if state['phase'] == 'warmup':
            if time.time() >= state['due']:
                number=state['next']
                state['reader'].UpdatePipeline(float(state['steps'][number]))
                logic.app.scene.AnimationTime=float(state['steps'][number])
                logic.onTimeChanged()
                state['next']+=1
                state['due']=time.time()+.1
            if state['next'] > frame_number:
                state['phase']='initial'
            return
        if state['phase'] == 'initial':
            if packet is None or packet['sequence'] != frame_number:
                if time.time()-state['started'] > 25:
                    raise RuntimeError('Online paused current scan timed out')
                return
            report.update(overlay_type=type(overlay).__name__,packet_sequence=packet['sequence'],
                calculation=packet['calculation'],worker_pid=packet['worker_pid'],
                current_source=packet['current_source'],pose_scope=packet['pose_scope'],
                initial_measurements=packet['measurements'][:4],initial_mode=packet['mode'],
                initial_scene_diagnostics=packet.get('scene_diagnostics',{}),
                initial_status_text=str(overlay.status_label.text),
                reader_frames=len(state['steps']),native_raw_points=state['reader'].GetClientSideObject().GetOutput().GetNumberOfPoints())
            if packet['calculation'] != 'online_current_cloud' or not packet['current_source']:
                raise AssertionError('Native online path did not calculate this input scan')
            for item in packet['measurements']:
                length=math.sqrt(sum((a-b)**2 for a,b in zip(item['projected_L'],item['foot_L'])))
                if abs(length-item['distance_m']) > 1e-8 or item['source_frame'] != frame_number:
                    raise AssertionError('Online displayed distance/native source frame disagree')
            camera_for(packet)
            if not validate:
                online_timer.stop()
                return  # Own overlay's timer continues; leave native window open.
            state['phase'] = 'paced'
            state['next'] = max(6,frame_number-8)
            state['paced_until'] = min(len(state['steps'])-1,frame_number+24)
            state['due'] = time.time()
        if state['phase'] == 'paced':
            if packet is not None and (not state['records'] or
                    (packet['epoch'],packet['sequence']) != (state['records'][-1]['epoch'],state['records'][-1]['sequence'])):
                state['records'].append(dict(sequence=packet['sequence'],epoch=packet['epoch'],
                    mode=packet['mode'],compute_ms=packet.get('timing_ms',{}).get('packet_total'),
                    age_s=packet['result_age_s'],current_source=packet['current_source'],
                    fresh_distance=packet['fresh_distance'],measurements=packet['measurements'][:4],
                    motion_status=packet.get('motion_status')))
            if state['next'] <= state['paced_until'] and time.time() >= state['due']:
                number = state['next']
                state['reader'].UpdatePipeline(float(state['steps'][number]))
                logic.app.scene.AnimationTime = float(state['steps'][number])
                logic.onTimeChanged()
                state['input_frames'].append(number)
                state['next'] += 1
                state['due'] = time.time()+.1  # 600 RPM lidar's scan arrival, not car wheel RPM.
            if state['next'] > state['paced_until']:
                diagnostic = overlay.host.diagnostics()
                if diagnostic['running'] is None and diagnostic['pending'] is None:
                    report.update(paced_inputs=state['input_frames'],paced_results=state['records'],
                        worker=diagnostic,poll_max_ms=max(state['poll_ms']),
                        status_text=str(overlay.status_label.text),
                        visible_label_texts=[label.GetInput() for label in overlay.labels if label.GetVisibility()],
                        live_0_2s_verified=False,absolute_accuracy_verified=False)
                    image_path = os.path.splitext(report_path)[0]+'.png'
                    descriptor, temp_image = tempfile.mkstemp(suffix='.png'); os.close(descriptor)
                    logic.smp.WriteImage(temp_image,view=logic.app.mainView)
                    shutil.move(temp_image,image_path)
                    report['image'] = image_path
                    online_timer.stop()
                    finish_report()
        if time.time()-state['started'] > 40:
            raise RuntimeError('Native online paced probe timeout')
    except Exception:
        report['error'] = traceback.format_exc()
        report['online_diagnostics'] = dict(worker=logic.app.railOverlay.host.diagnostics(),
            input_error=logic.app.railOverlay.input_error,
            last_input=logic.app.railOverlay.last_input,
            last_result_sequence=(logic.app.railOverlay.last_result or {}).get('sequence'))
        print(report['error'])
        online_timer.stop()
        finish_report()


def run():
    global pointer_folder
    choose, network = logic.chooseCalibrationFile, logic.networkConfigPcap
    dialog_factories = {}
    try:
        from clearance_native import prepare_pcap_reader,patch_recording_decoder
        dialog_factories = patch_recording_decoder(logic)
        logic.chooseCalibrationFile = lambda unused: True
        logic.networkConfigPcap = lambda unused: True
        pointer_folder = tempfile.mkdtemp(prefix='rsview-clearance-')
        pointer = os.path.join(pointer_folder, 'input_path.log')
        with open(pointer, 'wb') as stream:
            stream.write(input_path.encode('mbcs') if isinstance(input_path, unicode) else input_path)
        logic.openPCAP(pointer)
        logic.stop()
        logic.app.tempTimer.stop()
        reader = logic.app.reader
        steps = prepare_pcap_reader(reader)
        # Exercise the same decoder startup/calibration scans as normal input.
        for step in steps[:7]:
            reader.UpdatePipeline(float(step))
        if logic.app.railOverlay is None:
            raise RuntimeError('RSView did not create the trackside overlay')
        report['initial_reader_steps'] = len(steps)
        report['initial_step_limits'] = [steps[0],steps[-1]] if steps else []
        report['reader_filename'] = str(reader.FileName)
        report['reader_lidar_type'] = str(logic.app.curLidarType)
        report['reader_ports'] = [int(reader.MsopPort),int(reader.DifopPort)]
        if not 0 <= frame_number < len(steps):
            raise ValueError('frame outside PCAP')
        logic.app.scene.UpdateAnimationUsingDataTimeSteps()
        logic.updateSliderTimeRange()
        logic.app.tempTimer.start(1)
        if online:
            begin_online(reader,steps)
            return
        logic.app.scene.AnimationTime = float(steps[frame_number])
        reader.UpdatePipeline(float(steps[frame_number]))
        QtGui.QApplication.processEvents()
        logic.onTimeChanged()
        packet = logic.app.railOverlay.last_result
        if packet is None or packet['sequence'] != frame_number:
            raise RuntimeError('Normal RSView time-change callback did not select the requested frame')
        camera_for(packet)
        if validate:
            report['overlay_type'] = type(logic.app.railOverlay).__name__
            report['reader_frames'] = len(steps)
            report['packet_sequence'] = packet['sequence']
            report['timekeeper_frame'] = int(logic.getTimeKeeper().getTime())
            report['measurements'] = packet['measurements']
            report['visible_measurements'] = logic.app.railOverlay.current['measurements']
            report['label_texts'] = [label.GetInput() for label in logic.app.railOverlay.labels if label.GetVisibility()]
            report['visible_measure_lines'] = sum(actor.GetVisibility() for source, actor in logic.app.railOverlay.measure_lines)
            report['visible_center_segments'] = sum(actor.GetVisibility() for source, actor in logic.app.railOverlay.center_lines)
            report['ground_points'] = packet['clouds']['ground']['count']
            report['ground_visible'] = bool(logic.app.railOverlay.cloud_actors['ground'][1].GetVisibility())
            report['background_points'] = packet['clouds'].get('background',{}).get('count',0)
            report['background_visible'] = bool(logic.app.railOverlay.cloud_actors['background'][1].GetVisibility())
            report['scene_diagnostics'] = packet.get('scene_diagnostics',{})
            if report['background_points'] and not report['background_visible']:
                raise AssertionError('Confirmed background context was hidden instead of dimmed')
            for item in packet['measurements']:
                line_length = math.sqrt(sum((a-b)**2 for a,b in zip(item['projected_L'],item['foot_L'])))
                if abs(line_length-item['distance_m']) > 1e-8 or item['source_frame'] != frame_number:
                    raise AssertionError('Displayed distance geometry/native source frame disagree')
            if report['ground_points'] and not report['ground_visible']:
                raise AssertionError('Ground context is present in cache but hidden in RSView')
            # Sequential playback uses current coordinates and distance every
            # frame; identity is recorded separately, never a held measurement.
            report['sequential_frames'] = []
            for number in range(max(0,frame_number-4),frame_number+1):
                reader.UpdatePipeline(float(steps[number]))
                logic.app.scene.AnimationTime = float(steps[number])
                QtGui.QApplication.processEvents()
                logic.onTimeChanged()
                p = logic.app.railOverlay.last_result
                if p is None or p['sequence'] != number:
                    raise RuntimeError('Sequential playback used another frame result')
                report['sequential_frames'].append(dict(frame=number,
                    measurements=logic.app.railOverlay.current['measurements'],
                    label_texts=[label.GetInput() for label in logic.app.railOverlay.labels if label.GetVisibility()]))
            # Real backward/forward seeks, including a known ambiguous frame.
            report['seeks'] = []
            for number in (240, 280, 320, 380, frame_number):
                reader.UpdatePipeline(float(steps[number]))
                logic.app.scene.AnimationTime = float(steps[number])
                QtGui.QApplication.processEvents()
                logic.onTimeChanged()
                p = logic.app.railOverlay.last_result
                if p is None or p['sequence'] != number:
                    raise RuntimeError('Seek used another frame result')
                report['seeks'].append(dict(frame=number, packet_frame=p['sequence'], mode=p['mode'],
                    lines=sum(actor.GetVisibility() for source, actor in logic.app.railOverlay.measure_lines)))
            camera_for(packet)
            image_path = os.path.splitext(report_path)[0]+'.png'
            descriptor, temporary_image = tempfile.mkstemp(suffix='.png')
            os.close(descriptor)
            logic.smp.WriteImage(temporary_image, view=logic.app.mainView)
            shutil.move(temporary_image, image_path)
            report['image'] = image_path
    except Exception:
        report['error'] = traceback.format_exc()
        print(report['error'])
    finally:
        logic.chooseCalibrationFile, logic.networkConfigPcap = choose, network
        for name,factory in dialog_factories.items():
            setattr(logic,name,factory)
        if pointer_folder:
            try:
                os.remove(os.path.join(pointer_folder, 'input_path.log'))
                os.rmdir(pointer_folder)
            except OSError:
                pass  # No recursive removal of decoder-created files.
        if not online_active:
            finish_report()


timer = QtCore.QTimer()
timer.setSingleShot(True)
timer.connect('timeout()', run)
timer.start(1000)
