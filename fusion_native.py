# -*- coding: utf-8 -*-
"""Replay image modes, with an opt-in frozen real-window distance experiment."""
import sys
import uuid
try:
    from .clearance_stop import StopClearanceController
    from .clearance_native import enable_recorded,recorded_measurement_config
    from .parking_replay import ParkingReplayDriver,capture_clock
    from .p3_replay import P3ReplayEnhancement
except (ImportError,ValueError):
    from clearance_stop import StopClearanceController
    from clearance_native import enable_recorded,recorded_measurement_config
    from parking_replay import ParkingReplayDriver,capture_clock
    from p3_replay import P3ReplayEnhancement


def enable_image(logic,contour_mode='baseline',parking_detail=False,motion_detail=False,window_measurement=False):
    if contour_mode not in ('baseline','trajectory','window','quality','parking'):raise ValueError('invalid contour mode')
    if parking_detail and contour_mode!='parking':raise ValueError('detail registration is parking replay only')
    if type(motion_detail) is not bool:raise ValueError('motion detail must be a boolean')
    if motion_detail and contour_mode!='baseline':raise ValueError('motion detail requires rolling baseline replay')
    if type(window_measurement) is not bool:raise ValueError('window measurement must be a boolean')
    if window_measurement:
        if contour_mode!='baseline' or motion_detail or parking_detail:
            raise ValueError('window measurement requires the mature rolling baseline policy')
        sensor=getattr(logic,'getSensor',None)
        if sensor is not None and sensor() is not None:
            raise ValueError('window measurement requires a recorded Airy PCAP source')
    filename=logic.getReaderFileName()
    if sys.version_info[0]==2 and isinstance(filename,str):filename=filename.decode('mbcs')
    elif isinstance(filename,bytes):filename=filename.decode('mbcs')
    config=dict(image_fusion_only=True,image_source_path=filename,image_display_limit=150000,
        image_display_filter_preview=True,
        recorded_playback_continuity=True,history_seconds=.6,config_id='rsview-image-fusion-v2')
    if contour_mode!='baseline':
        config.update(image_contour_mode=contour_mode,config_id='rsview-image-contour-v1:'+contour_mode)
    else:config['image_snapshot_support']=True
    if parking_detail:config['image_parking_detail']=True
    if motion_detail:config['image_motion_detail']=True
    if contour_mode=='baseline' and not motion_detail and not parking_detail:
        # Capability is not an active measurement view. Every mature image
        # worker can measure its existing frozen window on an explicit click.
        config.update(image_window_measurement_capable=True,
            image_window_measurement_config=recorded_measurement_config('v32'))
    if window_measurement:config['image_window_measurement']=True
    return enable_recorded(logic,config)


class ImageFusionController(StopClearanceController):
    def __init__(self,logic,action,single_action,clean_action=None,display_action=None,contour_action=None,
                 parking_action=None,capture_action=None,detail_action=None,p3_action=None,p3_baseline_action=None,
                 surface_action=None,surface_actions=None,motion_detail_action=None,
                 window_measurement_action=None,measure_action=None,pending_window_action=None):
        self.single_action=single_action;self.clean_action=clean_action;self.display_action=display_action
        self.contour_action=contour_action;self.contour_mode='baseline'
        self.parking_action=parking_action;self.capture_action=capture_action;self.parking_driver=None
        self.detail_action=detail_action;self.parking_detail=False
        self.p3_action=p3_action;self.p3_baseline_action=p3_baseline_action;self.p3_driver=None
        self.surface_action=surface_action;self.surface_actions=surface_actions or {}
        self.motion_detail_action=motion_detail_action;self.motion_detail=False
        self.window_measurement_action=window_measurement_action;self.measure_action=measure_action
        self.pending_window_action=pending_window_action
        self.window_measurement=False
        StopClearanceController.__init__(self,logic,action)

    def _enable_recorded(self):
        if self.contour_mode=='baseline' and self.window_measurement:
            return enable_image(self.logic,window_measurement=True)
        if self.contour_mode=='baseline' and self.motion_detail:
            return enable_image(self.logic,motion_detail=True)
        if self.contour_mode=='parking' and self.parking_detail:
            return enable_image(self.logic,contour_mode='parking',parking_detail=True)
        return enable_image(self.logic) if self.contour_mode=='baseline' else enable_image(self.logic,contour_mode=self.contour_mode)

    def _restore(self):
        normal=getattr(self.logic.app,'clearanceController',None)
        if normal is not None:normal._restore()

    def _can_measure_existing_window(self):
        config=getattr(self.overlay,'worker_config',{})
        return (self.overlay is not None and self._available() and self.contour_mode=='baseline'
            and not self.motion_detail and bool(config.get('image_window_measurement_capable')
                or config.get('image_window_measurement')))

    def source_ready(self):
        StopClearanceController.source_ready(self)
        if self.overlay is not None:self.overlay.window_measurement_view=self.window_measurement
        # The inherited action is an entry for rolling image fusion, not a
        # master checkbox for every mode sharing this controller.
        self.action.setChecked(self.overlay is not None and self.contour_mode!='parking')
        self.single_action.setEnabled(self.overlay is not None)
        if self.overlay is None:self.single_action.setChecked(False)
        if self.clean_action is not None:
            self.clean_action.setEnabled(self.overlay is not None)
            if self.overlay is None:self.clean_action.setChecked(False)
        if self.display_action is not None:self.display_action.setEnabled(self.overlay is not None)
        if self.contour_action is not None:
            self.contour_action.setEnabled(self.overlay is not None and self._available())
            self.contour_action.setChecked(self.contour_mode=='quality' and self.overlay is not None)
        if self.parking_action is not None:
            self.parking_action.setEnabled(self._available())
            self.parking_action.setChecked(self.contour_mode=='parking' and self.overlay is not None)
        if self.capture_action is not None:
            self.capture_action.setEnabled(self.contour_mode=='parking' and self.overlay is not None and self._available())
        if self.detail_action is not None:
            enabled=self.contour_mode=='parking' and self.overlay is not None and self._available()
            self.detail_action.setEnabled(enabled);self.detail_action.setChecked(enabled and self.parking_detail)
        if self.motion_detail_action is not None:
            enabled=self.contour_mode=='baseline' and self.overlay is not None and self._available() and not self.window_measurement
            self.motion_detail_action.setEnabled(enabled);self.motion_detail_action.setChecked(enabled and self.motion_detail)
        if self.p3_action is not None:
            enabled=self.overlay is not None and self.contour_mode=='baseline' and self._available() and not self.window_measurement
            self.p3_action.setEnabled(enabled)
            self.p3_action.setChecked(enabled and self.p3_driver is not None and self.p3_driver.valid
                and getattr(self.p3_driver,'command','enhance')=='enhance')
        surface=self.p3_driver is not None and self.p3_driver.valid and getattr(self.p3_driver,'command','enhance')=='surface'
        if self.surface_action is not None:
            enabled=self.overlay is not None and self.contour_mode=='baseline' and self._available() and not self.window_measurement
            self.surface_action.setEnabled(enabled);self.surface_action.setChecked(enabled and surface)
        packet=getattr(self.overlay,'last_result',None) or {}
        if self.window_measurement_action is not None:
            enabled=self.overlay is not None and self._available() and self.contour_mode!='parking'
            self.window_measurement_action.setEnabled(enabled)
            self.window_measurement_action.setChecked(enabled and self.window_measurement)
        if self.measure_action is not None:
            enabled=self._can_measure_existing_window()
            self.measure_action.setEnabled(enabled and not (self.p3_driver is not None and self.p3_driver.processing))
        if self.pending_window_action is not None:
            enabled=(self.overlay is not None and self.window_measurement and self._available()
                and packet.get('measurement_basis')=='rolling_20_window' and bool(packet.get('pending_window_measurements')))
            self.pending_window_action.setEnabled(enabled)
            self.pending_window_action.setChecked(enabled and getattr(self.overlay,'window_pending_view',False))
        if packet.get('measurement_basis')=='rolling_20_window':
            self.single_action.setEnabled(False);self.single_action.setChecked(False)
            self.overlay.image_single_view=False;self.overlay.image_clean_view=False
            if self.clean_action is not None:self.clean_action.setEnabled(False);self.clean_action.setChecked(False)
        for name,action in self.surface_actions.items():
            available=('local' if name=='layer' else name) in packet.get('_surface_views',{})
            action.setEnabled(bool(surface and available))
            action.setChecked(surface and getattr(self.overlay,'surface_view',None)==name)
        if surface:
            self.single_action.setEnabled(False)
            if self.clean_action is not None:self.clean_action.setEnabled(False)
        if self.p3_baseline_action is not None:
            packet=getattr(self.overlay,'last_result',None) or {}
            enabled=self.p3_driver is not None and self.p3_driver.valid and 'detail_fused' in packet.get('_image_pairs',{})
            self.p3_baseline_action.setEnabled(enabled)
            if not enabled:self.p3_baseline_action.setChecked(False)

    def source_closed(self):
        self.cancel_p3_detail('source_closed',restore=False)
        self.p3_driver=None
        self.cancel_parking_capture('source_closed')
        warning=StopClearanceController.source_closed(self)
        self.single_action.setEnabled(False);self.single_action.setChecked(False)
        if self.clean_action is not None:
            self.clean_action.setEnabled(False);self.clean_action.setChecked(False)
        if self.display_action is not None:self.display_action.setEnabled(False)
        if self.surface_action is not None:self.surface_action.setEnabled(False);self.surface_action.setChecked(False)
        for action in self.surface_actions.values():action.setEnabled(False);action.setChecked(False)
        self.contour_mode='baseline'
        self.motion_detail=False
        self.window_measurement=False
        if self.window_measurement_action is not None:
            self.window_measurement_action.setEnabled(False);self.window_measurement_action.setChecked(False)
        if self.measure_action is not None:self.measure_action.setEnabled(False)
        if self.pending_window_action is not None:
            self.pending_window_action.setEnabled(False);self.pending_window_action.setChecked(False)
        if self.motion_detail_action is not None:
            self.motion_detail_action.setEnabled(False);self.motion_detail_action.setChecked(False)
        if self.contour_action is not None:
            self.contour_action.setEnabled(False);self.contour_action.setChecked(False)
        if self.parking_action is not None:
            self.parking_action.setEnabled(False);self.parking_action.setChecked(False)
        if self.capture_action is not None:self.capture_action.setEnabled(False)
        self.parking_detail=False
        if self.detail_action is not None:
            self.detail_action.setEnabled(False);self.detail_action.setChecked(False)
        return warning

    def toggle(self,checked):
        if not checked:self.cancel_p3_detail('image_disabled',restore=False)
        parking=getattr(self.logic.app,'stopClearanceController',None)
        if checked and parking is not None and parking.overlay is not None:parking.toggle(False)
        # Do not call Stop.toggle: its cross-mode hook would target this very
        # image controller. The ordinary mode is disposed explicitly instead.
        normal=getattr(self.logic.app,'clearanceController',None)
        if checked and normal is not None and normal.overlay is not None:normal.toggle(False)
        try:
            from .clearance_live import LiveClearanceController
        except (ImportError,ValueError):
            from clearance_live import LiveClearanceController
        LiveClearanceController.toggle(self,checked)
        self.source_ready()

    def set_image_fusion(self,checked):
        """User-facing rolling entry; internal power/rebuild stays toggle()."""
        if checked and self.overlay is not None and self.contour_mode=='parking':
            self._switch_image_mode('baseline')
        else:
            if checked and self.overlay is None:self.contour_mode='baseline'
            self.toggle(checked)

    def single_view(self,checked):
        if self.overlay is not None:self.overlay.set_image_single_view(bool(checked))
        else:self.single_action.setChecked(False)

    def clean_view(self,checked):
        if self.overlay is None or not self.overlay.set_image_clean_view(bool(checked)):
            if self.clean_action is not None:self.clean_action.setChecked(False)

    def set_p3_detail(self,checked):
        if not checked:
            if self.overlay is not None:self.overlay.p3_start_failure=None
            self.cancel_p3_detail('user');self.source_ready();return
        if self.overlay is None or self.contour_mode!='baseline' or not self._available() or self.window_measurement:
            if self.p3_action is not None:self.p3_action.setChecked(False)
            return
        if self.p3_driver is not None and self.p3_driver.processing:return
        if self.p3_driver is not None:self.cancel_p3_detail('recompute')
        self.logic.stop();overlay=self.overlay
        overlay.p3_start_failure=None
        if getattr(overlay,'input_error',None):
            self._p3_start_failed(overlay,'selected frame not decoded')
            return
        try:
            driver=P3ReplayEnhancement(overlay.host,capture_clock)
            source=(overlay.session,overlay.clock,overlay.host.epoch)
            if not driver.begin(source):raise RuntimeError('worker busy or cancellation pending')
            self.p3_driver=driver;overlay.p3_driver=driver
            overlay.set_p3_receiver(driver.accepts,self._p3_result,True)
            self.logic.app.statusLabel.setText('  P3 DETAIL | freezing completed baseline | NO MEASUREMENT')
        except Exception as error:
            self._p3_start_failed(overlay,str(error)[:160])
            return
        self.source_ready()

    def _p3_start_failed(self,overlay,reason):
        overlay.p3_start_failure=dict(reason=reason,frame=getattr(overlay,'requested_frame',None),
            source=(overlay.session,overlay.clock,overlay.host.epoch))
        self.logic.app.statusLabel.setText('  P3 DETAIL | start failed | baseline held | '+reason+' | NO MEASUREMENT')
        print('P3 DETAIL start failed: %r'%reason)
        if self.p3_action is not None:self.p3_action.setChecked(False)

    def _p3_result(self,packet):
        if self.overlay is not None and self.p3_driver is not None and self.p3_driver.accepts(packet):
            shown=getattr(self.overlay,'last_result',None) or {}
            if (packet.get('command')=='measure' and packet.get('state')=='accepted'
                and shown.get('measurement_basis')=='rolling_20_window'
                and not shown.get('_qualified_window_measurements') and shown.get('pending_window_measurements')):
                # An explicit calculation must not silently finish with no
                # visible feedback. These labels remain raw, uncertain previews
                # and never enter the qualified list or accuracy claims.
                self.overlay.set_window_pending_view(True)
            self.source_ready()

    def set_surface_diagnostics(self,checked):
        if not checked:
            if self.p3_driver is not None and getattr(self.p3_driver,'command','enhance')=='surface':
                self.cancel_p3_detail('surface_disabled')
            self.source_ready();return
        if self.overlay is None or self.contour_mode!='baseline' or not self._available() or self.window_measurement:self.source_ready();return
        settings=self.overlay.get_image_display_settings() or {}
        if settings.get('roi') is None:
            self.display_options();settings=self.overlay.get_image_display_settings() or {}
        if settings.get('roi') is None:
            self.logic.app.statusLabel.setText('  SURFACE DIAG | set a local XYZ observation ROI first | NO MEASUREMENT')
            self.source_ready();return
        if self.p3_driver is not None and self.p3_driver.processing:self.source_ready();return
        self.logic.stop();self.cancel_p3_detail('surface_recompute');overlay=self.overlay
        try:
            if getattr(overlay,'input_error',None):raise ValueError('selected frame not decoded')
            driver=P3ReplayEnhancement(overlay.host,capture_clock,command='surface',options={'roi':settings['roi']})
            if not driver.begin((overlay.session,overlay.clock,overlay.host.epoch)):
                raise ValueError('worker busy or cancellation pending')
            self.p3_driver=driver;overlay.p3_driver=driver;overlay.p3_start_failure=None
            overlay.set_p3_receiver(driver.accepts,self._surface_result,True)
            self.logic.app.statusLabel.setText('  SURFACE DIAG | freezing original 20 scans | unchanged XYZ | NO MEASUREMENT')
        except Exception as error:
            self.logic.app.statusLabel.setText('  SURFACE DIAG | baseline held | '+str(error)[:160]+' | NO MEASUREMENT')
            print('SURFACE DIAG start failed: %r'%str(error)[:160])
        self.source_ready()

    def _surface_result(self,packet):
        if self.overlay is not None and packet.get('command')=='surface' and packet.get('state')=='accepted':
            self.overlay.set_image_surface_view('local')
        self.source_ready()

    def surface_view(self,mode):
        if self.overlay is None:return
        if mode=='layer':
            settings=self.overlay.get_image_display_settings() or {};roi=settings.get('roi')
            if roi is None:return
            try:from .image_display import layer_projection
            except (ImportError,ValueError):from image_display import layer_projection
            camera=self.overlay.renderer.GetActiveCamera()
            axis,offset=layer_projection(camera.GetPosition(),camera.GetFocalPoint())
            center=offset+sum(axis[i]*(roi[2*i]+roi[2*i+1])/2. for i in range(3))
            gui=self.logic.QtGui;dialog=gui.QDialog(self.logic.getMainWindow())
            dialog.setObjectName('imageSurfaceLayerDialog');dialog.setWindowTitle(u'表面诊断：观察深度层（仅隐藏，不修改坐标）')
            layout=gui.QFormLayout(dialog);middle=gui.QDoubleSpinBox(dialog);width=gui.QDoubleSpinBox(dialog)
            middle.setObjectName('imageSurfaceLayerCenter');middle.setRange(-20.,20.);middle.setDecimals(3);middle.setSingleStep(.02)
            middle.setValue(center);width.setObjectName('imageSurfaceLayerWidth');width.setRange(.002,20.)
            width.setDecimals(3);width.setSingleStep(.02);width.setValue(.08)
            layout.addRow(u'相对观察焦点的层中心（m）',middle);layout.addRow(u'层厚（m）',width)
            buttons=gui.QDialogButtonBox(gui.QDialogButtonBox.Ok|gui.QDialogButtonBox.Cancel,dialog)
            buttons.connect('accepted()',dialog.accept);buttons.connect('rejected()',dialog.reject);layout.addRow(buttons)
            opening=self.overlay
            try:
                if dialog.exec_() and self.overlay is opening and not getattr(opening,'closed',False):
                    opening.set_image_surface_view(mode,float(middle.value),float(width.value))
            finally:dialog.deleteLater()
        else:self.overlay.set_image_surface_view(mode)
        self.source_ready()

    def set_p3_baseline_view(self,checked):
        if self.overlay is not None:self.overlay.set_image_detail_view(not checked)

    def cancel_p3_detail(self,reason,restore=True):
        driver=self.p3_driver
        if driver is not None:driver.cancel(reason)
        if self.p3_action is not None:self.p3_action.setChecked(False)
        if self.p3_baseline_action is not None:
            self.p3_baseline_action.setChecked(False);self.p3_baseline_action.setEnabled(False)
        if self.surface_action is not None:self.surface_action.setChecked(False)
        if self.pending_window_action is not None:
            self.pending_window_action.setChecked(False);self.pending_window_action.setEnabled(False)
        for action in self.surface_actions.values():action.setChecked(False);action.setEnabled(False)
        overlay=self.overlay
        if overlay is not None:
            overlay.window_pending_view=False
            if restore and hasattr(overlay,'set_image_surface_view'):overlay.set_image_surface_view(None)
            if restore and hasattr(overlay,'set_image_detail_view'):overlay.set_image_detail_view(False)
            if hasattr(overlay,'set_p3_receiver'):overlay.set_p3_receiver(None,None,False)

    def set_contour_refinement(self,checked):
        chosen='quality' if checked else 'baseline'
        if self.overlay is None or not self._available():
            if self.contour_action is not None:self.contour_action.setChecked(False)
            return
        self._switch_image_mode(chosen)

    def _switch_image_mode(self,chosen,force=False,parking_detail=None,motion_detail=None,window_measurement=False):
        if chosen==self.contour_mode and not force:return
        self.cancel_p3_detail('mode_changed',restore=False)
        detail=(self.parking_detail if parking_detail is None else bool(parking_detail)) if chosen=='parking' else False
        moving_detail=(self.motion_detail if motion_detail is None else bool(motion_detail)) if chosen=='baseline' else False
        old=self.overlay;getter=getattr(old,'get_image_display_settings',None)
        settings=getter() if getter is not None else None
        single=bool(getattr(old,'image_single_view',False));clean=bool(getattr(old,'image_clean_view',False))
        view_getter=getattr(self.logic,'getRenderView',None);view=view_getter() if view_getter is not None else None
        camera={}
        vtk_camera=None;vtk_values=None
        if view is not None:
            for name in ('CameraPosition','CameraFocalPoint','CameraViewUp','CameraParallelScale'):
                if hasattr(view,name):
                    value=getattr(view,name);camera[name]=list(value) if name!='CameraParallelScale' else value
        main_view=getattr(self.logic.app,'mainView',None)
        if main_view is not None and hasattr(main_view,'GetRenderer'):
            vtk_camera=main_view.GetRenderer().GetActiveCamera()
            vtk_values=(tuple(vtk_camera.GetPosition()),tuple(vtk_camera.GetFocalPoint()),tuple(vtk_camera.GetViewUp()),
                vtk_camera.GetParallelScale(),vtk_camera.GetParallelProjection())
        self.toggle(False);self.contour_mode=chosen;self.parking_detail=detail;self.motion_detail=moving_detail
        self.window_measurement=chosen=='baseline' and bool(window_measurement);self.toggle(True)
        if self.overlay is not None:
            if settings is not None:
                self.overlay.set_image_display_settings(settings['depth'],settings['point_size'],settings['roi'])
            self.single_action.setChecked(single);self.single_view(single)
            if self.clean_action is not None:self.clean_action.setChecked(clean)
            self.clean_view(clean)
            if self.window_measurement:
                self.logic.app.statusLabel.setText('  20F WINDOW DISTANCE | rebuilding mature real window | NO DISTANCE | EXPERIMENTAL | NOT CURRENT SCAN')
            else:self.logic.app.statusLabel.setText('  IMAGE CONTOUR %s | rebuilding real scan window | NO MEASUREMENT'%chosen)
        for name,value in camera.items():setattr(view,name,value)
        if vtk_values is not None:
            vtk_camera=main_view.GetRenderer().GetActiveCamera()
            vtk_camera.SetPosition(*vtk_values[0]);vtk_camera.SetFocalPoint(*vtk_values[1]);vtk_camera.SetViewUp(*vtk_values[2])
            vtk_camera.SetParallelScale(vtk_values[3]);vtk_camera.SetParallelProjection(vtk_values[4])
        self.source_ready()

    def set_motion_detail(self,checked):
        """Real fitting/deskew policy, not the old frozen-window P3 display."""
        if self.overlay is None or self.contour_mode!='baseline' or not self._available() or self.window_measurement:
            if self.motion_detail_action is not None:self.motion_detail_action.setChecked(False)
            return
        if bool(checked)!=self.motion_detail:
            self._switch_image_mode('baseline',force=True,motion_detail=bool(checked))

    def set_window_measurement(self,checked):
        if self.overlay is None or not self._available():
            if self.window_measurement_action is not None:self.window_measurement_action.setChecked(False)
            return
        if self._can_measure_existing_window():
            if bool(checked)!=self.window_measurement:
                self.window_measurement=bool(checked)
                self.overlay.window_measurement_view=self.window_measurement
                self.cancel_p3_detail('window_view_changed')
            self.source_ready()
            packet=getattr(self.overlay,'last_result',None)
            if packet is not None:self.logic.app.statusLabel.setText(self.overlay.packet_status(packet))
            return
        if bool(checked)!=self.window_measurement or (checked and self.contour_mode!='baseline'):
            self._switch_image_mode('baseline',force=True,window_measurement=bool(checked),motion_detail=False)

    def calculate_window_measurement(self,*unused):
        if not self._can_measure_existing_window():self.source_ready();return
        if self.p3_driver is not None and self.p3_driver.processing:return
        # The calculation itself enters the annotation view, without restarting
        # the mature worker or submitting a replacement single scan.
        self.window_measurement=True;self.overlay.window_measurement_view=True
        self.logic.stop();self.cancel_p3_detail('window_recompute');overlay=self.overlay
        overlay.p3_start_failure=None
        try:
            if getattr(overlay,'input_error',None):raise ValueError('selected frame not decoded')
            driver=P3ReplayEnhancement(overlay.host,capture_clock,command='measure')
            if not driver.begin((overlay.session,overlay.clock,overlay.host.epoch)):
                raise ValueError('worker busy or cancellation pending')
            self.p3_driver=driver;overlay.p3_driver=driver
            overlay.set_p3_receiver(driver.accepts,self._p3_result,True)
            self.logic.app.statusLabel.setText('  20F WINDOW DISTANCE | freezing same completed window | EXPERIMENTAL | NOT CURRENT SCAN')
        except Exception as error:
            reason=str(error)[:160]
            overlay.p3_start_failure=dict(reason=reason,frame=getattr(overlay,'requested_frame',None),
                source=(overlay.session,overlay.clock,overlay.host.epoch),command='measure')
            self.logic.app.statusLabel.setText('  20F WINDOW DISTANCE | baseline held | '+reason+' | EXPERIMENTAL | NOT CURRENT SCAN | NO DISTANCE')
        self.source_ready()

    def set_window_pending(self,checked):
        if self.overlay is not None and self.window_measurement and self._available():
            self.overlay.set_window_pending_view(bool(checked))
        self.source_ready()

    def set_parking_detail(self,checked):
        if self.overlay is None or self.contour_mode!='parking' or not self._available():
            if self.detail_action is not None:self.detail_action.setChecked(False)
            return
        if bool(checked)!=self.parking_detail:
            self.logic.stop()
            self._switch_image_mode('parking',force=True,parking_detail=bool(checked))
            self._start_parking_capture(rebuild=False)

    def set_parking_capture(self,checked):
        if not self._available():
            if self.parking_action is not None:self.parking_action.setChecked(False)
            return
        already_parking=self.overlay is not None and self.contour_mode=='parking'
        if checked:self.logic.stop()
        if self.overlay is None:
            if checked:self.contour_mode='parking';self.toggle(True)
        elif checked:self._switch_image_mode('parking')
        elif self.contour_mode=='parking':self._switch_image_mode('baseline')
        self.source_ready()
        if checked and not already_parking:self._start_parking_capture(rebuild=False)

    def begin_parking_capture(self,*unused):
        return self._start_parking_capture(rebuild=True)

    def _start_parking_capture(self,rebuild):
        if self.overlay is None or self.contour_mode!='parking' or not self._available():return False
        self.logic.stop()
        try:
            frames=list(self.logic.getCurrentTimesteps())
            frame=int(self.logic.getTimeKeeper().getTime())
            # Validate before replacing a held pair. Automatic starts reuse
            # the freshly created worker; manual restarts explicitly rebuild.
            capture_clock()
            def select(chosen):
                if (self.parking_driver is None or not self.parking_driver.active
                    or self.overlay is not overlay or getattr(overlay,'closed',False)):return
                self.logic.app.scene.AnimationTime=float(chosen);self.logic.onTimeChanged()
            driver=ParkingReplayDriver(frames,select,clock=capture_clock)
            if frame not in driver.frame_ids:raise ValueError('selected real frame unavailable')
            if rebuild:self._switch_image_mode('parking',force=True)
            overlay=self.overlay
            if overlay is None:return False
            self.parking_driver=driver
            source=(overlay.session,overlay.clock,overlay.host.epoch)
            overlay.set_parking_receiver(driver.accepts,self._parking_result,driver.request_metadata,driver=driver)
            self.logic.app.statusLabel.setText('  PARK IMAGE | awaiting stability | %s | real scan capture | NO MEASUREMENT'%(
                'DETAIL ON' if self.parking_detail else 'BASELINE'))
            driver.start(uuid.uuid4().hex,frame,source)
            if not driver.active:
                self.logic.app.statusLabel.setText('  PARK IMAGE | start failed: %s | retry capture'%driver.reason)
            return driver.active
        except Exception as error:
            self.cancel_parking_capture('start_failed')
            self.logic.app.statusLabel.setText('  PARK IMAGE | start failed: %s | retry capture'%error)
            return False

    def _parking_result(self,packet):
        driver=self.parking_driver
        if driver is not None:driver.ack(packet)

    def cancel_parking_capture(self,reason):
        driver=self.parking_driver
        if driver is not None:driver.cancel(reason)
        self.parking_driver=None
        overlay=self.overlay
        if overlay is not None and hasattr(overlay,'set_parking_receiver'):
            overlay.set_parking_receiver(None,None,None)

    def display_options(self,*unused):
        if self.overlay is None:return
        opening_overlay=self.overlay
        settings=opening_overlay.get_image_display_settings()
        if settings is None:return
        gui=self.logic.QtGui;dialog=gui.QDialog(self.logic.getMainWindow())
        dialog.setObjectName('imageDisplayOptionsDialog')
        dialog.setWindowTitle(u'\u8865\u507f\u6210\u50cf\u663e\u793a\u8bbe\u7f6e\uff08\u4ec5\u663e\u793a\uff09')
        layout=gui.QVBoxLayout(dialog)
        help_text=gui.QLabel(u'\u989c\u8272\u8868\u793a\u5f53\u524d\u89c6\u89d2\u524d\u540e\u6df1\u5ea6\uff0c\u4e0d\u8868\u793a\u706f\u8272\u6216\u8bbe\u5907\u7c7b\u522b\u3002\n\u89c2\u5bdf\u6846\u662f\u5f53\u524d\u53c2\u8003XYZ\u5750\u6807\uff0c\u4e0d\u6539\u914d\u51c6\u8f93\u5165\u3002',dialog)
        layout.addWidget(help_text)
        depth=gui.QCheckBox(u'\u6309\u5f53\u524d\u89c6\u89d2\u6df1\u5ea6\u7740\u8272\uff08\u53d6\u6d88\u6062\u590d\u5355\u8272\uff09',dialog)
        depth.setObjectName('imageDepthColoring')
        depth.setChecked(settings['depth']);layout.addWidget(depth)
        form=gui.QFormLayout();size=gui.QDoubleSpinBox(dialog)
        size.setObjectName('imagePointSize')
        size.setRange(.5,5.);size.setSingleStep(.5);size.setValue(settings['point_size'])
        form.addRow(u'\u70b9\u5927\u5c0f\uff08\u50cf\u7d20\uff09',size);layout.addLayout(form)
        roi_check=gui.QCheckBox(u'\u542f\u7528\u5c40\u90e8\u89c2\u5bdf\u6846\uff08\u6846\u5916\u9690\u85cf\uff0c\u53ef\u6062\u590d\uff09',dialog)
        roi_check.setObjectName('imageRoiEnabled')
        roi_check.setChecked(settings['roi'] is not None);layout.addWidget(roi_check)
        bounds=settings['roi'] or settings.get('paired_bounds') or (-10.,10.,-10.,10.,-10.,10.)
        bound_form=gui.QFormLayout();spins=[]
        for i,label in enumerate(('X min (m)','X max (m)','Y min (m)','Y max (m)','Z min (m)','Z max (m)')):
            spin=gui.QDoubleSpinBox(dialog);spin.setRange(-10.,10.);spin.setDecimals(3)
            spin.setSingleStep(.1);spin.setValue(bounds[i]);spins.append(spin);bound_form.addRow(label,spin)
        layout.addLayout(bound_form)
        buttons=gui.QDialogButtonBox(gui.QDialogButtonBox.Ok|gui.QDialogButtonBox.Cancel,dialog)
        buttons.connect('accepted()',dialog.accept);buttons.connect('rejected()',dialog.reject);layout.addWidget(buttons)
        try:
            if dialog.exec_() and self.overlay is opening_overlay and not getattr(opening_overlay,'closed',False):
                try:
                    roi=tuple(float(spin.value) for spin in spins) if roi_check.checked else None
                    opening_overlay.set_image_display_settings(bool(depth.checked),float(size.value),roi)
                except ValueError as error:gui.QMessageBox.warning(dialog,'Invalid display settings',str(error))
        finally:
            dialog.deleteLater()


def install(logic,menu,window):
    existing=getattr(logic.app,'imageFusionController',None)
    if existing is not None:return existing
    action=logic.QtGui.QAction(u'20\u5e27\u5c40\u90e8\u5730\u56fe\u8865\u507f\uff08\u5386\u53f2\u56de\u653e\u6210\u50cf\uff09',window)
    single=logic.QtGui.QAction(u'\u8865\u507f\u5bf9\u7167\uff1a\u663e\u793a\u5355\u5e27',window)
    clean=logic.QtGui.QAction(u'\u8865\u507f\u6210\u50cf\uff1a\u5b64\u7acb\u6563\u70b9\u8f7b\u6ee4\u6ce2\uff08\u4ec5\u663e\u793a\uff09',window)
    display=logic.QtGui.QAction(u'\u8865\u507f\u6210\u50cf\uff1a\u663e\u793a\u8bbe\u7f6e\uff08\u989c\u8272\u002f\u70b9\u5927\u5c0f\u002f\u89c2\u5bdf\u6846\uff09',window)
    contour=logic.QtGui.QAction(u'补偿成像：运动轮廓精修（实验）',window)
    contour.setObjectName('actionImageContourRefinement');contour.setCheckable(True)
    contour.setToolTip('Experimental real-time endpoint + joint-window refinement; switch rebuilds session; baseline retained')
    moving_detail=logic.QtGui.QAction(u'20帧移动补偿：小设备配准优化（历史回放实验）',window)
    moving_detail.setObjectName('actionImageMotionDetail');moving_detail.setCheckable(True)
    moving_detail.setToolTip('Rolling baseline only; small-structure real fitting samples plus bounded translation deskew; switch rebuilds real window; no smoothing or filled points')
    window_measurement=logic.QtGui.QAction(u'20帧移动补偿＋窗口测距（历史回放实验）',window)
    window_measurement.setObjectName('actionImageWindowMeasurement');window_measurement.setCheckable(True)
    window_measurement.setToolTip('Mature rolling 20-scan compensation plus experimental distance from the full same frozen real window; not current-scan distance; accuracy and object identity unverified')
    measure=logic.QtGui.QAction(u'计算当前20帧窗口距离',window)
    measure.setObjectName('actionImageMeasureWindow')
    measure.setToolTip('Pauses playback and freezes this completed real 20-scan window; incomplete or unobservable windows retain the image baseline without distances')
    pending=logic.QtGui.QAction(u'窗口测距：显示待核验原始距离（仅预览）',window)
    pending.setObjectName('actionImageWindowPending');pending.setCheckable(True)
    pending.setToolTip('Optional uncertain raw minimum preview from unresolved annotation-quality candidates; preserves qualified distance metrics and all rail, ground and conflict exclusions')
    parking=logic.QtGui.QAction(u'停车补偿成像（历史回放实验）',window)
    capture=logic.QtGui.QAction(u'开始/重新停车采集',window)
    detail=logic.QtGui.QAction(u'停车补偿：真实点精细配准（实验）',window)
    detail.setObjectName('actionImageParkingDetail');detail.setCheckable(True)
    detail.setToolTip('Replay parking only; switching automatically restarts capture; real returns; no smoothing or filled points')
    p3=logic.QtGui.QAction(u'20帧补偿增强：小设备几何精修（历史回放实验）',window)
    baseline_view=logic.QtGui.QAction(u'增强对照：显示原20帧基线',window)
    surface=logic.QtGui.QAction(u'20帧补偿：小设备表面诊断／保真显示（历史回放）',window)
    surface.setObjectName('actionImageSurfaceDiagnostics');surface.setCheckable(True)
    surface.setToolTip('Same frozen mature baseline; real input, scan owners, depth layers and local 2mm real samples; no geometry edits')
    surface_menu=menu.addMenu(u'表面诊断对照（同一20帧窗口）');surface_actions={}
    for key,title in (('raw_single',u'① 单帧原始局部点'),('frames',u'② 原20帧：按扫描帧着色'),
                      ('layer',u'③ 深度分层：查看一层表面'),('local',u'④ 局部保真显示（真实代表点）'),
                      ('support',u'⑤ 跨帧支持显影（全部真实点）'),('stable',u'⑥ 稳定点子集（实验，可恢复全部点）'),
                      ('baseline',u'对照：原20帧显示代表点')):
        item=logic.QtGui.QAction(title,window);item.setObjectName('actionImageSurface_'+key)
        item.setCheckable(True);surface_menu.addAction(item);surface_actions[key]=item
        if key in ('support','stable'):
            item.setToolTip('Nearby distinct scans in 20mm; stable subset >=3 scans; weak evidence is NOT noise; no coordinate edits')
    p3.setObjectName('actionImageP3Detail');p3.setCheckable(True)
    baseline_view.setObjectName('actionImageP3BaselineView');baseline_view.setCheckable(True)
    p3.setToolTip('Freeze the completed 20-frame baseline; shadow rigid refinement; no reacquisition or measurement')
    parking.setObjectName('actionImageParkingCapture');parking.setCheckable(True)
    capture.setObjectName('actionImageParkingStart')
    parking.setToolTip('Selecting pauses normal playback and starts real scan capture; stability then up to 20 scans; 60 seconds; no measurement')
    display.setObjectName('actionImageDisplayOptions')
    action.setObjectName('actionImageFusion');single.setObjectName('actionImageSingleView')
    action.setCheckable(True);single.setCheckable(True)
    clean.setObjectName('actionImageCleanView');clean.setCheckable(True)
    action.setToolTip('20 real scans, selected-recording IMU, local map; 10m display; no measurement')
    single.setToolTip('Same snapshot, camera, 8mm sampling and point size; no recomputation')
    clean.setToolTip('Optional real-return subset; raw map retained; sparse isolated equipment returns may be hidden')
    controller=ImageFusionController(logic,action,single,clean,display,contour,parking,capture,detail,p3,baseline_view,surface,surface_actions,moving_detail,window_measurement,measure,pending)
    action.connect('triggered(bool)',controller.set_image_fusion);single.connect('triggered(bool)',controller.single_view)
    clean.connect('triggered(bool)',controller.clean_view)
    display.connect('triggered()',controller.display_options)
    contour.connect('triggered(bool)',controller.set_contour_refinement)
    moving_detail.connect('triggered(bool)',controller.set_motion_detail)
    window_measurement.connect('triggered(bool)',controller.set_window_measurement)
    measure.connect('triggered()',controller.calculate_window_measurement)
    pending.connect('triggered(bool)',controller.set_window_pending)
    parking.connect('triggered(bool)',controller.set_parking_capture)
    capture.connect('triggered()',controller.begin_parking_capture)
    detail.connect('triggered(bool)',controller.set_parking_detail)
    p3.connect('triggered(bool)',controller.set_p3_detail)
    baseline_view.connect('triggered(bool)',controller.set_p3_baseline_view)
    surface.connect('triggered(bool)',controller.set_surface_diagnostics)
    for key,item in surface_actions.items():
        item.connect('triggered(bool)',lambda checked,name=key:controller.surface_view(name))
        logic.app.actions['actionImageSurface_'+key]=item
    menu.addAction(action);menu.addAction(single);menu.addAction(clean);menu.addAction(display);menu.addAction(contour)
    menu.addAction(moving_detail);logic.app.actions['actionImageMotionDetail']=moving_detail
    menu.addAction(window_measurement);menu.addAction(measure);menu.addAction(pending)
    logic.app.actions['actionImageWindowMeasurement']=window_measurement
    logic.app.actions['actionImageMeasureWindow']=measure
    logic.app.actions['actionImageWindowPending']=pending
    logic.app.actions['actionImageFusion']=action;logic.app.actions['actionImageSingleView']=single
    logic.app.actions['actionImageCleanView']=clean
    logic.app.actions['actionImageDisplayOptions']=display
    logic.app.actions['actionImageContourRefinement']=contour
    menu.addAction(parking);menu.addAction(detail);menu.addAction(capture)
    logic.app.actions['actionImageParkingCapture']=parking;logic.app.actions['actionImageParkingStart']=capture
    logic.app.actions['actionImageParkingDetail']=detail
    menu.addAction(p3);menu.addAction(baseline_view)
    menu.addAction(surface);logic.app.actions['actionImageSurfaceDiagnostics']=surface
    logic.app.actions['actionImageP3Detail']=p3;logic.app.actions['actionImageP3BaselineView']=baseline_view
    logic.app.imageFusionController=controller
    return controller
