# -*- coding: utf-8 -*-
"""Python-2-compatible opt-in native Airy live/recorded measurement menu."""
import os

try:
    from .clearance_native import enable_live, enable_recorded
except (ImportError, ValueError):
    from clearance_native import enable_live, enable_recorded


def restore_live_rail(logic):
    """Return to the pre-existing gauge overlay, without reconnecting the lidar."""
    overlay = logic.rail_live.LiveRailOverlay(logic.smp, logic.vtk,
        logic.app.mainView, logic.app.statusLabel,
        os.path.join(os.path.dirname(__file__), 'history'))
    logic.app.railOverlay = overlay
    return overlay


def restore_pcap_rail(logic):
    """Restore the existing filename-specific route; ordinary PCAP stays raw."""
    reader, filename = logic.getReader(), logic.getReaderFileName()
    rail = getattr(logic, 'rail_centerline', None)
    clearance = getattr(logic, 'clearance_overlay', None)
    overlay = None
    if rail is not None and rail.is_target_file(filename):
        overlay = rail.VtkRailOverlay(logic.smp, logic.vtk, logic.app.mainView,
            logic.app.statusLabel, rail.PcapPoseIndex.from_file(filename))
    elif clearance is not None and clearance.is_target_file(filename):
        overlay = clearance.make_overlay(logic.smp, logic.vtk, logic.app.mainView,
            logic.app.statusLabel, filename, reader)
    logic.app.railOverlay = overlay
    return overlay


def _spin_value(spin, value):
    """UI bookkeeping must not emit a decoder-changing TF callback."""
    block = getattr(spin, 'blockSignals', None)
    previous = block(True) if block is not None else None
    try:
        spin.setValue(value)
    finally:
        if block is not None:
            block(previous)


class LiveClearanceController(object):
    def __init__(self, logic, action):
        self.logic, self.action = logic, action
        self.overlay = None
        self.saved_trailing = None
        self.last_cleanup_warning = None
        self.source_ready()

    def _available(self):
        return (self.logic.app.curLidarType == 'RSAIRY' and
                (self.logic.getSensor() is not None or self._pcap_reader() is not None))

    def _pcap_reader(self):
        getter = getattr(self.logic, 'getReader', None)
        reader = getter() if getter is not None else None
        if reader is not None and self.logic.getReaderFileName().lower().endswith('.pcap'):
            return reader
        return None

    def source_ready(self):
        try:
            self._restore()
        except Exception as error:
            self.logic.app.statusLabel.setText('  Live rail restore failed: %s' % error)
        self.action.setEnabled(self._available())
        self.action.setChecked(self.overlay is not None)
        for companion in self._companions():companion.source_ready()

    def _companions(self):
        # The ordinary root owns mode/source lifecycle. Child controllers must
        # never recursively close or refresh each other.
        if type(self) is not LiveClearanceController:return ()
        return tuple(controller for controller in (getattr(self.logic.app,'stopClearanceController',None),
            getattr(self.logic.app,'imageFusionController',None)) if controller is not None)

    def _restore(self):
        if (self._available() and self.logic.getSensor() is not None and
                self.logic.app.railOverlay is None):
            restore_live_rail(self.logic)
        elif (self._available() and self._pcap_reader() is not None and
                self.logic.app.railOverlay is None):
            restore_pcap_rail(self.logic)

    def _save_trailing(self):
        app = self.logic.app
        spin = getattr(app, 'trailingFramesSpinBox', None)
        source = self.logic.getSensor() or self._pcap_reader()
        recorded = self.logic.getSensor() is None
        native = source.GetClientSideObject()
        getter = getattr(native, 'GetNumberOfTrailingFrames', None)
        proxy_value = getattr(source, 'NumberOfTrailingFrames', None) if recorded else None
        if recorded and not callable(getter) and (proxy_value is None or int(proxy_value) != app.numberOfTrailingFrames):
            raise ValueError('PCAP trailing setting is unverified; reopen the recording')
        native_value = int(getter()) if callable(getter) else int(proxy_value) if proxy_value is not None else None
        if recorded and native_value is None:
            raise ValueError('PCAP single-scan setting cannot be read; reconnect the recording')
        self.saved_trailing = dict(app=app.numberOfTrailingFrames,
            ui=int(spin.value) if spin is not None else None,
            enabled=bool(spin.enabled) if spin is not None else None,
            native=native_value, source=source, property=proxy_value)
        # This native build has only a setter. Never guess a hidden setting
        # from the UI: normal Airy connection already starts in single-scan mode.
        if self.saved_trailing['native'] is not None:
            if proxy_value is not None:
                source.NumberOfTrailingFrames = 0
            native.SetNumberOfTrailingFrames(0)
            if recorded:
                modified = getattr(native, 'Modified', None)
                if modified is not None:
                    modified()
            app.numberOfTrailingFrames = 0
            if spin is not None:
                _spin_value(spin, 0)
        if spin is not None:
            spin.setEnabled(False)

    def _restore_trailing(self):
        saved, self.saved_trailing = self.saved_trailing, None
        if saved is None:
            return
        app = self.logic.app
        spin = getattr(app, 'trailingFramesSpinBox', None)
        try:
            source = self.logic.getSensor() or self._pcap_reader()
            if source is saved['source'] and saved['native'] is not None:
                if saved['property'] is not None:
                    source.NumberOfTrailingFrames = saved['property']
                source.GetClientSideObject().SetNumberOfTrailingFrames(saved['native'])
                if self.logic.getSensor() is None:
                    modified = getattr(source.GetClientSideObject(), 'Modified', None)
                    if modified is not None:
                        modified()
        finally:
            if spin is not None:
                _spin_value(spin, saved['ui'])
                spin.setEnabled(saved['enabled'])
            app.numberOfTrailingFrames = saved['app']

    def toggle(self, checked):
        app = self.logic.app
        if checked:
            for companion in self._companions():
                if companion.overlay is not None:companion.toggle(False)
        if not checked:
            warning = self.source_closed()
            try:
                self._restore()
            except Exception as error:
                app.statusLabel.setText('  Live rail restore failed: %s' % error)
            self.source_ready()
            if self.logic.getSensor() is None and self._pcap_reader() is not None:
                callback = getattr(self.logic, 'onTimeChanged', None)
                if app.railOverlay is not None and callback is not None:
                    callback()
                elif app.railOverlay is None:
                    app.statusLabel.setText('  PCAP REPLAY | measurement disabled')
                smp = getattr(self.logic, 'smp', None)
                if smp is not None:
                    smp.Render(app.mainView)
            if warning:
                app.statusLabel.setText(warning)
            return
        if not self._available():
            self.action.setChecked(False)
            self.source_ready()
            return
        if self.overlay is not None:
            return
        try:
            self._save_trailing()
            if self.logic.getSensor() is not None:
                self.overlay = enable_live(self.logic)
                self.overlay.set_stream_running(app.playing)
            else:
                self.overlay = self._enable_recorded()
                # A paused selected frame is valid historical input, not a
                # disconnected live stream. Submit without starting playback.
                callback = getattr(self.logic, 'onTimeChanged', None)
                if callback is not None:
                    callback()
            self.action.setChecked(True)
        except Exception as error:
            try:
                self.source_closed()
            except Exception as cleanup_error:
                error = '%s; cleanup failed: %s' % (error, cleanup_error)
            try:
                self._restore()
            except Exception:
                pass
            self.action.setEnabled(self._available())
            app.statusLabel.setText('  Live clearance setup failed: %s' % error)

    def stream_changed(self, running):
        if self.overlay is not None:
            self.overlay.set_stream_running(running)

    def _enable_recorded(self):
        return enable_recorded(self.logic)

    def application_quit(self):
        """Clean while Python/Qt are alive, then release the application slot.

        This legacy PythonQt build can crash in python27.dll when QApplication
        retains our bound callback into interpreter teardown. Ordinary file
        close must keep the quit hook; only actual application quit removes it.
        """
        try:
            return self.source_closed()
        finally:
            self.logic.QtGui.QApplication.instance().disconnect('aboutToQuit()',self.application_quit)

    def source_closed(self):
        parking_warning=None
        for companion in self._companions():
            warning=companion.source_closed()
            if warning:parking_warning=warning
        overlay, self.overlay = self.overlay, None
        errors = []
        try:
            if overlay is not None:
                overlay.close()
        except Exception as error:
            errors.append(str(error))
        finally:
            if overlay is not None and self.logic.app.railOverlay is overlay:
                self.logic.app.railOverlay = None
            try:
                self._restore_trailing()
            except Exception as error:
                errors.append(str(error))
            finally:
                self.action.setChecked(False)
                self.action.setEnabled(False)
        if errors:
            warning = '  Live clearance cleanup warning: '+ '; '.join(errors)
            self.last_cleanup_warning = warning
            self.logic.app.statusLabel.setText(warning)
            return warning
        return parking_warning


def install(logic):
    """Register once after native actions are available; never auto-enable."""
    if getattr(logic.app, 'clearanceController', None) is not None:
        return logic.app.clearanceController
    window = logic.getMainWindow()
    menu = window.findChild('QMenu', 'menuTools')
    if menu is None:
        menu = window.menuBar().addMenu('Trackside')
    action = logic.QtGui.QAction(
        u'\u8f68\u65c1\u6d4b\u8ddd\uff08\u5b9e\u65f6/\u56de\u653e\u6d4b\u8bd5\uff09', window)
    action.setObjectName('actionLiveTracksideClearance')
    action.setCheckable(True)
    action.setToolTip('Airy live/PCAP distances within 5m, same current-cloud algorithm; relative LiDAR only, no GPS/INS accuracy validation')
    controller = LiveClearanceController(logic, action)
    action.connect('triggered(bool)', controller.toggle)
    menu.addAction(action)
    logic.app.actions['actionLiveTracksideClearance'] = action
    logic.app.clearanceController = controller
    try:
        from .clearance_stop import install as install_stop
    except (ImportError,ValueError):
        from clearance_stop import install as install_stop
    install_stop(logic,menu,window)
    try:
        from .fusion_native import install as install_image
    except (ImportError,ValueError):
        from fusion_native import install as install_image
    install_image(logic,menu,window)
    logic.QtGui.QApplication.instance().connect('aboutToQuit()', controller.application_quit)
    return controller
