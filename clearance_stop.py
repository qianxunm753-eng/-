# -*- coding: utf-8 -*-
"""Separate parking replay menu; never stops motors or enables live experiments."""
try:
    from .clearance_live import LiveClearanceController
    from .clearance_native import enable_recorded_stop
except (ImportError,ValueError):
    from clearance_live import LiveClearanceController
    from clearance_native import enable_recorded_stop


class StopClearanceController(LiveClearanceController):
    def _available(self):
        return (self.logic.app.curLidarType=='RSAIRY' and self.logic.getSensor() is None and self._pcap_reader() is not None)

    def _restore(self):
        # Normal controller owns legacy rail restoration. Do not create another
        # overlay while installing or refreshing this independent action.
        return None

    def _enable_recorded(self):
        return enable_recorded_stop(self.logic)

    def toggle(self,checked):
        image=getattr(self.logic.app,'imageFusionController',None)
        if checked and image is not None and image.overlay is not None:image.toggle(False)
        normal=getattr(self.logic.app,'clearanceController',None)
        if checked and normal is not None and normal.overlay is not None:
            normal.toggle(False)
        return LiveClearanceController.toggle(self,checked)


def install(logic,menu,window):
    existing=getattr(logic.app,'stopClearanceController',None)
    if existing is not None:return existing
    action=logic.QtGui.QAction(u'\u505c\u8f66\u878d\u5408\u6d4b\u8ddd\uff08\u5386\u53f2\u56de\u653e\uff09',window)
    action.setObjectName('actionStopWindowClearance')
    action.setCheckable(True)
    action.setToolTip('Parking replay: 10m, 20/40/80/160 independent scans, 60s budget; keep playback running through real stationary data')
    controller=StopClearanceController(logic,action)
    action.connect('triggered(bool)',controller.toggle)
    menu.addAction(action)
    logic.app.actions['actionStopWindowClearance']=action
    logic.app.stopClearanceController=controller
    return controller
