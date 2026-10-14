@echo off
setlocal
cd /d "%~dp0"
set RSVIEW_SIM_LOOPS=0
set RSVIEW_SIM_AUTO_EXIT=0
set RSVIEW_SIM_YIELD_MS=0
set "RSVIEW_SIM_PCAP="
set "RSVIEW_SIM_FRAMES=31"
set "RSVIEW_SIM_VIEW_AXIS=-Y"
if not "%~1"=="" (
  set "RSVIEW_SIM_PCAP=%~f1"
  set "RSVIEW_SIM_FRAMES=0"
  set "RSVIEW_SIM_VIEW_AXIS=-X"
)
start "" "C:\Users\mqx\Downloads\20251013145653_88801\RSView_Win_4.3.15_0514\bin\RSView.exe" --script=simulate_live.py
endlocal
