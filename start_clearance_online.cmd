@echo off
setlocal
cd /d "%~dp0"
set "RSVIEW_CLEARANCE_VALIDATE="
set "RSVIEW_CLEARANCE_REPORT="
set "RSVIEW_CLEARANCE_COMPUTE=online"
set "RSVIEW_CLEARANCE_FRAME=260"
start "" "C:\Users\mqx\Downloads\20251013145653_88801\RSView_Win_4.3.15_0514\bin\RSView.exe" --script=start_clearance_playback.py
endlocal
