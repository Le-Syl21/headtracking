@echo off
REM ----------------------------------------------------------------------
REM  HeadTracking — Kinect Windows setup launcher
REM ----------------------------------------------------------------------
REM
REM  Double-click this file. Windows will prompt for admin rights (UAC),
REM  then run cleanup-kinect.ps1 in the same directory. The PowerShell
REM  script:
REM    1. Downloads + installs UsbDk if it isn't already there
REM       (covers the Kinect v2 path entirely);
REM    2. Blocks Windows PnP from auto-installing partial drivers on
REM       the Kinect VID/PIDs (so usbaudio.sys won't re-grab the v1
REM       Audio interface between Zadig runs);
REM    3. Unbinds any current Kinect device instance from the system
REM       so PnP re-discovers it cleanly on next plug;
REM    4. Surfaces leftover Kinect-related OEM drivers in the
REM       Driver Store for optional manual deletion.
REM
REM  After it's done: replug the Kinect. Kinect v2 just works. For
REM  Kinect v1, run Zadig once to bind WinUSB to the three Xbox NUI
REM  sub-devices.
REM
REM  Why a .cmd wrapper at all: PowerShell scripts can't be double-
REM  clicked by default (their .ps1 association opens Notepad). This
REM  .cmd self-elevates and runs the .ps1 with -ExecutionPolicy Bypass
REM  so it works on a fresh Windows install with default settings.
REM ----------------------------------------------------------------------

setlocal

set SCRIPT_DIR=%~dp0
set PS1=%SCRIPT_DIR%cleanup-kinect.ps1

if not exist "%PS1%" (
    echo [ERROR] cleanup-kinect.ps1 not found next to this launcher:
    echo         %PS1%
    pause
    exit /b 1
)

REM Are we already elevated?
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo Re-launching with administrator rights...
    powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
    exit /b
)

REM Run the PS1 in this elevated console.
powershell -NoProfile -ExecutionPolicy Bypass -File "%PS1%"

endlocal
