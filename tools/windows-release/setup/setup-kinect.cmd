@echo off
REM ----------------------------------------------------------------------
REM  HeadTracking - Kinect Windows setup launcher
REM ----------------------------------------------------------------------
REM
REM  Double-click this file. Windows will prompt for admin rights (UAC),
REM  then run dont_run.ps1 in the same directory. The PowerShell script:
REM
REM    1. Denies Windows from auto-installing partial drivers on the
REM       Kinect v1 sub-device VID/PIDs (so usbaudio.sys can't re-grab
REM       the v1 Audio interface anymore).
REM    2. Same for Kinect v2 sensor VID/PIDs (so Windows Update can't
REM       silently re-bind a Microsoft driver over time).
REM    3. Removes any currently-attached Kinect device instances AND
REM       deletes any pre-existing Kinect drivers from the Driver
REM       Store (legacy Microsoft Kinect SDK, leftover Zadig output,
REM       etc.) so our INFs are the only candidates.
REM    4. Installs the bundled WinUSB INF packages from .\drivers\
REM       via pnputil /add-driver, covering all known Kinect v1 + v2
REM       hardware revisions.
REM    5. Triggers a PnP rescan so already-plugged Kinects get
REM       re-bound to WinUSB without the user having to physically
REM       unplug and replug the device (useful for hard-mounted
REM       Kinects in pinball cabinets).
REM
REM  After it's done, restart VPX / headtracking-demo and tracking
REM  should be live. No Zadig, no manual Device Manager step.
REM
REM  Why a .cmd wrapper at all: PowerShell scripts can't be double-
REM  clicked by default (their .ps1 association opens Notepad). This
REM  .cmd self-elevates and runs dont_run.ps1 with -ExecutionPolicy
REM  Bypass so it works on a fresh Windows install with default
REM  settings. The companion script is named "dont_run.ps1" so a user
REM  poking around the folder doesn't try to launch it directly -
REM  always launch THIS file (setup-kinect.cmd) instead.
REM ----------------------------------------------------------------------

setlocal

set SCRIPT_DIR=%~dp0
set PS1=%SCRIPT_DIR%dont_run.ps1

if not exist "%PS1%" (
    echo [ERROR] dont_run.ps1 not found next to this launcher:
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
