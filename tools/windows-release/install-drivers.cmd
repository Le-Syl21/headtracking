@echo off
REM ----------------------------------------------------------------------
REM  HeadTracking — UsbDk filter driver installer
REM ----------------------------------------------------------------------
REM
REM  Double-click this file to install the UsbDk filter driver.
REM  Windows will prompt for admin rights (UAC) — UsbDk is a kernel
REM  driver, this is unavoidable.
REM
REM  Why: by default, libusb on Windows can only talk to devices whose
REM  function driver is WinUSB / libusbK. Kinect v1 / v2 typically end
REM  up bound to the Microsoft Kinect SDK driver (or to no driver),
REM  neither of which libusb can drive. UsbDk is a *filter* driver
REM  signed by Daynix that slots above whatever's currently loaded and
REM  exposes a libusb-compatible interface — solving both cases without
REM  Zadig and without uninstalling the SDK.
REM
REM  After install, plug the Kinect, restart VPX, and the plugin should
REM  enumerate it.
REM
REM  Source: https://github.com/daynix/UsbDk
REM  License: Apache-2.0 (compatible with this plugin's GPL-3.0)
REM ----------------------------------------------------------------------

setlocal

REM Resolve to the MSI sitting next to this script, regardless of the
REM working directory the user double-clicked from.
set MSI=%~dp0UsbDk_1.0.22_x64.msi

if not exist "%MSI%" (
    echo [ERROR] UsbDk MSI not found at:
    echo         %MSI%
    echo.
    echo This script must live in the same folder as UsbDk_1.0.22_x64.msi.
    echo The HeadTracking release ZIP ships them together under
    echo     plugins\headtracking\drivers\
    echo.
    pause
    exit /b 1
)

echo Installing UsbDk from:
echo   %MSI%
echo.
echo Windows will now prompt for admin rights.
echo.

REM /qb = basic UI (progress bar only, no wizard); /norestart so we
REM don't reboot silently if the kernel asks for it on a busy system.
msiexec /i "%MSI%" /qb /norestart
set RC=%ERRORLEVEL%

if %RC% == 0 (
    echo.
    echo UsbDk installed successfully. Restart VPX to pick up the change.
) else if %RC% == 3010 (
    echo.
    echo UsbDk installed, but Windows requests a reboot before it loads.
    echo Reboot the machine, then start VPX.
) else if %RC% == 1602 (
    echo.
    echo Install cancelled by the user.
) else if %RC% == 1603 (
    echo.
    echo [ERROR] msiexec returned 1603 (fatal install error).
    echo Open an elevated PowerShell and run:
    echo     msiexec /i "%MSI%" /l*v "%TEMP%\UsbDk_install.log"
    echo Then attach the log when reporting the issue.
) else (
    echo.
    echo [ERROR] msiexec exit code %RC% — see Windows Event Viewer or
    echo         retry with /l*v as documented above.
)

echo.
pause
endlocal
exit /b %RC%
