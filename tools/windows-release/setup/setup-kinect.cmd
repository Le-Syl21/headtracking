@echo off
REM Double-click helper: elevate (UAC prompt) and run setup.ps1, the WinUSB
REM driver installer for the Kinect, bypassing the execution policy that
REM blocks unsigned local scripts on a fresh Windows.
REM Equivalent to the demo's "Install Kinect drivers" banner button.
powershell -NoProfile -ExecutionPolicy Bypass -Command "Start-Process powershell -Verb RunAs -ArgumentList '-NoProfile -ExecutionPolicy Bypass -File \"\"%~dp0setup.ps1\"\"'"
