@echo off
REM Double-click helper: elevate (UAC prompt) and run setup.ps1, the WinUSB
REM driver installer for the Kinect. The script ships Authenticode-signed in
REM release ZIPs, so RemoteSigned VERIFIES it (local unsigned builds still
REM run: no Mark-of-the-Web, no signature required).
REM Equivalent to the demo's "Install Kinect drivers" banner button.
powershell -NoProfile -Command "Start-Process powershell -Verb RunAs -ArgumentList '-NoProfile -ExecutionPolicy RemoteSigned -File \"\"%~dp0setup.ps1\"\"'"
