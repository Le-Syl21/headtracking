# --------------------------------------------------------------------
# HeadTracking — Kinect Windows setup
# --------------------------------------------------------------------
#
# Run as administrator. Performs four things:
#
#   1. Installs UsbDk (Daynix's WHQL-signed filter driver) if it's
#      not already loaded. v2's libfreenect2 opts into UsbDk at
#      `libusb_init` time, so this is what makes the Kinect v2 work
#      on Windows out of the box.
#
#   2. Adds registry entries under
#      HKLM\SOFTWARE\Policies\Microsoft\Windows\DeviceInstall\Restrictions
#      → "DenyDeviceIDs" so Windows PnP refuses to auto-install any
#      driver for the Kinect v1 (Audio / Camera / Motor) and v2
#      (Sensor) USB IDs. This survives reboots and Windows Updates,
#      and it's the only way to stop Windows from re-attaching the
#      partial `usbaudio.sys` to the Kinect v1 Audio interface
#      between Zadig runs.
#
#   3. Removes any currently-attached Kinect device instance from
#      the system (forcing PnP to re-discover them next plug, with
#      the new deny rules applying).
#
#   4. Lists Kinect-related OEM drivers still in the Driver Store
#      so you can optionally `pnputil /delete-driver oem<n>.inf
#      /uninstall /force` to nuke them too.
#
# After this script runs:
#   * Kinect v2: just replug. UsbDk is on, libfreenect2 will pick
#     it up. Done.
#   * Kinect v1: replug, then run Zadig
#     (https://zadig.akeo.ie/) → Options → List All Devices (on),
#     Ignore Hubs or Composite Parents (off) → for each Xbox NUI
#     <Audio|Camera|Motor> pick WinUSB on the right, Replace Driver.
#     WinUSB is preferred over libusbK on Windows 11 with Memory
#     Integrity / HVCI enabled (inbox Microsoft, signed at the
#     kernel level — works regardless of HVCI policy).
#
# Why WinUSB rather than UsbDk for v1: UsbDk is a filter driver
# that attaches above whatever function driver claims the device.
# The Kinect v1 Camera and Motor sub-devices have NO function
# driver after Windows' default partial install (only usbaudio.sys
# binds to the audio interface), so UsbDk has nothing to filter
# and libusb falls through. Replacing each sub-device's driver
# with WinUSB via Zadig gives libusb a real claim path.
#
# Re-running this script is idempotent: UsbDk is skipped if
# already installed, registry entries are overwritten with the
# same content, missing devices are silently skipped.
# --------------------------------------------------------------------

#Requires -RunAsAdministrator

$ErrorActionPreference = 'Stop'

Write-Host "================================================================" -ForegroundColor Cyan
Write-Host " HeadTracking - Kinect Windows setup" -ForegroundColor Cyan
Write-Host "================================================================" -ForegroundColor Cyan

# --------------------------------------------------------------------
# 1. Install UsbDk if it isn't already loaded.
# --------------------------------------------------------------------

Write-Host ""
Write-Host "[1/4] Checking UsbDk filter driver..." -ForegroundColor Yellow

function Test-UsbDkInstalled {
    # UsbDk's user-mode helper creates a `\\.\UsbDk` device symlink at
    # service start. If we can open it (or get ACCESS_DENIED, meaning
    # it's there but we lack the right ACL bits), the driver is loaded.
    # The fallback `Get-Service UsbDk` works even when the kernel
    # device isn't bound (e.g. service registered but not started).
    try {
        $svc = Get-Service -Name 'UsbDk' -ErrorAction Stop
        return $svc.Status -ne 'Stopped' -or $svc.StartType -ne 'Disabled'
    } catch {
        return $false
    }
}

if (Test-UsbDkInstalled) {
    Write-Host "  UsbDk is already installed. Skipping download." -ForegroundColor Green
} else {
    $usbdkUrl = 'https://github.com/daynix/UsbDk/releases/download/v1.00-22/UsbDk_1.0.22_x64.msi'
    $usbdkMsi = Join-Path $env:TEMP 'UsbDk_1.0.22_x64.msi'

    Write-Host "  UsbDk not detected. Downloading from Daynix..."
    Write-Host "    URL : $usbdkUrl"
    Write-Host "    Dest: $usbdkMsi"

    try {
        # `-UseBasicParsing` works even on early WinPE / Server Core
        # where the IE engine isn't initialised. TLS 1.2 is the
        # minimum GitHub Releases accept; force it on PS 5.x.
        [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
        Invoke-WebRequest -Uri $usbdkUrl -OutFile $usbdkMsi -UseBasicParsing
    } catch {
        Write-Host "  [ERROR] Download failed: $_" -ForegroundColor Red
        Write-Host "  Get UsbDk manually at https://github.com/daynix/UsbDk/releases" -ForegroundColor Red
        exit 1
    }

    # Sanity: a 404 served as 200 would land here as a tiny HTML page.
    # The real MSI is ~3.5 MB; under 1 MB is wrong.
    $size = (Get-Item $usbdkMsi).Length
    if ($size -lt 1MB) {
        Write-Host "  [ERROR] Downloaded file is too small ($size bytes), refusing to run." -ForegroundColor Red
        Remove-Item $usbdkMsi -ErrorAction SilentlyContinue
        exit 1
    }

    Write-Host "  Installing $($size) bytes via msiexec..."
    # /qb = basic UI (progress bar, no wizard); /norestart so we don't
    # reboot in the middle of this script if Windows asks.
    $proc = Start-Process -FilePath 'msiexec.exe' -ArgumentList "/i `"$usbdkMsi`" /qb /norestart" -Wait -PassThru
    switch ($proc.ExitCode) {
        0     { Write-Host "  UsbDk installed successfully." -ForegroundColor Green }
        3010  { Write-Host "  UsbDk installed; Windows requested a reboot." -ForegroundColor Yellow }
        1602  { Write-Host "  Install cancelled by user." -ForegroundColor Yellow }
        1603  {
            Write-Host "  [ERROR] msiexec exit 1603 (fatal install error)." -ForegroundColor Red
            Write-Host "    For a verbose log, re-run manually:" -ForegroundColor Red
            Write-Host "      msiexec /i `"$usbdkMsi`" /l*v `"$env:TEMP\UsbDk_install.log`"" -ForegroundColor Red
            exit 1
        }
        default {
            Write-Host "  [ERROR] msiexec exit $($proc.ExitCode). Aborting." -ForegroundColor Red
            exit 1
        }
    }
}

# --------------------------------------------------------------------
# 2. Block future PnP auto-install for these VID/PID combos.
# --------------------------------------------------------------------

Write-Host ""
Write-Host "[2/4] Configuring DenyDeviceIDs (registry)..." -ForegroundColor Yellow

$restrictionsKey = 'HKLM:\SOFTWARE\Policies\Microsoft\Windows\DeviceInstall\Restrictions'
$denyKey = "$restrictionsKey\DenyDeviceIDs"

New-Item -Path $restrictionsKey -Force | Out-Null
Set-ItemProperty -Path $restrictionsKey -Name 'DenyDeviceIDs' -Type DWord -Value 1
# Retroactive=0: don't yank already-bound drivers from running devices.
# We unbind those manually in step 3 instead.
Set-ItemProperty -Path $restrictionsKey -Name 'DenyDeviceIDsRetroactive' -Type DWord -Value 0
# AllowAdminInstall=1: by default, DenyDeviceIDs blocks ALL installs
# including admin-initiated ones (Zadig, Device Manager, pnputil…).
# We need admins to be able to override the deny so Zadig can bind
# WinUSB on the v1 sub-devices afterwards. This flag is exactly the
# Group Policy "Allow administrators to override Device Installation
# Restriction policies" toggle.
Set-ItemProperty -Path $restrictionsKey -Name 'AllowAdminInstall' -Type DWord -Value 1

New-Item -Path $denyKey -Force | Out-Null

# Kinect v1 sub-devices ONLY. The deny list is here to stop Windows
# PnP from auto-installing partial drivers (especially `usbaudio.sys`
# on the audio interface) that would later get in the way of Zadig's
# WinUSB binding. AllowAdminInstall=1 above lets Zadig override the
# deny when an admin runs it.
#
# We deliberately do NOT block the Kinect v2 PIDs (02C4 / 02D8 / 02D9):
# v2 needs Windows to attach SOME function driver (the Microsoft
# generic one is fine) so UsbDk can filter on top of it. Without a
# function driver, UsbDk's class filter has nothing to attach to and
# libfreenect2 fails. Earlier versions of this script blocked v2 too
# and made the Kinect v2 disappear from Device Manager entirely.
$kinectIds = @(
    'USB\VID_045E&PID_02AD',  # Xbox NUI Audio (v1, 1414 rev)
    'USB\VID_045E&PID_02AE',  # Xbox NUI Camera (v1, 1414 rev)
    'USB\VID_045E&PID_02B0',  # Xbox NUI Motor  (v1, 1414 rev)
    'USB\VID_045E&PID_02BB',  # Xbox NUI Audio (v1, 1473 rev)
    'USB\VID_045E&PID_02BE',  # Kinect for Windows v1 motor variant
    'USB\VID_045E&PID_02BF',  # Xbox NUI Camera (v1, 1473 rev)
    'USB\VID_045E&PID_02C2'   # Kinect for Windows v1 variant
)

# Wipe the existing list first so re-runs don't accumulate stale
# entries with diverged numbering.
Get-ItemProperty -Path $denyKey | Get-Member -MemberType NoteProperty | ForEach-Object {
    if ($_.Name -match '^\d+$') {
        Remove-ItemProperty -Path $denyKey -Name $_.Name -ErrorAction SilentlyContinue
    }
}

$index = 1
foreach ($id in $kinectIds) {
    Set-ItemProperty -Path $denyKey -Name "$index" -Type String -Value $id
    Write-Host "  + $id"
    $index++
}

Write-Host "  Registry updated. Reboot is NOT required for new plugs." -ForegroundColor Green

# --------------------------------------------------------------------
# 2. Remove currently-attached Kinect device instances.
# --------------------------------------------------------------------

Write-Host ""
Write-Host "[3/4] Removing currently-attached Kinect devices..." -ForegroundColor Yellow

$kinectPids = @('02AD', '02AE', '02B0', '02BB', '02BE', '02BF', '02C2', '02C4', '02D8', '02D9')

$devices = Get-PnpDevice -PresentOnly:$false -ErrorAction SilentlyContinue | Where-Object {
    $_.InstanceId -match 'USB\\VID_045E&PID_([0-9A-Fa-f]{4})' -and
    $kinectPids -contains $matches[1].ToUpper()
}

if ($devices.Count -eq 0) {
    Write-Host "  No Kinect device currently registered. Nothing to remove." -ForegroundColor Gray
} else {
    foreach ($d in $devices) {
        Write-Host "  Removing: $($d.FriendlyName) — $($d.InstanceId)"
        # /remove-device unregisters AND removes the driver binding.
        # /force avoids the prompt for non-removable devices.
        & pnputil.exe /remove-device "$($d.InstanceId)" /force | Out-Null
    }
    Write-Host "  $($devices.Count) device(s) removed." -ForegroundColor Green
}

# --------------------------------------------------------------------
# 3. Surface Kinect-related OEM drivers still in the Driver Store.
# --------------------------------------------------------------------

Write-Host ""
Write-Host "[4/4] OEM drivers in Driver Store mentioning Kinect:" -ForegroundColor Yellow

$drivers = & pnputil.exe /enum-drivers
$inBlock = $false
$buffer = New-Object System.Collections.Generic.List[string]
$matched = $false

foreach ($line in $drivers) {
    if ($line -match '^Published Name') {
        # Flush previous block.
        if ($inBlock) {
            $blockText = $buffer -join "`n"
            if ($blockText -match '(?i)kinect|xbox\s*nui|microsoft.+(camera|usb\s*audio)') {
                Write-Host ""
                Write-Host $blockText
                $matched = $true
            }
        }
        $buffer.Clear()
        $inBlock = $true
    }
    if ($inBlock) { $buffer.Add($line) | Out-Null }
}
# Flush last block.
if ($inBlock) {
    $blockText = $buffer -join "`n"
    if ($blockText -match '(?i)kinect|xbox\s*nui|microsoft.+(camera|usb\s*audio)') {
        Write-Host ""
        Write-Host $blockText
        $matched = $true
    }
}

if (-not $matched) {
    Write-Host "  No Kinect-related OEM driver found in the Driver Store." -ForegroundColor Gray
} else {
    Write-Host ""
    Write-Host "  To delete one of the above, run (as admin):" -ForegroundColor Gray
    Write-Host '    pnputil /delete-driver oem<N>.inf /uninstall /force' -ForegroundColor Gray
}

# --------------------------------------------------------------------
# Done.
# --------------------------------------------------------------------

Write-Host ""
Write-Host "================================================================" -ForegroundColor Cyan
Write-Host " Done." -ForegroundColor Green
Write-Host "================================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Next steps depend on which Kinect you have:"
Write-Host ""
Write-Host "  Kinect v2 (Xbox One):"
Write-Host "    - Replug the device. UsbDk + libfreenect2 do the rest."
Write-Host "    - Restart VPX / headtracking-demo, you should be live."
Write-Host ""
Write-Host "  Kinect v1 (Xbox 360):"
Write-Host "    - Replug the device."
Write-Host "    - Each Xbox NUI sub-device should appear in Device"
Write-Host "      Manager with a yellow '?' (no driver bound)."
Write-Host "    - Run Zadig (https://zadig.akeo.ie/):"
Write-Host "        Options -> List All Devices                 (CHECKED)"
Write-Host "        Options -> Ignore Hubs or Composite Parents (UNCHECKED)"
Write-Host "      For each Xbox NUI Audio / Camera / Motor: pick WinUSB"
Write-Host "      (v6.x or newer) on the right, then click Replace Driver."
Write-Host "    - Restart VPX / headtracking-demo, the v1 should enumerate."
Write-Host ""
Read-Host "Press Enter to close this window"
