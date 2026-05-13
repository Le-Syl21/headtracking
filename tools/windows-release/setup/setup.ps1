# --------------------------------------------------------------------
# HeadTracking - Kinect Windows setup (v1 + v2)
# --------------------------------------------------------------------
#
# Run as administrator. Performs four things:
#
#   1. Removes any currently-attached Kinect device instance from
#      the system AND deletes ALL Kinect-related driver packages
#      from the Driver Store (legacy Microsoft Kinect SDK drivers,
#      leftover Zadig output, AND our own kinect_v[12]_*.inf from
#      previous runs of this script). This guarantees that step 2
#      installs a fresh, deterministic set of 9 INFs from scratch.
#
#      Also cleans up any DenyDeviceIDs policy entries left by a
#      previous version of this script - we no longer use that
#      mechanism (see "Why no DenyDeviceIDs" below).
#
#   2. Installs the bundled WinUSB INF packages from the `drivers/`
#      folder next to this script. When devices re-enumerate (which
#      happens immediately after step 1), PnP picks our INF over
#      the inbox alternative based on rank: our libwdi INFs match
#      strictly on VID/PID and rank 00FF0001, beating usb.inf at
#      00FF2006 (compatible-ID match for USB\COMPOSITE).
#
#   3. Clears CONFIGFLAG_FAILEDINSTALL (bit 0x40) on any Kinect
#      device that's still in CM_PROB_FAILED_INSTALL (problem code 28).
#      Without this, devices that landed in FAILED_INSTALL on a
#      previous run will never get re-bound, because PnP refuses to
#      retry once the flag is set. The `pnputil /add-driver /install`
#      from step 2 doesn't clear it either. We do it ourselves.
#
#   4. Triggers `pnputil /scan-devices` to force PnP to re-enumerate
#      USB buses, which (a) re-discovers any device deregistered in
#      step 1, and (b) retries binding on devices whose FAILED_INSTALL
#      flag was just cleared in step 3.
#
# Why no DenyDeviceIDs: earlier versions of this script applied a
# DeviceInstall\Restrictions\DenyDeviceIDs policy to block usbaudio.sys
# (and any future MS Kinect driver) from auto-grabbing the v1 sub-
# devices over our WinUSB binding. In practice that policy turned
# out to be its own worst enemy: the hardware-initiated install
# triggered by step 1's `/remove-device` was blocked by the deny
# (error 0xE0000248), which set CONFIGFLAG_FAILEDINSTALL on the
# device, after which no subsequent pnputil call could rebind it -
# AllowAdminInstall=1 doesn't apply to silent installs. Microsoft
# has not shipped a new Kinect driver via Windows Update in years
# and the hardware is EOL, so the protection isn't worth the
# complexity. PnP rank already picks our INF first.
#
# Why WinUSB everywhere (rather than UsbDk for v2): WinUSB is
# Microsoft's inbox generic USB driver, kernel-signed, works under
# HVCI / Memory Integrity, and gives libusb a direct claim path
# to the device. UsbDk is a third-party filter driver that needs
# an underlying function driver to attach to - which the v1 Camera
# and Motor sub-devices don't have, and which on v2 means we're
# stacked above whatever Microsoft decides to bind today. WinUSB
# bound directly is the simplest stable solution.
#
# Re-running this script is idempotent: registry entries are wiped
# and rewritten from scratch, missing devices are silently skipped,
# `pnputil /add-driver` is a no-op when the same INF is already in
# the Driver Store at the same DriverVer.
# --------------------------------------------------------------------

#Requires -RunAsAdministrator

$ErrorActionPreference = 'Stop'

# Wrap everything in try/catch/finally so:
#  - any uncaught error is shown in red BEFORE the press-any-key prompt
#    (PowerShell only re-raises uncaught exceptions after `finally`
#    returns, by which point our cmd wrapper has closed the window)
#  - the press-any-key prompt always runs, even on `exit 1` paths
try {

Write-Host "================================================================" -ForegroundColor Cyan
Write-Host " HeadTracking - Kinect Windows setup" -ForegroundColor Cyan
Write-Host "================================================================" -ForegroundColor Cyan

# --------------------------------------------------------------------
# Pre-flight: explicit consent before we wipe the Driver Store.
#
# Step 1 deletes every Kinect driver package currently in the Driver
# Store (Microsoft Kinect for Windows v2 SDK runtime included) so
# our WinUSB INFs are the only candidates left for PnP rebind. That
# breaks any software that relies on the MS SDK driver - notably:
#   * BAM (Beautiful Authentic Mods) head tracking for VPX
#   * Kinect Studio / Kinect SDK Browser
#   * any other tool that opens the Kinect through the official SDK
# So we ask the user before proceeding. Set
# $env:HEADTRACKING_SETUP_FORCE=1 to skip the prompt (CI / scripted
# re-runs).
# --------------------------------------------------------------------

if (-not $env:HEADTRACKING_SETUP_FORCE) {
    Write-Host ""
    Write-Host " WARNING - this replaces any existing Kinect driver" -ForegroundColor Yellow
    Write-Host " ---------------------------------------------------" -ForegroundColor Yellow
    Write-Host " This script will:" -ForegroundColor Yellow
    Write-Host "   * Delete every Kinect driver currently in the Windows" -ForegroundColor Yellow
    Write-Host "     Driver Store - including the Microsoft Kinect for" -ForegroundColor Yellow
    Write-Host "     Windows v2 SDK runtime, if installed." -ForegroundColor Yellow
    Write-Host "   * Install our bundled WinUSB drivers in their place" -ForegroundColor Yellow
    Write-Host "     on all known Kinect v1 / v2 USB VID/PIDs." -ForegroundColor Yellow
    Write-Host ""
    Write-Host " If you currently use BAM head tracking, Kinect Studio," -ForegroundColor Yellow
    Write-Host " or anything else that needs the Microsoft Kinect SDK" -ForegroundColor Yellow
    Write-Host " driver, it WILL stop working until you reinstall the" -ForegroundColor Yellow
    Write-Host " MS SDK runtime (manual, no automated rollback)." -ForegroundColor Yellow
    Write-Host ""
    Write-Host " Type 'yes' to proceed, anything else to abort." -ForegroundColor Yellow
    Write-Host ""
    $answer = Read-Host " Continue?"
    if ($answer -ne 'yes') {
        Write-Host ""
        Write-Host "Aborted by user. No changes were made." -ForegroundColor Cyan
        Write-Host ""
        Read-Host "Press Enter to close"
        exit 0
    }
    Write-Host ""
}

# --------------------------------------------------------------------
# Shared constants used by multiple steps.
# --------------------------------------------------------------------

# All Kinect PIDs we care about (used by steps 1, 3, 4).
$kinectPids = @(
    # v1
    '02AD', '02AE', '02B0', '02BB', '02BE', '02BF', '02C2',
    # v2 (including 02D9 hub - we want to remove the device instance
    # in step 1, not the driver binding; PnP will re-bind usbhub.sys
    # on next plug since 02D9 is NOT in our deny list)
    '02C4', '02D8', '02D9'
)

# Clean up any DenyDeviceIDs policy entries left by a previous
# version of this script (which used to apply a deny policy). Older
# entries left in place will block our binding in step 2/4 with
# 0xE0000248 - we strip them out unconditionally. Setting the
# DenyDeviceIDs DWORD to 0 alone is not enough; the PnP manager
# still enforces entries in the sub-key, so we delete them.
$restrictionsKey = 'HKLM:\SOFTWARE\Policies\Microsoft\Windows\DeviceInstall\Restrictions'
$denyKey = "$restrictionsKey\DenyDeviceIDs"
if (Test-Path $denyKey) {
    $existing = @((Get-Item -Path $denyKey).GetValueNames())
    $kinectEntriesWiped = 0
    foreach ($name in $existing) {
        $val = (Get-ItemProperty -Path $denyKey -Name $name -ErrorAction SilentlyContinue).$name
        if ($val -match '(?i)VID_045E&PID_(02AD|02AE|02B0|02BB|02BE|02BF|02C2|02C4|02D8)') {
            Remove-ItemProperty -Path $denyKey -Name $name -ErrorAction SilentlyContinue
            $kinectEntriesWiped++
        }
    }
    if ($kinectEntriesWiped -gt 0) {
        Write-Host ""
        Write-Host "  (Cleaned up $kinectEntriesWiped legacy DenyDeviceIDs entry/entries from previous runs.)" -ForegroundColor Gray
    }
    # If the deny list is now empty, also clear the master flag so
    # the policy is fully disabled (cosmetic - the entries are gone
    # so nothing would match anyway).
    if (@((Get-Item -Path $denyKey).GetValueNames()).Count -eq 0) {
        if (Test-Path $restrictionsKey) {
            Set-ItemProperty -Path $restrictionsKey -Name 'DenyDeviceIDs' -Type DWord -Value 0 -ErrorAction SilentlyContinue
        }
    }
}

# --------------------------------------------------------------------
# 1. Remove currently-attached Kinect device instances + delete any
#    legacy Kinect drivers from the Driver Store (other than ours).
# --------------------------------------------------------------------

Write-Host ""
Write-Host "[1/4] Removing existing Kinect devices and legacy drivers..." -ForegroundColor Yellow

# 1a) Currently-attached AND ghost device instance removal.
#
# Two issues to handle:
#
# A) Get-PnpDevice -PresentOnly:$false has known inconsistencies on
#    Windows 11: depending on the device subsystem and recent PnP
#    events, it can return only the present devices, only the ghosts,
#    or both - never reliably ALL of them. So we run TWO queries and
#    union the results. Deduplication on InstanceId is enough since
#    each instance is unique.
#
# B) `pnputil /remove-device` MUST be called with `/subtree` for
#    composite devices like the Kinect v2 sensor, which exposes
#    USB\VID_045E&PID_02C4 as a parent and two child sub-interfaces
#    (MI_00 and MI_02). Without /subtree, only the parent is
#    unregistered and the orphan children leave the device tree in
#    a half-broken state - the /scan-devices in step 4 then fails
#    to rebind them, leaving Status=Error.
#    /subtree is documented as "Supprimer toute la sous-arborescence
#    de l'appareil, y compris tous les appareils enfants" and has
#    been available since Windows 10 version 2004 (May 2020).
#    /force on /remove-device only exists since Windows 11 22H2
#    (build 22621), and only matters for "system-critical" devices
#    (which our Kinect isn't), so we add it conditionally just in
#    case PnP flags something unexpected.
$winBuild = [System.Environment]::OSVersion.Version.Build
$haveForceFlag = ($winBuild -ge 22621)

$devsPresent = @(Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | Where-Object {
    $_.InstanceId -match 'USB\\VID_045E&PID_([0-9A-Fa-f]{4})' -and
    $kinectPids -contains $matches[1].ToUpper()
})
$devsAll = @(Get-PnpDevice -PresentOnly:$false -ErrorAction SilentlyContinue | Where-Object {
    $_.InstanceId -match 'USB\\VID_045E&PID_([0-9A-Fa-f]{4})' -and
    $kinectPids -contains $matches[1].ToUpper()
})
$seen = @{}
$devices = @()
foreach ($d in @($devsPresent) + @($devsAll)) {
    if (-not $seen.ContainsKey($d.InstanceId)) {
        $seen[$d.InstanceId] = $true
        $devices += $d
    }
}

if ($devices.Count -eq 0) {
    Write-Host "  No Kinect device currently registered. Nothing to remove." -ForegroundColor Gray
} else {
    foreach ($d in $devices) {
        Write-Host "  Removing device: $($d.FriendlyName) - $($d.InstanceId)"
        # Build the pnputil arg list. /subtree is mandatory for clean
        # removal of composite parents + children. /force is added
        # only when the running Windows supports it.
        $pnpArgs = @('/remove-device', $d.InstanceId, '/subtree')
        if ($haveForceFlag) { $pnpArgs += '/force' }
        & pnputil.exe @pnpArgs | Out-Null
    }
    Write-Host "  $($devices.Count) device instance(s) removed." -ForegroundColor Green
}

# 1b) Driver package deletion from the Driver Store.
# We use Get-WindowsDriver instead of parsing `pnputil /enum-drivers`
# text output. Microsoft's official docs explicitly state that the
# pnputil text format is localized and "should not be parsed by
# scripts" - even with $CurrentUICulture forced to en-US, edge cases
# (console code page, regional settings) can break a regex parser.
# Get-WindowsDriver returns strongly-typed objects, locale-independent,
# so the match works on any Windows install.
#
# We delete EVERY Kinect-related package, including our own
# kinect_v[12]_*.inf from previous runs of this script. Step 2 below
# will re-install all 9 INFs cleanly. This guarantees a deterministic
# from-scratch state regardless of past experimentation, half-finished
# Zadig sessions, accumulated duplicates, etc.
Write-Host ""
Write-Host "  Scanning Driver Store for Kinect drivers..."

$installedDrivers = @()
try {
    $installedDrivers = @(Get-WindowsDriver -Online -ErrorAction Stop)
} catch {
    Write-Host "  [WARN] Get-WindowsDriver failed: $($_.Exception.Message)" -ForegroundColor Yellow
    Write-Host "         Skipping the legacy-driver wipe step. Step 2 will" -ForegroundColor Yellow
    Write-Host "         still install our INFs (any pre-existing copies will" -ForegroundColor Yellow
    Write-Host "         be reported as 'Already in Driver Store')." -ForegroundColor Yellow
}

$kinectPattern = '(?i)kinect|xbox\s*nui'

$toDelete = @()
foreach ($drv in $installedDrivers) {
    # Match on any of the strongly-typed fields. OriginalFileName is
    # usually the most reliable (e.g. "kinect_v1_1414_audio.inf"), but
    # we also check ProviderName and ClassDescription so legacy
    # Microsoft Kinect SDK packages (which name their INF differently)
    # are also caught.
    $origName = if ($drv.OriginalFileName) { [System.IO.Path]::GetFileName($drv.OriginalFileName) } else { '' }
    if (
        ($origName              -match $kinectPattern) -or
        ($drv.ProviderName      -match $kinectPattern) -or
        ($drv.ClassDescription  -match $kinectPattern)
    ) {
        $toDelete += $drv
    }
}

if ($toDelete.Count -eq 0) {
    Write-Host "  No Kinect driver in the Driver Store." -ForegroundColor Gray
} else {
    Write-Host "  Found $($toDelete.Count) Kinect driver(s) to delete:" -ForegroundColor Yellow
    foreach ($drv in $toDelete) {
        # 'Driver' property is the published name (oemNN.inf).
        # OriginalFileName is the original INF filename (or full path).
        $publishedName = $drv.Driver
        $origLabel     = if ($drv.OriginalFileName) {
            [System.IO.Path]::GetFileName($drv.OriginalFileName)
        } else {
            '(unknown)'
        }
        Write-Host "    - $publishedName  (orig: $origLabel)"

        # Two-step delete strategy:
        #
        #   1. First try `/uninstall` alone. This tells pnputil to also
        #      unbind the driver from any device currently using it,
        #      then remove the package from the Driver Store. Clean
        #      path, works for most packages.
        #
        #   2. If that fails (typically because the package is held by
        #      multiple device instances and pnputil only released one),
        #      retry with `/force` alone. This deletes the package from
        #      the store unconditionally, leaving any still-bound device
        #      pointing at a non-existent INF for a moment - PnP will
        #      re-bind them at the next bus rescan (step 4/5) using the
        #      fresh INFs we install in step 2/5.
        #
        # Importantly, we do NOT pass `/uninstall /force` together:
        # recent pnputil versions explicitly print "Ignoring /force when
        # used with /uninstall to remove the driver package" - the two
        # flags are mutually exclusive on modern Windows 11.
        $output = & pnputil.exe /delete-driver $publishedName /uninstall 2>&1
        if ($LASTEXITCODE -eq 0) {
            Write-Host "      Deleted." -ForegroundColor Green
        } else {
            Write-Host "      In use, retrying with /force..." -ForegroundColor Gray
            $output = & pnputil.exe /delete-driver $publishedName /force 2>&1
            if ($LASTEXITCODE -eq 0) {
                Write-Host "      Deleted (forced)." -ForegroundColor Green
            } else {
                Write-Host "      [FAIL] pnputil exit $LASTEXITCODE" -ForegroundColor Red
                Write-Host "      $output" -ForegroundColor Red
            }
        }
    }
}

# --------------------------------------------------------------------
# 2. Install bundled WinUSB INF packages from drivers\ folder.
# --------------------------------------------------------------------

Write-Host ""
Write-Host "[2/4] Installing WinUSB drivers (libusb backend)..." -ForegroundColor Yellow

# Resolve the script's own directory. $PSScriptRoot works when invoked
# via `powershell -File`, which is how our .cmd wrapper calls us.
# Fallback covers dot-sourcing or unusual invocations.
$scriptDir = $PSScriptRoot
if (-not $scriptDir) {
    $scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
}
$driversDir = Join-Path $scriptDir 'drivers'

if (-not (Test-Path -Path $driversDir -PathType Container)) {
    Write-Host "  No 'drivers\' folder next to this script." -ForegroundColor Gray
    Write-Host "  Skipping automatic driver install." -ForegroundColor Gray
    Write-Host "  -> Run Zadig (https://zadig.akeo.ie/) manually to bind" -ForegroundColor Gray
    Write-Host "     WinUSB on each Kinect sub-device, see Next steps below." -ForegroundColor Gray
} else {
    $infFiles = @(Get-ChildItem -Path $driversDir -Filter '*.inf' -Recurse -ErrorAction SilentlyContinue)
    if ($infFiles.Count -eq 0) {
        Write-Host "  Folder '$driversDir' contains no .inf files." -ForegroundColor Gray
        Write-Host "  Skipping automatic driver install." -ForegroundColor Gray
    } else {
        Write-Host "  Found $($infFiles.Count) INF package(s) in $driversDir"

        # ---- Pre-trust the libwdi self-signed cert that signs our .cat files.
        #
        # libwdi (the lib Zadig uses to build these INFs) generates a
        # self-signed code-signing cert per build, packed into each
        # .cat. For Windows to install the driver silently via
        # `pnputil /add-driver /install`, the cert needs to be in
        # TWO LocalMachine stores:
        #
        #   - `Root`            -> makes the cert chain validate. A
        #                         self-signed cert *is* its own root,
        #                         so without this entry Windows says
        #                         "the certificate chain ends in an
        #                         untrusted root" no matter what else
        #                         we set.
        #   - `TrustedPublisher`-> tells Windows "skip the install-
        #                         confirmation prompt for drivers
        #                         signed by this publisher". With a
        #                         valid chain but no entry here, the
        #                         user still sees a dialog per INF.
        #
        # libwdi / Zadig add to both internally; we replicate the
        # same dual-store install so the unattended flow works.
        # Without these the alternative is either a wall of dialogs
        # or `Driver package failed signature validation` under HVCI.
        #
        # libwdi reuses the same self-signed cert for the whole batch
        # of INFs it generates, so all 9 .cats share an issuer; but
        # the PER-DEVICE subject still differs across .cats. We
        # extract from each .cat and dedup by thumbprint so re-runs
        # don't pile up entries.
        $catFiles = @(Get-ChildItem -Path $driversDir -Filter '*.cat' -Recurse -ErrorAction SilentlyContinue)
        if ($catFiles.Count -gt 0) {
            Write-Host "  Pre-trusting $($catFiles.Count) signer cert(s) into LocalMachine\Root + TrustedPublisher..."
            $stores = @(
                [System.Security.Cryptography.X509Certificates.StoreName]::Root,
                [System.Security.Cryptography.X509Certificates.StoreName]::TrustedPublisher
            )
            foreach ($storeName in $stores) {
                $store = New-Object System.Security.Cryptography.X509Certificates.X509Store(
                    $storeName,
                    [System.Security.Cryptography.X509Certificates.StoreLocation]::LocalMachine)
                $store.Open([System.Security.Cryptography.X509Certificates.OpenFlags]::ReadWrite)
                try {
                    $added = 0
                    foreach ($cat in $catFiles) {
                        $sig = Get-AuthenticodeSignature $cat.FullName
                        if ($sig.SignerCertificate) {
                            $existing = $store.Certificates.Find(
                                [System.Security.Cryptography.X509Certificates.X509FindType]::FindByThumbprint,
                                $sig.SignerCertificate.Thumbprint, $false)
                            if ($existing.Count -eq 0) {
                                $store.Add($sig.SignerCertificate)
                                $added++
                            }
                        }
                    }
                    if ($added -eq 0) {
                        Write-Host "    [$storeName] All signer certs already trusted." -ForegroundColor Gray
                    } else {
                        Write-Host "    [$storeName] $added new cert(s) added." -ForegroundColor Green
                    }
                } finally {
                    $store.Close()
                }
            }
        }

        $okCount = 0
        $failCount = 0
        foreach ($inf in $infFiles) {
            Write-Host "    -> $($inf.Name)"
            # /install: also bind to any present matching device right away
            # (otherwise INF just lands in the Driver Store and waits for
            # the next plug event).
            $output = & pnputil.exe /add-driver "$($inf.FullName)" /install 2>&1
            if ($LASTEXITCODE -eq 0) {
                Write-Host "       OK" -ForegroundColor Green
                $okCount++
            } elseif ($LASTEXITCODE -eq 259) {
                # ERROR_NO_MORE_ITEMS - pnputil reports this when the
                # exact same INF (same hash) is already in the store.
                Write-Host "       Already in Driver Store (no change)." -ForegroundColor Gray
                $okCount++
            } else {
                Write-Host "       [FAIL] pnputil exit $LASTEXITCODE" -ForegroundColor Red
                Write-Host "       $output" -ForegroundColor Red
                $failCount++
            }
        }
        Write-Host ""
        if ($failCount -eq 0) {
            Write-Host "  All $okCount driver package(s) installed successfully." -ForegroundColor Green
        } else {
            Write-Host "  $okCount package(s) OK, $failCount failed." -ForegroundColor Yellow
        }
    }
}

# --------------------------------------------------------------------
# 3. Clear CONFIGFLAG_FAILEDINSTALL on Kinect devices stuck in error.
# --------------------------------------------------------------------
#
# When a device hits CM_PROB_FAILED_INSTALL (problem code 28), PnP
# sets bit 0x40 (CONFIGFLAG_FAILEDINSTALL) on
# HKLM\SYSTEM\CurrentControlSet\Enum\<InstanceId>\ConfigFlags. Once
# this bit is set, PnP refuses to retry the install on subsequent
# /scan-devices or hardware events - even if the missing driver has
# just been added to the Driver Store. This is a "give up" marker.
#
# `pnputil /add-driver <inf> /install` does NOT clear this bit. The
# only way to unstick the device is to clear it ourselves (or use
# Device Manager UI's "Update driver" flow, which sets a different
# signal that triggers the retry).
#
# This step is what made the difference for devices that arrived to
# this script already in FAILED_INSTALL (e.g. from a previous Zadig
# attempt or a previous run of this script under the old ordering
# where DenyDeviceIDs blocked the install and stamped FAILEDINSTALL
# on the way down).
Write-Host ""
Write-Host "[3/4] Clearing FAILEDINSTALL flag on stuck Kinect devices..." -ForegroundColor Yellow

$stuck = @(Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | Where-Object {
    $_.InstanceId -match 'USB\\VID_045E&PID_([0-9A-Fa-f]{4})' -and
    $kinectPids -contains $matches[1].ToUpper()
})

if ($stuck.Count -eq 0) {
    Write-Host "  No Kinect device present to inspect." -ForegroundColor Gray
} else {
    $clearedCount = 0
    foreach ($d in $stuck) {
        $regPath = "HKLM:\SYSTEM\CurrentControlSet\Enum\$($d.InstanceId)"
        if (-not (Test-Path $regPath)) { continue }
        $cf = (Get-ItemProperty -Path $regPath -Name 'ConfigFlags' -ErrorAction SilentlyContinue).ConfigFlags
        if ($null -eq $cf) { continue }
        if ($cf -band 0x40) {
            $newVal = $cf -band (-bnot 0x40)
            Set-ItemProperty -Path $regPath -Name 'ConfigFlags' -Type DWord -Value $newVal
            Write-Host "    + Cleared FAILEDINSTALL on $($d.FriendlyName) ($($d.InstanceId))" -ForegroundColor Green
            $clearedCount++
        }
    }
    if ($clearedCount -eq 0) {
        Write-Host "  No device had the FAILEDINSTALL flag set." -ForegroundColor Gray
    } else {
        Write-Host "  $clearedCount device(s) unstuck." -ForegroundColor Green
    }
}

# --------------------------------------------------------------------
# 4. Trigger a PnP rescan to bind WinUSB on already-plugged Kinects.
# --------------------------------------------------------------------

Write-Host ""
Write-Host "[4/4] Re-scanning USB devices..." -ForegroundColor Yellow

# `pnputil /scan-devices` asks the PnP manager to walk every bus and
# re-enumerate hardware. Devices we deregistered in step 1 (or whose
# FAILEDINSTALL flag we just cleared in step 3) are still physically
# present on USB, so PnP rediscovers them, looks at the Driver Store
# (which now contains our INFs from step 2), finds an exact VID/PID
# match, and binds WinUSB - no physical replug needed. This is what
# makes the script usable when the Kinect is hard-mounted inside a
# pinball cabinet and unreachable from the front.
& pnputil.exe /scan-devices | Out-Null

# Sample the current binding state of every present Kinect device.
# Returns one row per device with the expected vs. actual kernel
# service and whether the binding is "correct". This is called
# repeatedly by the polling loop below until either all devices
# have stabilized on the right service or we hit the timeout.
function Get-KinectBindingState {
    param($pids)
    $devs = @(Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | Where-Object {
        $_.InstanceId -match 'USB\\VID_045E&PID_([0-9A-Fa-f]{4})' -and
        $pids -contains $matches[1].ToUpper()
    })
    $rows = @()
    foreach ($d in $devs) {
        $devPid = if ($d.InstanceId -match 'PID_([0-9A-Fa-f]{4})') { $matches[1].ToUpper() } else { '????' }
        $service = ''
        try {
            $sp = Get-PnpDeviceProperty -InstanceId $d.InstanceId -KeyName 'DEVPKEY_Device_Service' -ErrorAction Stop
            if ($sp.Data) { $service = $sp.Data }
        } catch {
            # Property unavailable; leave service blank, treated as "not bound yet".
        }
        # 02D9 (Kinect Adapter hub) is the one PID where we WANT the
        # inbox driver. It must stay on usbhub.sys so Windows can
        # enumerate the sensor (02C4) downstream of it. Anything else
        # bound there is a problem.
        $expected = if ($devPid -eq '02D9') { 'usbhub' } else { 'WINUSB' }
        $rows += [PSCustomObject]@{
            Device     = $d
            Pid        = $devPid
            Service    = $service
            Expected   = $expected
            IsCorrect  = ($service -ieq $expected) -and ($d.Status -eq 'OK')
        }
    }
    return ,$rows
}

# Poll the binding state. Right after `/scan-devices`, PnP has only
# scheduled the rebinds - the kernel service association doesn't
# show up in DEVPKEY_Device_Service for another few hundred
# milliseconds (sometimes a few seconds on busy systems / under
# HVCI). Without this wait, the script prints "driver=(unknown)"
# while the bindings are actually about to land seconds later.
$timeoutSec     = 15
$pollEverySec   = 0.5
$elapsedSec     = 0.0
$state          = @(Get-KinectBindingState -pids $kinectPids)

if ($state.Count -gt 0) {
    Write-Host "  Waiting for PnP to attach WinUSB (timeout ${timeoutSec}s)..." -ForegroundColor Gray
    while ($elapsedSec -lt $timeoutSec) {
        if (($state | Where-Object { -not $_.IsCorrect }).Count -eq 0) { break }
        Start-Sleep -Milliseconds ($pollEverySec * 1000)
        $elapsedSec += $pollEverySec
        $state = @(Get-KinectBindingState -pids $kinectPids)
    }
    if (($state | Where-Object { -not $_.IsCorrect }).Count -eq 0) {
        Write-Host ("  Stabilized after {0:N1} s." -f $elapsedSec) -ForegroundColor Green
    } else {
        Write-Host ("  Still not stable after {0}s - reporting current state." -f $timeoutSec) -ForegroundColor Yellow
    }
}

$allBoundCorrectly = $true
$detectedKinects   = @($state | ForEach-Object { $_.Device })

if ($detectedKinects.Count -eq 0) {
    $allBoundCorrectly = $false
    Write-Host "  No Kinect detected on USB." -ForegroundColor Gray
    Write-Host "  -> Plug the Kinect (or check it's powered on) and re-run" -ForegroundColor Gray
    Write-Host "     this script if needed." -ForegroundColor Gray
} else {
    Write-Host "  $($detectedKinects.Count) Kinect device(s) detected:" -ForegroundColor Green
    foreach ($row in $state) {
        $roleNote = if ($row.Pid -eq '02D9') { '(hub, must stay on usbhub)' } else { '' }
        $serviceLabel = if ($row.Service) { $row.Service } else { '(unknown)' }
        if ($row.IsCorrect) {
            $marker = "[OK driver=$serviceLabel] $roleNote"
            $color  = 'Green'
        } else {
            $marker = "[WRONG driver=$serviceLabel, expected=$($row.Expected), status=$($row.Device.Status)] $roleNote"
            $color  = 'Yellow'
            $allBoundCorrectly = $false
        }
        Write-Host "    + [$($row.Pid)] $($row.Device.FriendlyName) $marker" -ForegroundColor $color
    }

    if (-not $allBoundCorrectly) {
        Write-Host ""
        Write-Host "  WARNING: at least one device is not bound to the expected driver." -ForegroundColor Yellow
        Write-Host "  Re-run this script - step [3/4] (clear FAILEDINSTALL) +" -ForegroundColor Yellow
        Write-Host "  step [4/4] (rescan) should clear it on the second pass." -ForegroundColor Yellow
    }
}

# --------------------------------------------------------------------
# Done.
# --------------------------------------------------------------------

Write-Host ""
Write-Host "================================================================" -ForegroundColor Cyan
Write-Host " Done." -ForegroundColor Green
Write-Host "================================================================" -ForegroundColor Cyan
Write-Host ""

if ($detectedKinects.Count -gt 0 -and $allBoundCorrectly) {
    # All detected devices are bound to the expected driver. We're done.
    Write-Host "Next step:" -ForegroundColor Yellow
    Write-Host "  Restart VPX / headtracking-demo - tracking should be live."
} else {
    # Either no device detected, or at least one detected device is on
    # the wrong driver (which [4/4] already explained). Either way, the
    # user has nothing more to do locally - point them at the issue
    # tracker so we can investigate (most likely an unsupported PID).
    if ($detectedKinects.Count -eq 0) {
        Write-Host "No Kinect was detected on USB." -ForegroundColor Yellow
    } else {
        Write-Host "A Kinect is detected but at least one device has the wrong" -ForegroundColor Yellow
        Write-Host "driver (see [4/4] above)." -ForegroundColor Yellow
    }
    Write-Host ""
    Write-Host "Please check:" -ForegroundColor Yellow
    Write-Host "  - The Kinect is firmly plugged into a USB port (USB 3 for v2)."
    Write-Host "  - The external 12V power brick is connected (Kinect v1) or"
    Write-Host "    the Kinect Adapter is powered (Kinect v2 on PC)."
    Write-Host "  - The status LED on the Kinect is lit."
    Write-Host ""
    Write-Host "If everything is correctly connected and powered, please open"
    Write-Host "an issue at:"
    Write-Host "  https://github.com/Le-Syl21/headtracking/issues" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "Include the full output of this script in the issue (especially"
    Write-Host "the [4/4] section), plus the output of:"
    Write-Host '  Get-PnpDevice | Where-Object { $_.InstanceId -like "*VID_045E*" }'
}
Write-Host ""

} catch {
    # Capture and display the exception BEFORE the finally's ReadKey,
    # otherwise the cmd window closes after the keypress and the user
    # never sees what went wrong (PowerShell re-raises uncaught
    # exceptions only after `finally` returns control to the caller).
    Write-Host ""
    Write-Host "================================================================" -ForegroundColor Red
    Write-Host " [FATAL] Unhandled error" -ForegroundColor Red
    Write-Host "================================================================" -ForegroundColor Red
    Write-Host ""
    Write-Host "  Type    : $($_.Exception.GetType().FullName)" -ForegroundColor Red
    Write-Host "  Message : $($_.Exception.Message)" -ForegroundColor Red
    Write-Host "  At      : $($_.InvocationInfo.PositionMessage)" -ForegroundColor Red
    Write-Host ""
    if ($_.Exception.InnerException) {
        Write-Host "  Inner   : $($_.Exception.InnerException.Message)" -ForegroundColor Red
        Write-Host ""
    }
    Write-Host "  Stack trace:" -ForegroundColor Red
    Write-Host "$($_.ScriptStackTrace)" -ForegroundColor DarkRed
    Write-Host ""
} finally {
    # Always reached, even on `exit 1` or uncaught throw above.
    # try/ReadKey gives us "press any key" in a real ConsoleHost; the
    # catch falls back to Read-Host (Enter) for ISE / VS Code / remoting
    # / any host where RawUI.ReadKey is unsupported and throws.
    Write-Host ""
    try {
        Write-Host "Press any key to close this window..." -ForegroundColor Cyan
        $null = $Host.UI.RawUI.ReadKey('NoEcho,IncludeKeyDown')
    } catch {
        Read-Host "Press Enter to close this window"
    }
}
