# --------------------------------------------------------------------
# HeadTracking - Kinect Windows setup (v1 + v2)
# --------------------------------------------------------------------
#
# Run as administrator. Performs five things:
#
#   1. Adds DenyDeviceIDs entries for Kinect v1 sub-device PIDs
#      (Audio / Camera / Motor across all known hardware revisions),
#      so Windows PnP refuses to auto-install partial drivers (most
#      importantly `usbaudio.sys` on the v1 Audio interface, which
#      would otherwise re-grab the device after we bind WinUSB).
#
#   2. Adds DenyDeviceIDs entries for Kinect v2 sensor PIDs, so
#      Windows Update cannot silently re-bind a Microsoft driver
#      and dislodge our WinUSB binding over time.
#      NOTE: PID 02D9 (the Kinect Adapter's USB hub) is intentionally
#      NOT denied - it must keep its inbox `usbhub.sys` so Windows
#      can enumerate the sensor downstream of it.
#
#   3. Removes any currently-attached Kinect device instance from
#      the system AND deletes ALL Kinect-related driver packages
#      from the Driver Store (legacy Microsoft Kinect SDK drivers,
#      leftover Zadig output, AND our own kinect_v[12]_*.inf from
#      previous runs of this script). This guarantees that step 4
#      installs a fresh, deterministic set of 9 INFs from scratch -
#      no doublons, no half-stale state from past experimentation.
#
#   4. Installs the bundled WinUSB INF packages from the `drivers/`
#      folder next to this script (one .inf per Kinect VID/PID we
#      support), via `pnputil /add-driver ... /install`. When the
#      user plugs in a supported Kinect, Windows automatically binds
#      WinUSB and libfreenect/libfreenect2 see the device.
#      If the `drivers/` folder is missing or empty, this step is
#      skipped gracefully and the user is told to open an issue at
#      https://github.com/Le-Syl21/headtracking/issues
#
#   5. Triggers `pnputil /scan-devices` to force PnP to re-enumerate
#      USB buses, which re-discovers any plugged-in Kinect that step
#      3 deregistered and binds it to the freshly-installed WinUSB
#      INF from step 4 - without the user having to physically
#      unplug and replug the device. Useful when the Kinect is
#      hard-mounted (e.g. inside a pinball cabinet).
#
# Why WinUSB everywhere (rather than UsbDk for v2): WinUSB is
# Microsoft's inbox generic USB driver, kernel-signed, works under
# HVCI / Memory Integrity, and gives libusb a direct claim path
# to the device. UsbDk is a third-party filter driver that needs
# an underlying function driver to attach to - which the v1 Camera
# and Motor sub-devices don't have, and which on v2 means we're
# stacked above whatever Microsoft decides to bind today. WinUSB
# bound directly is shorter and more stable across Windows Updates,
# given DenyDeviceIDs blocks re-binding attempts.
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
# Shared registry setup (used by steps 1 + 2).
# --------------------------------------------------------------------

$restrictionsKey = 'HKLM:\SOFTWARE\Policies\Microsoft\Windows\DeviceInstall\Restrictions'
$denyKey = "$restrictionsKey\DenyDeviceIDs"

New-Item -Path $restrictionsKey -Force | Out-Null
Set-ItemProperty -Path $restrictionsKey -Name 'DenyDeviceIDs' -Type DWord -Value 1
# Retroactive=0: don't yank already-bound drivers from running devices.
# We unbind those manually in step 3 instead.
Set-ItemProperty -Path $restrictionsKey -Name 'DenyDeviceIDsRetroactive' -Type DWord -Value 0
# AllowAdminInstall=1: by default DenyDeviceIDs blocks ALL installs
# including admin-initiated ones (pnputil, Device Manager, Zadig).
# We need admins to be able to override the deny so step 4 (and any
# manual Zadig fallback) can bind WinUSB. This is the Group Policy
# "Allow administrators to override Device Installation Restriction
# policies" toggle.
Set-ItemProperty -Path $restrictionsKey -Name 'AllowAdminInstall' -Type DWord -Value 1

New-Item -Path $denyKey -Force | Out-Null

# Wipe any pre-existing numbered entries so re-runs don't accumulate
# stale rows. We use Get-Item().GetValueNames() rather than
# Get-ItemProperty | Get-Member, because on PowerShell 5.1
# Get-ItemProperty on a freshly-created (empty) registry key yields
# something the pipeline treats as nothing, and Get-Member then throws
# "You must specify an object for the Get-Member cmdlet" - which under
# $ErrorActionPreference='Stop' kills the whole script. GetValueNames
# is the registry provider's native API, always returns an array
# (possibly empty), and has no such failure mode.
$existingValues = @((Get-Item -Path $denyKey).GetValueNames())
$wiped = 0
foreach ($name in $existingValues) {
    if ($name -match '^\d+$') {
        Remove-ItemProperty -Path $denyKey -Name $name -ErrorAction SilentlyContinue
        $wiped++
    }
}
if ($wiped -gt 0) {
    Write-Host ""
    Write-Host "  (Wiped $wiped stale entry/entries from previous run.)" -ForegroundColor Gray
}

# Shared global counter: registry values must be uniquely numbered
# across both v1 and v2 sections, but we display a per-section index
# for readability.
$globalIndex = 1

# --------------------------------------------------------------------
# 1. Deny Windows driver auto-install for Kinect v1 sub-devices.
# --------------------------------------------------------------------

Write-Host ""
Write-Host "[1/5] Deny Windows driver for Kinect v1..." -ForegroundColor Yellow

$kinectV1Ids = @(
    'USB\VID_045E&PID_02AD',  # Xbox NUI Audio  (1414 rev)
    'USB\VID_045E&PID_02AE',  # Xbox NUI Camera (1414 rev)
    'USB\VID_045E&PID_02B0',  # Xbox NUI Motor  (1414 rev)
    'USB\VID_045E&PID_02BB',  # Xbox NUI Audio  (1473 rev)
    'USB\VID_045E&PID_02BE',  # Kinect for Windows v1 motor variant
    'USB\VID_045E&PID_02BF',  # Xbox NUI Camera (1473 rev)
    'USB\VID_045E&PID_02C2'   # Kinect for Windows v1 variant
)

Write-Host "  Adding $($kinectV1Ids.Count) Kinect v1 VID/PID entries to the deny list:"
$localIndex = 1
foreach ($id in $kinectV1Ids) {
    Set-ItemProperty -Path $denyKey -Name "$globalIndex" -Type String -Value $id
    Write-Host "    + [$localIndex] $id"
    $globalIndex++
    $localIndex++
}
Write-Host "  Kinect v1 deny entries written." -ForegroundColor Green

# --------------------------------------------------------------------
# 2. Deny Windows driver auto-install for Kinect v2 sensors.
# --------------------------------------------------------------------

Write-Host ""
Write-Host "[2/5] Deny Windows driver for Kinect v2..." -ForegroundColor Yellow

$kinectV2Ids = @(
    'USB\VID_045E&PID_02C4',  # Kinect for Xbox One sensor (1520)
    'USB\VID_045E&PID_02D8'   # Kinect for Windows v2 sensor
    # 02D9 (Kinect Adapter hub) deliberately omitted - leaving its
    # inbox usbhub.sys binding intact is what allows Windows to
    # enumerate the sensor (02C4) downstream of it.
)

Write-Host "  Adding $($kinectV2Ids.Count) Kinect v2 VID/PID entries to the deny list:"
$localIndex = 1
foreach ($id in $kinectV2Ids) {
    Set-ItemProperty -Path $denyKey -Name "$globalIndex" -Type String -Value $id
    Write-Host "    + [$localIndex] $id"
    $globalIndex++
    $localIndex++
}
Write-Host "  Kinect v2 deny entries written." -ForegroundColor Green
Write-Host "  Registry updated. Reboot is NOT required for new plugs." -ForegroundColor Green

# --------------------------------------------------------------------
# 3. Remove currently-attached Kinect device instances + delete any
#    legacy Kinect drivers from the Driver Store (other than ours).
# --------------------------------------------------------------------

Write-Host ""
Write-Host "[3/5] Removing existing Kinect devices and legacy drivers..." -ForegroundColor Yellow

$kinectPids = @(
    # v1
    '02AD', '02AE', '02B0', '02BB', '02BE', '02BF', '02C2',
    # v2 (including 02D9 hub - we want to remove the device instance,
    # not the driver binding; PnP will re-bind usbhub.sys on next plug
    # since 02D9 is NOT in our deny list)
    '02C4', '02D8', '02D9'
)

# 3a) Currently-attached AND ghost device instance removal.
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
#    a half-broken state - the /scan-devices in step 5 then fails
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

# 3b) Driver package deletion from the Driver Store.
# We use Get-WindowsDriver instead of parsing `pnputil /enum-drivers`
# text output. Microsoft's official docs explicitly state that the
# pnputil text format is localized and "should not be parsed by
# scripts" - even with $CurrentUICulture forced to en-US, edge cases
# (console code page, regional settings) can break a regex parser.
# Get-WindowsDriver returns strongly-typed objects, locale-independent,
# so the match works on any Windows install.
#
# We delete EVERY Kinect-related package, including our own
# kinect_v[12]_*.inf from previous runs of this script. Step 4 below
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
    Write-Host "         Skipping the legacy-driver wipe step. Step 4 will" -ForegroundColor Yellow
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
        #      re-bind them at the next bus rescan (step 5/5) using the
        #      fresh INFs we install in step 4/5.
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
# 4. Install bundled WinUSB INF packages from drivers\ folder.
# --------------------------------------------------------------------

Write-Host ""
Write-Host "[4/5] Installing WinUSB drivers (libusb backend)..." -ForegroundColor Yellow

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
        # .cat. Without this cert in the LocalMachine TrustedPublisher
        # store, `pnputil /add-driver /install` would either:
        #   - prompt the user with a Windows "Do you want to install
        #     this driver from libwdi (autogenerated)?" dialog (best
        #     case — every INF prompts, breaks the unattended flow);
        #   - or hard-fail with "Driver package failed signature
        #     validation" under DSE / HVCI strict.
        # Adding the issuer to TrustedPublisher BEFORE pnputil makes
        # the install completely silent.
        #
        # All 9 .cats share the same issuer ("CN = USB\VID_xxx&PID_xxx
        # (libwdi autogenerated)") because libwdi reuses the same
        # self-signed cert across one libwdi run — but the PER-DEVICE
        # subjects differ. We extract the signer cert from each .cat
        # and add them all; certutil deduplicates by thumbprint
        # internally so re-runs don't pile up.
        $catFiles = @(Get-ChildItem -Path $driversDir -Filter '*.cat' -Recurse -ErrorAction SilentlyContinue)
        if ($catFiles.Count -gt 0) {
            Write-Host "  Pre-trusting $($catFiles.Count) signer cert(s) into LocalMachine\TrustedPublisher..."
            $store = New-Object System.Security.Cryptography.X509Certificates.X509Store(
                [System.Security.Cryptography.X509Certificates.StoreName]::TrustedPublisher,
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
                    Write-Host "    All signer certs already trusted." -ForegroundColor Gray
                } else {
                    Write-Host "    $added new cert(s) added." -ForegroundColor Green
                }
            } finally {
                $store.Close()
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
# 5. Trigger a PnP rescan to bind WinUSB on already-plugged Kinects.
# --------------------------------------------------------------------

Write-Host ""
Write-Host "[5/5] Re-scanning USB devices..." -ForegroundColor Yellow

# `pnputil /scan-devices` asks the PnP manager to walk every bus and
# re-enumerate hardware. Devices we deregistered in step 3 are still
# physically present on USB, so PnP rediscovers them, looks at the
# Driver Store (which now contains our INFs from step 4), finds an
# exact VID/PID match, and binds WinUSB - no physical replug needed.
# This is what makes the script usable when the Kinect is hard-mounted
# inside a pinball cabinet and unreachable from the front.
& pnputil.exe /scan-devices | Out-Null

# Reuse $kinectPids from step 3.
$detectedKinects = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | Where-Object {
    $_.InstanceId -match 'USB\\VID_045E&PID_([0-9A-Fa-f]{4})' -and
    $kinectPids -contains $matches[1].ToUpper()
}

$allBoundCorrectly = $true

if ($detectedKinects.Count -eq 0) {
    $allBoundCorrectly = $false
    Write-Host "  No Kinect detected on USB." -ForegroundColor Gray
    Write-Host "  -> Plug the Kinect (or check it's powered on) and re-run" -ForegroundColor Gray
    Write-Host "     this script if needed." -ForegroundColor Gray
} else {
    Write-Host "  $($detectedKinects.Count) Kinect device(s) detected:" -ForegroundColor Green
    foreach ($d in $detectedKinects) {
        $devPid = if ($d.InstanceId -match 'PID_([0-9A-Fa-f]{4})') { $matches[1].ToUpper() } else { '????' }

        # Read the kernel service currently bound to the device. This is
        # the only way to confirm WinUSB is actually loaded - $d.Status
        # being 'OK' just means "some driver is attached", which would
        # also be true with usbaudio.sys or the Microsoft Kinect driver.
        $service = '(unknown)'
        try {
            $serviceProperty = Get-PnpDeviceProperty -InstanceId $d.InstanceId -KeyName 'DEVPKEY_Device_Service' -ErrorAction Stop
            if ($serviceProperty.Data) { $service = $serviceProperty.Data }
        } catch {
            # Property unavailable for this device, leave $service as unknown.
        }

        # 02D9 (Kinect Adapter hub) is the one PID where we WANT the
        # inbox driver. It must stay on usbhub.sys so Windows can
        # enumerate the sensor (02C4) downstream of it. Anything else
        # bound there is a problem.
        if ($devPid -eq '02D9') {
            $expectedService = 'usbhub'
            $roleNote = '(hub, must stay on usbhub)'
        } else {
            $expectedService = 'WINUSB'
            $roleNote = ''
        }

        $isCorrect = ($service -ieq $expectedService) -and ($d.Status -eq 'OK')
        if ($isCorrect) {
            $marker = "[OK driver=$service] $roleNote"
            $color  = 'Green'
        } else {
            $marker = "[WRONG driver=$service, expected=$expectedService, status=$($d.Status)] $roleNote"
            $color  = 'Yellow'
            $allBoundCorrectly = $false
        }
        Write-Host "    + [$devPid] $($d.FriendlyName) $marker" -ForegroundColor $color
    }

    if (-not $allBoundCorrectly) {
        Write-Host ""
        Write-Host "  WARNING: at least one device is not bound to the expected driver." -ForegroundColor Yellow
        Write-Host "  If you see 'usbaudio' on a v1 Audio sub-device, Windows re-grabbed" -ForegroundColor Yellow
        Write-Host "  it before our deny rules took effect. Re-run this script - the" -ForegroundColor Yellow
        Write-Host "  step [3/5] device removal + step [5/5] rescan should clear it." -ForegroundColor Yellow
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
    # the wrong driver (which [5/5] already explained). Either way, the
    # user has nothing more to do locally - point them at the issue
    # tracker so we can investigate (most likely an unsupported PID).
    if ($detectedKinects.Count -eq 0) {
        Write-Host "No Kinect was detected on USB." -ForegroundColor Yellow
    } else {
        Write-Host "A Kinect is detected but at least one device has the wrong" -ForegroundColor Yellow
        Write-Host "driver (see [5/5] above)." -ForegroundColor Yellow
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
    Write-Host "the [5/5] section), plus the output of:"
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
