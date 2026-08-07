# Install & configure headtracking

> Detailed install, VPX configuration, per-OS runtime setup (Kinect USB
> rules) and build-from-source notes. The [main README](../README.md) has
> the overview; this is the reference.

### Install the plugin

Grab the binary for your OS from the
[Releases page](https://github.com/Le-Syl21/headtracking/releases) — or, for
the freshest build, from the artifacts of any `main`-branch run on the
[Actions tab](https://github.com/Le-Syl21/headtracking/actions/workflows/release.yml)
(every commit builds the plugin + demo for all platforms; dev builds are
**unsigned** and need a GitHub login to download). Drop it into VPX's
plugin folder:

```
<VPX_install>/plugins/headtracking/
├── plugin.cfg
├── headtracking.dll          # Windows
├── libheadtracking.so        # Linux (x86_64 or aarch64)
└── libheadtracking.dylib     # macOS (arm64 or x86_64)
```

Both files (the binary **and** `plugin.cfg`) must live in the same folder —
VPX scans `plugins/<id>/plugin.cfg` to discover plugins.

### Enable & configure inside VPX

1. Launch VPX (10.8.1+) once after copying the files — the plugin manager
   discovers the new `plugin.cfg` automatically.
2. Load any table and press **F12** (Toggle In-Game UI) → **Plugin
   Settings** → **Head Tracking**, then tick **Enable**. Quit and reload
   the table: the tracker starts at game start.
3. Every setting lives on that same F12 page and applies **live** while
   you play — except **Backend** and **Camera**, which are read at game
   start (reload the table after changing them). Under the hood they're
   persisted in `VPinballX.ini` under `[Plugin.HeadTracking]`:

   ```ini
   [Plugin.HeadTracking]
   Enable          = 1
   Backend         = 0     ; 0=Auto (Kinect v2 → v1 → Webcam), 1=Kinect v2, 2=Kinect v1, 3=Webcam
   DeviceIndex     = 0     ; which webcam — the in-game dropdown shows real device names
   Gain            = 1.0   ; multiplier on the head-motion delta (0.5 is a good cab start)
   Smoothing       = 0     ; 0=Stable (field-tested default), 1=Normal, 2=Reactive
   MedianWindow    = 3     ; frames of spike-killing median pre-filter (1 = off)
   TrackingStream  = 0     ; 0=Auto (IR on Kinects — tracks in a dark room), 1=Color
   InvertX         = 0     ; flip left/right for mirrored / unusual mountings
   InvertY         = 0
   InvertZ         = 0
   WebcamFocalPx   = 0.0   ; webcam focal length in pixels, 0 = automatic
   BaselineOffsetX = 0.0   ; trim (mm) on the captured neutral head position
   BaselineOffsetY = 0.0
   BaselineOffsetZ = 0.0
   ```

   The ini lives in VPX's preferences folder: Linux
   `~/.local/share/VPinballX/10.8/VPinballX.ini`, Windows
   `%AppData%\VPinballX\10.8\VPinballX.ini`, macOS
   `~/Library/Application Support/VPinballX/10.8/VPinballX.ini`.
   Only edit it while VPX is **closed** — VPX rewrites the whole file on
   exit.

4. **View setup** — the plugin's startup notification reminds you of all
   of this:
   * **F12 → Cabinet Settings**: measure and enter your **lockbar width**
     and your **screen inclination** — auto-calibration and the eye
     mapping are anchored on those two values.
   * Table POV: pick the **Window** view layout with **rotation 0** and
     enable the **cabinet autofit** mode. Window is the mode designed for
     head tracking: the screen becomes a fixed window into the cabinet
     and the fish-tank effect happens inside the table.
   * Stand in your normal play position when the table loads — the first
     stable pose is captured as the neutral baseline
     (`BaselineOffsetX/Y/Z` nudges it afterwards without recapturing).

5. On game start the plugin pushes an on-screen notification with the
   detected camera and the calibration state, and the VPX log gets
   `HeadTracking` lines (camera enumeration, backend, anchor detection).
   No notification → driver/permission issue, see runtime requirements
   below.

### Runtime requirements per OS

#### Linux — udev rules for libusb access

`libfreenect` / `libfreenect2` reach the Kinect through `libusb`. Without
udev rules, only `root` can open the device — VPX will silently fail. Copy
the rules shipped by the upstream libs (already vendored as submodules in
this repo):

##### Kinect v2

```bash
sudo tee /etc/udev/rules.d/90-kinect2.rules > /dev/null <<'EOF'
# Microsoft Kinect v2 (Xbox One) — open USB access for libfreenect2
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c4", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d8", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d9", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then **unplug and replug** the Kinect (rules apply on attach).

##### Kinect v1

```bash
sudo tee /etc/udev/rules.d/51-kinect.rules > /dev/null <<'EOF'
# Microsoft Kinect v1 (Xbox 360) — open USB access for libfreenect
# Xbox NUI Motor / Audio / Camera
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
# Kinect for Windows (v1 variant)
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02be", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Same: unplug/replug.

##### Verification

```bash
lsusb | grep -i microsoft           # Kinect must show up
ls -l /dev/bus/usb/<bus>/<dev>      # MODE should be crw-rw-rw-
```

Best functional check: run `headtracking-demo` — the camera list should
show your Kinect and the video panel should stream. While you're there,
click **🎁 Contribute**: one capture of your cab trains the
auto-calibration model, and since the detector learns the *cabinet*
(lockbar + rails), an empty cab is exactly as useful — no need to stand
in frame.

##### Webcam

On most distros your user is already in the `video` group (`/dev/video0` is
`crw-rw----+ root:video`). If not:

```bash
sudo usermod -aG video "$USER"   # logout/login required afterwards
```

##### System libraries

No application dependency to install — `libfreenect`, `libfreenect2` and
`libjpeg-turbo` are statically linked. Only `libusb-1.0`, `libstdc++`,
`libgcc_s` and the libc are required, all present by default on
Debian/Ubuntu/Fedora/Arch.

#### Windows — libusb drivers for the Kinect

**Webcam-only** users have nothing to do (capture goes through SDL3 /
Media Foundation, not libusb). The rest of this section is Kinect-specific.

##### What Windows does out of the box (= nothing useful)

Tested 2026-05 on a fresh Windows pincab: even with all Windows Updates
installed, **the Kinect ships with no driver at all**. Plug a Kinect v1
or v2 in and Device Manager shows it as *Unknown / Other device* — no
device path for libusb to enumerate, plugin reports zero devices.
**You must install something before anything works.**

The release ZIP solves this in one click — see Option 1 below.

##### Option 1 — One-click setup from the demo (recommended)

The Windows release ZIP includes a `setup/` folder with the WinUSB
installer script and a fresh build of `headtracking-demo.exe` that
knows how to launch it elevated:

1. (Windows 7 only) install
   [Microsoft Security Advisory 3033929](https://learn.microsoft.com/en-us/security-updates/securityadvisories/2015/3033929)
   first or USB keyboards/mice may stop working.
2. Plug in the Kinect, then double-click `headtracking-demo.exe`. If
   no usable driver is bound, the app shows a yellow banner
   *"⚠ Kinect plugged in but not accessible"* with an **Install
   Kinect drivers (UAC prompt)** button.
3. Click the button, confirm the UAC prompt. The PowerShell window
   that opens shows a warning summary and asks you to **type `yes`**
   to proceed — this is destructive for **BAM** head tracking and
   any other software that depends on the Microsoft Kinect SDK
   runtime (see the warning text for details). Type anything else
   to abort without touching the system.
4. Wait for the script to finish (~10–30 s), then hit **rescan** in
   the demo's toolbar.

That's it for both Kinect v1 and v2. Plug the Kinect, restart VPX.

> 💡 **While the demo is open and the camera streaming**, click
> **🎁 Contribute** — one capture of your cab becomes training data for
> the auto-calibration model. The detector learns the *cabinet* (lockbar
> + side rails), so an empty cab is exactly as useful as a played one:
> no need to stand in frame.

> ⚠ **Coexists with BAM?** No. BAM relies on the Microsoft Kinect
> for Windows v2 SDK runtime, and this script removes that driver
> in favour of WinUSB. If you want to go back to BAM afterwards,
> reinstall the MS Kinect SDK runtime — there's no automated
> rollback.

Prefer the terminal? Open an elevated PowerShell and run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File setup\setup.ps1
```

(The `-ExecutionPolicy Bypass` matters — a fresh Windows defaults to
`Restricted`/`RemoteSigned` and would otherwise refuse to run an
unsigned local script.)

What the script does, in order:

| Step | Action |
|---|---|
| 1 | Removes any currently-attached Kinect device instance from the system and **deletes every Kinect driver package from the Driver Store** — legacy Microsoft Kinect SDK runtime, leftover Zadig output, and our own kinect_v[12]_*.inf from previous runs. Cleans up any obsolete `DenyDeviceIDs` registry entries left by older versions of this script (they used to block PnP rebinding but caused FAILEDINSTALL more often than they helped). |
| 2 | `pnputil /add-driver /install` on the 9 WinUSB INF/CAT packages in `setup\drivers\` — covers every known Kinect VID/PID. The bundled `.cat` files are signed by libwdi's self-signed cert, which the script pre-trusts in the certificate stores so `pnputil` doesn't prompt or fail under HVCI / Memory Integrity. |
| 3 | Clears `CONFIGFLAG_FAILEDINSTALL` on any Kinect device still flagged as problem 28 by a previous failed run — PnP refuses to retry binding otherwise. |
| 4 | `pnputil /scan-devices` re-enumerates USB so devices removed in step 1 (and unblocked in step 3) get re-bound to our freshly-installed WinUSB INF without a physical replug — useful when the Kinect is hard-mounted in a pinball cabinet. |

After the script: every Kinect interface is bound to **WinUSB** (the
Microsoft inbox kernel driver, signed at the kernel level — HVCI /
Memory Integrity safe). libusb claims them through its WinUSB
backend, libfreenect/libfreenect2 enumerate normally, no Zadig
required, no UsbDk required.

Re-running the script is idempotent: missing devices are silently
skipped, `pnputil /add-driver` is a no-op when the same INF is
already in the Driver Store at the same DriverVer.

##### Verification

Launch `headtracking-demo.exe` (or VPX with the plugin loaded). The
backend dropdown / plugin log should list the Kinect within a second
of opening.

If it doesn't:
* Open Device Manager (`devmgmt.msc`) → look under *Universal Serial
  Bus devices*. Your Kinect's interfaces should show as
  `kinect_v1_*` or `kinect_v2_*` with `WinUSB Device` in the *Driver*
  tab.
* If the entries still appear with a yellow `?` (no driver), the
  script's `pnputil /add-driver` step failed — the elevated
  PowerShell window shows the `[FAIL]` lines; copy them when
  reporting the issue.
* Set `FREENECT_LOG_LEVEL=spew` and
  `HEADTRACKING_LOG=libfreenect=debug,info` before launching the demo
  to surface libfreenect's full USB transcript via `tracing`.

##### Option 2 — Manual Zadig fallback

If you'd rather not run a script, or the script failed for some
reason (locked-down corporate Windows, code-signing policy, etc.),
you can bind WinUSB by hand with [Zadig](https://zadig.akeo.ie/).

1. Get Zadig (single ~5 MB executable). **Right-click → Run as
   administrator**.
2. *Options* → **check** *List All Devices*, **uncheck** *Ignore
   Hubs or Composite Parents*.
3. For **Kinect v1**: bind WinUSB to all three sub-devices —
   *Xbox NUI Audio*, *Xbox NUI Camera*, *Xbox NUI Motor*.
   For **Kinect v2**: bind WinUSB to *Xbox NUI Sensor (composite
   parent)* (USB ID `045E:02C4` or `045E:02D8`). ⚠ Skip *NuiSensor
   Adaptor* — that's the power brick, not the sensor.
4. On the right side of the Zadig window, leave the default
   `WinUSB (v6.x.x.x)` and click **Replace Driver**.

**To revert** any of these bindings: Device Manager → right-click
the device → *Uninstall device* → tick *Delete the driver software
for this device* → *Scan for hardware changes*.

##### v2 needs USB 3.0 root-port bandwidth

If you have a Kinect v2 and `setup.ps1` ran clean but the demo still
doesn't see it, replug into a **dedicated USB 3.0 port** on the
motherboard's back panel — the v2 needs the bandwidth of a root
port and won't enumerate behind a shared hub.

#### macOS

`libusb` works on macOS without an extra driver. Drop the `.dylib` into
VPX's plugin folder and plug the Kinect — done. On Apple Silicon, the
aarch64 build ships natively (no Rosetta).

For webcam access, allow VPX in **System Settings → Privacy & Security →
Camera** on first launch.

### Building from source

See `CLAUDE.md` for the full picture. Short version:

```bash
git clone --recurse-submodules https://github.com/Le-Syl21/headtracking
cd headtracking
cargo build --release                                            # all backends
cargo build --release --no-default-features --features kinect-v2 # one backend
```

Prerequisites: recent stable Rust (edition 2024), `cmake` ≥ 3.20, `libclang`
(for `bindgen`). libusb is vendored as a submodule and built statically
by `libusb-sys` on **all three** platforms — no `apt install
libusb-1.0-0-dev`, no `brew install libusb`, no vcpkg. SDL3 build deps
(only needed for the standalone demo's static SDL3) are still
distro-managed: see the workflow `release.yml` for the exhaustive list.

Native libs (libfreenect, libfreenect2, libjpeg-turbo, libusb) are all
vendored as submodules or via `turbojpeg-sys` and built **statically** —
zero external runtime dep on the end user's machine across Linux,
macOS, and Windows.

### Credits

- [libfreenect](https://github.com/OpenKinect/libfreenect) — Kinect v1 driver
  (Apache 2.0 / GPL 2.0)
- [libfreenect2](https://github.com/OpenKinect/libfreenect2) — Kinect v2
  driver (Apache 2.0 / GPL 2.0)
- [libjpeg-turbo](https://libjpeg-turbo.org/) (BSD-3)
- [SDL3](https://github.com/libsdl-org/SDL) (Zlib) — webcam capture
- [Visual Pinball X](https://github.com/vpinball/vpinball) — host, and the
  reference we follow for the plugin API

---


---

### Installation du plugin

Récupérer le binaire correspondant à votre OS depuis la
[page Releases](https://github.com/Le-Syl21/headtracking/releases) — ou, pour
la version la plus fraîche, depuis les artefacts de n'importe quel run de la
branche `main` dans l'onglet
[Actions](https://github.com/Le-Syl21/headtracking/actions/workflows/release.yml)
(chaque commit compile plugin + démo pour toutes les plateformes ; les dev
builds sont **non signées** et demandent un compte GitHub pour le
téléchargement). Puis le déposer dans le dossier des plugins de votre
install VPX :

```
<VPX_install>/plugins/headtracking/
├── plugin.cfg
├── headtracking.dll          # Windows
├── libheadtracking.so        # Linux (x86_64 ou aarch64)
└── libheadtracking.dylib     # macOS (arm64 ou x86_64)
```

Les deux fichiers (le binaire **et** `plugin.cfg`) doivent vivre dans le même
dossier — VPX scanne `plugins/<id>/plugin.cfg` pour découvrir les plugins.

### Activation et configuration dans VPX

1. Lancer VPX (10.8.1+) une fois après avoir copié les fichiers — le plugin
   manager découvre le nouveau `plugin.cfg` automatiquement.
2. Charger une table et presser **F12** (Toggle In-Game UI) → **Plugin
   Settings** → **Head Tracking**, puis cocher **Enable**. Quitter et
   recharger la table : le tracker démarre au lancement de la partie.
3. Tous les réglages vivent sur cette même page F12 et s'appliquent **en
   live** pendant le jeu — sauf **Backend** et **Camera**, lus au démarrage
   de la partie (recharger la table après les avoir changés). Sous le
   capot, tout est persisté dans `VPinballX.ini` sous
   `[Plugin.HeadTracking]` :

   ```ini
   [Plugin.HeadTracking]
   Enable          = 1
   Backend         = 0     ; 0=Auto (Kinect v2 → v1 → Webcam), 1=Kinect v2, 2=Kinect v1, 3=Webcam
   DeviceIndex     = 0     ; quelle webcam — la dropdown in-game affiche les vrais noms
   Gain            = 1.0   ; multiplicateur sur le delta tête (0.5 = bon départ sur cab)
   Smoothing       = 0     ; 0=Stable (défaut éprouvé), 1=Normal, 2=Reactive
   MedianWindow    = 3     ; frames de médiane anti-pics avant le filtre (1 = off)
   TrackingStream  = 0     ; 0=Auto (IR sur Kinect — tracke dans le noir), 1=Color
   InvertX         = 0     ; inverse gauche/droite (montage caméra atypique)
   InvertY         = 0
   InvertZ         = 0
   WebcamFocalPx   = 0.0   ; focale webcam en pixels, 0 = automatique
   BaselineOffsetX = 0.0   ; correction (mm) de la pose neutre capturée
   BaselineOffsetY = 0.0
   BaselineOffsetZ = 0.0
   ```

   L'ini vit dans le dossier de préférences VPX : Linux
   `~/.local/share/VPinballX/10.8/VPinballX.ini`, Windows
   `%AppData%\VPinballX\10.8\VPinballX.ini`, macOS
   `~/Library/Application Support/VPinballX/10.8/VPinballX.ini`.
   Ne l'éditer que VPX **fermé** — VPX réécrit tout le fichier en
   quittant.

4. **View setup** — la notification de démarrage du plugin rappelle tout
   ça :
   * **F12 → Cabinet Settings** : mesurer et renseigner la **largeur de
     lockbar** et l'**inclinaison de l'écran** — l'auto-calibration et le
     mapping de l'œil sont ancrés sur ces deux valeurs.
   * POV de table : choisir le layout **Window** avec **rotation 0** et
     activer le mode **cabinet autofit**. Window est le mode conçu pour le
     head tracking : l'écran devient une fenêtre fixe sur le cab et
     l'effet fish-tank se produit dans la table.
   * Se placer en position de jeu normale au chargement de la table — la
     première pose stable est capturée comme baseline neutre
     (`BaselineOffsetX/Y/Z` l'ajuste ensuite sans recapturer).

5. Au lancement de la partie, le plugin affiche une notification à l'écran
   avec la caméra détectée et l'état de calibration, et le log VPX reçoit
   des lignes `HeadTracking` (énumération caméras, backend, détection de
   l'ancre). Pas de notification → souci driver/permissions, voir les
   sections runtime plus bas.

### Pré-requis runtime par plateforme

#### Linux — règles udev pour accès USB sans root

`libfreenect` / `libfreenect2` accèdent aux Kinect via `libusb`. Sans règles
udev, seul `root` peut ouvrir le device — VPX échouera silencieusement. Il
faut copier les règles fournies par les libs upstream (déjà présentes dans
les submodules de ce repo) :

##### Kinect v2

```bash
sudo tee /etc/udev/rules.d/90-kinect2.rules > /dev/null <<'EOF'
# Microsoft Kinect v2 (Xbox One) — accès USB libre pour libfreenect2
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c4", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d8", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d9", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Puis **débrancher / rebrancher** la Kinect (les règles s'appliquent à
l'attachement).

##### Kinect v1

```bash
sudo tee /etc/udev/rules.d/51-kinect.rules > /dev/null <<'EOF'
# Microsoft Kinect v1 (Xbox 360) — accès USB libre pour libfreenect
# Xbox NUI Motor / Audio / Camera
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
# Kinect for Windows (variante v1)
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02be", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Idem : débrancher/rebrancher.

##### Vérification

```bash
lsusb | grep -i microsoft           # doit montrer la Kinect
ls -l /dev/bus/usb/<bus>/<dev>      # MODE doit valoir crw-rw-rw-
```

Meilleur test fonctionnel : lancer `headtracking-demo` — la liste des
caméras doit montrer la Kinect et le panneau vidéo doit streamer. Tant
qu'on y est, cliquer **🎁 Contribute** : un relevé de votre cab entraîne
le modèle d'auto-calibration, et comme le détecteur apprend le *cab*
(lockbar + rails), un cab vide est exactement aussi utile — pas besoin
d'être dans le champ.

##### Webcam

Sur la majorité des distros, l'utilisateur courant est dans le groupe `video`
(`/dev/video0` est en `crw-rw----+ root:video`). Si ce n'est pas le cas :

```bash
sudo usermod -aG video "$USER"   # logout/login requis ensuite
```

##### Bibliothèques système

Aucune dépendance applicative à installer (libfreenect, libfreenect2 et
libjpeg-turbo sont linkés statiquement dans le `.so`). Seules `libusb-1.0`,
`libstdc++`, `libgcc_s` et la libc sont requises — elles sont présentes par
défaut sur Debian/Ubuntu/Fedora/Arch.

#### Windows — drivers libusb pour la Kinect

Si vous utilisez **uniquement la webcam**, rien à faire : la capture
passe par SDL3 / Media Foundation, pas par libusb. Le reste de cette
section concerne la Kinect.

##### Comportement Windows par défaut (= rien d'utilisable)

Testé en mai 2026 sur un pincab Windows neuf : même avec toutes les
mises à jour Windows, **la Kinect arrive sans aucun driver**. Brancher
une Kinect v1 ou v2 et le Gestionnaire de périphériques affiche
*Périphérique inconnu* — pas de chemin device pour libusb à énumérer,
le plugin ne détecte rien. **Il faut installer quelque chose avant
que ça démarre.**

Le ZIP release règle ça en un clic — voir Option 1 ci-dessous.

##### Option 1 — Installation en un clic depuis la démo (recommandé)

Le ZIP release Windows inclut un dossier `setup/` avec le script
d'installation WinUSB et un `headtracking-demo.exe` qui sait le
lancer élevé :

1. (Windows 7 uniquement) installer d'abord
   [Microsoft Security Advisory 3033929](https://learn.microsoft.com/en-us/security-updates/securityadvisories/2015/3033929)
   sinon les claviers/souris USB peuvent cesser de fonctionner.
2. Brancher la Kinect, puis double-cliquer sur
   `headtracking-demo.exe`. Si aucun driver utilisable n'est bindé,
   l'appli affiche une bannière jaune *« ⚠ Kinect plugged in but
   not accessible »* avec un bouton **Install Kinect drivers (UAC
   prompt)**.
3. Cliquer le bouton, confirmer l'UAC. La fenêtre PowerShell qui
   s'ouvre affiche un résumé d'avertissement et te demande de
   **taper `yes`** pour confirmer — l'opération est destructive
   pour **BAM** et tout logiciel qui dépend du runtime Microsoft
   Kinect SDK (détails dans le texte d'avertissement). Taper autre
   chose annule sans rien toucher.
4. Attendre la fin du script (~10–30 s), puis cliquer **rescan**
   dans la barre d'outils de la démo.

C'est tout, pour les deux Kinect v1 et v2. Brancher la Kinect, relancer VPX.

> 💡 **Tant que la démo est ouverte et la caméra en flux**, cliquer
> **🎁 Contribute** — un relevé de votre cab devient donnée
> d'entraînement pour le modèle d'auto-calibration. Le détecteur apprend
> le *cab* (lockbar + rails latéraux), donc un cab vide est exactement
> aussi utile qu'une partie en cours : pas besoin d'être dans le champ.

> ⚠ **Cohabite avec BAM ?** Non. BAM dépend du runtime Microsoft
> Kinect for Windows v2 SDK, et le script remplace ce driver par
> WinUSB. Pour revenir à BAM ensuite, il faut réinstaller le
> runtime MS Kinect SDK à la main — pas de rollback automatique.

Tu préfères le terminal ? Ouvrir un PowerShell administrateur et lancer :

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File setup\setup.ps1
```

(Le `-ExecutionPolicy Bypass` est important — Windows neuf est par
défaut en `Restricted`/`RemoteSigned` et refuserait sinon un script
local non signé.)

Ce que fait le script, dans l'ordre :

| Étape | Action |
|---|---|
| 1 | Retire toute instance Kinect actuellement attachée et **supprime tous les packages de driver Kinect du Driver Store** — runtime Microsoft Kinect SDK legacy, output Zadig précédent, et nos propres kinect_v[12]_*.inf des runs antérieurs. Nettoie aussi les entrées `DenyDeviceIDs` obsolètes laissées par les anciennes versions du script (elles bloquaient le re-binding PnP mais causaient FAILEDINSTALL plus souvent qu'elles n'aidaient). |
| 2 | `pnputil /add-driver /install` sur les 9 paquets WinUSB INF/CAT dans `setup\drivers\` — couvre tous les VID/PID Kinect connus. Les `.cat` bundlés sont signés par le certificat self-signé libwdi, que le script pré-trust dans les magasins de certificats pour que `pnputil` ne prompte pas ni n'échoue sous HVCI / Memory Integrity. |
| 3 | Efface `CONFIGFLAG_FAILEDINSTALL` sur toute Kinect encore flaggée problème 28 par un run précédent échoué — sinon PnP refuse de réessayer le bind. |
| 4 | `pnputil /scan-devices` ré-énumère USB pour que les devices retirés à l'étape 1 (et débloqués à l'étape 3) soient re-bindés à notre INF WinUSB fraîchement installé sans replug physique — utile quand la Kinect est câblée à demeure dans un pincab. |

Après le script : chaque interface Kinect est bindée à **WinUSB** (le
driver kernel inbox Microsoft, signé au niveau kernel — compatible
HVCI / Memory Integrity). libusb les claim via son backend WinUSB,
libfreenect/libfreenect2 les énumèrent normalement, **pas de Zadig
requis, pas de UsbDk requis**.

Re-lancer le script est idempotent : devices manquants silencieusement
skippés, `pnputil /add-driver` ne fait rien quand le même INF est
déjà dans le Driver Store au même DriverVer.

##### Vérification

Lancer `headtracking-demo.exe` (ou VPX avec le plugin chargé). La
dropdown backend / le log plugin doit lister la Kinect en moins
d'une seconde après ouverture.

Si pas le cas :
* Ouvrir Device Manager (`devmgmt.msc`) → regarder sous *Universal
  Serial Bus devices*. Les interfaces Kinect doivent apparaître
  comme `kinect_v1_*` ou `kinect_v2_*` avec `WinUSB Device` dans
  l'onglet *Driver*.
* Si elles affichent encore un `?` jaune (pas de driver), le step
  `pnputil /add-driver` du script a échoué — la fenêtre PowerShell
  élevée affiche les lignes `[FAIL]`, les copier pour le rapport
  d'incident.
* Mettre `FREENECT_LOG_LEVEL=spew` et
  `HEADTRACKING_LOG=libfreenect=debug,info` dans l'environnement
  avant de lancer le demo pour avoir le transcript USB complet de
  libfreenect via `tracing`.

##### Option 2 — Zadig manuel (fallback)

Si tu ne veux pas lancer un script, ou si le script a échoué pour
une raison ou une autre (Windows verrouillé en entreprise, policy
de signature, etc.), tu peux binder WinUSB à la main via
[Zadig](https://zadig.akeo.ie/).

1. Récupérer Zadig (exécutable unique de ~5 Mo). **Clic droit →
   Exécuter en tant qu'administrateur**.
2. *Options* → **cocher** *List All Devices*, **décocher** *Ignore
   Hubs or Composite Parents*.
3. Pour la **Kinect v1** : binder WinUSB sur les trois
   sous-périphériques — *Xbox NUI Audio*, *Xbox NUI Camera*,
   *Xbox NUI Motor*.
   Pour la **Kinect v2** : binder WinUSB sur *Xbox NUI Sensor
   (composite parent)* (USB ID `045E:02C4` ou `045E:02D8`).
   ⚠ Ignorer *NuiSensor Adaptor* — c'est l'adaptateur secteur, pas
   le sensor.
4. Côté droit de la fenêtre Zadig, laisser le `WinUSB (v6.x.x.x)`
   par défaut et cliquer **Replace Driver**.

**Pour revenir en arrière** sur un binding : Device Manager → clic
droit sur le device → *Uninstall device* → cocher *Delete the
driver software for this device* → *Scan for hardware changes*.

##### v2 demande la bande passante d'un port USB 3.0 racine

Si tu as une Kinect v2 et que `setup.ps1` est passé sans erreur
mais le demo ne la voit toujours pas, rebrancher sur un **port USB
3.0 dédié** au dos de la carte mère — la v2 a besoin de la bande
passante d'un port racine et n'énumère pas derrière un hub partagé.

#### macOS

`libusb` fonctionne sans driver supplémentaire sur macOS. Aucune étape
spécifique : poser le `.dylib` dans le dossier plugins de VPX et brancher
la Kinect. Sur Apple Silicon, la build aarch64 est livrée nativement (pas
de Rosetta).

Pour la webcam, autoriser VPX à accéder à la caméra dans **System
Settings → Privacy & Security → Camera** au premier lancement.

### Build depuis les sources

Voir `CLAUDE.md` pour le détail. Résumé :

```bash
git clone --recurse-submodules https://github.com/Le-Syl21/headtracking
cd headtracking
cargo build --release                                            # tous les backends
cargo build --release --no-default-features --features kinect-v2 # un seul
```

Pré-requis : Rust stable récent (édition 2024), `cmake` ≥ 3.20, `libclang`
(pour `bindgen`). libusb est vendoré en submodule et compilé
statiquement par `libusb-sys` sur **les trois** plateformes — pas de
`apt install libusb-1.0-0-dev`, pas de `brew install libusb`, pas de
vcpkg. Les deps SDL3 (pour la build statique SDL3 du demo standalone
uniquement) restent gérées par la distro — voir le workflow
`release.yml` pour la liste complète.

Les libs natives (libfreenect, libfreenect2, libjpeg-turbo, libusb)
sont toutes vendorées en submodules ou via `turbojpeg-sys` et
compilées **statiquement** — zéro dep externe au runtime côté
utilisateur, sur Linux, macOS et Windows.

### Crédits

- [libfreenect](https://github.com/OpenKinect/libfreenect) — driver Kinect
  v1 (Apache 2.0 / GPL 2.0)
- [libfreenect2](https://github.com/OpenKinect/libfreenect2) — driver
  Kinect v2 (Apache 2.0 / GPL 2.0)
- [libjpeg-turbo](https://libjpeg-turbo.org/) (BSD-3)
- [SDL3](https://github.com/libsdl-org/SDL) (Zlib) — capture webcam
- [Visual Pinball X](https://github.com/vpinball/vpinball) — l'host, et la
  référence qu'on suit pour l'API plugin
