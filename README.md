# headtracking

[🇬🇧 English](#english) · [🇫🇷 Français](#français)

Real-time head tracking POV plugin for **Visual Pinball X 10.8.1+**, in pure Rust.
Drives the table's POV live from a **Kinect v1**, **Kinect v2** or **webcam**,
through VPX's new plugin system. Open-source alternative to BAM,
cross-platform (Linux, Windows, macOS).

- Repo: <https://github.com/Le-Syl21/headtracking>
- License: GPL-3.0-or-later
- Status: **beta (0.0.3) — early development, nothing tested end-to-end yet**

> ⚠ **Heads-up — work in progress.** Nothing has been physically wired up and
> tested against a running VPX install yet. The plugin builds on every target
> and the trackers run standalone (`headtracking-demo`), but the
> install/configure flow described below is the *intended* one and may still
> be wrong in places. Bug reports very welcome.

---

## English

### Install the plugin

Grab the binary for your OS from the Releases page and drop it into VPX's
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

1. Launch VPX once after copying the files. The plugin manager picks up the
   new `plugin.cfg` automatically.
2. Open **Preferences → Plugins**, find **Head Tracking** in the list, tick
   **Enable**, then restart VPX. The plugin only loads on next start, not
   live.
3. The plugin's settings appear in the same dialog (one row per setting).
   Under the hood they're persisted in `VPinballX.ini` under
   `[Plugin.HeadTracking]` — you can also edit that file by hand:

   ```ini
   [Plugin.HeadTracking]
   Backend             = 0     ; 0=Auto, 1=Kinect v2, 2=Kinect v1, 3=Webcam
   DeviceIndex         = 0     ; 0-based, ignored for Kinect v2
   Gain                = 1.0   ; multiplier applied to head delta → camera
   MinCutoffHz         = 0.4   ; 1€ filter baseline cutoff (lower = smoother when still)
   Beta                = 0.05  ; 1€ filter response to fast motion (higher = less lag)
   BaselineOffsetX     = 0.0   ; correction (mm) on the captured neutral pose
   BaselineOffsetY     = 0.0
   BaselineOffsetZ     = 0.0
   LockbarHandSpan     = 660.0 ; mm between hands on flipper buttons (~660 widebody)
   LockbarFloorHeight  = 850.0 ; mm from floor to top of lockbar (850 widebody)
   IPDmm               = 63.0  ; interpupillary distance — 63 adult mean
   ```

   On Linux this lives at `~/.vpinball/VPinballX.ini`, on Windows at
   `%APPDATA%\VPinballX\VPinballX.ini`, on macOS at
   `~/Library/Application Support/VPinballX/VPinballX.ini`.

4. **In-table view setup**: open a table, hit **F6** (or whichever key your
   build maps to *View Setup*), and pick the **Camera** view layout mode.
   Head tracking only steers the camera in that mode (`VLM_CAMERA`); in
   *Legacy* it has no effect. Re-center yourself in front of the
   sensor/webcam *before* loading the table — that pose is captured as the
   neutral baseline. `BaselineOffsetX/Y/Z` lets you nudge the baseline
   afterwards without recapturing.

5. Open the VPX log console; on plugin load you should see something like
   `kinect2 backend: 1 device(s) detected` (or v1 / webcam equivalents).
   No line → driver/permission issue, see runtime requirements below.

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
or v2 in and Device Manager shows it as *Unknown / Other device* — there
is no device path for libusb to enumerate, so the plugin reports zero
devices detected. **You must install something before anything works.**

There are two distinct things you may install, and they solve different
problems:

| Driver | Makes the device... visible to libusb? | …openable by libusb? |
|---|---|---|
| **None** (out of the box) | ❌ no | ❌ no |
| **Microsoft Kinect SDK driver** alone | ✅ yes | ❌ **`open` fails with rc=-12 / `LIBUSB_ERROR_NOT_SUPPORTED`** |
| **UsbDk filter** (any other driver may coexist) | ✅ yes | ✅ yes |
| **libusbK** via Zadig (replaces existing driver) | ✅ yes | ✅ yes |

If you've already installed the Microsoft SDK driver, you'll see the
Kinect listed when the plugin enumerates, but **opening it will fail with
error -12** (this is libusb saying *"another kernel driver owns this
device, I can't talk to it"*). That's expected — install UsbDk or replace
the driver via Zadig, below, to actually use it.

##### Option 1 — One-click setup script (recommended)

The Windows release ZIP includes `setup/cleanup-kinect.cmd` — a
double-clickable launcher that handles the full Windows side in one
shot:

1. (Windows 7 only) install
   [Microsoft Security Advisory 3033929](https://learn.microsoft.com/en-us/security-updates/securityadvisories/2015/3033929)
   first or USB keyboards/mice may stop working.
2. Open the extracted release folder, navigate to `setup\`, and
   **double-click `cleanup-kinect.cmd`**. Confirm the UAC prompt.
3. Wait for the script to finish — it will:
   - download + install Daynix's signed UsbDk MSI (skipped if
     already installed, ~3.5 MB pull from GitHub Releases over HTTPS);
   - block Windows from auto-installing partial drivers on the
     Kinect VID/PIDs (registry deny list, survives reboots);
   - unbind any current Kinect device instance so PnP re-discovers
     them cleanly on next plug.

Plug the Kinect, restart VPX.

If you have a **Kinect v1**, you also need to run [Zadig](https://zadig.akeo.ie/)
once after the script: enable *Options → List All Devices*, untick
*Ignore Hubs or Composite Parents*, then for each *Xbox NUI Audio /
Camera / Motor* pick **WinUSB** on the right and click *Replace
Driver*. The script's deny list ensures Windows won't undo your
WinUSB binding the next time you replug.

(Don't want to run a script? You can install UsbDk by hand from
<https://github.com/daynix/UsbDk/releases>. The plugin auto-detects
its presence at startup — missing-UsbDk shows up as a `WARN` line in
VPX's plugin log and a popup with a clickable link in the
`headtracking-demo` standalone tool.)

UsbDk coexists with the Kinect for Windows v2 SDK — no need to uninstall
anything. It handles the Kinect v2 entirely on its own.

##### Option 2 — libusbK via Zadig (alternative)

Zadig **replaces** whatever driver currently owns the device with
libusbK. Use this if UsbDk misbehaves on your machine, or if you don't
want to keep the Microsoft SDK around.

1. Get Zadig from <https://zadig.akeo.ie/>.
2. Run Zadig. In **Options**, tick **List All Devices** and untick
   **Ignore Hubs or Composite Parents**.
3. Pick from the dropdown:
   - Kinect v2 → **Xbox NUI Sensor (composite parent)** (USB ID `045E:02C4`
     or `045E:02D8`). ⚠ Skip **NuiSensor Adaptor**, that's the power brick,
     not the sensor.
   - Kinect v1 → install libusbK for **each** of the three sub-devices:
     *Xbox NUI Motor*, *Xbox NUI Audio*, *Xbox NUI Camera*.
4. On the right, pick **libusbK (v3.0.7.0 or newer)** from the replacement
   driver list.
5. Click **Replace Driver**. Confirm the warning (composite parent system
   driver).

**To revert to the Microsoft SDK driver**: Device Manager → libusbK USB
Devices → right-click the device → *Uninstall*, tick *Delete the driver
software for this device*, then Action → *Scan for hardware changes*.

##### Verification

Launch VPX, open the plugin console (or check the log): on PluginLoad you
should see `kinect2 backend: 1 device(s) detected` (or v1 equivalent).
- *Nothing detected* → no driver is registered for the device. Install
  the Kinect SDK driver (or just UsbDk, which is simpler).
- *Detected, but `OpenFailed (rc=-12)` / `LIBUSB_ERROR_NOT_SUPPORTED`* →
  only the SDK driver is in play. Install UsbDk **or** run Zadig as
  described above.
- *Still nothing on a v2 specifically* → replug the Kinect into a
  **dedicated USB 3.0 port** (v2 needs the bandwidth of a root port,
  not a shared hub).

##### Triage logs

For the deeper "Device Manager lists it but it still won't open" cases
the standalone `headtracking-demo` exposes libfreenect's own USB
trace. Two env vars combined:

```cmd
set FREENECT_LOG_LEVEL=spew
set HEADTRACKING_LOG=libfreenect=debug,info
headtracking-demo.exe
```

`FREENECT_LOG_LEVEL` (`fatal`/`error`/`warning`/`notice`/`info`/
`debug`/`spew`/`flood`) tunes libfreenect's verbosity; `HEADTRACKING_LOG`
is the standard `tracing` filter routing those lines through. Every
libfreenect log line shows up with a `libfreenect:` prefix in the
in-app log panel and in the stderr stream — easy to grep / paste in
a bug report.

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

## Français

### Installation du plugin

Récupérer le binaire correspondant à votre OS depuis la page Releases, puis le
déposer dans le dossier des plugins de votre install VPX :

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

1. Lancer VPX une fois après avoir copié les fichiers. Le plugin manager
   prend en compte le nouveau `plugin.cfg` automatiquement.
2. Ouvrir **Preferences → Plugins**, trouver **Head Tracking** dans la
   liste, cocher **Enable**, puis redémarrer VPX. Le plugin n'est chargé
   qu'au prochain démarrage, pas à chaud.
3. Les réglages du plugin apparaissent dans le même dialogue (une ligne par
   réglage). Ils sont persistés dans `VPinballX.ini` sous
   `[Plugin.HeadTracking]` — vous pouvez aussi éditer ce fichier à la main :

   ```ini
   [Plugin.HeadTracking]
   Backend             = 0     ; 0=Auto, 1=Kinect v2, 2=Kinect v1, 3=Webcam
   DeviceIndex         = 0     ; 0-based, ignoré pour la Kinect v2
   Gain                = 1.0   ; multiplicateur appliqué au delta tête → caméra
   MinCutoffHz         = 0.4   ; cutoff de base du filtre 1€ (plus bas = plus lissé au repos)
   Beta                = 0.05  ; réponse 1€ au mouvement rapide (plus haut = moins de lag)
   BaselineOffsetX     = 0.0   ; correction (mm) de la pose neutre capturée
   BaselineOffsetY     = 0.0
   BaselineOffsetZ     = 0.0
   LockbarHandSpan     = 660.0 ; mm entre les mains sur les boutons (~660 widebody)
   LockbarFloorHeight  = 850.0 ; mm du sol au sommet du lockbar (850 widebody)
   IPDmm               = 63.0  ; distance interpupillaire — 63 adulte moyen
   ```

   Sur Linux le fichier est à `~/.vpinball/VPinballX.ini`, sur Windows à
   `%APPDATA%\VPinballX\VPinballX.ini`, sur macOS à
   `~/Library/Application Support/VPinballX/VPinballX.ini`.

4. **View Setup en table** : ouvrir une table, presser **F6** (ou la
   touche que votre build mappe sur *View Setup*) et choisir le mode
   **Camera**. Le head tracking ne pilote la caméra que dans ce mode
   (`VLM_CAMERA`) ; en *Legacy* il n'a aucun effet. Se recentrer face au
   capteur/webcam **avant** de charger la table — cette pose est capturée
   comme baseline neutre. `BaselineOffsetX/Y/Z` permet d'ajuster la
   baseline ensuite sans tout recapturer.

5. Ouvrir la console plugin de VPX ; au PluginLoad on doit voir une ligne
   du genre `kinect2 backend: 1 device(s) detected` (ou v1 / webcam
   équivalent). Rien → souci driver/permissions, voir la section runtime
   plus bas.

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
*Périphérique inconnu* — il n'y a pas de chemin device pour libusb à
énumérer, donc le plugin ne détecte rien. **Il faut installer quelque
chose avant que ça démarre.**

Deux choses différentes peuvent être installées, et elles résolvent des
problèmes distincts :

| Driver | Visible par libusb ? | Ouvrable par libusb ? |
|---|---|---|
| **Aucun** (sortie d'usine) | ❌ non | ❌ non |
| **Driver SDK Kinect Microsoft** seul | ✅ oui | ❌ **`open` échoue avec rc=-12 / `LIBUSB_ERROR_NOT_SUPPORTED`** |
| **UsbDk** (filter, peut coexister avec un autre driver) | ✅ oui | ✅ oui |
| **libusbK** via Zadig (remplace le driver existant) | ✅ oui | ✅ oui |

Si vous avez déjà installé le driver SDK Microsoft, vous verrez la
Kinect remontée à l'énumération du plugin, mais **l'ouverture échouera
avec l'erreur -12** (libusb dit *« un autre driver kernel possède ce
device, je ne peux pas lui parler »*). C'est attendu — installer UsbDk
ou remplacer le driver via Zadig (ci-dessous) pour vraiment l'utiliser.

##### Option 1 — Script setup tout-en-un (recommandé)

Le ZIP release Windows inclut `setup/cleanup-kinect.cmd` — un
launcher double-cliquable qui gère toute la mise en place en un seul
geste :

1. (Windows 7 uniquement) installer d'abord
   [Microsoft Security Advisory 3033929](https://learn.microsoft.com/en-us/security-updates/securityadvisories/2015/3033929)
   sinon les claviers/souris USB peuvent cesser de fonctionner.
2. Ouvrir le dossier release extrait, aller dans `setup\`, et
   **double-cliquer sur `cleanup-kinect.cmd`**. Confirmer l'UAC.
3. Le script va :
   - télécharger + installer le MSI UsbDk signé Daynix (skippé s'il
     est déjà installé, ~3.5 Mo depuis GitHub Releases en HTTPS) ;
   - bloquer Windows pour qu'il n'installe plus de drivers partiels
     sur les VID/PID Kinect (registre, survit aux reboots) ;
   - délier toute instance Kinect actuelle pour que PnP ré-énumère
     proprement au prochain plug.

Brancher la Kinect, relancer VPX.

Pour une **Kinect v1**, il faut aussi lancer [Zadig](https://zadig.akeo.ie/)
une fois après le script : activer *Options → List All Devices*,
décocher *Ignore Hubs or Composite Parents*, puis pour chaque *Xbox
NUI Audio / Camera / Motor* choisir **WinUSB** à droite et cliquer
*Replace Driver*. La deny-list posée par le script empêche Windows
de défaire ton binding WinUSB au prochain replug.

(Tu ne veux pas lancer un script ? Tu peux installer UsbDk à la main
depuis <https://github.com/daynix/UsbDk/releases>. Le plugin détecte
sa présence au démarrage — UsbDk absent = ligne `WARN` dans le log
plugin VPX + popup avec lien cliquable dans l'outil standalone
`headtracking-demo`.)

UsbDk coexiste avec le SDK Kinect for Windows v2 — pas besoin de
désinstaller quoi que ce soit. Il règle entièrement le cas Kinect v2
de lui-même.

##### Option 2 — libusbK via Zadig (alternative)

Zadig **remplace** le driver qui possède actuellement le device par
libusbK. À utiliser si UsbDk se comporte mal sur votre machine, ou si
vous ne voulez pas garder le SDK Microsoft.

1. Télécharger Zadig depuis <https://zadig.akeo.ie/>.
2. Lancer Zadig. Dans **Options**, cocher **List All Devices** et décocher
   **Ignore Hubs or Composite Parents**.
3. Sélectionner dans la liste déroulante :
   - Kinect v2 → **Xbox NUI Sensor (composite parent)** (USB ID `045E:02C4`
     ou `045E:02D8`). ⚠ Ignorer **NuiSensor Adaptor**, qui est l'adaptateur
     secteur, pas la Kinect.
   - Kinect v1 → installer libusbK pour **chacun** des trois
     sous-périphériques : *Xbox NUI Motor*, *Xbox NUI Audio*, *Xbox NUI
     Camera*.
4. À droite, choisir **libusbK (v3.0.7.0 ou plus récent)** dans la liste
   des drivers de remplacement.
5. Cliquer sur **Replace Driver**. Confirmer le warning (driver système
   composite).

**Pour revenir au driver SDK Microsoft** : Device Manager → libusbK USB
Devices → clic droit sur le device → *Uninstall*, cocher *Delete the
driver software for this device*, puis Action → *Scan for hardware
changes*.

##### Vérification

Lancer VPX, ouvrir la console plugin (ou regarder le log) : au PluginLoad
on doit voir la ligne `kinect2 backend: 1 device(s) detected` (ou
équivalent v1).
- *Rien n'est détecté* → aucun driver enregistré pour le device.
  Installer le driver SDK Kinect (ou directement UsbDk, plus simple).
- *Détecté, mais `OpenFailed (rc=-12)` / `LIBUSB_ERROR_NOT_SUPPORTED`* →
  seul le driver SDK est en place. Installer UsbDk **ou** lancer Zadig
  comme décrit ci-dessus.
- *Toujours rien sur une v2 spécifiquement* → rebrancher la Kinect sur
  un **port USB 3.0 dédié** (la v2 demande la bande passante d'un port
  racine, pas un hub partagé).

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
