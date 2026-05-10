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
or v2 in and Device Manager shows it as *Unknown / Other device* — no
device path for libusb to enumerate, plugin reports zero devices.
**You must install something before anything works.**

The release ZIP solves this in one click — see Option 1 below.

##### Option 1 — One-click setup script (recommended)

The Windows release ZIP includes a `setup/` folder with a
double-clickable launcher that wires up the entire Windows USB stack
for both Kinect generations:

1. (Windows 7 only) install
   [Microsoft Security Advisory 3033929](https://learn.microsoft.com/en-us/security-updates/securityadvisories/2015/3033929)
   first or USB keyboards/mice may stop working.
2. Open the extracted release folder, navigate to `setup\`, and
   **double-click `setup-kinect.cmd`**. Confirm the UAC prompt.
3. Wait for the script to finish (~10–30 s).

That's it for both Kinect v1 and v2. Plug the Kinect, restart VPX.

What the script does, in order:

| Step | Action |
|---|---|
| 1 | Adds `DenyDeviceIDs` registry entries for every Kinect v1 sub-device PID (Audio / Camera / Motor across 1414, 1473, KfW revisions) so Windows PnP stops re-binding `usbaudio.sys` between replugs. |
| 2 | Same for Kinect v2 sensor PIDs, so Windows Update can't silently re-bind a Microsoft driver later. The hub PID (`02D9`) is deliberately *not* denied — it must keep `usbhub.sys` so the sensor can be enumerated downstream. |
| 3 | Removes any currently-attached Kinect instance and deletes any leftover Kinect drivers from the Driver Store (legacy SDK, prior Zadig output, prior runs of this script). |
| 4 | Pre-trusts the libwdi self-signed certificate that signs the bundled `.cat` files (so `pnputil` won't prompt or fail under HVCI), then `pnputil /add-driver /install` on the 9 WinUSB INF packages in `setup\drivers\` — covers every known Kinect VID/PID. |
| 5 | `pnputil /scan-devices` re-enumerates USB so already-plugged Kinects pick up WinUSB without a physical replug — useful when the Kinect is hard-mounted in a pinball cabinet. |

After the script: every Kinect interface is bound to **WinUSB** (the
Microsoft inbox kernel driver, signed at the kernel level — HVCI /
Memory Integrity safe). libusb claims them through its WinUSB
backend, libfreenect/libfreenect2 enumerate normally, no Zadig
required, no UsbDk required.

Re-running the script is idempotent: registry entries are wiped and
rewritten, missing devices are silently skipped, `pnputil
/add-driver` is a no-op when the same INF is already in the Driver
Store at the same DriverVer.

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
  script's `pnputil /add-driver` step failed — open `setup\dont_run.ps1`
  output (visible in the script's console window) and look for `[FAIL]`
  lines.
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

If you have a Kinect v2 and the `setup-kinect.cmd` script ran
clean but the demo still doesn't see it, replug into a **dedicated
USB 3.0 port** on the motherboard's back panel — the v2 needs the
bandwidth of a root port and won't enumerate behind a shared hub.

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
*Périphérique inconnu* — pas de chemin device pour libusb à énumérer,
le plugin ne détecte rien. **Il faut installer quelque chose avant
que ça démarre.**

Le ZIP release règle ça en un clic — voir Option 1 ci-dessous.

##### Option 1 — Script setup tout-en-un (recommandé)

Le ZIP release Windows inclut un dossier `setup/` avec un launcher
double-cliquable qui câble toute la stack USB Windows pour les deux
générations de Kinect :

1. (Windows 7 uniquement) installer d'abord
   [Microsoft Security Advisory 3033929](https://learn.microsoft.com/en-us/security-updates/securityadvisories/2015/3033929)
   sinon les claviers/souris USB peuvent cesser de fonctionner.
2. Ouvrir le dossier release extrait, aller dans `setup\`, et
   **double-cliquer sur `setup-kinect.cmd`**. Confirmer l'UAC.
3. Attendre que le script termine (~10–30 s).

C'est tout, pour les deux Kinect v1 et v2. Brancher la Kinect, relancer VPX.

Ce que fait le script, dans l'ordre :

| Étape | Action |
|---|---|
| 1 | Ajoute des entrées `DenyDeviceIDs` au registre pour chaque PID Kinect v1 sous-périphérique (Audio / Camera / Motor sur révs 1414, 1473, KfW), pour empêcher Windows PnP de re-binder `usbaudio.sys` entre les replugs. |
| 2 | Idem pour les PIDs Kinect v2 sensor, pour que Windows Update ne re-bind pas silencieusement un driver Microsoft plus tard. Le PID hub (`02D9`) est délibérément *non* dans la deny-list — il doit garder `usbhub.sys` pour pouvoir énumérer le sensor downstream. |
| 3 | Retire toute instance Kinect actuellement attachée et supprime tout driver Kinect résiduel du Driver Store (legacy SDK, output Zadig précédent, runs antérieurs de ce script). |
| 4 | Pré-trust le certificat self-signé libwdi qui signe les `.cat` bundlés (sinon `pnputil` prompte ou échoue sous HVCI), puis `pnputil /add-driver /install` sur les 9 INFs WinUSB dans `setup\drivers\` — couvre tous les VID/PID Kinect connus. |
| 5 | `pnputil /scan-devices` ré-énumère USB pour que les Kinect déjà branchées prennent WinUSB sans replug physique — utile quand la Kinect est câblée à demeure dans un pincab. |

Après le script : chaque interface Kinect est bindée à **WinUSB** (le
driver kernel inbox Microsoft, signé au niveau kernel — compatible
HVCI / Memory Integrity). libusb les claim via son backend WinUSB,
libfreenect/libfreenect2 les énumèrent normalement, **pas de Zadig
requis, pas de UsbDk requis**.

Re-lancer le script est idempotent : entrées registre wipées et
réécrites, devices manquants silencieusement skippés, `pnputil
/add-driver` ne fait rien quand le même INF est déjà dans le Driver
Store au même DriverVer.

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
  `pnputil /add-driver` du script a échoué — ouvrir la fenêtre
  console du script (qui reste affichée) et chercher les lignes
  `[FAIL]`.
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

Si tu as une Kinect v2 et que `setup-kinect.cmd` est passé sans
erreur mais le demo ne la voit toujours pas, rebrancher sur un
**port USB 3.0 dédié** au dos de la carte mère — la v2 a besoin
de la bande passante d'un port racine et n'énumère pas derrière
un hub partagé.

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
