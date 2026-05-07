# CLAUDE.md — headtracking (VPX plugin, pure Rust)

Ce fichier est lu en priorité par Claude Code à chaque session. Il contient le contexte projet, les invariants architecturaux et les conventions à respecter.

---

## 1. Mission

Plugin de **head tracking temps réel pour Visual Pinball X (VPX) 10.8.1+** qui modifie le POV de la table en live via le **nouveau système de plugin VPX**.

**Objectifs** :
- Cross-platform : Windows, Linux, macOS
- Multi-device : **Kinect v1**, **Kinect v2**, puis webcams (face landmarks)
- Alternative open source à BAM (closed source, Windows-only en pratique)
- Faible latence (< 16 ms idéalement, budget 1 frame à 60 Hz)
- 100 % Rust

**Non-objectifs** :
- VR (déjà géré par VPX nativement)
- Compatibilité Future Pinball (BAM s'en occupe)
- UI de calibration léchée pour le MVP (CLI / fichier toml suffit)

---

## 2. Layout du repo et environnement

```
~/dev/
├── headtracking/                ← CE REPO (working directory)
│   ├── src/
│   │   ├── lib.rs               ← cdylib, exports C-ABI pour VPX
│   │   ├── plugin/              ← integration plugin VPX (msg bus, registration)
│   │   │   ├── mod.rs
│   │   │   ├── ffi.rs           ← #[no_mangle] extern "C" entry points
│   │   │   ├── messages.rs      ← &CStr constants pour les msg IDs VPX
│   │   │   └── vpx_sys.rs       ← include! des bindings bindgen générés
│   │   ├── tracker/             ← trait HeadTracker + backends + session thread
│   │   │   ├── mod.rs
│   │   │   ├── session.rs       ← TrackerSession (thread + ArcSwap<Pose>)
│   │   │   ├── kinect_v1.rs     ← feature = "kinect-v1" (P2)
│   │   │   ├── kinect_v2.rs     ← feature = "kinect-v2" (P1, opérationnel)
│   │   │   └── webcam.rs        ← feature = "webcam" (P3)
│   │   ├── filter/              ← one-euro / Kalman pour lisser le pose
│   │   ├── camera/              ← mapping Pose → ViewSetupDef
│   │   │   ├── mod.rs
│   │   │   ├── mapping.rs       ← Pose → ViewDelta (mm → VPU + axis flip)
│   │   │   └── units.rs         ← MM ↔ VPU (50 VPU = 1.0625" = 26.9875 mm)
│   │   └── calibration/         ← lecture/écriture toml, repère device→VPX
│   ├── crates/                  ← bindings Rust maison (in-tree workspace members)
│   │   ├── freenect2-sys/       ← cxx::bridge sur libfreenect2 (Kinect v2)
│   │   │   ├── build.rs         ← cmake static + cxx-build
│   │   │   ├── src/lib.rs       ← #[cxx::bridge]
│   │   │   ├── src/shim.{h,cpp} ← C++ shim (DepthSink FrameListener)
│   │   │   └── vendor/libfreenect2/  ← submodule, pinned v0.2.1
│   │   ├── freenect2/           ← safe wrapper (Context, Device, ...)
│   │   ├── freenect-sys/        ← bindgen sur libfreenect (Kinect v1, P2)
│   │   │   └── vendor/libfreenect/   ← submodule, pinned v0.7.5
│   │   └── freenect/            ← safe wrapper v1
│   ├── build.rs                 ← bindgen sur les headers plugin VPX
│   ├── wrapper.h                ← header agrégé consommé par bindgen
│   ├── plugin.cfg               ← manifest VPX (id="HeadTracking", platforms)
│   ├── Cargo.toml               ← workspace root + cdylib
│   ├── tools/
│   │   └── ht-calibrate/        ← binaire CLI standalone (pas un plugin)
│   ├── docs/
│   └── CLAUDE.md                ← ce fichier
│
└── vpinball/                    ← READ-ONLY référence (sources VPX)
    ├── plugins/plugins/         ← headers C de l'API plugin (cibles bindgen)
    │   ├── MsgPlugin.h          ← bus de messages, callbacks, threading
    │   ├── VPXPlugin.h          ← VPXPluginAPI, ViewSetupDef, événements
    │   └── LoggingPlugin.h      ← API logging native VPX
    ├── plugins/<example>/       ← plugins de référence (helloworld, b2s, …)
    ├── docs/View Setup.md       ← sémantique caméra / POV
    └── standalone/              ← infos build cross-platform
```

**Règles strictes** :
- `../vpinball/` est en **lecture seule**. Jamais d'édit, jamais de PR à proposer dessus.
- L'exécution de VPX se fait sur le pincab via SSH, **jamais depuis Claude Code**.
- Claude Code ne fait que **build, lint, test**. Pas de `./VPinballX_BGFX`.

---

## 3. Architecture

### 3.1 Couches

```
┌─────────────────────────────────────────────────────┐
│ VPX (host C++)                                      │
│   └── plugin manager (dlopen/LoadLibrary)           │
└─────────────────────┬───────────────────────────────┘
                      │ C ABI (symboles plugin VPX)
┌─────────────────────▼───────────────────────────────┐
│ headtracking.{so,dylib,dll}  (Rust cdylib)          │
│                                                     │
│   src/plugin/ffi.rs                                 │
│     #[no_mangle] extern "C" fn PluginLoad(...)      │
│     #[no_mangle] extern "C" fn PluginUnload(...)    │
│     (signatures exactes selon ../vpinball/plugins/) │
│                ↓ Rust-only à partir d'ici           │
│   src/plugin/   ← dispatch messages, lifecycle      │
│   src/tracker/  ← trait HeadTracker + backends      │
│   src/filter/   ← lissage (one-euro)                │
│   src/camera/   ← Pose → params POV VPX             │
│   src/calibration/                                  │
└─────────────────────────────────────────────────────┘
```

### 3.2 FFI — règles

- **Tout symbole exporté** est dans `src/plugin/ffi.rs`, jamais ailleurs.
- **Convention de nommage des entry points** : VPX appelle `dlsym("<id>PluginLoad")` et `dlsym("<id>PluginUnload")` où `<id>` est exactement la valeur du champ `id` de `plugin.cfg`. Notre `id = "HeadTracking"` ⇒ on exporte `HeadTrackingPluginLoad` et `HeadTrackingPluginUnload` (cf. `~/dev/vpinball/src/plugins/MsgPluginManager.cpp`).
- Chaque `extern "C" fn` exportée est **`catch_unwind`** : un panic ne doit jamais traverser la frontière FFI.
- Toutes les structures partagées avec VPX sont `#[repr(C)]`, jamais `#[repr(Rust)]`.
- Bindings VPX générés par `bindgen` dans `build.rs` à partir de **`../vpinball/plugins/plugins/{MsgPlugin,VPXPlugin,LoggingPlugin}.h`** (chemin réel : sous-dossier `plugins/plugins/`). Le résultat est `include!`-é dans `src/plugin/vpx_sys.rs`. Override possible via `VPX_PLUGINS_DIR`.
- Pas d'`unsafe` en dehors de `plugin/ffi.rs` et `tracker/*` (appels SDK natifs).
- **Cleanup obligatoire au unload** : chaque `GetMsgID` ⇒ `ReleaseMsgID`, chaque `SubscribeMsg` ⇒ `UnsubscribeMsg`. Sinon UB au prochain reload du plugin.

### 3.2bis API VPX cible (depuis l'audit des sources)

- **Subscribe à** `VPX/OnPrepareFrame` pour modifier la POV chaque frame, plus `VPX/OnGameStart` / `VPX/OnGameEnd` pour le cycle de vie.
- **Récupérer `VPXPluginAPI`** via `BroadcastMsg(endpoint, GetMsgID("VPX","GetAPI"), &mut ptr)` au load. Le host répond synchroniquement.
- **Modifier la caméra** : `vpxApi->GetActiveViewSetup(&view)` puis muter `view.viewX/viewY/viewZ` (RW), puis `vpxApi->SetActiveViewSetup(&view)`.
- **Mode VPX recommandé** pour head tracking : `VLM_CAMERA` (position relative au centre bas de la table). `VLM_WINDOW` est aussi conçu pour ça mais moins testé côté upstream.
- **Threading** : tous les appels API sont attendus sur le thread principal (assertion côté VPX). Le tracker tourne sur son propre thread et publie via `ArcSwap<Pose>` ; le callback `OnPrepareFrame` (sur main thread) lit sans bloquer. Pour marshaller l'inverse il y a `MsgPluginAPI::RunOnMainThread`.

### 3.2ter Logging

`LoggingPlugin.h` expose une API host (`LPI_LOGI/W/E/D`) qu'il faut récupérer comme `VPXPluginAPI` (broadcast `Logging/GetAPI`). À utiliser plutôt qu'un fichier indépendant, pour que les logs apparaissent dans la console VPX. Pour l'instant on est encore sur `tracing` → stderr ; bascule vers l'API native = TODO.

### 3.3 Modèle Pose

```rust
#[derive(Debug, Clone, Copy)]
pub struct Pose {
    pub position_mm: [f32; 3],   // repère device (Kinect ou webcam)
    pub timestamp_us: u64,        // monotonic clock
    pub confidence: f32,          // 0.0 = perdu, 1.0 = parfait
}
```

Mapping vers VPX (cf. `../vpinball/docs/View Setup.md`, mode "Camera" recommandé) :
- `Pose.position_mm` (repère Kinect) → delta `(viewX, viewY, viewZ)` en VPU via `src/camera/mapping.rs`
- Conversions VPU↔mm dans `src/camera/units.rs` (50 VPU = 1.0625″ = 26.9875 mm, calculs en `f64` pour matcher la précision du macro `MMTOVPU` de VPX)
- Matrice de calibration plus complète stockée en `~/.config/headtracking/calibration.toml` (Linux/macOS) ou `%APPDATA%\headtracking\calibration.toml` (Windows)

### 3.4 Trait tracker

```rust
pub trait HeadTracker: Send {
    fn poll(&mut self) -> Option<Pose>;
    fn name(&self) -> &'static str;
    fn shutdown(&mut self);
}
```

Le tracker tourne dans un thread dédié, partage le dernier pose via `arc_swap::ArcSwap<Pose>` ou `crossbeam_channel`. Le callback frame VPX lit le dernier pose sans bloquer.

---

## 4. Backends device

Décision : **un seul chemin de code par capteur**, pas d'utilisation des SDK propriétaires Microsoft. On vendore `libfreenect` et `libfreenect2` en submodules et on link statique. Les bindings sont in-tree dans `crates/`.

| Backend     | Cross-platform via         | Statut         | Crate Rust                              |
|-------------|----------------------------|----------------|-----------------------------------------|
| Kinect v1   | `libfreenect` (C API)      | P2 ✅ capture + algo blob | `freenect-sys` (bindgen) + `freenect`   |
| Kinect v2   | `libfreenect2` (C++ API)   | P1 ✅ capture + algo blob | `freenect2-sys` (cxx) + `freenect2`     |
| Webcam      | **SDL3** `SDL_Camera`       | capture ✅, tracker P3 | `webcam` (sdl3-sys + safe wrapper) ; tracker via `rust-faces` ONNX (P3) |

**Stratégie "zéro dep utilisateur final"** :
- libfreenect / libfreenect2 compilés en static via `cmake` dans `build.rs` (CPU pipeline only pour libfreenect2 — pas de GPU dep).
- libjpeg-turbo (requis par libfreenect2 même si on n'utilise pas le RGB) tiré du crate `turbojpeg-sys` feature `cmake` : build static PIC depuis les sources vendorées du crate. Le linker élimine en dead-code l'encodeur JPEG (libfreenect2 n'appelle que `tjInitDecompress` / `tjDecompress2` / `tjGetErrorStr` / `tjDestroy`).
- Lien statique imposé dans `build.rs` **après** `libfreenect2` (les archives `.a` sont scannées une seule fois ; libfreenect2 référence les symboles `tj*` donc libturbojpeg.a doit suivre).
- SDL3 vendoré via `sdl3-sys` features `build-from-source-static` + `sdl-camera` + `sdl-video` **dans `headtracking-demo`**. Aucun dep utilisateur sur SDL3 (qui n'est de toute façon pas encore packagé partout).
- `libusb-1.0` reste linké dynamiquement contre la lib système (universelle sur Linux/macOS).

**Cas particulier SDL3 dans le plugin (P3, à ne pas oublier)** :
- VPX 10.8+ utilise SDL3 lui-même (`find_package(SDL3)` dans son `CMakeLists.txt`, `libSDL3.so` à côté du binaire) ⇒ une copie de SDL3 vit déjà dans le process VPX au moment où notre `cdylib` se charge.
- Si on link SDL3 **statiquement** dans le plugin, on aurait deux SDL3 dans le même process avec deux états globaux (event bus, ref count subsystems, claim V4L2). Recette pour des bugs subtils.
- Plan retenu : au `PluginLoad`, **résoudre les symboles `SDL_*` via `dlsym`** sur `libSDL3.so` déjà chargé par VPX, les stocker dans une struct de pointeurs de fonctions, et router les appels webcam à travers cette struct au lieu du link statique de `sdl3-sys`. La crate `webcam` garde sa surface API actuelle ; on swappe juste le backend FFI.
- Si SDL3 n'est pas encore initialisé au moment du `PluginLoad`, on peut appeler `SDL_Init(SDL_INIT_CAMERA)` nous-mêmes — SDL3 ref-count les subsystems en interne, donc un init multiple est sans danger.
- Pas applicable à `headtracking-demo` (binaire standalone, sans host SDL3) — il garde sa SDL3 statique.

Compilation conditionnelle via features Cargo :

```toml
[features]
default = []
kinect-v1 = []                  # déclenche `dep:freenect` quand le crate sera prêt
kinect-v2 = ["dep:freenect2"]   # opérationnel
all-trackers = ["kinect-v1", "kinect-v2"]
```

Activation runtime via la config (toml) : un seul backend actif à la fois pour le MVP.

---

## 5. Build

### Pré-requis

- Rust stable récent (édition 2024, testé 1.95)
- `bindgen` requiert `libclang` (paquet `libclang1-X` ou `clang`). `build.rs` cherche le sysroot dans plusieurs emplacements et tombe sur les headers GCC en dernier recours.
- `cmake` ≥ 3.20
- Selon backend :
  - Linux : `libusb-1.0-0-dev`
  - macOS : `brew install libusb`
  - Windows : `cargo-xwin` (cible MSVC) ; libusb via vcpkg
- **Pas besoin** d'installer `libfreenect-dev` / `libfreenect2-dev` / `libturbojpeg0-dev` ni de SDK Microsoft : tout est vendoré.
  - libfreenect / libfreenect2 : submodules dans `crates/*-sys/vendor/`
  - libjpeg-turbo : via le crate [`turbojpeg-sys`](https://crates.io/crates/turbojpeg-sys) feature `cmake` (build statique PIC à partir de sources bundled)
- **Cloner avec submodules** : `git clone --recurse-submodules ...` ou `git submodule update --init` après-coup.

### Runtime utilisateur (côté pincab)

Pour exécuter le `.so`/`.dll`/`.dylib` : aucune dépendance applicative à installer. Les seules `NEEDED` sur Linux sont les libs systèmes universelles : `libusb-1.0`, `libstdc++`, `libgcc_s`, `libm`, `libc`, plus `libudev` / `libcap` via libusb. Toutes déjà présentes par défaut sur Ubuntu/Debian/Fedora/Arch.

### Commandes

```bash
# Build release
cargo build --release --features kinect-v2

# Build avec tous les backends
cargo build --release --features all-trackers

# Tests
cargo test --all-features

# Lint (zéro warning attendu)
cargo clippy --all-features --all-targets -- -D warnings
cargo fmt --check
```

### Cibles & sortie

| Cible              | Output                       |
|--------------------|------------------------------|
| Linux x86_64       | `target/release/libheadtracking.so` |
| macOS aarch64      | `target/release/libheadtracking.dylib` |
| macOS x86_64       | idem (lipo si universal binary plus tard) |
| Windows x86_64     | `target/release/headtracking.dll` |

Cross-compilation possible via `cross` ou `cargo-xwin` (Windows depuis Linux).

### Déploiement (manuel, Sylvain)

Le binaire produit est copié dans le dossier de plugins de l'install VPX cible :
```
<VPX_install>/plugins/headtracking/headtracking.{so,dylib,dll}
<VPX_install>/plugins/headtracking/plugin.cfg   # manifest VPX, format à confirmer
```

---

## 6. Sources d'information à consulter dans `../vpinball/`

À lire **avant** d'écrire le moindre code FFI :
- `../vpinball/plugins/plugins/` — exemples de plugins existants, c'est le matériel de référence n°1
- `../vpinball/plugins/*.h` — headers C de l'API plugin (cibles `bindgen`)
- `../vpinball/docs/View Setup.md` — sémantique POV (Player X/Y/Z, Look At, Window mode)
- `../vpinball/docs/Plugin*.md` (s'ils existent) — doc officielle plugin
- `../vpinball/standalone/README.md` — contexte build cross-platform

Quand un détail d'API est ambigu, **lire le code de VPX est la source de vérité**, pas la doc.

---

## 7. Conventions de code

- **Édition** : Rust 2024
- **Formatage** : `rustfmt` par défaut, pas de surcharge
- **Lint** : `clippy::pedantic` activé sauf `module_name_repetitions`, `missing_errors_doc`
- **`unsafe`** : autorisé uniquement dans `src/plugin/ffi.rs` et `src/tracker/*`. Chaque bloc `unsafe` doit être commenté avec `// SAFETY: …`
- **Erreurs** : `thiserror` pour les erreurs typées du crate, pas d'`anyhow` dans `lib.rs` (on est une lib). `anyhow` toléré uniquement dans `tools/`.
- **Logging** : `tracing` (pas `log`), avec subscriber initialisé une seule fois côté plugin lors du `PluginLoad`. Sortie vers fichier dans le dossier de logs VPX par défaut.
- **Pas de `println!`** dans le code plugin (volerait stdout de VPX).
- **Commits** : conventional commits (`feat:`, `fix:`, `refactor:`, `docs:`, `chore:`).

---

## 8. Tests

- Unitaires Rust : tout module pur (filter, camera, calibration) doit être testé.
- Backends device : mockables via le trait `HeadTracker`. Un backend `MockTracker` est fourni pour les tests d'intégration.
- Pas de test qui requiert un device physique en CI. Tests device-réel marqués `#[ignore]` et lancés manuellement.
- Cible coverage : > 70 % sur `filter/`, `camera/`, `calibration/`.

---

## 9. Roadmap

### MVP (P1)
- [ ] Bindings `bindgen` sur l'API plugin VPX, plugin "hello world" qui se charge
- [ ] Trait `HeadTracker` + backend `MockTracker` (oscillation sinusoïdale)
- [ ] Mapping Pose → POV VPX, vérification visuelle sur table simple
- [ ] Backend Kinect v2 (Linux d'abord via libfreenect2, puis Windows)
- [ ] Filtre one-euro
- [ ] Outil `ht-calibrate` (CLI) : capture 4 points de référence → matrice toml

### P2
- [ ] Backend Kinect v1
- [ ] Backend Webcam (ONNX face mesh)
- [ ] UI calibration in-game (overlay via API plugin si possible)
- [ ] Profils par table (override dans `<table>.headtracking.toml`)
- [ ] Compatibilité protocole BAM (lecture du shared memory existant comme backend)

### P3
- [ ] Multi-camera fusion
- [ ] Prédiction (compensation latence affichage)

---

## 10. Décisions d'architecture (ADR-style, en bref)

| Décision | Raison |
|----------|--------|
| Pure Rust `cdylib`, pas de shim C | Build plus simple, un seul Cargo.toml, FFI maîtrisé |
| `bindgen` à chaque build (pas de bindings vendorés) | API plugin VPX encore WIP en 10.8.x, on suit l'amont |
| Un thread tracker + `ArcSwap` pour partager le pose | Découple I/O device de la cadence frame VPX |
| Backend choisi par config, pas auto-détection au MVP | Détection device fragile, on simplifie |
| Calibration toml à plat (pas de DB) | Lisible, diff-able, partageable sur le forum VPX |

---

## 11. Notes pour Claude Code

- Avant toute modif FFI : relire `../vpinball/plugins/plugins/` et confirmer que la signature exportée matche exactement.
- Si une dépendance Cargo n'est pas dispo dans le miroir interne (cas Sylvain au taf, mais ici c'est perso donc OK), le signaler.
- Ne jamais ajouter de dépendance lourde sans valider (pas de `tokio` pour 3 lignes d'async, pas de `reqwest` pour un GET, etc.).
- Quand tu hésites entre deux approches, **propose les deux** dans la réponse plutôt que de trancher seul.
- Ce projet est sous Linux pour le dev, déployé sur le pincab Linux + (à terme) Windows. Ne pas casser la compat Windows par flemme.
