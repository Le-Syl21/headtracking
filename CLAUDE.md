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
│   │   │   └── messages.rs      ← parsing/dispatch des messages VPX
│   │   ├── tracker/             ← trait HeadTracker + backends
│   │   │   ├── mod.rs
│   │   │   ├── kinect_v1.rs     ← feature = "kinect-v1"
│   │   │   ├── kinect_v2.rs     ← feature = "kinect-v2"
│   │   │   └── webcam.rs        ← feature = "webcam"
│   │   ├── filter/              ← one-euro / Kalman pour lisser le pose
│   │   ├── camera/              ← mapping pose → POV VPX (Player X/Y/Z)
│   │   └── calibration/         ← lecture/écriture toml, repère device→VPX
│   ├── build.rs                 ← bindgen sur headers VPX
│   ├── Cargo.toml
│   ├── tools/
│   │   └── ht-calibrate/        ← binaire CLI standalone (pas un plugin)
│   ├── docs/
│   └── CLAUDE.md                ← ce fichier
│
└── vpinball/                    ← READ-ONLY référence (sources VPX)
    ├── plugins/plugins/         ← exemples de plugins existants à étudier
    ├── plugins/*.h              ← headers C de l'API plugin (cibles bindgen)
    ├── docs/View Setup.md       ← sémantique caméra / POV
    ├── docs/Plugin*.md          ← docs API plugin (à lire avant tout dev)
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
- Chaque `extern "C" fn` exportée est **`catch_unwind`** : un panic ne doit jamais traverser la frontière FFI.
- Toutes les structures partagées avec VPX sont `#[repr(C)]`, jamais `#[repr(Rust)]`.
- Bindings VPX générés par `bindgen` dans `build.rs` à partir de `../vpinball/plugins/*.h`. Le résultat est `include!`-é dans un module `vpx_sys`.
- Pas d'`unsafe` en dehors de `plugin/ffi.rs` et `tracker/*` (appels SDK natifs).

### 3.3 Modèle Pose

```rust
#[derive(Debug, Clone, Copy)]
pub struct Pose {
    pub position_mm: [f32; 3],   // repère device (Kinect ou webcam)
    pub timestamp_us: u64,        // monotonic clock
    pub confidence: f32,          // 0.0 = perdu, 1.0 = parfait
}
```

Mapping vers VPX (cf. `../vpinball/docs/View Setup.md`, mode "Window projection") :
- `Pose.position_mm` × matrice calibration → `Player X/Y/Z`
- Matrice calibration stockée en `~/.config/headtracking/calibration.toml` (Linux/macOS) ou `%APPDATA%\headtracking\calibration.toml` (Windows)

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

| Backend     | Windows                       | Linux              | macOS              | Statut |
|-------------|-------------------------------|--------------------|--------------------|--------|
| Kinect v1   | Kinect SDK 1.8 (driver auto)  | libfreenect        | libfreenect        | MVP P1 |
| Kinect v2   | Kinect SDK 2.0 (driver auto)  | libfreenect2       | libfreenect2       | MVP P1 |
| Webcam      | ONNX (DirectML EP)            | ONNX (CPU/CUDA EP) | ONNX (CoreML EP)   | P2     |

**Note Windows** : les drivers Kinect sont fournis automatiquement via Windows Update depuis la mise en libre des SDK officiels. Pas besoin de packager les drivers.

**Webcam (P2)** : face landmarks via `ort` crate (ONNX Runtime). Candidats de modèle : MediaPipe Face Mesh exporté ONNX, ou modèle léger type SynergyNet / 3DDFA.

Compilation conditionnelle via features Cargo :

```toml
[features]
default = ["kinect-v2"]
kinect-v1 = ["dep:freenect"]
kinect-v2 = ["dep:libfreenect2-sys"]
webcam    = ["dep:ort", "dep:nokhwa"]
all-trackers = ["kinect-v1", "kinect-v2", "webcam"]
```

Activation runtime via la config (toml) : un seul backend actif à la fois pour le MVP.

---

## 5. Build

### Pré-requis

- Rust stable récent (édition 2024)
- `bindgen` requiert `libclang` (paquet `clang` / `llvm` / `libclang-dev`)
- Selon backend :
  - Linux : `libfreenect-dev`, `libfreenect2-dev`, `libusb-1.0-0-dev`
  - macOS : `brew install libfreenect libfreenect2`
  - Windows : Kinect SDK 1.8 et/ou 2.0 installés

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
