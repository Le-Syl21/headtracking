# Plugin port plan — demo pipeline → VPX cdylib

Status: agreed 2026-08-06. Source audits: the VPX-side integration report
(plugins API, settings, POV surface) and the plugin-side gap analysis
(23 items), both summarized below as decisions.

VPX facts this plan builds on (checked against vpinball @ 2026-08):
- `VPX.OnPrepareFrame` is the sanctioned per-frame hook (the helloworld
  sample names head tracking as its use case). `SetActiveViewSetup`
  writes ONLY `viewX/Y/Z` and triggers `InitLayout()` itself.
- `VPXViewSetupDef` exposes read-only `viewMode`, `screenWidth/Height/
  Inclination` (the VPX host screen geometry settings) and
  `realToVirtualScale` — the plugin READS these; it must never ask the
  user for what VPX already knows.
- Settings: `RegisterSetting` (host pushes value into the plugin's
  setter; there is no GetSetting) → `[Plugin.HeadTracking]` in
  VPinballX.ini + native in-game UI with LIVE edits and per-table
  override. `SaveSetting` exists for plugin-computed persistence
  (nobody uses it upstream yet — we will, for the measured webcam
  focal).
- The plugin manager lives in `Player`: the plugin is loaded/unloaded
  per game. No static state may assume process lifetime.
- The API asserts main-thread; the tracker stays on its own thread and
  publishes via ArcSwap (already our design).
- `DisableStaticPrerendering(1)` is required while the camera moves
  (the in-game POV page does the same).
- Upstream gaps (candidate patches later, they asked for volunteers in
  the changelog): playerPos↔viewPos helper not exposed (Window mode),
  `interpupillaryDistance` dead, `VRRecenter` action unmapped, FOV and
  offsets not writable.

## Phase 0 — foundations & risk removal

- **ort packaging decision: static.** blazepose+anchor pull ort
  rc.13 with `download-binaries` static libs; the demo + CI matrix
  already prove static onnxruntime on all 4 targets. The cdylib grows
  to tens of MB — accepted. Verify no symbol leakage from the MODULE
  (default hidden visibility + our 2 exports only).
- Refresh `third_party/vpx-plugin-headers` to current upstream (log
  signature already 5-arg on our side).
- Sync `plugin.cfg` version with Cargo (0.0.26+).

## Phase 1 — core pipeline (the big one)

Replace the tract-era detection with the demo's validated pipeline in
`src/tracker/`:

- All backends: BlazePose (ort) with the glabella head point,
  detect-once-then-track; per-axis one-euro (Z tighter) applied on the
  tracker thread; live parameter reload from settings each loop.
- Kinect v2: IR-first (`start_streams(false, true)`, autolevel gray16 →
  rgb888 into BlazePose, depth sampled directly on the IR grid — no
  registration); colour fallback uses `Registration::bigdepth` with the
  colour intrinsics and the row offset; fix the X mirror (the demo
  negates X on the colour path — the plugin currently ships mirrored).
- Kinect v1: IR stream via `set_video_stream(Ir)` (RGB fallback),
  sample the native u16 depth without full-frame widening, correct
  intrinsics (RGB 525 vs IR 580 — currently ~10 % biased).
- Webcam: BlazePose + shoulder-width Z (the IOD triangulation and the
  dead hand-fiducial path go away).
- Delete: `head` and `face` crates from the plugin deps (tract leaves
  the plugin → unblocks Windows ARM later), `face_depth.rs`,
  `hand_fiducial.rs`, depth-based `detect_lockbar`.

## Phase 2 — calibration

- Port the AnchorWorker (throttled ~400 ms, warmup, freeze-on-lock).
- `anchor_fixed.json` loader; search order: next to the plugin library
  (`<VPX>/plugins/headtracking/`), then `VPXInfo.prefPath`.
- `anchor::camera_pose` as diagnostics first (not in the control loop),
  surfaced via `PushNotification` ("anchor locked — camera 1.30 m from
  the lockbar…").

## Phase 3 — VPX integration

- Read `screenInclination` (+ screen geometry) from the ViewSetupDef —
  no user setting for it; apply the 90°−incline axis rotation from the
  demo in `camera/mapping.rs`.
- Mode-aware mapping: `VLM_CAMERA` (current path) and `VLM_WINDOW`
  (the cab/headtracking mode) — reimplement the internal
  `SetViewPosFromPlayerPosition` math since only table-frame
  `viewX/Y/Z` is writable.
- `DisableStaticPrerendering(1)` on game start when tracking is live.
- Recenter: `VPXACTION_VRRecenter` is unmapped upstream → own trigger:
  auto-baseline (first stable pose) + a configurable action via
  `OnActionChanged`, plus filter reset on recenter.
- Loss-of-tracking policy: ease back toward baseline instead of
  freezing the last offset.
- Status via notifications (tracker up, anchor locked, device busy).

## Phase 4 — user configuration (the agreed surface)

Level 1 (what most users may touch):
- `Backend`: auto / kinect-v2 / kinect-v1 / webcam (auto = v2 > v1 >
  webcam, hwlock arbitrates).
- `Gain`: float, default 1.0.
- `Smoothing`: enum preset stable / normal / reactive → per-axis 1€
  parameter sets (raw values not exposed; BAM lesson: one knob).

Level 2 (advanced):
- `TrackingStream`: auto / rgb (auto = IR on Kinects).
- `InvertX/Y/Z`: bools, default false (defaults field-validated).
- `WebcamFocalPx`: 0 = auto (playfield-rectangle homography when it
  lands; measured value persisted via `SaveSetting`), else manual.
- `LockbarWidthMm`: default 610 — kept as a plugin setting until the
  exact VPX host source is confirmed (screen geometry is host-side;
  whether it can stand in for the lockbar metric is an open point).

Read from VPX, never asked: table incline, screen geometry,
`realToVirtualScale`, view mode.

Removed settings: `IPDmm`, `LockbarHandSpan`, `LockbarFloorHeight`
(dead paths). `BaselineOffsetX/Y/Z` stay (recenter trim).

## Phase 5 — validation

- CI matrix builds the cdylib already; deploy to the cab's VPX 10.8.1,
  live table test (the demo stays the lab bench).
- Perf budget: BlazePose ~7 ms on the tracker thread; main thread only
  swaps an Arc — well inside a 60 Hz frame.
- The known-good numbers to reproduce: camera pose vs tape measure,
  IR 30 fps in the dark, glabella stability at rest.

## Out of scope (tracked, later)

- Upstream patches: expose playerPos↔viewPos, wire `VRRecenter`, IPD
  read-write, Window-mode distortion compensation.
- Windows ARM re-enable (falls out of tract removal).
- Multi-camera fusion, prediction.
