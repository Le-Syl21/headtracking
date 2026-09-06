//! `headtracking-demo`: standalone validation harness for the head-tracker
//! pipeline.
//!
//! Detects connected Kinect v1 / v2 sensors and webcams, exposes a dropdown
//! to pick the active input. The centre pane shows the live RGB feed with
//! the head bbox (SCUT-HEAD nano) overlaid; the bottom panel splits into a tracing
//! log on the left and the VPX-style view delta on the right.
//!
//! Run with `cargo run --release -p headtracking-demo`.

// Suppress the inherited Windows console for release builds — tracing
// writes to `headtracking-demo.log` next to the binary AND to the
// in-app log panel, so the console window adds nothing and only
// confuses end users. Debug builds keep stderr → console for the dev
// loop (`cargo run`).
#![cfg_attr(
    all(target_os = "windows", not(debug_assertions)),
    windows_subsystem = "windows"
)]

use std::collections::VecDeque;
use std::io::{self, IsTerminal as _, Write};
use std::sync::Arc;
use std::sync::mpsc;
use std::time::{Duration, Instant};

use std::num::NonZeroU32;

use arc_swap::ArcSwapOption;

use egui::{
    self, Align, CentralPanel, Color32, ColorImage, ComboBox, Layout, Panel, Pos2, Rect, RichText,
    ScrollArea, Sense, Stroke, TextureHandle, Vec2,
};
// `glow` comes from `egui_glow`, which re-exports it (`pub use glow;`), rather
// than being declared ourselves. Two crates naming the same GL bindings must
// agree on the version or the types are different types — and the version is
// egui's to choose, not ours. Taking it from there makes a mismatch
// impossible instead of merely unlikely, and stops `cargo update` reporting a
// glow bump we are not free to take.
use egui_glow::glow;
use egui_rotate::{Rotation, RotationPlugin, SoftwareCursor};
use egui_winit::winit;
use headtracking::plugin::logging::DEFAULT_LOG_FILTER;
use nalgebra::Matrix4;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicUsize, Ordering};

use parking_lot::{Condvar, Mutex};
use tracing::{error, info, warn};
use tracing_subscriber::fmt::MakeWriter;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;

// DISPLAY range only (turbo colormap normalisation) — head-distance
// sampling has no plausibility window any more, see
// `headtracking::tracker::pipeline` for the rationale.
const DEPTH_MIN_MM: f32 = 500.0;
const DEPTH_MAX_MM: f32 = 2_500.0;
/// Nominal webcam focal length as a fraction of the frame width (~58° HFOV).
/// One shared guess: the same value MUST be used everywhere a webcam focal is
/// assumed, or derived quantities (head Z vs lockbar Z) silently mix scales —
/// three call sites used to disagree (0.85 vs 0.9), a ~6 % systematic bias on
/// the head↔lockbar delta. Lockbar autocalibration will replace this with a
/// measured per-camera focal.
const WEBCAM_FX_PER_WIDTH: f32 = 0.9;

const LOG_BUFFER_LINES: usize = 1_000;

mod contribute;
mod perf_table;
mod usb_check;

/// Privacy-notice bullets for the Share-a-capture window (title, body).
const CONTRIB_TERMS: &[(&str, &str)] = &[
    (
        "What is sent",
        "one image per stream your sensor has -- colour, infrared, depth -- plus a \
         diagnostics log: the app version, your OS, the sensor model, its USB speed \
         and how many devices share its controller (a count, never their names), the \
         frame-rate measurements, and -- so that one cabinet can be followed from one \
         release to the next -- your Windows account name, your computer name, and a \
         random id we generate for this install. Nothing else about your machine, and \
         no file contents. The log is what lets a capture that tracks badly be told \
         apart from one that was merely starved by USB.",
    ),
    (
        "Sole use",
        "to train and improve the pincab head-tracking model. Nothing else.",
    ),
    (
        "Private storage",
        "each capture is uploaded to the maintainer's private, write-only server. On \
         the server no one — not even you — can list, read or download anything; only \
         the maintainer sees the uploads. You are also offered a folder to keep your \
         own copy in; declining is fine, and nothing else is written to your disk.",
    ),
    (
        "Never public",
        "never published, sold, shared with third parties, or used beyond training.",
    ),
    (
        "Your responsibility",
        "share only images you have the right to distribute. If people appear (children \
         included), you confirm you have their consent (or are the parent/guardian). You \
         must be of legal age.",
    ),
    (
        "Withdrawal",
        "uploads are anonymous and can't be searched by author — to have one removed, give \
         its exact file name (shown after upload) on Discord #headtracking \
         (https://discord.gg/cFcNrt9AY).",
    ),
];

// Overlay colours, kept here so the toolbar status text and the canvas
// drawing stay in sync. Head → soft red; the anchor geometry → a
// translucent cyan fill, with the lockbar quad derived from its closed
// edge drawn in solid cyan (high contrast against red, visible on both
// bright playfield reflections and dark cabinet interiors).
const LOCKBAR_COLOR: Color32 = Color32::from_rgb(0x00, 0xe5, 0xff);

fn main() {
    // The `windows_subsystem = "windows"` attribute above detaches release
    // builds from any console (no black window on double-click) — but the
    // CLI modes (--capture, --contribute, --list-cameras…) still need to
    // print when launched FROM a terminal. Re-attach to the parent's
    // console if there is one; failing (the double-click case) is normal.
    #[cfg(all(target_os = "windows", not(debug_assertions)))]
    {
        unsafe extern "system" {
            fn AttachConsole(process_id: u32) -> i32;
        }
        const ATTACH_PARENT_PROCESS: u32 = u32::MAX;
        // SAFETY: plain kernel32 call, no pointers involved.
        unsafe {
            AttachConsole(ATTACH_PARENT_PROCESS);
        }
    }

    let logs: Arc<Mutex<VecDeque<String>>> =
        Arc::new(Mutex::new(VecDeque::with_capacity(LOG_BUFFER_LINES)));
    init_tracing(Arc::clone(&logs));

    // Banner: lets a beta tester paste a single line that pins the
    // build they're running when something misbehaves.
    let host = os_info::get();
    info!(
        version = env!("CARGO_PKG_VERSION"),
        os = %host.os_type(),
        os_version = %host.version(),
        arch = host.architecture().unwrap_or("unknown"),
        "headtracking-demo starting"
    );
    // Right under the banner, so following one cabinet across releases is a
    // grep rather than an inference from capture paths.
    let (user, machine) = host_identity();
    info!(
        user,
        machine,
        install = install_id(),
        "host identity — for following one cabinet across releases"
    );
    // Bus context right under the banner: speed and contention explain most
    // of what the perf numbers below will show, and deducing them after the
    // fact took an hour on the first Windows report.
    usb_check::log_startup();

    // `--upload-test`: exercise the contribution upload path (ureq / rustls /
    // auth / write-only drop) end to end without the GUI, then exit. This is
    // what we ask a contributor to run when their captures never arrive: it
    // separates "this machine cannot reach the server" from "the server said
    // no", in one line each, without them having to capture anything.
    if std::env::args().any(|a| a == "--upload-test") {
        let reach = contribute::probe();
        println!("upload-test: reachability — {}", reach.explain());
        if !reach.is_up() {
            eprintln!(
                "upload-test: FAILED before sending anything.\n\
                 Captures can still be saved locally and handed over here: {}",
                contribute::DISCORD_INVITE
            );
            std::process::exit(1);
        }
        // A test file has no business landing in the contributor's rescue
        // folder if it fails — it is not a capture worth keeping.
        let uploader = contribute::Uploader::spawn(std::env::temp_dir());
        let name = format!("{}_uploadtest.txt", contribution_stem(Backend::None));
        println!("upload-test: PUT {name}");
        uploader.submit(name.clone(), b"headtracking-demo upload test\n".to_vec());
        // Wait on the same budget a real file gets, so a slow-but-working link
        // reports OK instead of a misleading timeout.
        let deadline = Instant::now() + contribute::batch_budget(1);
        while Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(100));
            let st = uploader.status();
            if st.pending == 0 {
                if st.uploaded > 0 {
                    println!("upload-test: OK ({name})");
                    std::process::exit(0);
                }
                eprintln!("upload-test: FAILED — {:?}", st.last_error);
                std::process::exit(1);
            }
        }
        eprintln!("upload-test: timeout");
        std::process::exit(1);
    }

    // `--list-cameras`: print what SDL enumerates (real SDL_CameraIDs, which
    // are opaque and NOT 0/1/2 indices), then exit. Diagnostic for "why isn't
    // my webcam picked up".
    if std::env::args().any(|a| a == "--list-cameras") {
        match webcam::list() {
            Ok(cams) if cams.is_empty() => println!("SDL enumerated 0 cameras."),
            Ok(cams) => {
                println!("SDL enumerated {} camera(s):", cams.len());
                for c in &cams {
                    println!("  id={} name={:?}", c.id, c.name);
                }
            }
            Err(e) => println!("camera enumeration failed: {e}"),
        }
        std::process::exit(0);
    }

    // `--pose-test --raw <png> [--depth <png>] [--ir <png>] [--out <png>]`:
    // headless validation of the BlazePose head/pose path on real captured
    // modalities (e.g. a contribution raw+depth pair). Runs BlazePose on the
    // raw, samples depth at the nose, renders the skeleton, prints + exits.
    if std::env::args().any(|a| a == "--pose-test") {
        let mut args = std::env::args();
        let (mut raw, mut depth, mut ir, mut out) = (None, None, None, None);
        while let Some(a) = args.next() {
            match a.as_str() {
                "--raw" => raw = args.next(),
                "--depth" => depth = args.next(),
                "--ir" => ir = args.next(),
                "--out" => out = args.next(),
                _ => {}
            }
        }
        let Some(raw) = raw else {
            eprintln!("--pose-test needs --raw <png>");
            std::process::exit(2);
        };
        match run_pose_test(&raw, depth.as_deref(), ir.as_deref(), out.as_deref()) {
            Ok(()) => std::process::exit(0),
            Err(e) => {
                eprintln!("pose-test failed: {e}");
                std::process::exit(1);
            }
        }
    }

    // CLI: `--capture <backend>` runs a headless capture (no eframe)
    // and exits. Used by `ssh` from a remote workstation to iterate
    // on the lockbar algorithm without needing a body in front of
    // the camera.
    match parse_cli() {
        Err(msg) => {
            eprintln!("error: {msg}\n\n{CLI_USAGE}");
            std::process::exit(2);
        }
        Ok(Some(cap)) => {
            let result = if cap.contribute {
                run_headless_contribute(cap)
            } else {
                run_headless_capture(cap)
            };
            match result {
                Ok(()) => std::process::exit(0),
                Err(e) => {
                    error!(error = %e, "headless capture failed");
                    eprintln!("capture failed: {e}");
                    std::process::exit(1);
                }
            }
        }
        Ok(None) => {}
    }

    let event_loop = winit::event_loop::EventLoop::new().expect("failed to create event loop");
    // Poll, not Wait: the camera feed drives continuous repaints, and we
    // request_redraw at the end of every frame to keep it live.
    event_loop.set_control_flow(winit::event_loop::ControlFlow::Poll);
    let mut shell = DemoShell::new(App::new(logs));
    if let Err(e) = event_loop.run_app(&mut shell) {
        error!(error = %e, "event loop failed");
        std::process::exit(1);
    }
    // Exit without running `shell`'s destructors. Dropping the glutin
    // surface/context calls `eglDestroySurface`, which segfaults inside the
    // NVIDIA EGL-Wayland driver at teardown (invisible to the user — the
    // window has already closed — but it leaves a core dump). GL resources
    // are reclaimed by the OS on exit; the `exiting()` handler already ran
    // egui/scene cleanup inside the event loop.
    std::process::exit(0);
}

// ===================================================== Window + GL plumbing
//
// Manual winit + glutin + glow + egui_glow integration (instead of eframe)
// so egui-rotate can splice into the render loop: it rotates `raw_input`
// before egui sees it and the tessellated primitives before the paint —
// a seam eframe doesn't expose. Window plumbing adapted from egui_glow's
// `pure_glow` example via `egui-rotate/examples/rotated_demo.rs`.

/// The embedded webcam glyph (original artwork, drawn for this project) as
/// the window/taskbar icon. `None` on a decode failure — a missing icon is
/// not worth failing startup for.
fn window_icon() -> Option<winit::window::Icon> {
    let decoder = png::Decoder::new(std::io::Cursor::new(
        &include_bytes!("../assets/icon.png")[..],
    ));
    let mut reader = decoder.read_info().ok()?;
    let mut buf = vec![0u8; reader.output_buffer_size()?];
    let info = reader.next_frame(&mut buf).ok()?;
    buf.truncate(info.buffer_size());
    winit::window::Icon::from_rgba(buf, info.width, info.height).ok()
}

struct GlutinWindowContext {
    window: winit::window::Window,
    gl_context: glutin::context::PossiblyCurrentContext,
    gl_display: glutin::display::Display,
    gl_surface: glutin::surface::Surface<glutin::surface::WindowSurface>,
}

impl GlutinWindowContext {
    // SAFETY: standard glutin bring-up — the GL context/surface are built
    // from the freshly created window's raw handle, which outlives them.
    unsafe fn new(event_loop: &winit::event_loop::ActiveEventLoop) -> Self {
        use glutin::context::NotCurrentGlContext as _;
        use glutin::display::GetGlDisplay as _;
        use glutin::display::GlDisplay as _;
        use glutin::prelude::GlSurface as _;
        use winit::raw_window_handle::HasWindowHandle as _;

        let winit_window_builder = winit::window::WindowAttributes::default()
            .with_resizable(true)
            .with_inner_size(winit::dpi::PhysicalSize {
                width: 1100u32,
                height: 800u32,
            })
            .with_title("headtracking-demo")
            .with_window_icon(window_icon())
            .with_visible(false);

        let config_template_builder = glutin::config::ConfigTemplateBuilder::new()
            .prefer_hardware_accelerated(None)
            .with_depth_size(0)
            .with_stencil_size(0)
            .with_transparency(false);

        let (mut window, gl_config) = glutin_winit::DisplayBuilder::new()
            .with_preference(glutin_winit::ApiPreference::FallbackEgl)
            .with_window_attributes(Some(winit_window_builder.clone()))
            .build(event_loop, config_template_builder, |mut it| {
                it.next().expect("no GL config")
            })
            .expect("failed to create gl_config");

        let gl_display = gl_config.display();
        let raw_window_handle = window.as_ref().map(|w| w.window_handle().unwrap().as_raw());

        let context_attributes =
            glutin::context::ContextAttributesBuilder::new().build(raw_window_handle);
        let fallback_context_attributes = glutin::context::ContextAttributesBuilder::new()
            .with_context_api(glutin::context::ContextApi::Gles(None))
            .build(raw_window_handle);
        // SAFETY: creating a GL context from a valid display + config.
        let not_current_gl_context = unsafe {
            gl_display
                .create_context(&gl_config, &context_attributes)
                .unwrap_or_else(|_| {
                    gl_display
                        .create_context(&gl_config, &fallback_context_attributes)
                        .expect("failed to create context")
                })
        };

        let window = window.take().unwrap_or_else(|| {
            glutin_winit::finalize_window(event_loop, winit_window_builder.clone(), &gl_config)
                .expect("failed to finalize window")
        });
        let (w, h): (u32, u32) = window.inner_size().into();
        let surface_attributes =
            glutin::surface::SurfaceAttributesBuilder::<glutin::surface::WindowSurface>::new()
                .build(
                    window.window_handle().unwrap().as_raw(),
                    NonZeroU32::new(w).unwrap_or(NonZeroU32::MIN),
                    NonZeroU32::new(h).unwrap_or(NonZeroU32::MIN),
                );
        // SAFETY: surface attributes derive from the same valid window handle.
        let gl_surface = unsafe {
            gl_display
                .create_window_surface(&gl_config, &surface_attributes)
                .unwrap()
        };
        let gl_context = not_current_gl_context.make_current(&gl_surface).unwrap();
        gl_surface
            // No VSync — this is a real-time tracker; we don't want the render
            // loop (which also drives camera polling) quantised to 60/N by
            // vblanks. Uncapped so v2's heavier frames run at their true rate.
            .set_swap_interval(&gl_context, glutin::surface::SwapInterval::DontWait)
            .unwrap();

        Self {
            window,
            gl_context,
            gl_display,
            gl_surface,
        }
    }

    fn window(&self) -> &winit::window::Window {
        &self.window
    }

    fn resize(&self, size: winit::dpi::PhysicalSize<u32>) {
        use glutin::surface::GlSurface as _;
        self.gl_surface.resize(
            &self.gl_context,
            size.width.try_into().unwrap(),
            size.height.try_into().unwrap(),
        );
    }

    fn swap_buffers(&self) -> glutin::error::Result<()> {
        use glutin::surface::GlSurface as _;
        self.gl_surface.swap_buffers(&self.gl_context)
    }

    fn get_proc_address(&self, addr: &std::ffi::CStr) -> *const std::ffi::c_void {
        use glutin::display::GlDisplay as _;
        self.gl_display.get_proc_address(addr)
    }
}

// Virtual screen + scene geometry for the parallax window, in millimetres.
// The screen is a rectangle centred on the origin in the z=0 plane; the
// scene recedes to negative z behind it, the eye sits in front (+z). The
// screen *height* is fixed; its *width* follows the panel aspect so the
// scene fills the panel edge-to-edge without distortion.
const PX_SCREEN_H_MM: f32 = 225.0;
const PX_BOX_DEPTH_MM: f32 = 900.0;
const PX_NEAR_MM: f32 = 60.0;
const PX_FAR_MM: f32 = 4000.0;
/// Nominal viewing distance — the eye's resting Z.
const PX_DVIEW_MM: f32 = 600.0;

/// Offscreen render target + GL resources for the parallax validation
/// window (fish-tank VR). Owns an FBO (colour texture registered with
/// egui's painter so egui-rotate rotates it for free, + depth renderbuffer),
/// a minimal shader program, and the static scene geometry: a receding
/// wireframe "shadow box" and a pinball-table diorama. Each
/// frame it renders the scene with an *off-axis* projection derived from
/// the supplied eye position (Kooima 2008), so moving the eye looks around
/// the window edges. See `docs/parallax-validation-window.md`.
struct ParallaxScene {
    fbo: glow::Framebuffer,
    color: glow::Texture,
    depth: glow::Renderbuffer,
    size: (i32, i32),
    /// egui handle to `color`, registered once. Stays valid across
    /// [`Self::resize`] because that reallocates storage on the *same*
    /// texture name rather than creating a new one.
    tex_id: Option<egui::TextureId>,
    /// Shader program (vertex applies the MVP, fragment passes the
    /// per-vertex colour through) and its `u_mvp` uniform location.
    program: glow::Program,
    u_mvp: Option<glow::UniformLocation>,
    /// Wireframe shadow box (`GL_LINES`) and the pinball diorama
    /// (`GL_TRIANGLES`): a VAO + VBO each, plus the vertex count to draw.
    box_vao: glow::VertexArray,
    box_vbo: glow::Buffer,
    box_count: i32,
    pts_vao: glow::VertexArray,
    pts_vbo: glow::Buffer,
    pts_count: i32,
    /// Panel aspect the box/target geometry was last built for — rebuilt in
    /// [`Self::render`] when the panel aspect changes, so the scene fills the
    /// panel edge-to-edge at any shape without distorting the markers.
    geom_aspect: f32,
}

impl ParallaxScene {
    /// Fixed offscreen resolution. 4:3 matches the virtual screen + camera
    /// feed; egui scales the texture into whatever space the panel gives it.
    const W: i32 = 640;
    const H: i32 = 480;

    /// # Safety
    /// All glow calls run on the GL/UI thread with the context current.
    unsafe fn new(gl: &glow::Context) -> Self {
        use glow::HasContext as _;
        // SAFETY: caller guarantees the GL context is current.
        unsafe {
            let fbo = gl.create_framebuffer().expect("create parallax FBO");
            let color = gl.create_texture().expect("create parallax colour texture");
            let depth = gl
                .create_renderbuffer()
                .expect("create parallax depth buffer");

            let embedded = gl.version().is_embedded;
            let (program, u_mvp) = build_parallax_program(gl, embedded);
            // Initial geometry at the placeholder 4:3; render() rebuilds it to
            // the real panel aspect on the first frame.
            let aspect0 = Self::W as f32 / Self::H as f32;
            let (box_vao, box_vbo, box_count) = upload_mesh(gl, &parallax_box_mesh(aspect0));
            let (pts_vao, pts_vbo, pts_count) = upload_mesh(gl, &parallax_table_mesh(aspect0));

            let mut scene = Self {
                fbo,
                color,
                depth,
                size: (0, 0),
                tex_id: None,
                program,
                u_mvp,
                box_vao,
                box_vbo,
                box_count,
                pts_vao,
                pts_vbo,
                pts_count,
                geom_aspect: aspect0,
            };
            scene.resize(gl, Self::W, Self::H);
            scene
        }
    }

    /// (Re)allocate the colour texture + depth buffer at `w`×`h` and
    /// reattach them to the FBO. No-op when the size is unchanged.
    ///
    /// # Safety
    /// GL thread, context current.
    unsafe fn resize(&mut self, gl: &glow::Context, w: i32, h: i32) {
        use glow::HasContext as _;
        if self.size == (w, h) {
            return;
        }
        // SAFETY: caller guarantees the GL context is current; all the
        // object names were created by `new` on this same context.
        unsafe {
            gl.bind_texture(glow::TEXTURE_2D, Some(self.color));
            gl.tex_image_2d(
                glow::TEXTURE_2D,
                0,
                glow::RGBA8 as i32,
                w,
                h,
                0,
                glow::RGBA,
                glow::UNSIGNED_BYTE,
                glow::PixelUnpackData::Slice(None),
            );
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_MIN_FILTER,
                glow::LINEAR as i32,
            );
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_MAG_FILTER,
                glow::LINEAR as i32,
            );
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_WRAP_S,
                glow::CLAMP_TO_EDGE as i32,
            );
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_WRAP_T,
                glow::CLAMP_TO_EDGE as i32,
            );

            gl.bind_renderbuffer(glow::RENDERBUFFER, Some(self.depth));
            gl.renderbuffer_storage(glow::RENDERBUFFER, glow::DEPTH_COMPONENT24, w, h);

            gl.bind_framebuffer(glow::FRAMEBUFFER, Some(self.fbo));
            gl.framebuffer_texture_2d(
                glow::FRAMEBUFFER,
                glow::COLOR_ATTACHMENT0,
                glow::TEXTURE_2D,
                Some(self.color),
                0,
            );
            gl.framebuffer_renderbuffer(
                glow::FRAMEBUFFER,
                glow::DEPTH_ATTACHMENT,
                glow::RENDERBUFFER,
                Some(self.depth),
            );
            debug_assert_eq!(
                gl.check_framebuffer_status(glow::FRAMEBUFFER),
                glow::FRAMEBUFFER_COMPLETE,
                "parallax FBO incomplete",
            );

            gl.bind_framebuffer(glow::FRAMEBUFFER, None);
            gl.bind_texture(glow::TEXTURE_2D, None);
            gl.bind_renderbuffer(glow::RENDERBUFFER, None);
        }
        self.size = (w, h);
    }

    /// Render the scene into the FBO with an off-axis projection from `eye`
    /// (screen-space mm: +x right, +y up, +z toward the viewer), then
    /// restore the default framebuffer so the egui paint that follows is
    /// unaffected.
    ///
    /// # Safety
    /// GL thread, context current.
    unsafe fn render(&mut self, gl: &glow::Context, eye: [f32; 3], aspect: f32) {
        use glow::HasContext as _;
        // SAFETY: caller guarantees the GL context is current.
        unsafe {
            // Match the FBO aspect to the panel so the scene fills it with no
            // distortion; height fixed for a stable resolution.
            let h = 512;
            let w = ((h as f32 * aspect).round() as i32).clamp(160, 2048);
            self.resize(gl, w, h);

            // Rebuild box + target geometry when the panel aspect changes so
            // the front frame keeps hugging the panel edges (markers stay
            // square — only their spread and the frame widen).
            if (aspect - self.geom_aspect).abs() > 0.01 {
                let bm = parallax_box_mesh(aspect);
                gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.box_vbo));
                gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, f32_as_bytes(&bm), glow::DYNAMIC_DRAW);
                self.box_count = (bm.len() / 6) as i32;
                let tm = parallax_table_mesh(aspect);
                gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.pts_vbo));
                gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, f32_as_bytes(&tm), glow::DYNAMIC_DRAW);
                self.pts_count = (tm.len() / 6) as i32;
                gl.bind_buffer(glow::ARRAY_BUFFER, None);
                self.geom_aspect = aspect;
            }

            gl.bind_framebuffer(glow::FRAMEBUFFER, Some(self.fbo));
            gl.viewport(0, 0, self.size.0, self.size.1);
            gl.enable(glow::DEPTH_TEST);
            gl.depth_func(glow::LESS);
            gl.clear_color(0.03, 0.05, 0.08, 1.0);
            gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);

            let mvp = parallax_mvp(eye, aspect);
            gl.use_program(Some(self.program));
            gl.uniform_matrix_4_f32_slice(self.u_mvp.as_ref(), false, mvp.as_slice());

            // Wireframe shadow box (lines) + pinball diorama (triangles).
            gl.bind_vertex_array(Some(self.box_vao));
            gl.draw_arrays(glow::LINES, 0, self.box_count);
            gl.bind_vertex_array(Some(self.pts_vao));
            gl.draw_arrays(glow::TRIANGLES, 0, self.pts_count);

            gl.bind_vertex_array(None);
            gl.use_program(None);
            gl.disable(glow::DEPTH_TEST);
            gl.bind_framebuffer(glow::FRAMEBUFFER, None);
        }
    }

    /// Register the colour texture with egui's painter (once) and return
    /// its stable `TextureId`.
    fn texture_id(&mut self, painter: &mut egui_glow::Painter) -> egui::TextureId {
        *self
            .tex_id
            .get_or_insert_with(|| painter.register_native_texture(self.color))
    }

    /// # Safety
    /// GL thread, context current.
    unsafe fn destroy(&mut self, gl: &glow::Context) {
        use glow::HasContext as _;
        // SAFETY: caller guarantees the GL context is current.
        unsafe {
            gl.delete_program(self.program);
            gl.delete_vertex_array(self.box_vao);
            gl.delete_buffer(self.box_vbo);
            gl.delete_vertex_array(self.pts_vao);
            gl.delete_buffer(self.pts_vbo);
            gl.delete_framebuffer(self.fbo);
            gl.delete_texture(self.color);
            gl.delete_renderbuffer(self.depth);
        }
    }
}

// ===================================== Parallax scene: shaders + geometry + math

const PARALLAX_VERT: &str = r"
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_col;
uniform mat4 u_mvp;
out vec3 v_col;
void main() {
    gl_Position = u_mvp * vec4(a_pos, 1.0);
    v_col = a_col;
}
";

const PARALLAX_FRAG: &str = r"
in vec3 v_col;
out vec4 frag_color;
void main() { frag_color = vec4(v_col, 1.0); }
";

/// Compile + link the parallax program; returns it with its uniform
/// locations. The GLSL version header is chosen at runtime to match the
/// context (desktop `330 core` vs `300 es`). Panics with the GL info log on
/// failure — a shader bug here is a dev error, not a runtime condition.
///
/// # Safety
/// GL thread, context current.
unsafe fn build_parallax_program(
    gl: &glow::Context,
    embedded: bool,
) -> (glow::Program, Option<glow::UniformLocation>) {
    use glow::HasContext as _;
    let header = if embedded {
        "#version 300 es\nprecision mediump float;\n"
    } else {
        "#version 330 core\n"
    };
    // SAFETY: GL context current (caller guarantee).
    unsafe {
        let vs = compile_shader(gl, glow::VERTEX_SHADER, &format!("{header}{PARALLAX_VERT}"));
        let fs = compile_shader(
            gl,
            glow::FRAGMENT_SHADER,
            &format!("{header}{PARALLAX_FRAG}"),
        );
        let program = gl.create_program().expect("create parallax program");
        gl.attach_shader(program, vs);
        gl.attach_shader(program, fs);
        gl.link_program(program);
        assert!(
            gl.get_program_link_status(program),
            "parallax program link failed: {}",
            gl.get_program_info_log(program)
        );
        gl.detach_shader(program, vs);
        gl.detach_shader(program, fs);
        gl.delete_shader(vs);
        gl.delete_shader(fs);
        let u_mvp = gl.get_uniform_location(program, "u_mvp");
        (program, u_mvp)
    }
}

/// # Safety
/// GL thread, context current.
unsafe fn compile_shader(gl: &glow::Context, kind: u32, src: &str) -> glow::Shader {
    use glow::HasContext as _;
    // SAFETY: GL context current (caller guarantee).
    unsafe {
        let sh = gl.create_shader(kind).expect("create shader");
        gl.shader_source(sh, src);
        gl.compile_shader(sh);
        assert!(
            gl.get_shader_compile_status(sh),
            "parallax shader compile failed: {}",
            gl.get_shader_info_log(sh)
        );
        sh
    }
}

/// Upload an interleaved `[x,y,z, r,g,b]` mesh into a fresh VAO+VBO and wire
/// the two `vec3` attributes. Returns `(vao, vbo, vertex_count)`.
///
/// # Safety
/// GL thread, context current.
unsafe fn upload_mesh(gl: &glow::Context, verts: &[f32]) -> (glow::VertexArray, glow::Buffer, i32) {
    use glow::HasContext as _;
    const STRIDE: i32 = 6 * 4; // 6 f32 per vertex
    // SAFETY: GL context current (caller guarantee).
    unsafe {
        let vao = gl.create_vertex_array().expect("create vao");
        let vbo = gl.create_buffer().expect("create vbo");
        gl.bind_vertex_array(Some(vao));
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
        gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, f32_as_bytes(verts), glow::STATIC_DRAW);
        gl.enable_vertex_attrib_array(0);
        gl.vertex_attrib_pointer_f32(0, 3, glow::FLOAT, false, STRIDE, 0);
        gl.enable_vertex_attrib_array(1);
        gl.vertex_attrib_pointer_f32(1, 3, glow::FLOAT, false, STRIDE, 12);
        gl.bind_vertex_array(None);
        gl.bind_buffer(glow::ARRAY_BUFFER, None);
        (vao, vbo, (verts.len() / 6) as i32)
    }
}

fn f32_as_bytes(v: &[f32]) -> &[u8] {
    // SAFETY: f32 is plain-old-data with no invalid bit patterns; the view
    // spans exactly size_of_val(v) bytes and is read-only, consumed
    // immediately by an immutable GL upload.
    unsafe { std::slice::from_raw_parts(v.as_ptr().cast::<u8>(), std::mem::size_of_val(v)) }
}

/// Half-extents of the virtual screen for a given panel aspect (the screen
/// height is fixed; its width follows the aspect so the scene fills the
/// panel without distortion).
fn parallax_screen_half(aspect: f32) -> (f32, f32) {
    let hh = PX_SCREEN_H_MM * 0.5;
    (hh * aspect, hh)
}

/// Receding wireframe "shadow box": rectangle rings at five depths plus the
/// four corner edges joining front (z=0) to back (z=-depth). The front ring
/// is the window frame, sized to the panel aspect. Dim teal.
fn parallax_box_mesh(aspect: f32) -> Vec<f32> {
    let (hw, hh) = parallax_screen_half(aspect);
    let col = [0.30f32, 0.55, 0.65];
    let mut v: Vec<f32> = Vec::new();
    let mut line = |a: [f32; 3], b: [f32; 3]| {
        v.extend_from_slice(&[a[0], a[1], a[2], col[0], col[1], col[2]]);
        v.extend_from_slice(&[b[0], b[1], b[2], col[0], col[1], col[2]]);
    };
    let depths = [0.0f32, -0.25, -0.5, -0.75, -1.0].map(|f| f * PX_BOX_DEPTH_MM);
    for &z in &depths {
        let c = [[-hw, -hh, z], [hw, -hh, z], [hw, hh, z], [-hw, hh, z]];
        for i in 0..4 {
            line(c[i], c[(i + 1) % 4]);
        }
    }
    for &(sx, sy) in &[(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)] {
        line([sx, sy, 0.0], [sx, sy, -PX_BOX_DEPTH_MM]);
    }
    v
}

/// Pinball diorama (`GL_TRIANGLES`): an inclined playfield receding into
/// the shadow box, dressed with the depth cues a pinball player knows by
/// heart — side rails, flipper bats, pop bumpers, drop targets and a
/// ball. Judging fish-tank parallax on these reads far closer to a VPX
/// table than abstract cube grids. Real-world sizes in mm; fake top-light
/// baked into per-face vertex colours (no lighting pass).
fn parallax_table_mesh(aspect: f32) -> Vec<f32> {
    use std::f32::consts::TAU;

    fn push_tri(v: &mut Vec<f32>, pts: [[f32; 3]; 3], col: [f32; 3], shade: f32) {
        for p in pts {
            v.extend_from_slice(&[
                p[0],
                p[1],
                p[2],
                col[0] * shade,
                col[1] * shade,
                col[2] * shade,
            ]);
        }
    }
    #[allow(clippy::too_many_arguments)]
    fn push_quad(
        v: &mut Vec<f32>,
        a: [f32; 3],
        b: [f32; 3],
        c: [f32; 3],
        d: [f32; 3],
        col: [f32; 3],
        shade: f32,
    ) {
        push_tri(v, [a, b, c], col, shade);
        push_tri(v, [a, c, d], col, shade);
    }

    type At<'a> = &'a dyn Fn(f32, f32, f32) -> [f32; 3];

    /// Box standing on the plane, rotated by `phi` around the plane normal
    /// (flipper bats). `wx`/`wl` are half-sizes across/along the slope.
    #[allow(clippy::too_many_arguments)]
    fn boxed(
        v: &mut Vec<f32>,
        at: At,
        x0: f32,
        t0: f32,
        phi: f32,
        wx: f32,
        wl: f32,
        hgt: f32,
        col: [f32; 3],
    ) {
        let (c, s) = (phi.cos(), phi.sin());
        let corner = |dx: f32, dt: f32, dh: f32| {
            at(
                x0 + dx * wx * c - dt * wl * s,
                t0 + dx * wx * s + dt * wl * c,
                dh * hgt,
            )
        };
        let p = [
            corner(-1.0, -1.0, 0.0),
            corner(1.0, -1.0, 0.0),
            corner(1.0, 1.0, 0.0),
            corner(-1.0, 1.0, 0.0),
            corner(-1.0, -1.0, 1.0),
            corner(1.0, -1.0, 1.0),
            corner(1.0, 1.0, 1.0),
            corner(-1.0, 1.0, 1.0),
        ];
        push_quad(v, p[4], p[5], p[6], p[7], col, 1.0); // top
        push_quad(v, p[0], p[1], p[5], p[4], col, 0.78); // front (player side)
        push_quad(v, p[3], p[2], p[6], p[7], col, 0.45); // back
        push_quad(v, p[0], p[3], p[7], p[4], col, 0.58); // left
        push_quad(v, p[1], p[2], p[6], p[5], col, 0.66); // right
    }

    /// Upright cylinder (pop bumper): shaded sides + a bright cap fan.
    #[allow(clippy::too_many_arguments)]
    fn cyl(
        v: &mut Vec<f32>,
        at: At,
        x0: f32,
        t0: f32,
        r: f32,
        hgt: f32,
        col: [f32; 3],
        cap: [f32; 3],
    ) {
        const SEG: usize = 14;
        for i in 0..SEG {
            let a0 = i as f32 / SEG as f32 * TAU;
            let a1 = (i + 1) as f32 / SEG as f32 * TAU;
            let (xa, ta) = (x0 + r * a0.cos(), t0 + r * a0.sin());
            let (xb, tb) = (x0 + r * a1.cos(), t0 + r * a1.sin());
            let shade = 0.62 + 0.28 * a0.cos();
            push_quad(
                v,
                at(xa, ta, 0.0),
                at(xb, tb, 0.0),
                at(xb, tb, hgt),
                at(xa, ta, hgt),
                col,
                shade,
            );
            push_tri(
                v,
                [at(x0, t0, hgt), at(xa, ta, hgt), at(xb, tb, hgt)],
                cap,
                1.0,
            );
        }
    }

    /// Low-poly sphere in world coordinates (the ball).
    fn ball(v: &mut Vec<f32>, c: [f32; 3], r: f32, col: [f32; 3]) {
        use std::f32::consts::PI;
        const SL: usize = 10;
        const ST: usize = 5;
        let pt = |i: usize, j: usize| -> ([f32; 3], f32) {
            let phi = (j as f32 / ST as f32 - 0.5) * PI;
            let theta = i as f32 / SL as f32 * TAU;
            let p = [
                c[0] + r * phi.cos() * theta.cos(),
                c[1] + r * phi.sin(),
                c[2] + r * phi.cos() * theta.sin(),
            ];
            let shade = 0.45 + 0.55 * phi.sin().max(0.0) + 0.10 * phi.cos();
            (p, shade.min(1.0))
        };
        for j in 0..ST {
            for i in 0..SL {
                let (a, sa) = pt(i, j);
                let (b, _) = pt(i + 1, j);
                let (cc, _) = pt(i + 1, j + 1);
                let (d, sd) = pt(i, j + 1);
                push_quad(v, a, b, cc, d, col, (sa + sd) * 0.5);
            }
        }
    }

    let (hw, hh) = parallax_screen_half(aspect);
    let mut v: Vec<f32> = Vec::new();

    // Playfield frame: front edge low near the window, back edge high and
    // deep. `u` = unit up-slope vector, `n` = plane normal (toward viewer),
    // both in the Y-Z plane.
    let front = [-0.78 * hh, -110.0f32]; // (y, z) of the front edge
    let back = [0.42 * hh, -0.93 * PX_BOX_DEPTH_MM];
    let (dy, dz) = (back[0] - front[0], back[1] - front[1]);
    let len = dy.hypot(dz);
    let u = [dy / len, dz / len];
    let n = [-dz / len, dy / len];
    let wpf = hw * 0.80; // playfield half-width

    // A point at lateral `x`, `t` mm up the slope, `h` mm above the plane.
    let at = move |x: f32, t: f32, h: f32| -> [f32; 3] {
        [
            x,
            front[0] + u[0] * t + n[0] * h,
            front[1] + u[1] * t + n[1] * h,
        ]
    };

    const WOOD: [f32; 3] = [0.45, 0.30, 0.18];
    const RAIL: [f32; 3] = [0.26, 0.17, 0.10];
    const CREAM: [f32; 3] = [0.95, 0.90, 0.72];
    const RED: [f32; 3] = [0.85, 0.16, 0.14];
    const WHITE: [f32; 3] = [0.95, 0.95, 0.90];
    const YELLOW: [f32; 3] = [0.95, 0.78, 0.10];
    const STEEL: [f32; 3] = [0.78, 0.80, 0.85];

    // Playfield surface.
    push_quad(
        &mut v,
        at(-wpf, 0.0, 0.0),
        at(wpf, 0.0, 0.0),
        at(wpf, len, 0.0),
        at(-wpf, len, 0.0),
        WOOD,
        0.92,
    );
    // Side rails + back wall.
    boxed(
        &mut v,
        &at,
        -(wpf - 9.0),
        len * 0.5,
        0.0,
        9.0,
        len * 0.5,
        24.0,
        RAIL,
    );
    boxed(
        &mut v,
        &at,
        wpf - 9.0,
        len * 0.5,
        0.0,
        9.0,
        len * 0.5,
        24.0,
        RAIL,
    );
    boxed(&mut v, &at, 0.0, len - 8.0, 0.0, wpf, 8.0, 32.0, RAIL);
    // Flipper bats, angled inward toward the drain.
    boxed(
        &mut v,
        &at,
        -0.30 * wpf,
        80.0,
        -0.42,
        36.0,
        9.0,
        15.0,
        CREAM,
    );
    boxed(&mut v, &at, 0.30 * wpf, 80.0, 0.42, 36.0, 9.0, 15.0, CREAM);
    // Pop bumpers (red, white caps).
    cyl(&mut v, &at, -0.42 * wpf, 0.62 * len, 34.0, 44.0, RED, WHITE);
    cyl(&mut v, &at, 0.0, 0.74 * len, 34.0, 44.0, RED, WHITE);
    cyl(&mut v, &at, 0.42 * wpf, 0.62 * len, 34.0, 44.0, RED, WHITE);
    // A bank of three drop targets.
    for k in -1i32..=1 {
        boxed(
            &mut v,
            &at,
            k as f32 * 95.0,
            0.88 * len,
            0.0,
            16.0,
            4.0,
            40.0,
            YELLOW,
        );
    }
    // The ball, resting mid-table.
    ball(&mut v, at(0.18 * wpf, 0.30 * len, 13.5), 13.5, STEEL);
    v
}

/// Off-axis MVP for the parallax scene: a Kooima generalized-perspective
/// frustum from `eye` (mm) onto the screen rectangle (sized to `aspect`) in
/// the z=0 plane, times the eye translation. Model = identity (static
/// scene), and the screen axes equal the world axes so the rotation `M` is
/// identity too.
///
/// Built with `nalgebra`: `Matrix4::new` takes row-major arguments (as the
/// matrix reads on paper) and stores them column-major, so `as_slice()`
/// hands `glUniformMatrix4fv(transpose=false)` exactly what it wants.
fn parallax_mvp(eye: [f32; 3], aspect: f32) -> Matrix4<f32> {
    let (hw, hh) = parallax_screen_half(aspect);
    let (ex, ey) = (eye[0], eye[1]);
    let ez = eye[2].max(PX_NEAR_MM + 1.0); // eye→screen distance, must exceed near
    let (n, f) = (PX_NEAR_MM, PX_FAR_MM);
    let s = n / ez;
    let (l, r, b, t) = ((-hw - ex) * s, (hw - ex) * s, (-hh - ey) * s, (hh - ey) * s);

    // Asymmetric `glFrustum`, then the eye translation (eye → origin).
    #[rustfmt::skip]
    let frustum = Matrix4::new(
        2.0 * n / (r - l), 0.0,               (r + l) / (r - l),  0.0,
        0.0,               2.0 * n / (t - b), (t + b) / (t - b),  0.0,
        0.0,               0.0,              -(f + n) / (f - n), -2.0 * f * n / (f - n),
        0.0,               0.0,              -1.0,                0.0,
    );
    #[rustfmt::skip]
    let translate = Matrix4::new(
        1.0, 0.0, 0.0, -ex,
        0.0, 1.0, 0.0, -ey,
        0.0, 0.0, 1.0, -ez,
        0.0, 0.0, 0.0,  1.0,
    );
    frustum * translate
}

/// Owns the GL window + egui integration and drives the `App` model
/// each frame, wrapping its UI in egui-rotate's input/output transforms.
struct DemoShell {
    app: App,
    gl_window: Option<GlutinWindowContext>,
    gl: Option<Arc<glow::Context>>,
    egui_ctx: egui::Context,
    egui_winit: Option<egui_winit::State>,
    painter: Option<egui_glow::Painter>,
    /// Lazily created the first frame the parallax window is enabled.
    parallax: Option<ParallaxScene>,
    /// When the next animation frame is due. The camera feed only produces new
    /// content at its own rate (~30 fps Kinect), so the continuous repaint is
    /// throttled to just above that instead of the display's 60 Hz vsync —
    /// halving the render CPU. Input events (`request_redraw` from
    /// `window_event`/`device_event`) still repaint immediately, so the UI
    /// stays responsive; only the idle animation cadence is capped.
    next_frame_at: Instant,
}

impl DemoShell {
    fn new(app: App) -> Self {
        let egui_ctx = egui::Context::default();
        install_extra_glyph_fonts(&egui_ctx);
        apply_cab_style(&egui_ctx);
        Self {
            app,
            gl_window: None,
            gl: None,
            egui_ctx,
            egui_winit: None,
            painter: None,
            parallax: None,
            next_frame_at: Instant::now(),
        }
    }

    fn redraw(&mut self) {
        let window = self.gl_window.as_ref().unwrap().window();
        let physical_dimensions: [u32; 2] = window.inner_size().into();
        let physical_size =
            egui::Vec2::new(physical_dimensions[0] as f32, physical_dimensions[1] as f32);
        let ctx = self.egui_ctx.clone();

        // 0. Parallax window: render the offscreen scene into its FBO
        //    *before* the UI runs, so `App::ui` can show this frame's
        //    texture. The RotationPlugin then rotates that image like any other.
        if self.app.parallax_enabled {
            let gl = self.gl.as_ref().unwrap();
            let painter = self.painter.as_mut().unwrap();
            let scene = self
                .parallax
                // SAFETY: GL context current on this (UI) thread.
                .get_or_insert_with(|| unsafe { ParallaxScene::new(gl) });
            // SAFETY: same.
            // The eye was computed during the previous frame's UI pass (the
            // FBO renders before run_ui) — a 1-frame lag, imperceptible here.
            unsafe { scene.render(gl, self.app.parallax_eye, self.app.parallax_aspect) };
            self.app.parallax_tex = Some(scene.texture_id(painter));
        } else {
            self.app.parallax_tex = None;
        }

        // 1. Mirror the UI-facing rotation into the plugin before the pass —
        //    its input/output hooks read it to rotate input, shapes and the
        //    software cursor. Everything else the plugin does transparently.
        ctx.plugin::<RotationPlugin>()
            .lock()
            .set_rotation(self.app.rotation);

        // 2. Gather raw winit input. egui must see the *physical* screen rect;
        //    the plugin swaps it to logical in its input hook — no manual
        //    transform, no per-frame cursor plumbing.
        let mut raw_input = self.egui_winit.as_mut().unwrap().take_egui_input(window);
        if raw_input.screen_rect.is_none() {
            raw_input.screen_rect =
                Some(egui::Rect::from_min_size(egui::Pos2::ZERO, physical_size));
        }

        // 3. Run the UI. The plugin rotates input + shapes and draws the cursor.
        let app = &mut self.app;
        let mut full_output = ctx.run_ui(raw_input, |ui| app.ui(ui));

        // 4. If the software cursor was released to the OS (soft-lock breakout
        //    or a no-lock edge contact), warp the real cursor to the exit point.
        //    The plugin drops its own OS grab via a viewport command.
        if let Some(warp) = ctx.plugin::<RotationPlugin>().lock().take_pending_warp() {
            let _ = window.set_cursor_position(winit::dpi::PhysicalPosition::new(
                f64::from(warp.x),
                f64::from(warp.y),
            ));
        }

        // 5. Platform output — the plugin already set the cursor icon (None
        //    while the software cursor is captured, remapped otherwise).
        self.egui_winit
            .as_mut()
            .unwrap()
            .handle_platform_output(window, full_output.platform_output);

        // 6. Shapes are already rotated by the plugin — tessellate and paint.
        let clipped = ctx.tessellate(full_output.shapes, full_output.pixels_per_point);

        let painter = self.painter.as_mut().unwrap();
        // egui 0.36: several `ImageDelta` can be batched per texture id —
        // apply them in order.
        for (id, image_deltas) in &full_output.textures_delta.set {
            for image_delta in image_deltas {
                painter.set_texture(*id, image_delta);
            }
        }
        // SAFETY: glow calls on the current GL context, on the UI thread.
        unsafe {
            use glow::HasContext as _;
            let gl = self.gl.as_ref().unwrap();
            gl.clear_color(0.05, 0.06, 0.08, 1.0);
            gl.clear(glow::COLOR_BUFFER_BIT);
        }
        painter.paint_primitives(physical_dimensions, full_output.pixels_per_point, &clipped);
        for id in &full_output.textures_delta.free {
            painter.free_texture(*id);
        }
        // egui 0.36: a TexturesDelta dropped non-empty debug-asserts —
        // everything above has been applied, say so.
        full_output.textures_delta.clear();

        self.gl_window.as_ref().unwrap().swap_buffers().unwrap();
        // Count this present toward the render-rate metric (every repaint, not
        // just camera-fresh ones) so `render fps` shows the GL cadence.
        if let Some(active) = self.app.active.as_mut() {
            active.metrics.note_render_frame();
        }
        // Keep the camera feed live, but pace the continuous repaint to the
        // camera cadence (set in `about_to_wait`) instead of hammering vsync —
        // a fresh frame only exists at camera rate, so faster repaints just
        // re-paint identical pixels and burn CPU.
        self.next_frame_at = Instant::now() + self.app.target_frame_interval();
    }
}

impl winit::application::ApplicationHandler for DemoShell {
    fn resumed(&mut self, event_loop: &winit::event_loop::ActiveEventLoop) {
        // SAFETY: builds the GL context from a freshly created window on the
        // event loop's own thread.
        let gl_window = unsafe { GlutinWindowContext::new(event_loop) };
        // SAFETY: resolves valid GL symbols via the platform's GL display.
        let gl = unsafe {
            glow::Context::from_loader_function(|s| {
                let s = std::ffi::CString::new(s).unwrap();
                gl_window.get_proc_address(&s)
            })
        };
        let gl = Arc::new(gl);
        gl_window.window().set_visible(true);
        // Borderless fullscreen on the current monitor — the demo runs on a
        // dedicated (rotated) pincab screen, so it should own it.
        gl_window
            .window()
            .set_fullscreen(Some(winit::window::Fullscreen::Borderless(None)));

        let painter = egui_glow::Painter::new(Arc::clone(&gl), "", None, true)
            .expect("failed to create egui_glow painter");
        let egui_winit = egui_winit::State::new(
            self.egui_ctx.clone(),
            egui::ViewportId::ROOT,
            event_loop,
            None,
            event_loop.system_theme(),
            Some(painter.max_texture_side()),
        );

        // The whole rotation/cursor integration: register the plugin once. It
        // rotates input + rendered shapes per viewport, draws a rotated
        // software cursor and hides the OS one. `with_os_cursor_pin` keeps the
        // hidden real cursor centred so it can never physically leave the
        // window (kiosk / cabinet) — replaces the old manual `set_cursor_grab`.
        // Hard lock: the pincab screen is fixed, the cursor stays put.
        self.egui_ctx.add_plugin(
            RotationPlugin::new(self.app.rotation).with_software_cursor(
                SoftwareCursor::new()
                    .with_lock(true)
                    .with_os_cursor_pin(true)
                    .with_scale(1.4),
            ),
        );
        // Remote UI inspection (egui_mcp & friends), opt-in via the standard
        // EGUI_INSPECTION env var. Registered AFTER the rotation plugin —
        // that ordering is a contract (see egui-rotate's inspection docs):
        // injected logical-space events must bypass the physical→logical
        // rotation that real window events go through.
        match egui_rotate::attach_inspection_from_env(
            &self.egui_ctx,
            Some("headtracking-demo".into()),
        ) {
            Ok(true) => info!("egui inspection server attached (EGUI_INSPECTION)"),
            Ok(false) => {}
            Err(e) => warn!("egui inspection attach failed: {e}"),
        }

        self.gl_window = Some(gl_window);
        self.gl = Some(gl);
        self.egui_winit = Some(egui_winit);
        self.painter = Some(painter);
    }

    fn window_event(
        &mut self,
        event_loop: &winit::event_loop::ActiveEventLoop,
        _wid: winit::window::WindowId,
        event: winit::event::WindowEvent,
    ) {
        use winit::event::WindowEvent;

        if matches!(event, WindowEvent::CloseRequested | WindowEvent::Destroyed) {
            event_loop.exit();
            return;
        }
        // Escape quits the app.
        if let WindowEvent::KeyboardInput {
            event:
                winit::event::KeyEvent {
                    logical_key: winit::keyboard::Key::Named(winit::keyboard::NamedKey::Escape),
                    state: winit::event::ElementState::Pressed,
                    ..
                },
            ..
        } = &event
        {
            event_loop.exit();
            return;
        }
        if let WindowEvent::Resized(size) = &event
            && let Some(w) = self.gl_window.as_ref()
        {
            w.resize(*size);
        }
        if matches!(event, WindowEvent::RedrawRequested) {
            self.redraw();
            if self.app.should_quit {
                event_loop.exit();
            }
            return;
        }
        let window = self.gl_window.as_ref().unwrap().window();
        let response = self
            .egui_winit
            .as_mut()
            .unwrap()
            .on_window_event(window, &event);
        if response.repaint {
            window.request_redraw();
        }
    }

    /// Raw pointer motion (relative deltas) drives the software cursor — the
    /// window-event `CursorMoved` gives absolute positions that move the wrong
    /// way once the viewport is rotated. `egui_winit::State::on_mouse_motion`
    /// feeds the delta into egui; the `RotationPlugin` input hook turns it into
    /// the rotated software-cursor movement.
    fn device_event(
        &mut self,
        _event_loop: &winit::event_loop::ActiveEventLoop,
        _device_id: winit::event::DeviceId,
        event: winit::event::DeviceEvent,
    ) {
        if let winit::event::DeviceEvent::MouseMotion { delta } = event
            && let Some(state) = self.egui_winit.as_mut()
            && state.on_mouse_motion(delta)
            && let Some(w) = self.gl_window.as_ref()
        {
            w.window().request_redraw();
        }
    }

    /// Drive the animation cadence: fire the next repaint once the camera-rate
    /// deadline (set at the end of `redraw`) passes, otherwise sleep until then.
    /// Input-driven repaints bypass this entirely — they call `request_redraw`
    /// directly, so latency on interaction is unaffected.
    fn about_to_wait(&mut self, event_loop: &winit::event_loop::ActiveEventLoop) {
        let now = Instant::now();
        if now >= self.next_frame_at {
            if let Some(w) = self.gl_window.as_ref() {
                w.window().request_redraw();
            }
            // Push the deadline so we don't re-request every wake before the
            // RedrawRequested lands; `redraw` overwrites it with the real time.
            self.next_frame_at = now + self.app.target_frame_interval();
            event_loop.set_control_flow(winit::event_loop::ControlFlow::Poll);
        } else {
            event_loop.set_control_flow(winit::event_loop::ControlFlow::WaitUntil(
                self.next_frame_at,
            ));
        }
    }

    fn exiting(&mut self, _: &winit::event_loop::ActiveEventLoop) {
        if let (Some(scene), Some(gl)) = (self.parallax.as_mut(), self.gl.as_ref()) {
            // SAFETY: GL context current on the UI thread at shutdown.
            unsafe { scene.destroy(gl) };
        }
        if let Some(painter) = &mut self.painter {
            painter.destroy();
        }
    }
}

// ============================================================ CLI parsing

const CLI_USAGE: &str = "\
Usage: headtracking-demo [--capture <backend> [--out <path>] [--wait <secs>]]
                         [--contribute <backend> [--wait <secs>] [--local-copy <dir>]]

  --capture <backend>   Run headless: open backend, settle for `--wait`
                        seconds, save one PNG, exit.
                        backend = kinect-v2 | kinect-v1 | webcam | webcam-<N>
  --contribute <backend>
                        Run headless: capture EVERY stream of the backend
                        (raw/det/depth/ir + previews, same file set as the
                        GUI Contribute button) and upload it to the training
                        drop, then exit. Made for unattended cron runs
                        collecting lighting variety.
  --local-copy <dir>    Also keep a copy of the capture in <dir>. Without it
                        nothing is written to disk — the GUI asks for this
                        folder, a cron run has to name it.
  --out <path>          Output PNG path. Default: next to the binary,
                        named `<backend>_<UTC-timestamp>.png`.
  --wait <secs>         Seconds to let the device warm up + head/lockbar
                        detectors lock on before the capture (default 3).
  --lockbar-mm <mm>     Real lockbar width in mm — the scale reference for the
                        cam↔bar distance (default 610). Set it to your cab's.
  --pf-deg <deg>        Playfield inclination in degrees (default 6.5).
  --list-cameras        Print the cameras SDL enumerates (id + name), exit.
  --selftest            Validate the skeleton pipeline with no one in front of
    [--image <jpg>]     the camera: synthesise an upper-body silhouette (depth
    [--webcam]          + mask), run skeleton-depth, write selftest-*.png next
                        to the binary + print the joints. --image also runs a
                        real JPEG through personseg → track_mask; --webcam grabs
                        a live camera frame and does the same.
  -h, --help            Print this message.

No arguments → launches the interactive GUI.";

struct CaptureArgs {
    backend: Backend,
    out_path: Option<std::path::PathBuf>,
    wait_secs: f32,
    lockbar_mm: f32,
    pf_deg: f32,
    /// `--contribute`: full multi-stream capture + upload instead of one PNG.
    contribute: bool,
    /// `--local-copy <dir>`: keep a copy of the capture there too. `None` =
    /// upload only, the same default the GUI offers when the picker is
    /// cancelled.
    local_copy: Option<std::path::PathBuf>,
}

fn parse_cli() -> Result<Option<CaptureArgs>, String> {
    let raw: Vec<String> = std::env::args().skip(1).collect();
    if raw.is_empty() {
        return Ok(None);
    }
    if raw.iter().any(|a| a == "-h" || a == "--help") {
        println!("{CLI_USAGE}");
        std::process::exit(0);
    }

    let mut backend: Option<Backend> = None;
    let mut out_path: Option<std::path::PathBuf> = None;
    let mut wait_secs: f32 = 3.0;
    let mut lockbar_mm: f32 = headtracking::calibration::LOCKBAR_WIDTH_MM;
    let mut pf_deg: f32 = DEFAULT_TABLE_INCL_DEG;
    let mut contribute = false;
    let mut local_copy: Option<std::path::PathBuf> = None;
    let mut iter = raw.iter();
    while let Some(arg) = iter.next() {
        match arg.as_str() {
            "--capture" => {
                let v = iter.next().ok_or("--capture needs a backend name")?;
                backend = Some(parse_backend_arg(v)?);
            }
            "--contribute" => {
                let v = iter.next().ok_or("--contribute needs a backend name")?;
                backend = Some(parse_backend_arg(v)?);
                contribute = true;
            }
            "--out" => {
                let v = iter.next().ok_or("--out needs a path")?;
                out_path = Some(std::path::PathBuf::from(v));
            }
            "--local-copy" => {
                let v = iter.next().ok_or("--local-copy needs a directory")?;
                local_copy = Some(std::path::PathBuf::from(v));
            }
            "--wait" => {
                let v = iter.next().ok_or("--wait needs a seconds value")?;
                wait_secs = v
                    .parse()
                    .map_err(|e| format!("--wait value '{v}' invalid: {e}"))?;
            }
            "--lockbar-mm" => {
                let v = iter.next().ok_or("--lockbar-mm needs a value")?;
                lockbar_mm = v
                    .parse()
                    .map_err(|e| format!("--lockbar-mm value '{v}' invalid: {e}"))?;
            }
            "--pf-deg" => {
                let v = iter.next().ok_or("--pf-deg needs a value")?;
                pf_deg = v
                    .parse()
                    .map_err(|e| format!("--pf-deg value '{v}' invalid: {e}"))?;
            }
            other => return Err(format!("unknown flag '{other}'")),
        }
    }
    let backend =
        backend.ok_or("--capture or --contribute <backend> is required for non-GUI mode")?;
    Ok(Some(CaptureArgs {
        backend,
        out_path,
        wait_secs,
        lockbar_mm,
        pf_deg,
        contribute,
        local_copy,
    }))
}

fn parse_backend_arg(s: &str) -> Result<Backend, String> {
    match s {
        "kinect-v2" => Ok(Backend::KinectV2),
        "kinect-v1" => Ok(Backend::KinectV1),
        "webcam" => Ok(Backend::Webcam(1)),
        s if s.starts_with("webcam-") => {
            let n: u32 = s
                .strip_prefix("webcam-")
                .unwrap()
                .parse()
                .map_err(|e| format!("bad webcam index in '{s}': {e}"))?;
            Ok(Backend::Webcam(n))
        }
        other => Err(format!(
            "unknown backend '{other}' (expected kinect-v2, kinect-v1, webcam, or webcam-<N>)"
        )),
    }
}

// ============================================================ Headless capture

/// Open a backend, wait for its first RGB frame (bouncing the stream like the
/// GUI does when a Kinect v1 opens silent), then let the pipeline settle for
/// `wait_secs` so the detectors lock on. Shared by `--capture`/`--contribute`.
fn open_and_settle(backend: Backend, wait_secs: f32) -> Result<Capture, String> {
    let mut active = open_backend(backend)?;
    // Same stream-liveness recovery as the GUI, but blocking: wait for the
    // first RGB frame and bounce the stream up to MAX_STREAM_BOUNCES times
    // (the Kinect v1 sometimes opens without ever delivering a frame).
    for bounce in 0..=MAX_STREAM_BOUNCES {
        let deadline = Instant::now() + FIRST_FRAME_WAIT;
        let mut live = false;
        while Instant::now() < deadline {
            if active.poll_first_rgb() {
                live = true;
                break;
            }
            std::thread::sleep(Duration::from_millis(20));
        }
        if live {
            break;
        }
        if bounce == MAX_STREAM_BOUNCES {
            return Err(format!(
                "{}: no video after {MAX_STREAM_BOUNCES} stream restarts",
                backend_slug(backend)
            ));
        }
        warn!(backend = ?backend, next = bounce + 1, "headless: no first frame, bouncing stream");
        active.bounce_stream()?;
    }
    let deadline = Instant::now() + Duration::from_secs_f32(wait_secs.max(0.1));
    while Instant::now() < deadline {
        poll_active_headless(&mut active);
        std::thread::sleep(Duration::from_millis(30));
    }
    Ok(active)
}

fn run_headless_capture(cap: CaptureArgs) -> Result<(), String> {
    info!(
        backend = ?cap.backend,
        wait_secs = cap.wait_secs,
        out = ?cap.out_path,
        "headless capture starting"
    );
    let active = open_and_settle(cap.backend, cap.wait_secs)?;

    let (w, h, pixels, layout) = active
        .last_rgb_frame
        .as_ref()
        .ok_or_else(|| format!("no RGB frame received in {:.1}s", cap.wait_secs))?;
    let (w, h, rgb) = (*w, *h, Arc::new(frame_to_rgb888(pixels, *layout)));

    let slug = backend_slug(active.backend);
    let mut meta = capture_meta(
        active.backend,
        (w, h),
        &slug,
        CabGeom {
            table_incl_deg: cap.pf_deg,
            lockbar_mm: cap.lockbar_mm,
        },
        active.last_head,
        active.last_pose.as_ref(),
        active.last_lockbar.as_ref(),
    );
    // Depth ground-truth (the measured Kinect depth at the bar).
    meta.extend(autocalib_meta(
        active.last_lockbar.as_ref(),
        (w, h),
        active.last_depth.as_deref(),
    ));
    // Surface the lockbar-derived geometry on stdout so an SSH-driven capture
    // round can read the cam↔bar distance + camera offset without opening the
    // PNG. `ht_lockbar` == "none" here means anchor never locked the bar.
    for key in [
        "ht_lockbar",
        "ht_lockbar_width_px",
        "ht_color_fx",
        "ht_lockbar_dist_mm",
        "ht_lockbar_center_px",
        "ht_cam_offset_x_mm",
        "ht_cam_offset_y_mm",
        "ht_lockbar_slope_deg",
        "ht_lockbar_depth_mm",
    ] {
        if let Some((_, v)) = meta.iter().find(|(k, _)| k == key) {
            info!(target: "capture", "{key} = {v}");
        }
    }

    let path = if let Some(p) = cap.out_path {
        save_rgb_screenshot_at(
            &p,
            w,
            h,
            &rgb,
            active.last_pose.as_ref(),
            active.last_anchor.as_ref(),
            &meta,
        )?;
        p
    } else {
        save_rgb_screenshot(
            &slug,
            w,
            h,
            &rgb,
            active.last_pose.as_ref(),
            active.last_anchor.as_ref(),
            &meta,
        )?
    };

    info!(
        path = %path.display(),
        pose_found = active.last_pose.is_some(),
        lockbar_found = active.last_lockbar.is_some(),
        "headless capture saved"
    );
    Ok(())
}

/// Headless `--contribute <backend>`: capture EVERY stream the camera has
/// (same file set as the GUI Contribute button) and upload it to the
/// write-only drop, then exit — plus a local copy when `--local-copy` names a
/// folder. Built for unattended cron runs collecting training data across
/// lighting changes.
fn run_headless_contribute(cap: CaptureArgs) -> Result<(), String> {
    info!(
        backend = ?cap.backend,
        wait_secs = cap.wait_secs,
        "headless contribution starting"
    );
    let mut active = open_and_settle(cap.backend, cap.wait_secs)?;

    // v1: colour and IR share one USB endpoint — grab both explicitly
    // through the momentary mode switch, exactly like the GUI does.
    let (rgb_v1, ir_v1) = if let Inner::KinectV1 { device, .. } = &mut active.inner {
        let rgb = match device.capture_rgb(3) {
            Ok(f) => Some(f),
            Err(e) => {
                warn!("contribution: v1 RGB capture failed: {e}");
                None
            }
        };
        let ir = match device.capture_ir(3) {
            Ok(f) => Some(f),
            Err(e) => {
                warn!("contribution: v1 IR capture failed: {e}");
                None
            }
        };
        (rgb, ir)
    } else {
        (None, None)
    };

    let (w, h, raw): (u32, u32, Vec<u8>) = match rgb_v1.as_ref() {
        Some(f) => (f.width, f.height, f.data.clone()),
        None => {
            let (w, h, pixels, layout) = active
                .last_rgb_frame
                .as_ref()
                .ok_or_else(|| format!("no RGB frame received in {:.1}s", cap.wait_secs))?;
            (*w, *h, frame_to_rgb888(pixels, *layout))
        }
    };

    let stem = contribution_stem(active.backend);
    let mut meta = capture_meta(
        active.backend,
        (w, h),
        &stem,
        CabGeom {
            table_incl_deg: cap.pf_deg,
            lockbar_mm: cap.lockbar_mm,
        },
        active.last_head,
        active.last_pose.as_ref(),
        active.last_lockbar.as_ref(),
    );
    meta.extend(autocalib_meta(
        active.last_lockbar.as_ref(),
        (w, h),
        active.last_depth.as_deref(),
    ));
    let det = bake_overlays(
        w,
        h,
        &raw,
        active.last_pose.as_ref(),
        active.last_anchor.as_ref(),
    );
    let files = build_contribution_files(
        &stem,
        active.backend,
        (w, h),
        &raw,
        &det,
        active.last_depth.as_deref(),
        active.last_ir.as_deref(),
        ir_v1.as_ref(),
        &meta,
    );
    if files.is_empty() {
        return Err("no image could be encoded".to_string());
    }

    // No folder is invented here either: a cron run names one with
    // `--local-copy` or gets upload-only. An unusable folder is fatal — an
    // unattended run that silently keeps nothing is worse than one that stops.
    if let Some(dir) = &cap.local_copy {
        probe_writable(dir).map_err(|e| format!("--local-copy {}: {e}", dir.display()))?;
    }
    // Ask whether the drop is reachable before spending the transfer budget on
    // a link that was never going to carry it. Files the run cannot upload
    // land in the rescue folder either way, so the capture survives the answer.
    let reach = contribute::probe();
    println!("contribute: reachability — {}", reach.explain());
    // With no route to the drop the capture must still land somewhere, even
    // when the run asked for upload-only: an unattended job that captures and
    // then throws the result away is the worst of both.
    let keep = cap
        .local_copy
        .clone()
        .or_else(|| (!reach.is_up()).then(default_rescue_dir));
    let uploader = contribute::Uploader::spawn(keep.clone().unwrap_or_else(default_rescue_dir));
    let count = files.len();
    for (name, bytes) in files {
        if let Some(dir) = &keep {
            if let Err(e) = std::fs::create_dir_all(dir) {
                warn!(dir = %dir.display(), "contribution: folder unusable: {e}");
            } else if let Err(e) = std::fs::write(dir.join(&name), &bytes) {
                warn!(name, "contribution: local save failed: {e}");
            }
        }
        if reach.is_up() {
            println!("contribute: queuing {name} ({} KiB)", bytes.len() / 1024);
            uploader.submit(name, bytes);
        }
    }
    if !reach.is_up() {
        return Err(format!(
            "not uploaded: {} — the capture was kept in {} and can be handed over on {}",
            reach.explain(),
            keep.unwrap_or_else(default_rescue_dir).display(),
            contribute::DISCORD_INVITE
        ));
    }
    // Block until the upload queue drains — cron has no UI to watch it.
    let deadline = Instant::now() + contribute::batch_budget(count);
    loop {
        let st = uploader.status();
        if st.pending == 0 {
            if st.uploaded == count {
                println!("contribute: OK — {count} file(s) uploaded ({stem})");
                return Ok(());
            }
            let kept = st
                .rescued_in
                .clone()
                .unwrap_or_else(|| uploader.rescue_dir());
            return Err(format!(
                "only {}/{count} file(s) uploaded — last error: {:?}; what did not go up was \
                 kept in {} and can be handed over on {}",
                st.uploaded,
                st.last_error,
                kept.display(),
                contribute::DISCORD_INVITE
            ));
        }
        if Instant::now() > deadline {
            return Err(format!(
                "upload timed out with {}/{count} file(s) done",
                st.uploaded
            ));
        }
        std::thread::sleep(Duration::from_millis(200));
    }
}

/// Same pipeline as the live capture loop, run inline (headless mode is
/// single-threaded — no GL thread to publish to). `compute_head = false`: the
/// screenshot only cares about RGB + pose + lockbar, not the depth deprojection.
fn poll_active_headless(active: &mut Capture) {
    active.poll_once(false, false);
}

// ============================================================ Backend dropdown

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Backend {
    None,
    KinectV1,
    KinectV2,
    /// Index in the enumerated webcam list.
    Webcam(u32),
}

impl Backend {
    /// `true` when this input produces 3D head poses (head-box + depth for
    /// Kinect, head-box width triangulation for webcam). Only `None` doesn't.
    fn has_head_tracker(self) -> bool {
        !matches!(self, Backend::None)
    }
}

#[derive(Debug, Clone)]
struct BackendEntry {
    backend: Backend,
    label: String,
    /// The sensor is on the USB bus but its OS access prerequisite is
    /// missing (Windows WinUSB binding / Linux udev rule): shown greyed
    /// out so the user sees the hardware IS detected, with the fix
    /// banner right above. Cleared by the automatic post-install rescan.
    needs_drivers: bool,
}

/// Probe USB for connected sensors. Always returns `None (off)` first; the
/// other entries are added when the corresponding library reports a device.
///
/// Logs each backend's enumeration outcome at INFO level. For triage of
/// the "Kinect v1 listed in Device Manager but won't open" case on
/// Windows:
///   * set `FREENECT_LOG_LEVEL=spew` (or `flood`) before launching to
///     make libfreenect itself emit its full USB transcript;
///   * set `HEADTRACKING_LOG=libfreenect=debug,info,ort::logging=warn` so the demo
///     surfaces those lines (they'll appear with the `libfreenect:`
///     prefix in both the stderr stream and the in-app log panel).
fn detect_backends() -> Vec<BackendEntry> {
    info!("scan: probing USB backends");
    // One probe for the whole scan: any present Kinect function without
    // its access prerequisite marks the Kinect entries as needing drivers.
    // HT_FORCE_ACCESS_HINT forces it, like the banner, for UI testing.
    let kinect_blocked = std::env::var_os("HT_FORCE_ACCESS_HINT").is_some_and(|v| v != "0")
        || kinect_present_but_not_set_up();
    let mut out = vec![BackendEntry {
        backend: Backend::None,
        label: "None (off)".to_string(),
        needs_drivers: false,
    }];

    // ---- Kinect v2 (libfreenect2)
    //
    // We deliberately *don't* fall back to a sysfs probe when
    // `enumerate()` returns 0: the access-hint banner already fires on
    // any uncovered Kinect PID, and the libfreenect2 logger bridge
    // surfaces the precise `LIBUSB_ERROR_ACCESS` reason in the log
    // panel. Adding a fake "Kinect v2" entry that errors on click was
    // misleading once the banner+log pipeline became actionable.
    match freenect2::Context::new() {
        Ok(ctx) => {
            let n = ctx.enumerate();
            info!(count = n, "scan: libfreenect2 enumerated devices");
            if n > 0 {
                out.push(BackendEntry {
                    backend: Backend::KinectV2,
                    label: "Kinect v2".to_string(),
                    needs_drivers: kinect_blocked,
                });
            }
        }
        Err(e) => warn!(?e, "scan: kinect v2 context init failed"),
    }

    // ---- Kinect v1 (libfreenect)
    match freenect::Context::new() {
        Ok(ctx) => {
            let n = ctx.enumerate();
            // Distinguish two states explicitly:
            //   n == 0  : libusb saw nothing matching VID 045E:02ae/02bf
            //             → no driver bound, or driver bound by another
            //             stack libusb can't drive. On Windows the
            //             bundled `setup\setup.ps1` installs the WinUSB
            //             INFs that fix this — the demo offers a
            //             one-click banner that launches it elevated.
            //   n  > 0  : libusb saw the device descriptor and can
            //             open it.
            if n > 0 {
                info!(count = n, "scan: kinect v1 detected");
                out.push(BackendEntry {
                    backend: Backend::KinectV1,
                    label: "Kinect v1".to_string(),
                    needs_drivers: kinect_blocked,
                });
            } else {
                info!(
                    "scan: kinect v1 — libfreenect counted 0 devices. \
                     On Windows that usually means the WinUSB driver \
                     hasn't been bound to the three Xbox NUI sub-devices \
                     yet — click the in-app 'Install Kinect drivers' \
                     banner button (or run setup\\setup.ps1 elevated). \
                     Check the libfreenect log lines just above (set \
                     FREENECT_LOG_LEVEL=spew for full USB trace)."
                );
            }
        }
        Err(e) => warn!(?e, "scan: kinect v1 context init failed"),
    }

    // ---- Webcam via SDL3
    match webcam::list() {
        Ok(cams) => {
            info!(count = cams.len(), "scan: SDL3 enumerated cameras");
            for cam in cams {
                // Always tag the SDL id so two cameras with the same name
                // (or no name) stay distinguishable in the dropdown.
                let label = if cam.name.is_empty() {
                    format!("Webcam #{}", cam.id)
                } else {
                    format!("Webcam: {} [{}]", cam.name, cam.id)
                };
                info!(index = cam.id, name = %cam.name, "scan: webcam entry");
                out.push(BackendEntry {
                    backend: Backend::Webcam(cam.id),
                    label,
                    needs_drivers: false,
                });
            }
        }
        Err(e) => warn!(?e, "scan: webcam enumerate failed"),
    }

    // Dev aid: HT_FAKE_CAMS=N appends N dummy entries so dropdown layout
    // and switching UX can be exercised on a machine with no sensor
    // plugged in. The entries reuse `Backend::Webcam` with ids far above
    // anything SDL hands out; selecting one just fails to open, which is
    // fine for UI work.
    if let Ok(n) = std::env::var("HT_FAKE_CAMS")
        && let Ok(n) = n.parse::<u32>()
    {
        for i in 0..n {
            out.push(BackendEntry {
                backend: Backend::Webcam(90_000 + i),
                label: format!("Fake camera #{i} (HT_FAKE_CAMS)"),
                needs_drivers: false,
            });
        }
    }

    info!(entries = out.len() - 1, "scan: complete");
    out
}

// ==================================================== Kinect access helper
//
// A Kinect only shows up in `detect_backends` if libusb can actually open
// it, and that has an OS-level prerequisite:
//   * Linux  — a udev rule giving each Kinect USB node 0666 (libfreenect
//              ships `66-kinect.rules` for v1 PIDs, libfreenect2 ships
//              `90-kinect2.rules` for v2 PIDs); without them
//              `freenect{,2}::Context::enumerate()` probes each candidate,
//              hits `LIBUSB_ERROR_ACCESS`, and silently drops it.
//   * Windows — a libusb-capable kernel driver (WinUSB) bound to the
//              Kinect interfaces; a fresh Windows binds nothing, the
//              sensor sits in Device Manager as "Other device".
//   * macOS  — nothing; libusb opens the device as-is.
// So: if a Kinect is on the USB bus but absent from the dropdown, offer a
// one-click fix (drop the udev rule via pkexec/sudo on Linux; spawn the
// bundled `setup\setup.ps1` WinUSB installer elevated via a hidden
// PowerShell trampoline that calls `Start-Process -Verb RunAs` on Windows).

// Banner copy — picked at compile time so the message names the real fix.
#[cfg(target_os = "linux")]
const KINECT_ACCESS_PROBLEM: &str =
    "— libfreenect / libfreenect2 need a udev rule (0666) to open the sensor.";
#[cfg(target_os = "linux")]
const KINECT_ACCESS_BUTTON: &str = "Install udev rule (asks for password)";
#[cfg(target_os = "linux")]
const KINECT_ACCESS_OK_NOTE: &str = "udev rule installed. Click 'rescan' — if the sensor still doesn't show up, unplug/replug it first.";

#[cfg(target_os = "windows")]
const KINECT_ACCESS_PROBLEM: &str = "— Windows binds no usable driver out of the box; libusb/libfreenect can't see it until WinUSB is installed on the Kinect interfaces.";
#[cfg(target_os = "windows")]
const KINECT_ACCESS_BUTTON: &str = "Install Kinect drivers (UAC prompt)";
#[cfg(target_os = "windows")]
const KINECT_ACCESS_OK_NOTE: &str = "driver installer launched — confirm the UAC prompt, let it finish (~10-30 s), then click 'rescan'. v2 also needs a dedicated USB 3.0 root port.";

#[cfg(not(any(target_os = "linux", target_os = "windows")))]
const KINECT_ACCESS_PROBLEM: &str = "";
#[cfg(not(any(target_os = "linux", target_os = "windows")))]
const KINECT_ACCESS_BUTTON: &str = "";
#[cfg(not(any(target_os = "linux", target_os = "windows")))]
const KINECT_ACCESS_OK_NOTE: &str = "";

/// `true` when at least one Kinect on the USB bus lacks its OS access
/// prerequisite (Linux udev rule / Windows WinUSB binding), so the fix
/// banner is worth showing. Always `false` on macOS.
///
/// We deliberately *don't* short-circuit on "a Kinect is already in the
/// dropdown" — on Linux, libfreenect's v1 enumeration (`freenect_num_devices`)
/// doesn't need device access, so a v1 with no udev rule still shows up in
/// the dropdown but errors out at open time with EACCES. The presence-in-
/// dropdown signal is therefore unreliable; `kinect_present_but_not_set_up`
/// is the only authoritative source.
fn compute_kinect_access_hint() -> bool {
    // UI debug hook (same family as HT_FAKE_CAMS): force the banner without
    // any hardware, to eyeball its rendering after layout reworks.
    if std::env::var_os("HT_FORCE_ACCESS_HINT").is_some_and(|v| v != "0") {
        return true;
    }
    if !kinect_present_but_not_set_up() {
        return false;
    }
    warn!(
        "a Kinect is on the USB bus but not accessible — the demo offers a one-click fix \
         ({})",
        if cfg!(target_os = "windows") {
            "WinUSB driver install"
        } else {
            "udev rule install"
        }
    );
    true
}

/// PIDs libfreenect / libfreenect2 need 0666 on to open over libusb.
#[cfg(target_os = "linux")]
const KINECT_PIDS: &[&str] = &[
    "02ae", "02ad", "02b0", // v1 Xbox 360 (1414): camera, audio, motor
    "02c2", "02be", "02bf", // v1 Kinect for Windows (1473): camera, audio, motor
    "02c4", "02d8", "02d9", // v2: sensor, firmware-update, adapter hub
];

/// Linux: scan `/sys/bus/usb/devices` for currently-plugged Kinect USB
/// devices and return the set of matching product IDs (lowercase hex).
/// Read-only; no privileges needed. Used by both the access-hint check
/// and the `detect_backends` fallback that surfaces a v2 in the dropdown
/// even when libfreenect2's open-based `enumerate()` rejects it.
#[cfg(target_os = "linux")]
fn sysfs_present_kinect_pids() -> std::collections::HashSet<String> {
    std::fs::read_dir("/sys/bus/usb/devices")
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|entry| {
            let dir = entry.path();
            let id = |name: &str| {
                std::fs::read_to_string(dir.join(name))
                    .ok()
                    .map(|s| s.trim().to_ascii_lowercase())
            };
            if id("idVendor").as_deref() != Some("045e") {
                return None;
            }
            let pid = id("idProduct")?;
            KINECT_PIDS.contains(&pid.as_str()).then_some(pid)
        })
        .collect()
}

/// Linux: any Kinect v1 (Xbox 360 model 1414: `02ae`/`02ad`/`02b0`;
/// Kinect-for-Windows model 1473: `02c2`/`02be`/`02bf`) or Kinect v2
/// (`02c4`/`02d8`/`02d9`) USB device is plugged in (read from sysfs without
/// privileges) and at least one of the present PIDs isn't covered by any
/// udev rule under the standard rules directories. Returns true if so —
/// the banner needs to fire.
#[cfg(target_os = "linux")]
fn kinect_present_but_not_set_up() -> bool {
    use std::collections::HashSet;

    let present = sysfs_present_kinect_pids();
    if present.is_empty() {
        return false;
    }

    const RULES_DIRS: &[&str] = &[
        "/etc/udev/rules.d",
        "/run/udev/rules.d",
        "/usr/local/lib/udev/rules.d",
        "/usr/lib/udev/rules.d",
        "/lib/udev/rules.d",
    ];
    // A PID is "covered" iff some `.rules` file mentions BOTH `045e` and
    // that PID — same per-file conjunction the v2-only check used,
    // generalised to the union of detected PIDs.
    let mut covered: HashSet<String> = HashSet::new();
    for path in RULES_DIRS
        .iter()
        .flat_map(std::fs::read_dir)
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|e| e.to_str()) == Some("rules"))
    {
        let Ok(txt) = std::fs::read_to_string(&path) else {
            continue;
        };
        let lower = txt.to_ascii_lowercase();
        if !lower.contains("045e") {
            continue;
        }
        for pid in &present {
            if lower.contains(pid.as_str()) {
                covered.insert(pid.clone());
            }
        }
    }
    // Banner fires if at least one present Kinect PID has no rule.
    !present.is_subset(&covered)
}

/// What the Windows driver probe actually found, in one sentence.
///
/// This used to be a legend — one fixed string spelling out what `present=0`
/// and `missing>0` would each mean — with the real numbers appended as
/// structured fields at the far right of the line. A contributor whose setup
/// was perfectly healthy (`present=1 missing=0`) shipped a log whose probe
/// line opened with "nothing on the USB bus: check the v2 power adapter",
/// and it read as a verdict to them and to us. A log line has to state its
/// own finding.
///
/// Gated on `test` as well so the Linux CI exercises it.
#[cfg(any(target_os = "windows", test))]
fn driver_probe_verdict(present: u32, missing: u32) -> String {
    match (present, missing) {
        // A v2 without its powered adapter is completely invisible — not even
        // an unknown device — so this is the adapter/port question, not a
        // driver one.
        (0, _) => "windows Kinect driver probe: no Kinect function on the USB bus — \
                   check the v2 power adapter and use a rear USB 3.0 port"
            .to_owned(),
        (p, 0) => format!(
            "windows Kinect driver probe: {p} Kinect function(s) on the bus, all libusb-capable — \
             nothing to fix here"
        ),
        (p, m) => format!(
            "windows Kinect driver probe: {m} of {p} Kinect function(s) still lack WinUSB — \
             run the in-app driver install (a half-bound sensor enumerates but fails to open)"
        ),
    }
}

/// Windows: a Kinect (v1 sub-device or v2 sensor) is present on the USB
/// bus but no libusb-capable driver (WinUSB / libusbK) is bound to any of
/// its interfaces. Queried via PowerShell's `Get-PnpDevice` — no extra
/// crate, and it works on stock Windows 8.1+.
#[cfg(target_os = "windows")]
fn kinect_present_but_not_set_up() -> bool {
    use std::os::windows::process::CommandExt;
    use std::process::Command;

    // PID lists: v1 = Camera/Audio/Motor across models 1414 & 1473/KfW;
    // v2 = sensor 02C4. The firmware-update PID (02D8) and the NuiSensor
    // Adaptor power hub (02D9) are deliberately NOT in the required set —
    // they must never be WinUSB-bound, requiring them would make the
    // banner permanent on a healthy setup.
    //
    // EVERY required function must be libusb-capable, not just one: a
    // half-Zadig'd v1 (camera bound, motor not) enumerates in the
    // dropdown but fails at open — that exact case must keep the banner
    // up (field report 2026-08-07).
    // Win32_PnPEntity via Get-WmiObject instead of Get-PnpDevice: the
    // latter only exists on PowerShell 5.1+/Win 8.1+, while Get-WmiObject
    // works on every `powershell.exe` ever shipped (it's only gone in
    // pwsh 6+, which we never invoke) — and the population still running
    // Kinect-SDK-1.8-era hardware skews old. A probe that errors out
    // silently shows no banner at all (field report 2026-08-07).
    const SCRIPT: &str = "\
        $req = 'VID_045E&PID_(02AE|02BF|02AD|02BE|02B0|02C2|02C4)'; \
        $ok = @('WinUSB','WinUsb','libusbK','libusb0'); \
        $d = @(Get-WmiObject Win32_PnPEntity -ErrorAction SilentlyContinue | \
               Where-Object { $_.DeviceID -match $req }); \
        $bad = @($d | Where-Object { $ok -notcontains $_.Service }); \
        Write-Output (\"present={0} missing={1}\" -f $d.Count, $bad.Count)";
    const CREATE_NO_WINDOW: u32 = 0x0800_0000;

    let out = Command::new("powershell")
        .args(["-NoProfile", "-NonInteractive", "-Command", SCRIPT])
        .creation_flags(CREATE_NO_WINDOW)
        .output();
    let Ok(out) = out else {
        warn!("could not run powershell Get-PnpDevice — skipping Kinect driver check");
        return false;
    };
    let stdout = String::from_utf8_lossy(&out.stdout);
    let field = |key: &str| -> u32 {
        stdout
            .split_whitespace()
            .find_map(|tok| tok.strip_prefix(key)?.parse().ok())
            .unwrap_or(0)
    };
    let present = field("present=");
    let missing = field("missing=");
    // Always log what the probe saw — "no banner" support cases are
    // undiagnosable otherwise. Say what was *found*, not what the two
    // numbers would mean: the legend version of this line read as a verdict,
    // and a healthy `present=1 missing=0` run still told the reader to go
    // check their power adapter (field report 2026-09-03).
    info!(
        present,
        missing,
        "{}",
        driver_probe_verdict(present, missing)
    );
    if !out.status.success() {
        warn!(
            "Kinect driver probe exited with {}: {}",
            out.status,
            String::from_utf8_lossy(&out.stderr).trim()
        );
    }
    present > 0 && missing > 0
}

#[cfg(not(any(target_os = "linux", target_os = "windows")))]
fn kinect_present_but_not_set_up() -> bool {
    false
}

/// Run the one-click fix for the current platform. `Ok(())` means it was
/// kicked off (the user still has to confirm an elevation prompt and wait);
/// `Err` carries a fallback hint for the UI to display.
///
/// * Linux: stage a combined v1 + v2 rules file (`90-kinect.rules`) and
///   install it into `/etc/udev/rules.d/` via `pkexec` then `sudo`, then
///   reload udev.
/// * Windows: locate `setup\setup.ps1` from the release layout and spawn
///   a hidden non-elevated PowerShell that calls
///   `Start-Process -Verb RunAs` on `powershell.exe -ExecutionPolicy
///   Bypass -File setup.ps1` (UAC prompt). The elevated script binds
///   WinUSB to every known Kinect VID/PID — see the project README.
#[cfg(target_os = "linux")]
fn fix_kinect_access() -> Result<(), String> {
    use std::io::ErrorKind;
    use std::process::Command;

    // Combined v1 + v2 rules — merge of libfreenect's 66-kinect.rules
    // (v1 PIDs) and libfreenect2's 90-kinect2.rules (v2 PIDs), inlined
    // so the installer works from a release build that doesn't ship
    // the submodules. MODE 0666 to drop the GROUP=video requirement
    // libfreenect's upstream rule uses (no group membership to manage).
    const RULES: &str = "\
# Kinect v1 + v2 — USB access for libfreenect / libfreenect2.
# Installed by headtracking-demo. Upstream sources: libfreenect
# 66-kinect.rules and libfreenect2 90-kinect2.rules, merged here.
# v1 Xbox 360 (model 1414):
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02ae\", MODE=\"0666\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02ad\", MODE=\"0666\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02b0\", MODE=\"0666\"
# v1 Kinect for Windows (model 1473):
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02c2\", MODE=\"0666\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02be\", MODE=\"0666\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02bf\", MODE=\"0666\"
# v2 sensor + firmware-update + adapter hub:
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02c4\", MODE=\"0666\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02d8\", MODE=\"0666\"
SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02d9\", MODE=\"0666\"
";
    let staged = std::env::temp_dir().join("headtracking-90-kinect.rules");
    std::fs::write(&staged, RULES).map_err(|e| format!("staging rules file: {e}"))?;
    let inner = format!(
        "set -e; install -m 0644 '{}' /etc/udev/rules.d/90-kinect.rules; \
         udevadm control --reload-rules; udevadm trigger",
        staged.display()
    );
    let manual = format!("sudo sh -c '{inner}'");

    let mut last: Option<String> = None;
    for elevator in ["pkexec", "sudo"] {
        match Command::new(elevator)
            .arg("sh")
            .arg("-c")
            .arg(&inner)
            .status()
        {
            Ok(s) if s.success() => return Ok(()),
            Ok(s) => last = Some(format!("`{elevator}` exited with {s}")),
            Err(e) if e.kind() == ErrorKind::NotFound => continue,
            Err(e) => last = Some(format!("could not run `{elevator}`: {e}")),
        }
    }
    Err(match last {
        Some(why) => format!("{why}.\nRun it yourself, then click 'rescan':\n{manual}"),
        None => format!("no `pkexec` or `sudo` on PATH.\nRun this, then click 'rescan':\n{manual}"),
    })
}

#[cfg(target_os = "windows")]
fn fix_kinect_access() -> Result<(), String> {
    use std::os::windows::process::CommandExt;
    use std::process::Command;
    const CREATE_NO_WINDOW: u32 = 0x0800_0000;

    let exe = std::env::current_exe().map_err(|e| format!("locating the executable: {e}"))?;
    let dir = exe.parent().unwrap_or_else(|| std::path::Path::new("."));
    // Release-ZIP layout puts the script under `setup\`; tolerate a flat
    // layout and a `bin\` subdir too.
    let candidates = [
        dir.join("setup").join("setup.ps1"),
        dir.join("setup.ps1"),
        dir.join("..").join("setup").join("setup.ps1"),
    ];
    let Some(script) = candidates.iter().find(|p| p.is_file()) else {
        return Err(format!(
            "couldn't find setup\\setup.ps1 next to {} — re-download the full release ZIP, or \
             bind WinUSB by hand with Zadig (see the project README, \"Manual Zadig \
             fallback\"): v1 → Xbox NUI Audio/Camera/Motor; v2 → Xbox NUI Sensor (045E:02C4).",
            dir.display()
        ));
    };
    let workdir = script.parent().unwrap_or(dir);

    // Two-step trampoline: spawn a hidden, *non*-elevated PowerShell
    // whose only job is to call `Start-Process -Verb RunAs` on
    // `powershell.exe -ExecutionPolicy RemoteSigned -File setup.ps1`. The
    // RunAs verb is what pops the UAC consent dialog; the spawned
    // elevated PowerShell opens its own visible console (we don't pass
    // `-WindowStyle Hidden`) so the user can read the script's output
    // and the type-`yes` prompt. `-ExecutionPolicy RemoteSigned` (not
    // Bypass): release ZIPs ship setup.ps1 Authenticode-signed, so the
    // policy VERIFIES the signature on Mark-of-the-Web files while
    // still running local/dev builds (no MOTW, no signature needed).
    //
    // We use this trampoline instead of a direct Win32 ShellExecuteW
    // call to keep the dependency tree std-only — the price is one
    // ephemeral PowerShell process (~100 ms) before UAC fires.
    // Quoting matters twice here: the paths sit inside PowerShell
    // single-quoted strings (escape ' as ''), and `Start-Process` joins
    // -ArgumentList items with spaces WITHOUT quoting — so the -File path
    // must carry its own embedded double quotes or an extraction dir like
    // `C:\Users\Jean Dupont\Downloads` breaks the elevated launch.
    let ps_quote = |p: &std::path::Path| p.display().to_string().replace('\'', "''");
    let inner = format!(
        "$ErrorActionPreference='Stop'; Start-Process powershell -Verb RunAs \
         -WorkingDirectory '{workdir}' \
         -ArgumentList '-NoProfile','-ExecutionPolicy','RemoteSigned','-File','\"{script}\"'",
        workdir = ps_quote(workdir),
        script = ps_quote(script),
    );
    let status = Command::new("powershell.exe")
        .args(["-NoProfile", "-NonInteractive", "-Command", &inner])
        .creation_flags(CREATE_NO_WINDOW)
        .status()
        .map_err(|e| format!("spawning powershell.exe: {e}"))?;
    if status.success() {
        return Ok(());
    }
    // Most common failure: the user clicked "No" on the UAC prompt,
    // which makes `Start-Process -Verb RunAs` throw under
    // `$ErrorActionPreference='Stop'` → launcher exits non-zero.
    Err(format!(
        "launcher PowerShell exited with {status} (often = UAC cancelled, or PowerShell \
         missing). Open an elevated PowerShell yourself and run:\n\
         powershell -NoProfile -ExecutionPolicy RemoteSigned -File \"{}\"",
        script.display()
    ))
}

#[cfg(not(any(target_os = "linux", target_os = "windows")))]
fn fix_kinect_access() -> Result<(), String> {
    Err("nothing to set up — libusb opens the Kinect without a driver on this platform".to_string())
}

// ============================================================ App state

/// Duration each of the two settle phases lasts when switching backend.
const SWITCH_SETTLE: Duration = Duration::from_millis(500);
/// How long to wait for the first RGB frame after opening before bouncing the
/// stream. The Kinect v1 in particular sometimes needs a stream restart.
const FIRST_FRAME_WAIT: Duration = Duration::from_millis(900);
/// Stream bounces (stop+start / reopen) attempted before giving up on a frame.
const MAX_STREAM_BOUNCES: u8 = 3;

/// Backend-switch state machine (see [`App::switch_state`]).
enum SwitchState {
    /// Not switching — the active backend (if any) matches the selection.
    Idle,
    /// The old backend was just closed; waiting out the release settle.
    Closing(Instant),
    /// Settle done; showing "opening" before spawning the capture thread.
    Opening(Instant),
    /// Capture thread spawned; polling its [`Startup`] handshake (device open +
    /// first-frame recovery all happen on that thread). Goes live on
    /// `Startup::Live`, or errors on `Failed` / after [`STARTUP_TIMEOUT`].
    Waiting {
        worker: CaptureWorker,
        since: Instant,
    },
}

struct App {
    selected: Backend,
    /// Last USB verdict and when it was taken. Enumerating the bus is cheap
    /// but not free, and the UI repaints far faster than a cable gets moved,
    /// so it is refreshed on a timer rather than per frame.
    /// Last USB snapshot, produced on a worker thread. `None` until the first
    /// probe lands.
    ///
    /// Enumerating the bus is not free: ~0.8 ms on this Linux box, and an
    /// order of magnitude more on Windows, where it goes through SetupAPI and
    /// the registry once per device. The old device brief called
    /// `usb_check::check` inline on every frame it was open — 60 full bus
    /// enumerations a second on the render thread — and on a Windows cabinet
    /// that is the whole frame budget and then some. The interface stopped
    /// dead while the popup was up. Nothing about USB is read on the render
    /// thread any more.
    usb_cache: Option<(usb_check::Sensor, UsbSnapshot)>,
    /// In-flight probe, if any.
    usb_probe: Option<mpsc::Receiver<UsbSnapshot>>,
    /// The USB window, opened from the toolbar badge. A plain window, never a
    /// modal: someone reading their bus tree usually wants to unplug something
    /// and watch the list change, which a modal makes impossible.
    /// Unit the point-of-view figures are shown in. Presentation only.
    pov_unit: PovUnit,
    usb_window_open: bool,
    /// What the freshly opened device can deliver, shown once. `None` when
    /// there is nothing to say or the user has dismissed it.
    brief: Option<DeviceBrief>,
    available: Vec<BackendEntry>,
    active: Option<Active>,
    error: Option<String>,
    logs: Arc<Mutex<VecDeque<String>>>,
    /// A Kinect is on the USB bus but the OS-level access prerequisite
    /// isn't set up (Linux: missing libfreenect2 udev rule; Windows: no
    /// libusb/WinUSB driver bound) — show the one-click fix banner. Always
    /// `false` on macOS, where libusb needs nothing.
    kinect_access_hint: bool,
    /// Outcome of the last "fix it" click: `Ok` carries a follow-up note,
    /// `Err` carries a fallback hint (manual command line / "re-download
    /// the release ZIP"). Cleared on rescan.
    kinect_access_result: Option<Result<String, String>>,
    /// Next time the access probe re-runs while the fix banner is up, so a
    /// finished driver install (button OR manual setup.ps1) triggers an
    /// automatic rescan — nobody reads "click rescan" notes mid-install.
    kinect_access_recheck_at: Option<Instant>,
    /// Outcome of the last "Screenshot" click — kept until the next click
    /// (or backend change) so the user has time to read the saved path.
    /// `Ok` carries the full saved path, `Err` carries the failure reason.
    screenshot_status: Option<Result<std::path::PathBuf, String>>,
    /// Viewport rotation for a physically-mounted (rotated) pincab display.
    /// Defaults to the player-facing "270°", which once applied is
    /// egui-rotate's `CW90`: the rotated screen inverts the apparent
    /// handedness, so our toolbar degrees run opposite to the egui enum
    /// (see [`rotation_label`]). The ⟳ button cycles +90° clockwise from
    /// the player's seat. Read by [`DemoShell::redraw`].
    rotation: Rotation,
    /// Set by the Quit button; [`DemoShell`] exits the event loop next frame.
    should_quit: bool,
    /// Physical lockbar width in mm (= sidebar separation at the lockbar).
    /// The metric ruler for the monocular (webcam) scale and the cabinet
    /// calibration; a cross-check for the depth backends. User-set here in the
    /// demo (the toolbar field, shown only for a webcam input — depth cameras
    /// measure scale directly); pulled from the VPX table config in the plugin.
    /// Defaults to `calibration::LOCKBAR_WIDTH_MM` (61 cm widebody).
    lockbar_width_mm: f32,
    /// Live-tunable 1€ filter knobs for the head pose (applied to all axes in
    /// [`App::poll`]). `min_cutoff` sets the baseline smoothing when still;
    /// `beta` how fast the cutoff rises with motion — the low default `beta`
    /// felt laggy on quick lateral moves, so it's tunable from the bench row.
    head_filter_min_cutoff: f32,
    head_filter_beta: f32,
    /// Median spike-gate window in frames (odd, 1 = off).
    median_window_frames: usize,
    /// Debug: bypass everything between raw detection and the pose — the 1€
    /// filter, the lockbar-centred picker (→ largest head), and most of the
    /// depth-sample gate — to see which stage is dropping the head.
    bypass_filters: bool,
    /// Stream shown in the camera view (info bar selection). Reset to
    /// [`StreamKind::Rgb`] on every backend change, since not every device
    /// offers the same streams.
    selected_stream: StreamKind,
    /// Colour-camera exposure, Kinect v2 only. Held here because the camera
    /// has no getter: the sliders read back what we last applied, not what the
    /// hardware currently believes.
    color_exposure: ColorExposureMode,
    /// "Share a capture" window toggle + state.
    contribute_open: bool,
    /// The informed-consent checkbox (see the privacy notice). Gates the
    /// share button; in-memory for the session.
    consent_checked: bool,
    /// Background uploader for shared captures (write-only Nextcloud drop).
    uploader: contribute::Uploader,
    /// Whether this machine can reach the drop at all, checked before the
    /// panel offers to upload anything. A pincab behind a firewall used to
    /// find out only after capturing — 35 files, no error, nothing received.
    drop_reach: ReachState,
    /// Receiver for the reachability probe running off the UI thread.
    drop_probe: Option<mpsc::Receiver<contribute::Reach>>,
    /// Stem of the last capture shared, shown so the user can note it (needed
    /// to request a removal, since the drop is anonymous).
    contrib_last: Option<String>,
    /// Where the user wants their own copy of a shared capture, if anywhere.
    contrib_local: LocalCopy,
    /// Folder the last shared capture's files actually reached. The window
    /// used to announce a folder it had never written to — which is how a
    /// tester ended up hunting a directory that did not exist.
    contrib_saved_in: Option<std::path::PathBuf>,
    /// Why the last local save failed, if it did. Shown next to the folder, so
    /// "saved locally" is never claimed without proof.
    contrib_save_error: Option<String>,
    /// Embedded help thumbnails (example capture + the two setup photos),
    /// decoded to textures on first open of the panel.
    contrib_thumbs: Option<[TextureHandle; 3]>,
    /// Backend-switch state machine. Switching device closes the old one, then
    /// waits ~500 ms ("closing"), then ~500 ms ("opening") before the actual
    /// open — ~1 s of USB settle so a Kinect released by one backend is ready
    /// before the next grabs it (see [`App::ensure_active`]). Non-blocking: the
    /// waits are checked each frame so the UI stays live and shows the status.
    switch_state: SwitchState,
    /// Parallax validation window toggle (🪟 button). When on, the central
    /// panel shows the camera feed with the off-axis 3D scene stacked below
    /// it (see `docs/parallax-validation-window.md`).
    parallax_enabled: bool,
    /// Show the camera view rectified onto the playfield plane — the cabinet
    /// as it would look from square-on. Off by default: it is a check, not a
    /// way to watch yourself.
    flatten_view: bool,
    /// Where the last flattened frame put the lockbar, in fractions of the
    /// view. Drives the two vertical guides; `None` whenever the flattened
    /// view is off or the anchor has not locked.
    flatten_guides: Option<FlattenGuides>,
    /// egui handle to the parallax scene's offscreen colour texture, set
    /// by [`DemoShell::redraw`] each frame the window is on (the FBO lives
    /// with the GL context in `DemoShell`, not here). `None` when off.
    parallax_tex: Option<egui::TextureId>,
    /// Eye-position source for the parallax scene (Live / Mouse / Auto-orbit).
    parallax_eye_mode: ParallaxEye,
    /// Current parallax eye in screen-space mm (+x right, +y up, +z toward
    /// the viewer), recomputed each frame by [`App::update_parallax_eye`] and
    /// read by [`DemoShell::redraw`] to build the off-axis projection.
    parallax_eye: [f32; 3],
    /// Debug gain + per-axis sign flips for the Live mapping — the bench
    /// knobs from the spec (used to find the right signs to bake into
    /// `camera/mapping.rs`, not a product calibration step).
    parallax_gain: f32,
    parallax_invert: [bool; 3],
    /// Playfield inclination (degrees from horizontal), a key input alongside
    /// the sidebar width: the VPX screen is the near-flat playfield, so the
    /// head motion is tilted by `90° − inclination` for a truthful parallax
    /// (see `update_parallax_eye`). VPX exposes this per table.
    table_incl_deg: f32,
    /// Last parallax panel rect (egui logical px) so Mouse mode can map the
    /// pointer. Logical space = post-egui-rotate, so the mouse "rotates" with
    /// the window for free.
    parallax_panel_rect: Option<Rect>,
    /// Eye Z for Mouse mode, nudged by the scroll wheel.
    parallax_mouse_z: f32,
    /// Parallax panel aspect (width/height), set in [`App::draw_parallax_view`]
    /// and read by [`DemoShell::redraw`] to size the FBO + projection so the
    /// scene fills the panel without distortion. 1-frame lag, imperceptible.
    parallax_aspect: f32,
}

/// Where the parallax scene's eye position comes from. Three sources so the
/// scene can be exercised without a camera (dev machine).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ParallaxEye {
    /// Real head pose (lockbar-relative deltas) — final validation, camera on.
    Live,
    /// Pointer over the panel drives X/Y, wheel drives Z — dev machine.
    Mouse,
    /// Slow programmed sinusoid — hands-free demo / video capture.
    AutoOrbit,
}

impl ParallaxEye {
    fn label(self) -> &'static str {
        match self {
            ParallaxEye::Live => "Live (head)",
            ParallaxEye::Mouse => "Mouse",
            ParallaxEye::AutoOrbit => "Auto-orbit",
        }
    }
}

impl App {
    /// Target interval between continuous (animation) repaints. Capped to just
    /// above the live camera rate — new pixels only exist at camera cadence, so
    /// rendering faster only re-paints identical frames. A 1.3× headroom over
    /// the measured input FPS avoids aliasing (a repaint landing just before
    /// each new frame). With no active camera we fall back to 60 Hz for a
    /// smooth menu; the input-event path repaints immediately regardless.
    fn target_frame_interval(&self) -> Duration {
        // FIXED cap, NOT a feedback off the measured fps. The old
        // `1 / (in_fps * 1.3)` was self-referential — the redraw cadence was set
        // from the measured fps, which is itself set by the redraw cadence. That
        // control loop oscillated and pinned every backend well below the real
        // camera rate (the ~20 fps variable ceiling). A fixed ~60 Hz cap lets the
        // loop poll fast enough to catch every camera frame; `in_fps` then settles
        // at the true camera/processing rate, and we still don't spin (VSync off).
        Duration::from_secs_f32(1.0 / 60.0)
    }

    /// Ask a worker thread for a fresh USB snapshot. Cheap to call twice: a
    /// probe already in flight wins.
    fn start_usb_probe(&mut self, sensor: usb_check::Sensor) {
        if self.usb_probe.is_some() {
            return;
        }
        let (tx, rx) = mpsc::channel();
        if std::thread::Builder::new()
            .name("usb-probe".into())
            .spawn(move || {
                let _ = tx.send(UsbSnapshot {
                    report: usb_check::check(sensor),
                    tree: usb_check::topology(sensor),
                });
            })
            .is_ok()
        {
            self.usb_probe = Some(rx);
        }
    }

    /// Collect a finished probe. Never blocks: a bus that takes 200 ms to
    /// enumerate must cost the render thread nothing.
    fn poll_usb_probe(&mut self, sensor: usb_check::Sensor) {
        let Some(rx) = &self.usb_probe else {
            return;
        };
        match rx.try_recv() {
            Ok(snap) => {
                self.usb_cache = Some((sensor, snap));
                self.usb_probe = None;
            }
            Err(mpsc::TryRecvError::Empty) => {}
            // The worker died without answering; drop it so the next click
            // can try again rather than waiting forever on a dead channel.
            Err(mpsc::TryRecvError::Disconnected) => self.usb_probe = None,
        }
    }

    fn label_for(&self, backend: Backend) -> String {
        self.available
            .iter()
            .find(|e| e.backend == backend)
            .map(|e| e.label.clone())
            .unwrap_or_else(|| match backend {
                Backend::None => "None (off)".to_string(),
                Backend::KinectV1 => "Kinect v1".to_string(),
                Backend::KinectV2 => "Kinect v2".to_string(),
                Backend::Webcam(i) => format!("Webcam #{i}"),
            })
    }
}

struct Active {
    backend: Backend,
    /// Width of the last displayed frame, so the read-out can say which focal
    /// the distance was worked out with.
    last_frame_w: u32,
    intrinsics: Intrinsics,
    rgb_texture: Option<TextureHandle>,
    /// Dedicated capture thread that owns the device + models + 1€ filter and
    /// publishes the latest processed frame. The GL thread never touches the
    /// device — it only reads [`CaptureWorker::latest`] (see [`App::poll`]).
    worker: CaptureWorker,
    /// UI-only Kinect v1 tilt/LED widget state (`Some` only for v1). The
    /// device commands go to the capture thread via [`CaptureWorker::cmd_tx`];
    /// the live tilt read-out comes from [`CaptureWorker::tilt_state`].
    v1_ui: Option<V1Controls>,
    /// `frame_id` of the last frame the GL thread uploaded, so a repaint that
    /// finds no new capture skips the texture upload + OUT count.
    last_consumed_id: u64,
    /// Cumulative capture count last seen — the IN counter advances by the
    /// delta so every captured frame is counted even if the GL thread, running
    /// slower, skipped some (that's exactly the `out < in` gap we want to show).
    last_captured: u64,
    /// Cumulative depth / IR counts last seen, same delta trick as
    /// [`Self::last_captured`] — these streams run at their own rate, so their
    /// deltas per GL tick are what make `depth`/`ir` fps meaningful.
    last_depth_count: u64,
    last_ir_count: u64,
    /// When each stream last delivered (from the latest snapshot). Aged at draw
    /// time by [`App::stream_bar`], so a stalled device goes red on its own
    /// even though no further snapshot arrives to say so.
    last_rgb_at: Option<Instant>,
    last_ir_at: Option<Instant>,
    last_depth_at: Option<Instant>,
    /// Frame dims the pose / anchor coords live in (see [`LatestFrame`]); the
    /// overlay normalises through these, not through the displayed texture.
    pose_src: (u32, u32),
    /// Nominal spec of the opened webcam mode (w, h, fps), for the info bar.
    /// `None` for the Kinects, whose specs are fixed.
    cam_spec: Option<(u32, u32, u32)>,
    /// 1€-smoothed head pose (computed capture-side), copied out of the latest
    /// published frame each GL tick for the crosshair + VPX delta panel.
    last_head: Option<HeadPixel>,
    baseline: Option<Baseline>,
    /// Lockbar quad consumed by the overlay + 3D-centre maths, derived from the
    /// anchor detection (its closed bar is the lockbar, of known physical width).
    last_lockbar: Option<headtracking::calibration::LockbarQuadRgb>,
    /// Latest cabinet-frame geometry from the anchor model (RGB): lockbar
    /// corners, sidebars, depth vanishing point, lockbar width, lateral offset.
    last_anchor: Option<anchor::AnchorGeometry>,
    /// `true` once the anchor worker froze its best detection (calibration
    /// locked) — gates the camera-pose read-out so we only show a settled
    /// estimate, not the warmup's churn.
    anchor_locked: bool,
    /// Latest RGB888 frame (width, height, bytes) — kept so the "Screenshot" /
    /// "Share a capture" buttons can write it without re-grabbing. `Arc` so
    /// copying it out of the published frame is a refcount bump, not a memcpy.
    last_rgb_frame: Option<(u32, u32, Arc<Vec<u8>>, FrameLayout)>,
    /// Latest depth frame (width, height, millimetres) — kept for the "Share a
    /// capture" export. `None` for the webcam backend (no depth). `u16` mm (v1
    /// native; v2's `f32` mm rounded on capture). `Arc` for cheap hand-off.
    last_depth: Option<Arc<(u32, u32, Vec<u16>)>>,
    /// Kinect v2 depth projected into the colour framing, published only while
    /// the depth stream is on screen. Preferred over [`Self::last_depth`] for
    /// the *view* (the overlay then needs no coordinate mapping); the raw
    /// `last_depth` stays the one exported with a shared capture.
    last_depth_color: Option<Arc<(u32, u32, Vec<u16>)>>,
    /// Latest IR frame (width, height, intensity as `u16`). Always present on
    /// the v2 (IR shares the depth listener); on the v1 only while its video
    /// endpoint is switched to IR, or after the momentary switch
    /// [`App::share_capture`] performs. Never for the webcam.
    last_ir: Option<Arc<(u32, u32, Vec<u16>)>>,
    last_pose: Option<blazepose::Pose>,
    /// Live perf counters (inference times, CPU%, in/out FPS). Reset per
    /// backend open — each device gets a fresh measurement window.
    metrics: Metrics,
}

impl Active {
    /// Build the GL-side handle once the capture thread reports `Live`. The
    /// device + models live on the worker's thread; this side only renders.
    fn new_live(worker: CaptureWorker, intrinsics: Intrinsics) -> Self {
        let backend = worker.backend;
        Self {
            last_frame_w: 0,
            backend,
            intrinsics,
            rgb_texture: None,
            worker,
            v1_ui: if backend == Backend::KinectV1 {
                Some(V1Controls::new())
            } else {
                None
            },
            last_consumed_id: 0,
            last_captured: 0,
            last_depth_count: 0,
            last_ir_count: 0,
            last_rgb_at: None,
            last_ir_at: None,
            last_depth_at: None,
            pose_src: (0, 0),
            cam_spec: match backend {
                Backend::Webcam(id) => webcam_nominal_spec(id),
                _ => None,
            },
            last_head: None,
            baseline: None,
            last_lockbar: None,
            last_anchor: None,
            anchor_locked: false,
            last_rgb_frame: None,
            last_depth: None,
            last_depth_color: None,
            last_ir: None,
            last_pose: None,
            metrics: Metrics::new(),
        }
    }
}

// ============================================================ Capture thread
//
// The device (Kinect / webcam), the pose + anchor models, and the 1€ filter
// all live on ONE dedicated thread — the device handle never crosses a thread
// boundary (so no `Send` bound is required on it). The GL thread only reads the
// latest processed frame through a lock-free `ArcSwapOption`, decoupling render
// (parallax, overlays) from capture: `in` counts every captured frame, `out`
// only the ones the GL thread actually showed, so they diverge honestly.

/// Everything the device pipeline owns on the capture thread. Also used by the
/// headless `--capture` path (single-threaded there — see `poll_active_headless`).
struct Capture {
    backend: Backend,
    intrinsics: Intrinsics,
    inner: Inner,
    /// Cross-process exclusivity on the device — held for the capture's
    /// lifetime so the VPX plugin / a cron capture / a second demo fail
    /// fast instead of fighting for the USB stream. `None` only between
    /// `new_capture` and `open_backend` finishing.
    hwlock: Option<headtracking::hwlock::HwLock>,
    blaze_worker: BlazePoseWorker,
    anchor_worker: AnchorWorker,
    pose_filter: filter_alias::OneEuroPose3D,
    /// Median spike gate ahead of the 1€ (same component as the plugin).
    median_gate: headtracking::filter::MedianGate,
    started_at: Instant,
    baseline: Option<Baseline>,
    last_pose: Option<blazepose::Pose>,
    last_head: Option<HeadPixel>,
    last_anchor: Option<anchor::AnchorGeometry>,
    last_lockbar: Option<headtracking::calibration::LockbarQuadRgb>,
    last_rgb_frame: Option<(u32, u32, Arc<Vec<u8>>, FrameLayout)>,
    last_depth: Option<Arc<(u32, u32, Vec<u16>)>>,
    last_ir: Option<Arc<(u32, u32, Vec<u16>)>>,
    /// Cumulative depth frames grabbed. Diagnostic only — the depth stream runs
    /// on its own listener, independent of RGB.
    depth_frames: u64,
    /// Cumulative IR frames grabbed. On the v2 this is diagnostic only (IR is
    /// exported with a shared capture, nothing else); on a v1 switched to IR it
    /// *is* the tracking input, since that sensor can't stream colour and IR at
    /// once. Worth measuring either way because IR is **actively illuminated**
    /// (the sensor lights the scene itself), so unlike the auto-exposed colour
    /// stream its rate should NOT drop in a dark room — the reference against
    /// which a low `in` tells us the colour camera, not USB or our code, is the
    /// bottleneck.
    ir_frames: u64,
    /// When each stream last delivered a frame — the info bar's green/red.
    /// `None` until the first frame of that kind ever arrives.
    last_rgb_at: Option<Instant>,
    last_ir_at: Option<Instant>,
    last_depth_at: Option<Instant>,
    /// Kinect v2 only: libfreenect2's depth↔colour registration, plus the
    /// colour intrinsics that go with it. `None` on every other backend (the
    /// v1 shares one sensor framing, the webcam has no depth), and `None` on
    /// the v2 if the model couldn't be built — see [`Capture::reg_warned`].
    registration: Option<freenect2::Registration>,
    color_intr: Intrinsics,
    /// Reused Kinect v2 frame buffers. The colour frame is 8.3 MB and depth
    /// and IR 868 KB each: the shim copies straight into these, so a poll is
    /// one memcpy and, after the first frame, no allocation at all.
    v2_rgb: freenect2::RgbFrame,
    v2_ir: freenect2::IrFrame,
    v2_depth: freenect2::DepthFrame,
    /// Colour-space depth around the head only — what the tracker actually
    /// reads out of the registration. See [`Capture::bigdepth`], which is now
    /// built for the on-screen depth *view* and nothing else.
    head_window: Vec<f32>,
    /// Reused `1920 × 1082` colour-space depth buffer (~8 MB) — allocated once
    /// per v2 open so the per-frame registration never reallocates.
    bigdepth: Vec<f32>,
    /// Scratch BGRX plane handed to the registration. Its *contents are never
    /// read* for the bigdepth output (libfreenect2 only samples the colour
    /// pixels to fill the `registered` frame, which we discard), but `apply()`
    /// hard-checks the buffer's dimensions — so a zeroed plane is enough, and
    /// we avoid retaining the real 8 MB colour frame just to satisfy it.
    rgb_scratch: Vec<u8>,
    /// `true` once [`Capture::bigdepth`] holds a registration of the current
    /// depth frame; cleared when the registration is unavailable or failed.
    bigdepth_ok: bool,
    /// Same, for [`Capture::head_window`] — the head path no longer depends on
    /// the full-plane projection, so it needs its own validity flag.
    head_window_ok: bool,
    /// One-shot: the windowed and full projections have been compared on a
    /// live frame (see the depth arm of [`Capture::poll_once`]).
    window_checked: bool,
    /// Latest registration cost (ms) and a one-shot flag so a failing or
    /// missing registration warns once instead of every frame.
    reg_ms: f32,
    /// Time spent getting one frame out of the driver and into the shape the
    /// pose model reads. Measured because it is the largest per-frame cost on
    /// a modest CPU and used to be invisible — the perf line summed the model
    /// and the registration, and could not explain the frame rate it printed
    /// beside them.
    copy_ms: f32,
    /// Per-stage smoothing cost for the latest head.
    filter_us: FilterUs,
    /// Time spent turning the camera's surface into a packed RGB buffer.
    /// Non-zero on the webcam only: a 1080p MJPG frame is fully decoded here
    /// before the pose model takes a 224x224 square of it, and whether that
    /// is a real ceiling or noise beside inference was never measured.
    convert_ms: f32,
    reg_warned: bool,
    /// Which stream the user is viewing. Only used capture-side to skip
    /// building the (2 M pixel) colour-space depth view unless it's on screen.
    selected_stream: StreamKind,
    /// When the colour frame was last polled+converted on the v2 while
    /// tracking on IR — throttles that path to ~2.5 Hz (see the v2 arm).
    last_rgb_refresh: Option<Instant>,
    /// Pending [`CaptureCmd::RefreshRgb`] ack: while `Some`, the v2 arm
    /// bypasses the colour throttle once and fires the ack right after the
    /// fresh conversion lands.
    rgb_refresh_ack: Option<mpsc::Sender<()>>,
    /// Colour-space depth view (v2 only): bigdepth cropped to the 1920×1080
    /// colour window, in `u16` mm with `0` = no reading. `Some` only while the
    /// depth stream is selected — it shares the colour framing, so the pose and
    /// anchor overlays land on it without any coordinate mapping.
    depth_color: Option<Arc<(u32, u32, Vec<u16>)>>,
    /// Dimensions of the frame the current pose / anchor were computed on.
    /// Travels with the snapshot so the overlay maps through the right space
    /// whichever stream the user is looking at.
    pose_src: (u32, u32),
    head_ms: f32,
    anchor_ms: f32,
}

impl Capture {
    /// Push the live-tunable 1€ knobs onto the filter (cheap; keeps state).
    /// The UI values drive X/Y; Z gets both knobs halved — depth is noisier
    /// (median over a small window jitters as the sampling point shifts a
    /// pixel), so it stays tighter. This preserves the per-axis profile from
    /// [`make_pose_filter`] instead of flattening all three axes to the UI
    /// values, which silently discarded the Z tuning.
    fn set_filter_params(&mut self, min_cutoff: f32, beta: f32, median_window: usize) {
        self.median_gate.set_window(median_window);
        let xy = filter_alias::OneEuroParams {
            min_cutoff_hz: min_cutoff,
            beta,
            derivative_cutoff_hz: 1.0,
        };
        let z = filter_alias::OneEuroParams {
            min_cutoff_hz: min_cutoff * 0.5,
            beta: beta * 0.5,
            derivative_cutoff_hz: 1.0,
        };
        self.pose_filter.set_params_per_axis([xy, xy, z]);
    }

    /// Poll the device once. Runs BlazePose + the anchor model, deprojects the
    /// head from depth (when `compute_head`), and refreshes every `last_*`
    /// buffer. Returns `true` iff a new RGB frame was grabbed this call.
    fn poll_once(&mut self, bypass: bool, compute_head: bool) -> bool {
        let depth_min = if bypass { 4 } else { 16 };
        let mut got_rgb = false;
        // Same for infrared: the anchor submits whichever stream is on screen.
        let mut got_ir = false;
        match &mut self.inner {
            Inner::KinectV2 { device, .. } => {
                // Unlike the v1, both streams flow at once here, so selecting IR
                // is a choice of *tracking input*, not a mode switch: BlazePose
                // reads whichever the user picked. Worth having — the IR emitter
                // lights the scene itself and holds 30 Hz in a dark cabinet,
                // where the auto-exposed colour camera halves to 15.
                let track_on_ir = self.selected_stream == StreamKind::Ir;
                // While tracking on IR, the colour frame feeds nothing but the
                // stream-bar liveness and an occasional screenshot — yet
                // converting it costs an 8.3 MB read + 6 MB write per frame at
                // 30 Hz. Throttle the poll+convert to ~2.5 Hz (under the 500 ms
                // liveness window, so the RGB chip stays green). The future VPX
                // plugin should go further and simply open with
                // `start_streams(false, true)` — never decode colour at all.
                let want_rgb = !track_on_ir
                    || self.rgb_refresh_ack.is_some()
                    || self
                        .last_rgb_refresh
                        .is_none_or(|t| t.elapsed() >= Duration::from_millis(400));
                let t_copy = Instant::now();
                if want_rgb && device.poll_rgb_into(&mut self.v2_rgb) {
                    self.last_rgb_at = Some(Instant::now());
                    self.last_rgb_refresh = Some(Instant::now());
                    // A contribution capture asked for an un-throttled frame:
                    // it just landed, let the GL thread proceed.
                    if let Some(ack) = self.rgb_refresh_ack.take() {
                        let _ = ack.send(());
                    }
                    let (w, h) = (self.v2_rgb.width, self.v2_rgb.height);
                    // Handed on exactly as libfreenect2 delivered it. Taking
                    // the buffer leaves the poll target empty, so the next
                    // poll allocates a fresh one and nothing is copied twice:
                    // the repack into packed RGB used to read 8.3 MB and
                    // write 6.2 MB per frame, on this thread, so that two
                    // models could sample a 256-pixel patch and the display
                    // could convert it a second time into RGBA.
                    let pixels = Arc::new(std::mem::take(&mut self.v2_rgb.data));
                    if !track_on_ir {
                        // What the frame costs before anything looks at it:
                        // one 8.3 MB copy out of the driver slot, and nothing
                        // else now that the colour repack is gone. Recorded
                        // only on the stream that feeds the pose model — while
                        // tracking on IR the colour path runs at 2.5 Hz for
                        // the preview alone, and charging it to every frame
                        // would libel the IR path.
                        self.copy_ms = t_copy.elapsed().as_secs_f32() * 1000.0;
                        got_rgb = true;
                        self.blaze_worker
                            .submit(Arc::clone(&pixels), w, h, FrameLayout::Bgrx8888);
                        self.pose_src = (w, h);
                        let pose_out = self.blaze_worker.snapshot();
                        self.last_pose = pose_out.pose;
                        if pose_out.ms > 0.0 {
                            self.head_ms = pose_out.ms;
                        }
                    }
                    self.last_rgb_frame = Some((w, h, pixels, FrameLayout::Bgrx8888));
                }
                // IR streams on the same listener as depth. Keep the latest for
                // the capture export; f32 intensity rounds into u16.
                let t_ir = Instant::now();
                if device.poll_ir_into(&mut self.v2_ir) {
                    let ir = &self.v2_ir;
                    self.ir_frames += 1;
                    self.last_ir_at = Some(Instant::now());
                    let mm: Vec<u16> = ir.data.iter().map(|&v| v as u16).collect();
                    if track_on_ir {
                        // Auto-level the raw intensity before handing it over —
                        // v2 IR is a wide-range 16-bit signal, and the untouched
                        // high byte is nearly black.
                        got_rgb = true;
                        let gray = autolevel_gray8_raw(&mm, false);
                        let rgb888 = Arc::new(gray8_to_rgb888(&gray));
                        // Same measurement as the colour path, on the stream
                        // that actually feeds the model here: poll copy plus
                        // the widening and levelling the model reads.
                        self.copy_ms = t_ir.elapsed().as_secs_f32() * 1000.0;
                        self.blaze_worker
                            .submit(rgb888, ir.width, ir.height, FrameLayout::Rgb888);
                        self.pose_src = (ir.width, ir.height);
                        let pose_out = self.blaze_worker.snapshot();
                        self.last_pose = pose_out.pose;
                        if pose_out.ms > 0.0 {
                            self.head_ms = pose_out.ms;
                        }
                    }
                    self.last_ir = Some(Arc::new((ir.width, ir.height, mm)));
                    got_ir = true;
                }
                if device.poll_depth_into(&mut self.v2_depth) {
                    let depth = &self.v2_depth;
                    self.depth_frames += 1;
                    self.last_depth_at = Some(Instant::now());
                    // Project depth into colour space. The colour and depth
                    // lenses sit ~5 cm apart with different fields of view, so
                    // scaling a colour coordinate into the 512×424 depth grid
                    // by resolution ratio samples the wrong pixel — worse the
                    // closer the player is. libfreenect2's registration is the
                    // proper correction.
                    //
                    // Two different questions, deliberately priced apart. The
                    // tracker wants depth at *one* place, so it gets the 17×17
                    // window around the head. The depth *view* wants the whole
                    // colour plane, so it pays for the full `bigdepth` — and
                    // only while it is on screen. Asking the full projection
                    // for a head was ~12 ms a frame: an 8.3 MB infinity fill,
                    // 3.3 M scattered min-writes and a registered colour image
                    // nobody ever read, to answer a question about 289 pixels.
                    //
                    // Both skipped while tracking on IR: the pose is already in
                    // the depth grid there, so the colour projection buys
                    // nothing — and those milliseconds are exactly the headroom
                    // the IR path exists to gain.
                    self.bigdepth_ok = false;
                    self.head_window_ok = false;
                    if !track_on_ir && let Some(reg) = self.registration.as_mut() {
                        let t0 = Instant::now();
                        let mut ok = true;
                        if let Some(pose) = self.last_pose.as_ref() {
                            let (hx, hy) = head_center_xy(pose);
                            self.head_window_ok = reg.depth_window(
                                &depth.data,
                                (hx as i32, hy as i32),
                                HEAD_WINDOW_HALF,
                                &mut self.head_window,
                            );
                            ok = self.head_window_ok;
                        }
                        self.reg_ms = t0.elapsed().as_secs_f32() * 1000.0;
                        // The windowed projection is meant to be *identical*
                        // to the patch of the full one it replaces. Whenever
                        // both happen to be live (the depth view is on screen
                        // and a pose is found) check that once, on real data,
                        // and say so: this is the only place the claim can be
                        // tested — a Registration needs a device, so no unit
                        // test can reach it.
                        if !self.window_checked && self.head_window_ok && self.bigdepth_ok {
                            self.window_checked = true;
                            if let Some(pose) = self.last_pose.as_ref() {
                                let (hx, hy) = head_center_xy(pose);
                                report_window_match(
                                    &self.head_window,
                                    &self.bigdepth,
                                    (hx as i32, hy as i32),
                                );
                            }
                        }
                        if !ok && !self.reg_warned {
                            self.reg_warned = true;
                            warn!(
                                "kinect v2: depth registration failed — falling back to \
                                 depth-grid scaling for head distance"
                            );
                        }
                    }
                    // Nothing displays the colour-space depth any more: the
                    // view it existed for is gone, and it cost a 2 M pixel
                    // conversion per frame. The depth the plugin uses is the
                    // sensor's own grid, which needs no projection.
                    self.depth_color = None;
                    if compute_head {
                        let head = self
                            .last_pose
                            .as_ref()
                            .and_then(|p| {
                                // On the colour stream a Kinect is treated as a
                                // plain webcam would treat itself: nominal focal
                                // from the frame width, distance triangulated
                                // from shoulder width, depth sensor untouched.
                                // That is not a fallback -- it is the point. The
                                // same board, same scene, same instant then
                                // yields both the webcam estimate and the
                                // sensor's own measurement, so the contribution
                                // screenshots say directly how far the
                                // no-depth method is off.
                                if !track_on_ir && self.selected_stream == StreamKind::Rgb {
                                    return head_pixel_from_pose_webcam(p, 1920, 1080);
                                }
                                if track_on_ir {
                                    // Tracking on IR: the pose already lives in
                                    // the depth camera's own grid (IR and depth
                                    // are the same sensor, same 512×424, pixel
                                    // aligned), so sampling is exact by
                                    // construction — no registration involved.
                                    head_pixel_from_pose_depth(
                                        p,
                                        self.pose_src,
                                        &depth.data,
                                        (depth.width, depth.height),
                                        &self.intrinsics,
                                        depth_min,
                                    )
                                } else if self.head_window_ok {
                                    head_pixel_from_window(
                                        p,
                                        &self.head_window,
                                        &self.color_intr,
                                        depth_min,
                                    )
                                } else {
                                    head_pixel_from_pose_depth(
                                        p,
                                        (1920, 1080),
                                        &depth.data,
                                        (depth.width, depth.height),
                                        &self.intrinsics,
                                        depth_min,
                                    )
                                }
                            })
                            .map(|mut h| {
                                // v2 colour frame is mirrored → negate X so the
                                // left/right POV travel matches v1. bigdepth
                                // inherits the same colour framing, so the
                                // correction applies to both paths alike.
                                h.x_mm = -h.x_mm;
                                h
                            });
                        let (smoothed, filter_us) = smooth_head(
                            head,
                            &mut self.pose_filter,
                            &mut self.median_gate,
                            self.started_at,
                            bypass,
                        );
                        self.filter_us = filter_us;
                        capture_baseline(&mut self.baseline, smoothed);
                        self.last_head = smoothed;
                    }
                    self.last_depth = Some(Arc::new((
                        self.v2_depth.width,
                        self.v2_depth.height,
                        self.v2_depth.data.iter().map(|&z| z as u16).collect(),
                    )));
                }
            }
            Inner::KinectV1 { device, .. } => {
                // One endpoint, one image: in IR mode there is no colour frame
                // at all, so the IR frame *becomes* the pipeline input. Gray8 is
                // expanded to RGB888 (byte replicated across R/G/B) and fed to
                // BlazePose and the anchor model exactly like a colour frame —
                // worth it because the IR emitter lights the scene itself, so it
                // holds its rate in a dark cabinet where the auto-exposed colour
                // camera halves.
                if device.video_stream() == freenect::VideoStream::Ir {
                    if let Some(ir) = device.poll_ir_frame() {
                        got_rgb = true;
                        self.ir_frames += 1;
                        self.last_ir_at = Some(Instant::now());
                        let rgb888 = Arc::new(gray8_to_rgb888(&ir.data));
                        self.blaze_worker.submit(
                            Arc::clone(&rgb888),
                            ir.width,
                            ir.height,
                            FrameLayout::Rgb888,
                        );
                        self.pose_src = (ir.width, ir.height);
                        let pose_out = self.blaze_worker.snapshot();
                        self.last_pose = pose_out.pose;
                        if pose_out.ms > 0.0 {
                            self.head_ms = pose_out.ms;
                        }
                        // Displayed through the IR path (16-bit widened), and
                        // kept as the RGB-shaped frame the rest of the pipeline
                        // (anchor submit, screenshots) expects.
                        self.last_ir = Some(Arc::new((
                            ir.width,
                            ir.height,
                            ir.data.iter().map(|&v| u16::from(v)).collect(),
                        )));
                        got_ir = true;
                        self.last_rgb_frame =
                            Some((ir.width, ir.height, rgb888, FrameLayout::Rgb888));
                    }
                } else if let Some(rgb) = device.poll_rgb() {
                    got_rgb = true;
                    self.last_rgb_at = Some(Instant::now());
                    let rgb888 = Arc::new(rgb.data);
                    self.blaze_worker.submit(
                        Arc::clone(&rgb888),
                        rgb.width,
                        rgb.height,
                        FrameLayout::Rgb888,
                    );
                    self.pose_src = (rgb.width, rgb.height);
                    let pose_out = self.blaze_worker.snapshot();
                    self.last_pose = pose_out.pose;
                    if pose_out.ms > 0.0 {
                        self.head_ms = pose_out.ms;
                    }
                    self.last_rgb_frame =
                        Some((rgb.width, rgb.height, rgb888, FrameLayout::Rgb888));
                }
                if let Some(depth) = device.poll_depth() {
                    self.depth_frames += 1;
                    self.last_depth_at = Some(Instant::now());
                    if compute_head {
                        // Sampled straight from the native u16 grid — the old
                        // full-frame u16→f32 widen copied 1.2 MB per frame to
                        // feed a 17×17 window.
                        let head = self.last_pose.as_ref().and_then(|p| {
                            // Same deal as the v2: on the colour stream this
                            // Kinect estimates distance the way a webcam has to,
                            // so the two methods can be compared on one board.
                            if self.selected_stream == StreamKind::Rgb {
                                return head_pixel_from_pose_webcam(p, 640, 480);
                            }
                            head_pixel_from_pose_depth(
                                p,
                                (640, 480),
                                &depth.data,
                                (depth.width, depth.height),
                                &self.intrinsics,
                                depth_min,
                            )
                        });
                        let (smoothed, filter_us) = smooth_head(
                            head,
                            &mut self.pose_filter,
                            &mut self.median_gate,
                            self.started_at,
                            bypass,
                        );
                        self.filter_us = filter_us;
                        capture_baseline(&mut self.baseline, smoothed);
                        self.last_head = smoothed;
                    }
                    self.last_depth =
                        Some(Arc::new((depth.width, depth.height, depth.data.clone())));
                }
            }
            Inner::Webcam { camera } => {
                if let Some(rgb) = camera.poll_rgb() {
                    got_rgb = true;
                    self.convert_ms = rgb.convert_ms;
                    self.last_rgb_at = Some(Instant::now());
                    let rgb888 = Arc::new(rgb.data);
                    self.blaze_worker.submit(
                        Arc::clone(&rgb888),
                        rgb.width,
                        rgb.height,
                        FrameLayout::Rgb888,
                    );
                    self.pose_src = (rgb.width, rgb.height);
                    let pose_out = self.blaze_worker.snapshot();
                    self.last_pose = pose_out.pose;
                    if pose_out.ms > 0.0 {
                        self.head_ms = pose_out.ms;
                    }
                    if compute_head {
                        let head = self
                            .last_pose
                            .as_ref()
                            .and_then(|p| head_pixel_from_pose_webcam(p, rgb.width, rgb.height));
                        let (smoothed, filter_us) = smooth_head(
                            head,
                            &mut self.pose_filter,
                            &mut self.median_gate,
                            self.started_at,
                            bypass,
                        );
                        self.filter_us = filter_us;
                        capture_baseline(&mut self.baseline, smoothed);
                        self.last_head = smoothed;
                    }
                    self.last_rgb_frame =
                        Some((rgb.width, rgb.height, rgb888, FrameLayout::Rgb888));
                }
            }
        }
        // Anchor model: submit the stream that is on screen, prepared the way
        // that stream's model was trained. Infrared goes through the same
        // square-root and contrast equalisation the corpus was rendered with --
        // feeding it the raw rescale would hand the model a distribution it
        // never saw, which is the whole reason there are two models.
        let anchor_frame = if self.selected_stream == StreamKind::Ir {
            self.last_ir.as_ref().map(|ir| {
                let (w, h, raw) = &**ir;
                let sensor = match self.backend {
                    Backend::KinectV1 => anchor::IrSensor::KinectV1,
                    _ => anchor::IrSensor::KinectV2,
                };
                (
                    *w,
                    *h,
                    Arc::new(anchor::prepare_ir_rgb888(raw, *w, *h, sensor)),
                    FrameLayout::Rgb888,
                    anchor::Stream::Infrared,
                )
            })
        } else {
            self.last_rgb_frame
                .clone()
                .map(|(w, h, buf, layout)| (w, h, buf, layout, anchor::Stream::Colour))
        };
        if let Some((w, h, buf, layout, stream)) = anchor_frame {
            let fresh = if stream == anchor::Stream::Infrared {
                got_ir
            } else {
                got_rgb
            };
            if fresh && !self.anchor_worker.is_locked() {
                // Arc bump — the warmup window used to full-frame-copy here,
                // right when the 1280² inference is already at its priciest.
                self.anchor_worker
                    .submit(Arc::clone(&buf), w, h, layout, stream);
            }
            let a_out = self.anchor_worker.snapshot();
            if let Some(g) = a_out.geom {
                self.last_anchor = Some(g);
                self.last_lockbar = Some(anchor_to_quad(&g, w, h));
            }
            if self.anchor_worker.is_locked() {
                self.anchor_ms = 0.0;
            } else if a_out.ms > 0.0 {
                self.anchor_ms = a_out.ms;
            }
        }
        got_rgb
    }

    /// Build an immutable snapshot for the GL thread. Only called right after
    /// `poll_once` returned `true`, so `last_rgb_frame` is `Some`.
    fn snapshot_frame(&self, frame_id: u64) -> LatestFrame {
        let (w, h, pixels, layout) = self
            .last_rgb_frame
            .clone()
            .expect("snapshot_frame with no RGB frame");
        LatestFrame {
            frame_id,
            w,
            h,
            pixels,
            layout,
            depth: self.last_depth.clone(),
            depth_color: self.depth_color.clone(),
            ir: self.last_ir.clone(),
            last_rgb_at: self.last_rgb_at,
            last_ir_at: self.last_ir_at,
            last_depth_at: self.last_depth_at,
            pose_src_w: self.pose_src.0,
            pose_src_h: self.pose_src.1,
            pose: self.last_pose.clone(),
            head: self.last_head,
            baseline: self.baseline,
            anchor: self.last_anchor,
            lockbar: self.last_lockbar,
            head_ms: self.head_ms,
            anchor_ms: self.anchor_ms,
            reg_ms: self.reg_ms,
            filter_us: self.filter_us,
            convert_ms: self.convert_ms,
            copy_ms: self.copy_ms,
            anchor_locked: self.anchor_worker.is_locked(),
        }
    }
}

/// Which of the device's streams the camera view shows.
///
/// On the Kinect v2 all three flow at once and this is purely a display
/// choice. On the Kinect v1 colour and IR share one USB endpoint, so picking
/// [`StreamKind::Ir`] genuinely *stops* the colour stream — which is the point:
/// the info bar turns RGB red and IR green, and the trade-off becomes obvious
/// without a word of documentation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StreamKind {
    Rgb,
    Ir,
    /// Kinect v1 only: the 1280×1024 video mode. Nothing is disabled for it --
    /// tracking simply runs at the 10 fps the sensor gives in this mode.
    RgbHigh,
}

impl StreamKind {
    fn label(self) -> &'static str {
        match self {
            StreamKind::Rgb => "RGB",
            StreamKind::Ir => "IR",
            StreamKind::RgbHigh => "RGB hi-res",
        }
    }
}

/// A stream a backend can deliver, with its **nominal** (datasheet) spec — the
/// measured rates live in the `perf:` line, this bar answers "what does this
/// device offer, and am I getting it right now?".
struct StreamSpec {
    kind: StreamKind,
    w: u32,
    h: u32,
    fps: u32,
}

impl StreamSpec {
    fn caption(&self) -> String {
        // "max" matters: the v2's colour sensor legitimately halves to ~15 fps
        // in a dim room, and a bare "30" invites reading that as a fault. And
        // `p` means progressive scan, not frames per second.
        format!(
            "{} {}×{} · {} fps max",
            self.kind.label(),
            self.w,
            self.h,
            self.fps
        )
    }
}

/// Streams each backend advertises. Kinect figures are the fixed sensor modes;
/// the webcam's come from the opened format (SDL only ever gives us one mode).
/// What a device can deliver, in the tester's terms.
///
/// The numbers in the perf read-out mean nothing without this. `render` is the
/// UI repaint rate and happily sits at 60 while the camera feeds 10; a Kinect
/// v2 colour stream at 15 fps is not a fault but an auto-exposure lengthening
/// its integration time in a dim room. Both are read as breakage by someone
/// seeing them for the first time, so the device says so itself on open.
struct DeviceBrief {
    title: String,
    /// One line per stream: what it is, and the rate to expect.
    streams: Vec<String>,
    /// Things that look like faults and are not.
    notes: Vec<String>,
}

fn device_brief(backend: Backend) -> Option<DeviceBrief> {
    let (title, streams, notes) = match backend {
        Backend::KinectV2 => (
            "Kinect v2 — what to expect",
            vec![
                "colour 1920x1080 up to 30 fps".to_string(),
                "infrared 512x424 at 30 fps".to_string(),
                "depth 512x424 at 30 fps".to_string(),
            ],
            vec![
                "Colour halves to ~15 fps in a dim room: the sensor lengthens its \
                 exposure. Turn a light on and it returns to 30. Infrared and depth \
                 are unaffected -- the sensor lights the scene itself."
                    .to_string(),
                "It needs USB 3.0 and its own power adapter. On USB 2.0 it still \
                 opens, then drops depth packets."
                    .to_string(),
            ],
        ),
        Backend::KinectV1 => (
            "Kinect v1 — what to expect",
            vec![
                "colour 640x480 at 30 fps (1280x1024 drops to 10)".to_string(),
                "infrared 640x480 at 30 fps".to_string(),
                "depth 640x480 at 30 fps".to_string(),
            ],
            vec![
                "The high-resolution modes are a third of the rate: 1280x1024 runs \
                 at 10 fps, not 30."
                    .to_string(),
                "USB 2.0 is enough for this one.".to_string(),
            ],
        ),
        Backend::Webcam(_) => (
            "Webcam — what to expect",
            vec!["rate and resolution are whatever the camera reports".to_string()],
            vec![
                "Most webcams drop their frame rate in low light exactly like the \
                 Kinect v2 colour sensor does."
                    .to_string(),
            ],
        ),
        Backend::None => return None,
    };
    let mut notes = notes;
    if matches!(backend, Backend::KinectV1 | Backend::KinectV2) {
        notes.push(
            "In VPX, the plugin tracks on infrared only -- it never opens the colour \
             stream on a Kinect. So a slow colour rate here costs the game nothing, \
             and tracking works in the dark. 30 head positions per second is far more \
             than head movement needs."
                .to_string(),
        );
    }
    notes.push(
        "`render` in the perf read-out is how often the window repaints, not what \
         the camera delivers. It is normally higher, because the smoothed pose \
         keeps moving between captures."
            .to_string(),
    );
    Some(DeviceBrief {
        title: title.to_string(),
        streams,
        notes,
    })
}

fn stream_specs(backend: Backend, cam: Option<(u32, u32, u32)>) -> Vec<StreamSpec> {
    let s = |kind, w, h, fps| StreamSpec { kind, w, h, fps };
    match backend {
        Backend::KinectV2 => vec![
            s(StreamKind::Rgb, 1920, 1080, 30),
            s(StreamKind::Ir, 512, 424, 30),
        ],
        Backend::KinectV1 => vec![
            s(StreamKind::Rgb, 640, 480, 30),
            s(StreamKind::RgbHigh, 1280, 1024, 10),
            s(StreamKind::Ir, 640, 480, 30),
        ],
        Backend::Webcam(_) => {
            let (w, h, fps) = cam.unwrap_or((640, 480, 30));
            vec![s(StreamKind::Rgb, w, h, fps)]
        }
        Backend::None => Vec::new(),
    }
}

/// What the driver delivered and what we let die in its slot, cumulative
/// since the device opened.
///
/// The listener keeps one frame per stream and overwrites it on the next
/// delivery, so a reader slower than the sensor silently loses the
/// difference. Counting both halves is the only way a log can tell a slow
/// Kinect from a slow us — the two look identical in a frame rate.
#[derive(Clone, Copy, Default)]
struct SensorCounts {
    delivered: u64,
    dropped: u64,
}

/// Immutable snapshot the capture thread publishes once per new RGB frame,
/// read by the GL thread through [`CaptureWorker::latest`].
struct LatestFrame {
    /// Increments per published frame; the GL thread uploads only when it changes.
    frame_id: u64,
    w: u32,
    h: u32,
    /// The frame as the driver produced it — see `layout`. Converting to
    /// packed RGB on the capture thread bought nothing: the models sample it,
    /// and the display converts to RGBA anyway.
    pixels: Arc<Vec<u8>>,
    layout: FrameLayout,
    depth: Option<Arc<(u32, u32, Vec<u16>)>>,
    /// Kinect v2 only, and only while the depth stream is selected: the depth
    /// map projected into the 1920×1080 colour framing, so the overlay lands on
    /// it with no coordinate mapping. Falls back to `depth` when absent.
    depth_color: Option<Arc<(u32, u32, Vec<u16>)>>,
    ir: Option<Arc<(u32, u32, Vec<u16>)>>,
    /// When each stream last delivered, for the green/red info bar. Published
    /// as instants rather than pre-computed booleans so the GL thread ages them
    /// at *draw* time: if the device stalls outright no new snapshot arrives,
    /// and baked-in booleans would sit there claiming green over a frozen
    /// image. Timestamps go stale on their own.
    last_rgb_at: Option<Instant>,
    last_ir_at: Option<Instant>,
    last_depth_at: Option<Instant>,
    /// Frame dimensions the pose / anchor coordinates belong to. The overlay
    /// must normalise through these, NOT through the displayed image: showing
    /// the 512×424 depth view while the pose came from a 1920×1080 colour
    /// frame would otherwise scatter the skeleton across the wrong pixels.
    pose_src_w: u32,
    pose_src_h: u32,
    pose: Option<blazepose::Pose>,
    head: Option<HeadPixel>,
    baseline: Option<Baseline>,
    anchor: Option<anchor::AnchorGeometry>,
    lockbar: Option<headtracking::calibration::LockbarQuadRgb>,
    head_ms: f32,
    anchor_ms: f32,
    /// Cost of decoding this frame into packed RGB (ms); webcam only.
    convert_ms: f32,
    /// Cost of getting this frame out of the driver and into the shape the
    /// pose model reads (ms). Kinect v2 only — the other backends hand over a
    /// ready buffer.
    copy_ms: f32,
    /// Cost of the v2 depth-to-colour alignment for this frame (ms); `0.0`
    /// when it isn't running.
    reg_ms: f32,
    /// Per-stage smoothing cost for this frame.
    filter_us: FilterUs,
    anchor_locked: bool,
}

/// Device I/O the GL thread asks the capture thread to run (things outside the
/// steady poll): the Kinect v1 motor + LED, a baseline reset, and the v1
/// video grabs that may need a momentary mode switch.
/// What the Kinect v2's colour camera should do about exposure.
///
/// Colour only, and that is a hardware fact rather than an omission:
/// libfreenect2 exposes `setColorAutoExposure` / `SemiAuto` / `Manual` for the
/// colour camera and nothing at all for IR or depth, whose integration the
/// firmware runs by itself. Which is also why a dim room halves the colour
/// rate while depth and IR hold ~30 Hz.
#[derive(Clone, Copy, PartialEq, Debug)]
enum ColorExposureMode {
    /// The camera decides. `compensation` in [-2.0, 2.0]: negative
    /// underexposes, positive overexposes. This is how the camera opens.
    Auto { compensation: f32 },
    /// Flicker-free: the requested time is rounded down to a whole mains-light
    /// period (10 ms at 50 Hz, 8.33 at 60) and gain compensates.
    SemiAuto { pseudo_ms: f32 },
    /// Shutter and analog gain fixed. `integration_ms` in (0.0, 66.0] —
    /// 66 ms being one whole frame period at 15 Hz, which is why anything past
    /// ~33 ms costs half the colour frame rate.
    Manual {
        integration_ms: f32,
        analog_gain: f32,
    },
}

impl Default for ColorExposureMode {
    fn default() -> Self {
        Self::Auto { compensation: 0.0 }
    }
}

enum CaptureCmd {
    SetTilt(f32),
    SetLed(freenect::LedState),
    ResetBaseline,
    /// Grab one v1 IR frame (borrows the video endpoint if RGB is live, then
    /// restores it); reply with the frame or `None` on failure / non-v1.
    GrabIrV1(mpsc::Sender<Option<freenect::IrFrame>>),
    /// Grab one v1 colour frame — the mirror of [`Self::GrabIrV1`], for the
    /// contribution capture while the IR stream is selected. A contribution
    /// must always export every stream a camera has, whatever is on screen.
    GrabRgbV1(mpsc::Sender<Option<freenect::RgbFrame>>),
    /// Ask for one un-throttled colour conversion (v2 tracking-on-IR throttles
    /// colour to ~2.5 Hz); the ack fires once a fresh frame has been converted,
    /// or immediately when no throttle is active. Used by the contribution
    /// capture so `_raw.png` is never ~400 ms stale.
    RefreshRgb(mpsc::Sender<()>),
    /// Run the cabinet anchor detection again from scratch (↻ Recalibrate):
    /// thaws the frozen best-of-warmup and drops the geometry derived from it,
    /// for when the camera has been moved or re-aimed.
    Recalibrate,
    /// Colour-camera exposure (Kinect v2 only). Applied on the capture thread
    /// because that is where the device lives; a no-op elsewhere.
    SetColorExposure(ColorExposureMode),
    /// Display-stream choice. Only the Kinect v1 acts on it at the device level
    /// (colour and IR are mutually exclusive there); every other backend keeps
    /// all its streams running and this stays a pure display concern.
    SelectStream(StreamKind),
}

/// How long after its last frame a stream still counts as "live" in the info
/// bar. Longer than a frame interval at 15 fps so a slow stream reads steady
/// green, short enough that a stopped one turns red almost at once.
const STREAM_LIVE_FOR: Duration = Duration::from_millis(500);

/// Where the capture thread's open + first-frame handshake is at. Polled by the
/// GL thread (see [`App::ensure_active`]) so the UI never blocks on the open.
enum Startup {
    Pending,
    Live(Intrinsics),
    Failed(String),
}

/// GL-side handle to the capture thread. Dropping it stops + joins the thread,
/// which drops the device (the only place it lives).
/// Counters the capture thread keeps current on *every* turn of its loop,
/// whether or not it published a frame.
///
/// The perf read-out used to be fed only from inside the GL thread's
/// "a new frame was published, consume it" branch — capture rate, sensor rate
/// and drop share alike. So the instrument went dark at exactly the moment it
/// was needed: a field log (2026-09-03) held thirteen minutes of
/// `cam 0.0 fps | ir 0.0 fps` with the ` of N sensor` suffix gone, and
/// that line cannot tell "the sensor delivered nothing" apart from "we stopped
/// reading it" — the two produce identical output. Exported captures from the
/// same session proved frames were still flowing. Reading these atomics
/// unconditionally is what makes the next such report answer the question by
/// itself.
/// How often the capture thread re-reads the driver's own delivered/dropped
/// counters. Ten times per perf window: enough to be current, rare enough not
/// to fight the frame listener for the device mutex.
const SENSOR_STATS_EVERY: Duration = Duration::from_millis(100);

#[derive(Default)]
struct CaptureVitals {
    /// Cumulative frames pulled out of the driver, per stream.
    captured: AtomicU64,
    depth_captured: AtomicU64,
    ir_captured: AtomicU64,
    /// Whether this backend's driver can report what it delivered at all.
    /// Only libfreenect2 can; elsewhere the capture rate already *is* the
    /// sensor rate, and a figure of 0 would be a lie rather than a finding.
    reports_sensor: AtomicBool,
    rgb_delivered: AtomicU64,
    rgb_dropped: AtomicU64,
    depth_delivered: AtomicU64,
    depth_dropped: AtomicU64,
    /// The colour camera's own auto-exposure, as `f32` bits. This is what
    /// explains a `cam` rate that sits at half the `ir` one: the v2
    /// auto-exposes and halves to 15 Hz in a dim room, while IR and depth hold
    /// ~30 Hz off their own illuminator. Without it, reading `cam 14.9` left
    /// the reader to guess whether the room was dark or we were struggling —
    /// two problems with opposite answers.
    exposure_bits: AtomicU32,
    gain_bits: AtomicU32,
    /// Step of the sensor's own frame clock, 0.125 ms per unit: 266 at 30 Hz,
    /// 533 at 15 Hz. The camera's own account of its cadence, owing nothing to
    /// any counter of ours.
    frame_step: AtomicU32,
}

impl CaptureVitals {
    /// Our own counters: three relaxed stores, cheap enough for every turn of
    /// the loop including the idle ones.
    fn store_counts(&self, captured: u64, cap: &Capture) {
        self.captured.store(captured, Ordering::Relaxed);
        self.depth_captured
            .store(cap.depth_frames, Ordering::Relaxed);
        self.ir_captured.store(cap.ir_frames, Ordering::Relaxed);
    }

    /// The driver's own counters. Deliberately *not* on the hot path:
    /// `stream_stats` takes the device mutex the frame listener also wants,
    /// and the idle branch of the loop turns over every millisecond — reading
    /// it there would add contention precisely when the pipeline is already
    /// struggling. The perf window is a second wide, so a refresh every
    /// [`SENSOR_STATS_EVERY`] costs nothing in resolution.
    fn store_sensor(&self, cap: &Capture) {
        let Inner::KinectV2 { device, .. } = &cap.inner else {
            return;
        };
        let e = device.color_exposure();
        self.exposure_bits
            .store(e.exposure.to_bits(), Ordering::Relaxed);
        self.gain_bits.store(e.gain.to_bits(), Ordering::Relaxed);
        self.frame_step.store(e.frame_step, Ordering::Relaxed);
        let s = device.stream_stats();
        self.rgb_delivered.store(s.rgb_received, Ordering::Relaxed);
        self.rgb_dropped.store(s.rgb_dropped, Ordering::Relaxed);
        self.depth_delivered
            .store(s.depth_received, Ordering::Relaxed);
        self.depth_dropped.store(s.depth_dropped, Ordering::Relaxed);
        self.reports_sensor.store(true, Ordering::Release);
    }

    /// What the colour camera's auto-exposure is doing, and the sensor's own
    /// frame step. `None` on a backend that cannot report it.
    fn exposure(&self) -> Option<(f32, f32, u32)> {
        self.reports_sensor.load(Ordering::Acquire).then(|| {
            (
                f32::from_bits(self.exposure_bits.load(Ordering::Relaxed)),
                f32::from_bits(self.gain_bits.load(Ordering::Relaxed)),
                self.frame_step.load(Ordering::Relaxed),
            )
        })
    }

    /// `(captured, depth, ir, sensor)` — `sensor` is `None` on a backend whose
    /// driver cannot report, which is not the same as one reporting zero.
    fn read(&self) -> (u64, u64, u64, Option<(SensorCounts, SensorCounts)>) {
        let sensor = self.reports_sensor.load(Ordering::Acquire).then(|| {
            (
                SensorCounts {
                    delivered: self.rgb_delivered.load(Ordering::Relaxed),
                    dropped: self.rgb_dropped.load(Ordering::Relaxed),
                },
                SensorCounts {
                    delivered: self.depth_delivered.load(Ordering::Relaxed),
                    dropped: self.depth_dropped.load(Ordering::Relaxed),
                },
            )
        });
        (
            self.captured.load(Ordering::Relaxed),
            self.depth_captured.load(Ordering::Relaxed),
            self.ir_captured.load(Ordering::Relaxed),
            sensor,
        )
    }
}

struct CaptureWorker {
    backend: Backend,
    latest: Arc<ArcSwapOption<LatestFrame>>,
    /// Live counters, readable whatever the GL thread is doing.
    vitals: Arc<CaptureVitals>,
    cmd_tx: mpsc::Sender<CaptureCmd>,
    /// Latest Kinect v1 tilt/accel read-out (v1 only), refreshed by the loop.
    tilt_state: Arc<Mutex<Option<freenect::TiltState>>>,
    startup: Arc<Mutex<Startup>>,
    filter_min_cutoff: Arc<AtomicU32>,
    filter_beta: Arc<AtomicU32>,
    median_window: Arc<AtomicUsize>,
    bypass: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl CaptureWorker {
    fn spawn(backend: Backend) -> Self {
        let latest = Arc::new(ArcSwapOption::empty());
        let vitals = Arc::new(CaptureVitals::default());
        let (cmd_tx, cmd_rx) = mpsc::channel();
        let tilt_state = Arc::new(Mutex::new(None));
        let startup = Arc::new(Mutex::new(Startup::Pending));
        let filter_min_cutoff = Arc::new(AtomicU32::new(1.0f32.to_bits()));
        let filter_beta = Arc::new(AtomicU32::new(0.0f32.to_bits()));
        let median_window = Arc::new(AtomicUsize::new(3));
        let bypass = Arc::new(AtomicBool::new(false));
        let stop = Arc::new(AtomicBool::new(false));
        let (latest_t, vitals_t, tilt_t, startup_t, mc_t, beta_t, mw_t, byp_t, stop_t) = (
            Arc::clone(&latest),
            Arc::clone(&vitals),
            Arc::clone(&tilt_state),
            Arc::clone(&startup),
            Arc::clone(&filter_min_cutoff),
            Arc::clone(&filter_beta),
            Arc::clone(&median_window),
            Arc::clone(&bypass),
            Arc::clone(&stop),
        );
        let handle = std::thread::Builder::new()
            .name("capture".into())
            .spawn(move || {
                capture_thread_loop(
                    backend, cmd_rx, &latest_t, &vitals_t, &tilt_t, &startup_t, &mc_t, &beta_t,
                    &mw_t, &byp_t, &stop_t,
                );
            })
            .expect("spawn capture thread");
        Self {
            backend,
            latest,
            vitals,
            cmd_tx,
            tilt_state,
            startup,
            filter_min_cutoff,
            filter_beta,
            median_window,
            bypass,
            stop,
            handle: Some(handle),
        }
    }

    /// Hand the live 1€ / bypass knobs to the capture thread (cheap atomics).
    fn set_filter(&self, min_cutoff: f32, beta: f32, median_window: usize, bypass: bool) {
        self.filter_min_cutoff
            .store(min_cutoff.to_bits(), Ordering::Relaxed);
        self.filter_beta.store(beta.to_bits(), Ordering::Relaxed);
        self.median_window.store(median_window, Ordering::Relaxed);
        self.bypass.store(bypass, Ordering::Relaxed);
    }
}

impl Drop for CaptureWorker {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// Longest the GL thread waits for the capture thread to resolve `Startup`
/// before declaring the open failed — generous cover over the internal
/// bounce/settle budget so a wedged thread can't hang the switch machine.
const STARTUP_TIMEOUT: Duration =
    Duration::from_millis(FIRST_FRAME_WAIT.as_millis() as u64 * (MAX_STREAM_BOUNCES as u64 + 2));

/// The capture thread: open the device here (so its handle never leaves this
/// thread), confirm the stream flows, then poll → publish until stopped.
#[allow(clippy::too_many_arguments)]
fn capture_thread_loop(
    backend: Backend,
    cmd_rx: mpsc::Receiver<CaptureCmd>,
    latest: &Arc<ArcSwapOption<LatestFrame>>,
    vitals: &Arc<CaptureVitals>,
    tilt_state: &Arc<Mutex<Option<freenect::TiltState>>>,
    startup: &Arc<Mutex<Startup>>,
    filter_min_cutoff: &Arc<AtomicU32>,
    filter_beta: &Arc<AtomicU32>,
    median_window: &Arc<AtomicUsize>,
    bypass: &Arc<AtomicBool>,
    stop: &Arc<AtomicBool>,
) {
    let mut cap = match open_backend(backend) {
        Ok(c) => c,
        Err(e) => {
            *startup.lock() = Startup::Failed(e);
            return;
        }
    };
    // Always the live anchor model — hand-fixed calibration files are gone
    // on purpose: when the model is wrong, we want to SEE it, not paper
    // over it with hand-placed lines.
    info!(?backend, "anchor: live model path");
    // Confirm the stream actually flows before going live (Kinect v1 can open
    // yet never deliver a frame until the stream is bounced) — same recovery as
    // the old `SwitchState::Waiting`, but on this thread so the UI never blocks.
    let mut live = false;
    'bounce: for bounce in 0..=MAX_STREAM_BOUNCES {
        let deadline = Instant::now() + FIRST_FRAME_WAIT;
        while Instant::now() < deadline {
            if stop.load(Ordering::Acquire) {
                return;
            }
            if cap.poll_first_rgb() {
                live = true;
                break 'bounce;
            }
            std::thread::sleep(Duration::from_millis(20));
        }
        if bounce == MAX_STREAM_BOUNCES {
            break;
        }
        warn!(
            ?backend,
            next = bounce + 1,
            "capture: no first frame, bouncing stream"
        );
        if let Err(e) = cap.bounce_stream() {
            *startup.lock() = Startup::Failed(e);
            return;
        }
    }
    if !live {
        *startup.lock() = Startup::Failed(format!(
            "{}: no video after {MAX_STREAM_BOUNCES} stream restarts — check the cable / USB",
            backend_slug(backend)
        ));
        return;
    }
    *startup.lock() = Startup::Live(cap.intrinsics);
    info!(
        ?backend,
        fx = cap.intrinsics.fx,
        fy = cap.intrinsics.fy,
        "capture thread live"
    );

    // One pose inference per frame — BlazePose (~7 ms) keeps up, no rate cap.
    cap.blaze_worker.set_min_interval_ms(0);
    let mut frame_id = 0u64;
    let mut captured = 0u64;
    // Far enough in the past that the first turn publishes immediately.
    let mut last_sensor_stats = Instant::now() - SENSOR_STATS_EVERY;
    let mut last_tilt_refresh = Instant::now() - Duration::from_millis(600);
    while !stop.load(Ordering::Acquire) {
        // Device I/O commands from the UI (v1 motor/LED, baseline, IR grab).
        while let Ok(cmd) = cmd_rx.try_recv() {
            match cmd {
                CaptureCmd::SetTilt(deg) => {
                    if let Inner::KinectV1 { device, .. } = &cap.inner
                        && let Err(e) = device.set_tilt_degrees(deg)
                    {
                        warn!(?e, "set_tilt failed");
                    }
                }
                CaptureCmd::SetLed(led) => {
                    if let Inner::KinectV1 { device, .. } = &cap.inner
                        && let Err(e) = device.set_led(led)
                    {
                        warn!(?e, "set_led failed");
                    }
                }
                CaptureCmd::ResetBaseline => cap.baseline = None,
                CaptureCmd::SetColorExposure(mode) => {
                    if let Inner::KinectV2 { device, .. } = &cap.inner {
                        match mode {
                            ColorExposureMode::Auto { compensation } => {
                                device.set_color_auto_exposure(compensation);
                            }
                            ColorExposureMode::SemiAuto { pseudo_ms } => {
                                device.set_color_semi_auto_exposure(pseudo_ms);
                            }
                            ColorExposureMode::Manual {
                                integration_ms,
                                analog_gain,
                            } => device.set_color_manual_exposure(integration_ms, analog_gain),
                        }
                        info!(?mode, "colour exposure applied");
                    }
                }
                CaptureCmd::SelectStream(kind) => {
                    // Remembered so the v2 only pays for the colour-space depth
                    // view while it's actually on screen.
                    cap.selected_stream = kind;
                    // Only the v1 trades one stream for another at the device
                    // level; everywhere else every stream keeps flowing and the
                    // choice is purely what the GL thread draws.
                    if let Inner::KinectV1 { device, .. } = &mut cap.inner {
                        let want = match kind {
                            StreamKind::Ir => freenect::VideoStream::Ir,
                            // Depth streams on its own endpoint, so viewing it
                            // leaves the colour stream running.
                            StreamKind::RgbHigh => freenect::VideoStream::RgbHigh,
                            StreamKind::Rgb => freenect::VideoStream::Rgb,
                        };
                        if device.video_stream() != want {
                            match device.set_video_stream(want) {
                                Ok(()) => info!(?want, "kinect v1: video stream switched"),
                                Err(e) => warn!(?e, ?want, "kinect v1: video switch failed"),
                            }
                        }
                    }
                }
                CaptureCmd::Recalibrate => {
                    cap.anchor_worker.recalibrate();
                    // Drop what the old lock produced as well: the overlay
                    // would otherwise keep drawing a lockbar the user has just
                    // told us is wrong, until a new detection lands.
                    cap.last_anchor = None;
                    cap.last_lockbar = None;
                    cap.anchor_ms = 0.0;
                }
                CaptureCmd::GrabIrV1(reply) => {
                    let frame = if let Inner::KinectV1 { device, .. } = &mut cap.inner {
                        match device.capture_ir(3) {
                            Ok(f) => Some(f),
                            Err(e) => {
                                warn!("contribution: v1 IR capture failed: {e}");
                                None
                            }
                        }
                    } else {
                        None
                    };
                    let _ = reply.send(frame);
                }
                CaptureCmd::GrabRgbV1(reply) => {
                    let frame = if let Inner::KinectV1 { device, .. } = &mut cap.inner {
                        match device.capture_rgb(3) {
                            Ok(f) => Some(f),
                            Err(e) => {
                                warn!("contribution: v1 RGB capture failed: {e}");
                                None
                            }
                        }
                    } else {
                        None
                    };
                    let _ = reply.send(frame);
                }
                CaptureCmd::RefreshRgb(reply) => {
                    // Only the v2-tracking-on-IR path throttles colour; in every
                    // other state the latest colour frame is already fresh, so
                    // ack straight away instead of parking the sender.
                    let throttled = matches!(cap.inner, Inner::KinectV2 { .. })
                        && cap.selected_stream == StreamKind::Ir;
                    if throttled {
                        cap.rgb_refresh_ack = Some(reply);
                    } else {
                        let _ = reply.send(());
                    }
                }
            }
        }
        // Refresh the v1 tilt/accel read-out every 500 ms (USB roundtrip).
        if matches!(cap.inner, Inner::KinectV1 { .. })
            && last_tilt_refresh.elapsed() >= Duration::from_millis(500)
        {
            if let Inner::KinectV1 { device, .. } = &cap.inner {
                match device.tilt_state() {
                    Ok(s) => *tilt_state.lock() = Some(s),
                    Err(e) => warn!(?e, "kinect v1: tilt_state refresh failed"),
                }
            }
            last_tilt_refresh = Instant::now();
        }
        cap.set_filter_params(
            f32::from_bits(filter_min_cutoff.load(Ordering::Relaxed)),
            f32::from_bits(filter_beta.load(Ordering::Relaxed)),
            median_window.load(Ordering::Relaxed),
        );
        let byp = bypass.load(Ordering::Relaxed);
        if cap.poll_once(byp, true) {
            captured += 1;
            frame_id += 1;
            latest.store(Some(Arc::new(cap.snapshot_frame(frame_id))));
        } else {
            // No new camera frame — yield briefly so we don't busy-spin.
            std::thread::sleep(Duration::from_millis(1));
        }
        // Outside the branch on purpose: a stall is exactly when these numbers
        // matter, and publishing them only alongside a frame made a stalled
        // pipeline and an unread one look identical. See [`CaptureVitals`].
        vitals.store_counts(captured, &cap);
        if last_sensor_stats.elapsed() >= SENSOR_STATS_EVERY {
            vitals.store_sensor(&cap);
            last_sensor_stats = Instant::now();
        }
    }
}

// ---------------------------------------------------------------- Head worker

/// Latest RGB frame handed to the [`HeadWorker`]. Only the most recent one
/// matters — the worker overwrites any still-unprocessed job. The buffer is
/// shared (`Arc`), not copied: the capture thread already keeps the same
/// frame alive for display, so a submit is a pointer bump instead of a
/// 6 MB memcpy per frame.
struct HeadJob {
    /// The frame exactly as the driver produced it — see `layout`. Not
    /// repacked into RGB888 on the way here: both models sample a small
    /// patch, and the repack cost more than everything they do with it.
    pixels: Arc<Vec<u8>>,
    w: u32,
    h: u32,
    layout: FrameLayout,
    /// Infrared or colour. The detector carries one model per stream, so the
    /// submitter has to say which image this is.
    stream: anchor::Stream,
}

/// Byte layout of a published frame. Mirrors `blazepose::PixelLayout` and
/// `anchor::PixelLayout`, which are each crate's own input contract; this is
/// the demo's single value that maps to both.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
enum FrameLayout {
    /// 3 bytes per pixel, `[R, G, B]`.
    #[default]
    Rgb888,
    /// 4 bytes per pixel, `[B, G, R, X]` — the Kinect v2 colour frame,
    /// forwarded untouched.
    Bgrx8888,
}

impl FrameLayout {
    const fn pose(self) -> blazepose::PixelLayout {
        match self {
            Self::Rgb888 => blazepose::PixelLayout::Rgb888,
            Self::Bgrx8888 => blazepose::PixelLayout::Bgrx8888,
        }
    }

    const fn anchor(self) -> anchor::PixelLayout {
        match self {
            Self::Rgb888 => anchor::PixelLayout::Rgb888,
            Self::Bgrx8888 => anchor::PixelLayout::Bgrx8888,
        }
    }

    /// Bytes per pixel.
    const fn bpp(self) -> usize {
        match self {
            Self::Rgb888 => 3,
            Self::Bgrx8888 => 4,
        }
    }

    /// Offsets of R, G and B inside one pixel.
    const fn channels(self) -> [usize; 3] {
        match self {
            Self::Rgb888 => [0, 1, 2],
            Self::Bgrx8888 => [2, 1, 0],
        }
    }
}

// -------------------------------------------------------------- Pose worker

/// What the [`BlazePoseWorker`] publishes after each inference.
#[derive(Clone, Default)]
struct PoseOut {
    pose: Option<blazepose::Pose>,
    /// Last inference time (ms); `0.0` until the first pose.
    ms: f32,
}

/// Runs BlazePose (detector + 33 landmarks, ~12 ms) off the UI thread — same
/// submit/snapshot pattern as [`HeadWorker`]. Loads its ONNX models on the
/// thread so the UI never blocks. Replaces the RGB head net + depth blob +
/// silhouette skeleton with one pose model that reads straight off the frame.
struct BlazePoseWorker {
    job: Arc<(Mutex<Option<HeadJob>>, Condvar)>,
    out: Arc<Mutex<PoseOut>>,
    stop: Arc<AtomicBool>,
    interval_ms: Arc<AtomicU32>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl BlazePoseWorker {
    fn spawn() -> Self {
        let job = Arc::new((Mutex::new(None::<HeadJob>), Condvar::new()));
        let out = Arc::new(Mutex::new(PoseOut::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let interval_ms = Arc::new(AtomicU32::new(0));
        let (job_t, out_t, stop_t, iv_t) = (
            Arc::clone(&job),
            Arc::clone(&out),
            Arc::clone(&stop),
            Arc::clone(&interval_ms),
        );
        let handle = std::thread::Builder::new()
            .name("blazepose".into())
            .spawn(move || blazepose_worker_loop(&job_t, &out_t, &stop_t, &iv_t))
            .expect("spawn blazepose thread");
        Self {
            job,
            out,
            stop,
            interval_ms,
            handle: Some(handle),
        }
    }

    fn set_min_interval_ms(&self, ms: u32) {
        self.interval_ms.store(ms, Ordering::Relaxed);
    }

    fn submit(&self, pixels: Arc<Vec<u8>>, w: u32, h: u32, layout: FrameLayout) {
        *self.job.0.lock() = Some(HeadJob {
            pixels,
            w,
            h,
            layout,
            // Meaningless here: the pose model is the same one whatever the
            // image is. The field exists for the anchor worker, which shares
            // this job type and does carry one model per stream.
            stream: anchor::Stream::Colour,
        });
        self.job.1.notify_one();
    }

    fn snapshot(&self) -> PoseOut {
        self.out.lock().clone()
    }
}

impl Drop for BlazePoseWorker {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        self.job.1.notify_one();
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

fn blazepose_worker_loop(
    job: &Arc<(Mutex<Option<HeadJob>>, Condvar)>,
    out: &Arc<Mutex<PoseOut>>,
    stop: &Arc<AtomicBool>,
    interval_ms: &Arc<AtomicU32>,
) {
    let mut bp = match blazepose::BlazePose::new() {
        Ok(b) => b,
        Err(e) => {
            warn!("blazepose init failed: {e}");
            return;
        }
    };
    let mut last_run = Instant::now();
    loop {
        let interval = Duration::from_millis(u64::from(interval_ms.load(Ordering::Relaxed)));
        while last_run.elapsed() < interval {
            if stop.load(Ordering::Acquire) {
                return;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        let job_item = {
            let (lock, cvar) = &**job;
            let mut slot = lock.lock();
            while slot.is_none() && !stop.load(Ordering::Acquire) {
                cvar.wait(&mut slot);
            }
            if stop.load(Ordering::Acquire) {
                return;
            }
            slot.take()
        };
        let Some(HeadJob {
            pixels,
            w,
            h,
            layout,
            stream: _,
        }) = job_item
        else {
            continue;
        };
        last_run = Instant::now();
        let t0 = Instant::now();
        // `poll` = MediaPipe detect-once-then-track: skips the detector while a
        // subject is tracked, so a still skeleton no longer trembles.
        match bp.poll(&pixels, w, h, layout.pose()) {
            Ok(pose) => {
                let ms = t0.elapsed().as_secs_f32() * 1000.0;
                let mut o = out.lock();
                o.pose = pose;
                o.ms = ms;
            }
            Err(e) => warn!("blazepose poll: {e}"),
        }
    }
}

/// What the [`AnchorWorker`] publishes after each inference.
#[derive(Clone, Default)]
struct AnchorOut {
    geom: Option<anchor::AnchorGeometry>,
    /// Last inference time (ms); `0.0` until the first detection.
    ms: f32,
}

/// Runs the cabinet **anchor** model (YOLO-pose, 6 keypoints) off the UI thread,
/// on RGB — same submit/snapshot pattern as [`BlazePoseWorker`]. Throttled: the
/// cabinet is fixed, so a low rate is plenty and keeps the UI smooth.
struct AnchorWorker {
    job: Arc<(Mutex<Option<HeadJob>>, Condvar)>,
    out: Arc<Mutex<AnchorOut>>,
    stop: Arc<AtomicBool>,
    /// Set once the best-of-warmup detection is frozen (the cabinet is fixed).
    locked: Arc<AtomicBool>,
    /// Raised by [`Self::recalibrate`] to thaw that freeze and run a fresh
    /// warmup. The cabinet is fixed, the camera on top of it is not.
    reset: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl AnchorWorker {
    fn spawn() -> Self {
        let job = Arc::new((Mutex::new(None::<HeadJob>), Condvar::new()));
        let out = Arc::new(Mutex::new(AnchorOut::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let locked = Arc::new(AtomicBool::new(false));
        let reset = Arc::new(AtomicBool::new(false));
        let (job_t, out_t, stop_t, locked_t, reset_t) = (
            Arc::clone(&job),
            Arc::clone(&out),
            Arc::clone(&stop),
            Arc::clone(&locked),
            Arc::clone(&reset),
        );
        let handle = std::thread::Builder::new()
            .name("anchor".into())
            .spawn(move || anchor_worker_loop(&job_t, &out_t, &stop_t, &locked_t, &reset_t))
            .expect("spawn anchor thread");
        Self {
            job,
            out,
            stop,
            locked,
            reset,
            handle: Some(handle),
        }
    }

    /// True once the warmup froze the best detection (the caller stops submitting).
    fn is_locked(&self) -> bool {
        self.locked.load(Ordering::Acquire)
    }

    /// Run the detection again from scratch: thaw the freeze, forget the best
    /// score and the published geometry, restart the warmup window. For when
    /// the camera has been moved or re-aimed after the lock — until this
    /// existed, the only way out was to switch backend and back.
    fn recalibrate(&self) {
        self.reset.store(true, Ordering::Release);
        // Wake the job wait too: a reset raised while the worker is still in
        // warmup is picked up at the top of the next iteration.
        self.job.1.notify_one();
    }

    fn submit(
        &self,
        pixels: Arc<Vec<u8>>,
        w: u32,
        h: u32,
        layout: FrameLayout,
        stream: anchor::Stream,
    ) {
        *self.job.0.lock() = Some(HeadJob {
            pixels,
            w,
            h,
            layout,
            stream,
        });
        self.job.1.notify_one();
    }

    fn snapshot(&self) -> AnchorOut {
        self.out.lock().clone()
    }
}

impl Drop for AnchorWorker {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        self.job.1.notify_one();
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

fn anchor_worker_loop(
    job: &Arc<(Mutex<Option<HeadJob>>, Condvar)>,
    out: &Arc<Mutex<AnchorOut>>,
    stop: &Arc<AtomicBool>,
    locked: &Arc<AtomicBool>,
    reset: &Arc<AtomicBool>,
) {
    // One detector per stream, built on first use. The demo can switch source
    // while running, and loading both up front would cost a second ONNX session
    // for a model that may never be asked for.
    let mut dets: [Option<anchor::AnchorDetector>; 2] = [None, None];
    // Throttle inference; the cabinet is fixed so a low rate is plenty.
    const INTERVAL: Duration = Duration::from_millis(400);
    // Keep the best-scoring detection for this long after the first hit, then
    // freeze it — the camera + cabinet don't move.
    const WARMUP: Duration = Duration::from_millis(2500);
    let mut last_run = Instant::now();
    let mut warmup_start: Option<Instant> = None;
    let mut best_score = 0.0f32;
    // Forget everything the last warmup concluded. Keeping the old best score
    // would make a recalibration pointless: the frozen detection is exactly
    // what the user is telling us is wrong now.
    let restart = |warmup_start: &mut Option<Instant>, best_score: &mut f32| {
        *warmup_start = None;
        *best_score = 0.0;
        *out.lock() = AnchorOut::default();
        locked.store(false, Ordering::Release);
        info!("anchor: recalibrating");
    };
    loop {
        if reset.swap(false, Ordering::AcqRel) {
            restart(&mut warmup_start, &mut best_score);
        }
        while last_run.elapsed() < INTERVAL {
            if stop.load(Ordering::Acquire) {
                return;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        let job_item = {
            let (lock, cvar) = &**job;
            let mut slot = lock.lock();
            while slot.is_none() && !stop.load(Ordering::Acquire) {
                cvar.wait(&mut slot);
            }
            if stop.load(Ordering::Acquire) {
                return;
            }
            slot.take()
        };
        let Some(HeadJob {
            pixels,
            w,
            h,
            layout,
            stream,
        }) = job_item
        else {
            continue;
        };
        last_run = Instant::now();
        // Load the model for this stream the first time it is asked for. The
        // demo can switch source while running; loading both up front would
        // cost a second ONNX session for a model that may never be used.
        let idx = usize::from(stream == anchor::Stream::Colour);
        if dets[idx].is_none() {
            match anchor::AnchorDetector::new(stream) {
                Ok(d) => dets[idx] = Some(d),
                Err(e) => {
                    warn!(?stream, "anchor init failed: {e}");
                    continue;
                }
            }
        }
        let Some(det) = dets[idx].as_mut() else {
            continue;
        };
        // Start the warmup clock on the first INFERENCE RUN, not the first
        // detection. The 1280² model on CPU costs ~180 ms; the proof model
        // detects only sporadically on a real scene, so gating the clock (and
        // the lock) on `Some` meant the worker could re-run forever — pinning
        // the CPU and dragging the camera down. The cabinet is fixed, so we run
        // for a bounded warmup then FREEZE regardless.
        let start = *warmup_start.get_or_insert_with(Instant::now);
        let t0 = Instant::now();
        let detn = det.detect(&pixels, w, h, layout.anchor());
        let ms = t0.elapsed().as_secs_f32() * 1000.0;
        match detn {
            Some(d) => {
                if d.score >= best_score {
                    best_score = d.score;
                    let mut o = out.lock();
                    o.geom = Some(d.geometry(w, h));
                    o.ms = ms;
                }
            }
            None => out.lock().ms = ms,
        }
        // Freeze after the warmup — Some or None — and stop running inference
        // for good (the caller stops submitting once `is_locked`).
        if start.elapsed() >= WARMUP {
            locked.store(true, Ordering::Release);
            // Park instead of exiting: ↻ Recalibrate has to be able to wake
            // this thread. The caller stops submitting while locked, so the
            // job wait below would never return on its own.
            while !stop.load(Ordering::Acquire) {
                if reset.swap(false, Ordering::AcqRel) {
                    restart(&mut warmup_start, &mut best_score);
                    break;
                }
                std::thread::sleep(Duration::from_millis(50));
            }
            if stop.load(Ordering::Acquire) {
                return;
            }
            // `last_run` is stale by now, so the next detection runs at once
            // rather than after another throttle interval.
            continue;
        }
    }
}

/// Build a [`LockbarQuadRgb`] from the anchor geometry so the existing autocalib
/// + capture-overlay paths consume the anchor detection unchanged.
fn anchor_to_quad(
    geom: &anchor::AnchorGeometry,
    w: u32,
    h: u32,
) -> headtracking::calibration::LockbarQuadRgb {
    let px = |p: (f32, f32)| (p.0.max(0.0) as u32, p.1.max(0.0) as u32);
    let c = geom.corners; // [player_L, player_R, screen_R, screen_L]
    headtracking::calibration::LockbarQuadRgb {
        frame_width: w,
        frame_height: h,
        corners: [px(c[0]), px(c[1]), px(c[2]), px(c[3])],
        slope_deg: 0.0,
        thickness_px: (c[0].1 - c[3].1).abs() as u32,
        n_inliers_top: 100,
        n_inliers_bottom: 100,
        left_rail: Some([px(geom.left_sidebar.0), px(geom.left_sidebar.1)]),
        right_rail: Some([px(geom.right_sidebar.0), px(geom.right_sidebar.1)]),
    }
}

#[derive(Debug, Clone, Copy)]
struct Vec3Mm {
    x: f32,
    y: f32,
    z: f32,
}

/// Recover the lockbar's 3D centre in the camera frame from its
/// detected pixel quad. Uses the pinhole inverse on the lockbar's
/// known physical width (`LOCKBAR_WIDTH_MM`, 61 cm widebody default).
/// Returns `None` when the quad is degenerate.
///
/// Webcam intrinsics are zero at construction time (no per-camera
/// calibration yet), so we fall back to the shared nominal focal
/// [`WEBCAM_FX_PER_WIDTH`], keyed off the frame dimensions stored in the
/// quad itself.
fn lockbar_3d_center(
    quad: &headtracking::calibration::LockbarQuadRgb,
    intr: &Intrinsics,
) -> Option<Vec3Mm> {
    let mean_w_px = quad.mean_width_px();
    if mean_w_px < 4 || quad.frame_width == 0 || quad.frame_height == 0 {
        return None;
    }
    let fx = if intr.fx > 0.0 {
        intr.fx
    } else {
        WEBCAM_FX_PER_WIDTH * quad.frame_width as f32
    };
    let fy = if intr.fy > 0.0 { intr.fy } else { fx };
    let cx = if intr.cx > 0.0 {
        intr.cx
    } else {
        quad.frame_width as f32 * 0.5
    };
    let cy = if intr.cy > 0.0 {
        intr.cy
    } else {
        quad.frame_height as f32 * 0.5
    };
    let z = headtracking::calibration::LOCKBAR_WIDTH_MM * fx / (mean_w_px as f32);
    let u_center = (quad.corners[0].0 + quad.corners[1].0) as f32 * 0.5;
    let v_center = quad.mean_row() as f32;
    let x = (u_center - cx) * z / fx;
    let y = (v_center - cy) * z / fy;
    Some(Vec3Mm { x, y, z })
}

mod filter_alias {
    // The plugin's filter module isn't exposed as a sibling crate, but it's
    // a standalone in-tree module — duplicate it here would mean another
    // copy of identical code. Instead, headtracking-demo pulls it directly via the
    // workspace's `headtracking` crate path.
    pub use headtracking::filter::{OneEuroParams, OneEuroPose3D};
}

/// Build the 3-axis 1€ filter for the head pose. X/Y get the library
/// defaults; Z gets a tighter `min_cutoff` because depth-camera readings
/// are inherently noisier (the median over a small pixel window
/// fluctuates as the head bbox shifts a pixel or two between frames).
fn make_pose_filter() -> filter_alias::OneEuroPose3D {
    let xy = filter_alias::OneEuroParams::default();
    let z = filter_alias::OneEuroParams {
        min_cutoff_hz: 0.4,
        beta: 0.05,
        derivative_cutoff_hz: 1.0,
    };
    filter_alias::OneEuroPose3D::new_per_axis([xy, xy, z])
}

/// State for the Kinect v1 tilt + LED panel. The desired values are kept
/// here so the user can drag the slider freely; we only push commands to
/// the device on `drag_stopped` / combo change to avoid hammering the
/// fragile motor gears.
struct V1Controls {
    desired_tilt_deg: f32,
    last_sent_tilt_deg: f32,
    selected_led: freenect::LedState,
    last_sent_led: freenect::LedState,
    last_state: Option<freenect::TiltState>,
    /// Set once the slider has been seeded to the device's real tilt from the
    /// first shared read-out, so it doesn't snap from 0 on first use.
    seeded: bool,
}

impl V1Controls {
    fn new() -> Self {
        Self {
            desired_tilt_deg: 0.0,
            last_sent_tilt_deg: 0.0,
            selected_led: freenect::LedState::Green,
            last_sent_led: freenect::LedState::Green,
            last_state: None,
            seeded: false,
        }
    }
}

#[derive(Clone, Copy)]
struct Intrinsics {
    fx: f32,
    fy: f32,
    cx: f32,
    cy: f32,
}

enum Inner {
    KinectV2 {
        device: freenect2::Device,
        _ctx: freenect2::Context,
    },
    KinectV1 {
        device: freenect::Device,
        _ctx: freenect::Context,
    },
    Webcam {
        camera: webcam::Camera,
    },
}

impl Capture {
    /// Non-blocking check for whether the RGB stream has produced a frame yet.
    /// Consumes that frame (the poll loop will get the next one) — used only to
    /// confirm the stream is alive right after opening.
    fn poll_first_rgb(&mut self) -> bool {
        match &mut self.inner {
            Inner::KinectV1 { device, .. } => device.poll_rgb().is_some(),
            Inner::KinectV2 { device, .. } => device.poll_rgb_into(&mut self.v2_rgb),
            Inner::Webcam { camera } => camera.poll_rgb().is_some(),
        }
    }

    /// Bounce the RGB stream to recover from a stalled open: stop+start for the
    /// Kinects, reopen the SDL device for the webcam. Returns the failure
    /// reason if the restart itself errors.
    fn bounce_stream(&mut self) -> Result<(), String> {
        match &mut self.inner {
            Inner::KinectV1 { device, .. } => {
                device.stop().map_err(|e| format!("v1 stop: {e}"))?;
                device
                    .start_streams(true, true)
                    .map_err(|e| format!("v1 start: {e}"))
            }
            Inner::KinectV2 { device, .. } => {
                device.stop().map_err(|e| format!("v2 stop: {e}"))?;
                device
                    .start_streams(true, true)
                    .map_err(|e| format!("v2 start: {e}"))
            }
            // Close-then-open, inside the crate: SDL will not open a device
            // that is still open, and this call site could not close first.
            Inner::Webcam { camera } => camera.reopen().map_err(|e| format!("webcam reopen: {e}")),
        }
    }
}

/// Head pixel from a BlazePose nose + a depth frame: map the nose (RGB px)
/// into the depth grid, take the median valid depth in a small window, and
/// deproject through the IR intrinsics. Mirrors [`head_pixel_from_depth`] but
/// keyed on the pose's nose instead of a head bbox — the depth path once
/// BlazePose replaces the head net.
/// POV "eye" position in RGB pixels — the **glabella / forehead** (between the
/// eyebrows), a better viewpoint than the nose. `eye_mid` is the mean of the 6
/// eye landmarks (indices 1..=6); we push up from the eye line, away from the
/// nose, toward the brow.
fn head_center_xy(pose: &blazepose::Pose) -> (f32, f32) {
    let nose = &pose.landmarks[0];
    let (mut ex, mut ey) = (0.0f32, 0.0f32);
    for l in &pose.landmarks[1..=6] {
        ex += l.x;
        ey += l.y;
    }
    let (ex, ey) = (ex / 6.0, ey / 6.0);
    (ex + (ex - nose.x) * 0.4, ey + (ey - nose.y) * 0.4)
}

/// Colour-space width/height of libfreenect2's `bigdepth` map, and the one-row
/// top border it carries (`filter_height_half = 1`), so colour row `y` lives at
/// bigdepth row `y + 1`.
const BIGDEPTH_W: usize = 1920;
const BIGDEPTH_H: usize = 1080;
const BIGDEPTH_ROW_OFFSET: usize = 1;

/// Compare the windowed colour-space projection against the same patch of the
/// full `bigdepth` plane and log the verdict once.
///
/// `Registration::depth_window` reimplements libfreenect2's own splat loop
/// over a 17×17 window instead of 1920×1082; the values are supposed to match
/// exactly. Nothing but a live Kinect can check that, so the check lives here
/// and runs on the first frame where both projections happen to be valid.
fn report_window_match(window: &[f32], bigdepth: &[f32], center: (i32, i32)) {
    let (cx, cy) = center;
    let mut compared = 0u32;
    let mut mismatched = 0u32;
    let mut worst = 0.0f32;
    for dv in -HEAD_WINDOW_HALF..=HEAD_WINDOW_HALF {
        let v = cy + dv;
        if v < 0 || v >= BIGDEPTH_H as i32 {
            continue;
        }
        let row = (v as usize + BIGDEPTH_ROW_OFFSET) * BIGDEPTH_W;
        for du in -HEAD_WINDOW_HALF..=HEAD_WINDOW_HALF {
            let u = cx + du;
            if u < 0 || u >= BIGDEPTH_W as i32 {
                continue;
            }
            let full = bigdepth[row + u as usize];
            let win = window[((dv + HEAD_WINDOW_HALF) * (2 * HEAD_WINDOW_HALF + 1)
                + (du + HEAD_WINDOW_HALF)) as usize];
            compared += 1;
            if full.is_infinite() && win.is_infinite() {
                continue;
            }
            let d = (full - win).abs();
            if d > worst {
                worst = d;
            }
            // Both come from the same `min` over the same samples, so equality
            // is exact; anything else is a real divergence, not rounding.
            if d != 0.0 {
                mismatched += 1;
            }
        }
    }
    if mismatched == 0 {
        info!(
            compared,
            "depth window: matches the full colour-space projection exactly"
        );
    } else {
        warn!(
            compared,
            mismatched,
            worst_mm = worst,
            "depth window: diverges from the full colour-space projection"
        );
    }
}

/// Half-width of the colour-space depth window sampled around the head, and
/// the resulting square side. 17×17 colour pixels: wide enough for a stable
/// median at cabinet distance, small enough that projecting depth into it
/// costs a fraction of a millisecond.
const HEAD_WINDOW_HALF: i32 = 8;
const HEAD_WINDOW_SIDE: usize = (2 * HEAD_WINDOW_HALF + 1) as usize;

/// Head pixel from a BlazePose landmark sampled in **colour space**, from the
/// depth window `Registration::depth_window` filled around it.
///
/// This is the accurate path for the Kinect v2: the landmark is already in
/// colour pixels and the window holds depth expressed in those same pixels,
/// so no cross-sensor mapping is needed at all. Deprojection therefore uses
/// the **colour** intrinsics — passing the IR ones here would reintroduce the
/// very error the registration removes.
///
/// Unmapped pixels come back `+inf` from libfreenect2 (not `0`), so the
/// validity gate checks `is_finite()` on top of the `> 0` no-reading test.
fn head_pixel_from_window(
    pose: &blazepose::Pose,
    window: &[f32],
    color: &Intrinsics,
    min_samples: usize,
) -> Option<HeadPixel> {
    if window.len() != HEAD_WINDOW_SIDE * HEAD_WINDOW_SIDE || color.fx <= 0.0 {
        return None;
    }
    let (hx, hy) = head_center_xy(pose);
    let mut samples: Vec<f32> = window
        .iter()
        .copied()
        .filter(|z| z.is_finite() && *z > 0.0)
        .collect();
    if samples.len() < min_samples.max(1) {
        return None;
    }
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let depth_mm = samples[samples.len() / 2];
    let zf = f64::from(depth_mm);
    Some(HeadPixel {
        u: hx.max(0.0) as u32,
        v: hy.max(0.0) as u32,
        depth_mm,
        x_mm: (f64::from(hx - color.cx) * zf / f64::from(color.fx)) as f32,
        y_mm: (f64::from(hy - color.cy) * zf / f64::from(color.fy)) as f32,
    })
}

/// Generic over the depth sample type so the v1's native `u16` grid is
/// sampled in place — the 17×17 window widens per-sample instead of paying a
/// full-frame `u16→f32` copy (1.2 MB at 30 Hz) up front. `f32: From<T>`
/// covers both `u16` (v1) and `f32` (v2) losslessly.
fn head_pixel_from_pose_depth<T: Copy>(
    pose: &blazepose::Pose,
    rgb: (u32, u32),
    depth_data: &[T],
    depth_dims: (u32, u32),
    intr: &Intrinsics,
    min_samples: usize,
) -> Option<HeadPixel>
where
    f32: From<T>,
{
    let (rgb_w, rgb_h) = rgb;
    let (depth_w, depth_h) = depth_dims;
    if rgb_w == 0 || rgb_h == 0 || depth_w == 0 || depth_h == 0 {
        return None;
    }
    let (hx, hy) = head_center_xy(pose);
    let depth_cx = hx * depth_w as f32 / rgb_w as f32;
    let depth_cy = hy * depth_h as f32 / rgb_h as f32;
    let (cx, cy) = (depth_cx as i32, depth_cy as i32);
    let half = 8i32;
    let mut samples: Vec<f32> = Vec::new();
    for dv in -half..=half {
        let v = cy + dv;
        if v < 0 || v >= depth_h as i32 {
            continue;
        }
        let row = v as usize * depth_w as usize;
        for du in -half..=half {
            let u = cx + du;
            if u < 0 || u >= depth_w as i32 {
                continue;
            }
            let z = f32::from(depth_data[row + u as usize]);
            if z.is_finite() && z > 0.0 {
                samples.push(z);
            }
        }
    }
    if samples.len() < min_samples.max(1) {
        return None;
    }
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let depth_mm = samples[samples.len() / 2];
    let zf = f64::from(depth_mm);
    Some(HeadPixel {
        u: depth_cx.max(0.0) as u32,
        v: depth_cy.max(0.0) as u32,
        depth_mm,
        x_mm: (f64::from(depth_cx - intr.cx) * zf / f64::from(intr.fx)) as f32,
        y_mm: (f64::from(depth_cy - intr.cy) * zf / f64::from(intr.fy)) as f32,
    })
}

/// Head pixel from a BlazePose pose on a **webcam** frame (no depth):
/// triangulate distance from the shoulder width — a stable ~0.40 m span, so
/// `Z = fx · W / w_px` — then deproject the nose. `None` if the shoulders
/// aren't confidently seen.
fn head_pixel_from_pose_webcam(
    pose: &blazepose::Pose,
    rgb_w: u32,
    rgb_h: u32,
) -> Option<HeadPixel> {
    use blazepose::idx::{LEFT_SHOULDER, RIGHT_SHOULDER};
    if rgb_w == 0 || rgb_h == 0 {
        return None;
    }
    let (ls, rs) = (
        &pose.landmarks[LEFT_SHOULDER],
        &pose.landmarks[RIGHT_SHOULDER],
    );
    let (hx, hy) = head_center_xy(pose);
    if ls.visibility < 0.5 || rs.visibility < 0.5 {
        return None;
    }
    let w_px = ((ls.x - rs.x).powi(2) + (ls.y - rs.y).powi(2)).sqrt();
    if w_px < 1.0 {
        return None;
    }
    // Webcams report no intrinsics (fx = 0), so assume a nominal focal from
    // the frame width (~55° horizontal FOV, typical for a webcam). Distance
    // then triangulates from the shoulder width (~0.40 m), and the nose
    // deprojects with the same nominal pinhole.
    let fx = rgb_w as f32 * WEBCAM_FX_PER_WIDTH;
    let cx = rgb_w as f32 * 0.5;
    let cy = rgb_h as f32 * 0.5;
    const SHOULDER_W_MM: f32 = 400.0;
    let depth_mm = fx * SHOULDER_W_MM / w_px;
    let zf = f64::from(depth_mm);
    Some(HeadPixel {
        u: hx.max(0.0) as u32,
        v: hy.max(0.0) as u32,
        depth_mm,
        x_mm: (f64::from(hx - cx) * zf / f64::from(fx)) as f32,
        y_mm: (f64::from(hy - cy) * zf / f64::from(fx)) as f32,
    })
}

#[derive(Debug, Clone, Copy)]
struct HeadPixel {
    /// Pixel coords inside the source frame (depth grid for Kinect, RGB
    /// frame for webcam). Surface only — used by the status label, no
    /// longer drives any overlay.
    u: u32,
    v: u32,
    depth_mm: f32,
    x_mm: f32,
    y_mm: f32,
}

#[derive(Debug, Clone, Copy)]
struct Baseline {
    x_mm: f32,
    y_mm: f32,
    z_mm: f32,
}

/// Read this process's accumulated CPU time (user + system) in clock ticks
/// Live performance counters, shown in the toolbar and logged every ~2 s:
/// per-model inference time (EWMA), process CPU%, and the input vs output
/// frame rates. Both count the **same unit** so they're comparable: `in` =
/// camera frames driving the pipeline (on the capture thread), `out` = those
/// same frames uploaded to the display (on the GL thread). Capture and
/// render now run on separate threads, so `out < in` genuinely happens when
/// rendering can't keep up — `in` advances by the cumulative-capture delta so
/// frames the GL thread skipped are still counted (see [`App::poll`]).
///
/// "Camera frames" rather than "RGB frames" because a Kinect v1 switched to its
/// IR stream has no colour frame at all — the IR frame is what feeds the models
/// there, so `in` tracks it and reads equal to `ir`.
struct Metrics {
    head_ms: f32,
    anchor_ms: f32,
    /// Kinect v2 depth↔colour registration cost (ms, EWMA); `0.0` when it
    /// isn't running. Surfaced because it's real per-frame work on the capture
    /// thread — if it ever gets expensive we want to see it, not hide it.
    reg_ms: f32,
    /// Surface→RGB decode cost (ms, EWMA); webcam only. A 1080p MJPG frame is
    /// decoded whole here before the pose model takes a 224x224 square of it.
    convert_ms: f32,
    /// Cost of getting one frame ready for the model (ms, EWMA); Kinect v2
    /// only. An 8.3 MB colour frame has to leave the driver's slot before the
    /// next delivery overwrites it, and that copy is the largest per-frame
    /// cost left on a modest CPU — it used to be the one thing the budget
    /// line did not count, on top of a repack into packed RGB that no longer
    /// happens at all.
    copy_ms: f32,
    /// Per-stage smoothing cost, smoothed like the other per-frame costs.
    median_us: f32,
    euro_us: f32,
    /// Locked anchors stop the detector, so a duration would be misleading.
    anchor_locked: bool,
    /// Previous budget verdict, to log the moment it changes.
    was_over: bool,
    /// One frame's worth of time at the sensor's nominal rate: the ceiling the
    /// per-frame costs must fit under to hold that rate. Both Kinects cap at
    /// 30 fps and most UVC webcams report 30, so that is the default.
    budget_ms: f32,
    in_fps: f32,
    out_fps: f32,
    /// Display repaints per second — the GL thread's own cadence, independent
    /// of the camera. Decoupled from capture by the thread split, so it sits
    /// near the ~60 Hz repaint cap even when `in`/`out` are camera-bound (e.g.
    /// 20 fps webcam). Makes the capture/render decoupling visible.
    render_fps: f32,
    /// What the sensor delivered over the window and the share of it we let
    /// die unread, per stream. `None` on backends whose driver cannot report
    /// it — there the capture rate already *is* the sensor rate.
    sensor_in_fps: Option<f32>,
    sensor_ir_fps: Option<f32>,
    in_drop_pct: Option<f32>,
    ir_drop_pct: Option<f32>,
    /// Latest cumulative counters, and the values at the start of the current
    /// window: the rates above are their difference over `elapsed`.
    sensor_rgb: SensorCounts,
    sensor_depth: SensorCounts,
    sensor_rgb_base: SensorCounts,
    sensor_depth_base: SensorCounts,
    /// Whether the backend reports delivered/dropped at all (libfreenect2 does,
    /// nothing else does).
    reports_sensor: bool,
    /// The colour camera's auto-exposure read-out and the sensor's own frame
    /// step, when the backend can give them.
    exposure: Option<(f32, f32, u32)>,
    /// Depth and IR capture rates, reported separately.
    ///
    /// They used to share one number printed as `ir+depth`, which was wrong
    /// twice over: the value was the IR stream alone, never a sum, and the
    /// label invited exactly the question it should have answered — whether
    /// the IR camera follows the colour one down in a dim room.
    ///
    /// It does not, and libfreenect2's own API says why. Exposure control
    /// exists for the colour camera only (`setColorAutoExposure`,
    /// `setColorSemiAutoExposure`, `setColorManualExposure`) with nothing
    /// equivalent for IR or depth, whose hardware settings the firmware
    /// handles by itself. `Frame::exposure`/`gain`/`gamma` are likewise
    /// written solely by the RGB packet processors, from the RGB packet
    /// footer — an IR or depth frame carries none of them.
    ///
    /// The colour halving has a mechanism, not just a habit: manual
    /// integration time runs to 66 ms (`setColorManualExposure`, range
    /// `(0.0, 66.0]`), which is one whole frame period at 15 Hz. Once
    /// auto-exposure needs more than ~33 ms of light, 30 Hz is arithmetically
    /// out of reach. The depth/IR side integrates under its own active
    /// illuminator and holds ~30 Hz across every session we have collected.
    ///
    /// So the ratio is the colour half falling, not the IR half rising. Two
    /// names, two numbers, and a divergence between them becomes visible
    /// instead of hidden.
    ///
    /// Diagnostic only — neither is a display rate.
    /// The Kinect streams them from its **own IR illuminator**, so they hold
    /// ~30 Hz in the dark while the auto-exposed colour stream halves to 15:
    /// `ir` staying at 30 while `in` sits at 15 is the proof the ceiling is the
    /// colour camera, not USB bandwidth or our pipeline.
    depth_fps: f32,
    ir_fps: f32,
    /// Process CPU across all cores (100% = one core saturated) and resident
    /// memory. Read through `sysinfo` so the numbers exist on Windows and
    /// macOS too: the previous reader parsed `/proc/self/stat` and quietly
    /// reported 0% everywhere else, which is exactly how the Kinect v2's CPU
    /// depth pipeline saturating on Windows went unseen for so long.
    cpu_pct: f32,
    ram_mib: u64,
    in_frames: u32,
    out_frames: u32,
    render_frames: u32,
    depth_frames: u32,
    ir_frames: u32,
    window_start: Instant,
    last_log: Instant,
    sys: sysinfo::System,
    pid: sysinfo::Pid,
}

impl Metrics {
    fn new() -> Self {
        let now = Instant::now();
        Self {
            head_ms: 0.0,
            anchor_ms: 0.0,
            reg_ms: 0.0,
            median_us: 0.0,
            euro_us: 0.0,
            convert_ms: 0.0,
            copy_ms: 0.0,
            sensor_in_fps: None,
            sensor_ir_fps: None,
            in_drop_pct: None,
            ir_drop_pct: None,
            sensor_rgb: SensorCounts::default(),
            sensor_depth: SensorCounts::default(),
            sensor_rgb_base: SensorCounts::default(),
            sensor_depth_base: SensorCounts::default(),
            reports_sensor: false,
            exposure: None,
            anchor_locked: false,
            was_over: false,
            budget_ms: 1000.0 / 30.0,
            in_fps: 0.0,
            out_fps: 0.0,
            render_fps: 0.0,
            depth_fps: 0.0,
            ir_fps: 0.0,
            cpu_pct: 0.0,
            ram_mib: 0,
            in_frames: 0,
            out_frames: 0,
            render_frames: 0,
            depth_frames: 0,
            ir_frames: 0,
            window_start: now,
            last_log: now,
            sys: sysinfo::System::new(),
            pid: sysinfo::get_current_pid().unwrap_or_else(|_| sysinfo::Pid::from(0)),
        }
    }

    // EWMAs so a single slow frame doesn't dominate the reading.
    fn note_head_ms(&mut self, ms: f32) {
        self.head_ms = if self.head_ms == 0.0 {
            ms
        } else {
            self.head_ms * 0.8 + ms * 0.2
        };
    }
    fn note_anchor_ms(&mut self, ms: f32) {
        self.anchor_ms = if self.anchor_ms == 0.0 {
            ms
        } else {
            self.anchor_ms * 0.8 + ms * 0.2
        };
    }
    /// Registration cost for the latest frame. Reported as an EWMA like the
    /// inference times; an exact `0.0` means "not running" and is passed
    /// through so the figure drops out cleanly on v1 / webcam.
    /// Same exponential smoothing as the other per-frame costs, so one slow
    /// decode does not dominate the read-out.
    /// Same smoothing as the other per-frame costs. Instrumented to check the
    /// claim that it is negligible, rather than assume it.
    fn note_filter_us(&mut self, c: FilterUs) {
        let ewma = |cur: f32, v: f32| if cur == 0.0 { v } else { cur * 0.8 + v * 0.2 };
        self.median_us = ewma(self.median_us, c.median);
        self.euro_us = ewma(self.euro_us, c.euro);
    }
    fn note_convert_ms(&mut self, ms: f32) {
        self.convert_ms = if ms == 0.0 || self.convert_ms == 0.0 {
            ms
        } else {
            self.convert_ms * 0.8 + ms * 0.2
        };
    }

    fn note_copy_ms(&mut self, ms: f32) {
        self.copy_ms = if ms == 0.0 || self.copy_ms == 0.0 {
            ms
        } else {
            self.copy_ms * 0.8 + ms * 0.2
        };
    }

    /// Latest cumulative sensor counters from the capture thread. Differenced
    /// against the window base in [`Self::tick`] — cumulative in, rates out.
    /// `None` means this backend's driver cannot report what it delivered —
    /// not that it delivered nothing. Keeping the two apart is the whole point
    /// of the suffix: a Kinect v2 reporting `0.0 of 0.0 sensor` says the
    /// sensor went quiet, while a webcam simply has nothing to add.
    /// The colour camera's own brightness read-out. See [`Self::light_note`].
    fn note_exposure(&mut self, e: Option<(f32, f32, u32)>) {
        self.exposure = e;
    }

    /// ` LIGHT(exposure 12.4, gain 1.0, sensor 15.0 Hz)`, or nothing on a
    /// backend that cannot say.
    ///
    /// `exposure` runs 0.5 (very bright) to ~60 (lens covered) and `gain` 1.0
    /// to 1.5 — libfreenect2 gives no unit, so this is a brightness index, not
    /// photometry. The Hz is derived from the sensor's own frame clock
    /// (0.125 ms per step: 266 → 30 Hz, 533 → 15 Hz), which is the one rate in
    /// this line that owes nothing to a counter of ours.
    fn light_note(&self) -> String {
        let Some((exposure, gain, step)) = self.exposure else {
            return String::new();
        };
        let hz = if step > 0 {
            format!("{:.1} Hz", 8000.0 / f64::from(step))
        } else {
            "?".to_owned()
        };
        format!(" LIGHT(exposure {exposure:.1}, gain {gain:.2}, sensor {hz}),")
    }

    fn note_sensor(&mut self, counts: Option<(SensorCounts, SensorCounts)>) {
        self.reports_sensor = counts.is_some();
        let Some((rgb, depth)) = counts else {
            return;
        };
        self.sensor_rgb = rgb;
        self.sensor_depth = depth;
    }

    fn note_reg_ms(&mut self, ms: f32) {
        self.reg_ms = if ms == 0.0 || self.reg_ms == 0.0 {
            ms
        } else {
            self.reg_ms * 0.8 + ms * 0.2
        };
    }
    /// anchor calibration is locked → the detector no longer runs, so report
    /// 0 ms instead of holding the last inference time.
    /// One frame's worth of time at the sensor's own rate. Called when a
    /// backend opens: a 60 fps webcam has half the budget a Kinect has, and a
    /// fixed 30 would have called it healthy at twice the real cost.
    fn set_nominal_fps(&mut self, fps: f32) {
        if fps > 1.0 {
            self.budget_ms = 1000.0 / fps;
        }
    }
    fn note_anchor_locked(&mut self) {
        self.anchor_ms = 0.0;
        self.anchor_locked = true;
    }
    /// Add `n` captured RGB frames to the IN counter. Called with the
    /// cumulative-capture delta each time the GL thread consumes a published
    /// frame, so frames captured but never shown are still counted.
    fn add_input(&mut self, n: u64) {
        self.in_frames += n as u32;
    }
    /// One captured RGB frame reached the display (texture uploaded). Counted
    /// per shown frame — same unit as [`Self::add_input`] — so `out` measures
    /// the display rate of captured frames.
    fn note_output_frame(&mut self) {
        self.out_frames += 1;
    }
    /// One display repaint was presented (buffers swapped). Counts every GL
    /// present, not just those carrying a fresh camera frame — so `render`
    /// reflects the render thread's true cadence.
    fn note_render_frame(&mut self) {
        self.render_frames += 1;
    }
    /// Add `n` depth frames grabbed by the capture thread (diagnostic).
    fn add_depth(&mut self, n: u64) {
        self.depth_frames += n as u32;
    }
    /// Add `n` IR frames grabbed by the capture thread (v2 only, diagnostic —
    /// IR feeds no tracking, it's only exported with a shared capture).
    fn add_ir(&mut self, n: u64) {
        self.ir_frames += n as u32;
    }

    /// What one frame costs the capture thread *in series*: getting the frame
    /// out of the driver, the depth-to-colour alignment, the webcam's own
    /// decode, and the filters.
    ///
    /// The pose model is deliberately absent. It runs on its own thread and
    /// overlaps the next capture, so charging it to the frame budget both
    /// over-counted the wall clock and — because nothing counted the copy —
    /// produced a figure that could not explain the frame rate printed beside
    /// it. This one can: at 30 Hz, over 33.3 ms here *is* the reason.
    fn serial_ms(&self) -> f32 {
        self.copy_ms + self.reg_ms + self.convert_ms + (self.median_us + self.euro_us) / 1000.0
    }

    /// ` of 30.0 sensor, 67% dropped`, or nothing when the driver does not
    /// report what it delivered.
    fn sensor_note(fps: Option<f32>, drop_pct: Option<f32>) -> String {
        match (fps, drop_pct) {
            (Some(f), Some(d)) => format!(" of {f:.1} sensor, {d:.0}% dropped"),
            _ => String::new(),
        }
    }

    /// Called once per poll: roll the 1 s window (recompute FPS + CPU%) and
    /// log a line every ~2 s so the downloadable log carries the same numbers
    /// as the toolbar.
    fn tick(&mut self) {
        let now = Instant::now();
        let elapsed = now.duration_since(self.window_start).as_secs_f32();
        if elapsed >= 1.0 {
            self.in_fps = self.in_frames as f32 / elapsed;
            self.out_fps = self.out_frames as f32 / elapsed;
            self.render_fps = self.render_frames as f32 / elapsed;
            self.depth_fps = self.depth_frames as f32 / elapsed;
            self.ir_fps = self.ir_frames as f32 / elapsed;
            // Cumulative counters differenced over the same window as the
            // capture rates, so the two are directly comparable: "9.9 of 30"
            // means the sensor did its job and we read one frame in three.
            // `got == 0` used to yield `None`, which erased the suffix and made
            // a silent sensor indistinguishable from a backend that never had
            // the figure. A driver that can count reports its zero.
            let reports = self.reports_sensor;
            let rate = |now: SensorCounts, base: SensorCounts| {
                let got = now.delivered.saturating_sub(base.delivered);
                let lost = now.dropped.saturating_sub(base.dropped);
                let drop_pct = if got > 0 {
                    100.0 * lost as f32 / got as f32
                } else {
                    0.0
                };
                reports.then(|| (got as f32 / elapsed, drop_pct))
            };
            let rgb = rate(self.sensor_rgb, self.sensor_rgb_base);
            let depth = rate(self.sensor_depth, self.sensor_depth_base);
            self.sensor_in_fps = rgb.map(|(f, _)| f);
            self.in_drop_pct = rgb.map(|(_, d)| d);
            self.sensor_ir_fps = depth.map(|(f, _)| f);
            self.ir_drop_pct = depth.map(|(_, d)| d);
            self.sensor_rgb_base = self.sensor_rgb;
            self.sensor_depth_base = self.sensor_depth;
            // sysinfo derives CPU% from the gap between two refreshes, so it
            // needs the previous sample to still be there — hence the long-lived
            // `System`. The 1 s window is comfortably above its 200 ms minimum.
            self.sys.refresh_processes_specifics(
                sysinfo::ProcessesToUpdate::Some(&[self.pid]),
                true,
                sysinfo::ProcessRefreshKind::nothing()
                    .with_cpu()
                    .with_memory(),
            );
            if let Some(proc_) = self.sys.process(self.pid) {
                self.cpu_pct = proc_.cpu_usage();
                self.ram_mib = proc_.memory() / (1024 * 1024);
            }
            self.in_frames = 0;
            self.out_frames = 0;
            self.render_frames = 0;
            self.depth_frames = 0;
            self.ir_frames = 0;
            self.window_start = now;
        }
        // 5 s in the steady state -- 2 s filled a field log with 868 near
        // identical lines -- but a status flip is logged the moment it
        // happens, so a short stall is never averaged away.
        let used_now = self.serial_ms();
        let over_now = used_now > self.budget_ms;
        let flipped = over_now != self.was_over;
        self.was_over = over_now;
        if flipped || now.duration_since(self.last_log).as_secs_f32() >= 5.0 {
            let used = self.serial_ms();
            let budget = self.budget_ms;
            perf_table::push_perf(perf_table::PerfRow {
                at: stamp_hms(),
                cam_fps: self.in_fps,
                sensor_fps: self.sensor_in_fps,
                drop_pct: self.in_drop_pct,
                ir_fps: self.ir_fps,
                copy_ms: self.copy_ms,
                align_ms: self.reg_ms,
                anchor_ms: (!self.anchor_locked).then_some(self.anchor_ms),
                head_ms: self.head_ms,
                median_us: self.median_us,
                euro_us: self.euro_us,
                image_fps: self.out_fps,
                render_fps: self.render_fps,
                cpu_pct: self.cpu_pct,
                ram_mib: self.ram_mib,
                used_ms: used,
                budget_ms: budget,
            });
            info!(
                // Pipeline order, so the line reads the way the data flows.
                // ASCII only: a `->` and not an arrow glyph, because Windows
                // tools open this file as ANSI and turn UTF-8 arrows into
                // mojibake.
                "perf IN(cam {:.1} fps{} | ir {:.1} fps{} | depth {:.1} fps),{} \
                 COPY(frame {:.1} ms), \
                 MAP(align {}), \
                 AI(anchor {} | head {:.1} ms, own thread), \
                 FILTER(median {:.0} us | 1euro {:.0} us), \
                 OUT(image {:.1} fps | render {:.1} fps), \
                 SYS(cpu {:.0}% | ram {} MiB) \
                 used {:.1} / {:.1} ms {}",
                self.in_fps,
                Self::sensor_note(self.sensor_in_fps, self.in_drop_pct),
                self.ir_fps,
                Self::sensor_note(self.sensor_ir_fps, self.ir_drop_pct),
                self.depth_fps,
                self.light_note(),
                self.copy_ms,
                if self.reg_ms > 0.0 {
                    format!("{:.1} ms", self.reg_ms)
                } else {
                    // Not a zero cost: depth-to-colour alignment only exists on
                    // the v2, and is skipped there when tracking on IR.
                    "n/a".to_string()
                },
                if self.anchor_locked {
                    // Same word the calibration itself logs: "cabinet locked
                    // from live detection". One state, one name.
                    "locked".to_string()
                } else {
                    format!("{:.1} ms", self.anchor_ms)
                },
                self.head_ms,
                self.median_us,
                self.euro_us,
                self.out_fps,
                self.render_fps,
                self.cpu_pct,
                self.ram_mib,
                used,
                budget,
                if used > budget { "OVERLOAD!" } else { "OK" }
            );
            self.last_log = now;
        }
    }

    /// One-line summary for the toolbar.
    /// Same fields and names as the log line, minus what the log carries for
    /// post-mortem only -- the toolbar has far less room than a file.
    fn summary(&self) -> String {
        let mut s = format!("cam {:.0}", self.in_fps);
        // The sensor rate only earns toolbar room when we are losing frames:
        // "cam 10 of 30" is the whole diagnosis, and at 0% it is noise.
        if let (Some(sensor), Some(drop)) = (self.sensor_in_fps, self.in_drop_pct)
            && drop >= 1.0
        {
            s.push_str(&format!(" of {sensor:.0}, {drop:.0}% dropped"));
        }
        if self.ir_fps > 0.0 {
            s.push_str(&format!(" | ir {:.0}", self.ir_fps));
        }
        if self.copy_ms > 0.0 {
            s.push_str(&format!(" · copy {:.0}ms", self.copy_ms));
        }
        if self.reg_ms > 0.0 {
            s.push_str(&format!(" · align {:.0}ms", self.reg_ms));
        }
        s.push_str(&format!(
            " · anchor {} · head {:.0}ms · image {:.0} | render {:.0} fps · cpu {:.0}% · ram {} MiB",
            if self.anchor_locked {
                "locked".to_string()
            } else {
                format!("{:.0}ms", self.anchor_ms)
            },
            self.head_ms,
            self.out_fps,
            self.render_fps,
            self.cpu_pct,
            self.ram_mib,
        ));
        let used = self.serial_ms();
        s.push_str(&format!(
            " · {:.1}/{:.1}ms {}",
            used,
            self.budget_ms,
            if used > self.budget_ms {
                "OVERLOAD!"
            } else {
                "OK"
            }
        ));
        s
    }
}

impl App {
    fn new(logs: Arc<Mutex<VecDeque<String>>>) -> Self {
        let available = detect_backends();
        let kinect_access_hint = compute_kinect_access_hint();
        Self {
            selected: Backend::None,
            usb_cache: None,
            usb_probe: None,
            pov_unit: PovUnit::default(),
            usb_window_open: false,
            brief: None,
            available,
            active: None,
            error: None,
            logs,
            kinect_access_hint,
            kinect_access_result: None,
            kinect_access_recheck_at: None,
            screenshot_status: None,
            // "270°" in the player's frame = egui-rotate CW90 once applied
            // (the rotated pincab screen inverts the apparent direction;
            // see `rotation_label`). This is the orientation that reads
            // upright on the cab.
            rotation: Rotation::CW90,
            should_quit: false,
            lockbar_width_mm: headtracking::calibration::LOCKBAR_WIDTH_MM,
            head_filter_min_cutoff: 1.0,
            median_window_frames: 3,
            // beta=0 would disable the 1€ filter's velocity adaptation — its
            // whole point — and make fast head moves lag behind. Small but
            // non-zero keeps stillness smooth AND fast moves responsive.
            head_filter_beta: 0.03,
            bypass_filters: false,
            selected_stream: StreamKind::Rgb,
            color_exposure: ColorExposureMode::default(),
            contribute_open: false,
            consent_checked: false,
            uploader: contribute::Uploader::spawn(default_rescue_dir()),
            drop_reach: ReachState::Unknown,
            drop_probe: None,
            contrib_last: None,
            contrib_local: LocalCopy::default(),
            contrib_saved_in: None,
            contrib_save_error: None,
            contrib_thumbs: None,
            switch_state: SwitchState::Idle,
            parallax_enabled: true,
            flatten_view: false,
            flatten_guides: None,
            parallax_tex: None,
            // Auto-orbit by default: the parallax illusion shows immediately
            // with no camera and no need to move — switch to Live on the cab.
            parallax_eye_mode: ParallaxEye::Live,
            parallax_eye: [0.0, 0.0, PX_DVIEW_MM],
            parallax_gain: 1.0,
            table_incl_deg: DEFAULT_TABLE_INCL_DEG,
            // Y flipped by default: Kinect Y points down, the eye's Y is up.
            // Z inverted so moving closer (head depth shrinks) pulls the eye
            // toward the screen → the scene grows, the fish-tank expectation.
            parallax_invert: [false, false, false],
            parallax_panel_rect: None,
            parallax_mouse_z: PX_DVIEW_MM,
            parallax_aspect: 16.0 / 9.0,
        }
    }

    fn refresh_available(&mut self) {
        // Drop the active device first — libfreenect[2] can't reliably
        // enumerate while a sibling context holds an open device on Linux,
        // and `webcam::force_refresh` below requires no live camera handle.
        if let Some(old) = self.active.take() {
            info!(backend = ?old.backend, "closing backend before scan");
            drop(old);
        }
        // Cycle SDL3's camera subsystem so a freshly plugged webcam shows
        // up. Mandatory on Windows (MediaFoundation has no hot-plug);
        // harmless on Linux/macOS — those backends already track hot-plug,
        // re-init just yields the same list.
        if let Err(e) = webcam::force_refresh() {
            info!(?e, "webcam subsystem refresh failed");
        }
        self.selected = Backend::None;
        self.available = detect_backends();
        // Rescanning means the bus is expected to have changed — that is the
        // whole reason to press the button. Keeping the old snapshot would
        // show someone the bus as it was before they plugged the sensor back
        // in, which is worse than showing nothing. The next frame starts a
        // fresh probe on its own.
        self.usb_cache = None;
        self.kinect_access_hint = compute_kinect_access_hint();
        self.kinect_access_result = None;
    }

    /// State machine, ticked once per frame. On a selection change it closes
    /// the old backend, waits out a "closing" then an "opening" settle (~1 s
    /// total of USB release time) and only then opens the new one — so the UI
    /// never blocks and a Kinect freed by one backend is ready before the next
    /// grabs it. A first open (nothing to close) skips straight to "opening".
    fn ensure_active(&mut self) {
        // Take the state out so the `Waiting` arm can own its `Box<Active>`;
        // each arm restores a non-`Idle` state when it should stay in-flight.
        match std::mem::replace(&mut self.switch_state, SwitchState::Idle) {
            SwitchState::Idle => {
                let needs_change = match (&self.active, self.selected) {
                    (Some(a), sel) => a.backend != sel,
                    (None, Backend::None) => false,
                    (None, _) => true,
                };
                if !needs_change {
                    return; // stays Idle
                }
                let had_old = self.active.is_some();
                if let Some(old) = self.active.take() {
                    info!(backend = ?old.backend, "closing backend");
                    drop(old);
                }
                self.error = None;
                if matches!(self.selected, Backend::None) {
                    return; // nothing to open — stay Idle
                }
                self.switch_state = if had_old {
                    SwitchState::Closing(Instant::now())
                } else {
                    SwitchState::Opening(Instant::now())
                };
            }
            SwitchState::Closing(t) => {
                self.switch_state = if t.elapsed() >= SWITCH_SETTLE {
                    SwitchState::Opening(Instant::now())
                } else {
                    SwitchState::Closing(t)
                };
            }
            SwitchState::Opening(t) => {
                if t.elapsed() < SWITCH_SETTLE {
                    self.switch_state = SwitchState::Opening(t);
                    return;
                }
                if matches!(self.selected, Backend::None) {
                    return; // selection changed to None during the settle → Idle
                }
                // Spawn the capture thread — it opens the device and confirms
                // the stream on its own thread, so this never blocks the UI. We
                // just poll its `Startup` handshake from the `Waiting` arm.
                info!(backend = ?self.selected, "spawning capture thread");
                self.switch_state = SwitchState::Waiting {
                    worker: CaptureWorker::spawn(self.selected),
                    since: Instant::now(),
                };
            }
            SwitchState::Waiting { worker, since } => {
                // Read the handshake without holding the lock across the move.
                let outcome = match &*worker.startup.lock() {
                    Startup::Pending => None,
                    Startup::Live(intr) => Some(Ok(*intr)),
                    Startup::Failed(e) => Some(Err(e.clone())),
                };
                match outcome {
                    Some(Ok(intr)) => {
                        info!(
                            backend = ?worker.backend,
                            fx = intr.fx,
                            fy = intr.fy,
                            "backend live"
                        );
                        // Devices don't offer the same streams (a webcam has no
                        // depth), and a fresh capture thread always starts on
                        // colour — so the view selection starts over too.
                        self.selected_stream = StreamKind::Rgb;
                        self.active = Some(Active::new_live(worker, intr)); // → Idle
                        // Say what this sensor can actually deliver, once,
                        // while the user is looking at it. Otherwise a 60 fps
                        // render bar reads as "all good" next to a camera
                        // feeding 10, and a colour stream at 15 in a dim room
                        // looks broken when it only wants a light switch.
                        self.brief = device_brief(self.selected);
                    }
                    Some(Err(e)) => {
                        error!("{e}");
                        self.error = Some(e);
                        self.selected = Backend::None;
                        drop(worker); // stop + join the thread, close the device
                    }
                    None => {
                        if since.elapsed() >= STARTUP_TIMEOUT {
                            let msg = format!(
                                "{}: capture thread did not start in time",
                                backend_slug(worker.backend)
                            );
                            error!("{msg}");
                            self.error = Some(msg);
                            self.selected = Backend::None;
                            drop(worker);
                        } else {
                            self.switch_state = SwitchState::Waiting { worker, since };
                        }
                    }
                }
            }
        }
    }

    /// Encode the current frame's raw + detection images, save both to
    /// `contributions/` and queue them for the write-only upload. No-op if no
    /// frame is available. Called from the "Share a capture" button.
    fn share_capture(&mut self) {
        // A contribution must export EVERY stream the camera has, whatever is
        // selected on screen. The v1's colour and IR share one USB endpoint,
        // so we always request BOTH from the capture thread: whichever is
        // already live costs one frame, the other borrows the endpoint through
        // a momentary mode switch and hands it back to the user's selected
        // stream (~500 ms). The GL thread never needs to know which mode the
        // device is in. A brief block here is fine (manual button); 2 s covers
        // the warmup + switch round-trip of each grab.
        let (rgb_v1, ir_v1) = match self.active.as_ref() {
            Some(active) if active.backend == Backend::KinectV1 => {
                let rgb = {
                    let (tx, rx) = mpsc::channel();
                    if active.worker.cmd_tx.send(CaptureCmd::GrabRgbV1(tx)).is_ok() {
                        rx.recv_timeout(Duration::from_secs(2)).ok().flatten()
                    } else {
                        None
                    }
                };
                let ir = {
                    let (tx, rx) = mpsc::channel();
                    if active.worker.cmd_tx.send(CaptureCmd::GrabIrV1(tx)).is_ok() {
                        rx.recv_timeout(Duration::from_secs(2)).ok().flatten()
                    } else {
                        None
                    }
                };
                (rgb, ir)
            }
            _ => (None, None),
        };
        // The v2 throttles colour conversion to ~2.5 Hz while tracking on IR —
        // ask for one un-throttled conversion so `_raw.png` isn't ~400 ms
        // stale. Acks immediately when no throttle is active.
        if let Some(active) = self.active.as_ref()
            && active.backend == Backend::KinectV2
        {
            let (tx, rx) = mpsc::channel();
            if active
                .worker
                .cmd_tx
                .send(CaptureCmd::RefreshRgb(tx))
                .is_ok()
            {
                let _ = rx.recv_timeout(Duration::from_secs(1));
            }
        }
        let payload = self.active.as_ref().and_then(|active| {
            // v1: prefer the freshly grabbed colour frame — while the IR
            // stream is selected, `last_rgb_frame` holds the gray-expanded IR
            // the pipeline tracked on, not true colour. Overlays stay
            // geometrically valid on the grab: the v1's colour and IR share
            // one sensor framing (same 640×480), so a pose computed on IR
            // lands on the right pixels of the colour frame.
            let (w, h, raw): (u32, u32, Arc<Vec<u8>>) = match rgb_v1.as_ref() {
                Some(f) => (f.width, f.height, Arc::new(f.data.clone())),
                None => {
                    let (w, h, pixels, layout) = active.last_rgb_frame.as_ref()?;
                    (*w, *h, Arc::new(frame_to_rgb888(pixels, *layout)))
                }
            };
            let det = bake_overlays(
                w,
                h,
                &raw,
                active.last_pose.as_ref(),
                active.last_anchor.as_ref(),
            );
            Some((
                active.backend,
                w,
                h,
                raw,
                det,
                active.last_depth.clone(),
                active.last_ir.clone(),
                active.last_head,
                active.last_pose.clone(),
                active.last_lockbar,
            ))
        });
        let Some((backend, w, h, raw, det, depth, ir_v2, head, pose, lockbar)) = payload else {
            return;
        };
        let stem = contribution_stem(backend);
        // Ask once, here, where the user's own copy should go — see
        // [`LocalCopy`] for why there is no default folder.
        if matches!(self.contrib_local, LocalCopy::Unasked) {
            self.contrib_local = match ask_local_copy_folder() {
                Some(dir) => LocalCopy::Folder(dir),
                None => LocalCopy::Declined,
            };
        }
        // Tracking read-out shared by both colour planes — embedded as PNG
        // tEXt so the capture is self-describing (head Z per backend, etc.).
        let mut meta = capture_meta(
            backend,
            (w, h),
            &stem,
            CabGeom {
                table_incl_deg: self.table_incl_deg,
                lockbar_mm: self.lockbar_width_mm,
            },
            head,
            pose.as_ref(),
            lockbar.as_ref(),
        );
        meta.extend(autocalib_meta(lockbar.as_ref(), (w, h), depth.as_deref()));
        let files = build_contribution_files(
            &stem,
            backend,
            (w, h),
            &raw,
            &det,
            depth.as_deref(),
            ir_v2.as_deref(),
            ir_v1.as_ref(),
            &meta,
        );
        // The upload happens whatever the local copy does — the drop is the
        // point — but record what the save really did so the window can say it
        // instead of assuming.
        self.contrib_saved_in = None;
        self.contrib_save_error = None;
        let upload = self.drop_reach.allows_upload();
        // With no route to the drop, the capture still has to land somewhere:
        // the folder the contributor chose, or ours if they declined one.
        // "Upload only" stops meaning anything when there is no upload.
        let local = match &self.contrib_local {
            LocalCopy::Folder(dir) => Some(dir.clone()),
            LocalCopy::Unasked | LocalCopy::Declined if !upload => Some(default_rescue_dir()),
            LocalCopy::Unasked | LocalCopy::Declined => None,
        };
        if let Some(dir) = &local {
            // Anything the drop refuses lands beside the copy they already
            // know about, so a hand-over is one folder, not two.
            self.uploader.set_rescue_dir(dir.clone());
            if let Err(e) = std::fs::create_dir_all(dir) {
                warn!(dir = %dir.display(), "contribution: local folder unusable: {e}");
                self.contrib_save_error = Some(format!("{}: {e}", dir.display()));
            }
        }
        // Counts describe this capture, not the session: a failure inside a
        // set of seven must not hide behind an accumulated success total.
        self.uploader.begin_batch();
        let mut saved = 0usize;
        for (name, bytes) in files {
            if let Some(dir) = &local {
                match std::fs::write(dir.join(&name), &bytes) {
                    Ok(()) => saved += 1,
                    Err(e) => {
                        warn!(name, "contribution: local save failed: {e}");
                        self.contrib_save_error
                            .get_or_insert_with(|| format!("{}: {e}", dir.display()));
                    }
                }
            }
            if upload {
                self.uploader.submit(name, bytes);
            }
        }
        if saved > 0 {
            self.contrib_saved_in = local;
        }
        info!(stem, saved, upload, "capture shared");
        self.contrib_last = Some(stem);
    }

    fn poll(&mut self, egui_ctx: &egui::Context) {
        let Some(active) = self.active.as_mut() else {
            return;
        };
        // Perf counters come from the capture thread's live atomics, never from
        // the frame this thread happens to consume: a stall is exactly when
        // they matter, and reading them inside the consume branch made "the
        // sensor delivered nothing" and "we stopped reading it" print the same
        // zeros. See [`CaptureVitals`].
        let (captured, depth_captured, ir_captured, sensor) = active.worker.vitals.read();
        active
            .metrics
            .add_input(captured.saturating_sub(active.last_captured));
        active.last_captured = captured;
        active
            .metrics
            .add_depth(depth_captured.saturating_sub(active.last_depth_count));
        active.last_depth_count = depth_captured;
        active
            .metrics
            .add_ir(ir_captured.saturating_sub(active.last_ir_count));
        active.last_ir_count = ir_captured;
        active.metrics.note_sensor(sensor);
        active
            .metrics
            .note_exposure(active.worker.vitals.exposure());
        active.metrics.tick();
        // Hand the live-tunable 1€ / bypass knobs to the capture thread (cheap
        // atomics); the device poll + inference now run over there.
        active.worker.set_filter(
            self.head_filter_min_cutoff,
            self.head_filter_beta,
            self.median_window_frames,
            self.bypass_filters,
        );
        // Consume the latest processed frame the capture thread published.
        // This branch now only feeds the display: OUT counts what we upload,
        // so `out ≤ in`, with a genuine gap whenever rendering runs slower
        // than capture — and IN keeps counting when it does not run at all.
        if let Some(frame) = active.worker.latest.load_full()
            && frame.frame_id != active.last_consumed_id
        {
            active.last_consumed_id = frame.frame_id;
            let mut img = stream_color_image(&frame, self.selected_stream);
            if self.flatten_view
                && let Some(geom) = frame.anchor.as_ref()
            {
                let fx = color_focal_px(active.backend, frame.w);
                let intr = anchor::CameraIntrinsics {
                    fx,
                    fy: fx,
                    cx: frame.w as f32 * 0.5,
                    cy: frame.h as f32 * 0.5,
                };
                // Half resolution: this is a diagnostic view that gets scaled
                // into a panel anyway, and the resampler walks every
                // destination pixel on the GL thread.
                let (dw, dh) = (
                    (img.width() / 2).max(2) as u32,
                    (img.height() / 2).max(2) as u32,
                );
                if let Some(flat) = anchor::flatten_homography(geom, &intr, dw, dh) {
                    img = flatten_image(&img, &flat.dst_to_src, dw as usize, dh as usize);
                    let norm = |p: (f32, f32)| (p.0 / dw as f32, p.1 / dh as f32);
                    self.flatten_guides = Some(FlattenGuides {
                        left: norm(flat.bar_left),
                        right: norm(flat.bar_right),
                        // Same defect the 'square' column reports: out of
                        // square there is a lean here, by the same amount.
                        lean_deg: anchor::camera_pose(geom, &intr, self.lockbar_width_mm)
                            .map_or(0.0, |p| p.rect_angle_deg - 90.0),
                    });
                } else {
                    self.flatten_guides = None;
                }
            } else {
                self.flatten_guides = None;
            }
            upload_texture(egui_ctx, &mut active.rgb_texture, img);
            active.metrics.note_output_frame();
            active.last_frame_w = frame.w;
            active.last_rgb_at = frame.last_rgb_at;
            active.last_ir_at = frame.last_ir_at;
            active.last_depth_at = frame.last_depth_at;
            active.pose_src = (frame.pose_src_w, frame.pose_src_h);
            active.last_pose = frame.pose.clone();
            active.last_head = frame.head;
            active.baseline = frame.baseline;
            active.last_anchor = frame.anchor;
            active.last_lockbar = frame.lockbar;
            active.last_rgb_frame = Some((frame.w, frame.h, frame.pixels.clone(), frame.layout));
            active.last_depth = frame.depth.clone();
            active.last_depth_color = frame.depth_color.clone();
            active.last_ir = frame.ir.clone();
            if frame.head_ms > 0.0 {
                active.metrics.note_head_ms(frame.head_ms);
            }
            active.metrics.note_reg_ms(frame.reg_ms);
            active.metrics.note_filter_us(frame.filter_us);
            active.metrics.note_convert_ms(frame.convert_ms);
            active.metrics.note_copy_ms(frame.copy_ms);
            active.anchor_locked = frame.anchor_locked;
            if frame.anchor_locked {
                active.metrics.note_anchor_locked();
            } else if frame.anchor_ms > 0.0 {
                active.metrics.note_anchor_ms(frame.anchor_ms);
            }
        }
    }
}

/// Apply the 1€ filter to the head pose in millimetres. The pixel coords
/// `u`, `v` are passed through unchanged — they record where on the depth
/// frame we sampled, not a re-projected smoothed point.
/// Per-stage cost of [`smooth_head`], in microseconds. Both stages are a
/// handful of arithmetic ops, so `Instant::now()` overhead (tens of ns each)
/// is a real fraction of what is being measured -- read these as an order of
/// magnitude, not an exact figure.
#[derive(Clone, Copy, Default)]
struct FilterUs {
    median: f32,
    euro: f32,
}

fn smooth_head(
    raw: Option<HeadPixel>,
    filter: &mut filter_alias::OneEuroPose3D,
    gate: &mut headtracking::filter::MedianGate,
    started_at: Instant,
    bypass: bool,
) -> (Option<HeadPixel>, FilterUs) {
    let Some(head) = raw else {
        return (None, FilterUs::default());
    };
    if bypass {
        // raw pose, no median gate, no 1-euro smoothing
        return (Some(head), FilterUs::default());
    }
    let mut head = head;
    let t_us = started_at.elapsed().as_micros() as u64;
    let t0 = Instant::now();
    let gated = gate.push([head.x_mm, head.y_mm, head.depth_mm]);
    let t1 = Instant::now();
    let smoothed = filter.update(gated, t_us);
    let cost = FilterUs {
        median: t1.duration_since(t0).as_secs_f32() * 1e6,
        euro: t1.elapsed().as_secs_f32() * 1e6,
    };
    head.x_mm = smoothed[0];
    head.y_mm = smoothed[1];
    head.depth_mm = smoothed[2];
    (Some(head), cost)
}

impl App {
    /// Render the Kinect v1 tilt + LED panel just below the toolbar.
    /// No-op when the active backend is anything else.
    fn show_v1_controls(&mut self, ui: &mut egui::Ui) {
        let Some(active) = self.active.as_mut() else {
            return;
        };
        if active.backend != Backend::KinectV1 {
            return;
        }
        // The device lives on the capture thread: commands go through the
        // channel, the live tilt/accel read-out comes from the shared cell.
        let cmd_tx = active.worker.cmd_tx.clone();
        let tilt_state = *active.worker.tilt_state.lock();
        let Some(controls) = active.v1_ui.as_mut() else {
            return;
        };
        if let Some(state) = tilt_state {
            controls.last_state = Some(state);
            // Seed the slider to the device's real tilt on the first read so it
            // doesn't snap from 0.
            if !controls.seeded {
                controls.desired_tilt_deg = state.angle_deg;
                controls.last_sent_tilt_deg = state.angle_deg;
                controls.seeded = true;
            }
        }

        Panel::top("v1-controls").show(ui, |ui| {
            ui.add_space(2.0);
            ui.horizontal(|ui| {
                ui.label(RichText::new("Kinect v1").strong());
                ui.separator();
                let response = ui.add(
                    egui::Slider::new(
                        &mut controls.desired_tilt_deg,
                        freenect::TILT_MIN_DEG..=freenect::TILT_MAX_DEG,
                    )
                    .text("tilt °")
                    .step_by(1.0),
                );
                let drag_release = response.drag_stopped();
                let typed_commit = response.lost_focus();
                if (drag_release || typed_commit)
                    && (controls.desired_tilt_deg - controls.last_sent_tilt_deg).abs() > 0.01
                {
                    let _ = cmd_tx.send(CaptureCmd::SetTilt(controls.desired_tilt_deg));
                    controls.last_sent_tilt_deg = controls.desired_tilt_deg;
                    info!(angle = controls.desired_tilt_deg, "tilt command sent");
                }

                ui.separator();
                ui.label("LED:");
                let prev_led = controls.selected_led;
                ComboBox::from_id_salt("led")
                    .selected_text(led_label(controls.selected_led))
                    .show_ui(ui, |ui| {
                        for led in LED_OPTIONS {
                            ui.selectable_value(&mut controls.selected_led, *led, led_label(*led));
                        }
                    });
                if controls.selected_led != prev_led {
                    let _ = cmd_tx.send(CaptureCmd::SetLed(controls.selected_led));
                    controls.last_sent_led = controls.selected_led;
                }

                ui.separator();
                if let Some(state) = controls.last_state {
                    ui.label(
                        RichText::new(format!(
                            "current {:>+5.1}°  status {:?}  accel ({:>+5.2}, {:>+5.2}, {:>+5.2}) m/s²",
                            state.angle_deg,
                            state.status,
                            state.accel_mks[0],
                            state.accel_mks[1],
                            state.accel_mks[2],
                        ))
                        .monospace()
                        .color(Color32::GRAY),
                    );
                } else {
                    ui.label(RichText::new("waiting for motor state…").color(Color32::GRAY));
                }
            });
            ui.add_space(2.0);
        });
    }
}

const LED_OPTIONS: &[freenect::LedState] = &[
    freenect::LedState::Off,
    freenect::LedState::Green,
    freenect::LedState::Red,
    freenect::LedState::Yellow,
    freenect::LedState::BlinkGreen,
    freenect::LedState::BlinkRedYellow,
];

fn led_label(state: freenect::LedState) -> &'static str {
    match state {
        freenect::LedState::Off => "off",
        freenect::LedState::Green => "green",
        freenect::LedState::Red => "red",
        freenect::LedState::Yellow => "yellow",
        freenect::LedState::BlinkGreen => "blink green",
        freenect::LedState::BlinkRedYellow => "blink red/yellow",
    }
}

fn upload_texture(ctx: &egui::Context, slot: &mut Option<TextureHandle>, img: ColorImage) {
    match slot.as_mut() {
        Some(tex) => tex.set(img, egui::TextureOptions::LINEAR),
        None => *slot = Some(ctx.load_texture("rgb", img, egui::TextureOptions::LINEAR)),
    }
}

fn capture_baseline(slot: &mut Option<Baseline>, head: Option<HeadPixel>) {
    if slot.is_some() {
        return;
    }
    let Some(head) = head else { return };
    let baseline = Baseline {
        x_mm: head.x_mm,
        y_mm: head.y_mm,
        z_mm: head.depth_mm,
    };
    *slot = Some(baseline);
    info!(
        x_mm = baseline.x_mm,
        y_mm = baseline.y_mm,
        z_mm = baseline.z_mm,
        "baseline captured"
    );
}

impl App {
    /// Recompute [`Self::parallax_eye`] for this frame from the active
    /// source. Mouse mode reads egui's *logical* pointer (already rotated by
    /// egui-rotate) against last frame's panel rect — a 1-frame lag that's
    /// imperceptible. Live mode maps the lockbar-relative head deltas through
    /// the debug gain + invert toggles.
    fn update_parallax_eye(&mut self, ctx: &egui::Context) {
        const RANGE_X: f32 = 220.0;
        const RANGE_Y: f32 = 160.0;
        match self.parallax_eye_mode {
            ParallaxEye::AutoOrbit => {
                let t = ctx.input(|i| i.time) as f32;
                self.parallax_eye = [
                    130.0 * (t * 0.9).sin(),
                    70.0 * (t * 0.6 + 1.0).sin(),
                    PX_DVIEW_MM + 90.0 * (t * 0.5).sin(),
                ];
            }
            ParallaxEye::Mouse => {
                let scroll = ctx.input(|i| i.smooth_scroll_delta.y);
                if scroll != 0.0 {
                    self.parallax_mouse_z = (self.parallax_mouse_z + scroll).clamp(200.0, 1200.0);
                }
                if let (Some(rect), Some(pos)) =
                    (self.parallax_panel_rect, ctx.pointer_latest_pos())
                    && rect.contains(pos)
                {
                    let nx = ((pos.x - rect.left()) / rect.width()) * 2.0 - 1.0;
                    let ny = ((pos.y - rect.top()) / rect.height()) * 2.0 - 1.0;
                    self.parallax_eye = [nx * RANGE_X, -ny * RANGE_Y, self.parallax_mouse_z];
                } else {
                    self.parallax_eye[2] = self.parallax_mouse_z;
                }
            }
            ParallaxEye::Live => {
                let pose = self
                    .active
                    .as_ref()
                    .and_then(|a| match (a.baseline, a.last_head) {
                        (Some(base), Some(head)) => Some((base, head)),
                        _ => None,
                    });
                if let Some((base, head)) = pose {
                    let g = self.parallax_gain;
                    let sign = |i: usize| if self.parallax_invert[i] { -1.0 } else { 1.0 };
                    let dx = head.x_mm - base.x_mm;
                    let dy = head.y_mm - base.y_mm;
                    let dz = head.depth_mm - base.z_mm;
                    // The camera faces the standing player (~vertical), but the
                    // VPX screen is the near-flat playfield at the bottom. Tilt
                    // the head's vertical/depth motion by (90° − table
                    // inclination) so the parallax feels right on the laid-flat
                    // screen. X (left/right) is unaffected.
                    let theta = (90.0 - self.table_incl_deg).to_radians();
                    let (ct, st) = (theta.cos(), theta.sin());
                    let dy_t = dy * ct + dz * st;
                    let dz_t = -dy * st + dz * ct;
                    // Axis conventions settled by field testing (2026-08-06):
                    // Y passes straight through, and moving TOWARD the cab
                    // brings the eye closer (the baked `-` on Z). The ±X/Y/Z
                    // toggles stay all-off by default — they exist for exotic
                    // camera mountings, not to fix the normal case.
                    self.parallax_eye = [
                        sign(0) * dx * g,
                        sign(1) * dy_t * g,
                        (PX_DVIEW_MM - sign(2) * dz_t * g).clamp(150.0, 1500.0),
                    ];
                } else {
                    self.parallax_eye = [0.0, 0.0, PX_DVIEW_MM];
                }
            }
        }
    }

    fn ui(&mut self, ui: &mut egui::Ui) {
        // egui 0.34 nests panels inside a root Ui; grab a Context handle
        // for the bits that still need one (texture upload in `poll`).
        let egui_ctx = ui.ctx().clone();
        self.ensure_active();
        // While the access banner is up, re-probe every few seconds and
        // rescan automatically once the drivers are in place — whether they
        // came from our button or from a manually-run setup.ps1.
        if self.kinect_access_hint {
            let due = self
                .kinect_access_recheck_at
                .is_none_or(|t| Instant::now() >= t);
            if due {
                self.kinect_access_recheck_at = Some(Instant::now() + Duration::from_secs(5));
                if !compute_kinect_access_hint() {
                    info!("Kinect access fixed — rescanning automatically");
                    self.refresh_available();
                    self.kinect_access_result = Some(Ok(
                        "drivers detected — device list rescanned automatically".into(),
                    ));
                }
            }
        } else {
            self.kinect_access_recheck_at = None;
        }
        self.poll(&egui_ctx);
        if self.parallax_enabled {
            self.update_parallax_eye(&egui_ctx);
        }

        // ----- Top toolbar: one button-only row, then an INPUT row
        // (raw camera measurements) and an OUTPUT row (what VPX consumes).
        Panel::top("toolbar").show(ui, |ui| {
            ui.add_space(4.0);

            // Row 1 — buttons only. Wrapped so a narrow window folds the bar
            // onto extra lines instead of clipping controls off the edge.
            ui.horizontal_wrapped(|ui| {
                ui.label("Input:");
                // What USB gives this sensor, beside the sensor itself. A
                // starved Kinect still opens and streams, so the only moment
                // this is cheap to notice is while picking the device.
                if let Some(sensor) = match self.selected {
                    Backend::KinectV1 => Some(usb_check::Sensor::KinectV1),
                    Backend::KinectV2 => Some(usb_check::Sensor::KinectV2),
                    _ => None,
                } {
                    self.poll_usb_probe(sensor);
                    // One probe per sensor, off-thread, and never again until
                    // the sensor changes or a rescan clears the snapshot —
                    // this is a scheduling decision made every frame, not a
                    // bus read. It is what lets the badge carry a colour and
                    // the window open with content already in it.
                    if !self.usb_cache.as_ref().is_some_and(|(s, _)| *s == sensor) {
                        self.start_usb_probe(sensor);
                    }
                    // Colour comes from a snapshot already in hand, if there
                    // is one. The button schedules; it never enumerates.
                    let colour = self
                        .usb_cache
                        .as_ref()
                        .filter(|(s, _)| *s == sensor)
                        .and_then(|(_, snap)| snap.report.as_ref())
                        .map_or(Color32::GRAY, |r| r.level.colour());
                    if ui
                        .button(RichText::new("USB").color(colour).monospace())
                        .on_hover_text(
                            "What USB gives this sensor, and everything plugged into every \
                             bus. Click to open.",
                        )
                        .clicked()
                    {
                        self.usb_window_open = true;
                        self.start_usb_probe(sensor);
                    }
                }
                let selected_label = self.label_for(self.selected);
                // Size the combo to the widest label currently on offer so no
                // entry wraps (a wrapped entry spans several lines and pushes
                // the popup past its max height → a scrollbar). Recomputed
                // every frame, so it grows when a rescan adds a long webcam
                // name and shrinks back when the camera is unplugged. Width is
                // estimated from character count (≈ button glyph advance) — a
                // hair generous, which is exactly what we want here.
                let font_h = egui::TextStyle::Button.resolve(ui.style()).size;
                let glyph_w = font_h * 0.58;
                let longest = self
                    .available
                    .iter()
                    .map(|e| e.label.chars().count())
                    .max()
                    .unwrap_or(0)
                    .max(selected_label.chars().count());
                // Room for the checkmark gutter + dropdown arrow + padding;
                // clamp so a pathological device name can't overflow the row.
                let combo_w = (longest as f32 * glyph_w + 48.0).clamp(140.0, 520.0);
                // Popup tall enough to show every entry without an inner
                // vertical scroll. `.height()` is a MAX (egui caps the popup's
                // ScrollArea at it). The old magic 38 px/row was smaller than the
                // real row height under the cab's larger UI font, so the list
                // clipped + scrolled instead of growing. Derive it from the
                // actual row height (font + button padding + inter-row spacing) —
                // same spirit as the width above — so it scales with the font and
                // always fits every entry.
                let sp = ui.spacing();
                let row_h =
                    font_h.max(sp.interact_size.y) + 2.0 * sp.button_padding.y + sp.item_spacing.y;
                let popup_h =
                    (self.available.len().max(1) as f32) * row_h + sp.item_spacing.y + 8.0;
                let combo_debug_var = std::env::var("HT_DEBUG_COMBO").ok();
                let combo_debug = combo_debug_var.is_some();
                let mut entries = self.available.clone();
                if combo_debug_var.as_deref() == Some("grow") {
                    // Repro harness for the stale-popup-size bug: draw only 3
                    // entries for the first 6 s, then all of them — mimics
                    // "open the dropdown, rescan adds a camera, open again".
                    static T0: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
                    if T0.get_or_init(Instant::now).elapsed() < Duration::from_secs(6) {
                        entries.truncate(3);
                    }
                }
                // (egui 0.36 reruns the popup sizing pass on reopen — the
                // 0.35-era workaround of salting the id with the entry count
                // is gone; upstream fix: emilk/egui#8315.)
                let combo_resp = ComboBox::from_id_salt("backend")
                    .width(combo_w)
                    .height(popup_h)
                    .selected_text(selected_label)
                    .show_ui(ui, |ui| {
                        // Keep each entry on a single line; the popup then
                        // stretches to exactly the number of entries.
                        ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
                        for entry in &entries {
                            if entry.needs_drivers {
                                // Visible but not selectable: the hardware IS
                                // detected, only the driver is missing — the
                                // fix banner above has the one-click install,
                                // and the auto-rescan re-enables the entry.
                                ui.add_enabled_ui(false, |ui| {
                                    let _ = ui.selectable_label(
                                        false,
                                        format!(
                                            "{} — install drivers first (see ⚠ banner)",
                                            entry.label
                                        ),
                                    );
                                });
                            } else {
                                ui.selectable_value(
                                    &mut self.selected,
                                    entry.backend,
                                    &entry.label,
                                );
                            }
                        }
                        if combo_debug {
                            // Measured from INSIDE the popup: whatever style /
                            // clip the rows actually got, vs our estimate.
                            let sp = ui.spacing();
                            eprintln!(
                                "combo popup: content_h={:.1} clip_h={:.1} max_h={:.1} \
                                 avail_h={:.1} | interact_y={:.1} pad_y={:.1} space_y={:.1} \
                                 font_h={:.1}",
                                ui.min_rect().height(),
                                ui.clip_rect().height(),
                                ui.max_rect().height(),
                                ui.available_rect_before_wrap().height(),
                                sp.interact_size.y,
                                sp.button_padding.y,
                                sp.item_spacing.y,
                                egui::TextStyle::Button.resolve(ui.style()).size,
                            );
                        }
                    });
                if combo_debug {
                    eprintln!(
                        "combo estimate: n={} row_h={row_h:.1} popup_h={popup_h:.1} \
                         combo_w={combo_w:.1} | toolbar interact_y={:.1} pad_y={:.1} \
                         space_y={:.1} font_h={font_h:.1}",
                        self.available.len(),
                        ui.spacing().interact_size.y,
                        ui.spacing().button_padding.y,
                        ui.spacing().item_spacing.y,
                    );
                    // Keep the popup open so layout can be inspected without a
                    // mouse — EXCEPT, in grow mode, during a short window
                    // around the 6 s entry-count change: the real-world flow
                    // (open → Rescan → reopen) closes the popup in between,
                    // and egui's reopen sizing pass (0.36, emilk/egui#8315)
                    // only runs on that closed→open transition.
                    static T0: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
                    let elapsed = T0.get_or_init(Instant::now).elapsed().as_secs_f32();
                    let grow_gap =
                        combo_debug_var.as_deref() == Some("grow") && (5.0..7.0).contains(&elapsed);
                    if grow_gap {
                        egui::Popup::close_id(ui.ctx(), combo_resp.response.id.with("popup"));
                    } else {
                        egui::Popup::open_id(ui.ctx(), combo_resp.response.id.with("popup"));
                    }
                }
                if ui.button("Rescan").clicked() {
                    self.refresh_available();
                }
                // Screenshot: writes the latest RGB frame next to the binary
                // as `<backend-slug>_<YYYYMMDD-HHMMSS>.png`. Disabled until a
                // frame has been received.
                let shot_ready = self
                    .active
                    .as_ref()
                    .is_some_and(|a| a.last_rgb_frame.is_some());
                let shot_resp = ui.add_enabled(shot_ready, egui::Button::new("📷 Screenshot"));
                if shot_resp.clicked()
                    && let Some(active) = self.active.as_ref()
                    && let Some((w, h, pixels, layout)) = active.last_rgb_frame.as_ref()
                {
                    let slug = backend_slug(active.backend);
                    let meta = capture_meta(
                        active.backend,
                        (*w, *h),
                        &slug,
                        CabGeom {
                            table_incl_deg: self.table_incl_deg,
                            lockbar_mm: self.lockbar_width_mm,
                        },
                        active.last_head,
                        active.last_pose.as_ref(),
                        active.last_lockbar.as_ref(),
                    );
                    self.screenshot_status = Some(save_rgb_screenshot(
                        &slug,
                        *w,
                        *h,
                        &frame_to_rgb888(pixels, *layout),
                        active.last_pose.as_ref(),
                        active.last_anchor.as_ref(),
                        &meta,
                    ));
                    match &self.screenshot_status {
                        Some(Ok(p)) => info!(path = %p.display(), "screenshot saved"),
                        Some(Err(e)) => error!(error = %e, "screenshot failed"),
                        None => {}
                    }
                }
                // ⟳ window rotation — cycles 0° → 90° → 180° → 270° per
                // click, for a physically rotated pincab display.
                if ui
                    .button(format!("⟳ {}", rotation_label(self.rotation)))
                    .on_hover_text(
                        "Tourner l'affichage de 90° par clic (écran monté de \
                         travers, ou à l'envers / 180°).",
                    )
                    .clicked()
                {
                    self.rotation = next_rotation(self.rotation);
                }
                // Theme: light / dark / follow-system — egui's icon switch.
                egui::global_theme_preference_buttons(ui);
                // Parallax — on/off toggle for the off-axis 3D validation view
                // stacked below the camera feed. A highlight toggle (blue when
                // on), matching the parallax eye-mode selector. The 🪟 glyph
                // renders via the vendored NotoEmoji subset (see
                // `install_extra_glyph_fonts`).
                ui.toggle_value(&mut self.parallax_enabled, "🪟 Parallax")
                    .on_hover_text("Show the off-axis 3D validation scene below the camera feed");
                ui.toggle_value(&mut self.flatten_view, "⊞ Flatten")
                    .on_hover_text(
                        "Redraw the camera view from square-on to the playfield, using the \
                         focal length and the detected rails. Right calibration: the cabinet \
                         stands upright as a rectangle. Wrong focal: it leans over like a \
                         parallelogram — but only if the camera is turned off the cabinet's \
                         axis, the same blind spot as 'square'. Wrong lines: the cabinet's \
                         real edges come out crooked while the detected ones sit straight, \
                         and that shows whatever the camera is aimed at. Everything off the \
                         playfield — you included — smears, because it is being shown from a \
                         viewpoint it was never seen from. Needs the anchor to have locked.",
                    );
                // 🎁 Contribute — same highlight toggle as the others (so it
                // never shifts the bar), with a red outline *painted on top* of
                // its rect as a call to action. Painting the border rather than
                // using Button::stroke keeps it out of the layout entirely, so
                // no widget below it moves on hover/press.
                let contrib = ui
                    .toggle_value(
                        &mut self.contribute_open,
                        RichText::new("🎁 Contribute").strong(),
                    )
                    .on_hover_text("Share a capture to help train the head-tracking model");
                ui.painter().rect_stroke(
                    contrib.rect,
                    egui::CornerRadius::same(3),
                    Stroke::new(2.0, Color32::from_rgb(0xe0, 0x3a, 0x3a)),
                    egui::StrokeKind::Outside,
                );
                if ui
                    .button("⏻ Quit")
                    .on_hover_text("Close the demo (fullscreen has no window buttons)")
                    .clicked()
                {
                    self.should_quit = true;
                }
                // Screenshot result lives on the bar — it's the button's
                // own feedback. Cleared on the next click / backend change.
                if let Some(status) = &self.screenshot_status {
                    match status {
                        Ok(path) => ui
                            .label(
                                RichText::new(format!(
                                    "saved → {}",
                                    path.file_name().and_then(|s| s.to_str()).unwrap_or("(?)")
                                ))
                                .color(Color32::from_rgb(0x90, 0xee, 0x90))
                                .small(),
                            )
                            .on_hover_text(path.display().to_string()),
                        Err(e) => ui.colored_label(Color32::LIGHT_RED, format!("save failed: {e}")),
                    };
                }
            });

            ui.add_space(3.0);
            contribution_banner(ui);

            ui.add_space(2.0);
            // Row 2 — camera INPUT (raw, before maths). `input_line` lays out
            // two rows: device + head measurements, then the anchor / lockbar
            // readout on its own line below.
            ui.vertical(|ui| self.input_line(ui));

            ui.add_space(2.0);
            // Row 3 — camera OUTPUT (after maths → what VPX consumes).
            ui.horizontal(|ui| {
                ui.label(
                    RichText::new("out ▸")
                        .strong()
                        .color(Color32::GRAY)
                        .monospace(),
                );
                self.output_line(ui);
            });

            ui.add_space(2.0);
            // Row 4 — live perf counters: per-model inference (ms), process
            // CPU%, and input (camera) vs output (filtered-pose) frame rates.
            if let Some(active) = self.active.as_ref() {
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new("perf ▸")
                            .strong()
                            .color(Color32::GRAY)
                            .monospace(),
                    );
                    ui.label(
                        RichText::new(active.metrics.summary())
                            .monospace()
                            .color(Color32::LIGHT_BLUE),
                    );
                });
            }
            // Shown once when a device opens: what it can deliver, and the
            // two readings that look like faults and are not.
            if let Some(brief) = &self.brief {
                let mut close = false;
                egui::Modal::new(egui::Id::new("device_brief")).show(ui.ctx(), |ui| {
                    ui.set_max_width(460.0);
                    ui.heading(&brief.title);
                    ui.add_space(6.0);
                    for line in &brief.streams {
                        ui.label(RichText::new(format!("- {line}")).monospace());
                    }
                    ui.add_space(8.0);
                    for n in &brief.notes {
                        ui.label(n);
                        ui.add_space(4.0);
                    }
                    ui.add_space(4.0);
                    if ui.button("Got it").clicked() {
                        close = true;
                    }
                });
                if close {
                    self.brief = None;
                }
            }
            // The same samples as history, with the labels in the header
            // instead of on every line -- and device chatter interleaved, so
            // "it started dropping here, and libfreenect2 said this at the
            // same moment" is one glance rather than two windows. Collapsed
            // by default: it is a diagnostic, not part of the normal view.
            egui::CollapsingHeader::new("Diagnostics table")
                .default_open(false)
                .show(ui, |ui| {
                    egui::ScrollArea::both()
                        .max_height(220.0)
                        .show(ui, perf_table::ui);
                });
            ui.add_space(4.0);
        });

        // ----- Kinect access nudge (Kinect on the bus but no udev rule /
        // ----- WinUSB driver — offer the one-click fix)
        if self.kinect_access_hint || self.kinect_access_result.is_some() {
            let amber = Color32::from_rgb(0xff, 0xc4, 0x40);
            let mut do_fix = false;
            Panel::top("kinect-access").show(ui, |ui| {
                ui.add_space(3.0);
                match &self.kinect_access_result {
                    None => {
                        ui.horizontal_wrapped(|ui| {
                            ui.label(
                                RichText::new("⚠ Kinect plugged in but not accessible")
                                    .color(amber)
                                    .strong(),
                            );
                            ui.label(RichText::new(KINECT_ACCESS_PROBLEM).color(Color32::GRAY));
                            if ui.button(KINECT_ACCESS_BUTTON).clicked() {
                                do_fix = true;
                            }
                        });
                    }
                    Some(Ok(msg)) => {
                        ui.label(
                            RichText::new(format!("✓ {msg}"))
                                .color(Color32::from_rgb(0x6c, 0xc7, 0x6c)),
                        );
                    }
                    Some(Err(detail)) => {
                        ui.label(
                            RichText::new("Couldn't do it automatically:")
                                .color(amber)
                                .strong(),
                        );
                        ui.label(RichText::new(detail).monospace().size(15.0));
                    }
                }
                ui.add_space(3.0);
            });
            if do_fix {
                match fix_kinect_access() {
                    Ok(()) => {
                        info!("Kinect access fix started");
                        self.kinect_access_hint = false;
                        self.kinect_access_result = Some(Ok(KINECT_ACCESS_OK_NOTE.to_string()));
                    }
                    Err(e) => {
                        error!(error = %e, "Kinect access fix failed");
                        self.kinect_access_result = Some(Err(e));
                    }
                }
            }
        }

        // ----- Optional Kinect v1 controls (tilt + LED)
        self.show_v1_controls(ui);

        // ----- Bottom split: logs (left) + VPX delta panel (right)
        Panel::bottom("debug-panels")
            .resizable(true)
            .default_size(220.0)
            .min_size(80.0)
            .show(ui, |ui| {
                ui.add_space(4.0);
                ui.columns(2, |cols| {
                    // Left: tracing event log
                    cols[0].horizontal(|ui| {
                        ui.label(RichText::new("logs").strong());
                        if ui.small_button("Clear").clicked() {
                            self.logs.lock().clear();
                        }
                    });
                    ScrollArea::vertical()
                        .id_salt("log-scroll")
                        .auto_shrink([false; 2])
                        .stick_to_bottom(true)
                        .show(&mut cols[0], |ui| {
                            let logs = self.logs.lock();
                            ui.with_layout(Layout::top_down(Align::LEFT), |ui| {
                                for line in logs.iter() {
                                    ui.label(RichText::new(line).monospace().size(15.0));
                                }
                            });
                        });

                    // Right: VPX delta panel
                    let mut reset_baseline = false;
                    cols[1].horizontal(|ui| {
                        ui.label(RichText::new("VPX output (Δ view)").strong());
                        if ui.small_button("Reset baseline").clicked() {
                            reset_baseline = true;
                        }
                    });
                    cols[1].add_space(2.0);
                    if reset_baseline && let Some(active) = self.active.as_mut() {
                        // Clear the capture thread's baseline (it owns the pose
                        // filter) as well as our local copy.
                        let _ = active.worker.cmd_tx.send(CaptureCmd::ResetBaseline);
                        active.baseline = None;
                    }
                    if let Some(active) = self.active.as_ref() {
                        if !active.backend.has_head_tracker() {
                            cols[1].label(
                                RichText::new(
                                    "this input has no head tracker yet\n\
                                     (head detection / monocular depth comes\n\
                                     with the webcam tracker — P3 roadmap)",
                                )
                                .color(Color32::GRAY)
                                .monospace(),
                            );
                            return;
                        }
                        match (active.baseline, active.last_head) {
                            (Some(base), Some(head)) => {
                                let dx_mm = head.x_mm - base.x_mm;
                                let dy_mm = head.y_mm - base.y_mm;
                                let dz_mm = head.depth_mm - base.z_mm;
                                let (vx, vy, vz) =
                                    pose_delta_to_view_delta_vpu(dx_mm, dy_mm, dz_mm);
                                cols[1].label(
                                    RichText::new({
                                        let u = self.pov_unit;
                                        let d = u.pose_decimals();
                                        let c = |v: f32| u.from_mm(v);
                                        format!(
                                            "baseline ({unit:<4}) ({:>7.d$}, {:>7.d$}, {:>7.d$})\n\
                                             current  ({unit:<4}) ({:>7.d$}, {:>7.d$}, {:>7.d$})\n\
                                             Δ pose   ({unit:<4}) ({:>+7.d$}, {:>+7.d$}, {:>+7.d$})",
                                            c(base.x_mm),
                                            c(base.y_mm),
                                            c(base.z_mm),
                                            c(head.x_mm),
                                            c(head.y_mm),
                                            c(head.depth_mm),
                                            c(dx_mm),
                                            c(dy_mm),
                                            c(dz_mm),
                                            unit = u.label(),
                                            d = d,
                                        )
                                    })
                                    .monospace()
                                    .size(15.0),
                                );
                                cols[1].add_space(6.0);
                                // Which focal the distance was worked out with,
                                // and where it came from. On the colour stream a
                                // Kinect is deliberately treated as a webcam, so
                                // it uses the nominal guess rather than its own
                                // factory intrinsics -- that is what makes the
                                // comparison with the infrared reading mean
                                // something, and it is worth saying out loud
                                // rather than leaving the reader to wonder which
                                // number produced the millimetres above.
                                let on_cam = self.selected_stream != StreamKind::Ir;
                                let fw = active.last_frame_w.max(1);
                                let (fx, how) = if on_cam {
                                    (
                                        fw as f32 * WEBCAM_FX_PER_WIDTH,
                                        "assumed from frame width, as a webcam must",
                                    )
                                } else {
                                    (color_focal_px(active.backend, fw), "the sensor's own")
                                };
                                cols[1].label(
                                    RichText::new(format!("focal: {fx:>6.0} px  ({how})"))
                                        .monospace()
                                        .size(13.0)
                                        .color(Color32::GRAY),
                                );
                                cols[1].add_space(4.0);
                                cols[1].horizontal(|ui| {
                                    ui.spacing_mut().item_spacing.x = 4.0;
                                    ui.label(
                                        RichText::new("units")
                                            .monospace()
                                            .size(12.0)
                                            .color(Color32::DARK_GRAY),
                                    );
                                    for u in PovUnit::ALL {
                                        if ui
                                            .selectable_label(
                                                self.pov_unit == u,
                                                RichText::new(u.label()).monospace().size(12.0),
                                            )
                                            .clicked()
                                        {
                                            self.pov_unit = u;
                                        }
                                    }
                                });
                                // The point of view the plugin would send to
                                // VPX, one line per axis. Named rather than
                                // numbered because the mapping is not obvious
                                // from the letters alone, and coloured so a
                                // figure that runs away is spotted without
                                // reading it: this is the readout that says
                                // whether the tracking is producing something
                                // sane.
                                //
                                // These are in the PLAYER's frame, not the
                                // camera's -- the mirror between the two is
                                // absorbed upstream. "left" here is the player's
                                // left, which is the right of the image on
                                // screen, so the header says so rather than
                                // leaving it to be discovered.
                                cols[1]
                                    .label(
                                        RichText::new("point of view — player's frame")
                                            .monospace()
                                            .size(13.0)
                                            .color(Color32::GRAY),
                                    )
                                    .on_hover_text(
                                        "VPU are Visual Pinball's own units, which is what \
                                         the plugin sends: 50 VPU = 1.0625 inch = 26.99 mm. \
                                         So a whole VPU is about half a millimetre of head \
                                         movement -- the figures are small on purpose.",
                                    );
                                for (name, sense, v, col) in [
                                    ("x", "high/low", vx, POV_X),
                                    ("y", "left/right", vy, POV_Y),
                                    ("z", "near/far", vz, POV_Z),
                                ] {
                                    cols[1].label(
                                        RichText::new(format!(
                                            "{name} ({sense}): {:>+8.*} {}",
                                            self.pov_unit.decimals(),
                                            self.pov_unit.from_vpu(v),
                                            self.pov_unit.label(),
                                        ))
                                        .monospace()
                                        .size(15.0)
                                        .color(col),
                                    );
                                }
                            }
                            _ => {
                                cols[1].label(
                                    RichText::new("waiting for baseline…")
                                        .color(Color32::GRAY)
                                        .monospace(),
                                );
                            }
                        }
                    } else {
                        cols[1].label(
                            RichText::new("input is off")
                                .color(Color32::GRAY)
                                .monospace(),
                        );
                    }
                });
            });

        // ----- Center: camera feed, with the parallax scene stacked BELOW it
        // (a resizable bottom panel) when enabled — side-by-side ate too much
        // width.
        CentralPanel::default().show(ui, |ui| {
            self.camera_placement_help(ui);
            ui.separator();
            if self.parallax_enabled {
                // Fixed 50/50 split between the camera feed (top) and the
                // parallax scene (bottom) of the area left below the placement
                // strip — not user-resizable.
                let half = ui.available_height() * 0.5;
                Panel::bottom("parallax-view")
                    .resizable(false)
                    .exact_size(half)
                    .show(ui, |ui| self.draw_parallax_view(ui));
            }
            self.stream_bar(ui);
            self.draw_camera_view(ui);
        });
        let ctx = ui.ctx().clone();
        self.contribute_window(&ctx);
        self.usb_window(&ctx);
    }

    /// Lockbar-width field — the metric ruler for the monocular (webcam)
    /// scale. Shown only for a webcam input: depth cameras measure scale
    /// directly, so they don't need it (there it's just a cross-check). mm
    /// with an inch read-out — the international inch is exactly 25.4 mm.
    /// Filters / tuning row — everything on one line as table-like grouped
    /// cells: the 1€ **stability filter** (two plain-language knobs, no jargon)
    /// plus the two cabinet-geometry inputs (lockbar width in **cm**, playfield/
    /// backglass inclination in **°**).
    fn filters_row(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            // Cell — stability filter (1€). The two knobs are the 1€ min-cutoff
            // and beta, relabelled for humans; hover text explains the effect.
            egui::Frame::group(ui.style()).show(ui, |ui| {
                ui.label(RichText::new("Stability filter (1€)").strong());
                ui.separator();
                ui.add(
                    egui::Slider::new(&mut self.head_filter_min_cutoff, 0.1..=5.0)
                        .text("Responsiveness"),
                )
                .on_hover_text(
                    "While you stand still: left = very stable and smooth (a slight \
                     delay), right = follows faster (may tremble a little).",
                );
                ui.add(
                    egui::Slider::new(&mut self.head_filter_beta, 0.0..=1.5)
                        .text("Motion catch-up"),
                )
                .on_hover_text(
                    "When you move fast: further right = catches up quicker, the \
                     view sticks to your head.",
                );
                ui.add(
                    egui::Slider::new(&mut self.median_window_frames, 1..=9)
                        .step_by(2.0)
                        .text("Median (frames)"),
                )
                .on_hover_text(
                    "Median window applied before the smoothing filter: erases isolated \
                     tracking spikes completely. 1 = off; each extra frame adds one frame \
                     of latency.",
                );
                ui.toggle_value(&mut self.bypass_filters, "no filter")
                    .on_hover_text(
                        "Disables the 1€ smoothing (+ picker scoring + depth gate) — debug.",
                    );
            });

            // Cell — lockbar width in cm (scale reference = distance between the
            // two sidebars). Always shown. Stored in mm internally.
            egui::Frame::group(ui.style()).show(ui, |ui| {
                ui.label("Lockbar width");
                let mut cm = self.lockbar_width_mm / 10.0;
                if ui
                    .add(egui::Slider::new(&mut cm, 20.0..=120.0).suffix(" cm"))
                    .on_hover_text("Width between the two side rails (the scale reference).")
                    .changed()
                {
                    self.lockbar_width_mm = cm * 10.0;
                }
            });

            // Cell — playfield / backglass inclination in degrees (all backends).
            egui::Frame::group(ui.style()).show(ui, |ui| {
                ui.label("Playfield incline");
                ui.add(egui::Slider::new(&mut self.table_incl_deg, 0.0..=30.0).suffix(" °"))
                    .on_hover_text(
                        "Playfield angle vs horizontal (VPX provides it per table). The \
                         parallax tilts the head motion by 90° minus this angle.",
                    );
            });

            // Cell — parallax bench: gain + axis sign flips.
            egui::Frame::group(ui.style()).show(ui, |ui| {
                ui.label("Parallax");
                ui.add(egui::Slider::new(&mut self.parallax_gain, 0.5..=6.0).text("gain"))
                    .on_hover_text("Amplification of head movement into the POV.");
                // Name the axis, not just its letter: while dragging you need
                // to know which way the cab is going to move.
                ui.toggle_value(&mut self.parallax_invert[0], "±X left/right")
                    .on_hover_text("Flip the left/right axis.");
                ui.toggle_value(&mut self.parallax_invert[1], "±Y up/down")
                    .on_hover_text("Flip the up/down axis.");
                ui.toggle_value(&mut self.parallax_invert[2], "±Z near/far")
                    .on_hover_text("Flip the depth axis (closer/farther).");
            });
        });
    }

    /// Camera-placement guidance shown at the top of the central panel, above
    /// the live view: a one-line reminder + three example thumbnails (a good
    /// frame, and two mounting shots). Useful to everyone, not just
    /// contributors, so it lives in the main UI rather than the share window.
    fn camera_placement_help(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        // Order: the two mounting shots first, then what the camera should see.
        let thumbs = self.contrib_thumbs.get_or_insert_with(|| {
            [
                load_thumb(
                    &ctx,
                    "ht_setup_cab",
                    include_bytes!("../assets/setup_cab.jpg"),
                ),
                load_thumb(
                    &ctx,
                    "ht_setup_bg",
                    include_bytes!("../assets/setup_bg.jpg"),
                ),
                load_thumb(
                    &ctx,
                    "ht_cam_view",
                    include_bytes!("../assets/cam_view.jpg"),
                ),
            ]
        });
        const CAPTIONS: [&str; 3] = [
            "Camera on the pincab",
            "Camera on the backglass / topper",
            "View from the camera (sidebars, lockbar, face)",
        ];
        ui.add_space(2.0);
        ui.label(
            "Camera placement: top of the backglass or topper, centred, facing the \
             playfield and the player. A good frame shows the lockbar, a bit of the \
             sidebars, and the head.",
        );
        ui.add_space(3.0);
        // One column per thumbnail: the image fills the whole column width,
        // its caption bold + centred underneath.
        ui.columns(3, |cols| {
            for (i, tex) in thumbs.iter().enumerate() {
                let w = cols[i].available_width();
                show_thumb(&mut cols[i], tex, w);
                cols[i].add_space(2.0);
                cols[i].vertical_centered(|ui| {
                    ui.label(RichText::new(CAPTIONS[i]).strong());
                });
            }
        });
    }

    /// Ask the drop whether it is reachable, off the UI thread.
    ///
    /// One HEAD request; the panel shows "checking…" meanwhile and the button
    /// stays disabled. Re-runnable from the panel, because the answer is about
    /// the network and the network changes.
    fn start_drop_probe(&mut self) {
        if matches!(self.drop_reach, ReachState::Checking) {
            return;
        }
        let (tx, rx) = mpsc::channel();
        if std::thread::Builder::new()
            .name("drop-probe".into())
            .spawn(move || {
                let reach = contribute::probe();
                info!(?reach, "contribution drop reachability");
                let _ = tx.send(reach);
            })
            .is_ok()
        {
            self.drop_reach = ReachState::Checking;
            self.drop_probe = Some(rx);
        }
    }

    /// Collect the probe's answer if it has landed. Never blocks.
    fn poll_drop_probe(&mut self) {
        let Some(rx) = &self.drop_probe else {
            return;
        };
        match rx.try_recv() {
            Ok(reach) => {
                self.drop_reach = ReachState::Known(reach);
                self.drop_probe = None;
            }
            Err(mpsc::TryRecvError::Disconnected) => {
                // The thread died without answering: treat as unreachable
                // rather than leaving the panel spinning forever.
                self.drop_reach = ReachState::Known(contribute::Reach::Unreachable(
                    "the check did not complete".into(),
                ));
                self.drop_probe = None;
            }
            Err(mpsc::TryRecvError::Empty) => {}
        }
    }

    /// The "Share a capture" window: the informed-consent notice + checkbox,
    /// the share button (gated on consent + a live frame), upload status, and
    /// a short capture reminder. All demo-only — the plugin has none of this.
    fn contribute_window(&mut self, ctx: &egui::Context) {
        // The reachability answer is needed before the capture, not after it,
        // so the check starts the moment the panel opens and is re-read on
        // every frame it stays open.
        self.poll_drop_probe();
        if matches!(self.drop_reach, ReachState::Unknown) {
            self.start_drop_probe();
        }
        let mut open = self.contribute_open;
        egui::Window::new("🎁 Share a capture")
            .open(&mut open)
            .resizable(true)
            .default_width(500.0)
            .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
            .show(ctx, |ui| {
                ui.label(RichText::new("Please read before accepting").strong());
                ui.add_space(4.0);
                egui::ScrollArea::vertical()
                    .max_height(420.0)
                    .show(ui, |ui| {
                        ui.label(
                            "By sharing, you upload images from your tracking camera (each \
                             capture = the raw image + the detection). These images may show \
                             places, people, and their faces.",
                        );
                        ui.add_space(4.0);
                        for (title, body) in CONTRIB_TERMS {
                            ui.horizontal_wrapped(|ui| {
                                ui.spacing_mut().item_spacing.x = 3.0;
                                ui.label(RichText::new(format!("{title}:")).strong());
                                ui.label(*body);
                            });
                        }
                    });
                ui.add_space(6.0);
                // Make the click-to-authorise action unmistakable: a call-out
                // above the toggle so it's obvious the upload is gated on it.
                ui.label(
                    RichText::new("👉 Click the box below to authorise sending your image:")
                        .strong()
                        .color(Color32::from_rgb(0xff, 0xcc, 0x33)),
                );
                // A real checkbox, not a toggle carrying a `☐` in its label.
                // That glyph was hard-coded, so the box never filled in however
                // many times it was clicked — reported from the field as "the
                // checkbox is always empty". And neither embedded font subset
                // even contains ☐/☑, so the character was at the mercy of
                // whatever egui's default font happened to provide.
                //
                // `Checkbox` draws its box and its tick as vector shapes: no
                // font involved, it ticks itself, and the whole sentence stays
                // clickable — which is what makes it obvious the block is the
                // thing to click.
                ui.checkbox(
                    &mut self.consent_checked,
                    "I have read the above and I freely give my informed consent to share \
                     these images under these terms.",
                );
                ui.add_space(6.0);
                let has_frame = self
                    .active
                    .as_ref()
                    .is_some_and(|a| a.last_rgb_frame.is_some());
                // Can this machine even reach the drop? Asked here, before a
                // capture is taken, because the alternative is what actually
                // happened to a contributor: 35 files shared, no error shown,
                // nothing received.
                let can_upload = self.drop_reach.allows_upload();
                ui.horizontal_wrapped(|ui| {
                    ui.spacing_mut().item_spacing.x = 4.0;
                    match &self.drop_reach {
                        ReachState::Unknown | ReachState::Checking => {
                            ui.spinner();
                            ui.label("checking that this machine can reach the capture server…");
                        }
                        ReachState::Known(reach) if reach.is_up() => {
                            ui.label(RichText::new("✓ capture server reachable").color(COLOR_OK));
                        }
                        ReachState::Known(reach) => {
                            ui.label(
                                RichText::new(format!(
                                    "✖ upload unavailable — {}",
                                    reach.explain()
                                ))
                                .color(COLOR_BAD)
                                .strong(),
                            );
                        }
                    }
                    if !matches!(self.drop_reach, ReachState::Checking)
                        && ui.small_button("↻ check again").clicked()
                    {
                        self.start_drop_probe();
                    }
                });
                ui.add_space(4.0);
                let ready = self.consent_checked && has_frame;
                // With no route to the server the button does not vanish: it
                // becomes a save. A capture that exists on disk can still be
                // handed over; a capture never taken is gone for good.
                let label = if can_upload {
                    "📸 Share this capture"
                } else {
                    "💾 Save this capture to send by hand"
                };
                if ui.add_enabled(ready, egui::Button::new(label)).clicked() {
                    self.share_capture();
                }
                if !has_frame {
                    ui.label(RichText::new("(select a device and wait for the feed first)").weak());
                }
                if !can_upload && !matches!(self.drop_reach, ReachState::Checking) {
                    ui.label(
                        "Your capture will be written to a folder on this machine, and you can \
                         send it to us on Discord — it is worth just as much as an upload.",
                    );
                    ui.hyperlink_to(
                        "Join the Discord to hand it over",
                        contribute::DISCORD_INVITE,
                    );
                }
                // Upload status — per share, so one failure inside a set of
                // seven cannot hide behind a green running total.
                let st = self.uploader.status();
                if st.pending > 0 {
                    ui.label(format!("uploading… {} file(s) pending", st.pending));
                    ui.label(
                        RichText::new("Please leave this window open until it reaches zero.")
                            .weak(),
                    );
                }
                if st.uploaded > 0 && !st.has_failure() {
                    ui.label(
                        RichText::new(format!("✓ {} file(s) uploaded", st.uploaded))
                            .color(COLOR_OK),
                    );
                }
                if st.has_failure() {
                    ui.add_space(4.0);
                    egui::Frame::group(ui.style())
                        .fill(Color32::from_rgb(0x3a, 0x12, 0x12))
                        .show(ui, |ui| {
                            ui.label(
                                RichText::new(format!(
                                    "✖ UPLOAD FAILED — {} file(s) did not reach us{}",
                                    st.failed,
                                    if st.uploaded > 0 {
                                        format!(" ({} did)", st.uploaded)
                                    } else {
                                        String::new()
                                    }
                                ))
                                .color(COLOR_BAD)
                                .strong()
                                .size(16.0),
                            );
                            if let Some(err) = &st.last_error {
                                ui.label(RichText::new(err.as_str()).monospace().color(COLOR_BAD));
                            }
                            match (&st.rescued_in, &st.rescue_error) {
                                (_, Some(e)) => {
                                    ui.label(
                                        RichText::new(format!(
                                            "and the copy we tried to keep could not be written \
                                             either — {e}"
                                        ))
                                        .color(COLOR_BAD),
                                    );
                                }
                                (Some(dir), None) => {
                                    ui.label("Your capture was kept here:");
                                    ui.monospace(dir.display().to_string());
                                    ui.label(
                                        "Nothing is lost — drop those files on our Discord and \
                                         they go straight into the training set.",
                                    );
                                    ui.hyperlink_to(
                                        "Join the Discord to hand them over",
                                        contribute::DISCORD_INVITE,
                                    );
                                }
                                (None, None) => {}
                            }
                        });
                }
                // The removal ID only means something for a capture that
                // actually reached us — offering it after a save-only run
                // would be telling the contributor we have something we don't.
                if let Some(stem) = &self.contrib_last
                    && st.uploaded > 0
                {
                    ui.add_space(4.0);
                    ui.label("Shared — note this if you may want it removed:");
                    ui.monospace(format!("{stem}_raw.png · {stem}_det.png"));
                }
                ui.add_space(4.0);
                // Your own copy: never a promise, always the checked answer —
                // the folder a share really reached, the folder chosen for the
                // next one, or a plain "none".
                if let Some(err) = &self.contrib_save_error {
                    ui.label(
                        RichText::new(format!("⚠ your copy could not be written — {err}"))
                            .color(Color32::from_rgb(0xff, 0x99, 0x66)),
                    );
                }
                ui.horizontal_wrapped(|ui| {
                    ui.spacing_mut().item_spacing.x = 4.0;
                    match (&self.contrib_saved_in, &self.contrib_local) {
                        (Some(dir), _) => {
                            ui.label("Your copy was saved in:");
                            ui.monospace(dir.display().to_string());
                        }
                        (None, LocalCopy::Folder(dir)) => {
                            ui.label("Your copy will be saved in:");
                            ui.monospace(dir.display().to_string());
                        }
                        (None, LocalCopy::Declined) => {
                            ui.label("Your copy: none kept — upload only.");
                        }
                        (None, LocalCopy::Unasked) => {
                            ui.label("Your copy: you'll be asked for a folder when you share.");
                        }
                    }
                    if ui
                        .button("📁 Choose a folder…")
                        .on_hover_text(
                            "Where to keep your own copy of each capture. Cancelling the \
                             picker means the capture is only uploaded.",
                        )
                        .clicked()
                    {
                        self.contrib_local = match ask_local_copy_folder() {
                            Some(dir) => LocalCopy::Folder(dir),
                            None => LocalCopy::Declined,
                        };
                    }
                });
                ui.separator();
                ui.label(RichText::new("Before you capture").strong());
                ui.label(
                    "Make sure your camera is well placed — see the placement guide above \
                     the camera view.",
                );
                ui.add_space(4.0);
                ui.label(RichText::new("Help us with variety:").strong());
                ui.label(
                    "different lighting, colours and brightness; with and without a player; \
                     the lockbar clear and with hands on it.",
                );
            });
        self.contribute_open = open;
    }

    /// Stream bar, directly above the camera image: one chip per stream the
    /// device offers, captioned with its nominal spec, **green when frames are
    /// arriving and red when they aren't**, and clickable to display it.
    ///
    /// This is where the Kinect v1's single video endpoint explains itself:
    /// select IR and the colour chip goes red while IR goes green, so the
    /// either/or is visible rather than documented.
    fn stream_bar(&mut self, ui: &mut egui::Ui) {
        let Some(active) = self.active.as_mut() else {
            return;
        };
        let specs = stream_specs(active.backend, active.cam_spec);
        if specs.is_empty() {
            return;
        }
        // The frame budget is one frame at the fastest rate this device
        // offers: that is the cadence the pipeline has to keep up with.
        if let Some(fps) = specs.iter().map(|s| s.fps).max() {
            active.metrics.set_nominal_fps(fps as f32);
        }
        let active = &*active;
        // Aged here, at draw time — see [`Active::last_rgb_at`].
        let live = |t: Option<Instant>| t.is_some_and(|t| t.elapsed() < STREAM_LIVE_FOR);
        let (rgb_live, ir_live) = (live(active.last_rgb_at), live(active.last_ir_at));
        let mut pick: Option<StreamKind> = None;
        ui.horizontal_wrapped(|ui| {
            ui.label(
                RichText::new("streams")
                    .strong()
                    .color(Color32::GRAY)
                    .monospace(),
            );
            ui.separator();
            for spec in &specs {
                let live = match spec.kind {
                    // Both colour modes ride the same endpoint, so the same
                    // liveness answers for either.
                    StreamKind::Rgb | StreamKind::RgbHigh => rgb_live,
                    StreamKind::Ir => ir_live,
                };
                let colour = if live {
                    Color32::from_rgb(60, 200, 90)
                } else {
                    Color32::from_rgb(220, 70, 70)
                };
                let selected = self.selected_stream == spec.kind;
                let text = RichText::new(spec.caption()).monospace().color(colour);
                let text = if selected { text.strong() } else { text };
                let hint = if live {
                    "receiving — click to display"
                } else {
                    "not receiving (this device can't stream it alongside the current one)"
                };
                if ui
                    .selectable_label(selected, text)
                    .on_hover_text(hint)
                    .clicked()
                {
                    pick = Some(spec.kind);
                }
            }
        });
        if let Some(kind) = pick
            && kind != self.selected_stream
        {
            // Switching between infrared and colour changes which model the
            // anchor runs and which image it sees, so the detection from the
            // previous stream means nothing now: start it over rather than
            // leave a frozen result that came from the other camera.
            let was_ir = self.selected_stream == StreamKind::Ir;
            let now_ir = kind == StreamKind::Ir;
            self.selected_stream = kind;
            if was_ir != now_ir
                && let Some(active) = self.active.as_ref()
            {
                let _ = active.worker.cmd_tx.send(CaptureCmd::Recalibrate);
            }
            // The v1 has to physically switch its video endpoint; the others
            // just need to know what's on screen.
            if let Some(active) = self.active.as_ref() {
                let _ = active.worker.cmd_tx.send(CaptureCmd::SelectStream(kind));
            }
        }
        ui.add_space(2.0);
        self.what_the_plugin_uses(ui);
        self.color_exposure_bar(ui);
    }

    /// The USB window: what this sensor has, what actually matters about it,
    /// and the whole bus tree.
    ///
    /// Opened from the toolbar badge, never on its own. The automatic version
    /// of this was an amber warning whenever anything else shared the sensor's
    /// controller — which on a motherboard with a single controller is always,
    /// and is fine: the v2 peaks near 2 Gbit/s of the 5 that controller
    /// carries. It told an owner his hardware was wrong when it was not, and
    /// sent him looking for a second controller his board does not have.
    ///
    /// A plain window rather than a modal, because the useful thing to do
    /// while reading it is unplug something and watch the list change.
    fn usb_window(&mut self, ctx: &egui::Context) {
        if !self.usb_window_open {
            return;
        }
        let sensor = match self.selected {
            Backend::KinectV1 => Some(usb_check::Sensor::KinectV1),
            Backend::KinectV2 => Some(usb_check::Sensor::KinectV2),
            _ => None,
        };
        if let Some(s) = sensor {
            self.poll_usb_probe(s);
        }
        // Whatever the last worker produced. Nothing here reads the bus.
        let snap = self
            .usb_cache
            .as_ref()
            .filter(|(s, _)| Some(*s) == sensor)
            .map(|(_, snap)| snap);
        let report = snap.and_then(|s| s.report.clone());
        let tree: Vec<usb_check::BusNode> = snap.map(|s| s.tree.clone()).unwrap_or_default();
        let scanning = self.usb_probe.is_some();
        let mut open = self.usb_window_open;
        let mut refresh = false;
        egui::Window::new("USB")
            .open(&mut open)
            .default_width(560.0)
            .show(ctx, |ui| {
                if let Some(r) = &report {
                    ui.label(RichText::new(format!("wants: {}", r.want)).monospace());
                    ui.label(
                        RichText::new(format!("has:   {}", r.got))
                            .monospace()
                            .color(r.level.colour()),
                    );
                    ui.add_space(6.0);
                    for note in &r.notes {
                        ui.label(RichText::new(note).color(Color32::GRAY));
                        ui.add_space(3.0);
                    }
                }
                ui.separator();
                let (buses, devices) = usb_check::counts(&tree);
                ui.label(
                    RichText::new(format!(
                        "This machine reports {buses} USB bus(es) and {devices} device(s)."
                    ))
                    .strong(),
                );
                ui.label(
                    RichText::new(
                        "Buses, not controllers: one controller presents two of them, a \
                         USB 2.0 bus and a SuperSpeed bus, because they run on separate \
                         wires with separate schedules. So the count above is roughly twice \
                         the number of chips on your board — and plenty of recent boards \
                         (USB 3.2, DDR5, the lot) have exactly one, with every socket on \
                         the back panel behind it. Dedicating a controller to the Kinect is \
                         then simply not possible, and that is fine.",
                    )
                    .color(Color32::GRAY),
                );
                ui.add_space(6.0);
                ui.label(
                    RichText::new(
                        "Everything below one heading shares that bus's reserved budget — but \
                         only while it streams. A camera reserves its bandwidth up front, \
                         for as long as it is running, and hands it back when it stops; a \
                         keyboard, a mouse or a cabinet I/O board reserve almost nothing at \
                         any time. The Kinect v2 claims about 2 Gbit/s of the 5 a USB 3.0 \
                         controller carries, so it has room to spare next to ordinary \
                         devices.",
                    )
                    .color(Color32::GRAY),
                );
                ui.add_space(4.0);
                ui.label(
                    RichText::new(
                        "Two cameras on one bus is the combination to avoid, and it \
                         does not show up as lag: if both reservations do not fit, the \
                         second one is refused and that camera simply never starts. If a \
                         webcam and the Kinect are under the same heading below, that is \
                         worth changing.",
                    )
                    .color(Color32::GRAY),
                );
                if tree
                    .iter()
                    .any(|n| n.depth == 0 && n.bulk_storage && n.demand_mbit > 0)
                {
                    ui.add_space(4.0);
                    ui.label(
                        RichText::new(
                            "A drive shares a bus with a camera below, marked `bulk`. That \
                             is not a reservation and it costs the camera nothing: bulk \
                             transfers are served with whatever the streams leave behind, \
                             so a disk can never stop a camera from starting. It is the \
                             one device here that is silent until you use it — worth \
                             knowing if a long copy happens to run during a game.",
                        )
                        .color(Color32::GRAY),
                    );
                }
                ui.add_space(6.0);
                if tree.iter().any(|n| n.sensor_underspeed) {
                    ui.add_space(4.0);
                    ui.label(
                        RichText::new(
                            "⚠ This sensor is connected below the speed it needs — the red \
                             line below. Move it to a rear USB 3.0 port straight on the \
                             motherboard; front panels and hubs often fall back.",
                        )
                        .color(COLOR_BAD)
                        .strong(),
                    );
                }
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    if ui
                        .add_enabled(!scanning, egui::Button::new("↻ Rescan the bus"))
                        .clicked()
                    {
                        refresh = true;
                    }
                    if scanning {
                        ui.spinner();
                        ui.label(RichText::new("reading the bus…").color(Color32::GRAY));
                    }
                });
                ui.add_space(4.0);
                // Each entry is written in the colour it describes, so the
                // legend is its own swatch. Fixed rather than built from
                // what happens to be on screen: a legend that changes shape
                // between two openings is harder to trust than a long one.
                ui.horizontal_wrapped(|ui| {
                    ui.spacing_mut().item_spacing.x = 4.0;
                    let entry = |ui: &mut egui::Ui, colour: Color32, text: &str| {
                        ui.label(RichText::new(text).monospace().color(colour));
                    };
                    entry(ui, COLOR_OK, "this sensor");
                    ui.label(RichText::new("·").color(Color32::DARK_GRAY));
                    entry(ui, COLOR_BAD, "too slow for it  ·  bus over budget");
                    ui.label(RichText::new("·").color(Color32::DARK_GRAY));
                    entry(ui, COLOR_RESERVES, "reserves bandwidth  ·  bus almost full");
                    ui.label(RichText::new("·").color(Color32::DARK_GRAY));
                    entry(ui, Color32::GRAY, "everything else");
                });
                ui.add_space(4.0);
                egui::ScrollArea::both().max_height(320.0).show(ui, |ui| {
                    if tree.is_empty() && !scanning {
                        ui.label(
                            RichText::new("The bus could not be enumerated on this system.")
                                .color(Color32::GRAY),
                        );
                    }
                    // One column for the rate, so a glance down the list
                    // compares like with like. Width from the longest line
                    // actually present rather than a guessed constant.
                    let width = tree
                        .iter()
                        .map(|n| n.label.chars().count() + if n.depth == 0 { 0 } else { 2 + 4 })
                        .max()
                        .unwrap_or(0);
                    for node in &tree {
                        let (prefix, colour) = if node.depth == 0 {
                            (
                                String::new(),
                                if node.sensor_underspeed || node.over_budget() {
                                    COLOR_BAD
                                } else if node.tight() {
                                    COLOR_RESERVES
                                } else {
                                    Color32::LIGHT_GRAY
                                },
                            )
                        } else {
                            // The arrow carries the nesting; a hub adds a
                            // level of indent under it.
                            let indent = "  ".repeat(node.depth);
                            let colour = if node.sensor_underspeed {
                                COLOR_BAD
                            } else if node.is_sensor {
                                COLOR_OK
                            } else if node.demand_mbit > 0 {
                                COLOR_RESERVES
                            } else {
                                Color32::GRAY
                            };
                            (format!("{indent}|-> "), colour)
                        };
                        let left = format!("{prefix}{}", node.label);
                        let pad = width.saturating_sub(left.chars().count());
                        // A bus heading that carries claims says how much of
                        // its budget they take; a device says what it claims.
                        // A drive says `bulk` instead of a figure: its claim
                        // really is nothing, and an empty column there would
                        // read the same as a keyboard's.
                        let claim = if node.depth == 0 && node.demand_mbit > 0 {
                            format!("  {} / {} Mbit", node.demand_mbit, node.budget_mbit)
                        } else if node.depth > 0 && node.demand_mbit > 0 {
                            format!("  {} Mbit", node.demand_mbit)
                        } else if node.depth > 0 && node.bulk_storage {
                            "  bulk".to_string()
                        } else {
                            String::new()
                        };
                        let line = format!("{left}{:pad$}  [{}]{claim}", "", node.rate);
                        let text = RichText::new(line).monospace().color(colour);
                        let text = if node.depth == 0 || node.is_sensor {
                            text.strong()
                        } else {
                            text
                        };
                        ui.label(text);
                    }
                });
            });
        if refresh && let Some(s) = sensor {
            self.start_usb_probe(s);
        }
        self.usb_window_open = open;
    }

    /// What the VPX plugin actually reads, said once and plainly.
    ///
    /// The demo shows colour first because it is the stream a human can judge,
    /// and testers reasonably conclude that colour is what the tracking runs
    /// on — then spend their effort lighting a room for a camera the plugin
    /// does not look at. On a Kinect the head is found in the INFRARED matrix
    /// and its distance read from the DEPTH matrix, with no setting to change
    /// that: the sensor carries its own illuminator, so it uses it. A webcam
    /// tracks on colour because colour is all it has.
    fn what_the_plugin_uses(&self, ui: &mut egui::Ui) {
        let Some(active) = self.active.as_ref() else {
            return;
        };
        if !matches!(active.backend, Backend::KinectV1 | Backend::KinectV2) {
            return;
        }
        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing.x = 3.0;
            ui.label(RichText::new("ℹ").color(Color32::from_rgb(0x66, 0xcc, 0xff)));
            ui.label(
                RichText::new(
                    "On a Kinect, the VPX plugin finds the head in the INFRARED image and \
                     reads its distance from the DEPTH image — always, with no setting to \
                     change it. Colour is preview and capture material. A webcam has no \
                     infrared, so it tracks on colour because that is all it has.",
                )
                .color(Color32::GRAY),
            );
        })
        .response
        .on_hover_text(
            "This is why tracking keeps working in the dark: the sensor lights the scene \
             itself in infrared, so it holds ~30 Hz where the colour camera halves to 15. \
             Depth gives the distance in both modes — the choice only decides which image \
             the head is found in.",
        );
        ui.add_space(2.0);
    }

    /// Colour-camera exposure controls, on the v2 while colour is on screen.
    ///
    /// Deliberately not offered for infrared or depth: libfreenect2 has no
    /// such knob for them, the firmware runs their integration itself, and a
    /// slider that silently does nothing is worse than no slider. Nothing here
    /// changes what the plugin tracks on — it changes what you see, and what a
    /// shared capture will contain.
    fn color_exposure_bar(&mut self, ui: &mut egui::Ui) {
        let Some(active) = self.active.as_ref() else {
            return;
        };
        if active.backend != Backend::KinectV2
            || !matches!(self.selected_stream, StreamKind::Rgb | StreamKind::RgbHigh)
        {
            return;
        }
        let before = self.color_exposure;
        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing.x = 6.0;
            ui.label(
                RichText::new("colour exposure")
                    .strong()
                    .color(Color32::GRAY)
                    .monospace(),
            )
            .on_hover_text("Colour camera only — infrared and depth are driven by the firmware.");
            ui.separator();

            let mut mode = self.color_exposure;
            // Radio over a combo: three options, and the choice governs which
            // sliders make sense below it.
            if ui
                .selectable_label(matches!(mode, ColorExposureMode::Auto { .. }), "auto")
                .on_hover_text("The camera decides. How it opens.")
                .clicked()
            {
                mode = ColorExposureMode::Auto { compensation: 0.0 };
            }
            if ui
                .selectable_label(
                    matches!(mode, ColorExposureMode::SemiAuto { .. }),
                    "flicker-free",
                )
                .on_hover_text(
                    "Rounds the shutter down to a whole mains-light period (10 ms at 50 Hz, \
                     8.3 at 60) and raises gain to compensate — kills the banding under \
                     fluorescent and LED lighting.",
                )
                .clicked()
            {
                mode = ColorExposureMode::SemiAuto { pseudo_ms: 16.0 };
            }
            if ui
                .selectable_label(matches!(mode, ColorExposureMode::Manual { .. }), "manual")
                .on_hover_text("Shutter and gain fixed. Nothing automatic left.")
                .clicked()
            {
                mode = ColorExposureMode::Manual {
                    integration_ms: 16.0,
                    analog_gain: 1.0,
                };
            }
            ui.separator();

            match &mut mode {
                ColorExposureMode::Auto { compensation } => {
                    ui.add(
                        egui::Slider::new(compensation, -2.0..=2.0)
                            .text("compensation")
                            .fixed_decimals(1),
                    )
                    .on_hover_text("Negative underexposes, positive overexposes.");
                }
                ColorExposureMode::SemiAuto { pseudo_ms } => {
                    ui.add(
                        egui::Slider::new(pseudo_ms, 1.0..=66.0)
                            .text("shutter (ms)")
                            .fixed_decimals(1),
                    )
                    .on_hover_text(
                        "Asking for less than one mains period brings the flicker back — \
                         that is the trade.",
                    );
                }
                ColorExposureMode::Manual {
                    integration_ms,
                    analog_gain,
                } => {
                    ui.add(
                        egui::Slider::new(integration_ms, 1.0..=66.0)
                            .text("shutter (ms)")
                            .fixed_decimals(1),
                    )
                    .on_hover_text(
                        "Past ~33 ms the colour stream drops to 15 fps: 66 ms is one whole \
                         frame at 15 Hz. Infrared and depth are unaffected.",
                    );
                    ui.add(
                        egui::Slider::new(analog_gain, 1.0..=4.0)
                            .text("gain")
                            .fixed_decimals(2),
                    )
                    .on_hover_text(
                        "Brightens without lengthening the shutter — at the cost of noise.",
                    );
                }
            }
            self.color_exposure = mode;
        });
        if self.color_exposure != before
            && let Some(active) = self.active.as_ref()
        {
            let _ = active
                .worker
                .cmd_tx
                .send(CaptureCmd::SetColorExposure(self.color_exposure));
        }
        ui.add_space(2.0);
    }

    /// Two verticals raised from the ends of the flattened lockbar, plus the
    /// baseline joining them.
    ///
    /// The lockbar lands horizontal by construction, so the rails are the only
    /// thing left free to be wrong: against a true vertical, a lean of a
    /// degree is obvious, where against nothing it is not. The reading is
    /// printed too, because "it looks a bit off" is not a measurement — it is
    /// the same number the 'square' column carries, minus 90.
    fn draw_flatten_guides(&self, ui: &egui::Ui, rect: Rect, g: FlattenGuides) {
        let at = |p: (f32, f32)| {
            Pos2::new(
                rect.min.x + p.0 * rect.width(),
                rect.min.y + p.1 * rect.height(),
            )
        };
        let (l, r) = (at(g.left), at(g.right));
        let painter = ui.painter().with_clip_rect(rect);
        let guide = Stroke::new(1.0, Color32::from_rgba_unmultiplied(0, 210, 255, 150));
        for x in [l.x, r.x] {
            painter.line_segment([Pos2::new(x, rect.min.y), Pos2::new(x, rect.max.y)], guide);
        }
        painter.line_segment([l, r], guide);
        let (text, colour) = if g.lean_deg.abs() < 0.5 {
            ("upright".to_string(), Color32::from_rgb(60, 230, 90))
        } else {
            (
                format!("leaning {:+.1}°", g.lean_deg),
                Color32::from_rgb(0xe8, 0xa0, 0x30),
            )
        };
        painter.text(
            Pos2::new(rect.min.x + 6.0, rect.min.y + 6.0),
            egui::Align2::LEFT_TOP,
            text,
            egui::FontId::monospace(13.0),
            colour,
        );
    }

    /// Camera feed with the lockbar + head overlays, scaled to fit while
    /// keeping the source aspect ratio.
    fn draw_camera_view(&self, ui: &mut egui::Ui) {
        let avail = ui.available_size();
        let aspect = match self.active.as_ref() {
            Some(active) => match (active.backend, active.rgb_texture.as_ref()) {
                (_, Some(tex)) => {
                    let s = tex.size_vec2();
                    if s.y > 0.0 { s.x / s.y } else { 16.0 / 9.0 }
                }
                (Backend::KinectV2, None) => 1920.0 / 1080.0,
                (Backend::KinectV1, None) => 640.0 / 480.0,
                (Backend::Webcam(_), None) => 640.0 / 480.0,
                (Backend::None, None) => 16.0 / 9.0,
            },
            None => 16.0 / 9.0,
        };
        let (img_w, img_h) = if avail.x / avail.y > aspect {
            (avail.y * aspect, avail.y)
        } else {
            (avail.x, avail.x / aspect)
        };
        // Allocate the whole available area and centre the (letterboxed) image
        // rect in it, so a feed narrower than the panel isn't stuck to the left.
        let (area, _) = ui.allocate_exact_size(avail, Sense::hover());
        let rect = Rect::from_center_size(area.center(), Vec2::new(img_w, img_h));

        if let Some(active) = self.active.as_ref() {
            if let Some(tex) = &active.rgb_texture {
                ui.painter().image(
                    tex.id(),
                    rect,
                    Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)),
                    Color32::WHITE,
                );
                if let Some(g) = self.flatten_guides.filter(|_| self.flatten_view) {
                    self.draw_flatten_guides(ui, rect, g);
                }
                // Overlay (skeleton bones + anchor geometry) — unified across
                // live/capture/headless via `draw_overlay`. Clipped to the camera
                // rect: off-frame joints and the sidebar→VP extensions must not
                // spill onto the parallax scene below.
                //
                // Normalised through the dims of the frame the pose was computed
                // on, NOT the displayed texture: viewing the 512×424 depth stream
                // while the pose came from a 1920×1080 colour frame would
                // otherwise stretch the skeleton across the wrong pixels. Falls
                // back to the texture size before the first pose arrives.
                let tex_size = tex.size_vec2();
                let (sw, sh) = match active.pose_src {
                    (w, h) if w > 0 && h > 0 => (w as f32, h as f32),
                    _ => (tex_size.x, tex_size.y),
                };
                // The overlay lives in camera pixels; the flattened view is a
                // different space, and the pose is off the plane the warp is
                // built for. Drawing it there would place bones nowhere.
                if sw > 0.0 && sh > 0.0 && !self.flatten_view {
                    let clipped = ui.painter().with_clip_rect(rect);
                    let mut canvas = EguiOverlay {
                        painter: &clipped,
                        rect,
                        fw: sw,
                        fh: sh,
                    };
                    draw_overlay(
                        &mut canvas,
                        active.last_pose.as_ref(),
                        active.last_anchor.as_ref(),
                        sw as u32,
                        sh as u32,
                    );
                }
            } else {
                centered(ui, rect, "waiting for first RGB frame…");
            }
        } else {
            // During a backend switch the old device is already closed, so we
            // land here — show the settle status instead of the idle prompt.
            let msg = match &self.switch_state {
                SwitchState::Closing(_) => "closing…".to_string(),
                SwitchState::Opening(_) => format!("opening {}…", self.label_for(self.selected)),
                SwitchState::Waiting { .. } => {
                    format!("opening {}…", self.label_for(self.selected))
                }
                SwitchState::Idle => self
                    .error
                    .as_deref()
                    .unwrap_or("select an input device above to start streaming")
                    .to_string(),
            };
            centered(ui, rect, &msg);
        }
    }

    /// The parallax scene, stacked below the camera feed: a header row (eye
    /// source selector + `pe` readout + Live debug knobs) and the offscreen
    /// scene blitted as a plain egui image (so egui-rotate rotates it for
    /// free). The FBO is rendered + registered by [`DemoShell::redraw`] from
    /// [`Self::parallax_eye`]; here we just present it and record the panel
    /// rect so Mouse mode can map the pointer.
    fn draw_parallax_view(&mut self, ui: &mut egui::Ui) {
        // Filters — between the camera feed (input, above) and the parallax
        // scene (output, below): stability (1€) + cabinet geometry + inversions.
        self.filters_row(ui);
        ui.add_space(3.0);
        // Row 1: mode selector + current eye readout.
        ui.horizontal(|ui| {
            ui.label(
                RichText::new("parallax")
                    .strong()
                    .color(Color32::GRAY)
                    .monospace(),
            );
            ui.separator();
            ui.label("eye:");
            for mode in [
                ParallaxEye::Live,
                ParallaxEye::Mouse,
                ParallaxEye::AutoOrbit,
            ] {
                ui.selectable_value(&mut self.parallax_eye_mode, mode, mode.label());
            }
            ui.separator();
            let [ex, ey, ez] = self.parallax_eye;
            ui.label(
                RichText::new(format!("pe ({ex:+.0}, {ey:+.0}, {ez:.0}) mm"))
                    .monospace()
                    .size(15.0)
                    .color(LOCKBAR_COLOR),
            );
        });
        // (gain + axis flips + the 1€ stability filter all live in `filters_row`,
        // drawn above — between the camera feed and this parallax scene.)

        // Fill the whole panel (no 4:3 letterbox) so the scene reaches the
        // edges; the FBO + projection adopt this aspect → no distortion.
        let (rect, _) = ui.allocate_exact_size(ui.available_size(), Sense::hover());
        // Record the rect (egui logical space, already rotated by egui-rotate)
        // for Mouse mode's pointer mapping next frame, plus its aspect for the
        // FBO/projection (read by redraw).
        self.parallax_panel_rect = Some(rect);
        if rect.height() > 1.0 {
            self.parallax_aspect = (rect.width() / rect.height()).clamp(0.4, 4.0);
        }
        if let Some(tex) = self.parallax_tex {
            // GL textures are bottom-left origin; flip V so the scene shows
            // upright.
            ui.painter().image(
                tex,
                rect,
                Rect::from_min_max(Pos2::new(0.0, 1.0), Pos2::new(1.0, 0.0)),
                Color32::WHITE,
            );
            // Fixed screen-space reticle at the window centre — the reference
            // against which the off-axis slide is read.
            let c = rect.center();
            let s = 7.0;
            let stroke = Stroke::new(1.0_f32, Color32::from_white_alpha(150));
            ui.painter()
                .line_segment([Pos2::new(c.x - s, c.y), Pos2::new(c.x + s, c.y)], stroke);
            ui.painter()
                .line_segment([Pos2::new(c.x, c.y - s), Pos2::new(c.x, c.y + s)], stroke);
        } else {
            centered(ui, rect, "starting parallax view…");
        }
    }

    /// Row 2 — camera INPUT: raw camera-frame measurements, before any
    /// maths. Head pixel/depth/3D-raw plus the raw lockbar pixel detection.
    fn input_line(&self, ui: &mut egui::Ui) {
        let prefix = || {
            RichText::new("in  ▸")
                .strong()
                .color(Color32::GRAY)
                .monospace()
        };
        let Some(active) = self.active.as_ref() else {
            ui.horizontal(|ui| {
                ui.label(prefix());
                if let Some(err) = &self.error {
                    ui.colored_label(Color32::LIGHT_RED, err);
                } else if self.available.len() <= 1 {
                    ui.label(
                        RichText::new("no input — plug a device and click 'rescan'")
                            .color(Color32::GRAY),
                    );
                } else {
                    ui.label(RichText::new("select an input").color(Color32::GRAY));
                }
            });
            return;
        };
        let label = self.label_for(active.backend);
        // Line 1 — device + raw head measurements.
        ui.horizontal(|ui| {
            ui.label(prefix());
            if !active.backend.has_head_tracker() {
                ui.label(
                    RichText::new(format!("{label} | capture only — head tracking pending"))
                        .color(Color32::GRAY),
                );
            } else if let Some(head) = active.last_head {
                ui.label(
                    RichText::new(format!(
                        "{label} | head px ({}, {}) | depth {:.0} mm | 3D raw ({:+.0}, {:+.0}, {:+.0}) mm",
                        head.u, head.v, head.depth_mm, head.x_mm, head.y_mm, head.depth_mm,
                    ))
                    .monospace(),
                );
            } else {
                ui.label(
                    RichText::new(format!("{label} | waiting for head detection…"))
                        .color(Color32::GRAY),
                );
            }
        });
        // Line 2 — the anchor-derived lockbar, on its own row below, next to
        // the control that runs the detection again. The warmup freezes the
        // best detection because the cabinet is fixed — but the camera on top
        // of it is not, and a lock taken before it was aimed stayed wrong for
        // the rest of the session.
        ui.horizontal(|ui| {
            match active.last_lockbar {
                Some(bar) => {
                    ui.label(
                        RichText::new(format!(
                            "lockbar (anchor) px: row {}, w {}px, t {}px, slope {:+.1}°",
                            bar.mean_row(),
                            bar.mean_width_px(),
                            bar.thickness_px,
                            bar.slope_deg,
                        ))
                        .color(LOCKBAR_COLOR)
                        .monospace()
                        .size(15.0),
                    );
                }
                None if active.anchor_locked => {
                    ui.label(
                        RichText::new("lockbar (anchor): not found — aim the camera, then ↻")
                            .color(Color32::GRAY)
                            .monospace()
                            .size(15.0),
                    );
                }
                None => {
                    ui.label(
                        RichText::new("lockbar (anchor): looking…")
                            .color(Color32::GRAY)
                            .monospace()
                            .size(15.0),
                    );
                }
            }
            if ui
                .button("↻ Recalibrate")
                .on_hover_text(
                    "Detect the cabinet again from scratch. Use it after moving or \
                     re-aiming the camera — the calibration freezes on purpose once \
                     it has settled, so it will not follow the change on its own.",
                )
                .clicked()
            {
                let _ = active.worker.cmd_tx.send(CaptureCmd::Recalibrate);
            }
        });
    }

    /// Row 3 — camera OUTPUT: the head expressed in the lockbar frame, i.e.
    /// the (ΔX, ΔY, ΔZ) in mm that VPX consumes to shift its POV. (0,0,0) =
    /// head dead-centre above the lockbar; +X right, +Y down, +Z further.
    /// Green once the lockbar calibration is locked, yellow during warmup.
    fn output_line(&self, ui: &mut egui::Ui) {
        let Some(active) = self.active.as_ref() else {
            ui.label(RichText::new("—").color(Color32::GRAY));
            return;
        };
        let (Some(head), Some(quad)) = (active.last_head, active.last_lockbar.as_ref()) else {
            ui.label(RichText::new("waiting for head + lockbar…").color(Color32::GRAY));
            return;
        };
        let Some(lb) = lockbar_3d_center(quad, &active.intrinsics) else {
            ui.label(RichText::new("lockbar geometry degenerate").color(Color32::GRAY));
            return;
        };
        let dx = head.x_mm - lb.x;
        let dy = head.y_mm - lb.y;
        let dz = head.depth_mm - lb.z;
        // We only reach here with a live lockbar quad — always the model's
        // own detection (hand-fixed calibration files are gone on purpose).
        let (tag, color, hover) = (
            "anchor live",
            Color32::LIGHT_GREEN,
            "Calibration detected by the anchor model.",
        );
        ui.label(
            RichText::new(format!(
                "→ VPX   ΔX {dx:+.0}   ΔY {dy:+.0}   ΔZ {dz:+.0} mm   [{tag}]"
            ))
            .monospace()
            .color(color),
        )
        .on_hover_text(hover);
        self.camera_pose_table(ui, active);
    }

    /// Camera-pose read-out under the VPX delta: what the anchor decided,
    /// beside the two numbers it was given to decide it with.
    ///
    /// A table rather than a sentence because the point is a glance: a player
    /// who knows their camera sits a hand's width left of centre can confirm
    /// it in one look. Every header carries what its number means and which
    /// way its sign points — those come from `anchor::PoseField`, next to the
    /// code that defines them and pinned by `tests/pose_conventions.rs`.
    fn camera_pose_table(&self, ui: &mut egui::Ui, active: &Active) {
        if !active.anchor_locked {
            return;
        }
        let (Some(geom), Some((fw, fh, ..))) =
            (active.last_anchor.as_ref(), active.last_rgb_frame.as_ref())
        else {
            return;
        };
        let fx = color_focal_px(active.backend, *fw);
        let intr = anchor::CameraIntrinsics {
            fx,
            fy: fx,
            cx: *fw as f32 * 0.5,
            cy: *fh as f32 * 0.5,
        };
        let Some(pose) = anchor::camera_pose(geom, &intr, self.lockbar_width_mm) else {
            return;
        };
        // A webcam has no factory intrinsics, so its focal is a nominal guess.
        // Marked on the number itself rather than on the whole line: it is the
        // one input that is an estimate, and the reader should know which.
        let nominal_focal = matches!(active.backend, Backend::Webcam(_));

        // Inputs first, then what the anchor made of them. Getting a wrong
        // answer out of a right method is nearly always one of these two.
        let mut cells: Vec<(&str, String, &str)> = vec![
            (
                "lockbar",
                format!("{:.0} cm", self.lockbar_width_mm / 10.0),
                "INPUT. Width across the cabinet rails, and the only thing \
                 that gives the reconstruction a scale. Wrong here and every \
                 distance below is wrong by the same ratio. VPX supplies it \
                 per cabinet; the slider sets it here.",
            ),
            (
                "incline",
                format!("{:.0}\u{00b0}", self.table_incl_deg),
                "INPUT. Playfield angle against horizontal, as VPX reports it \
                 for the table. It does not enter the pose below — it tilts \
                 how head movement is turned into a point of view.",
            ),
            (
                "focal",
                format!("{}{fx:.0} px", if nominal_focal { "\u{2248}" } else { "" }),
                "INPUT. Focal length in pixels. Measured from the sensor on a \
                 Kinect; on a webcam it is a nominal guess (shown with \u{2248}), \
                 so read the distances as approximate. 'square' is the check \
                 on it.",
            ),
        ];
        let report = pose.report();
        cells.extend(report.iter().map(|f| (f.label, f.value.clone(), f.help)));

        // Out-of-square is the one cell worth interrupting for: it means an
        // input is wrong, so every number beside it is suspect.
        let out_of_square = (pose.rect_angle_deg - 90.0).abs() > 3.0;
        egui::Grid::new("anchor_pose_table")
            .striped(true)
            .spacing([12.0, 2.0])
            .show(ui, |ui| {
                for (label, _, help) in &cells {
                    ui.label(RichText::new(*label).small().strong())
                        .on_hover_text(*help);
                }
                ui.end_row();
                for (label, value, help) in &cells {
                    let mut t = RichText::new(value).monospace();
                    if *label == "square" && out_of_square {
                        t = t.color(Color32::from_rgb(0xc0, 0x39, 0x2b)).strong();
                    }
                    ui.label(t).on_hover_text(*help);
                }
                ui.end_row();
            });
        // The same reading in words, so nobody has to remember which way a
        // sign points. This is the exact sentence the plugin shows at startup.
        ui.label(
            RichText::new(pose.describe())
                .monospace()
                .color(Color32::GRAY),
        );
        // `pitch` is measured against the playfield, so a sloped table reads
        // its own slope back even with a perfectly level camera — the one
        // number here that looks broken when it is right. We know the incline,
        // so we can hand over the angle the reader can check with a level.
        if self.table_incl_deg > 0.5 {
            let vs_horizon = pose.pitch_deg - self.table_incl_deg;
            ui.label(
                RichText::new(format!(
                    "pitch is against the {:.0}° playfield — the camera itself is {:.0}° {} level",
                    self.table_incl_deg,
                    vs_horizon.abs(),
                    if vs_horizon >= 0.0 { "below" } else { "above" },
                ))
                .monospace()
                .color(Color32::GRAY),
            );
        }
        if out_of_square {
            // Only two things bend the shape: a focal that does not belong to
            // the frame the outline was found in, or an outline that does not
            // follow the cabinet. The lockbar width is not one of them — it
            // scales the distances and leaves the angles alone — and saying so
            // here saves the reader from adjusting the one setting that cannot
            // help.
            ui.label(
                RichText::new(
                    "Cabinet does not rebuild square: the outline does not follow the real \
                     rails, or the focal above is not this camera's. Not the lockbar width \
                     — that scales the distances without bending the shape. Turn on Flatten \
                     to see it.",
                )
                .color(Color32::from_rgb(0xc0, 0x39, 0x2b)),
            );
        }
    }
}

/// Player-facing rotation label shown on the ⟳ toolbar button. Inverted on
/// purpose w.r.t. egui-rotate's clockwise enum: the physically-rotated pincab
/// display flips the apparent handedness, so the value the player reads as
/// "270°" is the one that, once applied, looks upright — egui-rotate's `CW90`.
/// Keeps the toolbar number matching how the user thinks of their screen.
fn rotation_label(r: Rotation) -> &'static str {
    match r {
        Rotation::None => "0°",
        Rotation::CW270 => "90°",
        Rotation::CW180 => "180°",
        Rotation::CW90 => "270°",
    }
}

/// One ⟳ click = +90° clockwise *from the player's seat*. Because our labels
/// run opposite to the egui enum (see [`rotation_label`]), a user-clockwise
/// step walks the enum the other way: 270° → 0° → 90° → 180° → 270°.
fn next_rotation(r: Rotation) -> Rotation {
    match r {
        Rotation::CW90 => Rotation::None,   // 270° → 0°
        Rotation::None => Rotation::CW270,  // 0°   → 90°
        Rotation::CW270 => Rotation::CW180, // 90°  → 180°
        Rotation::CW180 => Rotation::CW90,  // 180° → 270°
    }
}

fn centered(ui: &mut egui::Ui, rect: Rect, text: &str) {
    ui.painter().rect_filled(rect, 4.0, Color32::from_gray(20));
    ui.painter().text(
        rect.center(),
        egui::Align2::CENTER_CENTER,
        text,
        egui::FontId::proportional(18.0),
        Color32::LIGHT_GRAY,
    );
}

/// Red-edged banner under the main menu: the calibration model has only seen
/// a handful of cabinets, so we ask hard for contributions. Bold, centred.
fn contribution_banner(ui: &mut egui::Ui) {
    let red = Color32::from_rgb(0xD3, 0x2F, 0x2F);
    egui::Frame::group(ui.style())
        .stroke(Stroke::new(1.2, red))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.vertical_centered(|ui| {
                ui.label(
                    RichText::new(
                        "⚠  The auto-calibration model has only seen a \
                         handful of cabinets — it needs YOUR captures to \
                         learn.  🎁 Contribute!",
                    )
                    .strong()
                    .color(red),
                );
            });
        });
}

/// Overlay draw target. All inputs are in ORIGINAL IMAGE PIXEL coords; each impl
/// maps to its own surface. A single [`draw_overlay`] feeds the live egui view,
/// the capture RGB buffer and the headless `--pose-test`, so the overlay can
/// never drift out of sync between them again.
trait OverlayCanvas {
    fn stroke(&mut self, a: (f32, f32), b: (f32, f32), col: [u8; 3], width: f32);
    fn dashed(&mut self, a: (f32, f32), b: (f32, f32), col: [u8; 3]);
    fn disc(&mut self, p: (f32, f32), r: f32, col: [u8; 3]);
    /// Text at a fixed top-left screen offset (image px on the egui view;
    /// no-op on the raw-RGB buffer).
    fn text(&mut self, p: (f32, f32), s: &str, col: [u8; 3]);
}

/// Draws onto the live egui view: image px → view-rect screen coords.
struct EguiOverlay<'a> {
    painter: &'a egui::Painter,
    rect: Rect,
    fw: f32,
    fh: f32,
}

impl EguiOverlay<'_> {
    fn map(&self, p: (f32, f32)) -> Pos2 {
        self.rect.left_top()
            + Vec2::new(
                (p.0 / self.fw) * self.rect.width(),
                (p.1 / self.fh) * self.rect.height(),
            )
    }
}

impl OverlayCanvas for EguiOverlay<'_> {
    fn stroke(&mut self, a: (f32, f32), b: (f32, f32), col: [u8; 3], width: f32) {
        self.painter.line_segment(
            [self.map(a), self.map(b)],
            Stroke::new(width, Color32::from_rgb(col[0], col[1], col[2])),
        );
    }
    fn dashed(&mut self, a: (f32, f32), b: (f32, f32), col: [u8; 3]) {
        let (pa, pb) = (self.map(a), self.map(b));
        let n = ((pa.distance(pb) / 10.0).round() as usize).clamp(2, 300);
        let c = Color32::from_rgb(col[0], col[1], col[2]);
        for i in 0..=n {
            let t = i as f32 / n as f32;
            self.painter.circle_filled(pa + (pb - pa) * t, 1.5, c);
        }
    }
    fn disc(&mut self, p: (f32, f32), r: f32, col: [u8; 3]) {
        self.painter
            .circle_filled(self.map(p), r, Color32::from_rgb(col[0], col[1], col[2]));
    }
    fn text(&mut self, p: (f32, f32), s: &str, col: [u8; 3]) {
        self.painter.text(
            self.map(p),
            egui::Align2::LEFT_TOP,
            s,
            egui::FontId::monospace(13.0),
            Color32::from_rgb(col[0], col[1], col[2]),
        );
    }
}

/// Draws onto a raw RGB888 buffer (image px == buffer px).
struct RgbOverlay<'a> {
    buf: &'a mut [u8],
    w: usize,
    h: usize,
}

impl OverlayCanvas for RgbOverlay<'_> {
    fn stroke(&mut self, a: (f32, f32), b: (f32, f32), col: [u8; 3], _width: f32) {
        draw_line_rgb(
            self.buf,
            self.w,
            self.h,
            (a.0 as i32, a.1 as i32),
            (b.0 as i32, b.1 as i32),
            col,
        );
    }
    fn dashed(&mut self, a: (f32, f32), b: (f32, f32), col: [u8; 3]) {
        let len = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
        let n = ((len / 12.0).round() as i32).clamp(2, 300);
        for i in 0..=n {
            let t = i as f32 / n as f32;
            draw_disc_rgb(
                self.buf,
                self.w,
                self.h,
                (a.0 + (b.0 - a.0) * t) as i32,
                (a.1 + (b.1 - a.1) * t) as i32,
                1,
                col,
            );
        }
    }
    fn disc(&mut self, p: (f32, f32), r: f32, col: [u8; 3]) {
        draw_disc_rgb(
            self.buf, self.w, self.h, p.0 as i32, p.1 as i32, r as i32, col,
        );
    }
    fn text(&mut self, _p: (f32, f32), _s: &str, _col: [u8; 3]) {}
}

/// The one overlay: skeleton bones (NO landmark dots), the anchor lockbar
/// rectangle, the two sidebars (solid rail bottom→corner, dashed corner→
/// depth-VP), the lateral-offset tick, and a readout. Shared by live /
/// capture / headless.
fn draw_overlay<C: OverlayCanvas>(
    c: &mut C,
    pose: Option<&blazepose::Pose>,
    anchor: Option<&anchor::AnchorGeometry>,
    fw: u32,
    fh: u32,
) {
    const BONE: [u8; 3] = [0xcc, 0xcc, 0xcc];
    const GREEN: [u8; 3] = [60, 230, 90];
    const CYAN: [u8; 3] = [0, 210, 255];
    const YELLOW: [u8; 3] = [255, 225, 25];
    const MAGENTA: [u8; 3] = [255, 60, 220];
    let _ = (fw, fh);
    if let Some(p) = pose {
        use blazepose::idx::{
            LEFT_ELBOW, LEFT_SHOULDER, LEFT_WRIST, NOSE, RIGHT_ELBOW, RIGHT_SHOULDER, RIGHT_WRIST,
        };
        let g = |i: usize| (p.landmarks[i].x, p.landmarks[i].y);
        for (a, b) in [
            (LEFT_SHOULDER, RIGHT_SHOULDER),
            (LEFT_SHOULDER, LEFT_ELBOW),
            (LEFT_ELBOW, LEFT_WRIST),
            (RIGHT_SHOULDER, RIGHT_ELBOW),
            (RIGHT_ELBOW, RIGHT_WRIST),
            (NOSE, LEFT_SHOULDER),
            (NOSE, RIGHT_SHOULDER),
        ] {
            c.stroke(g(a), g(b), BONE, 2.0);
        }
        // The actual POV point — the glabella (between the eyes, pushed
        // toward the brow), NOT the nose. The skeleton bones converge on
        // the nose, which used to be mistaken for the tracked point; this
        // marker shows what really drives the camera.
        c.disc(head_center_xy(p), 5.0, MAGENTA);
    }
    if let Some(geo) = anchor {
        let cr = geo.corners;
        for k in 0..4 {
            c.stroke(cr[k], cr[(k + 1) % 4], GREEN, 2.5);
        }
        // Sidebars: solid real rail (bottom → player corner), then dashed
        // extrapolation to the depth vanishing point.
        for sb in [geo.left_sidebar, geo.right_sidebar] {
            c.stroke(sb.1, sb.0, CYAN, 2.5);
            if let Some(vp) = geo.depth_vp {
                c.dashed(sb.0, vp, CYAN);
            }
        }
        // The lockbar centre IS the intersection of the quad's diagonals
        // (the perspective-correct image of the rectangle's centre) — draw
        // the diagonals themselves so the construction is visible.
        c.stroke(cr[0], cr[2], YELLOW, 1.0);
        c.stroke(cr[1], cr[3], YELLOW, 1.0);
        c.disc(geo.lockbar_center, 4.0, GREEN);
        let vp_txt = geo
            .depth_vp
            .map_or_else(|| "inf".to_string(), |(x, y)| format!("({x:.0},{y:.0})"));
        c.text(
            (8.0, 8.0),
            &format!(
                "anchor · width {:.0}px · lateral {:+.0}px · vp {vp_txt}",
                geo.lockbar_width_px, geo.lateral_offset_px
            ),
            [255, 255, 255],
        );
    }
}

// ============================================================ Backend opening

fn open_backend(b: Backend) -> Result<Capture, String> {
    // Claim the cross-process lock BEFORE touching the hardware, so a
    // device busy in the plugin / a cron capture / another demo yields one
    // readable line instead of a driver-level failure. All webcams share
    // the "webcam" slug: SDL ids aren't stable across processes, and one
    // cab has one webcam.
    let slug = match b {
        Backend::None => return Err("no backend selected".to_string()),
        Backend::KinectV1 => "kinect-v1",
        Backend::KinectV2 => "kinect-v2",
        Backend::Webcam(_) => "webcam",
    };
    let hwlock = headtracking::hwlock::HwLock::acquire(slug)?;
    let mut cap = match b {
        Backend::None => unreachable!("handled above"),
        Backend::KinectV2 => open_kinect_v2()?,
        Backend::KinectV1 => open_kinect_v1()?,
        Backend::Webcam(idx) => open_webcam(idx)?,
    };
    cap.hwlock = Some(hwlock);
    Ok(cap)
}

/// Common `Capture` fields (models, filter, empty buffers) shared by every
/// backend opener — only `backend`, `intrinsics` and `inner` differ.
fn new_capture(backend: Backend, intrinsics: Intrinsics, inner: Inner) -> Capture {
    Capture {
        backend,
        intrinsics,
        inner,
        hwlock: None,
        convert_ms: 0.0,
        blaze_worker: BlazePoseWorker::spawn(),
        anchor_worker: AnchorWorker::spawn(),
        pose_filter: make_pose_filter(),
        median_gate: headtracking::filter::MedianGate::new(3),
        started_at: Instant::now(),
        baseline: None,
        last_pose: None,
        last_head: None,
        last_anchor: None,
        last_lockbar: None,
        last_rgb_frame: None,
        last_depth: None,
        last_ir: None,
        depth_frames: 0,
        ir_frames: 0,
        last_rgb_at: None,
        last_ir_at: None,
        last_depth_at: None,
        // Filled in by `open_kinect_v2`; every other backend leaves the
        // registration off and keeps the native depth path.
        registration: None,
        color_intr: Intrinsics {
            fx: 0.0,
            fy: 0.0,
            cx: 0.0,
            cy: 0.0,
        },
        v2_rgb: freenect2::RgbFrame::default(),
        v2_ir: freenect2::IrFrame::default(),
        v2_depth: freenect2::DepthFrame::default(),
        head_window: Vec::new(),
        bigdepth: Vec::new(),
        rgb_scratch: Vec::new(),
        bigdepth_ok: false,
        head_window_ok: false,
        window_checked: false,
        reg_ms: 0.0,
        copy_ms: 0.0,
        filter_us: FilterUs::default(),
        reg_warned: false,
        selected_stream: StreamKind::Rgb,
        last_rgb_refresh: None,
        rgb_refresh_ack: None,
        depth_color: None,
        pose_src: (0, 0),
        head_ms: 0.0,
        anchor_ms: 0.0,
    }
}

/// `HT_DEPTH_PIPELINE=cpu` forces the Kinect v2's CPU depth pipeline.
///
/// The GPU pipeline is the right default — it is what took a Windows tester
/// from 5 fps of depth to 30 — but a default nobody can override is a
/// hypothesis nobody can test. A tester whose preview freezes after a few
/// seconds (2026-09-03, RTX 5080 on OpenCL, working on 0.0.30 which predates
/// the GPU pipeline shipping) can now answer the question in one run instead
/// of waiting for us to guess.
///
/// Anything other than `cpu` keeps the default, so a typo cannot silently
/// downgrade someone.
fn gpu_depth_allowed() -> bool {
    gpu_depth_allowed_from(std::env::var("HT_DEPTH_PIPELINE").ok().as_deref())
}

/// The decision itself, away from the process environment so it can be tested.
fn gpu_depth_allowed_from(value: Option<&str>) -> bool {
    !matches!(value, Some(v) if v.trim().eq_ignore_ascii_case("cpu"))
}

fn open_kinect_v2() -> Result<Capture, String> {
    let ctx = freenect2::Context::new().map_err(|e| format!("freenect2 Context::new: {e}"))?;
    // Drain any stale libfreenect2 error from a previous call before we
    // run the one whose error we want to surface.
    let _ = freenect2::take_last_log_error();
    let count = ctx.enumerate();
    if count <= 0 {
        // `enumerate()` returns 0 with no `Err` for two distinct cases:
        // "really not plugged in" and "plugged in but libusb_open
        // failed". libfreenect2 logs the latter ("failed to open Kinect
        // v2: … LIBUSB_ERROR_ACCESS") via the bridge; pop that line
        // and use it verbatim so the UI shows the actual C++ reason.
        if let Some(reason) = freenect2::take_last_log_error() {
            return Err(reason);
        }
        return Err("no Kinect v2 found on USB".to_string());
    }
    let allow_gpu = gpu_depth_allowed();
    let device = ctx
        .open_with_gpu(allow_gpu)
        .map_err(|e| format!("freenect2 open_default: {e}"))?;
    // First thing in the log, before anything else can go wrong: which depth
    // pipeline actually opened. On the CPU one the v2 drops USB depth packets
    // and delivers ~5 fps instead of 30, and every downstream number inherits
    // it — so a report that starts with "CPU" needs no further diagnosis.
    let pipeline = device.depth_pipeline();
    if !allow_gpu {
        info!(
            pipeline,
            "Kinect v2 depth pipeline: forced by HT_DEPTH_PIPELINE"
        );
    } else if pipeline == "CPU" {
        warn!(
            "Kinect v2 depth pipeline: CPU — expect dropped USB packets and \
             ~5 fps of depth. No OpenCL device available (missing driver ICD?)."
        );
    } else {
        info!(pipeline, "Kinect v2 depth pipeline");
    }
    device
        .start_streams(true, true)
        .map_err(|e| format!("freenect2 start_streams: {e}"))?;
    let p = device.ir_params();
    // Built after `start_streams` on purpose: before the device has streamed
    // its factory intrinsics the params read all-zero and the registration
    // would map nothing. Colour params come from the same moment.
    let registration = device.registration();
    let c = device.color_params();
    let mut cap = new_capture(
        Backend::KinectV2,
        Intrinsics {
            fx: p.fx,
            fy: p.fy,
            cx: p.cx,
            cy: p.cy,
        },
        Inner::KinectV2 { device, _ctx: ctx },
    );
    if c.fx > 0.0 {
        info!(
            fx = c.fx,
            fy = c.fy,
            cx = c.cx,
            cy = c.cy,
            "kinect v2: colour intrinsics + depth↔colour registration ready"
        );
        cap.color_intr = Intrinsics {
            fx: c.fx,
            fy: c.fy,
            cx: c.cx,
            cy: c.cy,
        };
        cap.bigdepth = vec![0.0; freenect2::BIGDEPTH_LEN];
        cap.rgb_scratch = vec![0u8; 1920 * 1080 * 4];
        cap.head_window = vec![f32::INFINITY; HEAD_WINDOW_SIDE * HEAD_WINDOW_SIDE];
        cap.registration = Some(registration);
    } else {
        // Not fatal: the head path falls back to naive depth-grid scaling.
        warn!("kinect v2: colour intrinsics unavailable — depth registration disabled");
    }
    Ok(cap)
}

fn open_kinect_v1() -> Result<Capture, String> {
    info!("kinect v1 open: building context");
    let ctx = freenect::Context::new().map_err(|e| format!("freenect Context::new: {e}"))?;
    let count = ctx.enumerate();
    info!(count, "kinect v1 open: pre-open enumerate");
    if count <= 0 {
        return Err("no Kinect v1 found on USB".to_string());
    }
    info!(index = 0, "kinect v1 open: calling freenect_open_device");
    let mut device = ctx.open(0).map_err(|e| {
        // Surface a precise error on the path that's most likely to
        // bite Windows users. The wrapper Display impl already maps
        // -12 to a UsbDk/Zadig hint; copying it through verbatim
        // gives the demo log a single line a user can paste.
        let msg = format!("freenect open: {e}");
        warn!("{msg}");
        msg
    })?;
    info!("kinect v1 open: device handle acquired, starting streams");
    device
        .start_streams(true, true)
        .map_err(|e| format!("freenect start_streams: {e}"))?;

    // Light the LED at open (Green). The tilt read-out is seeded later from the
    // capture thread's periodic refresh (the slider seeds on its first read).
    if let Err(e) = device.set_led(freenect::LedState::Green) {
        warn!(?e, "kinect v1: initial set_led failed");
    }

    Ok(new_capture(
        Backend::KinectV1,
        Intrinsics {
            fx: freenect::FX,
            fy: freenect::FY,
            cx: freenect::CX,
            cy: freenect::CY,
        },
        Inner::KinectV1 { device, _ctx: ctx },
    ))
}

/// Nominal `(w, h, fps)` of a webcam's best advertised mode, for the stream
/// info bar. `index` is the 1-based position in SDL's list (what
/// [`Backend::Webcam`] carries), resolved to an `SDL_CameraID` the same way
/// [`open_webcam`] does. Picks the highest-resolution mode, breaking ties on
/// frame rate — SDL usually advertises exactly one. `None` if SDL says nothing,
/// in which case the bar falls back to a generic 640×480 30p.
fn webcam_nominal_spec(index: u32) -> Option<(u32, u32, u32)> {
    let cams = webcam::list().ok()?;
    if cams.is_empty() {
        return None;
    }
    let n = (index.max(1) as usize - 1).min(cams.len() - 1);
    let fmts = webcam::supported_formats(cams[n].id).ok()?;
    fmts.iter()
        .max_by_key(|f| (u64::from(f.width) * u64::from(f.height), f.fps as u64))
        .map(|f| (f.width, f.height, f.fps.round().max(0.0) as u32))
}

fn open_webcam(index: u32) -> Result<Capture, String> {
    // `index` is 1-based into SDL's enumerated list — NOT a raw SDL_CameraID
    // (those are opaque, assigned 1,2,… per hot-plug, so passing the index
    // straight to SDL_OpenCamera fails with "Invalid camera device instance
    // ID" the moment the IDs don't line up). Resolve it through list() first.
    let cams = webcam::list().map_err(|e| format!("webcam list: {e}"))?;
    if cams.is_empty() {
        return Err("no cameras found (SDL enumerated 0 devices)".to_string());
    }
    let n = (index.max(1) as usize - 1).min(cams.len() - 1);
    let chosen = &cams[n];
    info!(
        picked = n + 1,
        id = chosen.id,
        name = %chosen.name,
        total = cams.len(),
        "webcam selected"
    );
    let camera = webcam::Camera::open(chosen.id).map_err(|e| format!("webcam open: {e}"))?;
    Ok(new_capture(
        Backend::Webcam(index),
        // Without lockbar/disc calibration the webcam focal is unknown; the
        // zeroed intrinsics make consumers fall back to the shared
        // WEBCAM_FX_PER_WIDTH nominal. Replaced by ht-calibrate output when
        // that lands.
        Intrinsics {
            fx: 0.0,
            fy: 0.0,
            cx: 0.0,
            cy: 0.0,
        },
        Inner::Webcam { camera },
    ))
}

// ============================================================ Image conversion

/// Convert a BGRX (Kinect v2) buffer to packed RGB888 — needed because the
/// head detector takes RGB888 frames. Allocates a fresh `Vec` of size
/// `width * height * 3`. ~6 MB for 1920×1080; copy cost is negligible
/// compared to the detector's ~10 ms inference.
fn bgrx_to_rgb888(bgrx: &[u8]) -> Vec<u8> {
    let (src, _) = bgrx.as_chunks::<4>();
    // Sized up front and written through fixed-width chunks rather than
    // pushed a byte at a time: at 1080p that is 6.2 M pushes a frame, and the
    // bounds-checked push loop refuses to vectorise.
    let mut out = vec![0u8; src.len() * 3];
    let (dst, _) = out.as_chunks_mut::<3>();
    for (d, s) in dst.iter_mut().zip(src) {
        *d = [s[2], s[1], s[0]]; // R, G, B from B, G, R, X
    }
    out
}

/// Build the camera-view image for the stream the user selected. Falls back to
/// the colour frame whenever the chosen stream hasn't delivered anything yet
/// (e.g. the depth listener during the first moments after an open), so the
/// view never goes blank on a selection.
fn stream_color_image(frame: &LatestFrame, want: StreamKind) -> ColorImage {
    match want {
        StreamKind::Ir => {
            if let Some(ir) = frame.ir.as_deref() {
                let (w, h, data) = ir;
                // Auto-levelled so a 16-bit v2 intensity frame and an 8-bit v1
                // one both land in a visible range — same levelling the shared
                // captures use, so what you see matches what you'd upload.
                let gray = autolevel_gray8_raw(data, false);
                return rgb888_to_color_image(*w, *h, &gray8_to_rgb888(&gray));
            }
        }
        // Both colour modes arrive on the same RGB frame; only the size differs.
        StreamKind::Rgb | StreamKind::RgbHigh => {}
    }
    frame_to_color_image(frame.w, frame.h, &frame.pixels, frame.layout)
}

/// Google's Turbo colormap, subsampled to 16 control points. Perceptually far
/// easier to read than gray for depth: distance reads as hue (near = blue,
/// far = red) instead of as brightness, which the eye judges poorly.
const TURBO_LUT: [[u8; 3]; 16] = [
    [48, 18, 59],
    [65, 69, 171],
    [70, 117, 237],
    [57, 162, 252],
    [37, 200, 220],
    [30, 228, 176],
    [61, 244, 128],
    [123, 252, 82],
    [176, 244, 57],
    [217, 220, 56],
    [246, 187, 55],
    [254, 145, 45],
    [246, 100, 30],
    [225, 63, 17],
    [193, 35, 8],
    [122, 4, 3],
];

/// Map a depth frame (raw millimetres, `0` = no reading) to packed RGB888
/// through [`TURBO_LUT`], with linear interpolation between control points.
/// Distances are clamped to the tracking range; invalid pixels come out
/// near-black so dropouts stay obvious rather than reading as "very close".
///
/// Shared by the live depth view and the `*_depthview.png` contribution
/// preview, so a reviewer sees exactly what the operator saw. The lossless
/// `*_depth.png` keeps the raw 16-bit millimetres — this is only ever the
/// human-readable rendering.
fn depth_to_turbo_rgb888(mm: &[u16]) -> Vec<u8> {
    let (lo, hi) = (DEPTH_MIN_MM, DEPTH_MAX_MM);
    let mut out = Vec::with_capacity(mm.len() * 3);
    for &z in mm {
        if z == 0 {
            out.extend_from_slice(&[10, 10, 12]);
            continue;
        }
        let t = ((f32::from(z) - lo) / (hi - lo)).clamp(0.0, 1.0);
        let x = t * (TURBO_LUT.len() - 1) as f32;
        let i = x.floor() as usize;
        let j = (i + 1).min(TURBO_LUT.len() - 1);
        let f = x - i as f32;
        let (a, b) = (TURBO_LUT[i], TURBO_LUT[j]);
        out.extend_from_slice(&[
            (f32::from(a[0]) + (f32::from(b[0]) - f32::from(a[0])) * f) as u8,
            (f32::from(a[1]) + (f32::from(b[1]) - f32::from(a[1])) * f) as u8,
            (f32::from(a[2]) + (f32::from(b[2]) - f32::from(a[2])) * f) as u8,
        ]);
    }
    out
}

/// Expand 8-bit gray to packed RGB888 by replicating each byte across the three
/// channels — how a Kinect v1 IR frame is fed to the models, which take colour.
fn gray8_to_rgb888(gray: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(gray.len() * 3);
    for &v in gray {
        out.extend_from_slice(&[v, v, v]);
    }
    out
}

fn rgb888_to_color_image(width: u32, height: u32, data: &[u8]) -> ColorImage {
    debug_assert_eq!(data.len(), (width * height * 3) as usize);
    ColorImage::from_rgb([width as usize, height as usize], data)
}

/// Ends of the lockbar in the flattened view, as fractions of the view's
/// width and height — so the guides survive the texture being scaled into
/// whatever panel it lands in.
#[derive(Clone, Copy)]
struct FlattenGuides {
    left: (f32, f32),
    right: (f32, f32),
    /// How far the reconstruction is from square, in degrees. Zero means the
    /// cabinet should stand perfectly upright between the guides.
    lean_deg: f32,
}

/// Resample `src` through `h`, a destination-to-source homography, into a
/// `dst_w` x `dst_h` image.
///
/// Nearest neighbour on purpose: this is a geometry check, and a blurrier
/// picture would hide exactly the edge that has to be judged straight or not.
/// Destination pixels that fall outside the source come back dark grey, so
/// the extent of the real frame stays visible inside the rectified one.
fn flatten_image(src: &ColorImage, h: &[f64; 9], dst_w: usize, dst_h: usize) -> ColorImage {
    const OUTSIDE: egui::Color32 = egui::Color32::from_rgb(0x20, 0x20, 0x20);
    let (sw, sh) = (src.width(), src.height());
    let px = &src.pixels;
    let mut out = vec![OUTSIDE; dst_w * dst_h];
    for y in 0..dst_h {
        let fy = y as f64 + 0.5;
        for x in 0..dst_w {
            let fx = x as f64 + 0.5;
            let w = h[6] * fx + h[7] * fy + h[8];
            if w.abs() < 1e-12 {
                continue;
            }
            let u = (h[0] * fx + h[1] * fy + h[2]) / w;
            let v = (h[3] * fx + h[4] * fy + h[5]) / w;
            if u < 0.0 || v < 0.0 {
                continue;
            }
            let (u, v) = (u as usize, v as usize);
            if u >= sw || v >= sh {
                continue;
            }
            out[y * dst_w + x] = px[v * sw + u];
        }
    }
    ColorImage::new([dst_w, dst_h], out)
}

/// A published frame as an egui image, reading the driver's layout directly.
///
/// The display has to build RGBA either way, so a BGRX frame goes straight to
/// `Color32` in one pass instead of being repacked to RGB888 first — on the
/// capture thread, at that — and converted a second time here.
fn frame_to_color_image(width: u32, height: u32, pixels: &[u8], layout: FrameLayout) -> ColorImage {
    if layout == FrameLayout::Rgb888 {
        return rgb888_to_color_image(width, height, pixels);
    }
    let (bpp, ch) = (layout.bpp(), layout.channels());
    let px = (width as usize) * (height as usize);
    debug_assert_eq!(pixels.len(), px * bpp);
    let mut out = Vec::with_capacity(px);
    for p in pixels.chunks_exact(bpp) {
        out.push(egui::Color32::from_rgb(p[ch[0]], p[ch[1]], p[ch[2]]));
    }
    ColorImage::new([width as usize, height as usize], out)
}

/// Repack a published frame into tightly-packed RGB888 — what the PNG writer
/// and the overlay baker want. Free of charge when it already is.
fn frame_to_rgb888(pixels: &[u8], layout: FrameLayout) -> Vec<u8> {
    match layout {
        FrameLayout::Rgb888 => pixels.to_vec(),
        FrameLayout::Bgrx8888 => bgrx_to_rgb888(pixels),
    }
}

// ============================================================ Screenshot

/// Compact slug for a backend, used in screenshot filenames. Mirrors the
/// dropdown label but stripped of spaces / punctuation so the file is
/// easy to grep and predictable in shells.
fn backend_slug(b: Backend) -> String {
    match b {
        Backend::None => "demo".to_string(),
        Backend::KinectV1 => "kinect-v1".to_string(),
        Backend::KinectV2 => "kinect-v2".to_string(),
        Backend::Webcam(i) => format!("webcam-{i}"),
    }
}

/// Where the user's own copy of a shared capture goes.
///
/// There is deliberately no default. The demo is a single binary people run
/// straight out of a download — very often from a temporary extraction (WinRAR
/// and friends open archives in `%TEMP%`), and a folder picked next to the
/// binary is then wiped without warning. So we ask, once, at the moment it
/// matters; declining is a perfectly good answer, and the upload happens
/// either way.
#[derive(Debug, Clone, Default)]
enum LocalCopy {
    /// Nobody has been asked yet — the next share opens the picker.
    #[default]
    Unasked,
    /// The user cancelled the picker: upload only, and no nagging on the next
    /// share (the window still offers a button to change their mind).
    Declined,
    /// Keep a copy in this folder.
    Folder(std::path::PathBuf),
}

/// Open the native folder picker for the local copy. `None` when the user
/// cancels, when the folder they chose cannot actually be written to, or when
/// no picker is reachable at all (a bare compositor with no desktop portal) —
/// every one of those falls back to upload-only, which is a working outcome.
/// Status colours for the contribution panel. Green means "we have it", red
/// means "we do not" — an upload that failed used to be orange, one shade away
/// from the yellow call-outs, sitting under a green success counter.
const COLOR_OK: Color32 = Color32::from_rgb(0x66, 0xff, 0x99);
const COLOR_BAD: Color32 = Color32::from_rgb(0xff, 0x5c, 0x5c);
/// A device that streams isochronously — audio or video — and so reserves bus
/// bandwidth while it runs. Amber rather than red: it is a capability, not a
/// fault, and on a cabinet it is usually the sound card minding its business.
const COLOR_RESERVES: Color32 = Color32::from_rgb(0xd2, 0x9a, 0x22);

/// Unit the point-of-view read-out is shown in.
///
/// VPU is what the plugin actually sends, so it is the honest default -- but it
/// is Visual Pinball's own unit and means nothing to anyone else, and a whole
/// VPU is about half a millimetre, so the figures look implausibly small until
/// you know that. Millimetres and inches are the same number rescaled; nothing
/// about the tracking changes with this setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum PovUnit {
    #[default]
    Mm,
    Inch,
    Vpu,
}

impl PovUnit {
    const ALL: [Self; 3] = [Self::Mm, Self::Inch, Self::Vpu];

    fn label(self) -> &'static str {
        match self {
            Self::Mm => "mm",
            Self::Inch => "inch",
            Self::Vpu => "VPU",
        }
    }

    /// Convert a value the plugin expresses in VPU into this unit.
    fn from_vpu(self, vpu: f32) -> f32 {
        match self {
            Self::Mm => headtracking::camera::units::vpu_to_mm(vpu),
            Self::Inch => headtracking::camera::units::vpu_to_mm(vpu) / 25.4,
            Self::Vpu => vpu,
        }
    }

    /// Convert a value measured in millimetres into this unit.
    fn from_mm(self, mm: f32) -> f32 {
        match self {
            Self::Mm => mm,
            Self::Inch => mm / 25.4,
            Self::Vpu => headtracking::camera::units::mm_to_vpu(mm),
        }
    }

    /// Decimals for a distance rather than a delta: a head sits about a metre
    /// away, so millimetres need none and inches need one.
    fn pose_decimals(self) -> usize {
        match self {
            Self::Mm => 0,
            Self::Inch => 1,
            Self::Vpu => 0,
        }
    }

    /// Decimals worth showing: an inch of head movement is a lot, a VPU is not.
    fn decimals(self) -> usize {
        match self {
            Self::Mm => 1,
            Self::Inch => 3,
            Self::Vpu => 2,
        }
    }
}

/// One colour per point-of-view axis. Distinct enough to tell apart at a
/// glance, and legible on both the light and dark themes the demo runs in.
const POV_X: Color32 = Color32::from_rgb(0xe8, 0x8a, 0x3c);
const POV_Y: Color32 = Color32::from_rgb(0x5c, 0xc9, 0x6a);
const POV_Z: Color32 = Color32::from_rgb(0x5a, 0xa9, 0xe6);

/// One USB reading, produced entirely on a worker thread.
///
/// The report and the tree travel together because they come from the same
/// enumeration: splitting them would mean walking the bus twice to draw one
/// window.
struct UsbSnapshot {
    report: Option<usb_check::UsbReport>,
    tree: Vec<usb_check::BusNode>,
}

/// UI-side view of the drop's reachability.
#[derive(Debug, Clone, Default)]
enum ReachState {
    /// Never asked (the probe starts when the panel is first opened).
    #[default]
    Unknown,
    /// A probe is in flight.
    Checking,
    Known(contribute::Reach),
}

impl ReachState {
    /// Uploading is offered only on a proven-good answer. "Not asked yet" and
    /// "still asking" are not permission — the whole point is that a capture
    /// is never taken on the assumption that it can be sent.
    fn allows_upload(&self) -> bool {
        matches!(self, Self::Known(r) if r.is_up())
    }
}

/// Where a capture the drop refused is kept so it can still be handed over.
///
/// Not the install folder (Program Files is read-only for a normal user, and
/// a pincab runs the demo from wherever it was unzipped) and not a temp
/// directory (the contributor has to be able to find it and drag it into
/// Discord). The home directory is the one place that is writable, stable and
/// nameable in a sentence.
fn default_rescue_dir() -> std::path::PathBuf {
    let home = std::env::var_os("USERPROFILE")
        .or_else(|| std::env::var_os("HOME"))
        .map(std::path::PathBuf::from);
    match home {
        Some(dir) => dir.join("headtracking-captures-not-sent"),
        None => std::path::PathBuf::from("headtracking-captures-not-sent"),
    }
}

fn ask_local_copy_folder() -> Option<std::path::PathBuf> {
    let dir = rfd::FileDialog::new()
        .set_title("Keep your own copy of the capture — choose a folder (Cancel: upload only)")
        .pick_folder()?;
    match probe_writable(&dir) {
        Ok(()) => {
            info!(path = %dir.display(), "local copy folder chosen");
            Some(dir)
        }
        Err(e) => {
            warn!(path = %dir.display(), "local copy folder unusable: {e}");
            rfd::MessageDialog::new()
                .set_level(rfd::MessageLevel::Warning)
                .set_title("Cannot write there")
                .set_description(format!(
                    "{}\n\n{e}\n\nNo local copy will be kept — the upload still works.",
                    dir.display()
                ))
                .show();
            None
        }
    }
}

/// Prove a file can actually be written inside `dir`. Being able to *see* a
/// folder says nothing: the interesting failures (a read-only mount, a
/// policy-locked `Program Files`) only show up on the write.
fn probe_writable(dir: &std::path::Path) -> Result<(), String> {
    std::fs::create_dir_all(dir).map_err(|e| format!("cannot create: {e}"))?;
    let probe = dir.join(".ht-write-probe");
    std::fs::write(&probe, b"").map_err(|e| format!("cannot write: {e}"))?;
    let _ = std::fs::remove_file(&probe);
    Ok(())
}

/// Shared stem for a capture's two files: `ht_<backend>_<UTC>_<rand6>`, where
/// `rand6` is 6 hex chars from the sub-second clock (unique enough — the drop
/// rejects duplicate names — and unambiguous to read back for a removal).
fn contribution_stem(backend: Backend) -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    let rand6 = now.subsec_nanos() & 0x00ff_ffff;
    format!(
        "ht_{}_{}_{rand6:06x}",
        backend_slug(backend),
        format_utc_stamp(now.as_secs())
    )
}

/// Encode every stream of a capture into the contribution file set: `_raw` +
/// `_det` colour planes, lossless `_depth` (16-bit raw mm) + Turbo
/// `_depthview`, `_ir` (native bit depth) + auto-levelled `_irview`. Shared
/// by the GUI Contribute button and the headless `--contribute` mode so both
/// always export EVERY stream the camera has.
#[allow(clippy::too_many_arguments)]
fn build_contribution_files(
    stem: &str,
    backend: Backend,
    (w, h): (u32, u32),
    raw: &[u8],
    det: &[u8],
    depth: Option<&(u32, u32, Vec<u16>)>,
    ir_v2: Option<&(u32, u32, Vec<u16>)>,
    ir_v1: Option<&freenect::IrFrame>,
    meta: &[(String, String)],
) -> Vec<(String, Vec<u8>)> {
    // RGB planes are 8-bit colour; depth is 16-bit gray in raw mm; v2 IR is
    // 16-bit gray intensity; v1 IR is 8-bit gray.
    let mut files: Vec<(String, Vec<u8>)> = Vec::new();
    // The diagnostics log rides along: it is what separates "the model missed
    // this cabinet" from "USB starved the sensor and it never saw it".
    if let Some(log) = log_tail_for_contribution() {
        files.push((format!("{stem}_log.txt"), log));
    }
    for (kind, src) in [("raw", raw), ("det", det)] {
        match png_bytes_meta(w, h, src, meta) {
            Ok(bytes) => files.push((format!("{stem}_{kind}.png"), bytes)),
            Err(e) => warn!("contribution: {kind} png encode failed: {e}"),
        }
    }
    // Each depth/IR modality ships a lossless file (the real values) plus a
    // `*view` preview so it's reviewable by eye. Depth's preview is the same
    // Turbo false-colour the live view uses — distance reads as hue, which
    // the eye judges far better than brightness. The lossless `_depth.png`
    // below stays 16-bit raw mm: that's what training consumes.
    if let Some((dw, dh, mm)) = depth {
        match png_gray16_bytes(*dw, *dh, mm) {
            Ok(bytes) => files.push((format!("{stem}_depth.png"), bytes)),
            Err(e) => warn!("contribution: depth png encode failed: {e}"),
        }
        match png_bytes_meta(*dw, *dh, &depth_to_turbo_rgb888(mm), meta) {
            Ok(bytes) => files.push((format!("{stem}_depthview.png"), bytes)),
            Err(e) => warn!("contribution: depthview png encode failed: {e}"),
        }
    }
    // IR: v1 always exports the freshly grabbed native 8-bit frame (the
    // live `last_ir` is the same sensor but 8→16-widened while the IR
    // stream is selected — using the grab keeps the file format identical
    // across modes). v2 exports its live 16-bit stream.
    if backend == Backend::KinectV1 {
        if let Some(frame) = ir_v1 {
            match png_gray8_bytes(frame.width, frame.height, &frame.data) {
                Ok(bytes) => files.push((format!("{stem}_ir.png"), bytes)),
                Err(e) => warn!("contribution: ir png encode failed: {e}"),
            }
            // v1 IR is native 8-bit; widen to reuse the u16 auto-leveller.
            let widened: Vec<u16> = frame.data.iter().map(|&b| u16::from(b)).collect();
            match autolevel_gray8(frame.width, frame.height, &widened, false) {
                Ok(bytes) => files.push((format!("{stem}_irview.png"), bytes)),
                Err(e) => warn!("contribution: irview png encode failed: {e}"),
            }
        }
    } else if let Some((iw, ih, intensity)) = ir_v2 {
        match png_gray16_bytes(*iw, *ih, intensity) {
            Ok(bytes) => files.push((format!("{stem}_ir.png"), bytes)),
            Err(e) => warn!("contribution: ir png encode failed: {e}"),
        }
        match autolevel_gray8(*iw, *ih, intensity, false) {
            Ok(bytes) => files.push((format!("{stem}_irview.png"), bytes)),
            Err(e) => warn!("contribution: irview png encode failed: {e}"),
        }
    }
    files
}

/// Encode an RGB888 buffer to PNG bytes in memory (for the contribution
/// upload — the screenshot path writes straight to a file instead).
fn png_bytes(width: u32, height: u32, rgb888: &[u8]) -> Result<Vec<u8>, String> {
    png_bytes_meta(width, height, rgb888, &[])
}

/// Like [`png_bytes`] but embeds `meta` as PNG `tEXt` chunks, so a contribution
/// image carries its own tracking read-out (backend, head X/Y/Z mm, pose…) and
/// captures stay comparable across v1/v2/webcam without a side file.
fn png_bytes_meta(
    width: u32,
    height: u32,
    rgb888: &[u8],
    meta: &[(String, String)],
) -> Result<Vec<u8>, String> {
    let mut buf = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut buf, width, height);
        encoder.set_color(png::ColorType::Rgb);
        encoder.set_depth(png::BitDepth::Eight);
        for (k, v) in meta {
            // PNG keywords are Latin-1, ≤79 chars; values are free text.
            let _ = encoder.add_text_chunk(k.clone(), v.clone());
        }
        let mut wr = encoder
            .write_header()
            .map_err(|e| format!("png header: {e}"))?;
        wr.write_image_data(rgb888)
            .map_err(|e| format!("png write: {e}"))?;
    }
    Ok(buf)
}

/// Default table inclination (degrees) — a typical widebody playfield slope.
/// The GUI exposes it as an editable field; headless capture assumes it.
const DEFAULT_TABLE_INCL_DEG: f32 = 6.5;

/// Physical cab geometry that scales the lockbar-derived read-out: the real
/// bar width (mm) and the playfield inclination (deg). Both are per-cab and
/// eventually come from VPX; here they're GUI fields / CLI args.
#[derive(Debug, Clone, Copy)]
struct CabGeom {
    table_incl_deg: f32,
    lockbar_mm: f32,
}

/// Colour-camera focal length in pixels for the lockbar-distance estimate.
/// The Kinects have known **factory** colour intrinsics (independent of any
/// detection), so we use them directly — scaled to the actual frame width in
/// case the resolution differs from the native one. The webcam's focal is
/// unknown until the lockbar autocalib recovers it, so fall back to the
/// shared [`WEBCAM_FX_PER_WIDTH`] nominal.
fn color_focal_px(backend: Backend, frame_width: u32) -> f32 {
    let w = frame_width as f32;
    match backend {
        // Kinect v2 colour: ~1081 px at 1920×1080.
        Backend::KinectV2 => 1081.0 * w / 1920.0,
        // Kinect v1 RGB: ~525 px at 640×480 (the colour lens — NOT the 580 px
        // depth/IR focal, which lives on a different imager; see the const's
        // doc in the freenect crate).
        Backend::KinectV1 => freenect::RGB_FX * w / 640.0,
        // Webcam / none: nominal until autocalib supplies the real focal.
        _ => w * WEBCAM_FX_PER_WIDTH,
    }
}

/// Build the tracking read-out embedded into a contribution capture as PNG
/// `tEXt` chunks. Lets us line up v1/v2/webcam shots of the same scene and
/// compare what each backend recovered — head **Z** above all (depth-sampled
/// on the Kinects, shoulder-width-triangulated on the webcam). Keys are
/// `ht_`-prefixed so they don't clash with generic viewer metadata.
fn capture_meta(
    backend: Backend,
    dims: (u32, u32),
    stem: &str,
    geom: CabGeom,
    head: Option<HeadPixel>,
    pose: Option<&blazepose::Pose>,
    lockbar: Option<&headtracking::calibration::LockbarQuadRgb>,
) -> Vec<(String, String)> {
    let CabGeom {
        table_incl_deg,
        lockbar_mm,
    } = geom;
    let mut m: Vec<(String, String)> = Vec::new();
    let mut push = |k: &str, v: String| m.push((k.to_string(), v));
    push("ht_stem", stem.to_string());
    push(
        "ht_backend",
        match backend {
            Backend::None => "none".to_string(),
            Backend::KinectV1 => "kinect-v1".to_string(),
            Backend::KinectV2 => "kinect-v2".to_string(),
            Backend::Webcam(i) => format!("webcam-{i}"),
        },
    );
    let (w, h) = dims;
    push("ht_frame", format!("{w}x{h}"));
    push("ht_table_incl_deg", format!("{table_incl_deg:.2}"));
    push("ht_lockbar_mm", format!("{lockbar_mm:.0}"));
    // How the head Z was obtained differs by sensor — record it so a mm value
    // is never read out of context.
    push(
        "ht_z_source",
        match backend {
            Backend::KinectV1 | Backend::KinectV2 => "depth@nose".to_string(),
            Backend::Webcam(_) => "shoulder-width".to_string(),
            Backend::None => "none".to_string(),
        },
    );
    match head {
        Some(hp) => {
            push("ht_head_x_mm", format!("{:.1}", hp.x_mm));
            push("ht_head_y_mm", format!("{:.1}", hp.y_mm));
            push("ht_head_z_mm", format!("{:.1}", hp.depth_mm));
            push("ht_head_px", format!("{},{}", hp.u, hp.v));
        }
        None => push("ht_head", "none".to_string()),
    }
    match pose {
        Some(p) => {
            push("ht_pose_presence", format!("{:.3}", p.presence));
            let lm = |i: usize| -> String {
                let l = p.landmarks[i];
                format!("{:.0},{:.0} v{:.2}", l.x, l.y, l.visibility)
            };
            push("ht_nose", lm(blazepose::idx::NOSE));
            push("ht_l_shoulder", lm(blazepose::idx::LEFT_SHOULDER));
            push("ht_r_shoulder", lm(blazepose::idx::RIGHT_SHOULDER));
            push("ht_l_wrist", lm(blazepose::idx::LEFT_WRIST));
            push("ht_r_wrist", lm(blazepose::idx::RIGHT_WRIST));
            let ls = p.landmarks[blazepose::idx::LEFT_SHOULDER];
            let rs = p.landmarks[blazepose::idx::RIGHT_SHOULDER];
            let sw = ((ls.x - rs.x).powi(2) + (ls.y - rs.y).powi(2)).sqrt();
            push("ht_shoulder_width_px", format!("{sw:.1}"));
        }
        None => push("ht_pose", "none".to_string()),
    }
    // Lockbar-derived geometry: apparent width → estimated distance (the 610 mm
    // bar seen through the shared nominal focal), and the bar centre's
    // pixel offset → camera lateral/vertical offset off the playfield centreline
    // at that distance. Same nominal-focal placeholder as the webcam Z; the
    // autocalib homography will replace `fx` with the real one. See
    // [[headtracking-autocalib-vision]].
    match lockbar {
        Some(lb) => {
            let [tl, tr, br, bl] = lb.corners.map(|(u, v)| (u as f32, v as f32));
            let edge =
                |a: (f32, f32), b: (f32, f32)| ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt();
            let width_px = 0.5 * (edge(tl, tr) + edge(bl, br));
            // Factory colour focal for the Kinects, nominal for the webcam.
            let fx = color_focal_px(backend, lb.frame_width);
            let dist_mm = if width_px > 1.0 {
                fx * lockbar_mm / width_px
            } else {
                0.0
            };
            push("ht_color_fx", format!("{fx:.0}"));
            let cx = 0.25 * (tl.0 + tr.0 + br.0 + bl.0);
            let cy = 0.25 * (tl.1 + tr.1 + br.1 + bl.1);
            let frame_cx = lb.frame_width as f32 * 0.5;
            let frame_cy = lb.frame_height as f32 * 0.5;
            // Bar centred on the playfield: if it sits right of frame centre the
            // camera is left of the centreline, hence the flipped sign. +X = cam
            // to the right, +Y = cam above the bar centre.
            let off_x_mm = (frame_cx - cx) * dist_mm / fx;
            let off_y_mm = (frame_cy - cy) * dist_mm / fx;
            push("ht_lockbar_width_px", format!("{width_px:.1}"));
            push("ht_lockbar_dist_mm", format!("{dist_mm:.0}"));
            push("ht_lockbar_center_px", format!("{cx:.0},{cy:.0}"));
            push("ht_cam_offset_x_mm", format!("{off_x_mm:.0}"));
            push("ht_cam_offset_y_mm", format!("{off_y_mm:.0}"));
            push("ht_lockbar_slope_deg", format!("{:.2}", lb.slope_deg));
            push("ht_lockbar_thickness_px", lb.thickness_px.to_string());
        }
        None => push("ht_lockbar", "none".to_string()),
    }
    m
}

/// Median valid Kinect depth (mm) in a `2·half+1` window around an RGB pixel
/// `(u,v)`, mapped into the depth grid with the same crude scale as
/// [`head_pixel_from_pose_depth`]. `None` if too few valid samples.
fn sample_depth_at(
    depth: &[u16],
    depth_dims: (u32, u32),
    rgb_dims: (u32, u32),
    u: f32,
    v: f32,
    half: i32,
) -> Option<f32> {
    let (dw, dh) = depth_dims;
    let (rw, rh) = rgb_dims;
    if dw == 0 || dh == 0 || rw == 0 || rh == 0 {
        return None;
    }
    let cu = (u * dw as f32 / rw as f32) as i32;
    let cv = (v * dh as f32 / rh as f32) as i32;
    let (lo, hi) = (DEPTH_MIN_MM as u16, DEPTH_MAX_MM as u16);
    let mut s: Vec<u16> = Vec::new();
    for dv in -half..=half {
        let y = cv + dv;
        if y < 0 || y >= dh as i32 {
            continue;
        }
        let row = y as usize * dw as usize;
        for du in -half..=half {
            let x = cu + du;
            if x < 0 || x >= dw as i32 {
                continue;
            }
            let z = depth[row + x as usize];
            if (lo..=hi).contains(&z) {
                s.push(z);
            }
        }
    }
    if s.len() < 8 {
        return None;
    }
    s.sort_unstable();
    Some(f32::from(s[s.len() / 2]))
}

/// Autocalibration read-out for a capture: the focal + camera distance the
/// **lockbar homography** recovers (the zero-install calibration), the
/// sidebar vanishing-point focal when both rails were fit, and — on the
/// Kinects — the **measured depth at the lockbar** as ground truth that checks
/// both the recovered focal and the operator's tape measure in one shot.
/// See [[headtracking-autocalib-vision]].
fn autocalib_meta(
    lockbar: Option<&headtracking::calibration::LockbarQuadRgb>,
    rgb_dims: (u32, u32),
    depth: Option<&(u32, u32, Vec<u16>)>,
) -> Vec<(String, String)> {
    // The two experimental focal estimators that used to emit
    // `ht_autocalib_fx` / `ht_autocalib_dist_mm` / `ht_autocalib_vp_fx`
    // are gone: both leaned on the lockbar's ~70 mm band thickness, which
    // is too thin to measure (proven wrong on real captures). The metric
    // reference is the lockbar WIDTH; webcam focal will come from the full
    // playfield rectangle. Only the depth ground truth remains here.
    let mut m: Vec<(String, String)> = Vec::new();
    let Some(q) = lockbar else {
        return m;
    };
    // Ground truth: measured depth at the lockbar centre (Kinect only).
    if let Some((dw, dh, dd)) = depth {
        let c = q.corners.map(|(u, v)| (u as f32, v as f32));
        let bx = 0.25 * (c[0].0 + c[1].0 + c[2].0 + c[3].0);
        let by = 0.25 * (c[0].1 + c[1].1 + c[2].1 + c[3].1);
        if let Some(z) = sample_depth_at(dd, (*dw, *dh), rgb_dims, bx, by, 6) {
            m.push(("ht_lockbar_depth_mm".to_string(), format!("{z:.0}")));
        }
    }
    m
}

/// Encode an 8-bit single-channel buffer (e.g. Kinect v1 IR) to a grayscale
/// PNG. One intensity byte per pixel.
fn png_gray8_bytes(width: u32, height: u32, gray: &[u8]) -> Result<Vec<u8>, String> {
    let mut buf = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut buf, width, height);
        encoder.set_color(png::ColorType::Grayscale);
        encoder.set_depth(png::BitDepth::Eight);
        let mut wr = encoder
            .write_header()
            .map_err(|e| format!("png header: {e}"))?;
        wr.write_image_data(gray)
            .map_err(|e| format!("png write: {e}"))?;
    }
    Ok(buf)
}

/// Encode a depth buffer to a 16-bit grayscale PNG in raw millimetres — the
/// capture stays lossless and machine-readable (a plain viewer renders it
/// near-black since a few metres is a small slice of the 0–65535 range;
/// normalise offline to look at it). PNG stores samples wider than 8 bits
/// big-endian, so each `u16` is written high byte first.
fn png_gray16_bytes(width: u32, height: u32, mm: &[u16]) -> Result<Vec<u8>, String> {
    let mut be = Vec::with_capacity(mm.len() * 2);
    for &s in mm {
        be.extend_from_slice(&s.to_be_bytes());
    }
    let mut buf = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut buf, width, height);
        encoder.set_color(png::ColorType::Grayscale);
        encoder.set_depth(png::BitDepth::Sixteen);
        let mut wr = encoder
            .write_header()
            .map_err(|e| format!("png header: {e}"))?;
        wr.write_image_data(&be)
            .map_err(|e| format!("png write: {e}"))?;
    }
    Ok(buf)
}

/// Auto-level a `u16` sample buffer to an 8-bit grayscale PREVIEW so a capture
/// is reviewable at a glance — the lossless `_depth.png` / `_ir.png` still carry
/// the real values. Samples are stretched between their own min and max. When
/// `zero_is_hole` (depth: `0` = no data) zeros are excluded from the range and
/// stay black; otherwise (IR intensity) the full min..max is used.
fn autolevel_gray8(
    width: u32,
    height: u32,
    samples: &[u16],
    zero_is_hole: bool,
) -> Result<Vec<u8>, String> {
    let (mut lo, mut hi) = (u16::MAX, 0u16);
    for &v in samples {
        if zero_is_hole && v == 0 {
            continue;
        }
        lo = lo.min(v);
        hi = hi.max(v);
    }
    if hi < lo {
        // No usable samples (empty or all-hole) — emit a black frame.
        lo = 0;
        hi = 0;
    }
    let span = f32::from(hi.saturating_sub(lo)).max(1.0);
    let gray: Vec<u8> = samples
        .iter()
        .map(|&v| {
            if zero_is_hole && v == 0 {
                0
            } else {
                ((f32::from(v.saturating_sub(lo)) / span) * 255.0).clamp(0.0, 255.0) as u8
            }
        })
        .collect();
    png_gray8_bytes(width, height, &gray)
}

/// The auto-levelling of [`autolevel_gray8`] without the PNG encode: stretches
/// `samples` to the full 0–255 range and returns the raw bytes. Used for the
/// live IR view, so what the camera panel shows matches the `*view` PNG a
/// shared capture would carry.
fn autolevel_gray8_raw(samples: &[u16], zero_is_hole: bool) -> Vec<u8> {
    let (mut lo, mut hi) = (u16::MAX, 0u16);
    for &v in samples {
        if zero_is_hole && v == 0 {
            continue;
        }
        lo = lo.min(v);
        hi = hi.max(v);
    }
    if hi < lo {
        lo = 0;
        hi = 0;
    }
    let span = f32::from(hi.saturating_sub(lo)).max(1.0);
    samples
        .iter()
        .map(|&v| {
            if zero_is_hole && v == 0 {
                0
            } else {
                ((f32::from(v.saturating_sub(lo)) / span) * 255.0).clamp(0.0, 255.0) as u8
            }
        })
        .collect()
}

/// Decode an embedded JPEG thumbnail into an egui texture (once, at panel open).
fn load_thumb(ctx: &egui::Context, name: &str, bytes: &[u8]) -> TextureHandle {
    let color = match image::load_from_memory(bytes) {
        Ok(img) => {
            let rgba = img.to_rgba8();
            let (w, h) = rgba.dimensions();
            ColorImage::from_rgba_unmultiplied([w as usize, h as usize], rgba.as_raw())
        }
        Err(_) => ColorImage::from_rgba_unmultiplied([1, 1], &[0, 0, 0, 0]),
    };
    ctx.load_texture(name, color, egui::TextureOptions::LINEAR)
}

/// Append tiny subset fonts to egui's fallback chain so a few glyphs the
/// bundled fonts lack still render: ⏻ (U+23FB, power symbol — from Noto Sans
/// Symbols 2) and 🪟 (U+1FA9F, a 2020-era emoji absent from egui's bundled
/// NotoEmoji — from Noto Emoji). Both are subset to only the codepoints we use
/// (~5 KB total), vendored in `assets/` so the build stays offline. Pushed to
/// the end of each family, so they only kick in when nothing earlier covers the
/// glyph.
fn install_extra_glyph_fonts(ctx: &egui::Context) {
    use egui::{FontData, FontFamily};
    let mut fonts = egui::FontDefinitions::default();
    fonts.font_data.insert(
        "noto-symbols2".to_owned(),
        std::sync::Arc::new(FontData::from_static(include_bytes!(
            "../assets/NotoSymbolsSubset.ttf"
        ))),
    );
    fonts.font_data.insert(
        "noto-emoji-ext".to_owned(),
        std::sync::Arc::new(FontData::from_static(include_bytes!(
            "../assets/NotoEmojiSubset.ttf"
        ))),
    );
    for family in [FontFamily::Proportional, FontFamily::Monospace] {
        let chain = fonts.families.entry(family).or_default();
        chain.push("noto-symbols2".to_owned());
        chain.push("noto-emoji-ext".to_owned());
    }
    ctx.set_fonts(fonts);
}

/// Bump text sizes and hit-target padding for legibility on a pincab screen,
/// and stop widgets from resizing on hover (the ~1 px expansion looked odd,
/// especially on the red-outlined Contribute button).
fn apply_cab_style(ctx: &egui::Context) {
    use egui::{FontFamily, FontId, TextStyle};
    ctx.all_styles_mut(|style| {
        style
            .text_styles
            .insert(TextStyle::Body, FontId::new(18.0, FontFamily::Proportional));
        style.text_styles.insert(
            TextStyle::Button,
            FontId::new(19.0, FontFamily::Proportional),
        );
        style.text_styles.insert(
            TextStyle::Monospace,
            FontId::new(15.0, FontFamily::Monospace),
        );
        style.text_styles.insert(
            TextStyle::Heading,
            FontId::new(24.0, FontFamily::Proportional),
        );
        style.text_styles.insert(
            TextStyle::Small,
            FontId::new(15.0, FontFamily::Proportional),
        );
        style.spacing.button_padding = egui::vec2(9.0, 6.0);
        style.spacing.interact_size.y = 32.0;
        for w in [
            &mut style.visuals.widgets.inactive,
            &mut style.visuals.widgets.hovered,
            &mut style.visuals.widgets.active,
            &mut style.visuals.widgets.open,
        ] {
            w.expansion = 0.0;
        }
    });
}

/// Draw a help thumbnail at a given `width`, keeping aspect ratio (height
/// follows). Used to fill a column with an example/setup image.
fn show_thumb(ui: &mut egui::Ui, tex: &TextureHandle, width: f32) {
    let size = tex.size_vec2();
    let h = if size.x > 0.0 {
        width * size.y / size.x
    } else {
        width
    };
    ui.add(egui::Image::new(egui::load::SizedTexture::new(
        tex.id(),
        egui::vec2(width, h),
    )));
}

/// Format a UNIX-epoch-seconds value as `YYYYMMDD-HHMMSS` in UTC.
/// Inlined to avoid pulling `time` / `chrono` for one timestamp.
/// Algorithm: Howard Hinnant's civil-from-days. Valid 1970-01-01 → 9999.
/// `HH:MM:SS` UTC, for the diagnostics table. The date is in the log file;
/// on screen a session lasts minutes and the time of day is what locates an
/// event against what the user was doing.
fn stamp_hms() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
        % 86_400;
    format!(
        "{:02}:{:02}:{:02}",
        secs / 3600,
        (secs / 60) % 60,
        secs % 60
    )
}

fn format_utc_stamp(secs: u64) -> String {
    let total_days = (secs / 86_400) as i64;
    let secs_in_day = secs % 86_400;
    let h = secs_in_day / 3600;
    let m = (secs_in_day / 60) % 60;
    let s = secs_in_day % 60;
    let days = total_days + 719_468;
    let era = days.div_euclid(146_097);
    let doe = days.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y_civil = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let month_civil = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let y = y_civil + i64::from(mp >= 10);
    format!("{y:04}{month_civil:02}{d:02}-{h:02}{m:02}{s:02}")
}

/// Bake the unified overlay ([`draw_overlay`]: skeleton bones + anchor lockbar/
/// sidebars) into a copy of the RGB888 buffer, so a capture reads back what the
/// algorithms saw. Shared by the contribution `_det` export and the screenshot.
fn bake_overlays(
    width: u32,
    height: u32,
    rgb888: &[u8],
    pose: Option<&blazepose::Pose>,
    anchor: Option<&anchor::AnchorGeometry>,
) -> Vec<u8> {
    let mut out = rgb888.to_vec();
    let mut canvas = RgbOverlay {
        buf: &mut out,
        w: width as usize,
        h: height as usize,
    };
    draw_overlay(&mut canvas, pose, anchor, width, height);
    out
}

// ==================================================== Headless self-test
//
// Validate the skeleton pipeline with NOBODY in front of the camera, by
// synthesising an upper-body silhouette. Two paths, mirroring the two device
// families:
//   * a synthetic DEPTH frame → `skeleton_depth::Tracker::track`   (Kinect path)
//   * a synthetic MASK        → `skeleton_depth::Tracker::track_mask` (webcam path)
//   * optionally, a real RGB image → `personseg` → `track_mask`   (full webcam)
// Each renders a PNG with the detected joints drawn over the input and prints
// the joint coordinates, so a remote `ssh` run can eyeball the result.

fn put_rgb(buf: &mut [u8], w: usize, h: usize, x: i32, y: i32, c: [u8; 3]) {
    if x >= 0 && y >= 0 && (x as usize) < w && (y as usize) < h {
        let o = (y as usize * w + x as usize) * 3;
        buf[o..o + 3].copy_from_slice(&c);
    }
}

fn draw_disc_rgb(buf: &mut [u8], w: usize, h: usize, cx: i32, cy: i32, r: i32, c: [u8; 3]) {
    for dy in -r..=r {
        for dx in -r..=r {
            if dx * dx + dy * dy <= r * r {
                put_rgb(buf, w, h, cx + dx, cy + dy, c);
            }
        }
    }
}

fn draw_line_rgb(buf: &mut [u8], w: usize, h: usize, a: (i32, i32), b: (i32, i32), c: [u8; 3]) {
    let (mut x0, mut y0) = a;
    let (x1, y1) = b;
    let dx = (x1 - x0).abs();
    let dy = -(y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;
    loop {
        put_rgb(buf, w, h, x0, y0, c);
        if x0 == x1 && y0 == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x0 += sx;
        }
        if e2 <= dx {
            err += dx;
            y0 += sy;
        }
    }
}

/// Run the headless skeleton self-test. `image` (optional, JPEG) additionally
/// exercises the full webcam path via a file; `webcam` grabs a LIVE frame from
/// the camera and runs the same personseg → `track_mask` path.
/// Headless validation of the BlazePose head/pose path: run BlazePose on a real
/// `raw` frame, draw the 33 landmarks, and (if a 16-bit `depth` PNG is given)
/// sample the depth at the nose to derive the head distance. This is the same
/// derivation the live pipeline uses, exercised offline on captured modalities.
fn run_pose_test(
    raw_path: &str,
    depth_path: Option<&str>,
    _ir_path: Option<&str>,
    out_path: Option<&str>,
) -> Result<(), String> {
    use blazepose::idx::{LEFT_SHOULDER, LEFT_WRIST, NOSE, RIGHT_SHOULDER, RIGHT_WRIST};
    let img = image::open(raw_path)
        .map_err(|e| format!("open {raw_path}: {e}"))?
        .to_rgb8();
    let (w, h) = img.dimensions();
    let mut bp = blazepose::BlazePose::new().map_err(|e| format!("blazepose: {e}"))?;
    let Some(pose) = bp
        .detect(img.as_raw(), w, h, blazepose::PixelLayout::Rgb888)
        .map_err(|e| format!("detect: {e}"))?
    else {
        println!("NO POSE on {raw_path}");
        return Ok(());
    };

    // Copy of the frame; the unified overlay is painted onto it at the end.
    let mut buf = img.as_raw().clone();
    let (wu, hu) = (w as usize, h as usize);

    println!(
        "== pose-test {raw_path} ({w}x{h}) — presence={:.2}",
        pose.presence
    );
    for (n, i) in [
        ("nose", NOSE),
        ("Lsh", LEFT_SHOULDER),
        ("Rsh", RIGHT_SHOULDER),
        ("Lwr", LEFT_WRIST),
        ("Rwr", RIGHT_WRIST),
    ] {
        let l = &pose.landmarks[i];
        println!("  {n:5} ({:.0},{:.0}) vis={:.2}", l.x, l.y, l.visibility);
    }

    // Head distance: median non-zero depth in a small window at the nose.
    // Naive RGB→depth ratio (exact on v1; approximate on v2 — ignores the
    // IR-vs-RGB parallax, but validates the derivation offline).
    if let Some(dp) = depth_path {
        let d = image::open(dp)
            .map_err(|e| format!("open {dp}: {e}"))?
            .to_luma16();
        let (dw, dh) = d.dimensions();
        let nose = &pose.landmarks[NOSE];
        let dx = (nose.x / w as f32 * dw as f32) as i32;
        let dy = (nose.y / h as f32 * dh as f32) as i32;
        let mut samples = Vec::new();
        let rad = 6i32;
        for oy in -rad..=rad {
            for ox in -rad..=rad {
                let (sx, sy) = (dx + ox, dy + oy);
                if sx >= 0 && sy >= 0 && (sx as u32) < dw && (sy as u32) < dh {
                    let v = d.get_pixel(sx as u32, sy as u32).0[0];
                    if v != 0 {
                        samples.push(v);
                    }
                }
            }
        }
        if samples.is_empty() {
            println!("  head depth: no valid samples at nose ({dx},{dy} in {dw}x{dh})");
        } else {
            samples.sort_unstable();
            let mm = samples[samples.len() / 2];
            println!(
                "  head depth @nose ({dx},{dy} in {dw}x{dh}) = {mm} mm  ({} samples)",
                samples.len()
            );
        }
    }

    // --- Anchor model: cabinet frame → lateral / sidebars→∞ / width ---
    // Colour model: this path analyses a still image file, which is a colour
    // capture. An infrared still would need `anchor::prepare_ir` first.
    let anchor_geo = match anchor::AnchorDetector::new(anchor::Stream::Colour) {
        Ok(mut det) => match det.detect(img.as_raw(), w, h, anchor::PixelLayout::Rgb888) {
            Some(d) => {
                let geo = d.geometry(w, h);
                println!(
                    "  anchor score={:.2}  width={:.0}px  lateral={:+.0}px  vp={}",
                    d.score,
                    geo.lockbar_width_px,
                    geo.lateral_offset_px,
                    geo.depth_vp
                        .map_or("inf".to_string(), |(x, y)| format!("({x:.0},{y:.0})")),
                );
                Some(geo)
            }
            None => {
                println!("  anchor: no detection");
                None
            }
        },
        Err(e) => {
            println!("  anchor init failed: {e}");
            None
        }
    };

    // Paint the one unified overlay (skeleton bones + anchor geometry) — same
    // code path as the live view and screenshots.
    {
        let mut canvas = RgbOverlay {
            buf: &mut buf,
            w: wu,
            h: hu,
        };
        draw_overlay(&mut canvas, Some(&pose), anchor_geo.as_ref(), w, h);
    }

    let out = out_path.unwrap_or("pose-test.png");
    let bytes = png_bytes(w, h, &buf)?;
    std::fs::write(out, &bytes).map_err(|e| format!("write {out}: {e}"))?;
    println!("  -> {out}");
    Ok(())
}

fn save_rgb_screenshot_at(
    path: &std::path::Path,
    width: u32,
    height: u32,
    rgb888: &[u8],
    pose: Option<&blazepose::Pose>,
    anchor: Option<&anchor::AnchorGeometry>,
    meta: &[(String, String)],
) -> Result<(), String> {
    let painted = bake_overlays(width, height, rgb888, pose, anchor);
    let file = std::fs::File::create(path).map_err(|e| format!("create {path:?}: {e}"))?;
    let writer = std::io::BufWriter::new(file);
    let mut encoder = png::Encoder::new(writer, width, height);
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Eight);
    for (k, v) in meta {
        let _ = encoder.add_text_chunk(k.clone(), v.clone());
    }
    let mut wr = encoder
        .write_header()
        .map_err(|e| format!("png header: {e}"))?;
    wr.write_image_data(&painted)
        .map_err(|e| format!("png write: {e}"))?;
    Ok(())
}

/// Save a screenshot to the default location: next to the running
/// executable, named `<slug>_<UTC-timestamp>.png`. Used by the
/// "Screenshot" toolbar button and the headless capture mode when
/// the user didn't pass `--out`.
fn save_rgb_screenshot(
    slug: &str,
    width: u32,
    height: u32,
    rgb888: &[u8],
    pose: Option<&blazepose::Pose>,
    anchor: Option<&anchor::AnchorGeometry>,
    meta: &[(String, String)],
) -> Result<std::path::PathBuf, String> {
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|e| format!("clock before UNIX epoch: {e}"))?
        .as_secs();
    let stamp = format_utc_stamp(secs);
    let dir = std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(std::path::Path::to_path_buf))
        .unwrap_or_else(|| std::path::PathBuf::from("."));
    let path = dir.join(format!("{slug}_{stamp}.png"));
    save_rgb_screenshot_at(&path, width, height, rgb888, pose, anchor, meta)?;
    Ok(path)
}

// ============================================================ VPU mapping
//
// Mirrors `crate::camera::mapping::pose_delta_to_view_delta` from the plugin.
// Keep in sync with `src/camera/mapping.rs` if the axis convention changes.

const VPU_PER_MM: f64 = 50.0 / (25.4 * 1.0625);

fn pose_delta_to_view_delta_vpu(dx_mm: f32, dy_mm: f32, dz_mm: f32) -> (f32, f32, f32) {
    let to_vpu = |mm: f32| (f64::from(mm) * VPU_PER_MM) as f32;
    // Kinect Y points down (head going up → -Y) and Z grows away from the
    // sensor (head approaching → -Z). VPX Camera-mode Y is "forward away
    // from the player" and Z is upward.
    (to_vpu(dx_mm), -to_vpu(dz_mm), -to_vpu(dy_mm))
}

// ============================================================ Tracing capture

fn init_tracing(sink: Arc<Mutex<VecDeque<String>>>) {
    let env_filter = tracing_subscriber::EnvFilter::try_from_env("HEADTRACKING_LOG")
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new(DEFAULT_LOG_FILTER));

    // Console layer — only when stderr is actually a terminal. On a fresh
    // Windows release build (windows_subsystem = "windows") stderr is not
    // attached, so we don't even register the layer; on a `cargo run` from
    // a shell it lights up as usual.
    let stderr_layer = std::io::stderr().is_terminal().then(|| {
        tracing_subscriber::fmt::layer()
            .with_target(false)
            .with_writer(std::io::stderr)
    });

    // File layer — `headtracking-demo.log` next to the binary (append).
    // Same on every OS so triage instructions ("send me your log") are
    // uniform. If the file can't be opened (e.g. read-only Program Files
    // install), we skip it silently — the in-app panel still works.
    // The file keeps the target: it is what fills the diagnostics table's
    // `source` column, and it says at a glance whether a line came from us or
    // from libfreenect2 -- without hunting for a `[Component]` prefix.
    let file_layer = open_log_file().map(|mut f| {
        // First bytes of this session, before any event can be written.
        let _ = f.write_all(log_start_marker(env!("CARGO_PKG_VERSION")).as_bytes());
        tracing_subscriber::fmt::layer()
            .with_target(true)
            .with_ansi(false)
            .with_writer(FileSink {
                inner: Arc::new(Mutex::new(f)),
            })
    });

    let panel_layer = tracing_subscriber::fmt::layer()
        .with_target(false)
        .with_ansi(false)
        .with_writer(LogQueue { sink });

    tracing_subscriber::registry()
        .with(env_filter)
        .with(perf_table::TableLayer)
        .with(stderr_layer)
        .with(file_layer)
        .with(panel_layer)
        .init();
}

/// Who and which machine this session ran on, for following one cabinet
/// across releases.
///
/// Three fields because no single one does the job:
///
/// * `user` — the account name, which is what was asked for. It already
///   leaked into the logs incidentally, through capture paths like
///   `C:\Users\<name>\Downloads`, so naming it explicitly hides nothing new.
/// * `machine` — the host name. A tester with two cabinets has one account
///   and two machines; the account cannot tell them apart.
/// * `install` — a random id minted once beside the log and never sent
///   anywhere else. It is the only one that always exists: five of the first
///   twelve contributed logs carried **no** user name at all, because that
///   cabinet installs into `C:\Visual Pinball\` rather than a user folder,
///   and those were exactly the logs we could not attribute. It also survives
///   a rename, and it is the field to lean on when the other two are blank.
///
/// Every part is best effort; an unknown one is reported as `?` rather than
/// failing a startup over it.
fn host_identity() -> (String, String) {
    let user = std::env::var("USERNAME")
        .or_else(|_| std::env::var("USER"))
        .or_else(|_| std::env::var("LOGNAME"))
        .unwrap_or_else(|_| "?".to_owned());
    let machine = std::env::var("COMPUTERNAME")
        .or_else(|_| std::env::var("HOSTNAME"))
        .ok()
        .or_else(|| {
            std::fs::read_to_string("/etc/hostname")
                .ok()
                .map(|s| s.trim().to_owned())
                .filter(|s| !s.is_empty())
        })
        .unwrap_or_else(|| "?".to_owned());
    (user, machine)
}

/// A stable per-installation id, minted on first run beside the log.
///
/// Not derived from anything about the machine: a hash of the host name or
/// the MAC would be a fingerprint dressed as an id, and it would change the
/// moment either does. This is eight random-ish hex characters, kept in a
/// file, meaning nothing outside our own drop folder.
fn install_id() -> String {
    let Some(dir) = std::env::current_exe()
        .ok()
        .and_then(|e| e.parent().map(std::path::Path::to_path_buf))
    else {
        return "?".to_owned();
    };
    let path = dir.join("headtracking-install-id");
    if let Ok(existing) = std::fs::read_to_string(&path) {
        let id = existing.trim().to_owned();
        if !id.is_empty() {
            return id;
        }
    }
    // No `rand` dependency for eight characters: the clock in nanoseconds
    // mixed with the pid is distinct enough for an id whose only job is to
    // differ between installs.
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_nanos());
    let id = mint_install_id(nanos, std::process::id());
    // A read-only install directory just means we mint a fresh one each run;
    // the field is still better than nothing, and nothing here is worth
    // failing over.
    let _ = std::fs::write(&path, &id);
    id
}

/// The eight characters themselves, away from the clock and the filesystem so
/// they can be tested.
fn mint_install_id(nanos: u128, pid: u32) -> String {
    // The pid has to reach the low half: shifting it up and then keeping the
    // bottom eight hex characters threw it away entirely, and two installs
    // minted in the same nanosecond would have collided. Multiply by the
    // 64-bit golden ratio so both inputs touch every kept bit.
    let mixed = (nanos as u64)
        .rotate_left(17)
        .wrapping_mul(0x9E37_79B9_7F4A_7C15)
        ^ u64::from(pid).wrapping_mul(0xD6E8_FEB8_6659_FD93);
    format!("{mixed:016x}")[8..].to_owned()
}

/// Rigid session delimiter, written straight to the log file rather than
/// through `tracing`.
///
/// Deliberately not a tracing event: the point is to survive a change of log
/// format, and the format is exactly what drifted. Segmenting the contributed
/// logs by the startup banner silently lost every 0.0.30 session, because that
/// release wrote `INFO headtracking-demo starting` where later ones write
/// `INFO headtracking_demo: headtracking-demo starting` — sessions got merged
/// into their neighbour and a depth pipeline was read off the wrong version.
/// A fixed byte prefix cannot drift.
///
/// Start only, with no matching end: a session runs to the next start marker,
/// or to the end of the file if it is the last one. An end marker would add
/// nothing to that, and could not be trusted anyway — a crash or a kill never
/// writes one, so its absence would mean both "ended badly" and "still
/// running". A clean shutdown is worth logging, but as an ordinary event with
/// a timestamp and a reason, not as a delimiter.
///
/// No timestamp on the marker either: the tracing line right after it carries
/// one, and a second clock here would be one more thing to keep in sync.
const LOG_START_PREFIX: &str = "===== headtracking-demo log start v";
const LOG_MARKER_SUFFIX: &str = " =====";

fn log_start_marker(version: &str) -> String {
    format!("{LOG_START_PREFIX}{version}{LOG_MARKER_SUFFIX}\n")
}

/// The header prepended to a contributed log tail.
///
/// The delimiter above fixes session *boundaries*; it does nothing for a
/// session whose start scrolled out of the 256 KiB window — and five of the
/// first twelve contributed logs arrived exactly like that, with no version
/// line anywhere in them. There was no way to tell what build produced them.
/// The app knows its own version at the moment it packs the tail, so it says
/// so here, above the cut, where truncation cannot reach.
///
/// A prefix of its own, never `LOG_START_PREFIX`: this describes the upload,
/// not a session. Whatever sits between it and the first real start marker
/// belongs to a run that began above the cut — possibly an older build — and
/// stamping that with today's version would be a guess dressed as a fact.
const LOG_TAIL_PREFIX: &str = "===== headtracking-demo log tail v";

fn log_tail_header(version: &str, keep_bytes: u64, truncated: bool) -> String {
    let note = if truncated {
        format!(
            "last {} KiB — anything above the first start marker began before the cut",
            keep_bytes / 1024
        )
    } else {
        "whole file".to_owned()
    };
    format!("{LOG_TAIL_PREFIX}{version}, {note}{LOG_MARKER_SUFFIX}\n")
}

/// The tail of our own log, for the contribution.
///
/// Only the last stretch: a capture is judged against the minutes around it,
/// and the whole file would carry every earlier session for no gain. Sent as
/// text so it stays readable and reviewable by whoever shares it -- and it is
/// exactly the file the app itself writes, nothing assembled behind the
/// user's back.
fn log_tail_for_contribution() -> Option<Vec<u8>> {
    const KEEP_BYTES: u64 = 256 * 1024;
    let exe = std::env::current_exe().ok()?;
    let path = exe.parent()?.join("headtracking-demo.log");
    let text = std::fs::read_to_string(&path).ok()?;
    let cut = text.len().saturating_sub(KEEP_BYTES as usize);
    // Never cut mid-line: find the first newline at or after the cut.
    let start = text[cut..].find('\n').map_or(cut, |i| cut + i + 1);
    let mut out = log_tail_header(env!("CARGO_PKG_VERSION"), KEEP_BYTES, cut > 0).into_bytes();
    out.extend_from_slice(&text.as_bytes()[start..]);
    Some(out)
}

/// Open `headtracking-demo.log` next to the executable in append mode.
/// Returns `None` when the executable path can't be resolved or the file
/// can't be opened (read-only install dir, missing permissions) — the
/// caller drops the file layer in that case.
fn open_log_file() -> Option<std::fs::File> {
    let exe = std::env::current_exe().ok()?;
    let path = exe.parent()?.join("headtracking-demo.log");
    std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)
        .ok()
}

/// `MakeWriter` over a shared `File`. Each formatted event locks the file
/// for the duration of its write — fine here, the demo is mostly
/// single-threaded and tracing events are small. Avoids pulling
/// `tracing-appender` just for this.
#[derive(Clone)]
struct FileSink {
    inner: Arc<Mutex<std::fs::File>>,
}

impl<'a> MakeWriter<'a> for FileSink {
    type Writer = FileGuard<'a>;
    fn make_writer(&'a self) -> FileGuard<'a> {
        FileGuard(self.inner.lock())
    }
}

struct FileGuard<'a>(parking_lot::MutexGuard<'a, std::fs::File>);

impl Write for FileGuard<'_> {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        self.0.write(data)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.0.flush()
    }
}

#[derive(Clone)]
struct LogQueue {
    sink: Arc<Mutex<VecDeque<String>>>,
}

impl<'a> MakeWriter<'a> for LogQueue {
    type Writer = LogWriter;
    fn make_writer(&'a self) -> LogWriter {
        LogWriter {
            sink: Arc::clone(&self.sink),
            buf: Vec::with_capacity(256),
        }
    }
}

struct LogWriter {
    sink: Arc<Mutex<VecDeque<String>>>,
    buf: Vec<u8>,
}

impl Write for LogWriter {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        self.buf.extend_from_slice(data);
        Ok(data.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        if self.buf.is_empty() {
            return Ok(());
        }
        let text = String::from_utf8_lossy(&self.buf).into_owned();
        self.buf.clear();
        let mut sink = self.sink.lock();
        for line in text.lines() {
            if line.is_empty() {
                continue;
            }
            if sink.len() >= LOG_BUFFER_LINES {
                sink.pop_front();
            }
            sink.push_back(line.to_string());
        }
        Ok(())
    }
}

impl Drop for LogWriter {
    fn drop(&mut self) {
        let _ = self.flush();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A directory we can create and write in is accepted.
    #[test]
    fn probe_accepts_a_writable_folder() {
        let dir = std::env::temp_dir().join("ht-probe-ok");
        let _ = std::fs::remove_dir_all(&dir);
        assert!(probe_writable(&dir).is_ok());
        assert!(dir.is_dir(), "the probe creates the folder it validates");
        // The probe file must not survive — a contribution folder that starts
        // with a stray dotfile in it is confusing.
        assert!(!dir.join(".ht-write-probe").exists());
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A path that cannot be created is refused rather than assumed — this is
    /// the case that used to be swallowed by `let _ = create_dir_all(..)`.
    #[test]
    fn probe_refuses_a_path_under_a_file() {
        let file = std::env::temp_dir().join("ht-probe-not-a-dir");
        std::fs::write(&file, b"x").expect("write fixture");
        let err = probe_writable(&file.join("sub")).expect_err("a file is not a parent");
        assert!(err.starts_with("cannot create"), "{err}");
        let _ = std::fs::remove_file(&file);
    }

    /// Being able to see a folder says nothing about being able to write in
    /// it: the read-only case is exactly what a `Program Files` install or a
    /// read-only mount looks like, and it only shows up on the write.
    #[cfg(unix)]
    #[test]
    fn probe_refuses_a_read_only_folder() {
        use std::os::unix::fs::PermissionsExt;
        let dir = std::env::temp_dir().join("ht-probe-readonly");
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("create fixture");
        std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o500))
            .expect("chmod fixture");
        let refused = probe_writable(&dir);
        std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o700))
            .expect("restore fixture");
        let _ = std::fs::remove_dir_all(&dir);
        // Running as root defeats the permission bits entirely; skip rather
        // than assert something the environment cannot honour.
        if unsafe { libc_geteuid() } != 0 {
            let err = refused.expect_err("a read-only folder is not usable");
            assert!(err.starts_with("cannot write"), "{err}");
        }
    }

    #[cfg(unix)]
    unsafe fn libc_geteuid() -> u32 {
        unsafe extern "C" {
            fn geteuid() -> u32;
        }
        unsafe { geteuid() }
    }

    /// The process metrics must exist on whatever platform this is running on.
    /// The reader they replaced parsed `/proc/self/stat` and returned 0 on
    /// Windows and macOS, which turned the perf line into a lie precisely
    /// where we needed it — a saturated CPU depth pipeline reads as idle.
    #[test]
    fn process_metrics_are_reported_on_this_platform() {
        let mut m = Metrics::new();
        // sysinfo needs two samples spaced past its minimum interval before
        // CPU means anything; memory is available from the first.
        m.tick();
        std::thread::sleep(std::time::Duration::from_millis(400));
        m.window_start = std::time::Instant::now() - std::time::Duration::from_secs(2);
        m.tick();
        assert!(m.ram_mib > 0, "resident memory should not read as zero");
        assert!(
            m.cpu_pct.is_finite() && m.cpu_pct >= 0.0,
            "cpu {}",
            m.cpu_pct
        );
    }

    /// The probe line has to say what it found. The legend it used to print
    /// opened with "nothing on the USB bus: check the v2 power adapter" on
    /// every run, including the healthy ones.
    #[test]
    fn a_healthy_driver_probe_does_not_read_like_a_fault() {
        let ok = driver_probe_verdict(1, 0);
        assert!(ok.contains('1'), "{ok}");
        assert!(
            !ok.contains("power adapter") && !ok.contains("lack WinUSB"),
            "a healthy probe must not send anyone hunting: {ok}"
        );

        let absent = driver_probe_verdict(0, 0);
        assert!(
            absent.contains("power adapter"),
            "nothing on the bus is the adapter question: {absent}"
        );

        let half = driver_probe_verdict(3, 2);
        assert!(
            half.contains("2 of 3") && half.contains("WinUSB"),
            "a half-bound sensor must name the count: {half}"
        );
    }

    /// The frame clock is the one cadence in the perf line that owes nothing
    /// to a counter of ours: 266 units of 0.125 ms is 30 Hz, 533 is 15 Hz.
    /// Getting that arithmetic wrong would turn the most trustworthy number
    /// in the line into the least.
    #[test]
    fn the_sensor_frame_step_reads_as_the_sensor_own_rate() {
        let note = |step| {
            let mut m = Metrics::new();
            m.note_exposure(Some((12.4, 1.0, step)));
            m.light_note()
        };
        assert!(note(266).contains("30.1 Hz"), "{}", note(266));
        assert!(note(533).contains("15.0 Hz"), "{}", note(533));
        // Before two frames have arrived there is no step to divide by.
        assert!(note(0).contains('?'), "{}", note(0));
        assert!(note(266).contains("exposure 12.4"), "{}", note(266));

        // A webcam has no such figure; inventing a zero would read as darkness.
        let mut m = Metrics::new();
        m.note_exposure(None);
        assert!(m.light_note().is_empty());
    }

    /// Eight hex characters, and actually varying — an id that collides
    /// across installs would tell us two cabinets are one.
    #[test]
    fn an_install_id_is_eight_hex_characters_and_varies() {
        let a = mint_install_id(1_700_000_000_123_456_789, 4242);
        assert_eq!(a.len(), 8, "{a}");
        assert!(a.chars().all(|c| c.is_ascii_hexdigit()), "{a}");
        assert_ne!(a, mint_install_id(1_700_000_000_123_456_790, 4242));
        assert_ne!(a, mint_install_id(1_700_000_000_123_456_789, 4243));
    }

    /// A session runs from its start marker to the next one, or to the end of
    /// the file if it is the last. That rule is the whole contract, and it has
    /// to hold across a change of log format — which is what broke the
    /// previous scheme.
    #[test]
    fn a_session_ends_where_the_next_one_starts_or_at_the_end_of_the_file() {
        let log = format!(
            "{}{}{}{}{}",
            log_start_marker("0.0.30"),
            // The old format, target-less, as 0.0.30 actually wrote it.
            "2026-08-11T14:50:15Z  INFO headtracking-demo starting version=\"0.0.30\"\n\
             2026-08-11T14:50:16Z  INFO depth 9.9 fps | ir 9.9 fps\n",
            log_start_marker("0.0.38"),
            // The current format, with the target prefix.
            "2026-09-03T08:39:20Z  INFO headtracking_demo: perf IN(cam 29.5 fps)\n",
            // No trailing marker: the last session runs to EOF.
            "2026-09-03T08:39:25Z  INFO headtracking_demo: perf IN(cam 0.0 fps)\n",
        );

        let starts: Vec<_> = log
            .lines()
            .enumerate()
            .filter(|(_, l)| l.starts_with(LOG_START_PREFIX))
            .map(|(i, l)| {
                (
                    i,
                    l.trim_start_matches(LOG_START_PREFIX)
                        .trim_end_matches(LOG_MARKER_SUFFIX)
                        .to_owned(),
                )
            })
            .collect();
        assert_eq!(
            starts.iter().map(|(_, v)| v.as_str()).collect::<Vec<_>>(),
            ["0.0.30", "0.0.38"],
            "both sessions must be found, whatever their log format"
        );

        let lines: Vec<_> = log.lines().collect();
        let bounds: Vec<_> = starts
            .iter()
            .map(|(i, _)| *i)
            .zip(
                starts
                    .iter()
                    .skip(1)
                    .map(|(i, _)| *i)
                    .chain(std::iter::once(lines.len())),
            )
            .collect();
        // The 0.0.30 session must not swallow the 0.0.38 perf lines — that
        // exact merge is what put a depth pipeline on the wrong version.
        assert!(
            !lines[bounds[0].0..bounds[0].1]
                .iter()
                .any(|l| l.contains("cam 29.5"))
        );
        assert!(
            lines[bounds[1].0..bounds[1].1]
                .iter()
                .any(|l| l.contains("cam 0.0"))
        );
    }

    /// The tail header must never be mistaken for a session start: what sits
    /// above the first real marker came from a run we cannot name.
    #[test]
    fn the_tail_header_is_not_a_session_start() {
        let h = log_tail_header("0.0.38", 256 * 1024, true);
        assert!(!h.starts_with(LOG_START_PREFIX), "{h}");
        assert!(h.contains("0.0.38") && h.contains("256 KiB"), "{h}");
        assert!(
            log_tail_header("0.0.38", 256 * 1024, false).contains("whole file"),
            "an untruncated log should say so"
        );
    }

    /// Only `cpu` downgrades. A typo must not silently put someone on the
    /// slow pipeline and leave them wondering why depth crawls.
    #[test]
    fn only_cpu_turns_the_gpu_depth_pipeline_off() {
        assert!(!gpu_depth_allowed_from(Some("cpu")));
        assert!(!gpu_depth_allowed_from(Some("  CPU  ")));
        assert!(gpu_depth_allowed_from(None));
        assert!(gpu_depth_allowed_from(Some("opencl")));
        assert!(gpu_depth_allowed_from(Some("")));
        assert!(gpu_depth_allowed_from(Some("cpuu")));
    }

    /// A backend whose driver cannot count must stay silent rather than
    /// report a zero it never measured.
    #[test]
    fn a_backend_that_cannot_count_reports_nothing_not_zero() {
        let vitals = CaptureVitals::default();
        let (captured, depth, ir, sensor) = vitals.read();
        assert_eq!((captured, depth, ir), (0, 0, 0));
        assert!(sensor.is_none(), "silence is not a measurement of zero");
    }

    /// The regression that made a field report undiagnosable: a Kinect v2 that
    /// delivered nothing over the window printed a bare `0.0 fps`, exactly like
    /// a backend that never had the figure — so the log could not say whether
    /// the sensor had gone quiet or we had stopped reading it.
    #[test]
    fn a_silent_kinect_says_so_instead_of_going_blank() {
        let mut m = Metrics::new();
        // The driver can count, and counted nothing this window.
        m.note_sensor(Some((SensorCounts::default(), SensorCounts::default())));
        m.window_start = std::time::Instant::now() - std::time::Duration::from_secs(2);
        m.tick();
        let note = Metrics::sensor_note(m.sensor_in_fps, m.in_drop_pct);
        assert!(
            note.contains("0.0 sensor"),
            "a driver that can count must report its zero, got {note:?}"
        );

        // Same window, on a backend that cannot report: still nothing to add.
        let mut m = Metrics::new();
        m.note_sensor(None);
        m.window_start = std::time::Instant::now() - std::time::Duration::from_secs(2);
        m.tick();
        assert!(
            Metrics::sensor_note(m.sensor_in_fps, m.in_drop_pct).is_empty(),
            "a webcam has no sensor figure to give"
        );
    }
}
