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
use egui_rotate::{Rotation, RotationPlugin, SoftwareCursor};
use egui_winit::winit;
use nalgebra::Matrix4;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};

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

// Overlay colours, kept here so the toolbar status text and the canvas
// drawing stay in sync. Head → soft red; the anchor geometry → a
// translucent cyan fill, with the lockbar quad derived from its closed
// edge drawn in solid cyan (high contrast against red, visible on both
// bright playfield reflections and dark cabinet interiors).
const LOCKBAR_COLOR: Color32 = Color32::from_rgb(0x00, 0xe5, 0xff);

fn main() {
    // The `windows_subsystem = "windows"` attribute above detaches release
    // builds from any console (no black window on double-click) — but the
    // CLI modes (--capture, --list-cameras…) still need to
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

    // `--pose-test --raw <png> [--depth <png>] [--ir <png>] [--out <png>]`:
    // headless validation of the BlazePose head/pose path on real captured
    // modalities (e.g. a captured raw+depth pair). Runs BlazePose on the
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
        Ok(Some(cap)) => match run_headless_capture(cap) {
            Ok(()) => std::process::exit(0),
            Err(e) => {
                error!(error = %e, "headless capture failed");
                eprintln!("capture failed: {e}");
                std::process::exit(1);
            }
        },
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

  --capture <backend>   Run headless: open backend, settle for `--wait`
                        seconds, save one PNG, exit.
                        backend = kinect-v2 | kinect-v1 | webcam | webcam-<N>
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
    let mut iter = raw.iter();
    while let Some(arg) = iter.next() {
        match arg.as_str() {
            "--capture" => {
                let v = iter.next().ok_or("--capture needs a backend name")?;
                backend = Some(parse_backend_arg(v)?);
            }
            "--out" => {
                let v = iter.next().ok_or("--out needs a path")?;
                out_path = Some(std::path::PathBuf::from(v));
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
    let backend = backend.ok_or("--capture <backend> is required for non-GUI mode")?;
    Ok(Some(CaptureArgs {
        backend,
        out_path,
        wait_secs,
        lockbar_mm,
        pf_deg,
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
/// `wait_secs` so the detectors lock on.
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

    let (w, h, rgb) = active
        .last_rgb_frame
        .as_ref()
        .ok_or_else(|| format!("no RGB frame received in {:.1}s", cap.wait_secs))?;
    let (w, h, rgb) = (*w, *h, rgb.clone());

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
///   * set `HEADTRACKING_LOG=libfreenect=debug,info` so the demo
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
    // undiagnosable otherwise. present=0 usually means the Kinect isn't
    // on the USB bus at all (a v2 without its powered adapter is
    // completely invisible, not even an unknown device).
    info!(
        present,
        missing,
        "windows Kinect driver probe (present=0 → nothing on the USB bus: \
         check the v2 power adapter and use a rear USB 3.0 port; \
         missing>0 → that many Kinect functions still lack WinUSB)"
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
    /// Embedded camera-placement thumbnails (the two setup photos + an
    /// example frame), decoded to textures on first use.
    placement_thumbs: Option<[TextureHandle; 3]>,
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
    last_rgb_frame: Option<(u32, u32, Arc<Vec<u8>>)>,
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
    last_rgb_frame: Option<(u32, u32, Arc<Vec<u8>>)>,
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
    /// Latest registration cost (ms) and a one-shot flag so a failing or
    /// missing registration warns once instead of every frame.
    reg_ms: f32,
    reg_warned: bool,
    /// Which stream the user is viewing. Only used capture-side to skip
    /// building the (2 M pixel) colour-space depth view unless it's on screen.
    selected_stream: StreamKind,
    /// When the colour frame was last polled+converted on the v2 while
    /// tracking on IR — throttles that path to ~2.5 Hz (see the v2 arm).
    last_rgb_refresh: Option<Instant>,
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
                    || self
                        .last_rgb_refresh
                        .is_none_or(|t| t.elapsed() >= Duration::from_millis(400));
                if want_rgb && let Some(rgb) = device.poll_rgb() {
                    self.last_rgb_at = Some(Instant::now());
                    self.last_rgb_refresh = Some(Instant::now());
                    let rgb888 = Arc::new(bgrx_to_rgb888(&rgb.data));
                    if !track_on_ir {
                        got_rgb = true;
                        self.blaze_worker
                            .submit(Arc::clone(&rgb888), rgb.width, rgb.height);
                        self.pose_src = (rgb.width, rgb.height);
                        let pose_out = self.blaze_worker.snapshot();
                        self.last_pose = pose_out.pose;
                        if pose_out.ms > 0.0 {
                            self.head_ms = pose_out.ms;
                        }
                    }
                    self.last_rgb_frame = Some((rgb.width, rgb.height, rgb888));
                }
                // IR streams on the same listener as depth. Keep the latest for
                // the capture export; f32 intensity rounds into u16.
                if let Some(ir) = device.poll_ir() {
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
                        self.blaze_worker.submit(rgb888, ir.width, ir.height);
                        self.pose_src = (ir.width, ir.height);
                        let pose_out = self.blaze_worker.snapshot();
                        self.last_pose = pose_out.pose;
                        if pose_out.ms > 0.0 {
                            self.head_ms = pose_out.ms;
                        }
                    }
                    self.last_ir = Some(Arc::new((ir.width, ir.height, mm)));
                }
                if let Some(depth) = device.poll_depth() {
                    self.depth_frames += 1;
                    self.last_depth_at = Some(Instant::now());
                    // Project this depth frame into colour space. The colour and
                    // depth lenses sit ~5 cm apart with different fields of view,
                    // so scaling a colour coordinate into the 512×424 depth grid
                    // by resolution ratio samples the wrong pixel — worse the
                    // closer the player is. libfreenect2's registration is the
                    // proper correction.
                    // Skipped while tracking on IR: the pose is already in the
                    // depth grid there, so the colour projection buys nothing —
                    // and those milliseconds are exactly the headroom the IR
                    // path exists to gain.
                    // Also gated on a pose (or the depth view being watched):
                    // with nobody in frame there is no head to sample, so the
                    // 2 M-point projection would be pure waste.
                    self.bigdepth_ok = false;
                    if !track_on_ir
                        && (self.last_pose.is_some() || self.selected_stream == StreamKind::Depth)
                        && let Some(reg) = self.registration.as_mut()
                    {
                        let t0 = Instant::now();
                        let ok = reg.bigdepth(&self.rgb_scratch, &depth.data, &mut self.bigdepth);
                        self.reg_ms = t0.elapsed().as_secs_f32() * 1000.0;
                        self.bigdepth_ok = ok;
                        if !ok && !self.reg_warned {
                            self.reg_warned = true;
                            warn!(
                                "kinect v2: depth registration failed — falling back to \
                                 depth-grid scaling for head distance"
                            );
                        }
                    }
                    // Rebuild the colour-space depth view only while it's the
                    // stream on screen: it's a 2 M pixel conversion per frame.
                    self.depth_color = (self.bigdepth_ok
                        && self.selected_stream == StreamKind::Depth)
                        .then(|| Arc::new((1920, 1080, bigdepth_to_mm_u16(&self.bigdepth))));
                    if compute_head {
                        let head = self
                            .last_pose
                            .as_ref()
                            .and_then(|p| {
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
                                } else if self.bigdepth_ok {
                                    head_pixel_from_bigdepth(
                                        p,
                                        &self.bigdepth,
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
                        let smoothed = smooth_head(
                            head,
                            &mut self.pose_filter,
                            &mut self.median_gate,
                            self.started_at,
                            bypass,
                        );
                        capture_baseline(&mut self.baseline, smoothed);
                        self.last_head = smoothed;
                    }
                    self.last_depth = Some(Arc::new((
                        depth.width,
                        depth.height,
                        depth.data.iter().map(|&z| z as u16).collect(),
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
                        self.blaze_worker
                            .submit(Arc::clone(&rgb888), ir.width, ir.height);
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
                        self.last_rgb_frame = Some((ir.width, ir.height, rgb888));
                    }
                } else if let Some(rgb) = device.poll_rgb() {
                    got_rgb = true;
                    self.last_rgb_at = Some(Instant::now());
                    let rgb888 = Arc::new(rgb.data);
                    self.blaze_worker
                        .submit(Arc::clone(&rgb888), rgb.width, rgb.height);
                    self.pose_src = (rgb.width, rgb.height);
                    let pose_out = self.blaze_worker.snapshot();
                    self.last_pose = pose_out.pose;
                    if pose_out.ms > 0.0 {
                        self.head_ms = pose_out.ms;
                    }
                    self.last_rgb_frame = Some((rgb.width, rgb.height, rgb888));
                }
                if let Some(depth) = device.poll_depth() {
                    self.depth_frames += 1;
                    self.last_depth_at = Some(Instant::now());
                    if compute_head {
                        // Sampled straight from the native u16 grid — the old
                        // full-frame u16→f32 widen copied 1.2 MB per frame to
                        // feed a 17×17 window.
                        let head = self.last_pose.as_ref().and_then(|p| {
                            head_pixel_from_pose_depth(
                                p,
                                (640, 480),
                                &depth.data,
                                (depth.width, depth.height),
                                &self.intrinsics,
                                depth_min,
                            )
                        });
                        let smoothed = smooth_head(
                            head,
                            &mut self.pose_filter,
                            &mut self.median_gate,
                            self.started_at,
                            bypass,
                        );
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
                    self.last_rgb_at = Some(Instant::now());
                    let rgb888 = Arc::new(rgb.data);
                    self.blaze_worker
                        .submit(Arc::clone(&rgb888), rgb.width, rgb.height);
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
                        let smoothed = smooth_head(
                            head,
                            &mut self.pose_filter,
                            &mut self.median_gate,
                            self.started_at,
                            bypass,
                        );
                        capture_baseline(&mut self.baseline, smoothed);
                        self.last_head = smoothed;
                    }
                    self.last_rgb_frame = Some((rgb.width, rgb.height, rgb888));
                }
            }
        }
        // Anchor model (RGB): submit the freshest frame until it locks (the
        // worker throttles internally); snapshot the result every call.
        let anchor_frame = self.last_rgb_frame.clone(); // cheap Arc bump
        if let Some((w, h, buf)) = anchor_frame {
            if got_rgb && !self.anchor_worker.is_locked() {
                // Arc bump — the warmup window used to full-frame-copy here,
                // right when the 1280² inference is already at its priciest.
                self.anchor_worker.submit(Arc::clone(&buf), w, h);
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
    fn snapshot_frame(&self, frame_id: u64, captured: u64) -> LatestFrame {
        let (w, h, rgb888) = self
            .last_rgb_frame
            .clone()
            .expect("snapshot_frame with no RGB frame");
        LatestFrame {
            frame_id,
            captured,
            depth_captured: self.depth_frames,
            ir_captured: self.ir_frames,
            w,
            h,
            rgb888,
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
    Depth,
}

impl StreamKind {
    fn label(self) -> &'static str {
        match self {
            StreamKind::Rgb => "RGB",
            StreamKind::Ir => "IR",
            StreamKind::Depth => "Depth",
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
        format!("{} {}×{} {}p", self.kind.label(), self.w, self.h, self.fps)
    }
}

/// Streams each backend advertises. Kinect figures are the fixed sensor modes;
/// the webcam's come from the opened format (SDL only ever gives us one mode).
fn stream_specs(backend: Backend, cam: Option<(u32, u32, u32)>) -> Vec<StreamSpec> {
    let s = |kind, w, h, fps| StreamSpec { kind, w, h, fps };
    match backend {
        Backend::KinectV2 => vec![
            s(StreamKind::Rgb, 1920, 1080, 30),
            s(StreamKind::Ir, 512, 424, 30),
            s(StreamKind::Depth, 512, 424, 30),
        ],
        Backend::KinectV1 => vec![
            s(StreamKind::Rgb, 640, 480, 30),
            s(StreamKind::Ir, 640, 480, 30),
            s(StreamKind::Depth, 640, 480, 30),
        ],
        Backend::Webcam(_) => {
            let (w, h, fps) = cam.unwrap_or((640, 480, 30));
            vec![s(StreamKind::Rgb, w, h, fps)]
        }
        Backend::None => Vec::new(),
    }
}

/// Immutable snapshot the capture thread publishes once per new RGB frame,
/// read by the GL thread through [`CaptureWorker::latest`].
struct LatestFrame {
    /// Increments per published frame; the GL thread uploads only when it changes.
    frame_id: u64,
    /// Cumulative count of captured RGB frames (drives the IN counter delta).
    captured: u64,
    /// Cumulative depth / IR frame counts. Published every RGB frame but
    /// counted independently, so their rates stay correct even when the depth
    /// stream outruns the colour one (v2 in a dark room).
    depth_captured: u64,
    ir_captured: u64,
    w: u32,
    h: u32,
    rgb888: Arc<Vec<u8>>,
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
    /// Cost of the v2 depth↔colour registration for this frame (ms); `0.0`
    /// when it isn't running.
    reg_ms: f32,
    anchor_locked: bool,
}

/// Device I/O the GL thread asks the capture thread to run (things outside the
/// steady poll): the Kinect v1 motor + LED, a baseline reset, and the v1
/// video grabs that may need a momentary mode switch.
enum CaptureCmd {
    SetTilt(f32),
    SetLed(freenect::LedState),
    ResetBaseline,
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
struct CaptureWorker {
    backend: Backend,
    latest: Arc<ArcSwapOption<LatestFrame>>,
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
        let (cmd_tx, cmd_rx) = mpsc::channel();
        let tilt_state = Arc::new(Mutex::new(None));
        let startup = Arc::new(Mutex::new(Startup::Pending));
        let filter_min_cutoff = Arc::new(AtomicU32::new(1.0f32.to_bits()));
        let filter_beta = Arc::new(AtomicU32::new(0.0f32.to_bits()));
        let median_window = Arc::new(AtomicUsize::new(3));
        let bypass = Arc::new(AtomicBool::new(false));
        let stop = Arc::new(AtomicBool::new(false));
        let (latest_t, tilt_t, startup_t, mc_t, beta_t, mw_t, byp_t, stop_t) = (
            Arc::clone(&latest),
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
                    backend, cmd_rx, &latest_t, &tilt_t, &startup_t, &mc_t, &beta_t, &mw_t, &byp_t,
                    &stop_t,
                );
            })
            .expect("spawn capture thread");
        Self {
            backend,
            latest,
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
                            StreamKind::Rgb | StreamKind::Depth => freenect::VideoStream::Rgb,
                        };
                        if device.video_stream() != want {
                            match device.set_video_stream(want) {
                                Ok(()) => info!(?want, "kinect v1: video stream switched"),
                                Err(e) => warn!(?e, ?want, "kinect v1: video switch failed"),
                            }
                        }
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
            latest.store(Some(Arc::new(cap.snapshot_frame(frame_id, captured))));
        } else {
            // No new camera frame — yield briefly so we don't busy-spin.
            std::thread::sleep(Duration::from_millis(1));
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
    rgb888: Arc<Vec<u8>>,
    w: u32,
    h: u32,
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

    fn submit(&self, rgb888: Arc<Vec<u8>>, w: u32, h: u32) {
        *self.job.0.lock() = Some(HeadJob { rgb888, w, h });
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
        let Some(HeadJob { rgb888, w, h }) = job_item else {
            continue;
        };
        last_run = Instant::now();
        let t0 = Instant::now();
        // `poll` = MediaPipe detect-once-then-track: skips the detector while a
        // subject is tracked, so a still skeleton no longer trembles.
        match bp.poll(&rgb888, w, h) {
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
    handle: Option<std::thread::JoinHandle<()>>,
}

impl AnchorWorker {
    fn spawn() -> Self {
        let job = Arc::new((Mutex::new(None::<HeadJob>), Condvar::new()));
        let out = Arc::new(Mutex::new(AnchorOut::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let locked = Arc::new(AtomicBool::new(false));
        let (job_t, out_t, stop_t, locked_t) = (
            Arc::clone(&job),
            Arc::clone(&out),
            Arc::clone(&stop),
            Arc::clone(&locked),
        );
        let handle = std::thread::Builder::new()
            .name("anchor".into())
            .spawn(move || anchor_worker_loop(&job_t, &out_t, &stop_t, &locked_t))
            .expect("spawn anchor thread");
        Self {
            job,
            out,
            stop,
            locked,
            handle: Some(handle),
        }
    }

    /// True once the warmup froze the best detection (the caller stops submitting).
    fn is_locked(&self) -> bool {
        self.locked.load(Ordering::Acquire)
    }

    fn submit(&self, rgb888: Arc<Vec<u8>>, w: u32, h: u32) {
        *self.job.0.lock() = Some(HeadJob { rgb888, w, h });
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
) {
    let mut det = match anchor::AnchorDetector::new() {
        Ok(d) => d,
        Err(e) => {
            warn!("anchor init failed: {e}");
            return;
        }
    };
    // Throttle inference; the cabinet is fixed so a low rate is plenty.
    const INTERVAL: Duration = Duration::from_millis(400);
    // Keep the best-scoring detection for this long after the first hit, then
    // freeze it — the camera + cabinet don't move.
    const WARMUP: Duration = Duration::from_millis(2500);
    let mut last_run = Instant::now();
    let mut warmup_start: Option<Instant> = None;
    let mut best_score = 0.0f32;
    loop {
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
        let Some(HeadJob { rgb888, w, h }) = job_item else {
            continue;
        };
        last_run = Instant::now();
        // Start the warmup clock on the first INFERENCE RUN, not the first
        // detection. The 1280² model on CPU costs ~180 ms; the proof model
        // detects only sporadically on a real scene, so gating the clock (and
        // the lock) on `Some` meant the worker could re-run forever — pinning
        // the CPU and dragging the camera down. The cabinet is fixed, so we run
        // for a bounded warmup then FREEZE regardless.
        let start = *warmup_start.get_or_insert_with(Instant::now);
        let t0 = Instant::now();
        let detn = det.detect(&rgb888, w, h);
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
            while !stop.load(Ordering::Acquire) {
                std::thread::sleep(Duration::from_millis(50));
            }
            return;
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
    fn poll_first_rgb(&self) -> bool {
        match &self.inner {
            Inner::KinectV1 { device, .. } => device.poll_rgb().is_some(),
            Inner::KinectV2 { device, .. } => device.poll_rgb().is_some(),
            Inner::Webcam { camera } => camera.poll_rgb().is_some(),
        }
    }

    /// Bounce the RGB stream to recover from a stalled open: stop+start for the
    /// Kinects, reopen the SDL device for the webcam. Returns the failure
    /// reason if the restart itself errors.
    fn bounce_stream(&mut self) -> Result<(), String> {
        let backend = self.backend;
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
            Inner::Webcam { camera } => {
                let id = match backend {
                    Backend::Webcam(i) => i,
                    _ => 1,
                };
                *camera = webcam::Camera::open(id).map_err(|e| format!("webcam reopen: {e}"))?;
                Ok(())
            }
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

/// Head pixel from a BlazePose landmark sampled in **colour space**, using the
/// registration's `bigdepth` map instead of the raw depth grid.
///
/// This is the accurate path for the Kinect v2: the landmark is already in
/// colour pixels, `bigdepth` is depth expressed in those same pixels, so no
/// cross-sensor mapping is needed at all. Deprojection therefore uses the
/// **colour** intrinsics — passing the IR ones here would reintroduce the very
/// error the registration removes.
///
/// Unmapped pixels come back `+inf` from libfreenect2 (not `0`), so the
/// validity gate checks `is_finite()` before the range test.
///
/// Note the ±8 sampling window is in **colour** pixels here, against ±8
/// *depth* pixels on the legacy path — the same 17×17 pixel box, but colour
/// pixels are ~3.7× finer horizontally, so it covers a physically smaller
/// patch of the subject. That's the point (less background bleeding into the
/// median), but it does mean fewer contributing readings; if head distance
/// ever starts dropping out at range, this window is the knob.
fn head_pixel_from_bigdepth(
    pose: &blazepose::Pose,
    bigdepth: &[f32],
    color: &Intrinsics,
    min_samples: usize,
) -> Option<HeadPixel> {
    if bigdepth.len() < (BIGDEPTH_H + BIGDEPTH_ROW_OFFSET) * BIGDEPTH_W || color.fx <= 0.0 {
        return None;
    }
    let (hx, hy) = head_center_xy(pose);
    let (cx, cy) = (hx as i32, hy as i32);
    let half = 8i32;
    let mut samples: Vec<f32> = Vec::new();
    for dv in -half..=half {
        let v = cy + dv;
        if v < 0 || v >= BIGDEPTH_H as i32 {
            continue;
        }
        // Colour row `v` → bigdepth row `v + 1`.
        let row = (v as usize + BIGDEPTH_ROW_OFFSET) * BIGDEPTH_W;
        for du in -half..=half {
            let u = cx + du;
            if u < 0 || u >= BIGDEPTH_W as i32 {
                continue;
            }
            let z = bigdepth[row + u as usize];
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
        u: hx.max(0.0) as u32,
        v: hy.max(0.0) as u32,
        depth_mm,
        x_mm: (f64::from(hx - color.cx) * zf / f64::from(color.fx)) as f32,
        y_mm: (f64::from(hy - color.cy) * zf / f64::from(color.fy)) as f32,
    })
}

/// Crop `bigdepth` to the 1920×1080 colour window and round to `u16`
/// millimetres, mapping libfreenect2's `+inf` "no reading" to `0` — the same
/// sentinel [`depth_to_turbo_rgb888`] already renders as near-black, so the
/// colour-space depth view reuses the existing colormap unchanged.
fn bigdepth_to_mm_u16(bigdepth: &[f32]) -> Vec<u16> {
    let mut out = vec![0u16; BIGDEPTH_W * BIGDEPTH_H];
    if bigdepth.len() < (BIGDEPTH_H + BIGDEPTH_ROW_OFFSET) * BIGDEPTH_W {
        return out;
    }
    for y in 0..BIGDEPTH_H {
        let src = (y + BIGDEPTH_ROW_OFFSET) * BIGDEPTH_W;
        let dst = y * BIGDEPTH_W;
        for x in 0..BIGDEPTH_W {
            let z = bigdepth[src + x];
            out[dst + x] = if z.is_finite() && z > 0.0 {
                z.min(f32::from(u16::MAX)) as u16
            } else {
                0
            };
        }
    }
    out
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
/// from `/proc/self/stat`. `None` on any parse failure (non-Linux, etc.).
fn read_cpu_jiffies() -> Option<u64> {
    let s = std::fs::read_to_string("/proc/self/stat").ok()?;
    // The `comm` field is parenthesised and may itself contain spaces; every
    // field after the last ')' is plain space-separated. utime/stime are the
    // 12th/13th of those (0-based 11/12).
    let after = &s[s.rfind(')')? + 1..];
    let t: Vec<&str> = after.split_whitespace().collect();
    let utime: u64 = t.get(11)?.parse().ok()?;
    let stime: u64 = t.get(12)?.parse().ok()?;
    Some(utime + stime)
}

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
    in_fps: f32,
    out_fps: f32,
    /// Display repaints per second — the GL thread's own cadence, independent
    /// of the camera. Decoupled from capture by the thread split, so it sits
    /// near the ~60 Hz repaint cap even when `in`/`out` are camera-bound (e.g.
    /// 20 fps webcam). Makes the capture/render decoupling visible.
    render_fps: f32,
    /// Depth / IR capture rates. Diagnostic only — neither is a display rate.
    /// The Kinect streams them from its **own IR illuminator**, so they hold
    /// ~30 Hz in the dark while the auto-exposed colour stream halves to 15:
    /// `ir` staying at 30 while `in` sits at 15 is the proof the ceiling is the
    /// colour camera, not USB bandwidth or our pipeline.
    depth_fps: f32,
    ir_fps: f32,
    cpu_pct: f32,
    in_frames: u32,
    out_frames: u32,
    render_frames: u32,
    depth_frames: u32,
    ir_frames: u32,
    window_start: Instant,
    last_jiffies: u64,
    last_log: Instant,
}

impl Metrics {
    fn new() -> Self {
        let now = Instant::now();
        Self {
            head_ms: 0.0,
            anchor_ms: 0.0,
            reg_ms: 0.0,
            in_fps: 0.0,
            out_fps: 0.0,
            render_fps: 0.0,
            depth_fps: 0.0,
            ir_fps: 0.0,
            cpu_pct: 0.0,
            in_frames: 0,
            out_frames: 0,
            render_frames: 0,
            depth_frames: 0,
            ir_frames: 0,
            window_start: now,
            last_jiffies: read_cpu_jiffies().unwrap_or(0),
            last_log: now,
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
    fn note_reg_ms(&mut self, ms: f32) {
        self.reg_ms = if ms == 0.0 || self.reg_ms == 0.0 {
            ms
        } else {
            self.reg_ms * 0.8 + ms * 0.2
        };
    }
    /// anchor calibration is locked → the detector no longer runs, so report
    /// 0 ms instead of holding the last inference time.
    fn note_anchor_locked(&mut self) {
        self.anchor_ms = 0.0;
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
            let jiffies = read_cpu_jiffies().unwrap_or(self.last_jiffies);
            // USER_HZ is 100 on Linux x86_64; ticks → seconds = / 100.
            let cpu_secs = jiffies.saturating_sub(self.last_jiffies) as f32 / 100.0;
            self.cpu_pct = cpu_secs / elapsed * 100.0;
            self.last_jiffies = jiffies;
            self.in_frames = 0;
            self.out_frames = 0;
            self.render_frames = 0;
            self.depth_frames = 0;
            self.ir_frames = 0;
            self.window_start = now;
        }
        if now.duration_since(self.last_log).as_secs_f32() >= 2.0 {
            info!(
                "perf: head {:.1}ms | anchor {:.1}ms | reg {:.1}ms | cpu {:.0}% | in {:.1} fps | out {:.1} fps | render {:.1} fps | depth {:.1} fps | ir {:.1} fps",
                self.head_ms,
                self.anchor_ms,
                self.reg_ms,
                self.cpu_pct,
                self.in_fps,
                self.out_fps,
                self.render_fps,
                self.depth_fps,
                self.ir_fps
            );
            self.last_log = now;
        }
    }

    /// One-line summary for the toolbar.
    fn summary(&self) -> String {
        // Depth / IR only exist on the Kinects — appended when they're flowing
        // so the webcam read-out stays uncluttered.
        let mut s = format!(
            "head {:.0}ms · anchor {:.0}ms · cpu {:.0}% · in {:.0} / out {:.0} / render {:.0} fps",
            self.head_ms, self.anchor_ms, self.cpu_pct, self.in_fps, self.out_fps, self.render_fps
        );
        if self.reg_ms > 0.0 {
            s.push_str(&format!(" · reg {:.0}ms", self.reg_ms));
        }
        if self.depth_fps > 0.0 {
            s.push_str(&format!(" · depth {:.0}", self.depth_fps));
        }
        if self.ir_fps > 0.0 {
            s.push_str(&format!(" · ir {:.0}", self.ir_fps));
        }
        s
    }
}

impl App {
    fn new(logs: Arc<Mutex<VecDeque<String>>>) -> Self {
        let available = detect_backends();
        let kinect_access_hint = compute_kinect_access_hint();
        Self {
            selected: Backend::None,
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
            placement_thumbs: None,
            switch_state: SwitchState::Idle,
            parallax_enabled: true,
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

    fn poll(&mut self, egui_ctx: &egui::Context) {
        let Some(active) = self.active.as_mut() else {
            return;
        };
        active.metrics.tick();
        // Hand the live-tunable 1€ / bypass knobs to the capture thread (cheap
        // atomics); the device poll + inference now run over there.
        active.worker.set_filter(
            self.head_filter_min_cutoff,
            self.head_filter_beta,
            self.median_window_frames,
            self.bypass_filters,
        );
        // Consume the latest processed frame the capture thread published. IN
        // advances by the cumulative-capture delta (so it counts frames this GL
        // thread never saw); OUT counts only what we upload → `out ≤ in`, with a
        // genuine gap whenever rendering runs slower than capture.
        if let Some(frame) = active.worker.latest.load_full()
            && frame.frame_id != active.last_consumed_id
        {
            let delta = frame.captured.saturating_sub(active.last_captured);
            active.metrics.add_input(delta);
            active.last_captured = frame.captured;
            // Same delta trick for the sensor streams (diagnostic only).
            active
                .metrics
                .add_depth(frame.depth_captured.saturating_sub(active.last_depth_count));
            active
                .metrics
                .add_ir(frame.ir_captured.saturating_sub(active.last_ir_count));
            active.last_depth_count = frame.depth_captured;
            active.last_ir_count = frame.ir_captured;
            active.last_consumed_id = frame.frame_id;
            let img = stream_color_image(&frame, self.selected_stream);
            upload_texture(egui_ctx, &mut active.rgb_texture, img);
            active.metrics.note_output_frame();
            active.last_rgb_at = frame.last_rgb_at;
            active.last_ir_at = frame.last_ir_at;
            active.last_depth_at = frame.last_depth_at;
            active.pose_src = (frame.pose_src_w, frame.pose_src_h);
            active.last_pose = frame.pose.clone();
            active.last_head = frame.head;
            active.baseline = frame.baseline;
            active.last_anchor = frame.anchor;
            active.last_lockbar = frame.lockbar;
            active.last_rgb_frame = Some((frame.w, frame.h, frame.rgb888.clone()));
            active.last_depth = frame.depth.clone();
            active.last_depth_color = frame.depth_color.clone();
            active.last_ir = frame.ir.clone();
            if frame.head_ms > 0.0 {
                active.metrics.note_head_ms(frame.head_ms);
            }
            active.metrics.note_reg_ms(frame.reg_ms);
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
fn smooth_head(
    raw: Option<HeadPixel>,
    filter: &mut filter_alias::OneEuroPose3D,
    gate: &mut headtracking::filter::MedianGate,
    started_at: Instant,
    bypass: bool,
) -> Option<HeadPixel> {
    let head = raw?;
    if bypass {
        return Some(head); // raw pose, no median gate, no 1€ smoothing
    }
    let mut head = head;
    let t_us = started_at.elapsed().as_micros() as u64;
    let gated = gate.push([head.x_mm, head.y_mm, head.depth_mm]);
    let smoothed = filter.update(gated, t_us);
    head.x_mm = smoothed[0];
    head.y_mm = smoothed[1];
    head.depth_mm = smoothed[2];
    Some(head)
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
                    && let Some((w, h, bytes)) = active.last_rgb_frame.as_ref()
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
                        bytes,
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
                                    RichText::new(format!(
                                        "baseline (mm)  ({:>6.0}, {:>6.0}, {:>6.0})\n\
                                         current  (mm)  ({:>6.0}, {:>6.0}, {:>6.0})\n\
                                         Δ pose   (mm)  ({:>+6.0}, {:>+6.0}, {:>+6.0})\n\n\
                                         Δ view  (VPU)  ({:>+6.2}, {:>+6.2}, {:>+6.2})\n\
                                                       viewX += {:>+6.2}\n\
                                                       viewY += {:>+6.2}\n\
                                                       viewZ += {:>+6.2}",
                                        base.x_mm,
                                        base.y_mm,
                                        base.z_mm,
                                        head.x_mm,
                                        head.y_mm,
                                        head.depth_mm,
                                        dx_mm,
                                        dy_mm,
                                        dz_mm,
                                        vx,
                                        vy,
                                        vz,
                                        vx,
                                        vy,
                                        vz,
                                    ))
                                    .monospace()
                                    .size(15.0),
                                );
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
                ui.toggle_value(&mut self.parallax_invert[0], "±X")
                    .on_hover_text("Flip the left/right axis.");
                ui.toggle_value(&mut self.parallax_invert[1], "±Y")
                    .on_hover_text("Flip the up/down axis.");
                ui.toggle_value(&mut self.parallax_invert[2], "±Z")
                    .on_hover_text("Flip the depth axis (closer/farther).");
            });
        });
    }

    /// Camera-placement guidance shown at the top of the central panel, above
    /// the live view: a one-line reminder + three example thumbnails (a good
    /// frame, and two mounting shots).
    fn camera_placement_help(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        // Order: the two mounting shots first, then what the camera should see.
        let thumbs = self.placement_thumbs.get_or_insert_with(|| {
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

    /// Stream bar, directly above the camera image: one chip per stream the
    /// device offers, captioned with its nominal spec, **green when frames are
    /// arriving and red when they aren't**, and clickable to display it.
    ///
    /// This is where the Kinect v1's single video endpoint explains itself:
    /// select IR and the colour chip goes red while IR goes green, so the
    /// either/or is visible rather than documented.
    fn stream_bar(&mut self, ui: &mut egui::Ui) {
        let Some(active) = self.active.as_ref() else {
            return;
        };
        let specs = stream_specs(active.backend, active.cam_spec);
        if specs.is_empty() {
            return;
        }
        // Aged here, at draw time — see [`Active::last_rgb_at`].
        let live = |t: Option<Instant>| t.is_some_and(|t| t.elapsed() < STREAM_LIVE_FOR);
        let (rgb_live, ir_live, depth_live) = (
            live(active.last_rgb_at),
            live(active.last_ir_at),
            live(active.last_depth_at),
        );
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
                    StreamKind::Rgb => rgb_live,
                    StreamKind::Ir => ir_live,
                    StreamKind::Depth => depth_live,
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
            self.selected_stream = kind;
            // The v1 has to physically switch its video endpoint; the others
            // just need to know what's on screen.
            if let Some(active) = self.active.as_ref() {
                let _ = active.worker.cmd_tx.send(CaptureCmd::SelectStream(kind));
            }
        }
        ui.add_space(2.0);
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
                if sw > 0.0 && sh > 0.0 {
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
        // Line 2 — the anchor-derived lockbar, on its own row below.
        if let Some(bar) = active.last_lockbar {
            ui.horizontal(|ui| {
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
            });
        }
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
        self.camera_pose_line(ui, active);
    }

    /// One-line camera-pose read-out under the VPX delta: where the camera
    /// sits relative to the cab, computed by the validated `anchor::camera_pose`
    /// port from the **locked** anchor geometry + the colour intrinsics + the
    /// lockbar-width slider. The webcam has no factory focal (nominal guess
    /// until autocalib), so its line is flagged with "≈".
    fn camera_pose_line(&self, ui: &mut egui::Ui, active: &Active) {
        if !active.anchor_locked {
            return;
        }
        let (Some(geom), Some((fw, fh, _))) =
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
        let approx = if matches!(active.backend, Backend::Webcam(_)) {
            "≈ " // nominal focal — estimate, not a measurement
        } else {
            ""
        };
        let metres = format!("{:.2}", pose.distance_mm / 1000.0);
        ui.label(
            RichText::new(format!(
                "{approx}Camera: {metres} m from the lockbar · {:.0} cm above · offset {:+.0} cm · tilted {:.0}°",
                pose.height_mm / 10.0,
                pose.lateral_mm / 10.0,
                pose.pitch_deg,
            ))
            .monospace()
            .color(Color32::GRAY),
        );
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
        bigdepth: Vec::new(),
        rgb_scratch: Vec::new(),
        bigdepth_ok: false,
        reg_ms: 0.0,
        reg_warned: false,
        selected_stream: StreamKind::Rgb,
        last_rgb_refresh: None,
        depth_color: None,
        pose_src: (0, 0),
        head_ms: 0.0,
        anchor_ms: 0.0,
    }
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
    let device = ctx
        .open_default()
        .map_err(|e| format!("freenect2 open_default: {e}"))?;
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
    let mut out = Vec::with_capacity(bgrx.len() / 4 * 3);
    for chunk in bgrx.chunks_exact(4) {
        out.push(chunk[2]); // R from BGRX
        out.push(chunk[1]); // G
        out.push(chunk[0]); // B
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
        StreamKind::Depth => {
            // Prefer the colour-space projection when the v2 registration
            // produced one: it shares the colour framing, so the pose and
            // anchor overlays land exactly, with no lens-parallax offset.
            // v1 / webcam (and any v2 frame before the registration ran) fall
            // back to the sensor's native depth grid.
            if let Some(d) = frame.depth_color.as_deref().or(frame.depth.as_deref()) {
                let (w, h, mm) = d;
                return rgb888_to_color_image(*w, *h, &depth_to_turbo_rgb888(mm));
            }
        }
        StreamKind::Rgb => {}
    }
    rgb888_to_color_image(frame.w, frame.h, &frame.rgb888)
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
/// This is only ever the human-readable rendering — the depth the pipeline
/// consumes stays the raw 16-bit millimetres.
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

/// Encode an RGB888 buffer to PNG bytes in memory (the screenshot path
/// writes straight to a file instead).
fn png_bytes(width: u32, height: u32, rgb888: &[u8]) -> Result<Vec<u8>, String> {
    png_bytes_meta(width, height, rgb888, &[])
}

/// Like [`png_bytes`] but embeds `meta` as PNG `tEXt` chunks, so a saved
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

/// Build the tracking read-out embedded into a saved capture as PNG
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

/// Stretch `samples` to the full 0–255 range and return the raw grayscale
/// bytes. Used for the live IR view: raw sensor intensity spans a fraction of
/// the 16-bit range, so without this the panel shows near-black.
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
/// and stop widgets from resizing on hover (the ~1 px expansion looked odd).
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
/// algorithms saw. Used by the screenshot button and `--pose-test`.
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
        .detect(img.as_raw(), w, h)
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

    // --- Anchor model (RGB): cabinet frame → lateral / sidebars→∞ / width ---
    let anchor_geo = match anchor::AnchorDetector::new() {
        Ok(mut det) => match det.detect(img.as_raw(), w, h) {
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
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info"));

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
    let file_layer = open_log_file().map(|f| {
        tracing_subscriber::fmt::layer()
            .with_target(false)
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
        .with(stderr_layer)
        .with(file_layer)
        .with(panel_layer)
        .init();
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
