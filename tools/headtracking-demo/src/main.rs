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
use std::time::{Duration, Instant};

use std::num::NonZeroU32;

use egui::{
    self, Align, CentralPanel, Color32, ColorImage, ComboBox, Layout, Panel, Pos2, Rect, RichText,
    ScrollArea, Sense, Stroke, TextureHandle, Vec2,
};
use egui_rotate::{Rotation, RotationPlugin, SoftwareCursor};
use egui_winit::winit;
use nalgebra::Matrix4;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use parking_lot::{Condvar, Mutex};
use tracing::{error, info, warn};
use tracing_subscriber::fmt::MakeWriter;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;

const DEPTH_MIN_MM: f32 = 500.0;
const DEPTH_MAX_MM: f32 = 2_500.0;

const LOG_BUFFER_LINES: usize = 1_000;

mod contribute;

/// Privacy-notice bullets for the Share-a-capture window (title, body).
const CONTRIB_TERMS: &[(&str, &str)] = &[
    (
        "Sole use",
        "to train and improve the pincab head-tracking model. Nothing else.",
    ),
    (
        "Private storage",
        "each capture is saved locally in the demo's contributions/ folder (your own \
         copy) and uploaded to the maintainer's private, write-only server. On the \
         server no one — not even you — can list, read or download anything; only the \
         maintainer sees the uploads.",
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
// drawing stay in sync. Head → soft red; the U segmentation mask → a
// translucent cyan fill, with the lockbar quad derived from its closed
// edge drawn in solid cyan (high contrast against red, visible on both
// bright playfield reflections and dark cabinet interiors).
const LOCKBAR_COLOR: Color32 = Color32::from_rgb(0x00, 0xe5, 0xff);
/// Sidebars (playfield rails) — orange, distinct from the cyan lockbar.
const RAIL_COLOR: Color32 = Color32::from_rgb(0xff, 0x9a, 0x00);

fn main() {
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

    // Hidden `--upload-test`: exercise the contribution upload path (ureq /
    // rustls / auth / write-only drop) end to end without the GUI, then exit.
    if std::env::args().any(|a| a == "--upload-test") {
        let uploader = contribute::Uploader::spawn();
        let name = format!("{}_uploadtest.txt", contribution_stem(Backend::None));
        println!("upload-test: PUT {name}");
        uploader.submit(name.clone(), b"headtracking-demo upload test\n".to_vec());
        for _ in 0..100 {
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
            .set_swap_interval(
                &gl_context,
                glutin::surface::SwapInterval::Wait(NonZeroU32::MIN),
            )
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
/// wireframe "shadow box" and three depth layers of target points. Each
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
    /// Wireframe shadow box (`GL_LINES`) and solid 3D cube markers
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
            let (pts_vao, pts_vbo, pts_count) = upload_mesh(gl, &parallax_cube_mesh(aspect0));

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
                let tm = parallax_cube_mesh(aspect);
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

            // Wireframe shadow box (lines) + solid 3D cube markers (triangles).
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

/// Three 3×3 grids of solid **3D cubes** (`GL_TRIANGLES`) at increasing
/// depth, warm (near) → cool (far). Each face gets a fixed shade (fake
/// top-light) baked into its vertex colour, so the cubes read as volumes
/// without a lighting pass — far clearer than flat points/squares, and the
/// faces reveal 3D as the eye moves. Positions span the panel aspect; the
/// cube size is fixed (so perspective scales them near→far). Depth-tested,
/// no culling needed.
fn parallax_cube_mesh(aspect: f32) -> Vec<f32> {
    let layers = [
        (-150.0f32, [1.0f32, 0.62, 0.25]), // near, warm
        (-400.0, [0.45, 0.90, 0.45]),      // mid, green
        (-800.0, [0.40, 0.62, 1.00]),      // far, cool
    ];
    let (hw, hh) = parallax_screen_half(aspect);
    let (tx, ty) = (hw * 0.66, hh * 0.66);
    let hsz = 30.0f32; // cube half-size, mm (fixed)
    let n = 3i32;
    let mut v: Vec<f32> = Vec::new();
    // Two triangles (a,b,c)+(a,c,d) for a quad face, all in one colour.
    let mut quad = |a: [f32; 3], b: [f32; 3], c: [f32; 3], d: [f32; 3], col: [f32; 3]| {
        for p in [a, b, c, a, c, d] {
            v.extend_from_slice(&[p[0], p[1], p[2], col[0], col[1], col[2]]);
        }
    };
    for (z, base) in layers {
        for iy in 0..n {
            for ix in 0..n {
                let cx = (ix as f32 / (n - 1) as f32 * 2.0 - 1.0) * tx;
                let cy = (iy as f32 / (n - 1) as f32 * 2.0 - 1.0) * ty;
                let p = |sx: f32, sy: f32, sz: f32| [cx + sx * hsz, cy + sy * hsz, z + sz * hsz];
                let shade = |s: f32| [base[0] * s, base[1] * s, base[2] * s];
                // Fake top-light: brightest top, darkest bottom.
                quad(
                    p(-1., -1., 1.),
                    p(1., -1., 1.),
                    p(1., 1., 1.),
                    p(-1., 1., 1.),
                    shade(0.88),
                ); // +Z front
                quad(
                    p(1., -1., -1.),
                    p(-1., -1., -1.),
                    p(-1., 1., -1.),
                    p(1., 1., -1.),
                    shade(0.50),
                ); // -Z back
                quad(
                    p(-1., 1., 1.),
                    p(1., 1., 1.),
                    p(1., 1., -1.),
                    p(-1., 1., -1.),
                    shade(1.0),
                ); // +Y top
                quad(
                    p(-1., -1., -1.),
                    p(1., -1., -1.),
                    p(1., -1., 1.),
                    p(-1., -1., 1.),
                    shade(0.40),
                ); // -Y bottom
                quad(
                    p(1., -1., 1.),
                    p(1., -1., -1.),
                    p(1., 1., -1.),
                    p(1., 1., 1.),
                    shade(0.70),
                ); // +X right
                quad(
                    p(-1., -1., -1.),
                    p(-1., -1., 1.),
                    p(-1., 1., 1.),
                    p(-1., 1., -1.),
                    shade(0.60),
                ); // -X left
            }
        }
    }
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
        let full_output = ctx.run_ui(raw_input, |ui| app.ui(ui));

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
        for (id, image_delta) in &full_output.textures_delta.set {
            painter.set_texture(*id, image_delta);
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

        self.gl_window.as_ref().unwrap().swap_buffers().unwrap();
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

fn run_headless_capture(cap: CaptureArgs) -> Result<(), String> {
    info!(
        backend = ?cap.backend,
        wait_secs = cap.wait_secs,
        out = ?cap.out_path,
        "headless capture starting"
    );
    let mut active = open_backend(cap.backend)?;
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
                backend_slug(cap.backend)
            ));
        }
        warn!(backend = ?cap.backend, next = bounce + 1, "headless: no first frame, bouncing stream");
        active.bounce_stream()?;
    }
    let deadline = Instant::now() + Duration::from_secs_f32(cap.wait_secs.max(0.1));
    while Instant::now() < deadline {
        poll_active_headless(&mut active);
        std::thread::sleep(Duration::from_millis(30));
    }

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
    // Autocalibration + depth ground-truth (the lockbar homography focal/pose,
    // and the measured Kinect depth at the bar).
    meta.extend(autocalib_meta(
        active.last_lockbar.as_ref(),
        (w, h),
        cap.lockbar_mm,
        active.last_depth.as_ref(),
    ));
    // Surface the lockbar-derived geometry on stdout so an SSH-driven capture
    // round can read the cam↔bar distance + camera offset without opening the
    // PNG. `ht_lockbar` == "none" here means U-seg never locked the bar.
    for key in [
        "ht_lockbar",
        "ht_lockbar_width_px",
        "ht_color_fx",
        "ht_lockbar_dist_mm",
        "ht_lockbar_center_px",
        "ht_cam_offset_x_mm",
        "ht_cam_offset_y_mm",
        "ht_lockbar_slope_deg",
        "ht_autocalib_fx",
        "ht_autocalib_dist_mm",
        "ht_autocalib_vp_fx",
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
            active.last_lockbar.as_ref(),
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
            active.last_lockbar.as_ref(),
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

/// Same as `App::poll` minus the egui texture upload (we don't render
/// anything in headless mode) and the depth → head-pose path (the
/// screenshot only cares about RGB + head boxes + lockbar quad).
fn poll_active_headless(active: &mut Active) {
    match &mut active.inner {
        Inner::KinectV2 { device, .. } => {
            if let Some(rgb) = device.poll_rgb() {
                let rgb888 = bgrx_to_rgb888(&rgb.data);
                active
                    .blaze_worker
                    .submit(rgb888.clone(), rgb.width, rgb.height);
                active.last_pose = active.blaze_worker.snapshot().pose;
                active
                    .u_worker
                    .submit(rgb888.clone(), rgb.width, rgb.height);
                let u_out = active.u_worker.snapshot();
                active.last_u = u_out.u;
                active.last_lockbar = u_out.lockbar;
                active.last_rgb_frame = Some((rgb.width, rgb.height, rgb888));
            }
            // Depth too, so the capture can cross-check the lockbar distance.
            if let Some(depth) = device.poll_depth() {
                active.last_depth = Some((
                    depth.width,
                    depth.height,
                    depth.data.iter().map(|&z| z as u16).collect(),
                ));
            }
        }
        Inner::KinectV1 { device, .. } => {
            if let Some(rgb) = device.poll_rgb() {
                active
                    .blaze_worker
                    .submit(rgb.data.clone(), rgb.width, rgb.height);
                active.last_pose = active.blaze_worker.snapshot().pose;
                active
                    .u_worker
                    .submit(rgb.data.clone(), rgb.width, rgb.height);
                let u_out = active.u_worker.snapshot();
                active.last_u = u_out.u;
                active.last_lockbar = u_out.lockbar;
                active.last_rgb_frame = Some((rgb.width, rgb.height, rgb.data));
            }
            if let Some(depth) = device.poll_depth() {
                active.last_depth = Some((depth.width, depth.height, depth.data.clone()));
            }
        }
        Inner::Webcam { camera } => {
            if let Some(rgb) = camera.poll_rgb() {
                active
                    .blaze_worker
                    .submit(rgb.data.clone(), rgb.width, rgb.height);
                active.last_pose = active.blaze_worker.snapshot().pose;
                active
                    .u_worker
                    .submit(rgb.data.clone(), rgb.width, rgb.height);
                let u_out = active.u_worker.snapshot();
                active.last_u = u_out.u;
                active.last_lockbar = u_out.lockbar;
                active.last_rgb_frame = Some((rgb.width, rgb.height, rgb.data));
            }
        }
    }
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

#[derive(Debug, Clone)]
struct BackendEntry {
    backend: Backend,
    label: String,
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
    let mut out = vec![BackendEntry {
        backend: Backend::None,
        label: "None (off)".to_string(),
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
                });
            }
        }
        Err(e) => warn!(?e, "scan: webcam enumerate failed"),
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
    // v2 = sensor 02C4, firmware-update 02D8, NuiSensor Adaptor 02D9.
    const SCRIPT: &str = "\
        $ms = 'VID_045E&PID_(02AE|02BF|02AD|02BE|02B0|02C2|02C4|02D8|02D9)'; \
        $d = @(Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | \
               Where-Object { $_.InstanceId -match $ms }); \
        $drv = @($d | Where-Object { $_.Service -in 'WinUSB','WinUsb','libusbK','libusb0' }); \
        Write-Output (\"present={0} winusb={1}\" -f $d.Count, $drv.Count)";
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
    let winusb = field("winusb=");
    present > 0 && winusb == 0
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
    // `powershell.exe -ExecutionPolicy Bypass -File setup.ps1`. The
    // RunAs verb is what pops the UAC consent dialog; the spawned
    // elevated PowerShell opens its own visible console (we don't pass
    // `-WindowStyle Hidden`) so the user can read the script's output
    // and the type-`yes` prompt. `-ExecutionPolicy Bypass` is required
    // because a fresh Windows defaults to Restricted/RemoteSigned and
    // would otherwise refuse our unsigned local script.
    //
    // We use this trampoline instead of a direct Win32 ShellExecuteW
    // call to keep the dependency tree std-only — the price is one
    // ephemeral PowerShell process (~100 ms) before UAC fires.
    let inner = format!(
        "$ErrorActionPreference='Stop'; Start-Process powershell -Verb RunAs \
         -WorkingDirectory '{workdir}' \
         -ArgumentList '-NoProfile','-ExecutionPolicy','Bypass','-File','{script}'",
        workdir = workdir.display(),
        script = script.display(),
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
         powershell -NoProfile -ExecutionPolicy Bypass -File \"{}\"",
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
    /// Settle done; showing "opening" before the actual open call.
    Opening(Instant),
    /// Device opened; polling for the first RGB frame. If none arrives within
    /// [`FIRST_FRAME_WAIT`] the stream is bounced (up to [`MAX_STREAM_BOUNCES`]
    /// times, shown as "retry n/3") before the open is declared failed.
    Waiting {
        active: Box<Active>,
        /// Bounce attempts so far (0 = first wait, no bounce yet).
        bounces: u8,
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
    /// Target head-detection rate (Hz). The detector is capped to this so it
    /// stops pinning a full core running flat-out; the 1€ filter interpolates
    /// between detections. Live-tunable from the bench row.
    head_detect_hz: f32,
    /// Debug: bypass everything between raw detection and the pose — the 1€
    /// filter, the lockbar-centred picker (→ largest head), and most of the
    /// depth-sample gate — to see which stage is dropping the head.
    bypass_filters: bool,
    /// "Share a capture" window toggle + state.
    contribute_open: bool,
    /// The informed-consent checkbox (see the privacy notice). Gates the
    /// share button; in-memory for the session.
    consent_checked: bool,
    /// Background uploader for shared captures (write-only Nextcloud drop).
    uploader: contribute::Uploader,
    /// Stem of the last capture shared, shown so the user can note it (needed
    /// to request a removal, since the drop is anonymous).
    contrib_last: Option<String>,
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
        let fps = self.active.as_ref().map_or(0.0, |a| a.metrics.in_fps);
        if fps > 1.0 {
            Duration::from_secs_f32(1.0 / (fps * 1.3))
        } else {
            Duration::from_secs_f32(1.0 / 60.0)
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
    intrinsics: Intrinsics,
    rgb_texture: Option<TextureHandle>,
    /// 1€-smoothed head pose. Same shape as the raw `HeadPixel` but the
    /// position values come out of [`OneEuroPose3D`] so the crosshair and
    /// the VPX delta panel stop jittering at the pixel level.
    last_head: Option<HeadPixel>,
    baseline: Option<Baseline>,
    inner: Inner,
    /// `Some` only when [`Inner::KinectV1`] — drives the motorised base.
    v1_controls: Option<V1Controls>,
    pose_filter: filter_alias::OneEuroPose3D,
    started_at: Instant,
    /// Lockbar quad consumed by the overlay + 3D-centre maths. No longer
    /// detected directly: it's *derived* from the closed edge of the U
    /// segmentation (see [`u_to_lockbar_quad`]) — the closed bar of the U
    /// is the lockbar, of known physical width.
    last_lockbar: Option<headtracking::calibration::LockbarQuadRgb>,
    /// Latest raw U-seg detection (mask + box) — backs the translucent
    /// mask overlay and the derived [`last_lockbar`]. Snapshotted from the
    /// [`u_worker`] each poll.
    last_u: Option<u_onnx::UDetection>,
    /// Background thread running the heavy (~330 ms) U-seg detector off the
    /// UI thread. The poll loop submits the latest colour frame and reads
    /// [`last_lockbar`] / [`last_u`] back from its snapshot — so the U warmup
    /// no longer hitches rendering (see [`UWorker`]).
    u_worker: UWorker,
    /// Latest RGB888 frame (width, height, bytes) — kept so the
    /// "Screenshot" button can write it to disk without re-grabbing
    /// from the device. `None` until the first frame arrives.
    last_rgb_frame: Option<(u32, u32, Vec<u8>)>,
    /// Latest depth frame (width, height, millimetres) — kept so the
    /// "Share a capture" button can export it alongside the RGB. `None` for
    /// the webcam backend (no depth) and until the first depth poll. Stored
    /// as `u16` mm (v1 is native `u16`; v2's `f32` mm is rounded on capture).
    last_depth: Option<(u32, u32, Vec<u16>)>,
    /// Latest IR frame (width, height, intensity as `u16`) — Kinect v2 only,
    /// where IR streams alongside depth on the same listener. `None` for v1
    /// (its IR needs a momentary video-mode switch, done on demand in
    /// [`DemoShell::share_capture`]) and the webcam.
    last_ir: Option<(u32, u32, Vec<u16>)>,
    /// BlazePose worker (pose/head) — replaces the RGB head net + depth blob +
    /// silhouette skeleton with one model read straight off the frame. Its 33
    /// landmarks drive the head crosshair and (later) the hands↔lockbar player
    /// selection so only the person holding the flipper is tracked.
    blaze_worker: BlazePoseWorker,
    last_pose: Option<blazepose::Pose>,
    /// Live perf counters (inference times, CPU%, in/out FPS). Reset per
    /// backend open — each device gets a fresh measurement window.
    metrics: Metrics,
}

/// How long a U detection stays valid before the [`UWorker`] re-runs
/// inference. The playfield doesn't move while the camera is mounted, so
/// refreshing every ~1.5 s gives the overlay a live feel without paying
/// the ~330 ms tract cost on every frame.
const U_RECOMPUTE_INTERVAL: Duration = Duration::from_millis(1500);

/// Window between the first successful U detection and freezing the best
/// one — at ~1.5 s cadence we collect 3-4 candidate masks and keep the
/// highest-confidence one. After this elapses, the [`UWorker`] stops running
/// until a backend switch respawns it.
const U_WARMUP_DURATION: Duration = Duration::from_millis(2500);

/// Latest RGB frame handed to the [`UWorker`]. Only the most recent one
/// matters — the worker overwrites any still-unprocessed job.
struct UJob {
    rgb888: Vec<u8>,
    w: u32,
    h: u32,
}

/// What the [`UWorker`] publishes after each round it runs.
#[derive(Clone, Default)]
struct UOut {
    u: Option<u_onnx::UDetection>,
    lockbar: Option<headtracking::calibration::LockbarQuadRgb>,
    /// Last U inference time (ms); `0.0` until the first round completes.
    u_ms: f32,
}

/// Runs the heavy (~330 ms) U-seg detector off the UI thread. The UI submits
/// the latest colour frame each poll and reads the newest lockbar back via
/// [`UWorker::snapshot`] without ever blocking on inference — so the U warmup
/// no longer hitches rendering. The best-of-warmup auto-lock lives here now
/// (the cabinet camera is fixed, so freezing the lockbar after warmup is
/// correct and frees the core); once locked the worker just idles.
struct UWorker {
    job: Arc<(Mutex<Option<UJob>>, Condvar)>,
    out: Arc<Mutex<UOut>>,
    stop: Arc<AtomicBool>,
    /// Set once the warmup best-of lock has frozen the lockbar; the worker then
    /// idles, so callers can stop feeding it frames (and skip the colour
    /// conversion that only fed it).
    locked: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl UWorker {
    fn spawn() -> Self {
        let job = Arc::new((Mutex::new(None::<UJob>), Condvar::new()));
        let out = Arc::new(Mutex::new(UOut::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let locked = Arc::new(AtomicBool::new(false));
        let (job_t, out_t, stop_t, lock_t) = (
            Arc::clone(&job),
            Arc::clone(&out),
            Arc::clone(&stop),
            Arc::clone(&locked),
        );
        let handle = std::thread::Builder::new()
            .name("u-detector".into())
            .spawn(move || u_worker_loop(&job_t, &out_t, &stop_t, &lock_t))
            .expect("spawn u-detector thread");
        Self {
            job,
            out,
            stop,
            locked,
            handle: Some(handle),
        }
    }

    /// Whether the warmup lock has fired (worker now idle — no need to submit).
    fn is_locked(&self) -> bool {
        self.locked.load(Ordering::Acquire)
    }

    /// Hand the worker the latest colour frame, replacing any pending one.
    fn submit(&self, rgb888: Vec<u8>, w: u32, h: u32) {
        *self.job.0.lock() = Some(UJob { rgb888, w, h });
        self.job.1.notify_one();
    }

    /// Newest lockbar / U-seg the worker has produced. Never blocks on
    /// inference — returns whatever the last completed round published.
    fn snapshot(&self) -> UOut {
        self.out.lock().clone()
    }
}

impl Drop for UWorker {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        self.job.1.notify_one();
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// Body of the U-detector thread: the same best-of-warmup auto-lock as before,
/// just off the UI thread. Waits for a frame, throttles to
/// [`U_RECOMPUTE_INTERVAL`], runs inference, keeps the highest-confidence U,
/// and freezes after [`U_WARMUP_DURATION`].
fn u_worker_loop(
    job: &Arc<(Mutex<Option<UJob>>, Condvar)>,
    out: &Arc<Mutex<UOut>>,
    stop: &Arc<AtomicBool>,
    locked_flag: &Arc<AtomicBool>,
) {
    let mut detector: Option<u_onnx::UDetector> = None;
    let mut last_run_at: Option<Instant> = None;
    let mut first_detection_at: Option<Instant> = None;
    let mut locked = false;
    let mut best_conf = 0.0f32;
    let mut best_score = 0.0f32;

    loop {
        // Block until a frame arrives (or we're asked to stop).
        let item = {
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
        let Some(item) = item else { continue };
        if locked {
            continue; // calibration frozen — drop the frame.
        }
        let now = Instant::now();
        if let Some(last) = last_run_at
            && now.duration_since(last) < U_RECOMPUTE_INTERVAL
        {
            continue;
        }
        if detector.is_none() {
            match u_onnx::UDetector::new() {
                Ok(mut d) => {
                    // Below the 0.25 default so the lockbar fires on the v1
                    // 640×480 RGB (see the score history in git); on v2 the
                    // strong lockbar still wins the best-of.
                    d.set_score_threshold(0.10);
                    info!("U-seg detector initialised (score_threshold=0.10)");
                    detector = Some(d);
                }
                Err(e) => {
                    warn!(?e, "U-seg detector init failed");
                    last_run_at = Some(now);
                    continue;
                }
            }
        }
        let det = detector.as_ref().expect("init checked above");
        last_run_at = Some(now);
        let t0 = Instant::now();
        let dets = det.detect(&item.rgb888, item.w, item.h);
        let u_ms = t0.elapsed().as_secs_f32() * 1000.0;
        if let Some(best) = dets.into_iter().next() {
            // Derive the U geometry, then keep the frame with the best *lock
            // score* — a full 3-axis U (lockbar + both sidebars) beats any
            // lockbar-only frame, so a frame where an arm merely fakes the bar
            // can never win the calibration (see `u_lock_score`).
            let quad = u_to_lockbar_quad(&best, det.mask_threshold(), item.w, item.h);
            let score = quad
                .as_ref()
                .map_or(0.0, |q| u_lock_score(q, best.confidence));
            if score > best_score {
                best_score = score;
                best_conf = best.confidence;
                let both_rails = quad
                    .as_ref()
                    .is_some_and(|q| q.left_rail.is_some() && q.right_rail.is_some());
                let mut o = out.lock();
                if let Some(quad) = quad {
                    o.lockbar = Some(quad);
                }
                o.u = Some(best);
                o.u_ms = u_ms;
                drop(o);
                info!(
                    score = best_score,
                    conf = best_conf,
                    three_axes = both_rails,
                    "U: new best detection"
                );
                // Start the warmup window from the first usable geometry, so
                // the lock freezes a short moment after we first see the U.
                if first_detection_at.is_none() && score > 0.0 {
                    first_detection_at = Some(now);
                }
            } else {
                out.lock().u_ms = u_ms;
            }
        } else {
            out.lock().u_ms = u_ms;
        }
        if let Some(first) = first_detection_at
            && now.duration_since(first) >= U_WARMUP_DURATION
        {
            locked = true;
            locked_flag.store(true, Ordering::Release);
            info!(best_conf, "U: warmup over, calibration locked");
        }
    }
}

// ---------------------------------------------------------------- Head worker

/// Latest RGB frame handed to the [`HeadWorker`]. Only the most recent one
/// matters — the worker overwrites any still-unprocessed job.
struct HeadJob {
    rgb888: Vec<u8>,
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

    fn submit(&self, rgb888: Vec<u8>, w: u32, h: u32) {
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
        match bp.detect(&rgb888, w, h) {
            Ok(pose) => {
                let ms = t0.elapsed().as_secs_f32() * 1000.0;
                let mut o = out.lock();
                o.pose = pose;
                o.ms = ms;
            }
            Err(e) => warn!("blazepose detect: {e}"),
        }
    }
}

/// Score a candidate U for the best-of-warmup lock. The camera and lockbar
/// are fixed, so we freeze the single best frame — and "best" must mean the
/// most complete real-world reference, NOT the highest raw mask confidence.
/// A full **3-axis** U (lockbar + both sidebars) dominates any partial one, so
/// a frame where a player's arm fakes a bright bar (but yields no proper rails
/// on the open side) can never win the calibration. Ties: total rail length,
/// then U confidence.
fn u_lock_score(q: &headtracking::calibration::LockbarQuadRgb, conf: f32) -> f32 {
    // NOTE: do NOT reward rail *length* — on this overhead camera the longest
    // "rails" are the players' bodies standing behind the bar. Rails are now
    // band-limited + corner-attached at extraction, so their mere presence is
    // the signal; U confidence breaks ties.
    let axes_bonus = match (q.left_rail.is_some(), q.right_rail.is_some()) {
        (true, true) => 10_000.0, // the 3 axes — the frame we want to lock
        (true, false) | (false, true) => 1_000.0,
        (false, false) => 0.0, // lockbar only
    };
    axes_bonus + conf * 100.0
}

/// Derive the lockbar quad — the *closed* bar of the U, of known physical
/// width — from a U-seg detection. We don't assume it sits at a fixed
/// image edge: with the camera mounted above the backglass the U reads as
/// an inverted ∩, closed end near the top. So we scan the proto mask row
/// by row; the closed bar is the band of rows that are *filled across*,
/// sitting at one vertical extremity, while the open rails are the rows
/// where only two thin runs survive (low fill). We pick that band and box
/// its top + bottom edges into a (perspective) trapezoid, mapped back to
/// original-image pixels. Returns `None` when the mask is empty or the
/// derived bar collapses to a sliver.
fn u_to_lockbar_quad(
    det: &u_onnx::UDetection,
    thr: f32,
    w: u32,
    h: u32,
) -> Option<headtracking::calibration::LockbarQuadRgb> {
    let n = u_onnx::PROTO_SIDE;
    let mask = &det.proto_mask;
    // Per proto row: leftmost / rightmost "on" column + filled-cell count.
    let mut row_left = vec![0usize; n];
    let mut row_right = vec![0usize; n];
    let mut row_count = vec![0u32; n];
    let (mut py_min, mut py_max) = (usize::MAX, 0usize);
    for py in 0..n {
        let base = py * n;
        let (mut l, mut r, mut cnt) = (usize::MAX, 0usize, 0u32);
        for px in 0..n {
            if mask[base + px] >= thr {
                if px < l {
                    l = px;
                }
                r = px;
                cnt += 1;
            }
        }
        row_count[py] = cnt;
        if cnt > 0 {
            row_left[py] = l;
            row_right[py] = r;
            py_min = py_min.min(py);
            py_max = py;
        }
    }
    if py_min > py_max {
        return None; // empty mask
    }
    let cmax = row_count.iter().copied().max().unwrap_or(0);
    if cmax == 0 {
        return None;
    }
    // "Filled across" cut — separates the closed bar (≈full width) from
    // the two-rail region (two thin runs with a gap).
    let fill_t = (cmax as f32 * 0.6).ceil() as u32;
    let is_bar = |py: usize| row_count[py] >= fill_t;
    // How many contiguous filled rows hang off the top extremity…
    let mut top_end = py_min;
    while top_end <= py_max && is_bar(top_end) {
        top_end += 1;
    }
    let top_rows = top_end - py_min; // band = [py_min, top_end)
    // …and off the bottom extremity.
    let mut bot_start = py_max + 1;
    while bot_start > py_min && is_bar(bot_start - 1) {
        bot_start -= 1;
    }
    let bot_rows = (py_max + 1) - bot_start; // band = [bot_start, py_max]
    // The lockbar is the thicker filled band; a clean U fills only one end.
    let (bar_y0, bar_y1) = if top_rows >= bot_rows && top_rows > 0 {
        (py_min, top_end - 1)
    } else if bot_rows > 0 {
        (bot_start, py_max)
    } else {
        // No filled band at all (degenerate) — fall back to the single
        // widest row so we still emit a usable quad.
        let py = (0..n).max_by_key(|&py| row_count[py]).unwrap_or(py_min);
        (py, py)
    };
    let edge = |py: usize| -> Option<(usize, usize)> {
        (row_count[py] > 0).then_some((row_left[py], row_right[py]))
    };
    let (tl_px, tr_px) = edge(bar_y0)?;
    let (bl_px, br_px) = edge(bar_y1)?;
    let clamp = |(x, y): (f32, f32)| -> (u32, u32) {
        (
            x.round().clamp(0.0, (w.saturating_sub(1)) as f32) as u32,
            y.round().clamp(0.0, (h.saturating_sub(1)) as f32) as u32,
        )
    };
    let tl = clamp(det.proto_to_image(tl_px, bar_y0));
    let tr = clamp(det.proto_to_image(tr_px, bar_y0));
    let br = clamp(det.proto_to_image(br_px, bar_y1));
    let bl = clamp(det.proto_to_image(bl_px, bar_y1));
    let mean_w = (tr.0.saturating_sub(tl.0) + br.0.saturating_sub(bl.0)) / 2;
    if mean_w < 4 {
        return None;
    }
    let slope_deg = (tr.1 as f32 - tl.1 as f32)
        .atan2(tr.0 as f32 - tl.0 as f32)
        .to_degrees();
    let thickness_px = (((bl.1 + br.1) as f32 - (tl.1 + tr.1) as f32) * 0.5).abs() as u32;
    let n_inliers = (det.confidence * 100.0).clamp(0.0, 1_000.0) as u32;

    // --- Sidebars (the rails of the U). Only the part *near* the lockbar is a
    //     real rail; further out on the open side is the receding playfield —
    //     or, from this overhead camera, the players standing behind the bar.
    //     So: (1) open side only, never past the bar; (2) a short band right
    //     next to the bar (a body read as a long rail is cut off); (3) each
    //     rail must *start at* the matching lockbar corner (attachment). Fit a
    //     straight line to each rail-row's outer edge, with outlier rejection
    //     so a forearm laid across it does not bend it. Emitted `[near, far]`.
    const RAIL_ATTACH_TOL: f32 = 6.0; // proto px: rail base must meet the corner
    // The rails follow the PLAYFIELD, which is the side of the lockbar carrying
    // the most mask (the bright lit table) — NOT the sparse player side. Using
    // raw fill instead of "which extremity is the bar" stops the rails from
    // shooting up a player standing right behind the bar. `bar_at_top` here
    // means "playfield is below the bar → rails go down (large py)". The rails
    // then span the WHOLE playfield side: from the lockbar corner down to the
    // near/bottom edge of the visible playfield.
    let fill_below: u64 = if bar_y1 < py_max {
        row_count[bar_y1 + 1..=py_max]
            .iter()
            .map(|&c| u64::from(c))
            .sum()
    } else {
        0
    };
    let fill_above: u64 = if bar_y0 > py_min {
        row_count[py_min..bar_y0]
            .iter()
            .map(|&c| u64::from(c))
            .sum()
    } else {
        0
    };
    let bar_at_top = fill_below >= fill_above;
    let (band_lo, band_hi) = if bar_at_top {
        ((bar_y1 + 1).min(py_max), py_max)
    } else {
        (py_min, bar_y0.saturating_sub(1).max(py_min))
    };
    // Lockbar edge on the open side — where the rails must attach.
    let open_bar_row = if bar_at_top { bar_y1 } else { bar_y0 };
    let bar_left_x = row_left[open_bar_row] as f32;
    let bar_right_x = row_right[open_bar_row] as f32;
    let mut left_pts: Vec<(f32, f32)> = Vec::new();
    let mut right_pts: Vec<(f32, f32)> = Vec::new();
    let (mut rail_lo, mut rail_hi) = (usize::MAX, 0usize);
    if band_lo <= band_hi {
        for py in band_lo..=band_hi {
            if row_count[py] == 0 {
                continue; // empty row — nothing to attach a rail to
            }
            // The two rails are the LEFT and RIGHT edges of the mask on this
            // row — whether or not the row is "filled across" (a bright solid
            // playfield fills the near-lockbar rows, and its two side edges ARE
            // the rails). Fitting each edge over the band gives one rail per
            // side; band + attachment keep them short and at the corners.
            left_pts.push((row_left[py] as f32, py as f32));
            right_pts.push((row_right[py] as f32, py as f32));
            rail_lo = rail_lo.min(py);
            rail_hi = rail_hi.max(py);
        }
    }
    let make_rail = |pts: &[(f32, f32)], bar_edge_x: f32| -> Option<[(u32, u32); 2]> {
        let (a, b, _) = fit_rail(pts)?;
        // near = lockbar-adjacent extremity, far = open extremity.
        let (y_near, y_far) = if bar_at_top {
            (rail_lo, rail_hi)
        } else {
            (rail_hi, rail_lo)
        };
        let near_x = a * y_near as f32 + b;
        // The rail base must meet the lockbar corner, else it is not a rail.
        if (near_x - bar_edge_x).abs() > RAIL_ATTACH_TOL {
            return None;
        }
        let at = |x: f32, y: usize| -> (u32, u32) {
            let xc = x.round().clamp(0.0, (n.saturating_sub(1)) as f32) as usize;
            clamp(det.proto_to_image(xc, y))
        };
        Some([at(near_x, y_near), at(a * y_far as f32 + b, y_far)])
    };
    let (left_rail, right_rail) = if rail_lo <= rail_hi {
        (
            make_rail(&left_pts, bar_left_x),
            make_rail(&right_pts, bar_right_x),
        )
    } else {
        (None, None)
    };

    Some(headtracking::calibration::LockbarQuadRgb {
        frame_width: w,
        frame_height: h,
        corners: [tl, tr, br, bl],
        slope_deg,
        thickness_px,
        n_inliers_top: n_inliers,
        n_inliers_bottom: n_inliers,
        left_rail,
        right_rail,
    })
}

/// Max distance (proto-grid px, 160-side) a rail point may sit from the
/// first-pass line and still count — a forearm laid across a rail bulges well
/// past this, so it is dropped before the refit.
const RAIL_OUTLIER_PX: f32 = 3.5;

/// Least-squares fit of a near-vertical rail as `x = a*y + b`, with one refit
/// after dropping points more than [`RAIL_OUTLIER_PX`] off the first line — so
/// a forearm laid across a rail is rejected rather than bending it. Returns
/// `(a, b, inlier_count)`, or `None` when too few points survive.
fn fit_rail(pts: &[(f32, f32)]) -> Option<(f32, f32, usize)> {
    const MIN_PTS: usize = 4;
    if pts.len() < MIN_PTS {
        return None;
    }
    let lsq = |pts: &[(f32, f32)]| -> Option<(f32, f32)> {
        let n = pts.len() as f32;
        let (mut sy, mut sx, mut syy, mut sxy) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
        for &(x, y) in pts {
            sy += y;
            sx += x;
            syy += y * y;
            sxy += x * y;
        }
        let den = n * syy - sy * sy;
        if den.abs() < 1e-3 {
            return None; // all points on one row — undefined slope
        }
        let a = (n * sxy - sy * sx) / den;
        let b = (sx - a * sy) / n;
        Some((a, b))
    };
    let (a, b) = lsq(pts)?;
    let inliers: Vec<(f32, f32)> = pts
        .iter()
        .copied()
        .filter(|&(x, y)| (x - (a * y + b)).abs() <= RAIL_OUTLIER_PX)
        .collect();
    if inliers.len() < MIN_PTS {
        return None;
    }
    let (a2, b2) = lsq(&inliers)?;
    Some((a2, b2, inliers.len()))
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
/// calibration yet), so we fall back to the same approximation
/// `head_to_pixel_webcam` uses — fx ≈ 0.85 × frame_width — keyed off the
/// frame dimensions stored in the quad itself.
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
        0.85 * quad.frame_width as f32
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
    last_refresh: Instant,
}

impl V1Controls {
    fn new() -> Self {
        Self {
            desired_tilt_deg: 0.0,
            last_sent_tilt_deg: 0.0,
            selected_led: freenect::LedState::Green,
            last_sent_led: freenect::LedState::Green,
            last_state: None,
            // Seed with a stale instant so the first poll triggers a refresh.
            last_refresh: Instant::now() - Duration::from_secs(60),
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

impl Inner {
    /// `true` when this input pipeline produces 3D head poses (head-box +
    /// depth for Kinect, head-box width triangulation for webcam).
    fn has_head_tracker(&self) -> bool {
        matches!(
            self,
            Inner::KinectV1 { .. } | Inner::KinectV2 { .. } | Inner::Webcam { .. }
        )
    }
}

impl Active {
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
fn head_pixel_from_pose_depth(
    pose: &blazepose::Pose,
    rgb: (u32, u32),
    depth_data: &[f32],
    depth_dims: (u32, u32),
    intr: &Intrinsics,
    min_samples: usize,
) -> Option<HeadPixel> {
    let (rgb_w, rgb_h) = rgb;
    let (depth_w, depth_h) = depth_dims;
    if rgb_w == 0 || rgb_h == 0 || depth_w == 0 || depth_h == 0 {
        return None;
    }
    let nose = &pose.landmarks[blazepose::idx::NOSE];
    let depth_cx = nose.x * depth_w as f32 / rgb_w as f32;
    let depth_cy = nose.y * depth_h as f32 / rgb_h as f32;
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
            let z = depth_data[row + u as usize];
            if (DEPTH_MIN_MM..=DEPTH_MAX_MM).contains(&z) {
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
    use blazepose::idx::{LEFT_SHOULDER, NOSE, RIGHT_SHOULDER};
    if rgb_w == 0 || rgb_h == 0 {
        return None;
    }
    let (ls, rs, nose) = (
        &pose.landmarks[LEFT_SHOULDER],
        &pose.landmarks[RIGHT_SHOULDER],
        &pose.landmarks[NOSE],
    );
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
    let fx = rgb_w as f32 * 0.9;
    let cx = rgb_w as f32 * 0.5;
    let cy = rgb_h as f32 * 0.5;
    const SHOULDER_W_MM: f32 = 400.0;
    let depth_mm = fx * SHOULDER_W_MM / w_px;
    let zf = f64::from(depth_mm);
    Some(HeadPixel {
        u: nose.x.max(0.0) as u32,
        v: nose.y.max(0.0) as u32,
        depth_mm,
        x_mm: (f64::from(nose.x - cx) * zf / f64::from(fx)) as f32,
        y_mm: (f64::from(nose.y - cy) * zf / f64::from(fx)) as f32,
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
/// per-model inference time (EWMA), process CPU%, and the input (camera) vs
/// output (filtered-pose) frame rates. Detection runs inline on the UI
/// thread, so `out_fps` is exactly the tracking rate the player feels.
struct Metrics {
    head_ms: f32,
    u_ms: f32,
    in_fps: f32,
    out_fps: f32,
    cpu_pct: f32,
    in_frames: u32,
    out_poses: u32,
    window_start: Instant,
    last_jiffies: u64,
    last_log: Instant,
}

impl Metrics {
    fn new() -> Self {
        let now = Instant::now();
        Self {
            head_ms: 0.0,
            u_ms: 0.0,
            in_fps: 0.0,
            out_fps: 0.0,
            cpu_pct: 0.0,
            in_frames: 0,
            out_poses: 0,
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
    fn note_u_ms(&mut self, ms: f32) {
        self.u_ms = if self.u_ms == 0.0 {
            ms
        } else {
            self.u_ms * 0.8 + ms * 0.2
        };
    }
    fn note_input_frame(&mut self) {
        self.in_frames += 1;
    }
    fn note_output_pose(&mut self) {
        self.out_poses += 1;
    }

    /// Called once per poll: roll the 1 s window (recompute FPS + CPU%) and
    /// log a line every ~2 s so the downloadable log carries the same numbers
    /// as the toolbar.
    fn tick(&mut self) {
        let now = Instant::now();
        let elapsed = now.duration_since(self.window_start).as_secs_f32();
        if elapsed >= 1.0 {
            self.in_fps = self.in_frames as f32 / elapsed;
            self.out_fps = self.out_poses as f32 / elapsed;
            let jiffies = read_cpu_jiffies().unwrap_or(self.last_jiffies);
            // USER_HZ is 100 on Linux x86_64; ticks → seconds = / 100.
            let cpu_secs = jiffies.saturating_sub(self.last_jiffies) as f32 / 100.0;
            self.cpu_pct = cpu_secs / elapsed * 100.0;
            self.last_jiffies = jiffies;
            self.in_frames = 0;
            self.out_poses = 0;
            self.window_start = now;
        }
        if now.duration_since(self.last_log).as_secs_f32() >= 2.0 {
            info!(
                "perf: head {:.1}ms | U {:.1}ms | cpu {:.0}% | in {:.1} fps | out {:.1} fps",
                self.head_ms, self.u_ms, self.cpu_pct, self.in_fps, self.out_fps
            );
            self.last_log = now;
        }
    }

    /// One-line summary for the toolbar.
    fn summary(&self) -> String {
        format!(
            "head {:.0}ms · U {:.0}ms · cpu {:.0}% · in {:.0} / out {:.0} fps",
            self.head_ms, self.u_ms, self.cpu_pct, self.in_fps, self.out_fps
        )
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
            screenshot_status: None,
            // "270°" in the player's frame = egui-rotate CW90 once applied
            // (the rotated pincab screen inverts the apparent direction;
            // see `rotation_label`). This is the orientation that reads
            // upright on the cab.
            rotation: Rotation::CW90,
            should_quit: false,
            lockbar_width_mm: headtracking::calibration::LOCKBAR_WIDTH_MM,
            head_filter_min_cutoff: 1.0,
            head_filter_beta: 0.4,
            head_detect_hz: 15.0,
            bypass_filters: false,
            contribute_open: false,
            consent_checked: false,
            uploader: contribute::Uploader::spawn(),
            contrib_last: None,
            contrib_thumbs: None,
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
            parallax_invert: [false, true, true],
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
                match open_backend(self.selected) {
                    Ok(active) => {
                        info!(
                            backend = ?active.backend,
                            fx = active.intrinsics.fx,
                            fy = active.intrinsics.fy,
                            cx = active.intrinsics.cx,
                            cy = active.intrinsics.cy,
                            "backend opened"
                        );
                        // Opening succeeded, but confirm the stream actually
                        // flows before going live (Kinect v1 can open yet never
                        // deliver a frame until the stream is bounced).
                        self.switch_state = SwitchState::Waiting {
                            active: Box::new(active),
                            bounces: 0,
                            since: Instant::now(),
                        };
                    }
                    Err(e) => {
                        error!(?e, "failed to open backend");
                        self.error = Some(e);
                        self.selected = Backend::None;
                    }
                }
            }
            SwitchState::Waiting {
                mut active,
                bounces,
                since,
            } => {
                if active.poll_first_rgb() {
                    info!(backend = ?active.backend, bounces, "stream live");
                    self.active = Some(*active); // → Idle
                } else if since.elapsed() < FIRST_FRAME_WAIT {
                    self.switch_state = SwitchState::Waiting {
                        active,
                        bounces,
                        since,
                    }; // keep waiting this window
                } else if bounces >= MAX_STREAM_BOUNCES {
                    let msg = format!(
                        "{}: no video after {MAX_STREAM_BOUNCES} stream restarts — check the cable / USB",
                        backend_slug(active.backend)
                    );
                    error!("{msg}");
                    self.error = Some(msg);
                    self.selected = Backend::None; // drop active → Idle
                } else {
                    warn!(backend = ?active.backend, next = bounces + 1, "no first frame, bouncing stream");
                    match active.bounce_stream() {
                        Ok(()) => {
                            self.switch_state = SwitchState::Waiting {
                                active,
                                bounces: bounces + 1,
                                since: Instant::now(),
                            };
                        }
                        Err(e) => {
                            error!("stream restart failed: {e}");
                            self.error = Some(e);
                            self.selected = Backend::None;
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
        // IR source differs per sensor. Kinect v2 streams IR alongside depth,
        // so we already have `last_ir`. Kinect v1 shares one USB endpoint
        // between RGB and IR, so there's no live IR — we briefly flip the
        // video mode to grab one frame (3rd, once the auto-exposure settles),
        // then flip back to RGB.
        let ir_v1 = self
            .active
            .as_mut()
            .and_then(|active| match &mut active.inner {
                Inner::KinectV1 { device, .. } => match device.capture_ir(3) {
                    Ok(frame) => Some(frame),
                    Err(e) => {
                        warn!("contribution: v1 IR capture failed: {e}");
                        None
                    }
                },
                _ => None,
            });
        let payload = self.active.as_ref().and_then(|active| {
            active.last_rgb_frame.as_ref().map(|(w, h, raw)| {
                let det = bake_overlays(
                    *w,
                    *h,
                    raw,
                    active.last_pose.as_ref(),
                    active.last_lockbar.as_ref(),
                );
                (
                    active.backend,
                    *w,
                    *h,
                    raw.clone(),
                    det,
                    active.last_depth.clone(),
                    active.last_ir.clone(),
                    active.last_head,
                    active.last_pose.clone(),
                    active.last_lockbar,
                )
            })
        });
        let Some((backend, w, h, raw, det, depth, ir_v2, head, pose, lockbar)) = payload else {
            return;
        };
        let stem = contribution_stem(backend);
        let dir = contributions_dir();
        let _ = std::fs::create_dir_all(&dir);
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
        meta.extend(autocalib_meta(
            lockbar.as_ref(),
            (w, h),
            self.lockbar_width_mm,
            depth.as_ref(),
        ));
        // Collect every image this capture produced, then save + queue them
        // in one pass. RGB planes are 8-bit colour; depth is 16-bit gray in
        // raw mm; v2 IR is 16-bit gray intensity; v1 IR is 8-bit gray.
        let mut files: Vec<(String, Vec<u8>)> = Vec::new();
        for (kind, src) in [("raw", &raw), ("det", &det)] {
            match png_bytes_meta(w, h, src, &meta) {
                Ok(bytes) => files.push((format!("{stem}_{kind}.png"), bytes)),
                Err(e) => warn!("contribution: {kind} png encode failed: {e}"),
            }
        }
        // Each depth/IR modality ships a lossless file (the real values) plus
        // an auto-levelled 8-bit `*view` preview so it's reviewable by eye.
        if let Some((dw, dh, mm)) = depth.as_ref() {
            match png_gray16_bytes(*dw, *dh, mm) {
                Ok(bytes) => files.push((format!("{stem}_depth.png"), bytes)),
                Err(e) => warn!("contribution: depth png encode failed: {e}"),
            }
            match autolevel_gray8(*dw, *dh, mm, true) {
                Ok(bytes) => files.push((format!("{stem}_depthview.png"), bytes)),
                Err(e) => warn!("contribution: depthview png encode failed: {e}"),
            }
        }
        // IR: v2 (live, 16-bit) or v1 (mode-switch, 8-bit) — only one applies.
        if let Some((iw, ih, intensity)) = ir_v2.as_ref() {
            match png_gray16_bytes(*iw, *ih, intensity) {
                Ok(bytes) => files.push((format!("{stem}_ir.png"), bytes)),
                Err(e) => warn!("contribution: ir png encode failed: {e}"),
            }
            match autolevel_gray8(*iw, *ih, intensity, false) {
                Ok(bytes) => files.push((format!("{stem}_irview.png"), bytes)),
                Err(e) => warn!("contribution: irview png encode failed: {e}"),
            }
        } else if let Some(frame) = ir_v1.as_ref() {
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
        for (name, bytes) in files {
            if let Err(e) = std::fs::write(dir.join(&name), &bytes) {
                warn!(name, "contribution: local save failed: {e}");
            }
            self.uploader.submit(name, bytes);
        }
        info!(stem, "capture shared");
        self.contrib_last = Some(stem);
    }

    fn poll(&mut self, egui_ctx: &egui::Context) {
        let Some(active) = self.active.as_mut() else {
            return;
        };
        active.metrics.tick();
        // Push the live-tunable 1€ knobs onto the filter each frame (cheap;
        // set_params keeps the running state, no reset).
        active.pose_filter.set_params(filter_alias::OneEuroParams {
            min_cutoff_hz: self.head_filter_min_cutoff,
            beta: self.head_filter_beta,
            derivative_cutoff_hz: 1.0,
        });
        // Cap the head detector's rate (0 Hz slider = flat-out).
        let head_ms = if self.head_detect_hz > 0.1 {
            (1000.0 / self.head_detect_hz) as u32
        } else {
            0
        };
        // Rate-cap the pose inference (0 Hz slider = flat-out).
        active.blaze_worker.set_min_interval_ms(head_ms);
        // Debug bypass: raw pose, relaxed depth gate.
        let bypass = self.bypass_filters;
        let depth_min = if bypass { 4 } else { 16 };
        match &mut active.inner {
            Inner::KinectV2 { device, .. } => {
                if let Some(rgb) = device.poll_rgb() {
                    active.metrics.note_input_frame();
                    // RGB888 feeds BlazePose (every frame) and the U worker
                    // (until it locks); reused for the display texture.
                    let rgb888 = bgrx_to_rgb888(&rgb.data);
                    active
                        .blaze_worker
                        .submit(rgb888.clone(), rgb.width, rgb.height);
                    let pose_out = active.blaze_worker.snapshot();
                    active.last_pose = pose_out.pose;
                    if pose_out.ms > 0.0 {
                        active.metrics.note_head_ms(pose_out.ms);
                    }
                    if !active.u_worker.is_locked() {
                        active
                            .u_worker
                            .submit(rgb888.clone(), rgb.width, rgb.height);
                    }
                    let img = rgb888_to_color_image(rgb.width, rgb.height, &rgb888);
                    active.last_rgb_frame = Some((rgb.width, rgb.height, rgb888));
                    let u_out = active.u_worker.snapshot();
                    active.last_u = u_out.u;
                    active.last_lockbar = u_out.lockbar;
                    if u_out.u_ms > 0.0 {
                        active.metrics.note_u_ms(u_out.u_ms);
                    }
                    upload_texture(egui_ctx, &mut active.rgb_texture, img);
                }
                // IR streams on the same listener as depth (both produced by
                // the depth pipeline). Keep the latest for the capture export;
                // f32 intensity rounds into u16.
                if let Some(ir) = device.poll_ir() {
                    active.last_ir = Some((
                        ir.width,
                        ir.height,
                        ir.data.iter().map(|&v| v as u16).collect(),
                    ));
                }
                if let Some(depth) = device.poll_depth() {
                    // Keep the latest depth (f32 mm rounded to u16) for the
                    // "Share a capture" export. Cheap next to the RGB clone
                    // we already keep every frame.
                    active.last_depth = Some((
                        depth.width,
                        depth.height,
                        depth.data.iter().map(|&z| z as u16).collect(),
                    ));
                    // Head = BlazePose nose sampled in the depth frame (the
                    // pose comes from the RGB block above, async).
                    let head = active
                        .last_pose
                        .as_ref()
                        .and_then(|p| {
                            head_pixel_from_pose_depth(
                                p,
                                (1920, 1080),
                                &depth.data,
                                (depth.width, depth.height),
                                &active.intrinsics,
                                depth_min,
                            )
                        })
                        .map(|mut h| {
                            // The Kinect v2 colour frame is horizontally mirrored
                            // (text reads backwards), so the deprojected head X is
                            // flipped vs the real world — negate it so the
                            // left/right POV travelling matches v1.
                            h.x_mm = -h.x_mm;
                            h
                        });
                    let smoothed =
                        smooth_head(head, &mut active.pose_filter, active.started_at, bypass);
                    capture_baseline(&mut active.baseline, smoothed);
                    active.last_head = smoothed;
                    if smoothed.is_some() {
                        active.metrics.note_output_pose();
                    }
                }
            }
            Inner::KinectV1 { device, .. } => {
                if let Some(rgb) = device.poll_rgb() {
                    active.metrics.note_input_frame();
                    active
                        .blaze_worker
                        .submit(rgb.data.clone(), rgb.width, rgb.height);
                    let pose_out = active.blaze_worker.snapshot();
                    active.last_pose = pose_out.pose;
                    if pose_out.ms > 0.0 {
                        active.metrics.note_head_ms(pose_out.ms);
                    }
                    if !active.u_worker.is_locked() {
                        active
                            .u_worker
                            .submit(rgb.data.clone(), rgb.width, rgb.height);
                    }
                    let u_out = active.u_worker.snapshot();
                    active.last_u = u_out.u;
                    active.last_lockbar = u_out.lockbar;
                    if u_out.u_ms > 0.0 {
                        active.metrics.note_u_ms(u_out.u_ms);
                    }
                    let img = rgb888_to_color_image(rgb.width, rgb.height, &rgb.data);
                    upload_texture(egui_ctx, &mut active.rgb_texture, img);
                    active.last_rgb_frame = Some((rgb.width, rgb.height, rgb.data));
                }
                if let Some(depth) = device.poll_depth() {
                    // Keep the latest depth (native u16 mm) for the "Share a
                    // capture" export.
                    active.last_depth = Some((depth.width, depth.height, depth.data.clone()));
                    // libfreenect ships u16 mm; widen for the depth sampler.
                    let f32_data: Vec<f32> = depth.data.iter().map(|&v| f32::from(v)).collect();
                    // Head = BlazePose nose sampled in the depth frame.
                    let head = active.last_pose.as_ref().and_then(|p| {
                        head_pixel_from_pose_depth(
                            p,
                            (640, 480),
                            &f32_data,
                            (depth.width, depth.height),
                            &active.intrinsics,
                            depth_min,
                        )
                    });
                    let smoothed =
                        smooth_head(head, &mut active.pose_filter, active.started_at, bypass);
                    capture_baseline(&mut active.baseline, smoothed);
                    active.last_head = smoothed;
                    if smoothed.is_some() {
                        active.metrics.note_output_pose();
                    }
                }
            }
            Inner::Webcam { camera } => {
                if let Some(rgb) = camera.poll_rgb() {
                    active.metrics.note_input_frame();
                    active
                        .blaze_worker
                        .submit(rgb.data.clone(), rgb.width, rgb.height);
                    let pose_out = active.blaze_worker.snapshot();
                    active.last_pose = pose_out.pose;
                    if pose_out.ms > 0.0 {
                        active.metrics.note_head_ms(pose_out.ms);
                    }
                    // Webcam has no depth: triangulate the head distance from
                    // the pose's shoulder width.
                    let head = active
                        .last_pose
                        .as_ref()
                        .and_then(|p| head_pixel_from_pose_webcam(p, rgb.width, rgb.height));
                    let smoothed =
                        smooth_head(head, &mut active.pose_filter, active.started_at, bypass);
                    capture_baseline(&mut active.baseline, smoothed);
                    active.last_head = smoothed;
                    if smoothed.is_some() {
                        active.metrics.note_output_pose();
                    }
                    if !active.u_worker.is_locked() {
                        active
                            .u_worker
                            .submit(rgb.data.clone(), rgb.width, rgb.height);
                    }
                    let u_out = active.u_worker.snapshot();
                    active.last_u = u_out.u;
                    active.last_lockbar = u_out.lockbar;
                    if u_out.u_ms > 0.0 {
                        active.metrics.note_u_ms(u_out.u_ms);
                    }
                    let img = rgb888_to_color_image(rgb.width, rgb.height, &rgb.data);
                    upload_texture(egui_ctx, &mut active.rgb_texture, img);
                    active.last_rgb_frame = Some((rgb.width, rgb.height, rgb.data));
                }
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
    started_at: Instant,
    bypass: bool,
) -> Option<HeadPixel> {
    let head = raw?;
    if bypass {
        return Some(head); // raw pose, no 1€ smoothing
    }
    let mut head = head;
    let t_us = started_at.elapsed().as_micros() as u64;
    let smoothed = filter.update([head.x_mm, head.y_mm, head.depth_mm], t_us);
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
        let (Inner::KinectV1 { device, .. }, Some(controls)) =
            (&mut active.inner, active.v1_controls.as_mut())
        else {
            return;
        };

        // Refresh tilt + accel every 500 ms (USB roundtrip).
        if controls.last_refresh.elapsed() >= Duration::from_millis(500) {
            match device.tilt_state() {
                Ok(state) => controls.last_state = Some(state),
                Err(e) => warn!(?e, "kinect v1: tilt_state refresh failed"),
            }
            controls.last_refresh = Instant::now();
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
                    if let Err(e) = device.set_tilt_degrees(controls.desired_tilt_deg) {
                        warn!(?e, "set_tilt failed");
                    } else {
                        controls.last_sent_tilt_deg = controls.desired_tilt_deg;
                        info!(angle = controls.desired_tilt_deg, "tilt command sent");
                    }
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
                    if let Err(e) = device.set_led(controls.selected_led) {
                        warn!(?e, "set_led failed");
                    } else {
                        controls.last_sent_led = controls.selected_led;
                    }
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
                    self.parallax_eye = [
                        sign(0) * dx * g,
                        sign(1) * dy_t * g,
                        (PX_DVIEW_MM + sign(2) * dz_t * g).clamp(150.0, 1500.0),
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
                // vertical scroll (the taller cab fonts made the default clip
                // rows). Sized to the entry count with generous per-row height.
                let popup_h = (self.available.len().max(1) as f32) * 38.0 + 12.0;
                ComboBox::from_id_salt("backend")
                    .width(combo_w)
                    .height(popup_h)
                    .selected_text(selected_label)
                    .show_ui(ui, |ui| {
                        // Keep each entry on a single line; the popup then
                        // stretches to exactly the number of entries.
                        ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
                        let entries = self.available.clone();
                        for entry in &entries {
                            ui.selectable_value(&mut self.selected, entry.backend, &entry.label);
                        }
                    });
                if ui.button("Rescan").clicked() {
                    self.refresh_available();
                }
                self.lockbar_width_field(ui);
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
                        active.last_lockbar.as_ref(),
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
                    .on_hover_text("Rotate the window 90° (physically rotated screen)")
                    .clicked()
                {
                    self.rotation = next_rotation(self.rotation);
                }
                // Parallax — on/off toggle for the off-axis 3D validation view
                // stacked below the camera feed. A highlight toggle (blue when
                // on), matching the parallax eye-mode selector. The 🪟 glyph
                // renders via the vendored NotoEmoji subset (see
                // `install_extra_glyph_fonts`).
                ui.toggle_value(&mut self.parallax_enabled, "🪟 Parallax")
                    .on_hover_text("Show the off-axis 3D validation scene below the camera feed");
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

            ui.add_space(2.0);
            // Row 2 — camera INPUT (raw, before maths). `input_line` lays out
            // two rows: device + head measurements, then the U-seg / lockbar
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
                        active.baseline = None;
                    }
                    if let Some(active) = self.active.as_ref() {
                        if !active.inner.has_head_tracker() {
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
            self.draw_camera_view(ui);
        });
        let ctx = ui.ctx().clone();
        self.contribute_window(&ctx);
    }

    /// Lockbar-width field — the metric ruler for the monocular (webcam)
    /// scale. Shown only for a webcam input: depth cameras measure scale
    /// directly, so they don't need it (there it's just a cross-check). mm
    /// with an inch read-out — the international inch is exactly 25.4 mm.
    fn lockbar_width_field(&mut self, ui: &mut egui::Ui) {
        let is_webcam = matches!(
            self.active.as_ref().map(|a| a.backend),
            Some(Backend::Webcam(_))
        );
        // Sidebar/lockbar physical width — the scale reference (webcam only;
        // the Kinect gets scale from depth).
        if is_webcam {
            ui.separator();
            ui.label("Sidebar:");
            ui.add(
                egui::DragValue::new(&mut self.lockbar_width_mm)
                    .speed(1.0)
                    .range(200.0..=1200.0)
                    .suffix(" mm"),
            );
            let inches = self.lockbar_width_mm / 25.4;
            ui.label(RichText::new(format!("({inches:.1} in)")).weak());
        }
        // Table inclination — the second key input. The VPX screen is the
        // near-flat playfield, so the parallax tilts head motion by
        // 90° − inclination. Shown for every backend.
        ui.separator();
        ui.label("Incl:");
        ui.add(
            egui::DragValue::new(&mut self.table_incl_deg)
                .speed(0.5)
                .range(0.0..=30.0)
                .suffix("°"),
        )
        .on_hover_text(
            "Playfield inclination from horizontal (VPX gives this per table). \
             The parallax tilts head motion by 90° − this angle, since the VPX \
             screen is the laid-flat playfield.",
        );
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

    /// The "Share a capture" window: the informed-consent notice + checkbox,
    /// the share button (gated on consent + a live frame), upload status, and
    /// a short capture reminder. All demo-only — the plugin has none of this.
    fn contribute_window(&mut self, ctx: &egui::Context) {
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
                // Explicit opt-in as a highlight toggle (blue when on) — an
                // affirmative, deliberate action before any upload is allowed.
                ui.toggle_value(
                    &mut self.consent_checked,
                    "☐ I have read the above and I freely give my informed consent to share \
                     these images under these terms.",
                );
                ui.add_space(6.0);
                let has_frame = self
                    .active
                    .as_ref()
                    .is_some_and(|a| a.last_rgb_frame.is_some());
                let ready = self.consent_checked && has_frame;
                if ui
                    .add_enabled(ready, egui::Button::new("📸 Share this capture"))
                    .clicked()
                {
                    self.share_capture();
                }
                if !has_frame {
                    ui.label(RichText::new("(select a device and wait for the feed first)").weak());
                }
                // Upload status.
                let st = self.uploader.status();
                if st.pending > 0 {
                    ui.label(format!("uploading… {} file(s) pending", st.pending));
                }
                if st.uploaded > 0 {
                    ui.label(
                        RichText::new(format!("✓ {} file(s) uploaded", st.uploaded))
                            .color(Color32::from_rgb(0x66, 0xff, 0x99)),
                    );
                }
                if let Some(err) = &st.last_error {
                    ui.label(
                        RichText::new(format!("upload issue: {err} — saved locally"))
                            .color(Color32::from_rgb(0xff, 0x99, 0x66)),
                    );
                }
                if let Some(stem) = &self.contrib_last {
                    ui.add_space(4.0);
                    ui.label("Shared — note this if you may want it removed:");
                    ui.monospace(format!("{stem}_raw.png · {stem}_det.png"));
                }
                ui.add_space(4.0);
                // Where captures land on this machine (also queued for upload).
                ui.horizontal_wrapped(|ui| {
                    ui.spacing_mut().item_spacing.x = 4.0;
                    ui.label("Also saved locally in:");
                    ui.monospace(contributions_dir().display().to_string());
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

    /// Camera feed with the lockbar + head overlays, scaled to fit while
    /// keeping the source aspect ratio.
    fn draw_camera_view(&self, ui: &mut egui::Ui) {
        let avail = ui.available_size();
        let aspect = match self.active.as_ref() {
            Some(active) => match (&active.inner, active.rgb_texture.as_ref()) {
                (_, Some(tex)) => {
                    let s = tex.size_vec2();
                    if s.y > 0.0 { s.x / s.y } else { 16.0 / 9.0 }
                }
                (Inner::KinectV2 { .. }, None) => 1920.0 / 1080.0,
                (Inner::KinectV1 { .. }, None) => 640.0 / 480.0,
                (Inner::Webcam { .. }, None) => 640.0 / 480.0,
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
                // Lockbar contour (cyan) + sidebars (orange) derived from the
                // U — no translucent mask fog any more.
                if let Some(bar) = active.last_lockbar {
                    draw_lockbar(ui.painter(), rect, bar);
                }
                // Source dimensions for bbox normalisation come from
                // the texture itself, NOT from the backend's spec.
                // On macOS AVFoundation the camera often delivers
                // frames at a size different from what
                // `SDL_GetCameraFormat` reported at open time —
                // using the spec here mapped boxes to the wrong
                // place (or off-screen). The texture was built from
                // the same buffer the detector saw, so its size is
                // authoritative.
                // BlazePose skeleton overlay: landmarks are in frame pixels,
                // mapped onto the view rect (same space for every backend).
                let src_size = tex.size_vec2();
                if let Some(p) = &active.last_pose
                    && src_size.x > 0.0
                    && src_size.y > 0.0
                {
                    // Clip to the camera rect: BlazePose extrapolates off-frame
                    // joints (knees/feet), so keep them for tracking but never
                    // paint them outside the RGB view (they'd spill onto the
                    // parallax scene below).
                    let clipped = ui.painter().with_clip_rect(rect);
                    draw_pose_overlay(&clipped, rect, p, src_size);
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
                SwitchState::Waiting { bounces: 0, .. } => {
                    format!("opening {}…", self.label_for(self.selected))
                }
                SwitchState::Waiting { bounces, .. } => {
                    format!("retry {bounces}/{MAX_STREAM_BOUNCES}…")
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
        // Row 2 (Live only): the bench knobs — gain on the same line as the
        // per-axis sign flips. Find the right signs/gain here, then bake them
        // into camera/mapping.rs (not a product calibration step).
        if self.parallax_eye_mode == ParallaxEye::Live {
            ui.horizontal(|ui| {
                ui.add(egui::Slider::new(&mut self.parallax_gain, 0.5..=6.0).text("gain"));
                ui.separator();
                ui.toggle_value(&mut self.parallax_invert[0], "±X");
                ui.toggle_value(&mut self.parallax_invert[1], "±Y");
                ui.toggle_value(&mut self.parallax_invert[2], "±Z");
            });
            // 1€ filter knobs — live, so lag on fast head moves can be dialed
            // out on the cab (higher beta = snappier; higher cutoff = less
            // smoothing when still). Applied to the pose in `poll`.
            ui.horizontal(|ui| {
                ui.add(
                    egui::Slider::new(&mut self.head_filter_min_cutoff, 0.1..=5.0)
                        .text("1€ cutoff"),
                );
                ui.add(egui::Slider::new(&mut self.head_filter_beta, 0.0..=1.5).text("1€ beta"));
            });
            // Head-detection rate cap — trade CPU for re-acquisition latency
            // live on the cab (30 = flat-out-ish, lower = less CPU).
            ui.horizontal(|ui| {
                ui.add(egui::Slider::new(&mut self.head_detect_hz, 3.0..=30.0).text("head Hz"));
                ui.separator();
                ui.toggle_value(&mut self.bypass_filters, "no filters")
                    .on_hover_text("Bypass 1€ filter + picker scoring + most of the depth gate");
            });
        }

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
            if !active.inner.has_head_tracker() {
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
        // Line 2 — raw U-seg confidence + the derived lockbar, on their own
        // row below (shown whenever a U exists, independent of head presence).
        if active.last_u.is_some() || active.last_lockbar.is_some() {
            ui.horizontal(|ui| {
                if let Some(u) = &active.last_u {
                    ui.label(
                        RichText::new(format!("U seg {:.0}%", u.confidence * 100.0))
                            .color(LOCKBAR_COLOR)
                            .monospace()
                            .size(15.0),
                    );
                }
                if let Some(bar) = active.last_lockbar {
                    if active.last_u.is_some() {
                        ui.separator();
                    }
                    ui.label(
                        RichText::new(format!(
                            "lockbar (U base) px: row {}, w {}px, t {}px, slope {:+.1}°",
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
        // We only reach here with a live lockbar quad; the U warmup/lock state
        // now lives on the worker thread, so just flag that the delta is live.
        let (tag, color) = ("U live", Color32::LIGHT_GREEN);
        ui.label(
            RichText::new(format!(
                "→ VPX   ΔX {dx:+.0}   ΔY {dy:+.0}   ΔZ {dz:+.0} mm   [{tag}]"
            ))
            .monospace()
            .color(color),
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

/// Draw the BlazePose skeleton (bones + joints) over the camera view. The 33
/// landmarks are in frame pixels; `src` is the texture size they map onto.
fn draw_pose_overlay(painter: &egui::Painter, rect: Rect, pose: &blazepose::Pose, src: Vec2) {
    use blazepose::idx::{
        LEFT_ELBOW, LEFT_SHOULDER, LEFT_WRIST, NOSE, RIGHT_ELBOW, RIGHT_SHOULDER, RIGHT_WRIST,
    };
    let to_screen = |i: usize| {
        let l = &pose.landmarks[i];
        Pos2::new(
            rect.left() + l.x / src.x * rect.width(),
            rect.top() + l.y / src.y * rect.height(),
        )
    };
    let bone = Color32::from_gray(0xcc);
    for (a, b) in [
        (LEFT_SHOULDER, RIGHT_SHOULDER),
        (LEFT_SHOULDER, LEFT_ELBOW),
        (LEFT_ELBOW, LEFT_WRIST),
        (RIGHT_SHOULDER, RIGHT_ELBOW),
        (RIGHT_ELBOW, RIGHT_WRIST),
        (NOSE, LEFT_SHOULDER),
        (NOSE, RIGHT_SHOULDER),
    ] {
        painter.line_segment([to_screen(a), to_screen(b)], Stroke::new(2.0, bone));
    }
    for i in 0..33 {
        let c = if (11..=16).contains(&i) {
            Color32::from_rgb(0xff, 0x96, 0x00)
        } else {
            Color32::from_rgb(0x00, 0xdc, 0x3c)
        };
        painter.circle_filled(to_screen(i), 4.0, c);
    }
}

fn draw_lockbar(
    painter: &egui::Painter,
    rect: Rect,
    bar: headtracking::calibration::LockbarQuadRgb,
) {
    if bar.frame_width == 0 || bar.frame_height == 0 {
        return;
    }
    let fw = bar.frame_width as f32;
    let fh = bar.frame_height as f32;
    let to_screen = |col: u32, row: u32| -> Pos2 {
        rect.left_top()
            + Vec2::new(
                (col as f32 / fw) * rect.width(),
                (row as f32 / fh) * rect.height(),
            )
    };
    let pts: [Pos2; 4] = [
        to_screen(bar.corners[0].0, bar.corners[0].1),
        to_screen(bar.corners[1].0, bar.corners[1].1),
        to_screen(bar.corners[2].0, bar.corners[2].1),
        to_screen(bar.corners[3].0, bar.corners[3].1),
    ];
    let stroke = Stroke::new(3.0_f32, LOCKBAR_COLOR);
    painter.line_segment([pts[0], pts[1]], stroke);
    painter.line_segment([pts[1], pts[2]], stroke);
    painter.line_segment([pts[2], pts[3]], stroke);
    painter.line_segment([pts[3], pts[0]], stroke);
    // Sidebars (rails) in orange, fitted straight segments.
    let rail_stroke = Stroke::new(3.0_f32, RAIL_COLOR);
    for rail in [bar.left_rail, bar.right_rail].into_iter().flatten() {
        painter.line_segment(
            [
                to_screen(rail[0].0, rail[0].1),
                to_screen(rail[1].0, rail[1].1),
            ],
            rail_stroke,
        );
    }
}

// ============================================================ Backend opening

fn open_backend(b: Backend) -> Result<Active, String> {
    match b {
        Backend::None => Err("no backend selected".to_string()),
        Backend::KinectV2 => open_kinect_v2(),
        Backend::KinectV1 => open_kinect_v1(),
        Backend::Webcam(idx) => open_webcam(idx),
    }
}

fn open_kinect_v2() -> Result<Active, String> {
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
    Ok(Active {
        backend: Backend::KinectV2,
        intrinsics: Intrinsics {
            fx: p.fx,
            fy: p.fy,
            cx: p.cx,
            cy: p.cy,
        },
        rgb_texture: None,
        last_head: None,
        baseline: None,
        inner: Inner::KinectV2 { device, _ctx: ctx },
        v1_controls: None,
        pose_filter: make_pose_filter(),
        started_at: Instant::now(),
        last_lockbar: None,
        last_u: None,
        u_worker: UWorker::spawn(),
        blaze_worker: BlazePoseWorker::spawn(),
        last_pose: None,
        last_rgb_frame: None,
        last_depth: None,
        last_ir: None,
        metrics: Metrics::new(),
    })
}

fn open_kinect_v1() -> Result<Active, String> {
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

    // Seed the v1 controls with the device's current tilt so the slider
    // doesn't snap on first use. Failures are non-fatal — we just log.
    let mut controls = V1Controls::new();
    match device.tilt_state() {
        Ok(state) => {
            controls.desired_tilt_deg = state.angle_deg;
            controls.last_sent_tilt_deg = state.angle_deg;
            controls.last_state = Some(state);
            controls.last_refresh = Instant::now();
        }
        Err(e) => warn!(?e, "kinect v1: tilt_state read at open failed"),
    }
    if let Err(e) = device.set_led(controls.selected_led) {
        warn!(?e, "kinect v1: initial set_led failed");
    }

    Ok(Active {
        backend: Backend::KinectV1,
        intrinsics: Intrinsics {
            fx: freenect::FX,
            fy: freenect::FY,
            cx: freenect::CX,
            cy: freenect::CY,
        },
        rgb_texture: None,
        last_head: None,
        baseline: None,
        inner: Inner::KinectV1 { device, _ctx: ctx },
        v1_controls: Some(controls),
        pose_filter: make_pose_filter(),
        started_at: Instant::now(),
        last_lockbar: None,
        last_u: None,
        u_worker: UWorker::spawn(),
        blaze_worker: BlazePoseWorker::spawn(),
        last_pose: None,
        last_rgb_frame: None,
        last_depth: None,
        last_ir: None,
        metrics: Metrics::new(),
    })
}

fn open_webcam(index: u32) -> Result<Active, String> {
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
    Ok(Active {
        backend: Backend::Webcam(index),
        // Without lockbar/disc calibration, fx ≈ 0.85 × frame_width is a
        // reasonable placeholder for a generic 60° HFOV webcam. The values
        // get replaced by ht-calibrate output when that lands.
        intrinsics: Intrinsics {
            fx: 0.0,
            fy: 0.0,
            cx: 0.0,
            cy: 0.0,
        },
        rgb_texture: None,
        last_head: None,
        baseline: None,
        inner: Inner::Webcam { camera },
        v1_controls: None,
        pose_filter: make_pose_filter(),
        started_at: Instant::now(),
        last_lockbar: None,
        last_u: None,
        u_worker: UWorker::spawn(),
        blaze_worker: BlazePoseWorker::spawn(),
        last_pose: None,
        last_rgb_frame: None,
        last_depth: None,
        last_ir: None,
        metrics: Metrics::new(),
    })
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

/// Directory for shared captures, next to the running binary.
fn contributions_dir() -> std::path::PathBuf {
    std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(std::path::Path::to_path_buf))
        .unwrap_or_else(|| std::path::PathBuf::from("."))
        .join("contributions")
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
/// unknown until the lockbar autocalib recovers it, so fall back to a nominal
/// `0.9 × width`.
fn color_focal_px(backend: Backend, frame_width: u32) -> f32 {
    let w = frame_width as f32;
    match backend {
        // Kinect v2 colour: ~1081 px at 1920×1080.
        Backend::KinectV2 => 1081.0 * w / 1920.0,
        // Kinect v1 RGB: ~525 px at 640×480.
        Backend::KinectV1 => 525.0 * w / 640.0,
        // Webcam / none: nominal until autocalib supplies the real focal.
        _ => w * 0.9,
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
    // bar seen through a nominal focal `fx = frame_w × 0.9`), and the bar centre's
    // pixel offset → camera lateral/vertical offset off the playfield centreline
    // at that distance. Same nominal-focal placeholder as the webcam Z; the
    // autocalib homography will replace `fx` with the real one. See
    // [[headtracking-autocalib-vision]].
    match lockbar {
        Some(lb) => {
            let [tl, tr, br, bl] = lb.corners.map(|(u, v)| (u as f32, v as f32));
            let edge = |a: (f32, f32), b: (f32, f32)| ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt();
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
    lockbar_mm: f32,
    depth: Option<&(u32, u32, Vec<u16>)>,
) -> Vec<(String, String)> {
    use headtracking::calibration::autocalib;
    let mut m: Vec<(String, String)> = Vec::new();
    let Some(q) = lockbar else {
        return m;
    };
    // Lockbar-rectangle homography (Zhang): focal + pose, no manual step. `t`
    // is the front-edge centre in camera coords, so |t| = cam↔bar distance.
    if let Some(cal) =
        autocalib::calibrate_homography(q, lockbar_mm, autocalib::DEFAULT_LOCKBAR_DEPTH_MM)
    {
        let dist = (cal.t[0] * cal.t[0] + cal.t[1] * cal.t[1] + cal.t[2] * cal.t[2]).sqrt();
        m.push(("ht_autocalib_fx".to_string(), format!("{:.0}", cal.fx)));
        m.push(("ht_autocalib_dist_mm".to_string(), format!("{dist:.0}")));
    }
    // Sidebar vanishing-point focal (needs both rails; long lever arm → robust
    // depth axis). `None` when the U-seg only yielded the front bar.
    if let Some(cal) = autocalib::calibrate_from_lockbar(q, lockbar_mm) {
        m.push(("ht_autocalib_vp_fx".to_string(), format!("{:.0}", cal.fx)));
    }
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

/// Bake the same overlays the GUI draws (U mask tint, head bbox in red,
/// derived lockbar quad in cyan) directly into a copy of the RGB888
/// buffer. Used by the screenshot path so users can read back what the
/// algorithms saw, not just the raw frame.
/// Draw the BlazePose skeleton (bones + joints) and the lockbar contour onto a
/// copy of an RGB888 frame. Shared by the contribution `_det` export and the
/// headless screenshot.
fn bake_overlays(
    width: u32,
    height: u32,
    rgb888: &[u8],
    pose: Option<&blazepose::Pose>,
    lockbar: Option<&headtracking::calibration::LockbarQuadRgb>,
) -> Vec<u8> {
    use blazepose::idx::{
        LEFT_ELBOW, LEFT_SHOULDER, LEFT_WRIST, NOSE, RIGHT_ELBOW, RIGHT_SHOULDER, RIGHT_WRIST,
    };
    const LOCKBAR_RGB: [u8; 3] = [0x00, 0xe5, 0xff]; // cyan
    const RAIL_RGB: [u8; 3] = [0xff, 0x9a, 0x00]; // orange
    const ARM_RGB: [u8; 3] = [0xff, 0x96, 0x00]; // shoulders/elbows/wrists
    const FACE_RGB: [u8; 3] = [0x00, 0xdc, 0x3c]; // everything else
    let mut out = rgb888.to_vec();
    let (wu, hu) = (width as usize, height as usize);
    if let Some(p) = pose {
        let g = |i: usize| (p.landmarks[i].x as i32, p.landmarks[i].y as i32);
        for (a, b) in [
            (LEFT_SHOULDER, RIGHT_SHOULDER),
            (LEFT_SHOULDER, LEFT_ELBOW),
            (LEFT_ELBOW, LEFT_WRIST),
            (RIGHT_SHOULDER, RIGHT_ELBOW),
            (RIGHT_ELBOW, RIGHT_WRIST),
            (NOSE, LEFT_SHOULDER),
            (NOSE, RIGHT_SHOULDER),
        ] {
            draw_line_rgb(&mut out, wu, hu, g(a), g(b), C_BONE);
        }
        let r = (width / 220).max(4) as i32;
        for (i, l) in p.landmarks.iter().enumerate() {
            let c = if (11..=16).contains(&i) {
                ARM_RGB
            } else {
                FACE_RGB
            };
            draw_disc_rgb(&mut out, wu, hu, l.x as i32, l.y as i32, r, c);
        }
    }
    if let Some(bar) = lockbar {
        for w in 0..4 {
            let a = bar.corners[w];
            let b = bar.corners[(w + 1) % 4];
            draw_line_rgb(
                &mut out,
                wu,
                hu,
                (a.0 as i32, a.1 as i32),
                (b.0 as i32, b.1 as i32),
                LOCKBAR_RGB,
            );
        }
        for rail in [bar.left_rail, bar.right_rail].into_iter().flatten() {
            draw_line_rgb(
                &mut out,
                wu,
                hu,
                (rail[0].0 as i32, rail[0].1 as i32),
                (rail[1].0 as i32, rail[1].1 as i32),
                RAIL_RGB,
            );
        }
    }
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

const C_BONE: [u8; 3] = [0x88, 0x88, 0x88]; // grey

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
    use blazepose::idx::{
        LEFT_ELBOW, LEFT_SHOULDER, LEFT_WRIST, NOSE, RIGHT_ELBOW, RIGHT_SHOULDER, RIGHT_WRIST,
    };
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

    // Draw skeleton on a copy of the frame.
    let mut buf = img.as_raw().clone();
    let (wu, hu) = (w as usize, h as usize);
    let bones = [
        (LEFT_SHOULDER, RIGHT_SHOULDER),
        (LEFT_SHOULDER, LEFT_ELBOW),
        (LEFT_ELBOW, LEFT_WRIST),
        (RIGHT_SHOULDER, RIGHT_ELBOW),
        (RIGHT_ELBOW, RIGHT_WRIST),
        (NOSE, LEFT_SHOULDER),
        (NOSE, RIGHT_SHOULDER),
    ];
    let g = |i: usize| (pose.landmarks[i].x as i32, pose.landmarks[i].y as i32);
    for (a, b) in bones {
        draw_line_rgb(&mut buf, wu, hu, g(a), g(b), [0xcc, 0xcc, 0xcc]);
    }
    let r = (w / 220).max(4) as i32;
    for (i, l) in pose.landmarks.iter().enumerate() {
        let c = if (11..=16).contains(&i) {
            [0xff, 0x96, 0x00]
        } else {
            [0x00, 0xdc, 0x3c]
        };
        draw_disc_rgb(&mut buf, wu, hu, l.x as i32, l.y as i32, r, c);
    }

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
    lockbar: Option<&headtracking::calibration::LockbarQuadRgb>,
    meta: &[(String, String)],
) -> Result<(), String> {
    let painted = bake_overlays(width, height, rgb888, pose, lockbar);
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
    lockbar: Option<&headtracking::calibration::LockbarQuadRgb>,
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
    save_rgb_screenshot_at(&path, width, height, rgb888, pose, lockbar, meta)?;
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
