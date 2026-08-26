//! Build libfreenect2 statically from the vendored submodule and compile the
//! cxx shim that bridges its C++ API to Rust.

use std::env;
use std::path::{Path, PathBuf};
use std::process::Command;

fn main() {
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/shim.h");
    println!("cargo:rerun-if-changed=src/shim.cpp");
    println!("cargo:rerun-if-changed=build.rs");

    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let vendor_dir = manifest_dir.join("vendor/libfreenect2");

    if !vendor_dir.join("CMakeLists.txt").is_file() {
        panic!(
            "libfreenect2 sources missing at {}\n\
             run: git submodule update --init --recursive",
            vendor_dir.display()
        );
    }

    // Cross-compile fix: libfreenect2's GenerateResources.cmake does
    //   COMMAND generate_resources_tool …
    // which CMake expects to auto-resolve to the target's binary path.
    // When cross-compiling macOS x86_64 from an ARM runner, the rule
    // ends up invoking the bare name through `/bin/sh` and fails with
    // "generate_resources_tool: command not found". Replacing with a
    // `$<TARGET_FILE:…>` generator expression forces cmake to emit
    // the resolved absolute path.
    patch_libfreenect2_resource_tool_path(&vendor_dir);
    patch_libfreenect2_icdl_version_clash(&vendor_dir);

    // libfreenect2's CMakeLists `find_package(TurboJPEG REQUIRED)` — point it
    // at the vendored libjpeg-turbo built by `turbojpeg-sys` so we don't need
    // anything from the system. `DEP_TURBOJPEG_*` come from `turbojpeg-sys`'s
    // build script (Cargo propagates them because that crate declares
    // `links = "turbojpeg"`).
    let tj_root = env::var("DEP_TURBOJPEG_ROOT")
        .or_else(|_| env::var("DEP_TURBOJPEG_OUT_DIR"))
        .expect(
            "turbojpeg-sys did not export DEP_TURBOJPEG_ROOT — \
             ensure freenect2-sys depends on turbojpeg-sys with the `cmake` feature",
        );
    let tj_include =
        env::var("DEP_TURBOJPEG_INCLUDE").unwrap_or_else(|_| format!("{tj_root}/include"));
    // libturbojpeg.a depends on libjpeg.a internally; pass both as a cmake
    // list so libfreenect2's TURBOJPEG_WORKS try-compile actually links.
    let tj_libs = format!("{tj_root}/lib/libturbojpeg.a;{tj_root}/lib/libjpeg.a");

    // Build libfreenect2 statically, with the OpenCL depth pipeline requested
    // on EVERY platform.
    //
    // The Kinect v2's phase unwrap + bilateral/edge-aware filtering is the
    // expensive part, and on the CPU pipeline it does not merely run slowly:
    // libfreenect2 starts dropping USB depth packets it cannot consume in
    // time (`DepthPacketStreamParser: N packets were lost`), and depth lands
    // at ~5 fps instead of 30. Field report from a Windows tester: the head
    // position then updates five times a second, and the parallax follows
    // "as if you would be drunk". This was CPU-only everywhere but Linux.
    //
    // No OpenGL (needs GLFW3), no CUDA (needs the toolkit at build time), no
    // OpenNI2, no examples — the dep tree stays libusb + libc++ + the static
    // libjpeg-turbo we hand it, plus the platform's OpenCL loader.
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let mut config = cmake::Config::new(&vendor_dir);
    // On Windows, force Ninja over the default Visual Studio generator.
    // VS is multi-config and installs static libs at
    // `<dst>/lib/Release/freenect2.lib`, but our `rustc-link-search`
    // points at `<dst>/lib`. Ninja is single-config and aligns the two.
    // windows-latest runners ship `ninja.exe` in PATH (via Visual Studio
    // Build Tools / chocolatey).
    if env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows") {
        config.generator("Ninja");
    }
    config
        .profile("Release")
        // libfreenect2's top-level `cmake_minimum_required` predates 3.5,
        // which CMake ≥ 4 refuses outright. Vendored source we don't want
        // to patch, so tell CMake to apply ≥3.5 policies to the old
        // project. (Env var `CMAKE_POLICY_VERSION_MINIMUM` works too, but
        // baking it in keeps CI and local builds reproducible.)
        .define("CMAKE_POLICY_VERSION_MINIMUM", "3.5")
        .define("BUILD_SHARED_LIBS", "OFF")
        .define("BUILD_EXAMPLES", "OFF")
        .define("BUILD_OPENNI2_DRIVER", "OFF")
        .define("ENABLE_OPENGL", "OFF")
        // libfreenect2 treats ENABLE_OPENCL as a *request*: if
        // `find_package(OpenCL)` fails in the build image it silently
        // compiles CPU-only, `LIBFREENECT2_WITH_OPENCL_SUPPORT` stays
        // undefined, and the shim falls back to the CPU pipeline. So asking
        // for it can never break a build that lacks the SDK — it only decides
        // whether the GPU path exists in the binary at all. Asking everywhere
        // is therefore free; NOT asking is what left Windows and macOS on the
        // CPU. See shim.cpp for the runtime fallback.
        .define("ENABLE_OPENCL", "ON")
        .define("ENABLE_CUDA", "OFF")
        .define("ENABLE_VAAPI", "OFF")
        .define("ENABLE_TEGRAJPEG", "OFF")
        .define("CMAKE_POSITION_INDEPENDENT_CODE", "ON")
        .define("TurboJPEG_INCLUDE_DIRS", &tj_include)
        .define("TurboJPEG_LIBRARIES", &tj_libs)
        // libfreenect2's FindTurboJPEG.cmake runs `check_c_source_compiles`
        // to validate the lib via try_compile. cmake mangles our `;`-list of
        // .a paths and that test fails — but we don't actually need it: the
        // final cdylib link is wired by Cargo through turbojpeg-sys's
        // transitive link metadata. Pre-cache the success flag.
        .define("TURBOJPEG_WORKS", "TRUE");
    // On Windows the OpenCL loader comes from a *different* vcpkg triplet
    // (`-static-md`, so it links into us instead of shipping a DLL), which the
    // vcpkg toolchain's own `find_package` will not look in. When CI hands us
    // the paths, pass them straight through rather than relying on a CMake
    // policy to resolve `OpenCL_ROOT` inside a project whose
    // `cmake_minimum_required` predates it.
    if let Ok(dir) = env::var("HT_OPENCL_INCLUDE_DIR") {
        config.define("OpenCL_INCLUDE_DIR", &dir);
    }
    if let Ok(lib) = env::var("HT_OPENCL_LIBRARY") {
        config.define("OpenCL_LIBRARY", &lib);
    }

    // libfreenect2's bundled FindLibUSB.cmake unconditionally calls
    // `pkg_check_modules(libusb-1.0)` whenever PKG_CONFIG is found,
    // ignoring any pre-set LibUSB_* variables. Override the module via
    // CMAKE_MODULE_PATH with a shim that respects pre-set values, then
    // pre-set them from the libusb-sys metadata.
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let modules_dir = write_findlibusb_shim(&out_dir);
    // Same backslash trap one level up: cmake-rs passes OUT_DIR as the
    // native-path install prefix, CMake copies it VERBATIM into the
    // generated cmake_install.cmake, and `D:\a\...` (GitHub runners'
    // workspace root!) dies on the invalid `\a` escape at install time.
    // Overriding the prefix with forward slashes keeps every generated
    // script parseable.
    config.define(
        "CMAKE_INSTALL_PREFIX",
        out_dir.to_string_lossy().replace('\\', "/"),
    );
    // CMake on Windows accepts forward slashes; passing native
    // backslashes triggers "Syntax error in cmake code" inside the
    // auto-generated try_compile scratch file because cmake interprets
    // `\` as an escape sequence in `set(...)` arguments.
    let modules_path = modules_dir.to_string_lossy().replace('\\', "/");
    config.define("CMAKE_MODULE_PATH", &modules_path);

    // libusb-sys exposes its location via DEP_USB_1.0_* env vars. Note
    // the literal `.` — Cargo's `links` → env-var translation preserves
    // it (only `-` becomes `_`). See `crates/libusb-sys/build.rs`.
    let libusb_lib = env::var("DEP_USB_1.0_LIB").expect("libusb-sys must expose DEP_USB_1.0_LIB");
    let libusb_include =
        env::var("DEP_USB_1.0_INCLUDE").expect("libusb-sys must expose DEP_USB_1.0_INCLUDE");
    let lib_s = libusb_lib.replace('\\', "/");
    let include_s = libusb_include.replace('\\', "/");
    config
        .define("LibUSB_LIBRARIES", &lib_s)
        .define("LibUSB_INCLUDE_DIRS", &include_s);

    let dst = config.build();

    println!("cargo:rustc-link-search=native={}/lib", dst.display());
    // libfreenect2's CMakeLists.txt has an MSVC-specific quirk:
    //   IF(MSVC AND NOT BUILD_SHARED_LIBS)
    //     set_target_properties(freenect2 PROPERTIES SUFFIX "static.lib")
    //   ENDIF()
    // so the static lib is shipped as `freenect2static.lib` on Windows
    // (default `.lib` suffix replaced by `static.lib`). Match that
    // naming when linking; everywhere else the lib is plainly
    // `libfreenect2.a` / `libfreenect2.dylib`.
    if env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows") {
        println!("cargo:rustc-link-lib=static=freenect2static");
    } else {
        println!("cargo:rustc-link-lib=static=freenect2");
    }
    // Only pull in the OpenCL loader if libfreenect2 actually compiled the
    // pipeline. The request above is best-effort — with no SDK in the build
    // image `find_package(OpenCL)` fails, the archive references no `cl*`
    // symbols, and asking to link OpenCL would be an error against a library
    // that isn't there. The installed `config.h` records the outcome via
    // `LIBFREENECT2_WITH_OPENCL_SUPPORT`; trust that, not our own guess. Must
    // follow `-lfreenect2` on the link line (static archive scanned once).
    let cfg_h = dst.join("include/libfreenect2/config.h");
    let has_opencl = std::fs::read_to_string(&cfg_h)
        .map(|s| s.contains("#define LIBFREENECT2_WITH_OPENCL_SUPPORT"))
        .unwrap_or(false);
    // Publish the outcome so it can be asserted rather than assumed: a `cfg`
    // for our own tests, and link metadata (`DEP_FREENECT2_OPENCL`) for the
    // crates above us. Shipping the CPU pipeline by accident is exactly what
    // happened before, and it is invisible until someone's depth stream
    // collapses to 5 fps in the field.
    println!("cargo::rustc-check-cfg=cfg(freenect2_opencl)");
    if has_opencl {
        println!("cargo::rustc-cfg=freenect2_opencl");
        println!("cargo::metadata=opencl=1");
        match target_os.as_str() {
            // A system framework since 10.7. Deprecated by Apple, still
            // present and still the only GPU path we can use without a
            // vendor SDK.
            "macos" => println!("cargo:rustc-link-lib=framework=OpenCL"),
            // Windows links the Khronos loader statically (see the release
            // workflow), so it needs the loader's own Win32 dependencies:
            // the registry for the ICD list, and cfgmgr32 for the newer
            // device-enumeration path.
            //
            // rustc also needs the directory: telling CMake where the library
            // is says nothing to the Rust link step, and the whole build dies
            // on `could not find native static library OpenCL`. Without an
            // explicit path (a local build with no CI-provided loader) fall
            // back to the system's dynamic one.
            "windows" => {
                let static_lib = env::var("HT_OPENCL_LIBRARY")
                    .ok()
                    .map(PathBuf::from)
                    .filter(|p| p.is_file());
                if let Some(lib) = static_lib {
                    if let Some(dir) = lib.parent() {
                        println!("cargo:rustc-link-search=native={}", dir.display());
                    }
                    println!("cargo:rustc-link-lib=static=OpenCL");
                } else {
                    println!("cargo:rustc-link-lib=dylib=OpenCL");
                }
                println!("cargo:rustc-link-lib=dylib=cfgmgr32");
                println!("cargo:rustc-link-lib=dylib=ole32");
                println!("cargo:rustc-link-lib=dylib=advapi32");
            }
            // Linux: the distribution ships only the shared loader, and it is
            // present wherever a GPU driver is.
            _ => println!("cargo:rustc-link-lib=dylib=OpenCL"),
        }
    } else {
        println!(
            "cargo:warning=freenect2-sys: no OpenCL SDK in the build image — \
             the Kinect v2 depth pipeline falls back to the CPU, which drops \
             USB packets and delivers ~5 fps instead of 30."
        );
    }
    // Static archives are scanned once in the order they appear on the link
    // line. libfreenect2.a references tj* / jpeg* symbols, so the JPEG
    // archives MUST come after it. turbojpeg-sys emits its own link
    // directives, but they end up before freenect2 (deps come first), so we
    // re-emit them here to force the right order.
    println!("cargo:rustc-link-search=native={tj_root}/lib");
    // libjpeg-turbo's CMakeLists names the static libs `turbojpeg-static`
    // and `jpeg-static` on MSVC (matching `turbojpeg-sys`'s own link
    // directives). Everywhere else they're plain `libturbojpeg.a` /
    // `libjpeg.a`.
    let is_msvc = env::var("CARGO_CFG_TARGET_ENV").as_deref() == Ok("msvc");
    let tj_suffix = if is_msvc { "-static" } else { "" };
    println!("cargo:rustc-link-lib=static=turbojpeg{tj_suffix}");
    println!("cargo:rustc-link-lib=static=jpeg{tj_suffix}");

    // Re-emit libusb's link directives — same rationale as in
    // freenect-sys/build.rs: rustc elides the empty libusb-sys rlib,
    // taking its `cargo:rustc-link-lib=` directives down with it.
    // freenect2-sys's rlib *is* in the dep graph, so directives we
    // emit here survive to the cdylib's link line.
    let libusb_lib_dir = env::var("DEP_USB_1.0_LIB_DIR").unwrap_or_default();
    if !libusb_lib_dir.is_empty() {
        println!("cargo:rustc-link-search=native={libusb_lib_dir}");
    }
    let libusb_lib_name = env::var("DEP_USB_1.0_LIB_NAME").unwrap_or_else(|_| "usb-1.0".into());
    let libusb_link_kind = env::var("DEP_USB_1.0_LINK_KIND").unwrap_or_else(|_| "dylib".into());
    println!("cargo:rustc-link-lib={libusb_link_kind}={libusb_lib_name}");
    let target_os_for_sys = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    match target_os_for_sys.as_str() {
        "windows" => {
            for sys in ["user32", "advapi32", "setupapi", "ole32"] {
                println!("cargo:rustc-link-lib={sys}");
            }
        }
        "macos" => {
            // libusb's darwin_usb.c uses these frameworks; re-emit
            // here because libusb-sys's rlib gets elided.
            for fw in ["IOKit", "CoreFoundation", "Security"] {
                println!("cargo:rustc-link-lib=framework={fw}");
            }
            println!("cargo:rustc-link-lib=objc");
        }
        _ => {}
    }

    // C++ runtime
    if cfg!(target_os = "linux") {
        println!("cargo:rustc-link-lib=dylib=stdc++");
    } else if env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("macos") {
        println!("cargo:rustc-link-lib=dylib=c++");
        // libfreenect2's VTRgbPacketProcessorImpl uses Apple's
        // VideoToolbox to decode the JPEG colour stream; CoreMedia /
        // CoreVideo / CoreFoundation are its transitive dependencies.
        // Without these frameworks the final link fails with
        // "Undefined symbols for architecture arm64 …
        //  VTDecompressionSessionCreate / CMVideoFormatDescription* / …".
        for framework in ["VideoToolbox", "CoreMedia", "CoreVideo", "CoreFoundation"] {
            println!("cargo:rustc-link-lib=framework={framework}");
        }
        // SDL3 (and any clang-built object that uses
        // `__builtin_available()`) emits calls to
        // `__isPlatformVersionAtLeast`, a runtime version-check helper
        // that lives in libclang_rt.osx.a. Auto-linking is supposed to
        // happen, but cmake-rs invocations sometimes drop it. Explicitly
        // surface the path so the final cdylib / binary link finds it.
        if let Some(rt_lib_dir) = clang_runtime_lib_dir() {
            println!("cargo:rustc-link-search=native={}", rt_lib_dir.display());
            println!("cargo:rustc-link-lib=clang_rt.osx");
        }
    }

    // Compile our cxx bridge + C++ shim, linked against the static
    // libfreenect2 we just built.
    let mut bridge = cxx_build::bridge("src/lib.rs");
    bridge
        .file("src/shim.cpp")
        .include(manifest_dir.join("src"))
        .include(dst.join("include"))
        .flag_if_supported("-std=c++14")
        .flag_if_supported("-Wno-unused-parameter")
        .compile("freenect2-shim");
}

/// Write a `FindLibUSB.cmake` shim into `<out>/cmake_modules/`. The
/// shim respects pre-set `LibUSB_LIBRARIES` / `LibUSB_INCLUDE_DIRS`
/// and falls through to libfreenect2's bundled module otherwise.
/// libfreenect2's vendored module always invokes `pkg_check_modules`
/// regardless of pre-set values, breaking Windows / cross-compile.
fn write_findlibusb_shim(out_dir: &Path) -> PathBuf {
    let modules_dir = out_dir.join("cmake_modules");
    std::fs::create_dir_all(&modules_dir).expect("create cmake_modules dir");
    let shim = r#"# Generated by freenect2-sys/build.rs.
# Honour LibUSB_LIBRARIES / LibUSB_INCLUDE_DIRS already set by the
# parent cmake-rs invocation; fall through to the bundled module
# (which does pkg_check_modules) when they aren't.
if(LibUSB_LIBRARIES AND LibUSB_INCLUDE_DIRS)
  set(LibUSB_FOUND TRUE)
  return()
endif()
include("${CMAKE_SOURCE_DIR}/cmake_modules/FindLibUSB.cmake")
"#;
    std::fs::write(modules_dir.join("FindLibUSB.cmake"), shim).expect("write FindLibUSB shim");
    modules_dir
}

/// Idempotently rewrite libfreenect2's `GenerateResources.cmake` so
/// the custom command invokes `generate_resources_tool` via a
/// `$<TARGET_FILE:…>` generator expression instead of a bare name.
/// Bare-name invocation breaks under cross-compile (macOS x86_64
/// from an ARM runner) where the build dir's bin/ isn't on PATH and
/// the make rule can't find the tool.
/// libfreenect2 declares a local `const int CL_ICDL_VERSION = 2;` inside both
/// OpenCL packet processors. Since OpenCL 3.0 the loader-info extension makes
/// that an unconditional macro (`CL/cl_ext.h`: `#define CL_ICDL_VERSION 2`),
/// so the declaration expands to `const int 2 = 2` and the compile dies with
/// "expected unqualified-id before numeric constant".
///
/// Upstream v0.2.1 predates those headers and there is no release with the
/// fix, so we guard the name ourselves. Doing it here rather than by hand:
/// the same edit had been sitting uncommitted in the submodule's working tree,
/// which is why OpenCL built on this machine and nowhere else — CI checks out
/// a pristine v0.2.1 and the whole GPU pipeline failed to compile.
///
/// A submodule bump that renames or removes the declaration makes the anchor
/// miss; the build then fails loudly on the original error rather than
/// silently dropping the patch.
fn patch_libfreenect2_icdl_version_clash(vendor_dir: &Path) {
    const MARKER: &str = "// freenect2-sys: guard against the CL_ICDL_VERSION macro";
    let target = "    const int CL_ICDL_VERSION = 2;";
    let replacement = concat!(
        "// freenect2-sys: guard against the CL_ICDL_VERSION macro\n",
        "#ifdef CL_ICDL_VERSION\n",
        "#undef CL_ICDL_VERSION\n",
        "#endif\n",
        "    const int CL_ICDL_VERSION = 2;"
    );
    for name in [
        "src/opencl_depth_packet_processor.cpp",
        "src/opencl_kde_depth_packet_processor.cpp",
    ] {
        let path = vendor_dir.join(name);
        let Ok(original) = std::fs::read_to_string(&path) else {
            continue;
        };
        if original.contains(MARKER) || !original.contains(target) {
            continue;
        }
        let _ = std::fs::write(&path, original.replace(target, replacement));
    }
}

fn patch_libfreenect2_resource_tool_path(vendor_dir: &Path) {
    let path = vendor_dir.join("cmake_modules/GenerateResources.cmake");
    let Ok(original) = std::fs::read_to_string(&path) else {
        return;
    };
    const MARKER: &str = "# freenect2-sys: use TARGET_FILE for cross-compile";
    if original.contains(MARKER) {
        return;
    }
    let target = "COMMAND generate_resources_tool ${BASE_FOLDER} ${ARGN} > ${OUTPUT}";
    let replacement =
        "COMMAND $<TARGET_FILE:generate_resources_tool> ${BASE_FOLDER} ${ARGN} > ${OUTPUT}";
    if !original.contains(target) {
        return;
    }
    let patched = format!("{MARKER}\n{}", original.replace(target, replacement));
    let _ = std::fs::write(&path, patched);
}

/// Locate clang's compiler-rt darwin lib dir (where libclang_rt.osx.a
/// lives). Asks `clang -print-resource-dir` which returns something
/// like `/Applications/Xcode.app/.../usr/lib/clang/<version>`; the
/// runtime lib lives at `<that>/lib/darwin/`.
fn clang_runtime_lib_dir() -> Option<PathBuf> {
    let out = Command::new("clang")
        .arg("-print-resource-dir")
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    let resource = PathBuf::from(String::from_utf8_lossy(&out.stdout).trim());
    let darwin = resource.join("lib").join("darwin");
    darwin.is_dir().then_some(darwin)
}
