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

    // Build libfreenect2 statically with the CPU packet pipeline only.
    // No GPU, no OpenNI2, no examples — keeps the dep tree to libusb + libc++
    // plus the static libjpeg-turbo we hand it.
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
        .define("BUILD_SHARED_LIBS", "OFF")
        .define("BUILD_EXAMPLES", "OFF")
        .define("BUILD_OPENNI2_DRIVER", "OFF")
        .define("ENABLE_OPENGL", "OFF")
        .define("ENABLE_OPENCL", "OFF")
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

    // libfreenect2's bundled FindLibUSB.cmake unconditionally calls
    // `pkg_check_modules(libusb-1.0)` whenever PKG_CONFIG is found,
    // ignoring any pre-set LibUSB_* variables. Override the module via
    // CMAKE_MODULE_PATH with a shim that respects pre-set values, then
    // pre-set them from the libusb-sys metadata.
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let modules_dir = write_findlibusb_shim(&out_dir);
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

    // libfreenect2's USB transport. libusb-sys already emits the right
    // `cargo:rustc-link-lib=` for the platform (static usb-1.0 + Win32
    // helpers on Windows via cc-rs; pkg-config dynamic link directives
    // on Linux/macOS) — nothing to add here.

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
