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
    // ignoring any pre-set LibUSB_* variables. On Windows (vcpkg-managed
    // libusb) and macOS cross-compile (Intel libusb under /usr/local on
    // an ARM runner) that path doesn't resolve. Override the module via
    // CMAKE_MODULE_PATH with a shim that respects pre-set values, then
    // pre-set them when we know where libusb lives.
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let modules_dir = write_findlibusb_shim(&out_dir);
    // CMake on Windows accepts forward slashes; passing native
    // backslashes triggers "Syntax error in cmake code" inside the
    // auto-generated try_compile scratch file because cmake interprets
    // `\` as an escape sequence in `set(...)` arguments.
    let modules_path = modules_dir.to_string_lossy().replace('\\', "/");
    config.define("CMAKE_MODULE_PATH", &modules_path);
    if let Some((lib, include)) = detect_libusb_paths() {
        let lib = lib.to_string_lossy().replace('\\', "/");
        let include = include.to_string_lossy().replace('\\', "/");
        config
            .define("LibUSB_LIBRARIES", &lib)
            .define("LibUSB_INCLUDE_DIRS", &include);
    }

    let dst = config.build();

    println!("cargo:rustc-link-search=native={}/lib", dst.display());
    println!("cargo:rustc-link-lib=static=freenect2");
    // Static archives are scanned once in the order they appear on the link
    // line. libfreenect2.a references tj* / jpeg* symbols, so the JPEG
    // archives MUST come after it. turbojpeg-sys emits its own link
    // directives, but they end up before freenect2 (deps come first), so we
    // re-emit them here to force the right order.
    println!("cargo:rustc-link-search=native={tj_root}/lib");
    println!("cargo:rustc-link-lib=static=turbojpeg");
    println!("cargo:rustc-link-lib=static=jpeg");

    // libfreenect2 USB transport. On platforms where pkg-config is
    // unreliable (Windows vcpkg, macOS cross-compile) we already located
    // libusb above; emit the link directives directly. Otherwise let
    // pkg-config probe the system install.
    if let Some((lib, _)) = detect_libusb_paths() {
        if let Some(parent) = lib.parent() {
            println!("cargo:rustc-link-search=native={}", parent.display());
        }
        // On Windows the import lib is `libusb-1.0.lib`; rustc strips the
        // .lib extension. On macOS we link the .dylib by name.
        println!("cargo:rustc-link-lib=libusb-1.0");
    } else {
        pkg_config::Config::new()
            .atleast_version("1.0.20")
            .probe("libusb-1.0")
            .expect("libusb-1.0 not found via pkg-config");
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

/// Locate libusb for targets where pkg-config can't be trusted:
/// Windows (vcpkg-managed) and macOS x86_64 cross-compile from an
/// Apple Silicon runner (Intel libusb under /usr/local from
/// `arch -x86_64 brew install libusb`). Returns `(library, include_dir)`
/// or `None` to fall back to pkg-config.
fn detect_libusb_paths() -> Option<(PathBuf, PathBuf)> {
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let target_arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();

    if target_os == "windows" {
        let root = env::var_os("VCPKG_ROOT").or_else(|| env::var_os("VCPKG_INSTALLATION_ROOT"))?;
        let triplet = env::var("VCPKG_TARGET_TRIPLET").unwrap_or_else(|_| "x64-windows".into());
        let installed = PathBuf::from(root).join("installed").join(triplet);
        let lib = installed.join("lib").join("libusb-1.0.lib");
        let include = installed.join("include").join("libusb-1.0");
        return (lib.is_file() && include.is_dir()).then_some((lib, include));
    }

    if target_os == "macos" && target_arch == "x86_64" {
        let lib = PathBuf::from("/usr/local/lib/libusb-1.0.dylib");
        let include = PathBuf::from("/usr/local/include/libusb-1.0");
        if lib.is_file() && include.is_dir() {
            return Some((lib, include));
        }
    }

    None
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
