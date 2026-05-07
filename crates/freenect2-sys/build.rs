//! Build libfreenect2 statically from the vendored submodule and compile the
//! cxx shim that bridges its C++ API to Rust.

use std::env;
use std::path::PathBuf;

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

    // On Windows, libfreenect2's `find_package(LibUSB)` falls back to
    // `pkg_check_modules(libusb-1.0)`, which requires a working pkg-config
    // installation that can resolve vcpkg's .pc files. Even with pkgconf
    // on PATH and PKG_CONFIG_PATH set, that path fails on the GitHub
    // runners (the FindPkgConfig module reports "libusb-1.0 not found").
    // Short-circuit by handing libfreenect2 the LibUSB_* values directly
    // — find_package then skips pkg_check_modules entirely.
    if let Some((lib, include)) = vcpkg_libusb() {
        config
            .define("LibUSB_LIBRARIES", lib.to_string_lossy().as_ref())
            .define("LibUSB_INCLUDE_DIRS", include.to_string_lossy().as_ref());
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

    // libfreenect2 USB transport — system libusb on Linux/macOS, vcpkg on
    // Windows (pkg-config isn't reliable there even with pkgconf installed).
    if let Some((lib, _)) = vcpkg_libusb() {
        if let Some(parent) = lib.parent() {
            println!("cargo:rustc-link-search=native={}", parent.display());
        }
        // The .lib name on Windows is `libusb-1.0.lib`; Rust's link
        // directive omits the extension and the `lib` prefix on MSVC.
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
    } else if cfg!(target_os = "macos") {
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

/// Locate the vcpkg-managed libusb on Windows. Returns
/// `(import_lib, include_dir)` where `include_dir` is the directory
/// containing `libusb.h` (i.e. `…/include/libusb-1.0`). Returns `None`
/// outside Windows or when vcpkg env vars aren't set / libusb isn't
/// installed.
fn vcpkg_libusb() -> Option<(PathBuf, PathBuf)> {
    if env::var_os("CARGO_CFG_TARGET_OS").as_deref() != Some(std::ffi::OsStr::new("windows")) {
        return None;
    }
    let root = env::var_os("VCPKG_ROOT").or_else(|| env::var_os("VCPKG_INSTALLATION_ROOT"))?;
    let triplet = env::var("VCPKG_TARGET_TRIPLET").unwrap_or_else(|_| "x64-windows".to_string());
    let installed = PathBuf::from(root).join("installed").join(triplet);
    let lib = installed.join("lib").join("libusb-1.0.lib");
    let include = installed.join("include").join("libusb-1.0");
    if lib.is_file() && include.is_dir() {
        Some((lib, include))
    } else {
        None
    }
}
