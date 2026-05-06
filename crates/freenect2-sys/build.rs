//! Build libfreenect2 statically from the vendored submodule and compile the
//! cxx shim that bridges its C++ API to Rust.

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

    // Build libfreenect2 statically with the CPU packet pipeline only.
    // No GPU, no OpenNI2, no examples — keeps the dep tree to libusb + libc++.
    let dst = cmake::Config::new(&vendor_dir)
        .profile("Release")
        .define("BUILD_SHARED_LIBS", "OFF")
        .define("BUILD_EXAMPLES", "OFF")
        .define("BUILD_OPENNI2_DRIVER", "OFF")
        .define("ENABLE_OPENGL", "OFF")
        .define("ENABLE_OPENCL", "OFF")
        .define("ENABLE_CUDA", "OFF")
        .define("ENABLE_VAAPI", "OFF")
        .define("ENABLE_TEGRAJPEG", "OFF")
        .define("ENABLE_LIBJPEG_TURBO", "OFF")
        .define("CMAKE_POSITION_INDEPENDENT_CODE", "ON")
        .build();

    println!("cargo:rustc-link-search=native={}/lib", dst.display());
    println!("cargo:rustc-link-lib=static=freenect2");

    // libfreenect2 hard-requires TurboJPEG at cmake time (FindTurboJPEG.cmake
    // is REQUIRED) for the RGB JPEG decoder. We never use the RGB stream, but
    // the symbols still end up linked into libfreenect2.a, so we must satisfy
    // them.
    //
    // Ubuntu's `libturbojpeg.a` is built without `-fPIC` and refuses to land
    // in a `cdylib`, so we link against the shared `libturbojpeg.so.0`
    // instead. This brings back a small runtime user dep on Linux/macOS.
    // TODO (task #11): patch libfreenect2's CMakeLists to make TurboJPEG
    // genuinely optional so we can drop the dep entirely; or vendor
    // libjpeg-turbo with PIC and link statically.
    println!("cargo:rustc-link-lib=dylib=turbojpeg");

    // libfreenect2 USB transport — system libusb on Linux/macOS for now.
    // Switching to a vendored libusb (libusb1-sys/static) is the next step
    // toward fully self-contained binaries.
    pkg_config::Config::new()
        .atleast_version("1.0.20")
        .probe("libusb-1.0")
        .expect("libusb-1.0 not found via pkg-config");

    // C++ runtime
    if cfg!(target_os = "linux") {
        println!("cargo:rustc-link-lib=dylib=stdc++");
    } else if cfg!(target_os = "macos") {
        println!("cargo:rustc-link-lib=dylib=c++");
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
