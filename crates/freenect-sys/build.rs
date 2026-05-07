//! Build libfreenect statically from the vendored submodule and generate
//! Rust bindings via bindgen.

use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-changed=build.rs");

    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let vendor_dir = manifest_dir.join("vendor/libfreenect");

    if !vendor_dir.join("CMakeLists.txt").is_file() {
        panic!(
            "libfreenect sources missing at {}\n\
             run: git submodule update --init --recursive",
            vendor_dir.display()
        );
    }

    // Static cmake build. Drop the bits we don't need (audio, OpenNI2, examples).
    let mut config = cmake::Config::new(&vendor_dir);
    config
        .profile("Release")
        .define("BUILD_SHARED_LIBS", "OFF")
        .define("BUILD_EXAMPLES", "OFF")
        .define("BUILD_FAKENECT", "OFF")
        .define("BUILD_C_SYNC", "OFF")
        .define("BUILD_CPP", "OFF")
        .define("BUILD_CV", "OFF")
        .define("BUILD_PYTHON", "OFF")
        .define("BUILD_AS3_SERVER", "OFF")
        .define("BUILD_AUDIO", "OFF")
        .define("BUILD_OPENNI2_DRIVER", "OFF")
        .define("BUILD_REDIST_PACKAGE", "OFF")
        .define("CMAKE_POSITION_INDEPENDENT_CODE", "ON");

    // Same Windows libusb workaround as freenect2-sys: pre-fill
    // LibUSB_* so libfreenect's CMakeLists doesn't try pkg_check_modules.
    if let Some((lib, include)) = vcpkg_libusb() {
        config
            .define("LIBUSB_LIBRARIES", lib.to_string_lossy().as_ref())
            .define("LIBUSB_INCLUDE_DIRS", include.to_string_lossy().as_ref())
            // libfreenect's cmake module is named LibUSB-1.0 (with dash)
            // — define both naming conventions defensively.
            .define("LibUSB-1.0_LIBRARIES", lib.to_string_lossy().as_ref())
            .define(
                "LibUSB-1.0_INCLUDE_DIRS",
                include.to_string_lossy().as_ref(),
            );
    }

    let dst = config.build();

    println!("cargo:rustc-link-search=native={}/lib", dst.display());
    println!("cargo:rustc-link-lib=static=freenect");

    if let Some((lib, _)) = vcpkg_libusb() {
        if let Some(parent) = lib.parent() {
            println!("cargo:rustc-link-search=native={}", parent.display());
        }
        println!("cargo:rustc-link-lib=libusb-1.0");
    } else {
        pkg_config::Config::new()
            .atleast_version("1.0.20")
            .probe("libusb-1.0")
            .expect("libusb-1.0 not found via pkg-config");
    }

    // Bindgen on the public header. Resource-dir fallback mirrors the root
    // build.rs (handles dev systems without libclang-common-* installed).
    let mut builder = bindgen::Builder::default()
        .header("wrapper.h")
        .clang_arg(format!("-I{}/include", dst.display()))
        .clang_arg(format!("-I{}/include/libfreenect", dst.display()))
        .allowlist_function("freenect_.*")
        .allowlist_type("freenect_.*")
        .allowlist_var("FREENECT_.*")
        .derive_default(true)
        .derive_debug(true)
        .layout_tests(false);

    if let Some(dir) = compiler_resource_include_dir() {
        builder = builder
            .clang_arg("-isystem")
            .clang_arg(dir.to_string_lossy().into_owned());
    }

    let bindings = builder
        .generate()
        .expect("bindgen failed for libfreenect.h");
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap()).join("bindings.rs");
    bindings
        .write_to_file(&out_path)
        .expect("failed to write libfreenect bindings");
}

/// Same heuristic as the root `build.rs`: walk through known clang resource
/// dirs, fall back to GCC's, give up if nothing has `stddef.h`.
fn compiler_resource_include_dir() -> Option<PathBuf> {
    if let Ok(out) = Command::new("clang").arg("-print-resource-dir").output()
        && out.status.success()
    {
        let dir = PathBuf::from(String::from_utf8_lossy(&out.stdout).trim()).join("include");
        if dir.join("stddef.h").is_file() {
            return Some(dir);
        }
    }
    let candidates: &[&str] = &[
        "/usr/lib/llvm-19/lib/clang/19/include",
        "/usr/lib/llvm-18/lib/clang/18/include",
        "/usr/lib/llvm-17/lib/clang/17/include",
        "/usr/lib/clang/19/include",
        "/usr/lib/clang/18/include",
        "/usr/lib/clang/17/include",
    ];
    for path in candidates {
        let p = std::path::Path::new(path);
        if p.join("stddef.h").is_file() {
            return Some(p.to_path_buf());
        }
    }
    let gcc_root = std::path::Path::new("/usr/lib/gcc/x86_64-linux-gnu");
    if let Ok(entries) = std::fs::read_dir(gcc_root) {
        let mut versions: Vec<PathBuf> = entries
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.join("include").join("stddef.h").is_file())
            .collect();
        versions.sort();
        if let Some(p) = versions.pop() {
            return Some(p.join("include"));
        }
    }
    None
}

/// Locate vcpkg-managed libusb on Windows. Mirrors the same helper in
/// `freenect2-sys`. See its docstring for the contract.
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
