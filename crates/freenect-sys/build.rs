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
    // Windows: prefer Ninja (single-config) so the static lib lands at
    // `<dst>/lib/freenect.lib` rather than `<dst>/lib/Release/...`.
    if env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows") {
        config.generator("Ninja");
    }
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

    // FindLibUSB shim + pre-set values for Windows / macOS x86_64
    // cross-compile. See freenect2-sys/build.rs for the rationale.
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let modules_dir = write_findlibusb_shim(&out_dir);
    // Forward slashes for Windows cmake compatibility (see
    // freenect2-sys/build.rs comment).
    let modules_path = modules_dir.to_string_lossy().replace('\\', "/");
    config.define("CMAKE_MODULE_PATH", &modules_path);
    if let Some((lib, include)) = detect_libusb_paths() {
        let lib_s = lib.to_string_lossy().replace('\\', "/");
        let include_s = include.to_string_lossy().replace('\\', "/");
        config
            .define("LIBUSB_LIBRARIES", &lib_s)
            .define("LIBUSB_INCLUDE_DIRS", &include_s)
            // libfreenect's cmake module is named LibUSB-1.0 (with a
            // hyphen). Define both conventions defensively.
            .define("LibUSB-1.0_LIBRARIES", &lib_s)
            .define("LibUSB-1.0_INCLUDE_DIRS", &include_s);
    }

    let dst = config.build();

    println!("cargo:rustc-link-search=native={}/lib", dst.display());
    println!("cargo:rustc-link-lib=static=freenect");

    if let Some((lib, _)) = detect_libusb_paths() {
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

/// Locate libusb on platforms where pkg-config is unreliable. Mirrors
/// the same helper in `freenect2-sys`. See its docstring for the
/// contract.
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

/// Write a `FindLibUSB.cmake` shim that respects pre-set values. Same
/// rationale as `freenect2-sys/build.rs`'s shim; libfreenect's bundled
/// module also unconditionally calls `pkg_check_modules`.
fn write_findlibusb_shim(out_dir: &std::path::Path) -> PathBuf {
    let modules_dir = out_dir.join("cmake_modules");
    std::fs::create_dir_all(&modules_dir).expect("create cmake_modules dir");
    let shim = r#"# Generated by freenect-sys/build.rs.
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
