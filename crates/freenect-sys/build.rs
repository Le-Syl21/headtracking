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

    // libfreenect's src/CMakeLists.txt unconditionally declares
    // `add_library(freenect SHARED ...)` *and*
    // `add_library(freenectstatic STATIC ... OUTPUT_NAME freenect)`. On
    // Linux/macOS the two outputs differ by extension (.so/.dylib vs
    // .a), so cmake is happy. On Windows + Ninja both targets produce
    // `freenect.lib`, and Ninja errors out with
    // "multiple rules generate lib/freenect.lib". Wrap the SHARED
    // target's declaration + install + link lines in `if(NOT WIN32)`
    // so only the static lib remains on Windows.
    patch_libfreenect_skip_shared(&vendor_dir);

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
        // BUILD_REDIST_PACKAGE=ON. The OFF branch's `firmware` custom
        // target runs `fwfetcher.py` to download Microsoft-hosted Kinect
        // audio firmware files via HTTPS — the URLs and CDN are flaky
        // in CI (HTTP 403 / SSL handshake failures) and we don't use
        // the audio path anyway (`BUILD_AUDIO=OFF`). The ON branch only
        // installs `fwfetcher.py` to share/ as a script we never run,
        // sidestepping the network dep entirely. This also removes the
        // need for a working Python at build time.
        .define("BUILD_REDIST_PACKAGE", "ON")
        .define("CMAKE_POSITION_INDEPENDENT_CODE", "ON");

    // libusb-sys (the dedicated workspace member) handles all platform
    // sourcing — vendored static build on Windows, pkg-config probe on
    // Linux/macOS. It exposes its outputs to us via cargo metadata
    // surfaced as `DEP_USB_1_0_*` env vars. We just feed those into the
    // libfreenect cmake invocation.
    // Cargo's `links` → env-var translation preserves the `.` literally
    // (only `-` becomes `_`), so `links = "usb-1.0"` produces vars named
    // `DEP_USB_1.0_*`, not `DEP_USB_1_0_*`. Unusual but documented.
    let libusb_include =
        env::var("DEP_USB_1.0_INCLUDE").expect("libusb-sys must expose DEP_USB_1.0_INCLUDE");
    let libusb_lib = env::var("DEP_USB_1.0_LIB").expect("libusb-sys must expose DEP_USB_1.0_LIB");

    // FindLibUSB shim — same idea as before: stop libfreenect's bundled
    // module from re-probing pkg-config when we've already pinned the
    // values via cmake -D below.
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let modules_dir = write_findlibusb_shim(&out_dir);
    let modules_path = modules_dir.to_string_lossy().replace('\\', "/");
    config.define("CMAKE_MODULE_PATH", &modules_path);

    // Forward slashes for Windows cmake compatibility (backslashes are
    // interpreted as escape characters in cmake variable expansion).
    let lib_s = libusb_lib.replace('\\', "/");
    let include_s = libusb_include.replace('\\', "/");
    // libfreenect's `Findlibusb-1.0.cmake` checks the `_1_` form. Keep
    // the older CamelCase / no-suffix variants too as defensive aliases
    // in case the bundled module gets refactored upstream.
    config
        .define("LIBUSB_1_LIBRARIES", &lib_s)
        .define("LIBUSB_1_INCLUDE_DIRS", &include_s)
        .define("LIBUSB_LIBRARIES", &lib_s)
        .define("LIBUSB_INCLUDE_DIRS", &include_s)
        .define("LibUSB-1.0_LIBRARIES", &lib_s)
        .define("LibUSB-1.0_INCLUDE_DIRS", &include_s);

    let dst = config.build();

    println!("cargo:rustc-link-search=native={}/lib", dst.display());
    println!("cargo:rustc-link-lib=static=freenect");

    // Re-emit libusb's link directives. libusb-sys's own build script
    // emits these too, but rustc elides empty rlibs (libusb-sys's
    // lib.rs has no public Rust symbols → no rlib content → its
    // `cargo:rustc-link-lib=` directives never reach the final
    // cdylib's link line). Replaying them from this crate, whose
    // .rlib *is* referenced by `freenect`, keeps them alive.
    let libusb_lib_dir = env::var("DEP_USB_1.0_LIB_DIR").unwrap_or_default();
    if !libusb_lib_dir.is_empty() {
        println!("cargo:rustc-link-search=native={libusb_lib_dir}");
    }
    let libusb_lib_name = env::var("DEP_USB_1.0_LIB_NAME").unwrap_or_else(|_| "usb-1.0".into());
    let libusb_link_kind = env::var("DEP_USB_1.0_LINK_KIND").unwrap_or_else(|_| "dylib".into());
    println!("cargo:rustc-link-lib={libusb_link_kind}={libusb_lib_name}");

    // System-lib deps that the static libusb archive pulls in. Same
    // elision rationale as the libusb directives just above:
    // libusb-sys's rlib is empty, rustc drops it from the final
    // cdylib's link line, and any `cargo:rustc-link-lib=` it emitted
    // disappears with it. We re-emit from this crate's build script
    // (its rlib *is* referenced) to keep them alive.
    let target_os_for_sys = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    match target_os_for_sys.as_str() {
        "windows" => {
            for sys in ["user32", "advapi32", "setupapi", "ole32"] {
                println!("cargo:rustc-link-lib={sys}");
            }
        }
        "macos" => {
            // libusb's darwin_usb.c uses CoreFoundation /
            // IOKit / Security frameworks plus the Objective-C
            // runtime (notification path).
            for fw in ["IOKit", "CoreFoundation", "Security"] {
                println!("cargo:rustc-link-lib=framework={fw}");
            }
            println!("cargo:rustc-link-lib=objc");
        }
        _ => {}
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

/// Idempotently patch `vendor/libfreenect/src/CMakeLists.txt` to skip
/// the SHARED `freenect` target on Windows AND macOS. Reasons differ:
///
///   * Windows + Ninja: SHARED + STATIC both resolve to `freenect.lib`
///     and Ninja (single-config) errors out with "multiple rules
///     generate lib/freenect.lib".
///   * macOS: the SHARED target's link line tries to resolve symbols
///     from libusb-1.0.a (now built static via `libusb-sys`), which on
///     macOS pulls in CoreFoundation / IOKit / Security framework
///     symbols. cmake's `target_link_libraries(freenect
///     ${LIBUSB_1_LIBRARIES})` doesn't carry those frameworks as
///     transitive deps, so the link errors out with `kCFAllocatorDefault`
///     etc. undefined. We don't consume `libfreenect.dylib` from Rust
///     anyway (only the `.a`), so dropping the SHARED build is the
///     cleanest fix.
///
/// Patch is a no-op on Linux (system libusb is dynamic, the .so
/// resolves transitive frameworks-equivalent libs cleanly) and
/// re-runs are idempotent via a marker comment.
fn patch_libfreenect_skip_shared(vendor_dir: &std::path::Path) {
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    if target_os != "windows" && target_os != "macos" {
        return;
    }
    let path = vendor_dir.join("src/CMakeLists.txt");
    let Ok(original) = std::fs::read_to_string(&path) else {
        return;
    };
    const MARKER: &str = "# freenect-sys: skip SHARED on Windows + macOS (build-deps conflict)";
    if original.contains(MARKER) {
        return;
    }
    let nl = if original.contains("\r\n") {
        "\r\n"
    } else {
        "\n"
    };
    // CMake guard expression: WIN32 OR APPLE — covers both targets we
    // want to skip the SHARED build on, in a single conditional block
    // applied at C-source patch time. The same NOT condition is used
    // for the install + target_link_libraries lines below.
    const GUARD_OPEN: &str = "if(NOT WIN32 AND NOT APPLE)";
    let mut out = String::with_capacity(original.len() + 256);
    out.push_str(MARKER);
    out.push_str(nl);

    let mut iter = original.split(nl);
    while let Some(line) = iter.next() {
        let trimmed = line.trim();
        if trimmed == "add_library (freenect SHARED ${SRC})" {
            out.push_str(GUARD_OPEN);
            out.push_str(nl);
            out.push_str(line);
            out.push_str(nl);
            // Consume up to and including the matching
            // `install (TARGETS freenect ... DESTINATION …)` block.
            for next in iter.by_ref() {
                out.push_str(next);
                out.push_str(nl);
                if next.contains("PROJECT_LIBRARY_INSTALL_DIR") && next.contains("DESTINATION") {
                    break;
                }
            }
            out.push_str("endif()  # NOT WIN32 AND NOT APPLE (freenect SHARED)");
            out.push_str(nl);
            continue;
        }
        if trimmed == "target_link_libraries (freenect ${LIBUSB_1_LIBRARIES})" {
            out.push_str(GUARD_OPEN);
            out.push_str(nl);
            out.push_str(line);
            out.push_str(nl);
            out.push_str("endif()");
            out.push_str(nl);
            continue;
        }
        out.push_str(line);
        out.push_str(nl);
    }
    // Strip the trailing newline we always appended on the final empty
    // line of `split`.
    if out.ends_with(nl) && !original.ends_with(nl) {
        for _ in 0..nl.len() {
            out.pop();
        }
    }
    let _ = std::fs::write(&path, out);
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
