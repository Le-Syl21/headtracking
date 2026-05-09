//! libusb-1.0 sourcing.
//!
//! - **Windows**: build a static `usb-1.0` archive from the vendored
//!   `vendor/libusb` submodule via `cc::Build`. We hand-list the
//!   sources (libusb 1.0.29 still ships no upstream CMakeLists.txt)
//!   and pull `msvc/config.h` for the MSVC defines. Side-effects: emits
//!   `cargo:rustc-link-lib=` for the Win32 system libs libusb pulls in
//!   (`setupapi`, `advapi32`, `user32`, `ole32`).
//! - **Linux / macOS**: probe the system copy via `pkg-config`. The
//!   vendored sources are *not* used here — distros ship a stable
//!   libusb already, building from source would just diverge from
//!   what end-users have on their machines.
//!
//! In both cases we emit three metadata keys consumed by downstream
//! sys crates:
//!   `cargo:include=...`    → C header directory (passed to bindgen / cmake)
//!   `cargo:lib=...`        → full path to the archive / shared lib
//!                            (kept for cmake `LIBUSB_1_LIBRARIES`)
//!   `cargo:lib_name=...`   → short link name (e.g. `usb-1.0` on Unix,
//!                            `libusb-1.0` on MSVC)
//!
//! Cargo translates these into `DEP_USB_1_0_INCLUDE`, `DEP_USB_1_0_LIB`,
//! `DEP_USB_1_0_LIB_NAME` env vars in the build scripts of any direct
//! dependent.

use std::env;
use std::path::{Path, PathBuf};

fn main() {
    println!("cargo:rerun-if-changed=build.rs");

    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let vendor_root = manifest_dir.join("vendor/libusb");

    if !vendor_root.join("libusb/libusb.h").is_file() {
        panic!(
            "libusb sources missing at {}\n\
             run: git submodule update --init --recursive",
            vendor_root.display()
        );
    }

    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    if target_os == "windows" {
        build_windows_static(&vendor_root);
    } else {
        probe_unix_system();
    }
}

fn build_windows_static(vendor_root: &Path) {
    let libusb_dir = vendor_root.join("libusb");
    let os_dir = libusb_dir.join("os");
    let msvc_dir = vendor_root.join("msvc");

    // Sources to compile. List taken from libusb 1.0.29's
    // `Makefile.am` (`WIN32_USB_SRC` block + the platform-agnostic
    // core); identical to 1.0.27 / 1.0.28 — no new files added between
    // those tags. Keep alphabetised within each group for diff stability.
    let common = [
        "core.c",
        "descriptor.c",
        "hotplug.c",
        "io.c",
        "strerror.c",
        "sync.c",
    ];
    let windows = [
        "events_windows.c",
        "threads_windows.c",
        "windows_common.c",
        "windows_usbdk.c",
        "windows_winusb.c",
    ];

    let mut build = cc::Build::new();
    build
        .include(&libusb_dir)
        .include(&os_dir)
        // msvc/config.h ships pre-baked for cl.exe; libusb's sources do
        // `#include "config.h"` and rely on this directory being on the
        // include path before any other config.h.
        .include(&msvc_dir)
        .define("DEFAULT_VISIBILITY", Some(""))
        .define("PLATFORM_WINDOWS", Some("1"))
        .define("HAVE_CONFIG_H", Some("1"))
        .define("_CRT_SECURE_NO_WARNINGS", Some("1"))
        // Silence the avalanche of MS-specific warnings; not actionable
        // from our side and they hide real problems if they ever surface.
        .warnings(false)
        .flag_if_supported("/wd4100") // unreferenced formal parameter
        .flag_if_supported("/wd4267") // size_t -> int conversion
        .flag_if_supported("/wd4244"); // narrowing conversion

    for f in common {
        build.file(libusb_dir.join(f));
    }
    for f in windows {
        build.file(os_dir.join(f));
    }
    build.compile("usb-1.0");

    // libusb on Windows uses these system libs. cc-rs's `compile`
    // already emits `cargo:rustc-link-lib=static=usb-1.0`; the Win32
    // imports below are needed by the resulting archive at link time.
    for sys in ["user32", "advapi32", "setupapi", "ole32"] {
        println!("cargo:rustc-link-lib={sys}");
    }

    // Where cc-rs landed the archive.
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    // cc-rs writes the static lib as `usb-1.0.lib` on MSVC, `libusb-1.0.a`
    // on MinGW. Surface a normalised absolute path either way.
    let target_env = env::var("CARGO_CFG_TARGET_ENV").unwrap_or_default();
    let archive_name = if target_env == "msvc" {
        "usb-1.0.lib"
    } else {
        "libusb-1.0.a"
    };
    let archive = out_dir.join(archive_name);

    println!("cargo:include={}", libusb_dir.display());
    println!("cargo:lib={}", archive.display());
    // MSVC needs the `lib` prefix in the link directive; ld auto-prepends.
    println!(
        "cargo:lib_name={}",
        if target_env == "msvc" {
            "libusb-1.0"
        } else {
            "usb-1.0"
        }
    );
}

fn probe_unix_system() {
    let lib = pkg_config::Config::new()
        .atleast_version("1.0.20")
        .probe("libusb-1.0")
        .unwrap_or_else(|err| {
            panic!(
                "libusb-1.0 not found via pkg-config: {err}\n\
                 install libusb-1.0-0-dev (Linux) or `brew install libusb` (macOS)"
            )
        });

    if let Some(p) = lib.include_paths.first() {
        println!("cargo:include={}", p.display());
    }

    // libfreenect[2]'s bundled FindLibUSB module sets `LIBUSB_1_LIBRARIES`
    // and uses it via `target_link_libraries(... ${LIBUSB_1_LIBRARIES})`.
    // CMake accepts both file paths and bare library names there. We
    // *always* emit a `cargo:lib` value so the freenect-sys /
    // freenect2-sys build scripts can `env::var("DEP_USB_1_0_LIB")`
    // unconditionally — picking, in priority order:
    //
    //   1. `<libdir>/libusb-1.0.{so,dylib}` if pkg-config returned a
    //      `link_paths` entry. Most reliable; cmake validates as a file.
    //   2. A scan of common system paths (`/usr/lib`, `/usr/lib/<triple>`,
    //      `/usr/local/lib`, `/opt/homebrew/lib`) for `libusb-1.0.*`
    //      when pkg-config didn't surface a libdir (typical for system
    //      libs in the default search path).
    //   3. The bare lib name `usb-1.0` as a last-resort sentinel — cmake
    //      will treat it as a transitive `-lusb-1.0` directive, which
    //      is exactly what we want for the static archive build path.
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let suffix = if target_os == "macos" { "dylib" } else { "so" };
    let lib_name = format!("libusb-1.0.{suffix}");

    let path = lib
        .link_paths
        .iter()
        .map(|libdir| libdir.join(&lib_name))
        .find(|p| p.exists())
        .or_else(|| {
            // pkg-config didn't give us a libdir (or the file isn't
            // there) — scan the usual suspects.
            let mut candidates: Vec<PathBuf> = vec![
                PathBuf::from("/usr/lib").join(&lib_name),
                PathBuf::from("/usr/local/lib").join(&lib_name),
                PathBuf::from("/opt/homebrew/lib").join(&lib_name),
            ];
            if target_os == "linux" {
                let triple = env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
                let multiarch = match triple.as_str() {
                    "x86_64" => "x86_64-linux-gnu",
                    "aarch64" => "aarch64-linux-gnu",
                    _ => "",
                };
                if !multiarch.is_empty() {
                    candidates.push(PathBuf::from("/usr/lib").join(multiarch).join(&lib_name));
                }
            }
            candidates.into_iter().find(|p| p.exists())
        });

    match path {
        Some(p) => println!("cargo:lib={}", p.display()),
        None => {
            // Fall back to the bare name. cmake's target_link_libraries
            // turns this into `-lusb-1.0` which the linker resolves
            // through its own search path. Less informative for cmake's
            // file-existence checks, but unblocks the build.
            println!("cargo:lib=usb-1.0");
        }
    }
    println!("cargo:lib_name=usb-1.0");
}
