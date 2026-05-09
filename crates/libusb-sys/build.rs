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
    match target_os.as_str() {
        "windows" => build_windows_static(&vendor_root),
        "macos" => build_macos_static(&vendor_root),
        "linux" => build_linux_static(&vendor_root),
        _ => probe_unix_system(),
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
    // OUT_DIR exposed separately so downstream crates can emit
    // `cargo:rustc-link-search` even when the libusb-sys rlib gets
    // elided by rustc (its lib.rs has no public symbols, so without a
    // re-emit the link directives we declare via `cargo:` would
    // disappear from the final cdylib's link line — see the
    // belt-and-suspenders block in freenect-sys/freenect2-sys).
    println!("cargo:lib_dir={}", out_dir.display());
    // Bare lib name for `cargo:rustc-link-lib=…` in downstream crates.
    // cc-rs / cargo-rustc translation:
    //   - on MSVC, `static=usb-1.0` → `usb-1.0.lib`
    //   - elsewhere static archives are `libusb-1.0.a` / `.so` /
    //     `.dylib`; the bare name `usb-1.0` is what ld expects.
    println!("cargo:lib_name=usb-1.0");
    // Static vs dylib: the Windows path produces a static archive via
    // cc-rs; consumers should pass `kind=static` to rustc-link-lib.
    println!("cargo:link_kind=static");
}

fn build_macos_static(vendor_root: &Path) {
    let libusb_dir = vendor_root.join("libusb");
    let os_dir = libusb_dir.join("os");
    // Upstream ships a hand-tuned Xcode/config.h that already covers
    // every HAVE_* / PLATFORM_POSIX define libusb's sources expect on
    // macOS — including the version-gated `HAVE_PTHREAD_THREADID_NP`
    // which uses MAC_OS_X_VERSION_MIN_REQUIRED. We add the directory
    // to the include path verbatim, no patching needed.
    let xcode_dir = vendor_root.join("Xcode");

    let common = [
        "core.c",
        "descriptor.c",
        "hotplug.c",
        "io.c",
        "strerror.c",
        "sync.c",
    ];
    let darwin = ["darwin_usb.c", "events_posix.c", "threads_posix.c"];

    let mut build = cc::Build::new();
    build
        .include(&libusb_dir)
        .include(&os_dir)
        .include(&xcode_dir) // libusb's hand-baked macOS config.h
        .define("HAVE_CONFIG_H", Some("1"))
        // Match what an autotools `./configure` would emit for
        // visibility — keeps the static archive's exported-symbol
        // surface clean even though cc-rs writes a `.a`.
        .warnings(false);

    for f in common {
        build.file(libusb_dir.join(f));
    }
    for f in darwin {
        build.file(os_dir.join(f));
    }
    build.compile("usb-1.0");

    // Apple system frameworks libusb's darwin backend pulls in.
    // `Security` is required since libusb 1.0.27+ (entitlement check
    // for the new sandbox-friendly USB device access path).
    for fw in ["IOKit", "CoreFoundation", "Security"] {
        println!("cargo:rustc-link-lib=framework={fw}");
    }
    // `objc` is needed by the IOKit USB notification path.
    println!("cargo:rustc-link-lib=objc");

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let archive = out_dir.join("libusb-1.0.a");

    println!("cargo:include={}", libusb_dir.display());
    println!("cargo:lib={}", archive.display());
    println!("cargo:lib_dir={}", out_dir.display());
    println!("cargo:lib_name=usb-1.0");
    println!("cargo:link_kind=static");
}

fn build_linux_static(vendor_root: &Path) {
    let libusb_dir = vendor_root.join("libusb");
    let os_dir = libusb_dir.join("os");

    // Hand-written config.h. libusb ships an `android/config.h` that
    // covers most of what we want, but it pulls in
    // `USE_SYSTEM_LOGGING_FACILITY` which routes through `liblog`
    // (Android's `__android_log_*`) — we want the default stderr fall-
    // back instead. Generate our own at build time so the include
    // ordering is unambiguous (vs adding android/ to the path and
    // hoping no header collision).
    //
    // Hot-plug path: NETLINK (`linux_netlink.c`) rather than libudev,
    // so we don't pull in `libudev.so` at runtime — keeps the "zero
    // user dep" promise for distros where that lib isn't preinstalled
    // (rare, but eliminates the ambiguity).
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let config_dir = out_dir.join("config_include");
    std::fs::create_dir_all(&config_dir).expect("create config_include dir");
    let config_h = r#"/* libusb config.h — generated by libusb-sys/build.rs for Linux desktop. */
#define DEFAULT_VISIBILITY __attribute__ ((visibility ("default")))
#define ENABLE_LOGGING 1
#define HAVE_ASM_TYPES_H 1
#define HAVE_CLOCK_GETTIME 1
#define HAVE_DECL_EFD_CLOEXEC 1
#define HAVE_DECL_EFD_NONBLOCK 1
#define HAVE_DECL_TFD_CLOEXEC 1
#define HAVE_DECL_TFD_NONBLOCK 1
#define HAVE_EVENTFD 1
#define HAVE_NFDS_T 1
#define HAVE_PIPE2 1
#define HAVE_PTHREAD_SETNAME_NP 1
#define HAVE_STRUCT_TIMESPEC 1
#define HAVE_SYS_TIME_H 1
#define HAVE_TIMERFD 1
#define HAVE_SYSLOG 1
#define PLATFORM_POSIX 1
#define PRINTF_FORMAT(a, b) __attribute__ ((__format__ (__printf__, a, b)))
#define _GNU_SOURCE 1
"#;
    std::fs::write(config_dir.join("config.h"), config_h).expect("write generated config.h");

    let common = [
        "core.c",
        "descriptor.c",
        "hotplug.c",
        "io.c",
        "strerror.c",
        "sync.c",
    ];
    let linux = [
        "linux_usbfs.c",
        "linux_netlink.c",
        "events_posix.c",
        "threads_posix.c",
    ];

    let mut build = cc::Build::new();
    build
        .include(&libusb_dir)
        .include(&os_dir)
        // Generated config.h directory comes first so libusb's
        // `#include "config.h"` resolves against ours.
        .include(&config_dir)
        .define("HAVE_CONFIG_H", Some("1"))
        // libusb's source casts away const in a few spots that GCC
        // 14+ flags as warnings. Suppress to keep the build quiet —
        // not actionable from our side.
        .warnings(false)
        .flag_if_supported("-Wno-unused-parameter")
        .flag_if_supported("-Wno-pointer-arith");

    for f in common {
        build.file(libusb_dir.join(f));
    }
    for f in linux {
        build.file(os_dir.join(f));
    }
    build.compile("usb-1.0");

    // pthread is needed by `threads_posix.c`. glibc auto-links it
    // historically, but on musl / older toolchains the explicit
    // directive avoids surprises.
    println!("cargo:rustc-link-lib=pthread");

    let archive = out_dir.join("libusb-1.0.a");

    println!("cargo:include={}", libusb_dir.display());
    println!("cargo:lib={}", archive.display());
    println!("cargo:lib_dir={}", out_dir.display());
    println!("cargo:lib_name=usb-1.0");
    println!("cargo:link_kind=static");
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

    match &path {
        Some(p) => println!("cargo:lib={}", p.display()),
        None => {
            // Fall back to the bare name. cmake's target_link_libraries
            // turns this into `-lusb-1.0` which the linker resolves
            // through its own search path. Less informative for cmake's
            // file-existence checks, but unblocks the build.
            println!("cargo:lib=usb-1.0");
        }
    }
    if let Some(p) = path.as_ref().and_then(|p| p.parent()) {
        println!("cargo:lib_dir={}", p.display());
    }
    println!("cargo:lib_name=usb-1.0");
    println!("cargo:link_kind=dylib");
}
