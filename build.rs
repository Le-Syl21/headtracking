use std::env;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Developer-local checkout of the upstream `vpinball` repo. Preferred when
/// present so we always bind against the headers a Sylvain-style dev
/// environment is editing live.
const DEV_PLUGINS_DIR: &str = "../vpinball/plugins/plugins";
/// Vendored copy of the same headers shipped in-tree under
/// `third_party/vpx-plugin-headers/`. Used by CI / package builds where the
/// vpinball source tree isn't available; refreshed by hand when upstream
/// changes the API (see that directory's README).
const VENDORED_PLUGINS_DIR: &str = "third_party/vpx-plugin-headers";

fn main() {
    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=VPX_PLUGINS_DIR");

    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let plugins_dir = env::var("VPX_PLUGINS_DIR")
        .map(PathBuf::from)
        .ok()
        .or_else(|| {
            let dev = root.join(DEV_PLUGINS_DIR);
            dev.is_dir().then_some(dev)
        })
        .or_else(|| {
            let vendored = root.join(VENDORED_PLUGINS_DIR);
            vendored.is_dir().then_some(vendored)
        })
        .unwrap_or_else(|| {
            panic!(
                "VPX plugin headers directory not found. Tried VPX_PLUGINS_DIR, \
                 then {}, then {}. Set VPX_PLUGINS_DIR to override.",
                DEV_PLUGINS_DIR, VENDORED_PLUGINS_DIR
            )
        });

    if !plugins_dir.is_dir() {
        panic!(
            "VPX plugin headers directory not found: {} \
             (set VPX_PLUGINS_DIR to override)",
            plugins_dir.display()
        );
    }

    let mut builder = bindgen::Builder::default()
        .header("wrapper.h")
        .clang_arg(format!("-I{}", plugins_dir.display()))
        .clang_arg("-xc++")
        .clang_arg("-std=c++17");

    // libclang ships compiler-internal headers (stddef.h, stdarg.h, …) in its
    // resource directory; on Debian/Ubuntu these only land when the
    // `libclang-common-*-dev` package (or `clang`) is installed. Without them
    // any system include that pulls in `stddef.h` fails to parse. Locate the
    // most appropriate include directory and fall back to GCC's if needed.
    if let Some(dir) = compiler_resource_include_dir() {
        builder = builder
            .clang_arg("-isystem")
            .clang_arg(dir.to_string_lossy().into_owned());
    }

    let bindings = builder
        // Only translate the VPX API we use; let bindgen prune the rest.
        .allowlist_type("MsgPluginAPI")
        .allowlist_type("MsgEndpointInfo")
        .allowlist_type("MsgSettingDef")
        .allowlist_type("VPXPluginAPI")
        .allowlist_type("VPXViewSetupDef")
        .allowlist_type("VPXInfo")
        .allowlist_type("VPXTableInfo")
        .allowlist_type("VPXTexture.*")
        .allowlist_type("VPXAction")
        .allowlist_type("VPXAuxiliaryRenderer")
        .allowlist_type("VPXNudgeState")
        .allowlist_type("VPXPlungerState")
        .allowlist_type("LoggingPluginAPI")
        .allowlist_type("msgpi_.*")
        .allowlist_var("MSGPI_.*")
        .allowlist_var("VPXPI_.*")
        .allowlist_var("LOGPI_.*")
        // String macros come through as C strings; keep them as &CStr-friendly.
        .generate_cstr(true)
        .derive_default(true)
        .derive_debug(true)
        .layout_tests(false)
        .generate()
        .expect("bindgen failed for VPX plugin headers");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap()).join("vpx_bindings.rs");
    bindings
        .write_to_file(&out_path)
        .expect("failed to write VPX bindings");
}

/// Find a directory containing `stddef.h` that we can hand to clang via
/// `-isystem`. Tries (in order):
///   1. `clang -print-resource-dir` if a clang binary is on PATH
///   2. Common Debian/Ubuntu locations for libclang's resource dir
///   3. GCC's installed multilib include dir (works for our pure-C headers)
///
/// Returns `None` if nothing usable is found, in which case bindgen will
/// surface its own error pointing at the missing header.
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
        let p = Path::new(path);
        if p.join("stddef.h").is_file() {
            return Some(p.to_path_buf());
        }
    }

    // Last resort: GCC's headers. Pick the highest-numbered version available.
    let gcc_root = Path::new("/usr/lib/gcc/x86_64-linux-gnu");
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
