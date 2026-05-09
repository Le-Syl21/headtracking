//! Build-only facade: this crate's only job is to produce the right
//! libusb-1.0 archive (vendored static build on Windows, system probe
//! on Linux/macOS) and expose its location to downstream sys crates
//! via cargo metadata. There's no Rust API — `freenect-sys` and
//! `freenect2-sys` read `DEP_USB_1_0_INCLUDE` / `DEP_USB_1_0_LIB` /
//! `DEP_USB_1_0_LIB_NAME` in their own build scripts.
