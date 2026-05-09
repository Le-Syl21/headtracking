//! Cheap one-shot probe for the UsbDk filter driver on Windows.
//!
//! libfreenect / libfreenect2 only manage to open the Kinect on Windows
//! when one of these is true:
//!   * the device's driver is libusbK (after Zadig replacement), or
//!   * UsbDk's filter driver is installed and libusb has been told to use
//!     its backend (which both libs now do at `libusb_init` time).
//!
//! This module checks whether UsbDk is even loaded, so we can emit one
//! actionable log line at PluginLoad pointing the user at the installer
//! instead of letting them stare at a later `LIBUSB_ERROR_NOT_SUPPORTED`.
//! The probe is best-effort; a false-negative just means a redundant hint.

/// Public download URL for the UsbDk releases page. Surfaced both in
/// the warn log and in the headtracking-demo popup so users have a
/// single source of truth they can click on.
pub const RELEASES_URL: &str = "https://github.com/daynix/UsbDk/releases";

/// `true` if UsbDk is loaded and reachable. Always `true` on
/// non-Windows targets — UsbDk only matters when libusb's WinUSB
/// backend would otherwise fail. Cheap (one `CreateFileW` call); safe
/// to invoke from any thread.
pub fn is_present() -> bool {
    #[cfg(target_os = "windows")]
    {
        return windows::is_usbdk_present();
    }
    #[cfg(not(target_os = "windows"))]
    {
        true
    }
}

/// Log one actionable line if UsbDk is missing on Windows. No-op on
/// other platforms (including the "info: detected" line, which would
/// just be noise).
pub fn warn_if_missing() {
    #[cfg(target_os = "windows")]
    {
        if is_present() {
            tracing::info!("UsbDk filter driver detected");
        } else {
            tracing::warn!(
                releases = RELEASES_URL,
                "UsbDk filter driver not detected — Kinect open will fail with \
                 LIBUSB_ERROR_NOT_SUPPORTED. Install UsbDk from the link above \
                 and restart VPX."
            );
        }
    }
}

#[cfg(target_os = "windows")]
mod windows {
    use std::ffi::c_void;

    // Minimal Win32 surface — opening `\\.\UsbDk` for existence check
    // only. Avoids pulling in `windows-sys` for three symbols.
    #[link(name = "kernel32")]
    unsafe extern "system" {
        fn CreateFileW(
            file_name: *const u16,
            desired_access: u32,
            share_mode: u32,
            security_attributes: *mut c_void,
            creation_disposition: u32,
            flags_and_attributes: u32,
            template_file: *mut c_void,
        ) -> *mut c_void;
        fn CloseHandle(handle: *mut c_void) -> i32;
        fn GetLastError() -> u32;
    }

    const INVALID_HANDLE_VALUE: *mut c_void = !0_usize as *mut c_void;
    const FILE_SHARE_READ: u32 = 0x0000_0001;
    const FILE_SHARE_WRITE: u32 = 0x0000_0002;
    const OPEN_EXISTING: u32 = 3;
    const ERROR_ACCESS_DENIED: u32 = 5;

    pub fn is_usbdk_present() -> bool {
        // UTF-16 NUL-terminated `\\.\UsbDk`. UsbDk's user-mode helper
        // creates this device symlink at service start; if the open
        // succeeds (or fails specifically with ACCESS_DENIED, meaning
        // it's there but we lack rights), the driver is loaded.
        let path: [u16; 9] = [
            b'\\' as u16,
            b'\\' as u16,
            b'.' as u16,
            b'\\' as u16,
            b'U' as u16,
            b's' as u16,
            b'b' as u16,
            b'D' as u16,
            b'k' as u16,
        ];
        let mut nul_terminated = [0u16; 10];
        nul_terminated[..9].copy_from_slice(&path);

        // SAFETY: pointer is to a stack-local buffer with a trailing NUL,
        // every other arg is a documented Win32 sentinel. Returned handle
        // is closed below if valid.
        let h = unsafe {
            CreateFileW(
                nul_terminated.as_ptr(),
                0,
                FILE_SHARE_READ | FILE_SHARE_WRITE,
                std::ptr::null_mut(),
                OPEN_EXISTING,
                0,
                std::ptr::null_mut(),
            )
        };
        if h != INVALID_HANDLE_VALUE {
            // SAFETY: handle came from CreateFileW above.
            unsafe { CloseHandle(h) };
            return true;
        }
        // SAFETY: GetLastError reads thread-local error state, no preconditions.
        let err = unsafe { GetLastError() };
        // ACCESS_DENIED = symlink exists, we just can't open it without
        // privileges — still a positive signal that the driver is loaded.
        err == ERROR_ACCESS_DENIED
    }
}
