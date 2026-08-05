//! Cross-process hardware locks.
//!
//! The plugin, the interactive demo and the demo's headless capture modes
//! can all try to open the same sensor; libfreenect/libfreenect2/SDL fail
//! late and confusingly when the device is already streamed by another
//! process. This module puts a cooperative advisory lock in front of every
//! device open so the second claimant fails fast with a readable message.
//!
//! Mechanics: one 0-byte file per device slug in the system temp dir,
//! locked exclusively with [`std::fs::File::try_lock`] (flock on Unix,
//! `LockFileEx` on Windows). The OS drops the lock when the file handle
//! closes — including on crash or SIGKILL — so a dead process never wedges
//! the hardware. The holder writes its PID into the file purely for the
//! loser's error message. Lock files are never unlinked: removing them
//! would race a concurrent claimant onto a different inode, silently
//! disabling the exclusion.
//!
//! Same-user assumption: all headtracking processes run as the cab user.
//! (A lock file created by a *different* user could be unopenable, but
//! that's not a supported deployment.)

use std::fs::{File, OpenOptions, TryLockError};
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::PathBuf;

/// Held for as long as the device is open — keep it next to the device
/// handle and let it drop with it. Releasing is automatic (handle close).
#[derive(Debug)]
pub struct HwLock {
    _file: File,
}

fn lock_path(device: &str) -> PathBuf {
    std::env::temp_dir().join(format!("headtracking-{device}.lock"))
}

impl HwLock {
    /// Take the exclusive cross-process lock for `device` — a stable slug
    /// like `"kinect-v1"`, `"kinect-v2"` or `"webcam"`. Call it *before*
    /// touching the hardware. Errors are user-facing strings naming the
    /// holder when another process already streams the device.
    pub fn acquire(device: &str) -> Result<Self, String> {
        let path = lock_path(device);
        let mut file = OpenOptions::new()
            .create(true)
            .read(true)
            .write(true)
            .truncate(false)
            .open(&path)
            .map_err(|e| format!("{device}: lock file {} unusable: {e}", path.display()))?;
        match file.try_lock() {
            Ok(()) => {}
            Err(TryLockError::WouldBlock) => {
                let mut holder = String::new();
                let _ = file.read_to_string(&mut holder);
                let holder = holder.trim();
                let holder = if holder.is_empty() {
                    "another process".to_string()
                } else {
                    holder.to_string()
                };
                return Err(format!(
                    "{device} is already in use by another headtracking process ({holder})"
                ));
            }
            Err(TryLockError::Error(e)) => {
                return Err(format!("{device}: hardware lock failed: {e}"));
            }
        }
        // Best-effort holder tag for the message above — the lock itself is
        // the file handle, not this content.
        let tag = format!("pid {}", std::process::id());
        let _ = file.set_len(0);
        let _ = file.seek(SeekFrom::Start(0));
        let _ = file.write_all(tag.as_bytes());
        let _ = file.flush();
        tracing::debug!(device, path = %path.display(), "hardware lock acquired");
        Ok(Self { _file: file })
    }
}

#[cfg(test)]
mod tests {
    use super::HwLock;

    /// flock/LockFileEx are per file *handle*, not per process — a second
    /// `acquire` from the same process contends exactly like another
    /// process would, which makes the whole exclusion testable in-process.
    #[test]
    fn second_acquire_fails_until_first_drops() {
        let dev = format!("test-{}", std::process::id());
        let first = HwLock::acquire(&dev).expect("first acquire");
        let second = HwLock::acquire(&dev);
        let msg = second.expect_err("second acquire must fail");
        assert!(msg.contains("already in use"), "unexpected message: {msg}");
        assert!(
            msg.contains(&format!("pid {}", std::process::id())),
            "holder pid missing: {msg}"
        );
        drop(first);
        HwLock::acquire(&dev).expect("re-acquire after drop");
    }
}
