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
//!
//! **Crash safety is the whole point.** VPX can crash, be killed, or abort —
//! and this plugin ships `panic = "abort"`, so a panic in our own code takes
//! the process down with no destructors run at all. None of that may leave a
//! Kinect claimed until reboot, so the release path is the kernel's, not
//! ours: no cleanup code exists to be skipped. `a_holder_killed_without_
//! cleanup_still_frees_the_device` kills a real holder with SIGKILL and
//! proves it, rather than trusting the claim.
//!
//! One caveat, measured rather than assumed: `flock` belongs to the open file
//! description, and `fork` duplicates it. A child spawned while we hold a lock
//! carries a copy of the descriptor from the `fork` until its `execve`, since
//! CLOEXEC only fires at `exec`. Inside that window a release does not take
//! effect. It is microseconds wide and closes itself, and nothing re-claims a
//! sensor that fast — but it is why the tests here take turns.

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

    /// These two tests must not overlap, and the reason is worth knowing.
    ///
    /// `flock` belongs to the open file *description*, and `fork` duplicates
    /// it. Between the `fork` and the `execve` of a spawned child, that child
    /// holds a copy of every descriptor the parent had open — CLOEXEC only
    /// takes effect at `exec`. So while the kill-test is spawning its holder,
    /// a lock another test drops stays held for the width of that window, and
    /// the drop test sees a release that has not happened yet.
    ///
    /// It is microseconds and it resolves itself, which is why production is
    /// not affected: nothing re-takes a sensor lock that fast. It is still
    /// real, so the tests take turns rather than the assertion being softened
    /// to hide it.
    static ONE_AT_A_TIME: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// flock/LockFileEx are per file *handle*, not per process — a second
    /// `acquire` from the same process contends exactly like another
    /// process would, which makes the whole exclusion testable in-process.
    #[test]
    fn second_acquire_fails_until_first_drops() {
        let _serialised = ONE_AT_A_TIME.lock();
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

    /// The property the whole design rests on: a holder that dies **without
    /// running any cleanup** still frees the device.
    ///
    /// VPX can crash, be killed, or abort — and this plugin is built with
    /// `panic = "abort"`, so a panic in our own code takes the process down
    /// with no destructors either. None of that may leave a Kinect wedged
    /// until reboot. The lock is a `flock`/`LockFileEx` on an open handle
    /// precisely so the kernel releases it when the process table entry goes,
    /// but "the OS will do it" is a claim, and this test is the evidence.
    ///
    /// The child is this same test binary, re-run to hold the lock and block;
    /// `Child::kill` is SIGKILL on Unix and TerminateProcess on Windows —
    /// uncatchable on both, which is exactly the scenario.
    #[test]
    fn a_holder_killed_without_cleanup_still_frees_the_device() {
        let _serialised = ONE_AT_A_TIME.lock();
        let dev = format!("crash-{}", std::process::id());
        let ready = std::env::temp_dir().join(format!("headtracking-{dev}.held"));
        let _ = std::fs::remove_file(&ready);

        let exe = std::env::current_exe().expect("test binary path");
        let mut child = std::process::Command::new(exe)
            .args([
                "--exact",
                "hwlock::tests::holds_the_lock_for_the_kill_test",
                "--ignored",
            ])
            .env("HT_HWLOCK_HOLD", &dev)
            .spawn()
            .expect("spawn lock holder");

        // Wait for the child to actually hold it. Polling `acquire` here
        // would take the lock ourselves and defeat the test, so the child
        // reports through a marker file instead.
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        while !ready.exists() {
            assert!(
                std::time::Instant::now() < deadline,
                "child never took the lock"
            );
            std::thread::sleep(std::time::Duration::from_millis(20));
        }
        // Held by someone else: the exclusion is real before we kill anything.
        HwLock::acquire(&dev).expect_err("child should hold the lock");

        child.kill().expect("kill the holder");
        child.wait().expect("reap the holder");

        // The kernel releases on process teardown, but not necessarily before
        // `wait` returns on every platform — give it a moment rather than
        // racing it.
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        let freed = loop {
            if HwLock::acquire(&dev).is_ok() {
                break true;
            }
            if std::time::Instant::now() >= deadline {
                break false;
            }
            std::thread::sleep(std::time::Duration::from_millis(20));
        };
        let _ = std::fs::remove_file(&ready);
        assert!(
            freed,
            "a killed holder left {dev} locked — a crashed VPX would wedge the sensor"
        );
    }

    /// Not a test: the child half of the one above. Ignored so it never runs
    /// on its own, and inert unless `HT_HWLOCK_HOLD` names a device.
    #[test]
    #[ignore = "child process of a_holder_killed_without_cleanup_still_frees_the_device"]
    fn holds_the_lock_for_the_kill_test() {
        let Ok(dev) = std::env::var("HT_HWLOCK_HOLD") else {
            return;
        };
        let _lock = HwLock::acquire(&dev).expect("child acquire");
        std::fs::write(
            std::env::temp_dir().join(format!("headtracking-{dev}.held")),
            "held",
        )
        .expect("signal readiness");
        // Block until killed. Sleeping beats spinning; the parent never lets
        // this run to completion.
        std::thread::sleep(std::time::Duration::from_secs(120));
    }
}
