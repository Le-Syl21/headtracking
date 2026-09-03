//! Optional "Share a capture" contribution uploader.
//!
//! Saves are done by the caller; this module just ships a file to a **write-
//! only** Nextcloud "file drop" off the UI thread. The drop only allows `PUT`
//! (no list / read / download — verified against the server), so the token
//! below is safe to ship in a public binary: leaking it lets someone *add*
//! files, never read anyone else's. Only the maintainer, authenticated on the
//! server, can see the uploads.
//!
//! Everything here is built around one field report: a contributor sent 35
//! files, saw no error, and not one of them ever reached the drop. So the
//! rules are:
//!
//! * **Ask before capturing.** [`probe`] proves the drop is reachable from
//!   *this* machine before the UI offers to upload anything.
//! * **Never lose a capture.** A file that fails every retry is written to a
//!   rescue folder, so there is always something left to hand over by other
//!   means.
//! * **Fail loudly.** The counts below drive a red panel, not a quiet log line.

use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::OnceLock;
use std::thread::JoinHandle;
use std::time::Duration;

use parking_lot::{Condvar, Mutex};
use tracing::{info, warn};

/// Public write-only share token (the tail of the share URL).
const DROP_TOKEN: &str = "cnYQZtzPHQSpCiW";
const DROP_HOST: &str = "https://nextcloud.syl21.org";
const UPLOAD_RETRIES: u32 = 3;

/// Where a contributor can hand over captures the drop refused.
pub const DISCORD_INVITE: &str = "https://discord.gg/cFcNrt9AY";

/// Two timeouts, because "nothing is moving" and "this is slow" are different
/// questions and one number cannot answer both.
///
/// `CONNECT_TIMEOUT` covers DNS + TCP + the TLS handshake — the phase where a
/// captive portal, a blocked port or a TLS-intercepting antivirus shows up. If
/// the connection isn't established by then, nothing is going to move at all,
/// and a pincab should hear about it quickly.
///
/// `TRANSFER_TIMEOUT` covers the body once the connection is up. A capture set
/// is a few MB, the server sits behind a 2 Gbit/s LACP pair, and a domestic
/// upstream link is the narrow part: a transfer that is *working* may well
/// need minutes. Cutting that at 30 s (as we used to, with a single global
/// timeout) turned slow-but-fine into failure.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(30);
const TRANSFER_TIMEOUT: Duration = Duration::from_secs(300);
/// The reachability probe answers in seconds or not at all — it must never
/// make the UI feel hung.
const PROBE_TIMEOUT: Duration = Duration::from_secs(10);

fn drop_url(name: &str) -> String {
    format!("{DROP_HOST}/public.php/dav/files/{DROP_TOKEN}/{name}")
}

/// `Basic base64("<token>:")` — the share authenticates with the token as the
/// username and an empty password.
fn basic_auth() -> String {
    format!("Basic {}", base64_std(format!("{DROP_TOKEN}:").as_bytes()))
}

/// Minimal standard-alphabet base64 (used only for the fixed auth string, so
/// no need for a dependency).
fn base64_std(input: &[u8]) -> String {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(input.len().div_ceil(3) * 4);
    for chunk in input.chunks(3) {
        let b0 = chunk[0];
        let b1 = chunk.get(1).copied().unwrap_or(0);
        let b2 = chunk.get(2).copied().unwrap_or(0);
        let n = (u32::from(b0) << 16) | (u32::from(b1) << 8) | u32::from(b2);
        out.push(T[(n >> 18 & 63) as usize] as char);
        out.push(T[(n >> 12 & 63) as usize] as char);
        out.push(if chunk.len() > 1 {
            T[(n >> 6 & 63) as usize] as char
        } else {
            '='
        });
        out.push(if chunk.len() > 2 {
            T[(n & 63) as usize] as char
        } else {
            '='
        });
    }
    out
}

/// One agent for the whole process: 35 files over one share is 35 TLS
/// handshakes if every PUT builds its own, which on a slow link is most of the
/// cost. Connections are pooled and reused instead.
fn agent() -> &'static ureq::Agent {
    static AGENT: OnceLock<ureq::Agent> = OnceLock::new();
    AGENT.get_or_init(|| {
        ureq::Agent::config_builder()
            .timeout_connect(Some(CONNECT_TIMEOUT))
            .timeout_send_request(Some(CONNECT_TIMEOUT))
            .timeout_send_body(Some(TRANSFER_TIMEOUT))
            .timeout_recv_response(Some(CONNECT_TIMEOUT))
            // Deliberately no global timeout: it would cap the body transfer
            // at whatever the connect budget is, which is the bug this split
            // exists to fix.
            .build()
            .new_agent()
    })
}

/// How long an unattended run may wait for a whole batch of `count` files.
///
/// Derived from the per-file budget rather than a flat number, so a slow link
/// and a big capture set don't get cut off halfway: that is exactly how a
/// contribution ends up as a log file with no images next to it.
#[must_use]
pub fn batch_budget(count: usize) -> Duration {
    let files = u32::try_from(count).unwrap_or(u32::MAX).clamp(1, 64);
    CONNECT_TIMEOUT + TRANSFER_TIMEOUT * files
}

/// What a reachability probe found.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Reach {
    /// The drop answered as a write-only share should. Uploads may proceed.
    Up,
    /// A server answered, but not the way a live share does — gone, expired,
    /// revoked, or plainly broken. Nothing the contributor can fix.
    ServerSaysNo(u16),
    /// We never got an answer: DNS, routing, firewall, proxy, TLS.
    Unreachable(String),
}

impl Reach {
    #[must_use]
    pub fn is_up(&self) -> bool {
        matches!(self, Self::Up)
    }

    /// One line, in the contributor's terms, for the panel.
    #[must_use]
    pub fn explain(&self) -> String {
        match self {
            Self::Up => "the capture server is reachable".into(),
            Self::ServerSaysNo(code) => format!(
                "the server answered HTTP {code}: the problem is on our side, not yours — \
                 please tell us on Discord"
            ),
            Self::Unreachable(why) => format!(
                "this machine cannot reach {DROP_HOST} ({why}) — a firewall, a proxy or an \
                 antivirus intercepting HTTPS is the usual cause"
            ),
        }
    }
}

/// Ask the drop whether it is there, without uploading anything.
///
/// A write-only share answers `HEAD` with **405 Method Not Allowed** — it
/// refuses to be read, which is exactly the point. So 405 is the healthy
/// answer here, and 404/410 is the one that means the share itself is gone.
/// Anything that isn't an HTTP answer at all is a network problem on this
/// side, which is what we most want to catch before a contributor spends five
/// minutes capturing.
#[must_use]
pub fn probe() -> Reach {
    let probe_agent = ureq::Agent::config_builder()
        .timeout_global(Some(PROBE_TIMEOUT))
        .build()
        .new_agent();
    let url = format!("{DROP_HOST}/public.php/dav/files/{DROP_TOKEN}/");
    match probe_agent
        .head(&url)
        .header("Authorization", &basic_auth())
        .call()
    {
        // 2xx would be a readable share; unexpected, but reachable.
        Ok(_) => Reach::Up,
        Err(ureq::Error::StatusCode(code)) => classify(code),
        Err(e) => Reach::Unreachable(e.to_string()),
    }
}

/// Read a `HEAD` status the way a write-only share means it.
///
/// Kept separate from [`probe`] so the part that decides whether a contributor
/// may upload can be tested without a network: getting this backwards either
/// blocks a healthy cabinet or lets a broken one capture into the void.
fn classify(code: u16) -> Reach {
    match code {
        // The share itself is missing — that one is ours to fix.
        404 | 410 => Reach::ServerSaysNo(code),
        // 401/403/405: a Nextcloud answered and declined to be read, which is
        // exactly what a write-only drop is supposed to do.
        400..=499 => Reach::Up,
        // 5xx, and anything else that isn't a refusal to be read.
        _ => Reach::ServerSaysNo(code),
    }
}

/// Snapshot of the uploader's progress, shown in the panel.
///
/// Counts are per *share*, not per session: [`Uploader::begin_batch`] resets
/// them, so the panel always describes the capture in front of the user rather
/// than an accumulated total in which one failure is invisible.
#[derive(Clone, Default)]
pub struct UploadStatus {
    pub pending: usize,
    pub uploaded: usize,
    pub failed: usize,
    pub last_error: Option<String>,
    /// Where files that could not be uploaded were written instead.
    pub rescued_in: Option<PathBuf>,
    /// Set when even the rescue write failed — the only case where a capture
    /// is really lost, and it has to be said out loud.
    pub rescue_error: Option<String>,
}

impl UploadStatus {
    #[must_use]
    pub fn has_failure(&self) -> bool {
        self.failed > 0 || self.last_error.is_some()
    }
}

#[derive(Default)]
struct Queue {
    items: VecDeque<(String, Vec<u8>)>,
    stop: bool,
}

/// Background thread that PUTs queued files to the drop, with a few retries.
pub struct Uploader {
    queue: Arc<(Mutex<Queue>, Condvar)>,
    status: Arc<Mutex<UploadStatus>>,
    rescue: Arc<Mutex<PathBuf>>,
    _handle: JoinHandle<()>,
}

impl Uploader {
    #[must_use]
    pub fn spawn(rescue_dir: PathBuf) -> Self {
        let queue = Arc::new((Mutex::new(Queue::default()), Condvar::new()));
        let status = Arc::new(Mutex::new(UploadStatus::default()));
        let rescue = Arc::new(Mutex::new(rescue_dir));
        let (q, st, rs) = (Arc::clone(&queue), Arc::clone(&status), Arc::clone(&rescue));
        let handle = std::thread::Builder::new()
            .name("uploader".into())
            .spawn(move || uploader_loop(&q, &st, &rs))
            .expect("spawn uploader thread");
        Self {
            queue,
            status,
            rescue,
            _handle: handle,
        }
    }

    /// Point the rescue folder at the one the contributor picked for their own
    /// copy: if an upload fails, the files they can forward are then all in
    /// one place they already know about.
    pub fn set_rescue_dir(&self, dir: PathBuf) {
        *self.rescue.lock() = dir;
    }

    #[must_use]
    pub fn rescue_dir(&self) -> PathBuf {
        self.rescue.lock().clone()
    }

    /// Start a new share: the panel speaks about this capture only.
    pub fn begin_batch(&self) {
        let mut s = self.status.lock();
        s.uploaded = 0;
        s.failed = 0;
        s.last_error = None;
        s.rescued_in = None;
        s.rescue_error = None;
    }

    /// Queue one file (already encoded) for upload under `name`.
    pub fn submit(&self, name: String, bytes: Vec<u8>) {
        self.status.lock().pending += 1;
        let (m, cv) = &*self.queue;
        m.lock().items.push_back((name, bytes));
        cv.notify_one();
    }

    #[must_use]
    pub fn status(&self) -> UploadStatus {
        self.status.lock().clone()
    }

    /// Write whatever is still queued to the rescue folder and stop the
    /// worker. Called when the window closes: the old code dropped the queue
    /// on the floor, which is how a contribution once arrived as its log file
    /// and nothing else.
    ///
    /// Returns how many files were saved this way.
    pub fn rescue_pending(&self) -> usize {
        let leftovers: Vec<(String, Vec<u8>)> = {
            let (m, cv) = &*self.queue;
            let mut g = m.lock();
            g.stop = true;
            cv.notify_all();
            g.items.drain(..).collect()
        };
        if leftovers.is_empty() {
            return 0;
        }
        let dir = self.rescue.lock().clone();
        let mut saved = 0;
        for (name, bytes) in leftovers {
            let mut s = self.status.lock();
            s.pending = s.pending.saturating_sub(1);
            s.failed += 1;
            drop(s);
            if write_rescue(&dir, &name, &bytes, &self.status) {
                saved += 1;
            }
        }
        warn!(saved, dir = %dir.display(), "queued contributions saved instead of uploaded");
        saved
    }
}

impl Drop for Uploader {
    /// Last line of defence: whatever is still queued when the demo shuts down
    /// is written to disk rather than evaporating with the process. A capture
    /// on disk can still be handed over; a capture that only ever existed in a
    /// queue is gone, and nobody learns it was lost.
    fn drop(&mut self) {
        self.rescue_pending();
    }
}

fn uploader_loop(
    queue: &Arc<(Mutex<Queue>, Condvar)>,
    status: &Arc<Mutex<UploadStatus>>,
    rescue: &Arc<Mutex<PathBuf>>,
) {
    let (m, cv) = &**queue;
    loop {
        let next = {
            let mut g = m.lock();
            loop {
                if g.stop {
                    return;
                }
                if let Some(item) = g.items.pop_front() {
                    break item;
                }
                cv.wait(&mut g);
            }
        };
        let (name, bytes) = next;
        let mut result = Ok(());
        for attempt in 1..=UPLOAD_RETRIES {
            match put_file(&name, &bytes) {
                Ok(()) => {
                    result = Ok(());
                    break;
                }
                Err(e) => {
                    warn!(name, attempt, "contribution upload failed: {e}");
                    result = Err(e);
                    if attempt < UPLOAD_RETRIES {
                        std::thread::sleep(Duration::from_secs(u64::from(2 * attempt)));
                    }
                }
            }
        }
        let mut s = status.lock();
        s.pending = s.pending.saturating_sub(1);
        match result {
            Ok(()) => {
                s.uploaded += 1;
                drop(s);
                info!(name, "contribution uploaded");
            }
            Err(e) => {
                s.failed += 1;
                s.last_error = Some(e);
                drop(s);
                // A failed upload must still leave the contributor something
                // to hand over — that is the whole point of the rescue folder.
                let dir = rescue.lock().clone();
                write_rescue(&dir, &name, &bytes, status);
            }
        }
    }
}

/// Write one file the drop refused. Records where it went (or why it could
/// not) in the status, so the panel never has to guess.
fn write_rescue(dir: &Path, name: &str, bytes: &[u8], status: &Arc<Mutex<UploadStatus>>) -> bool {
    if let Err(e) = std::fs::create_dir_all(dir) {
        warn!(dir = %dir.display(), "contribution rescue folder unusable: {e}");
        status.lock().rescue_error = Some(format!("{}: {e}", dir.display()));
        return false;
    }
    match std::fs::write(dir.join(name), bytes) {
        Ok(()) => {
            info!(name, dir = %dir.display(), "contribution kept for manual hand-over");
            status.lock().rescued_in = Some(dir.to_path_buf());
            true
        }
        Err(e) => {
            warn!(name, dir = %dir.display(), "contribution rescue write failed: {e}");
            status.lock().rescue_error = Some(format!("{}: {e}", dir.display()));
            false
        }
    }
}

fn put_file(name: &str, bytes: &[u8]) -> Result<(), String> {
    // ureq 3 moved timeouts onto the agent rather than the request, and no
    // longer turns a non-2xx into `Error::Status`: it is `StatusCode` now, and
    // the code is read off the response.
    match agent()
        .put(&drop_url(name))
        .header("Authorization", &basic_auth())
        .send(bytes)
    {
        Ok(_) => Ok(()),
        Err(ureq::Error::StatusCode(code)) => Err(format!("HTTP {code}")),
        Err(e) => Err(e.to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The two budgets exist to be different: one covers "the connection never
    /// came up", the other "the transfer is slow but alive". Collapsing them
    /// back into one number is the regression this guards.
    #[test]
    fn transfer_budget_is_far_longer_than_connect_budget() {
        assert!(TRANSFER_TIMEOUT >= CONNECT_TIMEOUT * 5);
        assert!(PROBE_TIMEOUT < CONNECT_TIMEOUT);
    }

    /// A write-only share refuses to be read — that refusal is the healthy
    /// answer, and reading it as a failure would ground every cabinet.
    #[test]
    fn refusing_to_be_read_means_the_drop_is_alive() {
        for code in [401, 403, 405] {
            assert_eq!(classify(code), Reach::Up, "HTTP {code}");
        }
        assert_eq!(classify(404), Reach::ServerSaysNo(404));
        assert_eq!(classify(410), Reach::ServerSaysNo(410));
        assert_eq!(classify(503), Reach::ServerSaysNo(503));
        assert!(!classify(500).is_up());
    }

    #[test]
    fn a_failed_upload_leaves_the_file_on_disk() {
        let dir = std::env::temp_dir().join(format!("ht-rescue-{}", std::process::id()));
        let status = Arc::new(Mutex::new(UploadStatus::default()));
        assert!(write_rescue(&dir, "capture.png", b"payload", &status));
        assert_eq!(std::fs::read(dir.join("capture.png")).unwrap(), b"payload");
        assert_eq!(status.lock().rescued_in.as_deref(), Some(dir.as_path()));
        assert!(status.lock().rescue_error.is_none());
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The one case where a capture is truly lost has to be visible, not
    /// swallowed: an unusable rescue folder sets `rescue_error`.
    #[test]
    fn an_unusable_rescue_folder_is_reported() {
        let blocked = std::env::temp_dir().join(format!("ht-rescue-file-{}", std::process::id()));
        std::fs::write(&blocked, b"I am a file, not a folder").unwrap();
        let status = Arc::new(Mutex::new(UploadStatus::default()));
        assert!(!write_rescue(
            &blocked.join("sub"),
            "capture.png",
            b"payload",
            &status
        ));
        assert!(status.lock().rescue_error.is_some());
        assert!(status.lock().rescued_in.is_none());
        let _ = std::fs::remove_file(&blocked);
    }

    /// Closing the window mid-queue used to drop the remaining files silently.
    #[test]
    fn closing_mid_queue_saves_what_was_left() {
        let dir = std::env::temp_dir().join(format!("ht-rescue-queue-{}", std::process::id()));
        let uploader = Uploader::spawn(dir.clone());
        // Stop the worker first so nothing races us to the network.
        {
            let (m, cv) = &*uploader.queue;
            let mut g = m.lock();
            g.stop = true;
            cv.notify_all();
        }
        uploader.status.lock().pending += 2;
        {
            let (m, _) = &*uploader.queue;
            let mut g = m.lock();
            g.items.push_back(("a.png".into(), b"a".to_vec()));
            g.items.push_back(("b.png".into(), b"b".to_vec()));
        }
        assert_eq!(uploader.rescue_pending(), 2);
        assert_eq!(std::fs::read(dir.join("a.png")).unwrap(), b"a");
        assert_eq!(std::fs::read(dir.join("b.png")).unwrap(), b"b");
        let st = uploader.status();
        assert_eq!(st.pending, 0);
        assert_eq!(st.failed, 2);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
