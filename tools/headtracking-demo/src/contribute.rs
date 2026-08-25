//! Optional "Share a capture" contribution uploader.
//!
//! Saves are done by the caller; this module just ships a file to a **write-
//! only** Nextcloud "file drop" off the UI thread. The drop only allows `PUT`
//! (no list / read / download — verified against the server), so the token
//! below is safe to ship in a public binary: leaking it lets someone *add*
//! files, never read anyone else's. Only the maintainer, authenticated on the
//! server, can see the uploads.

use std::sync::Arc;
use std::sync::mpsc::{Receiver, Sender};
use std::thread::JoinHandle;
use std::time::Duration;

use parking_lot::Mutex;
use tracing::{info, warn};

/// Public write-only share token (the tail of the share URL).
const DROP_TOKEN: &str = "cnYQZtzPHQSpCiW";
const DROP_HOST: &str = "https://nextcloud.syl21.org";
const UPLOAD_RETRIES: u32 = 3;

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

/// Snapshot of the uploader's progress, shown in the panel.
#[derive(Clone, Default)]
pub struct UploadStatus {
    pub pending: usize,
    pub uploaded: usize,
    pub last_error: Option<String>,
}

/// Background thread that PUTs queued files to the drop, with a few retries.
pub struct Uploader {
    tx: Sender<(String, Vec<u8>)>,
    status: Arc<Mutex<UploadStatus>>,
    _handle: JoinHandle<()>,
}

impl Uploader {
    pub fn spawn() -> Self {
        let (tx, rx) = std::sync::mpsc::channel::<(String, Vec<u8>)>();
        let status = Arc::new(Mutex::new(UploadStatus::default()));
        let st = Arc::clone(&status);
        let handle = std::thread::Builder::new()
            .name("uploader".into())
            .spawn(move || uploader_loop(&rx, &st))
            .expect("spawn uploader thread");
        Self {
            tx,
            status,
            _handle: handle,
        }
    }

    /// Queue one file (already encoded) for upload under `name`.
    pub fn submit(&self, name: String, bytes: Vec<u8>) {
        self.status.lock().pending += 1;
        let _ = self.tx.send((name, bytes));
    }

    pub fn status(&self) -> UploadStatus {
        self.status.lock().clone()
    }
}

fn uploader_loop(rx: &Receiver<(String, Vec<u8>)>, status: &Arc<Mutex<UploadStatus>>) {
    while let Ok((name, bytes)) = rx.recv() {
        let mut ok = false;
        for attempt in 1..=UPLOAD_RETRIES {
            match put_file(&name, &bytes) {
                Ok(()) => {
                    ok = true;
                    break;
                }
                Err(e) => {
                    warn!(name, attempt, "contribution upload failed: {e}");
                    if attempt < UPLOAD_RETRIES {
                        std::thread::sleep(Duration::from_secs(u64::from(2 * attempt)));
                    } else {
                        status.lock().last_error = Some(e);
                    }
                }
            }
        }
        let mut s = status.lock();
        s.pending = s.pending.saturating_sub(1);
        if ok {
            s.uploaded += 1;
            info!(name, "contribution uploaded");
        }
    }
}

fn put_file(name: &str, bytes: &[u8]) -> Result<(), String> {
    // ureq 3 moved timeouts onto the agent rather than the request, and no
    // longer turns a non-2xx into `Error::Status`: it is `StatusCode` now, and
    // the code is read off the response.
    let agent = ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(30)))
        .build()
        .new_agent();
    match agent
        .put(&drop_url(name))
        .header("Authorization", &basic_auth())
        .send(bytes)
    {
        Ok(_) => Ok(()),
        Err(ureq::Error::StatusCode(code)) => Err(format!("HTTP {code}")),
        Err(e) => Err(e.to_string()),
    }
}
