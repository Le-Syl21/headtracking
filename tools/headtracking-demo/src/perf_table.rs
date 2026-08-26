//! The rolling table behind the demo's diagnostics panel.
//!
//! The log file gets one self-describing line per sample, because a file is
//! read long after the fact and out of context. On screen that is the wrong
//! trade: the labels are the same every time, so they belong in a header, and
//! the room they free goes to *history* -- which is what actually answers
//! "when did it start dropping, and what else happened at that moment".
//!
//! So the same events land here as typed rows: our own samples, and whatever
//! libfreenect2 has to say, interleaved on one timeline. The source column
//! comes from the `tracing` target, not from parsing the message text -- the
//! `freenect2-sys` bridge already tags its lines `target: "libfreenect2"`.

use std::collections::VecDeque;
use std::sync::{LazyLock, Mutex};

use tracing::field::{Field, Visit};
use tracing_subscriber::Layer;
use tracing_subscriber::layer::Context;

/// How many rows to keep. At one sample every 5 s plus device chatter, this is
/// roughly half an hour of history -- enough to cover a session without the
/// panel becoming a memory leak.
const CAPACITY: usize = 400;

/// One perf sample, in pipeline order.
#[derive(Debug, Clone)]
pub struct PerfRow {
    pub at: String,
    pub cam_fps: f32,
    pub ir_fps: f32,
    pub align_ms: f32,
    /// `None` once the anchor is locked: the detector has stopped, so a
    /// duration would be a stale reading rather than a measurement.
    pub anchor_ms: Option<f32>,
    pub head_ms: f32,
    pub median_us: f32,
    pub euro_us: f32,
    pub image_fps: f32,
    pub render_fps: f32,
    pub cpu_pct: f32,
    pub ram_mib: u64,
    pub used_ms: f32,
    pub budget_ms: f32,
}

impl PerfRow {
    #[must_use]
    pub fn over(&self) -> bool {
        self.used_ms > self.budget_ms
    }
}

/// Anything else that was logged while the samples were being taken.
#[derive(Debug, Clone)]
pub struct LogRow {
    pub at: String,
    pub level: &'static str,
    /// The `tracing` target: `libfreenect2`, `libfreenect`, or our own module.
    pub source: String,
    pub text: String,
}

#[derive(Debug, Clone)]
pub enum Row {
    Perf(Box<PerfRow>),
    Log(LogRow),
}

static ROWS: LazyLock<Mutex<VecDeque<Row>>> =
    LazyLock::new(|| Mutex::new(VecDeque::with_capacity(CAPACITY)));

fn push(row: Row) {
    let Ok(mut rows) = ROWS.lock() else {
        return; // a poisoned diagnostics buffer must never take the app down
    };
    if rows.len() == CAPACITY {
        rows.pop_front();
    }
    rows.push_back(row);
}

/// Record one perf sample. Called from the same place that writes the log
/// line, so the two can never disagree.
pub fn push_perf(row: PerfRow) {
    push(Row::Perf(Box::new(row)));
}

/// Snapshot for drawing, newest first.
#[must_use]
pub fn snapshot() -> Vec<Row> {
    ROWS.lock()
        .map(|r| r.iter().rev().cloned().collect())
        .unwrap_or_default()
}

pub fn clear() {
    if let Ok(mut rows) = ROWS.lock() {
        rows.clear();
    }
}

// ------------------------------------------------------------------ capture

#[derive(Default)]
struct MessageVisitor(String);

impl Visit for MessageVisitor {
    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        if field.name() == "message" {
            self.0 = format!("{value:?}");
        }
    }
    fn record_str(&mut self, field: &Field, value: &str) {
        if field.name() == "message" {
            self.0 = value.to_string();
        }
    }
}

/// Feeds [`ROWS`] from the tracing pipeline.
///
/// Our own `perf` lines are skipped: they arrive as [`PerfRow`] through
/// [`push_perf`], already typed, so capturing the formatted string too would
/// double every sample.
pub struct TableLayer;

impl<S: tracing::Subscriber> Layer<S> for TableLayer {
    fn on_event(&self, event: &tracing::Event<'_>, _: Context<'_, S>) {
        let mut v = MessageVisitor::default();
        event.record(&mut v);
        if v.0.starts_with("perf ") {
            return;
        }
        let meta = event.metadata();
        push(Row::Log(LogRow {
            at: super::stamp_hms(),
            level: meta.level().as_str(),
            source: meta.target().to_string(),
            text: v.0,
        }));
    }
}

// ----------------------------------------------------------------- drawing

/// Column groups, and the fields under each. The header is where the labels
/// live, so a row can be pure numbers -- that is the whole point of a table
/// over the one-line form the log file uses.
const GROUPS: &[(&str, &[&str])] = &[
    ("", &["time", "source"]),
    ("IN (fps)", &["cam", "ir+depth"]),
    ("MAP (ms)", &["align"]),
    ("AI (ms)", &["anchor", "head"]),
    ("FILTER (us)", &["median", "1euro"]),
    ("OUT (fps)", &["image", "render"]),
    ("SYS", &["cpu %", "ram MB"]),
    ("BUDGET (ms)", &["used", "max"]),
    ("", &["status"]),
];

fn num(v: f32, dec: usize) -> String {
    format!("{v:.dec$}")
}

/// Draw the rolling table, newest at the top.
pub fn ui(ui: &mut egui::Ui) {
    use egui_extras::{Column, TableBuilder};

    let rows = snapshot();
    ui.horizontal(|ui| {
        ui.label(format!("{} rows", rows.len()));
        if ui.button("Clear").clicked() {
            clear();
        }
    });

    let cols: Vec<&str> = GROUPS.iter().flat_map(|(_, f)| f.iter().copied()).collect();
    let mut builder = TableBuilder::new(ui).striped(true).resizable(true);
    for _ in &cols {
        builder = builder.column(Column::auto().at_least(46.0));
    }
    // `egui_extras` gives one header row, so the two levels are stacked as
    // two lines of text inside it: the group above its first field, blank
    // above the rest. Same reading as a spanned header, no colspan needed.
    builder
        .header(30.0, |mut header| {
            for (group, fields) in GROUPS {
                for (i, field) in fields.iter().enumerate() {
                    header.col(|ui| {
                        let top = if i == 0 { *group } else { "" };
                        ui.vertical(|ui| {
                            ui.label(egui::RichText::new(top).small().strong());
                            ui.label(egui::RichText::new(*field).small());
                        });
                    });
                }
            }
        })
        .body(|body| {
            body.rows(16.0, rows.len(), |mut row| {
                let idx = row.index();
                match &rows[idx] {
                    Row::Perf(p) => {
                        let cells: Vec<String> = vec![
                            p.at.clone(),
                            "perf".into(),
                            num(p.cam_fps, 1),
                            num(p.ir_fps, 1),
                            num(p.align_ms, 1),
                            p.anchor_ms.map_or_else(|| "done".into(), |v| num(v, 1)),
                            num(p.head_ms, 1),
                            num(p.median_us, 0),
                            num(p.euro_us, 0),
                            num(p.image_fps, 1),
                            num(p.render_fps, 1),
                            num(p.cpu_pct, 0),
                            p.ram_mib.to_string(),
                            num(p.used_ms, 1),
                            num(p.budget_ms, 1),
                            if p.over() {
                                "OVERLOAD!".into()
                            } else {
                                "OK".into()
                            },
                        ];
                        let over = p.over();
                        for (i, c) in cells.iter().enumerate() {
                            row.col(|ui| {
                                let mut t = egui::RichText::new(c).monospace();
                                if over && i + 1 == cells.len() {
                                    t = t.color(egui::Color32::from_rgb(0xc0, 0x39, 0x2b)).strong();
                                }
                                ui.label(t);
                            });
                        }
                    }
                    Row::Log(l) => {
                        row.col(|ui| {
                            ui.label(egui::RichText::new(&l.at).monospace());
                        });
                        row.col(|ui| {
                            ui.label(egui::RichText::new(&l.source).monospace().small());
                        });
                        // Device chatter has no columns of its own: it spills
                        // into the numeric span, which is what makes it read
                        // as an event on the same timeline rather than a
                        // measurement.
                        row.col(|ui| {
                            let colour = match l.level {
                                "ERROR" => Some(egui::Color32::from_rgb(0xc0, 0x39, 0x2b)),
                                "WARN" => Some(egui::Color32::from_rgb(0xd2, 0x9a, 0x22)),
                                _ => None,
                            };
                            let mut t = egui::RichText::new(&l.text).monospace().small();
                            if let Some(c) = colour {
                                t = t.color(c);
                            }
                            ui.add(egui::Label::new(t).truncate())
                                .on_hover_text(&l.text);
                        });
                        for _ in 3..cols.len() {
                            row.col(|_| {});
                        }
                    }
                }
            });
        });
}
