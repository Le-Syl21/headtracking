//! Pose-smoothing filters. We default to the 1€ filter for low-latency,
//! low-jitter head tracking.

pub mod median;
pub mod one_euro;

pub use median::MedianGate;
pub use one_euro::{OneEuro, OneEuroParams, OneEuroPose3D};
