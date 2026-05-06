//! Head tracking plugin for Visual Pinball X 10.8.1+.
//!
//! Compiled as a `cdylib`, loaded by VPX's `MsgPluginManager` at runtime.
//! The host calls `HeadTrackingPluginLoad` / `HeadTrackingPluginUnload`,
//! and we drive the player POV in real time via `VPXPluginAPI::SetActiveViewSetup`.

#![deny(unsafe_op_in_unsafe_fn)]

pub mod calibration;
pub mod camera;
pub mod filter;
pub mod plugin;
pub mod tracker;
