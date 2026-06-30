//! # obd2-core
//!
//! Cross-platform OBD-II diagnostic library for Rust.
//!
//! The supported pre-`1.0` integration surface is session-first:
//!
//! - construct a [`session::Session`]
//! - let `Session` own initialization, discovery, routing, diagnostics, and polling
//! - treat adapters and transports as lower-level implementation details
//!
//! The current supported surface covers the non-J1939 path. J1939 types and helpers
//! may exist in the crate, but they remain a separate workstream and should not be
//! treated as complete `1.0` integration guidance yet.

pub mod adapter;
pub mod error;
pub mod protocol;
pub mod session;
pub mod specs;
pub mod store;
pub mod transport;
pub mod vehicle;
