//! Link trait and built-in transport implementations.
//!
//! A link represents the raw byte-level connection to an OBD-II adapter
//! (serial port, BLE GATT, WiFi socket, etc.). This is an open trait —
//! anyone can implement it for custom byte links.

#[cfg(feature = "ble")]
pub mod ble;
pub mod framed;
pub mod isotp;
pub mod logging;
pub mod mock;
#[cfg(feature = "serial")]
pub mod serial;
#[cfg(feature = "ble")]
pub use ble::{is_adapter_match, ADAPTER_NAME_PATTERNS};
pub use logging::{parse_raw_capture, CaptureMetadata, LoggingTransport};

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use crate::error::Obd2Error;
use async_trait::async_trait;

/// Callback invoked on each raw read chunk before response assembly.
/// Used by LoggingTransport to record transport-level reads.
pub type ChunkObserver = Arc<Mutex<dyn Fn(&[u8]) + Send>>;

/// Raw byte-level connection to an OBD-II adapter.
///
/// Implementors handle the raw byte-level communication over a specific
/// medium (serial, BLE, WiFi, etc.). The adapter layer builds on top
/// of this to implement protocol-specific commands.
///
/// # Example
///
/// ```rust,no_run
/// use obd2_core::transport::Link;
/// use obd2_core::error::Obd2Error;
/// use async_trait::async_trait;
///
/// struct MyTransport;
///
/// #[async_trait]
/// impl Link for MyTransport {
///     async fn write(&mut self, data: &[u8]) -> Result<(), Obd2Error> { Ok(()) }
///     async fn read(&mut self) -> Result<Vec<u8>, Obd2Error> { Ok(vec![]) }
///     async fn reset(&mut self) -> Result<(), Obd2Error> { Ok(()) }
///     fn name(&self) -> &str { "my-transport" }
///     fn set_chunk_observer(&mut self, _observer: Option<obd2_core::transport::ChunkObserver>) {}
/// }
/// ```
#[async_trait]
pub trait Link: Send + Sync {
    /// Send raw bytes to the adapter.
    async fn write(&mut self, data: &[u8]) -> Result<(), Obd2Error>;

    /// Read a response from the adapter.
    /// Returns the complete response as bytes (including any framing).
    async fn read(&mut self) -> Result<Vec<u8>, Obd2Error>;

    /// Reset the physical connection.
    async fn reset(&mut self) -> Result<(), Obd2Error>;

    /// Human-readable transport name for logging.
    fn name(&self) -> &str;

    /// Set a callback invoked on each raw read chunk during `read()`.
    /// Default implementation does nothing.
    fn set_chunk_observer(&mut self, _observer: Option<ChunkObserver>) {}

    /// Start raw protocol capture to a file. Returns true if capture started.
    /// Default: no-op (returns false). Overridden by LoggingTransport.
    fn start_raw_capture(&mut self, _path: &Path, _metadata: &logging::CaptureMetadata) -> bool {
        false
    }

    /// Stop raw protocol capture. Returns the file path if capture was active.
    /// Default: no-op (returns None). Overridden by LoggingTransport.
    fn stop_raw_capture(&mut self) -> Option<PathBuf> {
        None
    }

    /// Rename the active raw protocol capture file.
    /// Returns the new file path if the rename succeeded.
    /// Default: no-op (returns None). Overridden by LoggingTransport.
    fn rename_raw_capture(&mut self, _path: &Path) -> Option<PathBuf> {
        None
    }

    /// Write an annotation entry into the active raw capture.
    /// Default: no-op. Overridden by LoggingTransport.
    fn annotate_raw_capture(&mut self, _note: &str) {}

    /// Whether raw protocol capture is currently active.
    /// Default: false. Overridden by LoggingTransport.
    fn is_raw_capturing(&self) -> bool {
        false
    }
}
