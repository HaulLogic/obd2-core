//! Adapter trait and built-in implementations.
//!
//! An adapter interprets OBD-II diagnostic requests over a transport.
//! For example, the ELM327 adapter translates service requests into
//! AT commands and hex strings, while a raw CAN adapter would frame
//! differently.

use std::collections::HashSet;
use async_trait::async_trait;
use crate::error::Obd2Error;
use crate::protocol::pid::Pid;
use crate::protocol::service::ServiceRequest;

pub mod detect;

/// Protocol interpreter for OBD-II communication.
///
/// Translates high-level diagnostic requests (read PID, read DTCs)
/// into adapter-specific commands and parses the responses.
#[async_trait]
pub trait Adapter: Send {
    /// Initialize the adapter: reset, detect chipset, configure protocol.
    /// Returns information about the detected adapter hardware.
    async fn initialize(&mut self) -> Result<AdapterInfo, Obd2Error>;

    /// Send a diagnostic service request and return the raw response data bytes.
    /// Response should NOT include the service ID echo or padding — just data.
    async fn request(&mut self, req: &ServiceRequest) -> Result<Vec<u8>, Obd2Error>;

    /// Query which standard PIDs are supported (Mode 01 PID 00/20/40/60 bitmaps).
    async fn supported_pids(&mut self) -> Result<HashSet<Pid>, Obd2Error>;

    /// Read the adapter's battery voltage measurement (if supported).
    async fn battery_voltage(&mut self) -> Result<Option<f64>, Obd2Error>;

    /// Return adapter information detected during initialization.
    fn info(&self) -> &AdapterInfo;
}

/// Information about the connected OBD-II adapter hardware.
#[derive(Debug, Clone)]
pub struct AdapterInfo {
    /// Detected chipset type.
    pub chipset: Chipset,
    /// Firmware version string from adapter.
    pub firmware: String,
    /// Detected OBD-II protocol.
    pub protocol: crate::vehicle::Protocol,
    /// Adapter capabilities.
    pub capabilities: Capabilities,
}

/// Known OBD-II adapter chipset types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chipset {
    /// ELM327 clone (v1.0-v1.5) — limited features.
    Elm327Clone,
    /// Genuine ELM327 (v2.0+) — full feature set.
    Elm327Genuine,
    /// STN chip (STN1110, STN2120) — enhanced features.
    Stn,
    /// Unknown or undetected chipset.
    Unknown,
}

/// Adapter hardware capabilities.
#[derive(Debug, Clone)]
pub struct Capabilities {
    /// Can clear DTCs (Mode 04).
    pub can_clear_dtcs: bool,
    /// Supports dual CAN buses.
    pub dual_can: bool,
    /// Supports enhanced diagnostics (Mode 22, etc.).
    pub enhanced_diag: bool,
    /// Can read battery voltage (AT RV).
    pub battery_voltage: bool,
    /// Supports adaptive timing (AT AT).
    pub adaptive_timing: bool,
}

impl Default for Capabilities {
    fn default() -> Self {
        Self {
            can_clear_dtcs: false,
            dual_can: false,
            enhanced_diag: false,
            battery_voltage: false,
            adaptive_timing: false,
        }
    }
}
