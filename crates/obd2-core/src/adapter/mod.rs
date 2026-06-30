//! Adapter trait and built-in implementations.
//!
//! An adapter interprets OBD-II diagnostic requests over a transport.
//! For example, the ELM327 adapter translates service requests into
//! AT commands and hex strings, while a raw CAN adapter would frame
//! differently.

use crate::error::Obd2Error;
use crate::protocol::pid::Pid;
use crate::protocol::service::ServiceRequest;
use crate::transport::Transport;
use crate::vehicle::{KLineInit, PhysicalAddress, Protocol};
use async_trait::async_trait;
use std::collections::HashSet;

pub mod detect;
pub mod elm327;
pub mod mock;

/// Physical request target resolved from discovery/profile data.
#[derive(Debug, Clone)]
pub enum PhysicalTarget {
    Broadcast,
    Addressed(PhysicalAddress),
}

/// Diagnostic request with resolved physical routing.
#[derive(Debug, Clone)]
pub struct RoutedRequest {
    pub service_id: u8,
    pub data: Vec<u8>,
    pub target: PhysicalTarget,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolSelectionSource {
    AutoDetect,
    ExplicitProbe,
    SpecPreference,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProbeResult {
    Success,
    NoResponse,
    BusInitFailure,
    BusError,
    CanError,
    UnableToConnect,
    UnsupportedProtocol,
    AdapterFault,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProbeAttempt {
    pub protocol: Protocol,
    pub source: ProtocolSelectionSource,
    pub result: ProbeResult,
    pub detail: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AdapterEventKind {
    Reset,
    ProtocolSearching,
    ProtocolSelected(Protocol),
    HeaderChanged { header: String },
    HeaderReset { header: String },
    NullBytesFiltered { count: usize },
    SearchingDisplayed,
    BusBusy,
    BusError,
    CanError,
    DataError,
    RxError,
    Stopped,
    Err94,
    LowVoltageReset,
    UnknownCommand,
    UnsupportedProtocol,
    RecoveryAction { action: String },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AdapterEvent {
    pub kind: AdapterEventKind,
    pub detail: Option<String>,
}

#[derive(Debug, Clone)]
pub struct InitializationReport {
    pub info: AdapterInfo,
    pub probe_attempts: Vec<ProbeAttempt>,
    pub events: Vec<AdapterEvent>,
}

/// Protocol interpreter for OBD-II communication.
///
/// Translates high-level diagnostic requests (read PID, read DTCs)
/// into adapter-specific commands and parses the responses.
#[async_trait]
pub trait Adapter: Send {
    /// Initialize the adapter: reset, detect chipset, configure protocol.
    /// Returns information about the detected adapter hardware.
    async fn initialize(&mut self) -> Result<InitializationReport, Obd2Error>;

    /// Send a diagnostic service request and return the raw response data bytes.
    /// Response should NOT include the service ID echo or padding — just data.
    async fn request(&mut self, req: &ServiceRequest) -> Result<Vec<u8>, Obd2Error>;

    /// Send a diagnostic request with resolved physical routing information.
    async fn routed_request(&mut self, req: &RoutedRequest) -> Result<Vec<u8>, Obd2Error> {
        let target = match &req.target {
            PhysicalTarget::Broadcast => crate::protocol::service::Target::Broadcast,
            PhysicalTarget::Addressed(_) => {
                return Err(Obd2Error::Adapter(
                    "adapter does not support addressed routed requests".into(),
                ));
            }
        };
        self.request(&ServiceRequest {
            service_id: req.service_id,
            data: req.data.clone(),
            target,
        })
        .await
    }

    /// Query which standard PIDs are supported (Mode 01 PID 00/20/40/60 bitmaps).
    async fn supported_pids(&mut self) -> Result<HashSet<Pid>, Obd2Error>;

    /// Read the adapter's battery voltage measurement (if supported).
    async fn battery_voltage(&mut self) -> Result<Option<f64>, Obd2Error>;

    /// Return adapter information detected during initialization.
    fn info(&self) -> &AdapterInfo;

    /// Drain adapter events observed since the previous call.
    fn drain_events(&mut self) -> Vec<AdapterEvent> {
        Vec::new()
    }

    /// Mutable access to the underlying transport (if any).
    /// Returns None for adapters without a real transport (e.g., MockAdapter).
    fn transport_mut(&mut self) -> Option<&mut dyn Transport> {
        None
    }
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
#[derive(Debug, Clone, Default)]
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
    /// Supports K-line initialization control.
    pub kline_init: bool,
    /// Supports K-line wakeup configuration.
    pub kline_wakeup: bool,
    /// Supports CAN receive filtering.
    pub can_filtering: bool,
    /// Supports CAN flow control configuration.
    pub can_flow_control: bool,
    /// Supports CAN extended addressing.
    pub can_extended_addressing: bool,
    /// Supports CAN silent mode configuration.
    pub can_silent_mode: bool,
}

impl Protocol {
    pub fn from_elm_code(code: char) -> Option<Self> {
        match code {
            '1' => Some(Self::J1850Pwm),
            '2' => Some(Self::J1850Vpw),
            '3' => Some(Self::Iso9141(KLineInit::SlowInit)),
            '4' => Some(Self::Kwp2000(KLineInit::SlowInit)),
            '5' => Some(Self::Kwp2000(KLineInit::FastInit)),
            '6' => Some(Self::Can11Bit500),
            '7' => Some(Self::Can29Bit500),
            '8' => Some(Self::Can11Bit250),
            '9' => Some(Self::Can29Bit250),
            _ => None,
        }
    }
}
