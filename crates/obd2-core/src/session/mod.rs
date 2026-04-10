//! Session orchestrator -- the primary entry point for consumers.

mod diag_session;
pub mod discovery;
pub mod diagnostics;
mod enhanced;
mod modes;
pub mod poller;
pub mod threshold;

use std::collections::HashSet;
use std::path::{Path, PathBuf};
use crate::adapter::{
    Adapter, AdapterEvent, AdapterEventKind, InitializationReport, PhysicalTarget, ProbeAttempt,
    RoutedRequest,
};
use crate::error::Obd2Error;
use crate::protocol::pid::Pid;
use crate::protocol::dtc::{Dtc, DtcStatus};
use crate::protocol::enhanced::{Reading, ReadingSource};
use crate::protocol::service::{ActuatorCommand, DiagSession, ServiceRequest, Target, VehicleInfo};
use crate::transport::CaptureMetadata;
use crate::vehicle::{VehicleSpec, VehicleProfile, SpecRegistry, ModuleId};
use discovery::{DiscoveryProfile, VisibleEcu};
use std::time::Instant;
pub use diag_session::{KeyFunction, SessionState as DiagnosticSessionState};

/// The primary entry point for all OBD-II operations.
///
/// A Session wraps an Adapter and provides high-level methods for
/// reading PIDs, DTCs, identifying vehicles, and more.
///
/// # Example
///
/// ```rust,no_run
/// use obd2_core::adapter::mock::MockAdapter;
/// use obd2_core::session::Session;
/// use obd2_core::protocol::pid::Pid;
///
/// # async fn example() -> Result<(), obd2_core::error::Obd2Error> {
/// let adapter = MockAdapter::new();
/// let mut session = Session::new(adapter);
/// let profile = session.identify_vehicle().await?;
/// let rpm = session.read_pid(Pid::ENGINE_RPM).await?;
/// # Ok(())
/// # }
/// ```
pub struct Session<A: Adapter> {
    adapter: A,
    specs: SpecRegistry,
    profile: Option<VehicleProfile>,
    discovery: Option<DiscoveryProfile>,
    connection_state: ConnectionState,
    diagnostic_state: DiagnosticSessionState,
    probe_attempts: Vec<ProbeAttempt>,
    visible_ecus: Vec<VisibleEcu>,
    supported_pids_cache: Option<HashSet<Pid>>,
    raw_capture: RawCaptureConfig,
    initialized: bool,
    request_in_flight: bool,
}

/// Lifecycle state of the adapter/vehicle connection.
///
/// Transitions are driven by adapter events and session operations.
/// Consumers can inspect this via [`Session::connection_state()`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConnectionState {
    /// Adapter object exists but has not been initialized.
    AdapterPresent,
    /// Adapter has been reset and configured but protocol is not yet confirmed.
    AdapterInitialized,
    /// Protocol auto-detection or explicit probing is in progress.
    ProtocolNegotiating,
    /// Communication with the vehicle is active.
    Connected,
    /// Adapter reported ignition-off / low-voltage condition (e.g., ELM/STN `LP ALERT`).
    /// Any successful subsequent request transitions back to `Connected`.
    IgnitionOff,
    /// Protocol probe exhausted all candidates without success.
    UnsupportedProtocol,
    /// Communication lost (adapter removed, transport failure, etc.).
    Disconnected,
    /// Unrecoverable error with descriptive message.
    Error(String),
}

/// Configuration for automatic raw protocol capture.
///
/// In debug builds, capture is enabled by default and writes `.obd2raw` files
/// to the configured directory. The file is renamed to include the VIN after
/// vehicle identification.
#[derive(Debug, Clone)]
pub struct RawCaptureConfig {
    /// Whether automatic capture is enabled (default: `true` in debug, `false` in release).
    pub enabled: bool,
    /// Directory for capture files (default: `"raw-captures"`).
    pub directory: PathBuf,
    current_path: Option<PathBuf>,
}

impl Default for RawCaptureConfig {
    fn default() -> Self {
        Self {
            enabled: cfg!(debug_assertions),
            directory: PathBuf::from("raw-captures"),
            current_path: None,
        }
    }
}

impl<A: Adapter> Session<A> {
    /// Create a new Session with default embedded specs.
    pub fn new(adapter: A) -> Self {
        Self {
            adapter,
            specs: SpecRegistry::with_defaults(),
            profile: None,
            discovery: None,
            connection_state: ConnectionState::AdapterPresent,
            diagnostic_state: DiagnosticSessionState::default(),
            probe_attempts: Vec::new(),
            visible_ecus: Vec::new(),
            supported_pids_cache: None,
            raw_capture: RawCaptureConfig::default(),
            initialized: false,
            request_in_flight: false,
        }
    }

    // -- Initialization --

    /// Initialize the underlying adapter (reset, echo off, protocol detect).
    ///
    /// Must be called before any PID reads. Returns adapter info on success.
    pub async fn initialize(&mut self) -> Result<crate::adapter::AdapterInfo, Obd2Error> {
        self.start_debug_raw_capture();
        self.connection_state = ConnectionState::ProtocolNegotiating;
        self.annotate_capture("session_state=ProtocolNegotiating");
        let report = self.adapter.initialize().await?;
        self.initialized = true;
        self.connection_state = ConnectionState::AdapterInitialized;
        self.apply_initialization_report(report);
        self.connection_state = ConnectionState::Connected;
        self.refresh_discovery_profile();
        Ok(self.adapter.info().clone())
    }

    // -- Spec Management --

    /// Load a vehicle spec from a YAML file.
    pub fn load_spec(&mut self, path: &std::path::Path) -> Result<(), Obd2Error> {
        self.specs.load_file(path)
    }

    /// Load all specs from a directory.
    pub fn load_spec_dir(&mut self, dir: &std::path::Path) -> Result<usize, Obd2Error> {
        self.specs.load_directory(dir)
    }

    /// Access the spec registry.
    pub fn specs(&self) -> &SpecRegistry {
        &self.specs
    }

    /// Current resolved discovery profile.
    pub fn discovery(&self) -> Option<&DiscoveryProfile> {
        self.discovery.as_ref()
    }

    /// Current connection lifecycle state.
    pub fn connection_state(&self) -> &ConnectionState {
        &self.connection_state
    }

    /// Current diagnostic session state (Default, Extended, or Programming).
    pub fn diagnostic_state(&self) -> &DiagnosticSessionState {
        &self.diagnostic_state
    }

    /// ECUs observed during communication (may differ from spec-declared topology).
    pub fn visible_ecus(&self) -> &[VisibleEcu] {
        &self.visible_ecus
    }

    /// Enable or disable automatic raw capture.
    pub fn set_raw_capture_enabled(&mut self, enabled: bool) {
        self.raw_capture.enabled = enabled;
    }

    /// Set the directory used for raw capture files.
    pub fn set_raw_capture_directory(&mut self, dir: impl Into<PathBuf>) {
        self.raw_capture.directory = dir.into();
    }

    /// Current raw capture path, if capture is active.
    pub fn raw_capture_path(&self) -> Option<&Path> {
        self.raw_capture.current_path.as_deref()
    }

    /// Start a manual raw capture on the session-owned transport.
    ///
    /// This preserves the session-first integration boundary for applications that
    /// need explicit capture controls without reaching through the adapter.
    pub fn start_raw_capture(
        &mut self,
        path: &Path,
        metadata: &CaptureMetadata,
    ) -> Result<(), Obd2Error> {
        let transport = self.adapter.transport_mut().ok_or_else(|| {
            Obd2Error::Transport("raw capture is unavailable: adapter has no transport".into())
        })?;

        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).map_err(|e| {
                Obd2Error::Transport(format!(
                    "failed to create raw capture directory {}: {}",
                    parent.display(),
                    e
                ))
            })?;
        }

        if transport.start_raw_capture(path, metadata) {
            self.raw_capture.current_path = Some(path.to_path_buf());
            Ok(())
        } else {
            Err(Obd2Error::Transport(
                "failed to start raw capture on the active transport".into(),
            ))
        }
    }

    /// Stop a manual raw capture on the session-owned transport.
    pub fn stop_raw_capture(&mut self) -> Result<Option<PathBuf>, Obd2Error> {
        let transport = self.adapter.transport_mut().ok_or_else(|| {
            Obd2Error::Transport("raw capture is unavailable: adapter has no transport".into())
        })?;

        let path = transport.stop_raw_capture();
        self.raw_capture.current_path = None;
        Ok(path)
    }

    // -- Mode 01: Current Data --

    /// Read a single standard PID.
    pub async fn read_pid(&mut self, pid: Pid) -> Result<Reading, Obd2Error> {
        self.ensure_initialized().await?;
        let req = ServiceRequest::read_pid(pid);
        let data = self.send_request(&req).await?;
        let value = pid.parse(&data)?;
        Ok(Reading {
            value,
            unit: pid.unit(),
            timestamp: Instant::now(),
            raw_bytes: data,
            source: ReadingSource::Live,
        })
    }

    /// Read multiple standard PIDs in sequence.
    pub async fn read_pids(&mut self, pids: &[Pid]) -> Result<Vec<(Pid, Reading)>, Obd2Error> {
        self.ensure_initialized().await?;
        let mut results = Vec::with_capacity(pids.len());
        for &pid in pids {
            match self.read_pid(pid).await {
                Ok(reading) => results.push((pid, reading)),
                Err(Obd2Error::NoData) => continue, // skip unsupported
                Err(e) => return Err(e),
            }
        }
        Ok(results)
    }

    /// Query which standard PIDs this vehicle supports.
    pub async fn supported_pids(&mut self) -> Result<HashSet<Pid>, Obd2Error> {
        self.ensure_initialized().await?;
        if let Some(cached) = &self.supported_pids_cache {
            return Ok(cached.clone());
        }
        let pids = self.query_supported_pids().await?;
        self.supported_pids_cache = Some(pids.clone());
        Ok(pids)
    }

    // -- Mode 03/07/0A: DTCs --

    /// Read stored (confirmed) DTCs via broadcast.
    pub async fn read_dtcs(&mut self) -> Result<Vec<Dtc>, Obd2Error> {
        self.ensure_initialized().await?;
        let req = ServiceRequest::read_dtcs();
        let data = self.send_request(&req).await?;
        Ok(modes::decode_dtc_bytes(&data, DtcStatus::Stored))
    }

    /// Read pending DTCs (Mode 07).
    pub async fn read_pending_dtcs(&mut self) -> Result<Vec<Dtc>, Obd2Error> {
        self.ensure_initialized().await?;
        let req = ServiceRequest {
            service_id: 0x07,
            data: vec![],
            target: Target::Broadcast,
        };
        let data = self.send_request(&req).await?;
        Ok(modes::decode_dtc_bytes(&data, DtcStatus::Pending))
    }

    /// Read permanent DTCs (Mode 0A).
    pub async fn read_permanent_dtcs(&mut self) -> Result<Vec<Dtc>, Obd2Error> {
        self.ensure_initialized().await?;
        let req = ServiceRequest {
            service_id: 0x0A,
            data: vec![],
            target: Target::Broadcast,
        };
        let data = self.send_request(&req).await?;
        Ok(modes::decode_dtc_bytes(&data, DtcStatus::Permanent))
    }

    /// Read and enrich all standard DTC classes.
    pub async fn read_all_dtcs(&mut self) -> Result<Vec<Dtc>, Obd2Error> {
        self.ensure_initialized().await?;

        let mut all_dtcs = Vec::new();
        all_dtcs.extend(self.read_dtcs().await?);

        if let Ok(mut pending) = self.read_pending_dtcs().await {
            all_dtcs.append(&mut pending);
        }
        if let Ok(mut permanent) = self.read_permanent_dtcs().await {
            all_dtcs.append(&mut permanent);
        }

        diagnostics::dedup_dtcs(&mut all_dtcs);
        diagnostics::enrich_dtcs(&mut all_dtcs, self.spec());
        Ok(all_dtcs)
    }

    // -- Mode 04: Clear DTCs --

    /// Clear all DTCs and reset monitors (broadcast).
    pub async fn clear_dtcs(&mut self) -> Result<(), Obd2Error> {
        self.ensure_initialized().await?;
        tracing::warn!("Clearing all DTCs -- readiness monitors will be reset");
        let req = ServiceRequest {
            service_id: 0x04,
            data: vec![],
            target: Target::Broadcast,
        };
        self.send_request(&req).await?;
        Ok(())
    }

    /// Clear DTCs on a specific module.
    pub async fn clear_dtcs_on_module(&mut self, module: ModuleId) -> Result<(), Obd2Error> {
        self.ensure_initialized().await?;
        tracing::warn!(module = %module.0, "clearing DTCs on specific module");
        let req = ServiceRequest {
            service_id: 0x04,
            data: vec![],
            target: Target::Module(module.0),
        };
        self.send_request(&req).await?;
        Ok(())
    }

    // -- Mode 09: Vehicle Information --

    /// Read VIN (17 characters).
    pub async fn read_vin(&mut self) -> Result<String, Obd2Error> {
        self.ensure_initialized().await?;
        let req = ServiceRequest::read_vin();
        let data = self.send_request(&req).await?;
        // Filter printable ASCII, take first 17 chars
        let vin: String = data.iter()
            .filter(|&&b| (0x20..=0x7E).contains(&b))
            .map(|&b| b as char)
            .take(17)
            .collect();
        if vin.len() == 17 {
            Ok(vin)
        } else {
            Err(Obd2Error::ParseError(format!("VIN too short: {} chars", vin.len())))
        }
    }

    /// Identify vehicle: read VIN, decode offline, match spec.
    ///
    /// Populates the VehicleProfile with:
    /// - Offline VIN decode (manufacturer, year, vehicle class)
    /// - Matched vehicle spec (if any)
    /// - Supported standard PIDs
    pub async fn identify_vehicle(&mut self) -> Result<VehicleProfile, Obd2Error> {
        self.ensure_initialized().await?;
        let vin = self.read_vin().await?;
        self.rename_raw_capture_for_vin(&vin);
        let supported = self.supported_pids().await.unwrap_or_default();

        // Decode VIN offline — manufacturer, year, vehicle class
        let decoded = crate::vehicle::vin::decode(&vin);

        // Match spec by VIN
        let spec = self.specs.match_vin(&vin).cloned();

        let profile = VehicleProfile {
            vin: vin.clone(),
            decoded_vin: Some(decoded),
            info: Some(VehicleInfo {
                vin: vin.clone(),
                calibration_ids: vec![],
                cvns: vec![],
                ecu_name: None,
            }),
            spec,
            supported_pids: supported,
        };

        self.profile = Some(profile.clone());
        self.refresh_discovery_profile();
        Ok(profile)
    }

    /// Read freeze frame data for a PID and frame index.
    pub async fn read_freeze_frame(&mut self, pid: Pid, frame: u8) -> Result<Reading, Obd2Error> {
        self.ensure_initialized().await?;
        let req = ServiceRequest {
            service_id: 0x02,
            data: vec![pid.0, frame],
            target: Target::Broadcast,
        };
        let data = self.send_request(&req).await?;
        let value = pid.parse(&data)?;
        Ok(Reading {
            value,
            unit: pid.unit(),
            timestamp: Instant::now(),
            raw_bytes: data,
            source: ReadingSource::FreezeFrame,
        })
    }

    /// Read readiness status from Mode 01 PID 01.
    pub async fn read_readiness(
        &mut self,
    ) -> Result<crate::protocol::service::ReadinessStatus, Obd2Error> {
        self.ensure_initialized().await?;
        let data = self.send_request(&ServiceRequest::read_pid(Pid(0x01))).await?;
        modes::decode_readiness(&data)
    }

    // -- Enhanced PIDs (Mode 21/22) --

    /// Read an enhanced PID from a specific module.
    pub async fn read_enhanced(&mut self, did: u16, module: ModuleId) -> Result<Reading, Obd2Error> {
        self.ensure_initialized().await?;
        // Look up the service ID for this enhanced PID from the spec
        let service_id = self.lookup_enhanced_service_id(did, &module);

        let req = ServiceRequest::enhanced_read(
            service_id,
            did,
            Target::Module(module.0.clone()),
        );
        let data = self.send_request(&req).await?;

        let value = enhanced::decode_enhanced_value(self.spec(), did, &module, &data);
        Ok(Reading {
            value,
            unit: "",
            timestamp: Instant::now(),
            raw_bytes: data,
            source: ReadingSource::Live,
        })
    }

    /// Look up the service ID for an enhanced PID from the spec.
    fn lookup_enhanced_service_id(&self, did: u16, module: &ModuleId) -> u8 {
        enhanced::find_service_id_from_spec(self.spec(), did, module)
    }

    /// List enhanced PIDs available for a module (from matched spec).
    pub fn module_pids(&self, module: ModuleId) -> Vec<&crate::protocol::enhanced::EnhancedPid> {
        enhanced::list_module_pids(self.spec(), &module)
    }

    // -- Mode 05: O2 Sensor Monitoring (non-CAN) --

    /// Read O2 sensor monitoring test results for a specific TID.
    pub async fn read_o2_monitoring(
        &mut self,
        test_id: u8,
    ) -> Result<Vec<crate::protocol::service::O2TestResult>, Obd2Error> {
        self.ensure_initialized().await?;
        let mut results = Vec::new();

        for sensor_byte in 0x01..=0x08u8 {
            let req = ServiceRequest {
                service_id: 0x05,
                data: vec![test_id, sensor_byte],
                target: Target::Broadcast,
            };

            match self.send_request(&req).await {
                Ok(data) if data.len() >= 2 => {
                    let Some(sensor) = crate::protocol::service::O2SensorLocation::from_byte(sensor_byte) else {
                        continue;
                    };
                    let raw_value = u16::from_be_bytes([data[0], data[1]]);
                    let (test_name, unit, convert) = crate::protocol::service::o2_test_info(test_id);
                    results.push(crate::protocol::service::O2TestResult {
                        test_id,
                        test_name,
                        sensor,
                        value: convert(raw_value),
                        unit,
                    });
                }
                Err(Obd2Error::NoData) => continue,
                Err(e) => return Err(e),
                _ => continue,
            }
        }

        Ok(results)
    }

    /// Read all O2 sensor monitoring tests (TIDs 0x01-0x09).
    pub async fn read_all_o2_monitoring(
        &mut self,
    ) -> Result<Vec<crate::protocol::service::O2TestResult>, Obd2Error> {
        self.ensure_initialized().await?;
        let mut results = Vec::new();
        for tid in 0x01..=0x09u8 {
            match self.read_o2_monitoring(tid).await {
                Ok(mut tid_results) => results.append(&mut tid_results),
                Err(Obd2Error::NoData) => continue,
                Err(e) => return Err(e),
            }
        }
        Ok(results)
    }

    /// Read on-board monitoring test results (Mode 06).
    pub async fn read_test_results(
        &mut self,
        test_id: u8,
    ) -> Result<Vec<crate::protocol::service::TestResult>, Obd2Error> {
        self.ensure_initialized().await?;
        let req = ServiceRequest {
            service_id: 0x06,
            data: vec![test_id],
            target: Target::Broadcast,
        };
        let data = self.send_request(&req).await?;
        Ok(modes::decode_test_results(&data))
    }

    /// Read full vehicle information (VIN, CALIDs, CVNs, ECU name).
    pub async fn read_vehicle_info(&mut self) -> Result<VehicleInfo, Obd2Error> {
        self.ensure_initialized().await?;
        let vin = self.read_vin().await?;

        let calibration_ids = match self.send_request(&ServiceRequest {
            service_id: 0x09,
            data: vec![0x04],
            target: Target::Broadcast,
        }).await {
            Ok(data) => {
                let cal_str: String = data.iter()
                    .filter(|&&b| (0x20..=0x7E).contains(&b))
                    .map(|&b| b as char)
                    .collect();
                if cal_str.is_empty() { vec![] } else { vec![cal_str] }
            }
            Err(_) => vec![],
        };

        let cvns = match self.send_request(&ServiceRequest {
            service_id: 0x09,
            data: vec![0x06],
            target: Target::Broadcast,
        }).await {
            Ok(data) if data.len() >= 4 => vec![u32::from_be_bytes([data[0], data[1], data[2], data[3]])],
            _ => vec![],
        };

        let ecu_name = match self.send_request(&ServiceRequest {
            service_id: 0x09,
            data: vec![0x0A],
            target: Target::Broadcast,
        }).await {
            Ok(data) => {
                let name: String = data.iter()
                    .filter(|&&b| (0x20..=0x7E).contains(&b))
                    .map(|&b| b as char)
                    .collect();
                if name.is_empty() { None } else { Some(name) }
            }
            Err(_) => None,
        };

        Ok(VehicleInfo {
            vin,
            calibration_ids,
            cvns,
            ecu_name,
        })
    }

    // -- J1939 Heavy-Duty Protocol --

    /// Read a J1939 Parameter Group from a heavy-duty vehicle.
    ///
    /// Sends a CAN 29-bit request for the specified PGN and returns the raw
    /// response bytes. Use the decoder functions in [`crate::protocol::j1939`]
    /// to parse the response.
    ///
    /// Requires an ELM327/STN adapter on a J1939-capable vehicle (CAN 29-bit 250 kbps).
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// # use obd2_core::protocol::j1939::{Pgn, decode_eec1};
    /// # async fn example(session: &mut obd2_core::session::Session<obd2_core::adapter::mock::MockAdapter>) {
    /// let data = session.read_j1939_pgn(Pgn::EEC1).await.unwrap();
    /// if let Some(eec1) = decode_eec1(&data) {
    ///     if let (Some(rpm), Some(torque)) = (eec1.engine_rpm, eec1.actual_torque_pct) {
    ///         println!("RPM: {rpm:.0}, Torque: {torque:.0}%");
    ///     }
    /// }
    /// # }
    /// ```
    pub async fn read_j1939_pgn(
        &mut self,
        pgn: crate::protocol::j1939::Pgn,
    ) -> Result<Vec<u8>, Obd2Error> {
        self.ensure_initialized().await?;
        // J1939 request PGN via CAN 29-bit.
        // ELM327/STN: use service 0x00 with the PGN encoded in the data bytes.
        // The PGN is sent as a 3-byte request: [PGN_low, PGN_mid, PGN_high]
        let pgn_bytes = [
            (pgn.0 & 0xFF) as u8,
            ((pgn.0 >> 8) & 0xFF) as u8,
            ((pgn.0 >> 16) & 0xFF) as u8,
        ];
        // Use raw_request with a J1939-specific service marker (0xEA = Request PGN)
        self.raw_request(0xEA, &pgn_bytes, Target::Broadcast).await
    }

    /// Read and decode J1939 active DTCs (DM1 — PGN 65226).
    ///
    /// Returns J1939-format DTCs (SPN + FMI), distinct from OBD-II P-codes.
    pub async fn read_j1939_dtcs(&mut self) -> Result<Vec<crate::protocol::j1939::J1939Dtc>, Obd2Error> {
        let data = self.read_j1939_pgn(crate::protocol::j1939::Pgn::DM1).await?;
        Ok(crate::protocol::j1939::decode_dm1(&data))
    }

    // -- Diagnostic Sessions --

    /// Enter a diagnostic session on a specific module (UDS Mode 0x10).
    ///
    /// Extended or Programming sessions unlock security-gated operations.
    /// Call [`security_access`](Self::security_access) after entering Extended
    /// to unlock actuator control on a module.
    pub async fn enter_diagnostic_session(
        &mut self,
        session: DiagSession,
        module: ModuleId,
    ) -> Result<(), Obd2Error> {
        self.ensure_initialized().await?;
        let sub = match session {
            DiagSession::Default => 0x01,
            DiagSession::Programming => 0x02,
            DiagSession::Extended => 0x03,
        };
        let req = ServiceRequest {
            service_id: 0x10,
            data: vec![sub],
            target: Target::Module(module.0.clone()),
        };
        self.send_request(&req).await?;
        self.diagnostic_state = match session {
            DiagSession::Default => DiagnosticSessionState::Default,
            DiagSession::Extended => DiagnosticSessionState::Extended { unlocked_modules: vec![] },
            DiagSession::Programming => DiagnosticSessionState::Programming,
        };
        Ok(())
    }

    /// Perform seed-key security access on a module (UDS Mode 0x27).
    ///
    /// Requests a seed from the module, computes the key using `key_fn`,
    /// and sends the key back. On success, the module is marked as unlocked
    /// in the diagnostic session state.
    ///
    /// If the seed is all zeros, the module is already unlocked and no key
    /// exchange is performed.
    pub async fn security_access(
        &mut self,
        module: ModuleId,
        key_fn: &KeyFunction,
    ) -> Result<(), Obd2Error> {
        self.ensure_initialized().await?;
        let seed_req = ServiceRequest {
            service_id: 0x27,
            data: vec![0x01],
            target: Target::Module(module.0.clone()),
        };
        let seed = self.send_request(&seed_req).await?;
        if seed.is_empty() {
            return Err(Obd2Error::Adapter("empty seed from Mode 27".into()));
        }
        if seed.iter().all(|&b| b == 0) {
            return Ok(());
        }
        let key = key_fn(&seed);
        let key_req = ServiceRequest {
            service_id: 0x27,
            data: std::iter::once(0x02).chain(key.into_iter()).collect(),
            target: Target::Module(module.0.clone()),
        };
        self.send_request(&key_req).await?;
        match &mut self.diagnostic_state {
            DiagnosticSessionState::Extended { unlocked_modules } => {
                if !unlocked_modules.contains(&module.0) {
                    unlocked_modules.push(module.0.clone());
                }
            }
            _ => {
                self.diagnostic_state = DiagnosticSessionState::Extended {
                    unlocked_modules: vec![module.0.clone()],
                };
            }
        }
        Ok(())
    }

    /// Send an actuator control command to a module (UDS Mode 0x2F).
    ///
    /// Requires an active Extended diagnostic session with the module
    /// security-unlocked. Returns [`Obd2Error::SecurityRequired`] otherwise.
    pub async fn actuator_control(
        &mut self,
        did: u16,
        module: ModuleId,
        command: &ActuatorCommand,
    ) -> Result<(), Obd2Error> {
        self.ensure_initialized().await?;
        match &self.diagnostic_state {
            DiagnosticSessionState::Extended { unlocked_modules } if unlocked_modules.contains(&module.0) => {}
            _ => return Err(Obd2Error::SecurityRequired),
        }
        let mut data = vec![(did >> 8) as u8, (did & 0xFF) as u8];
        match command {
            ActuatorCommand::ReturnToEcu => data.push(0x00),
            ActuatorCommand::Activate => data.push(0x03),
            ActuatorCommand::Adjust(bytes) => {
                data.push(0x03);
                data.extend_from_slice(bytes);
            }
        }
        let req = ServiceRequest {
            service_id: 0x2F,
            data,
            target: Target::Module(module.0.clone()),
        };
        self.send_request(&req).await?;
        Ok(())
    }

    /// Release actuator control, returning the DID to ECU control (UDS Mode 0x2F sub 0x00).
    pub async fn actuator_release(&mut self, did: u16, module: ModuleId) -> Result<(), Obd2Error> {
        self.ensure_initialized().await?;
        let req = ServiceRequest {
            service_id: 0x2F,
            data: vec![(did >> 8) as u8, (did & 0xFF) as u8, 0x00],
            target: Target::Module(module.0.clone()),
        };
        self.send_request(&req).await?;
        Ok(())
    }

    /// Send a tester-present keepalive to a module (UDS Mode 0x3E).
    ///
    /// Prevents the module from timing out the active diagnostic session.
    pub async fn tester_present(&mut self, module: ModuleId) -> Result<(), Obd2Error> {
        self.ensure_initialized().await?;
        let req = ServiceRequest {
            service_id: 0x3E,
            data: vec![],
            target: Target::Module(module.0.clone()),
        };
        self.send_request(&req).await?;
        Ok(())
    }

    /// End the diagnostic session, returning the module to Default mode (UDS Mode 0x10 sub 0x01).
    pub async fn end_diagnostic_session(&mut self, module: ModuleId) -> Result<(), Obd2Error> {
        self.ensure_initialized().await?;
        let req = ServiceRequest {
            service_id: 0x10,
            data: vec![0x01],
            target: Target::Module(module.0.clone()),
        };
        self.send_request(&req).await?;
        self.diagnostic_state = DiagnosticSessionState::Default;
        Ok(())
    }

    // -- Thresholds --

    /// Evaluate a standard PID reading against the matched spec's thresholds.
    ///
    /// Returns `None` if the value is in normal range, no threshold is defined
    /// for this PID, or no spec is matched.
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// # use obd2_core::protocol::pid::Pid;
    /// # use obd2_core::vehicle::AlertLevel;
    /// # async fn example(session: &obd2_core::session::Session<obd2_core::adapter::mock::MockAdapter>) {
    /// if let Some(result) = session.evaluate_threshold(Pid::COOLANT_TEMP, 110.0) {
    ///     match result.level {
    ///         AlertLevel::Warning => eprintln!("Warning: {}", result.message),
    ///         AlertLevel::Critical => eprintln!("CRITICAL: {}", result.message),
    ///         AlertLevel::Normal => {}
    ///     }
    /// }
    /// # }
    /// ```
    pub fn evaluate_threshold(&self, pid: Pid, value: f64) -> Option<crate::vehicle::ThresholdResult> {
        threshold::evaluate_pid_threshold(self.spec(), pid, value)
    }

    /// Evaluate an enhanced PID (DID) reading against the matched spec's thresholds.
    pub fn evaluate_enhanced_threshold(&self, did: u16, value: f64) -> Option<crate::vehicle::ThresholdResult> {
        threshold::evaluate_enhanced_threshold(self.spec(), did, value)
    }

    // -- State Accessors --

    /// Current vehicle profile (after identify_vehicle()).
    pub fn vehicle(&self) -> Option<&VehicleProfile> {
        self.profile.as_ref()
    }

    /// Matched spec (shorthand).
    pub fn spec(&self) -> Option<&VehicleSpec> {
        self.profile.as_ref().and_then(|p| p.spec.as_ref())
    }

    /// Adapter info.
    pub fn adapter_info(&self) -> &crate::adapter::AdapterInfo {
        self.adapter.info()
    }

    /// Mutable access to the underlying adapter (escape hatch).
    ///
    /// Prefer session-level methods for normal operations. Direct adapter
    /// access bypasses routing, lifecycle, and single-flight safety.
    pub fn adapter_mut(&mut self) -> &mut A {
        &mut self.adapter
    }

    /// Battery voltage.
    pub async fn battery_voltage(&mut self) -> Result<Option<f64>, Obd2Error> {
        self.ensure_initialized().await?;
        if self.request_in_flight {
            return Err(Obd2Error::AdapterBusy);
        }
        self.request_in_flight = true;
        let result = self.adapter.battery_voltage().await;
        let events = self.adapter.drain_events();
        self.request_in_flight = false;
        self.apply_adapter_events(&events);
        if result.is_ok() {
            self.mark_connection_active_if_recovered();
        }
        result
    }

    /// Raw service request (escape hatch).
    pub async fn raw_request(&mut self, service: u8, data: &[u8], target: Target) -> Result<Vec<u8>, Obd2Error> {
        self.ensure_initialized().await?;
        let req = ServiceRequest {
            service_id: service,
            data: data.to_vec(),
            target,
        };
        self.send_request(&req).await
    }

    async fn ensure_initialized(&mut self) -> Result<(), Obd2Error> {
        if !self.initialized {
            self.initialize().await?;
        }
        Ok(())
    }

    fn start_debug_raw_capture(&mut self) {
        if !self.raw_capture.enabled {
            return;
        }

        let Some(transport) = self.adapter.transport_mut() else {
            return;
        };

        if transport.is_raw_capturing() {
            return;
        }

        if std::fs::create_dir_all(&self.raw_capture.directory).is_err() {
            return;
        }

        let filename = format!(
            "unknown-{}-{}.obd2raw",
            sanitize_capture_name(transport.name()),
            capture_timestamp(),
        );
        let path = self.raw_capture.directory.join(filename);
        let metadata = CaptureMetadata {
            transport_type: capture_transport_type(transport.name()),
            port_or_device: transport.name().to_string(),
            baud_rate: None,
        };

        if transport.start_raw_capture(&path, &metadata) {
            self.raw_capture.current_path = Some(path);
        }
    }

    fn rename_raw_capture_for_vin(&mut self, vin: &str) {
        if !self.raw_capture.enabled {
            return;
        }

        let Some(transport) = self.adapter.transport_mut() else {
            return;
        };

        if !transport.is_raw_capturing() {
            return;
        }

        let filename = format!(
            "{}-{}-{}.obd2raw",
            sanitize_capture_name(vin),
            sanitize_capture_name(transport.name()),
            capture_timestamp(),
        );
        let new_path = self.raw_capture.directory.join(filename);
        if let Some(path) = transport.rename_raw_capture(&new_path) {
            self.raw_capture.current_path = Some(path);
        }
    }

    fn refresh_discovery_profile(&mut self) {
        self.discovery = Some(discovery::resolve_discovery_profile(
            self.adapter.info(),
            self.spec(),
            &self.probe_attempts,
            &self.visible_ecus,
        ));
    }

    fn mark_connection_active_if_recovered(&mut self) {
        if matches!(
            &self.connection_state,
            ConnectionState::IgnitionOff | ConnectionState::Disconnected | ConnectionState::Error(_)
        ) {
            self.connection_state = ConnectionState::Connected;
        }
    }

    async fn query_supported_pids(&mut self) -> Result<HashSet<Pid>, Obd2Error> {
        let mut all_supported = HashSet::new();

        for base in [0x00u8, 0x20, 0x40, 0x60] {
            let req = ServiceRequest::read_pid(Pid(base));
            match self.send_request(&req).await {
                Ok(data) if data.len() >= 4 => {
                    for pid_code in Self::parse_supported_pid_bitmap(&data, base) {
                        all_supported.insert(Pid(pid_code));
                    }
                }
                Ok(_) => break,
                Err(Obd2Error::NoData) => break,
                Err(e) => return Err(e),
            }
        }

        Ok(all_supported)
    }

    async fn send_request(&mut self, req: &ServiceRequest) -> Result<Vec<u8>, Obd2Error> {
        if self.request_in_flight {
            return Err(Obd2Error::AdapterBusy);
        }
        self.request_in_flight = true;
        let routed = match self.resolve_request(req) {
            Ok(routed) => routed,
            Err(err) => {
                self.request_in_flight = false;
                return Err(err);
            }
        };
        self.record_visible_target(&routed.target);
        let result = self.adapter.routed_request(&routed).await;
        let events = self.adapter.drain_events();
        self.request_in_flight = false;
        self.apply_adapter_events(&events);
        if result.is_ok() {
            self.mark_connection_active_if_recovered();
        }
        result
    }

    fn resolve_request(&self, req: &ServiceRequest) -> Result<RoutedRequest, Obd2Error> {
        let target = match &req.target {
            Target::Broadcast => PhysicalTarget::Broadcast,
            Target::Module(module) => PhysicalTarget::Addressed(self.resolve_module_address(module)?),
        };
        Ok(RoutedRequest {
            service_id: req.service_id,
            data: req.data.clone(),
            target,
        })
    }

    fn resolve_module_address(&self, module: &str) -> Result<crate::vehicle::PhysicalAddress, Obd2Error> {
        let discovery = self.discovery.as_ref()
            .ok_or_else(|| Obd2Error::Adapter("no discovery profile available".into()))?;
        let module_id = ModuleId::new(module);
        let resolved = discovery.modules.get(&module_id)
            .ok_or_else(|| Obd2Error::ModuleNotFound(module.to_string()))?;
        if let Some(active_bus) = discovery.active_bus.as_ref() {
            if resolved.bus != active_bus.id {
                return Err(Obd2Error::BusNotAvailable(resolved.bus.0.clone()));
            }
        }
        Ok(resolved.address.clone())
    }

    fn apply_initialization_report(&mut self, report: InitializationReport) {
        self.probe_attempts = report.probe_attempts;
        self.apply_adapter_events(&report.events);
        self.discovery = Some(discovery::resolve_discovery_profile(
            &report.info,
            self.spec(),
            &self.probe_attempts,
            &self.visible_ecus,
        ));
    }

    fn apply_adapter_events(&mut self, events: &[AdapterEvent]) {
        for event in events {
            self.annotate_capture(&format!("adapter_event={:?}", event.kind));
            match &event.kind {
                AdapterEventKind::BusBusy
                | AdapterEventKind::BusError
                | AdapterEventKind::CanError
                | AdapterEventKind::DataError
                | AdapterEventKind::RxError
                | AdapterEventKind::Stopped => {
                    self.connection_state = ConnectionState::Error(
                        event.detail.clone().unwrap_or_else(|| format!("{:?}", event.kind))
                    );
                }
                AdapterEventKind::Err94 | AdapterEventKind::LowVoltageReset => {
                    self.connection_state = ConnectionState::Disconnected;
                }
                AdapterEventKind::RecoveryAction { action } if action == "LP ALERT" => {
                    self.connection_state = ConnectionState::IgnitionOff;
                }
                AdapterEventKind::UnsupportedProtocol => {
                    self.connection_state = ConnectionState::UnsupportedProtocol;
                }
                AdapterEventKind::ProtocolSelected(_) => {
                    self.connection_state = ConnectionState::Connected;
                }
                _ => {}
            }
        }
        if self.initialized {
            self.refresh_discovery_profile();
        }
    }

    fn parse_supported_pid_bitmap(data: &[u8], base_pid: u8) -> Vec<u8> {
        let mut pids = Vec::new();
        for (byte_idx, &byte) in data.iter().take(4).enumerate() {
            for bit in 0..8 {
                if byte & (0x80 >> bit) != 0 {
                    pids.push(base_pid + (byte_idx as u8 * 8) + bit + 1);
                }
            }
        }
        pids
    }

    fn record_visible_target(&mut self, target: &PhysicalTarget) {
        let (id, address) = match target {
            PhysicalTarget::Broadcast => return,
            PhysicalTarget::Addressed(address) => (format!("{address:?}"), Some(address.clone())),
        };
        if let Some(existing) = self.visible_ecus.iter_mut().find(|ecu| ecu.id == id) {
            existing.observation_count += 1;
            return;
        }
        let (bus, module) = self.lookup_module_for_address(address.as_ref().unwrap());
        self.visible_ecus.push(VisibleEcu {
            id,
            bus,
            module,
            address,
            observation_count: 1,
        });
    }

    fn lookup_module_for_address(&self, address: &crate::vehicle::PhysicalAddress) -> (Option<crate::vehicle::BusId>, Option<ModuleId>) {
        let Some(spec) = self.spec() else {
            return (None, None);
        };
        for bus in &spec.communication.buses {
            for module in &bus.modules {
                if &module.address == address {
                    return (Some(bus.id.clone()), Some(module.id.clone()));
                }
            }
        }
        (None, None)
    }

    fn annotate_capture(&mut self, note: &str) {
        if let Some(transport) = self.adapter.transport_mut() {
            transport.annotate_raw_capture(note);
        }
    }
}

fn sanitize_capture_name(value: &str) -> String {
    let mut out = String::with_capacity(value.len());
    for ch in value.chars() {
        if ch.is_ascii_alphanumeric() {
            out.push(ch);
        } else if matches!(ch, '-' | '_') {
            out.push(ch);
        } else {
            out.push('_');
        }
    }
    let trimmed = out.trim_matches('_');
    if trimmed.is_empty() {
        "unknown".to_string()
    } else {
        trimmed.to_string()
    }
}

fn capture_timestamp() -> String {
    chrono::Utc::now().format("%Y%m%dT%H%M%S").to_string()
}

fn capture_transport_type(name: &str) -> String {
    let lower = name.to_ascii_lowercase();
    if lower.contains("obdlink") || lower.contains("ble") {
        "ble".to_string()
    } else if lower.contains("tty") || lower.contains("cu.") || lower.contains("com") || lower == "mock" {
        "serial".to_string()
    } else {
        "transport".to_string()
    }
}

impl<A: Adapter> std::fmt::Debug for Session<A> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Session")
            .field("profile", &self.profile)
            .field("discovery", &self.discovery)
            .field("specs_loaded", &self.specs.specs().len())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;
    use crate::adapter::elm327::Elm327Adapter;
    use crate::adapter::mock::MockAdapter;
    use crate::transport::LoggingTransport;
    use crate::transport::mock::MockTransport;
    use crate::vehicle::{BusConfig, BusId, CommunicationSpec, EngineSpec, Module, PhysicalAddress, SpecIdentity, VehicleSpec, VinMatcher};

    fn test_spec_with_modules() -> VehicleSpec {
        VehicleSpec {
            spec_version: Some("1.0".into()),
            identity: SpecIdentity {
                name: "Routing Test".into(),
                model_years: (2004, 2005),
                makes: vec!["Chevrolet".into()],
                models: vec!["Silverado".into()],
                engine: EngineSpec {
                    code: "LLY".into(),
                    displacement_l: 6.6,
                    cylinders: 8,
                    layout: "V8".into(),
                    aspiration: "turbocharged".into(),
                    fuel_type: "diesel".into(),
                    fuel_system: None,
                    compression_ratio: None,
                    max_power_kw: None,
                    max_torque_nm: None,
                    redline_rpm: 3250,
                    idle_rpm_warm: 680,
                    idle_rpm_cold: 780,
                    firing_order: None,
                    ecm_hardware: None,
                },
                transmission: None,
                vin_match: Some(VinMatcher {
                    vin_8th_digit: Some(vec!['2']),
                    wmi_prefixes: vec!["1GC".into()],
                    year_range: Some((2004, 2005)),
                }),
            },
            communication: CommunicationSpec {
                buses: vec![
                    BusConfig {
                        id: BusId("j1850vpw".into()),
                        protocol: crate::vehicle::Protocol::J1850Vpw,
                        speed_bps: 10400,
                        modules: vec![Module {
                            id: ModuleId::new("ecm"),
                            name: "ECM".into(),
                            address: PhysicalAddress::J1850 {
                                node: 0x10,
                                header: [0x6C, 0x10, 0xF1],
                            },
                            bus: BusId("j1850vpw".into()),
                        }],
                        description: None,
                    },
                    BusConfig {
                        id: BusId("can".into()),
                        protocol: crate::vehicle::Protocol::Can11Bit500,
                        speed_bps: 500_000,
                        modules: vec![Module {
                            id: ModuleId::new("tcm"),
                            name: "TCM".into(),
                            address: PhysicalAddress::Can11Bit {
                                request_id: 0x7E1,
                                response_id: 0x7E9,
                            },
                            bus: BusId("can".into()),
                        }],
                        description: None,
                    },
                ],
                elm327_protocol_code: Some("2".into()),
            },
            thresholds: None,
            polling_groups: vec![],
            diagnostic_rules: vec![],
            known_issues: vec![],
            dtc_library: None,
            enhanced_pids: vec![],
        }
    }

    fn set_profile_and_discovery<A: Adapter>(
        session: &mut Session<A>,
        spec: VehicleSpec,
        protocol: crate::vehicle::Protocol,
    ) {
        session.initialized = true;
        session.profile = Some(crate::vehicle::VehicleProfile {
            vin: "1GCHK23224F000001".into(),
            decoded_vin: None,
            info: None,
            spec: Some(spec.clone()),
            supported_pids: HashSet::new(),
        });
        session.discovery = Some(discovery::resolve_discovery_profile(
            &crate::adapter::AdapterInfo {
                chipset: crate::adapter::Chipset::Stn,
                firmware: "test".into(),
                protocol,
                capabilities: crate::adapter::Capabilities::default(),
            },
            Some(&spec),
            &[],
            &[],
        ));
    }

    #[tokio::test]
    async fn test_session_read_pid() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        let reading = session.read_pid(Pid::ENGINE_RPM).await.unwrap();
        // MockAdapter returns [0x0A, 0xA0] for RPM = 680
        assert_eq!(reading.value.as_f64().unwrap(), 680.0);
        assert_eq!(reading.unit, "RPM");
        assert_eq!(reading.source, ReadingSource::Live);
        assert!(session.initialized);
    }

    #[tokio::test]
    async fn test_session_read_multiple_pids() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        let results = session.read_pids(&[Pid::ENGINE_RPM, Pid::COOLANT_TEMP, Pid::VEHICLE_SPEED]).await.unwrap();
        assert_eq!(results.len(), 3);
    }

    #[tokio::test]
    async fn test_session_supported_pids() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        let pids = session.supported_pids().await.unwrap();
        assert!(pids.contains(&Pid::ENGINE_RPM));
        assert!(pids.contains(&Pid::VEHICLE_SPEED));
    }

    #[tokio::test]
    async fn test_session_supported_pids_cached() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        let pids1 = session.supported_pids().await.unwrap();
        let pids2 = session.supported_pids().await.unwrap();
        assert_eq!(pids1, pids2); // Second call uses cache
    }

    #[tokio::test]
    async fn test_session_read_vin() {
        let adapter = MockAdapter::with_vin("1GCHK23224F000001");
        let mut session = Session::new(adapter);
        let vin = session.read_vin().await.unwrap();
        assert_eq!(vin, "1GCHK23224F000001");
        let discovery = session.discovery().expect("discovery should be populated");
        assert_eq!(discovery.selected_protocol, crate::vehicle::Protocol::Can11Bit500);
    }

    #[tokio::test]
    async fn test_session_identify_vehicle() {
        let adapter = MockAdapter::with_vin("1GCHK23224F000001");
        let mut session = Session::new(adapter);
        let profile = session.identify_vehicle().await.unwrap();
        assert_eq!(profile.vin, "1GCHK23224F000001");
        // Should match the embedded Duramax spec
        assert!(profile.spec.is_some(), "should match Duramax spec by VIN");
        assert_eq!(profile.spec.as_ref().unwrap().identity.engine.code, "LLY");
        assert!(session.initialized);
        let discovery = session.discovery().expect("discovery should be populated");
        assert_eq!(discovery.selected_protocol, crate::vehicle::Protocol::Can11Bit500);
        assert_eq!(discovery.active_bus.as_ref().unwrap().id.0, "j1850vpw");
    }

    #[tokio::test]
    async fn test_session_identify_no_spec() {
        let adapter = MockAdapter::with_vin("JH4KA7660PC000001"); // Acura, no spec
        let mut session = Session::new(adapter);
        let profile = session.identify_vehicle().await.unwrap();
        assert!(profile.spec.is_none());
    }

    #[tokio::test]
    async fn test_session_read_dtcs() {
        let mut adapter = MockAdapter::new();
        adapter.set_dtcs(vec![
            Dtc::from_code("P0420"),
            Dtc::from_code("P0171"),
        ]);
        let mut session = Session::new(adapter);
        let dtcs = session.read_dtcs().await.unwrap();
        assert_eq!(dtcs.len(), 2);
        assert!(dtcs.iter().any(|d| d.code == "P0420"));
        assert!(dtcs.iter().any(|d| d.code == "P0171"));
    }

    #[tokio::test]
    async fn test_session_clear_dtcs() {
        let mut adapter = MockAdapter::new();
        adapter.set_dtcs(vec![Dtc::from_code("P0420")]);
        let mut session = Session::new(adapter);

        session.clear_dtcs().await.unwrap();
        let dtcs = session.read_dtcs().await.unwrap();
        assert!(dtcs.is_empty());
    }

    #[tokio::test]
    async fn test_session_battery_voltage() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        let voltage = session.battery_voltage().await.unwrap();
        assert_eq!(voltage, Some(14.4));
    }

    #[tokio::test]
    async fn test_session_battery_voltage_respects_busy_guard() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        session.request_in_flight = true;
        let err = session.battery_voltage().await.unwrap_err();
        assert!(matches!(err, Obd2Error::AdapterBusy));
    }

    #[tokio::test]
    async fn test_session_read_vehicle_info() {
        let adapter = MockAdapter::with_vin("1GCHK23224F000001");
        let mut session = Session::new(adapter);
        let info = session.read_vehicle_info().await.unwrap();
        assert_eq!(info.vin, "1GCHK23224F000001");
    }

    #[tokio::test]
    async fn test_session_read_all_dtcs() {
        let mut adapter = MockAdapter::new();
        adapter.set_dtcs(vec![Dtc::from_code("P0420"), Dtc::from_code("P0171")]);
        let mut session = Session::new(adapter);
        let dtcs = session.read_all_dtcs().await.unwrap();
        assert!(dtcs.len() >= 2);
        assert!(dtcs.iter().any(|d| d.code == "P0420"));
    }

    #[tokio::test]
    async fn test_session_read_o2_monitoring_owned_by_session() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        let results = session.read_o2_monitoring(0x01).await.unwrap();
        assert_eq!(results.len(), 2);
        assert_eq!(results[0].test_name, "Rich-to-Lean Threshold Voltage");
    }

    #[tokio::test]
    async fn test_session_read_all_o2_monitoring_owned_by_session() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        let results = session.read_all_o2_monitoring().await.unwrap();
        assert_eq!(results.len(), 18);
    }

    #[tokio::test]
    async fn test_session_debug_raw_capture_renamed_after_identify() {
        let mut transport = MockTransport::new();
        transport.expect("ATZ", "ELM327 v2.1\r\r>");
        transport.expect("STI", "?\r>");
        transport.expect("ATE0", "OK\r>");
        transport.expect("ATL0", "OK\r>");
        transport.expect("ATH0", "OK\r>");
        transport.expect("ATS0", "OK\r>");
        transport.expect("ATAT1", "OK\r>");
        transport.expect("ATSP0", "OK\r>");
        transport.expect("0100", "41 00 BE 3E B8 11\r>");
        transport.expect("ATDPN", "A6\r>");
        transport.expect("ATCAF1", "OK\r>");
        transport.expect("ATCFC1", "OK\r>");
        transport.expect("0902", "49 02 01 31 47 43 48 4B 32 33 32 32 34 46 30 30 30 30 30 31\r>");
        transport.expect("0100", "41 00 BE 3E B8 11\r>");

        let adapter = Elm327Adapter::new(Box::new(LoggingTransport::new(transport)));
        let mut session = Session::new(adapter);
        let dir = tempfile::tempdir().unwrap();
        session.set_raw_capture_directory(dir.path());
        session.set_raw_capture_enabled(true);

        session.initialize().await.unwrap();
        let initial = session.raw_capture_path().unwrap().to_path_buf();
        assert!(initial.exists());
        assert!(initial.file_name().unwrap().to_string_lossy().contains("unknown"));

        let profile = session.identify_vehicle().await.unwrap();
        assert_eq!(profile.vin, "1GCHK23224F000001");

        let renamed = session.raw_capture_path().unwrap().to_path_buf();
        assert!(renamed.exists());
        assert!(renamed.file_name().unwrap().to_string_lossy().contains("1GCHK23224F000001"));
    }

    #[tokio::test]
    async fn test_session_enters_ignition_off_on_low_power_alert_and_recovers() {
        let mut transport = MockTransport::new();
        setup_elm_init_can11(&mut transport);
        transport.expect("010C", "LP ALERT\r>");
        transport.expect("010C", "41 0C 0A A0\r>");

        let adapter = Elm327Adapter::new(Box::new(transport));
        let mut session = Session::new(adapter);
        session.initialize().await.unwrap();

        let first = session.read_pid(Pid::ENGINE_RPM).await;
        assert!(first.is_err());
        assert!(matches!(session.connection_state(), &ConnectionState::IgnitionOff));

        let second = session.read_pid(Pid::ENGINE_RPM).await.unwrap();
        assert!(second.value.as_f64().unwrap() > 0.0);
        assert!(matches!(session.connection_state(), &ConnectionState::Connected));
    }

    #[test]
    fn test_session_manual_raw_capture_owned_by_session() {
        let transport = MockTransport::new();
        let adapter = Elm327Adapter::new(Box::new(LoggingTransport::new(transport)));
        let mut session = Session::new(adapter);
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("manual.obd2raw");
        let metadata = CaptureMetadata {
            transport_type: "mock".into(),
            port_or_device: "mock".into(),
            baud_rate: None,
        };

        session.start_raw_capture(&path, &metadata).unwrap();
        assert_eq!(session.raw_capture_path(), Some(path.as_path()));

        let stopped = session.stop_raw_capture().unwrap();
        assert_eq!(stopped.as_deref(), Some(path.as_path()));
        assert!(session.raw_capture_path().is_none());
    }

    #[tokio::test]
    async fn test_session_routed_request_uses_elm_addressing_for_j1850() {
        let mut transport = MockTransport::new();
        setup_elm_init_j1850(&mut transport);
        transport.expect("AT SH 6C10F1", "OK\r>");
        transport.expect("22162F", "62 16 2F 80 00\r>");

        let adapter = Elm327Adapter::new(Box::new(transport));
        let mut session = Session::new(adapter);
        let spec = test_spec_with_modules();
        set_profile_and_discovery(&mut session, spec, crate::vehicle::Protocol::J1850Vpw);
        session.adapter.initialize().await.unwrap();
        session.refresh_discovery_profile();

        let reading = session.read_enhanced(0x162F, ModuleId::new("ecm")).await.unwrap();
        assert_eq!(reading.raw_bytes, vec![0x80, 0x00]);
    }

    #[tokio::test]
    async fn test_session_routed_request_switches_targeted_can_headers() {
        let mut transport = MockTransport::new();
        setup_elm_init_can11(&mut transport);
        transport.expect("AT SH 7E0", "OK\r>");
        transport.expect("221234", "62 12 34 12 34\r>");
        transport.expect("AT SH 7E1", "OK\r>");
        transport.expect("221235", "62 12 35 56 78\r>");

        let adapter = Elm327Adapter::new(Box::new(transport));
        let mut session = Session::new(adapter);
        let spec = can11_spec_with_two_modules();
        session.adapter.initialize().await.unwrap();
        set_profile_and_discovery(&mut session, spec, crate::vehicle::Protocol::Can11Bit500);

        let first = session.read_enhanced(0x1234, ModuleId::new("ecm")).await.unwrap();
        let second = session.read_enhanced(0x1235, ModuleId::new("tcm")).await.unwrap();
        assert_eq!(first.raw_bytes, vec![0x12, 0x34]);
        assert_eq!(second.raw_bytes, vec![0x56, 0x78]);
    }

    #[tokio::test]
    async fn test_session_routed_request_resets_to_broadcast_for_can11() {
        let mut transport = MockTransport::new();
        setup_elm_init_can11(&mut transport);
        transport.expect("AT SH 7E0", "OK\r>");
        transport.expect("221234", "62 12 34 12 34\r>");
        transport.expect("AT SH 7DF", "OK\r>");
        transport.expect("010C", "41 0C 0A A0\r>");

        let adapter = Elm327Adapter::new(Box::new(transport));
        let mut session = Session::new(adapter);
        let spec = can11_spec_with_two_modules();
        session.adapter.initialize().await.unwrap();
        set_profile_and_discovery(&mut session, spec, crate::vehicle::Protocol::Can11Bit500);

        let _ = session.read_enhanced(0x1234, ModuleId::new("ecm")).await.unwrap();
        let rpm = session.read_pid(Pid::ENGINE_RPM).await.unwrap();
        assert_eq!(rpm.value.as_f64().unwrap(), 680.0);
    }

    #[tokio::test]
    async fn test_session_module_request_without_discovery_fails() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        session.initialized = true;
        let err = session.read_enhanced(0x1234, ModuleId::new("ecm")).await.unwrap_err();
        assert!(matches!(err, Obd2Error::Adapter(_)));
    }

    #[tokio::test]
    async fn test_session_unknown_module_fails() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        let spec = test_spec_with_modules();
        set_profile_and_discovery(&mut session, spec, crate::vehicle::Protocol::J1850Vpw);
        let err = session.read_enhanced(0x1234, ModuleId::new("unknown")).await.unwrap_err();
        assert!(matches!(err, Obd2Error::ModuleNotFound(_)));
    }

    #[tokio::test]
    async fn test_session_module_on_wrong_bus_fails() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        session.initialized = true;
        session.profile = Some(crate::vehicle::VehicleProfile {
            vin: "1GCHK23224F000001".into(),
            decoded_vin: None,
            info: None,
            spec: Some(test_spec_with_modules()),
            supported_pids: HashSet::new(),
        });
        let mut modules = HashMap::new();
        modules.insert(
            ModuleId::new("tcm"),
            discovery::ResolvedModule {
                id: ModuleId::new("tcm"),
                name: "TCM".into(),
                bus: BusId("can".into()),
                address: PhysicalAddress::Can11Bit {
                    request_id: 0x7E1,
                    response_id: 0x7E9,
                },
            },
        );
        session.discovery = Some(discovery::DiscoveryProfile {
            adapter: crate::adapter::AdapterInfo {
                chipset: crate::adapter::Chipset::Stn,
                firmware: "test".into(),
                protocol: crate::vehicle::Protocol::J1850Vpw,
                capabilities: crate::adapter::Capabilities::default(),
            },
            selected_protocol: crate::vehicle::Protocol::J1850Vpw,
            protocol_choice_source: crate::adapter::ProtocolSelectionSource::AutoDetect,
            active_bus: Some(discovery::ResolvedBus {
                id: BusId("j1850vpw".into()),
                protocol: crate::vehicle::Protocol::J1850Vpw,
                speed_bps: 10400,
                description: None,
            }),
            modules,
            probe_attempts: Vec::new(),
            visible_ecus: Vec::new(),
        });

        let err = session.read_enhanced(0x1234, ModuleId::new("tcm")).await.unwrap_err();
        assert!(matches!(err, Obd2Error::BusNotAvailable(_)));
    }

    #[test]
    fn test_session_resolve_module_address_from_discovery() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        session.initialized = true;
        let spec = test_spec_with_modules();
        session.profile = Some(crate::vehicle::VehicleProfile {
            vin: "1GCHK23224F000001".into(),
            decoded_vin: None,
            info: None,
            spec: Some(spec.clone()),
            supported_pids: HashSet::new(),
        });
        session.discovery = Some(discovery::resolve_discovery_profile(
            &crate::adapter::AdapterInfo {
                chipset: crate::adapter::Chipset::Stn,
                firmware: "test".into(),
                protocol: crate::vehicle::Protocol::J1850Vpw,
                capabilities: crate::adapter::Capabilities::default(),
            },
            Some(&spec),
            &[],
            &[],
        ));

        let address = session.resolve_module_address("ecm").unwrap();
        match address {
            PhysicalAddress::J1850 { header, .. } => assert_eq!(header, [0x6C, 0x10, 0xF1]),
            other => panic!("unexpected address: {other:?}"),
        }
    }

    #[test]
    fn test_session_resolve_module_address_bus_mismatch() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        session.initialized = true;
        session.profile = Some(crate::vehicle::VehicleProfile {
            vin: "1GCHK23224F000001".into(),
            decoded_vin: None,
            info: None,
            spec: Some(test_spec_with_modules()),
            supported_pids: HashSet::new(),
        });
        let mut modules = HashMap::new();
        modules.insert(
            ModuleId::new("tcm"),
            discovery::ResolvedModule {
                id: ModuleId::new("tcm"),
                name: "TCM".into(),
                bus: BusId("can".into()),
                address: PhysicalAddress::Can11Bit {
                    request_id: 0x7E1,
                    response_id: 0x7E9,
                },
            },
        );
        session.discovery = Some(discovery::DiscoveryProfile {
            adapter: crate::adapter::AdapterInfo {
                chipset: crate::adapter::Chipset::Stn,
                firmware: "test".into(),
                protocol: crate::vehicle::Protocol::J1850Vpw,
                capabilities: crate::adapter::Capabilities::default(),
            },
            selected_protocol: crate::vehicle::Protocol::J1850Vpw,
            protocol_choice_source: crate::adapter::ProtocolSelectionSource::AutoDetect,
            active_bus: Some(discovery::ResolvedBus {
                id: BusId("j1850vpw".into()),
                protocol: crate::vehicle::Protocol::J1850Vpw,
                speed_bps: 10400,
                description: None,
            }),
            modules,
            probe_attempts: Vec::new(),
            visible_ecus: Vec::new(),
        });

        let err = session.resolve_module_address("tcm").unwrap_err();
        assert!(matches!(err, Obd2Error::BusNotAvailable(_)));
    }

    #[tokio::test]
    async fn test_session_no_spec_still_reads_pids() {
        let adapter = MockAdapter::with_vin("JH4KA7660PC000001");
        let mut session = Session::new(adapter);
        // Standard PIDs work without a spec
        let reading = session.read_pid(Pid::ENGINE_RPM).await.unwrap();
        assert!(reading.value.as_f64().is_ok());
    }

    #[tokio::test]
    async fn test_session_raw_request() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        let data = session.raw_request(0x09, &[0x02], Target::Broadcast).await.unwrap();
        assert!(!data.is_empty()); // VIN bytes
    }

    #[tokio::test]
    async fn test_session_diagnostic_methods_are_session_owned() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        let spec = test_spec_with_modules();
        session.profile = Some(crate::vehicle::VehicleProfile {
            vin: "1GCHK23224F000001".into(),
            decoded_vin: None,
            info: None,
            spec: Some(spec),
            supported_pids: HashSet::new(),
        });
        let _ = session.initialize().await.unwrap();
        session.refresh_discovery_profile();

        session
            .enter_diagnostic_session(DiagSession::Extended, ModuleId::new("tcm"))
            .await
            .unwrap();
        assert!(matches!(
            session.diagnostic_state(),
            DiagnosticSessionState::Extended { .. }
        ));

        let key_fn: KeyFunction = Box::new(|seed| seed.iter().map(|b| b ^ 0xFF).collect());
        session
            .security_access(ModuleId::new("tcm"), &key_fn)
            .await
            .unwrap();
        session
            .actuator_control(0x1196, ModuleId::new("tcm"), &ActuatorCommand::Activate)
            .await
            .unwrap();
        session
            .tester_present(ModuleId::new("tcm"))
            .await
            .unwrap();
        session
            .end_diagnostic_session(ModuleId::new("tcm"))
            .await
            .unwrap();

        assert_eq!(session.diagnostic_state(), &DiagnosticSessionState::Default);
    }

    fn setup_elm_init_can11(transport: &mut MockTransport) {
        transport.expect("ATZ", "ELM327 v2.1\r\r>");
        transport.expect("STI", "?\r>");
        transport.expect("ATE0", "OK\r>");
        transport.expect("ATL0", "OK\r>");
        transport.expect("ATH0", "OK\r>");
        transport.expect("ATS0", "OK\r>");
        transport.expect("ATAT1", "OK\r>");
        transport.expect("ATSP0", "OK\r>");
        transport.expect("0100", "41 00 BE 3E B8 11\r>");
        transport.expect("ATDPN", "A6\r>");
        transport.expect("ATCAF1", "OK\r>");
        transport.expect("ATCFC1", "OK\r>");
    }

    fn setup_elm_init_j1850(transport: &mut MockTransport) {
        transport.expect("ATZ", "ELM327 v2.1\r\r>");
        transport.expect("STI", "?\r>");
        transport.expect("ATE0", "OK\r>");
        transport.expect("ATL0", "OK\r>");
        transport.expect("ATH0", "OK\r>");
        transport.expect("ATS0", "OK\r>");
        transport.expect("ATAT1", "OK\r>");
        transport.expect("ATSP0", "OK\r>");
        transport.expect("0100", "NO DATA\r>");
        transport.expect("ATTP6", "OK\r>");
        transport.expect("0100", "NO DATA\r>");
        transport.expect("ATTP7", "OK\r>");
        transport.expect("0100", "NO DATA\r>");
        transport.expect("ATTP8", "OK\r>");
        transport.expect("0100", "NO DATA\r>");
        transport.expect("ATTP9", "OK\r>");
        transport.expect("0100", "NO DATA\r>");
        transport.expect("ATTP2", "OK\r>");
        transport.expect("0100", "41 00 BE 3E B8 11\r>");
    }

    fn can11_spec_with_two_modules() -> VehicleSpec {
        let mut spec = test_spec_with_modules();
        spec.communication.buses = vec![BusConfig {
            id: BusId("can".into()),
            protocol: crate::vehicle::Protocol::Can11Bit500,
            speed_bps: 500_000,
            modules: vec![
                Module {
                    id: ModuleId::new("ecm"),
                    name: "ECM".into(),
                    address: PhysicalAddress::Can11Bit {
                        request_id: 0x7E0,
                        response_id: 0x7E8,
                    },
                    bus: BusId("can".into()),
                },
                Module {
                    id: ModuleId::new("tcm"),
                    name: "TCM".into(),
                    address: PhysicalAddress::Can11Bit {
                        request_id: 0x7E1,
                        response_id: 0x7E9,
                    },
                    bus: BusId("can".into()),
                },
            ],
            description: None,
        }];
        spec
    }

    #[tokio::test]
    async fn test_session_records_visible_targeted_ecu() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        let spec = test_spec_with_modules();
        session.profile = Some(crate::vehicle::VehicleProfile {
            vin: "1GCHK23224F000001".into(),
            decoded_vin: None,
            info: None,
            spec: Some(spec.clone()),
            supported_pids: HashSet::new(),
        });
        let _ = session.initialize().await.unwrap();
        session.refresh_discovery_profile();

        let _ = session.read_enhanced(0x162F, ModuleId::new("tcm")).await.unwrap();

        let visible = session.visible_ecus();
        assert_eq!(visible.len(), 1);
        assert_eq!(visible[0].module.as_ref().unwrap().0, "tcm");
        assert_eq!(visible[0].observation_count, 1);
    }
}
