//! Session orchestrator -- the primary entry point for consumers.

pub mod diag_session;
pub mod diagnostics;
pub mod enhanced;
pub mod modes;
pub mod poller;
pub mod threshold;

use std::collections::HashSet;
use crate::adapter::Adapter;
use crate::error::Obd2Error;
use crate::protocol::pid::Pid;
use crate::protocol::dtc::{Dtc, DtcStatus};
use crate::protocol::enhanced::{Value, Reading, ReadingSource};
use crate::protocol::service::{ServiceRequest, Target, VehicleInfo};
use crate::vehicle::{VehicleSpec, VehicleProfile, SpecRegistry, ModuleId};
use std::time::Instant;

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
    supported_pids_cache: Option<HashSet<Pid>>,
}

impl<A: Adapter> Session<A> {
    /// Create a new Session with default embedded specs.
    pub fn new(adapter: A) -> Self {
        Self {
            adapter,
            specs: SpecRegistry::with_defaults(),
            profile: None,
            supported_pids_cache: None,
        }
    }

    // -- Initialization --

    /// Initialize the underlying adapter (reset, echo off, protocol detect).
    ///
    /// Must be called before any PID reads. Returns adapter info on success.
    pub async fn initialize(&mut self) -> Result<crate::adapter::AdapterInfo, Obd2Error> {
        self.adapter.initialize().await
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

    // -- Mode 01: Current Data --

    /// Read a single standard PID.
    pub async fn read_pid(&mut self, pid: Pid) -> Result<Reading, Obd2Error> {
        let req = ServiceRequest::read_pid(pid);
        let data = self.adapter.request(&req).await?;
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
        if let Some(cached) = &self.supported_pids_cache {
            return Ok(cached.clone());
        }
        let pids = self.adapter.supported_pids().await?;
        self.supported_pids_cache = Some(pids.clone());
        Ok(pids)
    }

    // -- Mode 03/07/0A: DTCs --

    /// Read stored (confirmed) DTCs via broadcast.
    pub async fn read_dtcs(&mut self) -> Result<Vec<Dtc>, Obd2Error> {
        let req = ServiceRequest::read_dtcs();
        let data = self.adapter.request(&req).await?;
        Ok(Self::decode_dtc_response(&data, DtcStatus::Stored))
    }

    /// Read pending DTCs (Mode 07).
    pub async fn read_pending_dtcs(&mut self) -> Result<Vec<Dtc>, Obd2Error> {
        let req = ServiceRequest {
            service_id: 0x07,
            data: vec![],
            target: Target::Broadcast,
        };
        let data = self.adapter.request(&req).await?;
        Ok(Self::decode_dtc_response(&data, DtcStatus::Pending))
    }

    /// Read permanent DTCs (Mode 0A).
    pub async fn read_permanent_dtcs(&mut self) -> Result<Vec<Dtc>, Obd2Error> {
        let req = ServiceRequest {
            service_id: 0x0A,
            data: vec![],
            target: Target::Broadcast,
        };
        let data = self.adapter.request(&req).await?;
        Ok(Self::decode_dtc_response(&data, DtcStatus::Permanent))
    }

    /// Decode DTC bytes from Mode 03/07/0A response.
    fn decode_dtc_response(data: &[u8], status: DtcStatus) -> Vec<Dtc> {
        let mut dtcs = Vec::new();
        let mut i = 0;
        while i + 1 < data.len() {
            // Skip 00 00 padding
            if data[i] == 0 && data[i + 1] == 0 {
                i += 2;
                continue;
            }
            let mut dtc = Dtc::from_bytes(data[i], data[i + 1]);
            dtc.status = status;
            dtcs.push(dtc);
            i += 2;
        }
        dtcs
    }

    // -- Mode 04: Clear DTCs --

    /// Clear all DTCs and reset monitors (broadcast).
    pub async fn clear_dtcs(&mut self) -> Result<(), Obd2Error> {
        tracing::warn!("Clearing all DTCs -- readiness monitors will be reset");
        let req = ServiceRequest {
            service_id: 0x04,
            data: vec![],
            target: Target::Broadcast,
        };
        self.adapter.request(&req).await?;
        Ok(())
    }

    // -- Mode 09: Vehicle Information --

    /// Read VIN (17 characters).
    pub async fn read_vin(&mut self) -> Result<String, Obd2Error> {
        let req = ServiceRequest::read_vin();
        let data = self.adapter.request(&req).await?;
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
        let vin = self.read_vin().await?;
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
        Ok(profile)
    }

    // -- Enhanced PIDs (Mode 21/22) --

    /// Read an enhanced PID from a specific module.
    pub async fn read_enhanced(&mut self, did: u16, module: ModuleId) -> Result<Reading, Obd2Error> {
        // Look up the service ID for this enhanced PID from the spec
        let service_id = self.lookup_enhanced_service_id(did, &module);

        let req = ServiceRequest::enhanced_read(
            service_id,
            did,
            Target::Module(module.0.clone()),
        );
        let data = self.adapter.request(&req).await?;

        // Return raw bytes as Value::Raw until we have formula evaluation
        Ok(Reading {
            value: Value::Raw(data.clone()),
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
        modes::read_o2_monitoring(&mut self.adapter, test_id).await
    }

    /// Read all O2 sensor monitoring tests (TIDs 0x01-0x09).
    pub async fn read_all_o2_monitoring(
        &mut self,
    ) -> Result<Vec<crate::protocol::service::O2TestResult>, Obd2Error> {
        modes::read_all_o2_monitoring(&mut self.adapter).await
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

    /// Battery voltage.
    pub async fn battery_voltage(&mut self) -> Result<Option<f64>, Obd2Error> {
        self.adapter.battery_voltage().await
    }

    /// Raw service request (escape hatch).
    pub async fn raw_request(&mut self, service: u8, data: &[u8], target: Target) -> Result<Vec<u8>, Obd2Error> {
        let req = ServiceRequest {
            service_id: service,
            data: data.to_vec(),
            target,
        };
        self.adapter.request(&req).await
    }
}

impl<A: Adapter> std::fmt::Debug for Session<A> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Session")
            .field("profile", &self.profile)
            .field("specs_loaded", &self.specs.specs().len())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::mock::MockAdapter;

    #[tokio::test]
    async fn test_session_read_pid() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        let reading = session.read_pid(Pid::ENGINE_RPM).await.unwrap();
        // MockAdapter returns [0x0A, 0xA0] for RPM = 680
        assert_eq!(reading.value.as_f64().unwrap(), 680.0);
        assert_eq!(reading.unit, "RPM");
        assert_eq!(reading.source, ReadingSource::Live);
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
}
