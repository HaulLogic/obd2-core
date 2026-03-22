//! Session orchestrator -- the primary entry point for consumers.

pub mod diagnostics;
pub mod enhanced;
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
            .filter(|&&b| b >= 0x20 && b <= 0x7E)
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
    pub async fn identify_vehicle(&mut self) -> Result<VehicleProfile, Obd2Error> {
        let vin = self.read_vin().await?;
        let supported = self.supported_pids().await.unwrap_or_default();

        // Decode VIN offline (for side effects / logging; result unused for now)
        let _decoded = crate::vehicle::vin::decode(&vin);

        // Match spec by VIN
        let spec = self.specs.match_vin(&vin).cloned();

        let profile = VehicleProfile {
            vin: vin.clone(),
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
    fn lookup_enhanced_service_id(&self, _did: u16, _module: &ModuleId) -> u8 {
        if let Some(_spec) = self.spec() {
            // Search modules in the spec for this DID
            // Default to 0x22 if not found
        }
        0x22 // Default: Mode 22
    }

    /// List enhanced PIDs available for a module (from matched spec).
    pub fn module_pids(&self, _module: ModuleId) -> Vec<&crate::protocol::enhanced::EnhancedPid> {
        // Would search spec modules for matching module ID
        // For now return empty -- full impl needs spec module data
        vec![]
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
