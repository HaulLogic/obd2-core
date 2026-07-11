//! Mock adapter for testing.

use super::{
    Adapter, AdapterInfo, Capabilities, Chipset, InitializationReport, PhysicalTarget,
    RoutedRequest,
};
use crate::error::Obd2Error;
use crate::protocol::dtc::Dtc;
use crate::protocol::pid::Pid;
use crate::protocol::service::ServiceRequest;
use crate::vehicle::Protocol;
use async_trait::async_trait;
use std::collections::HashSet;

/// A mock adapter that simulates a vehicle for testing.
///
/// Returns realistic values for standard PIDs and configurable DTCs.
#[derive(Debug)]
pub struct MockAdapter {
    info: AdapterInfo,
    vin: String,
    dtcs: Vec<Dtc>,
    initialized: bool,
    supported: HashSet<Pid>,
    /// When false (default), J1939 Request-PGN (0xEA) returns NoData so standard
    /// OBD-II mocks do not falsely prove a heavy-duty bus (Desktop D3 honesty).
    j1939_enabled: bool,
}

impl MockAdapter {
    /// Create with default settings (generic vehicle).
    pub fn new() -> Self {
        Self::with_vin("1GCHK23224F000001") // Duramax VIN by default
    }

    /// Create with a specific VIN.
    pub fn with_vin(vin: &str) -> Self {
        let mut supported = HashSet::new();
        // Standard PIDs most vehicles support
        for pid in &[
            0x00u8, 0x01, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
            0x0F, 0x10, 0x11, 0x1C, 0x1F, 0x20, 0x21, 0x23, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31,
            0x33, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x49, 0x4A,
            0x4C, 0x59, 0x5C, 0x5E, 0x60, 0x61, 0x62, 0x63,
        ] {
            supported.insert(Pid(*pid));
        }

        Self {
            info: AdapterInfo {
                chipset: Chipset::Elm327Genuine,
                firmware: "MockAdapter v1.0".to_string(),
                protocol: Protocol::Can11Bit500,
                capabilities: Capabilities {
                    can_clear_dtcs: true,
                    dual_can: false,
                    enhanced_diag: true,
                    battery_voltage: true,
                    adaptive_timing: true,
                    kline_init: true,
                    kline_wakeup: true,
                    can_filtering: true,
                    can_flow_control: true,
                    can_extended_addressing: true,
                    can_silent_mode: true,
                },
            },
            vin: vin.to_string(),
            dtcs: Vec::new(),
            initialized: false,
            supported,
            j1939_enabled: false,
        }
    }

    /// Enable mock J1939 PGN responses (and advertise 29-bit CAN) for HD path tests.
    pub fn with_j1939_enabled(mut self) -> Self {
        self.j1939_enabled = true;
        self.info.protocol = Protocol::Can29Bit250;
        self
    }

    /// Set the DTCs that will be returned by Mode 03 queries.
    pub fn set_dtcs(&mut self, dtcs: Vec<Dtc>) {
        self.dtcs = dtcs;
    }

    /// Generate a realistic mock response for a standard PID.
    fn mock_pid_response(&self, pid: u8) -> Vec<u8> {
        match pid {
            // Engine performance (1-byte)
            0x04 => vec![0x40], // Engine load: 25%
            0x05 => vec![0x5A], // Coolant temp: 50°C (90-40)
            0x06 => vec![0x80], // STFT Bank 1: 0%
            0x07 => vec![0x80], // LTFT Bank 1: 0%
            0x08 => vec![0x80], // STFT Bank 2: 0%
            0x09 => vec![0x80], // LTFT Bank 2: 0%
            0x0A => vec![0x64], // Fuel pressure: 300 kPa
            0x0B => vec![0x65], // MAP: 101 kPa
            0x0D => vec![0x00], // Speed: 0 km/h (idle)
            0x0E => vec![0x8C], // Timing: 6°
            0x0F => vec![0x41], // IAT: 25°C
            0x11 => vec![0x26], // Throttle: 15%
            0x1C => vec![0x06], // OBD standard: EOBD
            0x2C => vec![0x1A], // Commanded EGR: 10%
            0x2D => vec![0x80], // EGR error: 0%
            0x2E => vec![0x40], // Commanded EVAP purge: 25%
            0x2F => vec![0xB3], // Fuel tank: 70%
            0x30 => vec![0x32], // Warm-ups since clear: 50
            0x33 => vec![0x65], // Baro: 101 kPa
            0x45 => vec![0x26], // Relative throttle: 15%
            0x46 => vec![0x41], // Ambient: 25°C
            0x47 => vec![0x40], // Abs throttle B: 25%
            0x49 => vec![0x1A], // Accel pedal D: 10%
            0x4A => vec![0x1A], // Accel pedal E: 10%
            0x4C => vec![0x26], // Commanded throttle: 15%
            0x5C => vec![0x78], // Oil temp: 80°C
            0x61 => vec![0x8D], // Demanded torque: 16%
            0x62 => vec![0x8D], // Actual torque: 16%
            // Engine performance (2-byte)
            0x0C => vec![0x0A, 0xA0], // RPM: 680
            0x10 => vec![0x00, 0xFA], // MAF: 2.5 g/s
            0x1F => vec![0x00, 0x3C], // Runtime: 60s
            0x21 => vec![0x00, 0x00], // Distance with MIL: 0 km
            0x23 => vec![0x13, 0x88], // Fuel rail gauge: 5000 kPa
            0x31 => vec![0x27, 0x10], // Distance since DTC clear: 10000 km
            0x3C => vec![0x0F, 0xA0], // Catalyst B1S1: 360°C
            0x3D => vec![0x0F, 0xA0], // Catalyst B2S1: 360°C
            0x3E => vec![0x0B, 0xB8], // Catalyst B1S2: 260°C
            0x3F => vec![0x0B, 0xB8], // Catalyst B2S2: 260°C
            0x42 => vec![0x38, 0x5C], // Voltage: 14.428V
            0x43 => vec![0x00, 0x64], // Absolute load: 39.2%
            0x44 => vec![0x80, 0x00], // Commanded equiv ratio: 1.0
            0x59 => vec![0x00, 0xC8], // Fuel rail abs: 200 kPa
            0x5E => vec![0x00, 0x64], // Fuel rate: 5.0 L/h
            0x63 => vec![0x03, 0x7F], // Reference torque: 895 Nm
            // Bitmaps
            0x00 => vec![0xBE, 0x3E, 0xB8, 0x11], // Supported PIDs 01-20
            0x01 => vec![0x00, 0x07, 0x65, 0x00], // Monitor status
            0x20 => vec![0x80, 0x12, 0xA0, 0x13], // Supported PIDs 21-40
            0x40 => vec![0xFA, 0xDC, 0x80, 0x00], // Supported PIDs 41-60
            0x60 => vec![0xE0, 0x00, 0x00, 0x00], // Supported PIDs 61-80
            _ => vec![0x00],                      // Unknown: return 0
        }
    }

    /// Generate a mock J1939 PGN response.
    fn mock_j1939_response(&self, pgn: u32) -> Vec<u8> {
        match pgn {
            // EEC1 (61444): RPM 680, torque 30%
            61444 => {
                let rpm_raw = (680.0_f64 / 0.125) as u16; // 5440
                vec![
                    0x00,                   // torque mode
                    155,                    // demand torque: -125 + 155 = 30%
                    155,                    // actual torque: -125 + 155 = 30%
                    (rpm_raw & 0xFF) as u8, // RPM low
                    (rpm_raw >> 8) as u8,   // RPM high
                    0xFF,
                    0xFF,
                    0xFF, // reserved
                ]
            }
            // CCVS (65265): 0 km/h, brake off, cruise off
            65265 => vec![0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            // ET1 (65262): coolant 50°C, fuel 20°C
            65262 => vec![90, 60, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF],
            // EFLP1 (65263): oil 400kPa, coolant 100kPa
            65263 => vec![0xFF, 50, 0xFF, 100, 0xFF, 0xFF, 0xFF, 0xFF],
            // LFE (65266): fuel rate 5.0 L/h
            65266 => {
                let rate_raw = (5.0_f64 / 0.05) as u16; // 100
                vec![
                    (rate_raw & 0xFF) as u8,
                    (rate_raw >> 8) as u8,
                    0x00,
                    0x02, // fuel economy
                    0xFF,
                    0xFF,
                    0xFF,
                    0xFF,
                ]
            }
            // DM1 (65226): no active DTCs
            65226 => vec![0x00, 0x00], // lamp status only, no DTCs
            // Unknown PGN
            _ => vec![0xFF; 8],
        }
    }
}

impl Default for MockAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Adapter for MockAdapter {
    async fn initialize(&mut self) -> Result<InitializationReport, Obd2Error> {
        self.initialized = true;
        Ok(InitializationReport {
            info: self.info.clone(),
            probe_attempts: Vec::new(),
            events: Vec::new(),
        })
    }

    async fn request(&mut self, req: &ServiceRequest) -> Result<Vec<u8>, Obd2Error> {
        match req.service_id {
            // Mode 01: Current data
            0x01 => {
                if let Some(&pid_code) = req.data.first() {
                    if self.supported.contains(&Pid(pid_code)) {
                        Ok(self.mock_pid_response(pid_code))
                    } else {
                        Err(Obd2Error::NoData)
                    }
                } else {
                    Err(Obd2Error::ParseError("no PID in request".into()))
                }
            }

            // Mode 03: Read stored DTCs
            0x03 => {
                let mut result = Vec::new();
                for dtc in &self.dtcs {
                    // Encode DTC back to bytes (simplified)
                    if dtc.code.len() == 5 {
                        let prefix = match dtc.code.chars().next() {
                            Some('P') => 0x00u8,
                            Some('C') => 0x40,
                            Some('B') => 0x80,
                            Some('U') => 0xC0,
                            _ => 0x00,
                        };
                        if let Ok(num) = u16::from_str_radix(&dtc.code[1..], 16) {
                            let a = prefix | ((num >> 8) as u8 & 0x3F);
                            let b = (num & 0xFF) as u8;
                            result.push(a);
                            result.push(b);
                        }
                    }
                }
                Ok(result)
            }

            // Mode 04: Clear DTCs
            0x04 => {
                self.dtcs.clear();
                Ok(vec![])
            }

            // Mode 09: Vehicle info
            0x09 => {
                match req.data.first() {
                    Some(0x02) => Ok(self.vin.as_bytes().to_vec()), // VIN
                    _ => Ok(vec![]),
                }
            }

            // Mode 05: O2 sensor monitoring
            0x05 => {
                match (req.data.first(), req.data.get(1)) {
                    // Return mock O2 data for B1S1 and B1S2 (sensors 0x01, 0x02)
                    (Some(_tid), Some(&sensor)) if sensor <= 0x02 => {
                        // Return a realistic threshold voltage ~0.45V = 90 * 0.005
                        Ok(vec![0x00, 0x5A])
                    }
                    // Other sensors not present
                    _ => Err(Obd2Error::NoData),
                }
            }

            // Mode 22: Enhanced read (return mock data)
            0x21 | 0x22 => Ok(vec![0x80, 0x00]),

            // Mode 10: Diagnostic session control
            0x10 => Ok(vec![]),

            // Mode 27: Security access (return mock seed)
            0x27 => {
                match req.data.first() {
                    Some(0x01) => Ok(vec![0xAA, 0xBB, 0xCC, 0xDD]), // Mock seed
                    Some(0x02) => Ok(vec![]),                       // Key accepted
                    _ => Ok(vec![]),
                }
            }

            // Mode 2F: Actuator control
            0x2F => Ok(vec![]),

            // Mode 3E: Tester present
            0x3E => Ok(vec![]),

            // J1939 Request PGN (0xEA)
            0xEA => {
                if !self.j1939_enabled {
                    return Err(Obd2Error::NoData);
                }
                let pgn = match (req.data.first(), req.data.get(1), req.data.get(2)) {
                    (Some(&lo), Some(&mid), Some(&hi)) => {
                        (lo as u32) | ((mid as u32) << 8) | ((hi as u32) << 16)
                    }
                    _ => return Err(Obd2Error::NoData),
                };
                Ok(self.mock_j1939_response(pgn))
            }

            _ => Err(Obd2Error::NoData),
        }
    }

    async fn routed_request(&mut self, req: &RoutedRequest) -> Result<Vec<u8>, Obd2Error> {
        if matches!(req.target, PhysicalTarget::Addressed(_))
            && matches!(req.service_id, 0x03 | 0x07 | 0x0A)
        {
            return Err(Obd2Error::NoData);
        }

        self.request(&ServiceRequest {
            service_id: req.service_id,
            data: req.data.clone(),
            target: crate::protocol::service::Target::Broadcast,
        })
        .await
    }

    async fn supported_pids(&mut self) -> Result<HashSet<Pid>, Obd2Error> {
        Ok(self.supported.clone())
    }

    async fn battery_voltage(&mut self) -> Result<Option<f64>, Obd2Error> {
        Ok(Some(14.4))
    }

    fn info(&self) -> &AdapterInfo {
        &self.info
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_mock_adapter_initialize() {
        let mut adapter = MockAdapter::new();
        let info = adapter.initialize().await.unwrap();
        assert_eq!(info.info.chipset, Chipset::Elm327Genuine);
    }

    #[tokio::test]
    async fn test_mock_adapter_supported_pids() {
        let mut adapter = MockAdapter::new();
        let pids = adapter.supported_pids().await.unwrap();
        assert!(pids.contains(&Pid::ENGINE_RPM));
        assert!(pids.contains(&Pid::VEHICLE_SPEED));
        assert!(pids.contains(&Pid::COOLANT_TEMP));
    }

    #[tokio::test]
    async fn test_mock_adapter_read_pid() {
        let mut adapter = MockAdapter::new();
        adapter.initialize().await.unwrap();
        let req = ServiceRequest::read_pid(Pid::ENGINE_RPM);
        let response = adapter.request(&req).await.unwrap();
        assert_eq!(response.len(), 2); // RPM is 2 bytes
    }

    #[tokio::test]
    async fn test_mock_adapter_read_vin() {
        let mut adapter = MockAdapter::with_vin("1GCHK23224F000001");
        let req = ServiceRequest::read_vin();
        let response = adapter.request(&req).await.unwrap();
        let vin = String::from_utf8_lossy(&response);
        assert_eq!(vin.len(), 17);
        assert_eq!(vin, "1GCHK23224F000001");
    }

    #[tokio::test]
    async fn test_mock_adapter_read_dtcs() {
        let mut adapter = MockAdapter::new();
        adapter.set_dtcs(vec![Dtc::from_code("P0420"), Dtc::from_code("P0171")]);
        let req = ServiceRequest::read_dtcs();
        let response = adapter.request(&req).await.unwrap();
        assert_eq!(response.len(), 4); // 2 DTCs * 2 bytes each
    }

    #[tokio::test]
    async fn test_mock_adapter_clear_dtcs() {
        let mut adapter = MockAdapter::new();
        adapter.set_dtcs(vec![Dtc::from_code("P0420")]);

        // Clear
        let req = ServiceRequest {
            service_id: 0x04,
            data: vec![],
            target: crate::protocol::service::Target::Broadcast,
        };
        adapter.request(&req).await.unwrap();

        // Verify cleared
        let req = ServiceRequest::read_dtcs();
        let response = adapter.request(&req).await.unwrap();
        assert!(response.is_empty());
    }

    #[tokio::test]
    async fn test_mock_adapter_unsupported_pid() {
        let mut adapter = MockAdapter::new();
        // PID 0xFF is not in supported set
        let req = ServiceRequest::read_pid(Pid(0xFF));
        let result = adapter.request(&req).await;
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_mock_adapter_battery_voltage() {
        let mut adapter = MockAdapter::new();
        let voltage = adapter.battery_voltage().await.unwrap();
        assert_eq!(voltage, Some(14.4));
    }
}
