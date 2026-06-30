//! Diagnostic service definitions.

/// Diagnostic session type (Mode 10).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagSession {
    Default,
    Programming,
    Extended,
}

/// Actuator command for Mode 2F.
#[derive(Debug, Clone)]
pub enum ActuatorCommand {
    ReturnToEcu,
    Adjust(Vec<u8>),
    Activate,
}

/// Readiness monitor status (decoded from Mode 01 PID 01).
#[derive(Debug, Clone)]
pub struct ReadinessStatus {
    pub mil_on: bool,
    pub dtc_count: u8,
    pub compression_ignition: bool,
    pub monitors: Vec<MonitorStatus>,
}

/// Status of a single readiness monitor.
#[derive(Debug, Clone)]
pub struct MonitorStatus {
    pub name: String,
    pub supported: bool,
    pub complete: bool,
}

/// Mode 06 test result.
#[derive(Debug, Clone)]
pub struct TestResult {
    pub test_id: u8,
    pub name: String,
    pub value: f64,
    pub min: Option<f64>,
    pub max: Option<f64>,
    pub passed: bool,
    pub unit: String,
}

/// Vehicle identification info (Mode 09).
#[derive(Debug, Clone)]
pub struct VehicleInfo {
    pub vin: String,
    pub calibration_ids: Vec<String>,
    pub cvns: Vec<u32>,
    pub ecu_name: Option<String>,
}

/// Extended DTC detail (Mode 19 sub-function 06).
#[derive(Debug, Clone)]
pub struct DtcDetail {
    pub code: String,
    pub occurrence_count: u16,
    pub aging_counter: u16,
}

/// Mode 05: O2 sensor monitoring test result (non-CAN only).
#[derive(Debug, Clone)]
pub struct O2TestResult {
    pub test_id: u8,
    pub test_name: &'static str,
    pub sensor: O2SensorLocation,
    pub value: f64,
    pub unit: &'static str,
}

/// O2 sensor location encoding for Mode 05.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum O2SensorLocation {
    Bank1Sensor1,
    Bank1Sensor2,
    Bank2Sensor1,
    Bank2Sensor2,
    Bank3Sensor1,
    Bank3Sensor2,
    Bank4Sensor1,
    Bank4Sensor2,
}

impl O2SensorLocation {
    /// Decode from the Mode 05 sensor number byte.
    pub fn from_byte(b: u8) -> Option<Self> {
        match b {
            0x01 => Some(Self::Bank1Sensor1),
            0x02 => Some(Self::Bank1Sensor2),
            0x03 => Some(Self::Bank2Sensor1),
            0x04 => Some(Self::Bank2Sensor2),
            0x05 => Some(Self::Bank3Sensor1),
            0x06 => Some(Self::Bank3Sensor2),
            0x07 => Some(Self::Bank4Sensor1),
            0x08 => Some(Self::Bank4Sensor2),
            _ => None,
        }
    }
}

impl std::fmt::Display for O2SensorLocation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Bank1Sensor1 => write!(f, "B1S1"),
            Self::Bank1Sensor2 => write!(f, "B1S2"),
            Self::Bank2Sensor1 => write!(f, "B2S1"),
            Self::Bank2Sensor2 => write!(f, "B2S2"),
            Self::Bank3Sensor1 => write!(f, "B3S1"),
            Self::Bank3Sensor2 => write!(f, "B3S2"),
            Self::Bank4Sensor1 => write!(f, "B4S1"),
            Self::Bank4Sensor2 => write!(f, "B4S2"),
        }
    }
}

/// Return the test name and unit for a Mode 05 TID.
pub fn o2_test_info(tid: u8) -> (&'static str, &'static str, fn(u16) -> f64) {
    match tid {
        0x01 => ("Rich-to-Lean Threshold Voltage", "V", |v| v as f64 * 0.005),
        0x02 => ("Lean-to-Rich Threshold Voltage", "V", |v| v as f64 * 0.005),
        0x03 => ("Low Sensor Voltage for Switch Time", "V", |v| {
            v as f64 * 0.005
        }),
        0x04 => ("High Sensor Voltage for Switch Time", "V", |v| {
            v as f64 * 0.005
        }),
        0x05 => ("Rich-to-Lean Switch Time", "s", |v| v as f64 * 0.004),
        0x06 => ("Lean-to-Rich Switch Time", "s", |v| v as f64 * 0.004),
        0x07 => ("Minimum Sensor Voltage", "V", |v| v as f64 * 0.005),
        0x08 => ("Maximum Sensor Voltage", "V", |v| v as f64 * 0.005),
        0x09 => ("Time Between Transitions", "s", |v| v as f64 * 0.04),
        _ => ("Unknown O2 Test", "", |v| v as f64),
    }
}

/// A raw diagnostic service request.
#[derive(Debug, Clone)]
pub struct ServiceRequest {
    pub service_id: u8,
    pub data: Vec<u8>,
    pub target: Target,
}

/// Request targeting.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Target {
    Broadcast,
    Module(String),
}

impl ServiceRequest {
    /// Create a Mode 01 read PID request.
    pub fn read_pid(pid: super::pid::Pid) -> Self {
        Self {
            service_id: 0x01,
            data: vec![pid.0],
            target: Target::Broadcast,
        }
    }

    /// Create a Mode 09 read VIN request.
    pub fn read_vin() -> Self {
        Self {
            service_id: 0x09,
            data: vec![0x02],
            target: Target::Broadcast,
        }
    }

    /// Create a Mode 03 read stored DTCs request.
    pub fn read_dtcs() -> Self {
        Self {
            service_id: 0x03,
            data: vec![],
            target: Target::Broadcast,
        }
    }

    /// Create a Mode 22 enhanced PID read.
    pub fn enhanced_read(service_id: u8, did: u16, target: Target) -> Self {
        Self {
            service_id,
            data: vec![(did >> 8) as u8, (did & 0xFF) as u8],
            target,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::pid::Pid;

    #[test]
    fn test_service_request_read_pid() {
        let req = ServiceRequest::read_pid(Pid::ENGINE_RPM);
        assert_eq!(req.service_id, 0x01);
        assert_eq!(req.data, vec![0x0C]);
        assert_eq!(req.target, Target::Broadcast);
    }

    #[test]
    fn test_service_request_read_vin() {
        let req = ServiceRequest::read_vin();
        assert_eq!(req.service_id, 0x09);
        assert_eq!(req.data, vec![0x02]);
    }

    #[test]
    fn test_service_request_enhanced_read() {
        let req = ServiceRequest::enhanced_read(0x22, 0x162F, Target::Module("ecm".into()));
        assert_eq!(req.service_id, 0x22);
        assert_eq!(req.data, vec![0x16, 0x2F]);
    }

    #[test]
    fn test_service_request_read_dtcs() {
        let req = ServiceRequest::read_dtcs();
        assert_eq!(req.service_id, 0x03);
        assert!(req.data.is_empty());
    }

    #[test]
    fn test_o2_sensor_location_roundtrip() {
        for b in 0x01..=0x08u8 {
            assert!(O2SensorLocation::from_byte(b).is_some());
        }
    }

    #[test]
    fn test_o2_test_info_all_standard_tids() {
        for tid in 0x01..=0x09u8 {
            let (name, unit, _) = o2_test_info(tid);
            assert!(
                !name.contains("Unknown"),
                "TID {:#04X} should be known",
                tid
            );
            assert!(!unit.is_empty(), "TID {:#04X} should have a unit", tid);
        }
    }

    #[test]
    fn test_o2_test_info_unknown_tid() {
        let (name, _, _) = o2_test_info(0xFF);
        assert!(name.contains("Unknown"));
    }
}
