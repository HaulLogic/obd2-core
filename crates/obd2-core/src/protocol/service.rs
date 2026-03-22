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
}
