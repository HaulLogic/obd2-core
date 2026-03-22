//! OBD-II protocol types and parsing.

pub mod pid;
pub mod dtc;
pub mod enhanced;
pub mod service;
pub mod codec;

// Re-export key types
pub use pid::{Pid, ValueType};
pub use dtc::{Dtc, DtcCategory, DtcStatus, DtcStatusByte, Severity};
pub use enhanced::{EnhancedPid, Formula, Confidence};
pub use service::{
    DiagSession, ActuatorCommand, ReadinessStatus, MonitorStatus,
    TestResult, VehicleInfo, ServiceRequest,
};
