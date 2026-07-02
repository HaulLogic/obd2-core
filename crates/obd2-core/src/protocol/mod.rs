//! OBD-II and J1939 protocol types and parsing.

pub mod client;
pub mod codec;
pub mod dtc;
pub mod enhanced;
pub mod j1939;
pub mod pid;
pub mod service;

// Re-export key types
pub use client::{DiagResponse, J1979Client, ProtocolClient, RequestKind};
pub use codec::{
    decode_can_headers_off, decode_can_headers_on, decode_frame, decode_iso_kline_headers_on,
    decode_j1850_headers_on, BusFamily, CanFrame, CanFrameKind, DecodedFrame, IsoKLineFrame,
    J1850Frame,
};
pub use dtc::{Dtc, DtcCategory, DtcStatus, DtcStatusByte, Severity};
pub use enhanced::{Confidence, EnhancedPid, Formula};
pub use j1939::{
    decode_ccvs, decode_dm1, decode_eec1, decode_eflp1, decode_et1, decode_lfe, Ccvs, Eec1, Eflp1,
    Et1, J1939Dtc, Lfe, Pgn,
};
pub use pid::{Pid, ValueType};
pub use service::{
    ActuatorCommand, DiagSession, MonitorStatus, O2SensorLocation, O2TestResult, ReadinessStatus,
    ServiceRequest, TestResult, VehicleInfo,
};
