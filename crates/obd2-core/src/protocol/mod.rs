//! OBD-II and J1939 protocol types and parsing.

pub mod client;
pub mod codec;
pub mod doip;
pub mod dtc;
pub mod enhanced;
pub mod j1939;
pub mod pid;
pub mod service;
pub mod uds;

// Re-export key types
pub use client::{DiagResponse, J1979Client, ProtocolClient, RequestKind};
pub use codec::{
    decode_can_headers_off, decode_can_headers_on, decode_frame, decode_iso_kline_headers_on,
    decode_j1850_headers_on, BusFamily, CanFrame, CanFrameKind, DecodedFrame, IsoKLineFrame,
    J1850Frame,
};
pub use doip::{
    encode_doip_message, parse_doip_message, parse_doip_message_prefix, DoIpDiagnosticAck,
    DoIpDiagnosticMessage, DoIpHeader, DoIpMessage, DoIpPayloadType, DoIpRoutingActivationRequest,
    DoIpRoutingActivationResponse, DoIpVehicleIdentification,
};
pub use dtc::{Dtc, DtcCategory, DtcStatus, DtcStatusByte, Severity};
pub use enhanced::{Confidence, EnhancedPid, Formula};
pub use j1939::{
    decode_ccvs, decode_dm1, decode_dm2, decode_eec1, decode_eflp1, decode_et1, decode_lfe,
    parse_dm1, parse_dm11_response, parse_dm2, parse_dm24, parse_dm25, parse_dm3_response,
    parse_dm5, request_dm1, request_dm11, request_dm2, request_dm24, request_dm25, request_dm3,
    request_dm5, Acknowledgement, AcknowledgementControl, AddressClaim, Ccvs, Dm24, Dm24Entry,
    Dm25, Dm25FreezeFrame, Dm5, DmDtcMessage, DmLampStatus, Eec1, Eflp1, Et1, Fmi, J1939CanId,
    J1939Client, J1939Dtc, J1939Frame, J1939Message, J1939Name, J1939Request, J1939Transport,
    LampFlash, LampStatus, Lfe, Pgn, Spn, TpControlMessage, TpDataTransfer, TpReassembler,
};
pub use pid::{Pid, ValueType};
pub use service::{
    ActuatorCommand, DiagSession, MonitorStatus, O2SensorLocation, O2TestResult, ReadinessStatus,
    ServiceRequest, TestResult, VehicleInfo,
};
pub use uds::{
    decode_dtc_and_status_records, parse_uds_response, positive_response_sid, UdsClient,
    UdsDiagnosticSessionResponse, UdsDtc, UdsDtcReport, UdsEcuResetResponse, UdsPositiveResponse,
    UdsReadDtcResponse, DID_OBDONUDS, SID_CLEAR_DIAGNOSTIC_INFORMATION,
    SID_DIAGNOSTIC_SESSION_CONTROL, SID_ECU_RESET, SID_READ_DATA_BY_IDENTIFIER,
    SID_READ_DTC_INFORMATION, SID_TESTER_PRESENT,
};
