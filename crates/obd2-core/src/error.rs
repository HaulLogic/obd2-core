//! Error types for obd2-core.

/// OBD-II / UDS Negative Response Code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum NegativeResponse {
    /// 0x10
    GeneralReject,
    /// 0x11
    ServiceNotSupported,
    /// 0x12
    SubFunctionNotSupported,
    /// 0x13
    IncorrectMessageLength,
    /// 0x14
    ResponseTooLong,
    /// 0x22
    ConditionsNotCorrect,
    /// 0x31
    RequestOutOfRange,
    /// 0x33
    SecurityAccessDenied,
    /// 0x35
    InvalidKey,
    /// 0x36
    ExceededAttempts,
    /// 0x37
    TimeDelayNotExpired,
    /// 0x72
    GeneralProgrammingFailure,
    /// 0x78
    ResponsePending,
    /// Any NRC not yet modeled by this crate.
    Unknown(u8),
}

impl NegativeResponse {
    pub fn from_byte(b: u8) -> Option<Self> {
        match b {
            0x10 => Some(Self::GeneralReject),
            0x11 => Some(Self::ServiceNotSupported),
            0x12 => Some(Self::SubFunctionNotSupported),
            0x13 => Some(Self::IncorrectMessageLength),
            0x14 => Some(Self::ResponseTooLong),
            0x22 => Some(Self::ConditionsNotCorrect),
            0x31 => Some(Self::RequestOutOfRange),
            0x33 => Some(Self::SecurityAccessDenied),
            0x35 => Some(Self::InvalidKey),
            0x36 => Some(Self::ExceededAttempts),
            0x37 => Some(Self::TimeDelayNotExpired),
            0x72 => Some(Self::GeneralProgrammingFailure),
            0x78 => Some(Self::ResponsePending),
            _ => None,
        }
    }

    pub fn from_byte_or_unknown(b: u8) -> Self {
        Self::from_byte(b).unwrap_or(Self::Unknown(b))
    }

    pub fn code(&self) -> u8 {
        match self {
            Self::GeneralReject => 0x10,
            Self::ServiceNotSupported => 0x11,
            Self::SubFunctionNotSupported => 0x12,
            Self::IncorrectMessageLength => 0x13,
            Self::ResponseTooLong => 0x14,
            Self::ConditionsNotCorrect => 0x22,
            Self::RequestOutOfRange => 0x31,
            Self::SecurityAccessDenied => 0x33,
            Self::InvalidKey => 0x35,
            Self::ExceededAttempts => 0x36,
            Self::TimeDelayNotExpired => 0x37,
            Self::GeneralProgrammingFailure => 0x72,
            Self::ResponsePending => 0x78,
            Self::Unknown(code) => *code,
        }
    }
}

impl std::fmt::Display for NegativeResponse {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?} (0x{:02X})", self, self.code())
    }
}

/// Errors returned by obd2-core operations.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum Obd2Error {
    #[error("transport error: {0}")]
    Transport(String),

    #[error("adapter error: {0}")]
    Adapter(String),

    #[error("adapter is busy (stop polling first)")]
    AdapterBusy,

    #[error("timeout waiting for response")]
    Timeout,

    #[error("no data (vehicle did not respond)")]
    NoData,

    #[error("PID {pid:#04x} not supported by this vehicle")]
    UnsupportedPid { pid: u8 },

    #[error("module '{0}' not found in vehicle spec")]
    ModuleNotFound(String),

    #[error("negative response: {nrc} for service {service:#04x}")]
    NegativeResponse { service: u8, nrc: NegativeResponse },

    #[error("security access required (call enter_diagnostic_session + security_access first)")]
    SecurityRequired,

    #[error("no vehicle spec matched — load a spec or call identify_vehicle()")]
    NoSpec,

    #[error("bus '{0}' not available on this vehicle")]
    BusNotAvailable(String),

    #[error("spec parse error: {0}")]
    SpecParse(String),

    #[error("parse error: {0}")]
    ParseError(String),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error(transparent)]
    Other(#[from] Box<dyn std::error::Error + Send + Sync>),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display_timeout() {
        let err = Obd2Error::Timeout;
        assert_eq!(err.to_string(), "timeout waiting for response");
    }

    #[test]
    fn test_error_display_nrc() {
        let err = Obd2Error::NegativeResponse {
            service: 0x22,
            nrc: NegativeResponse::RequestOutOfRange,
        };
        let s = err.to_string();
        assert!(s.contains("RequestOutOfRange"));
        assert!(s.contains("0x22"));
    }

    #[test]
    fn test_io_error_conversion() {
        let io_err = std::io::Error::new(std::io::ErrorKind::BrokenPipe, "gone");
        let obd_err: Obd2Error = io_err.into();
        assert!(matches!(obd_err, Obd2Error::Io(_)));
    }

    #[test]
    fn test_nrc_from_byte() {
        assert_eq!(
            NegativeResponse::from_byte(0x31),
            Some(NegativeResponse::RequestOutOfRange)
        );
        assert_eq!(
            NegativeResponse::from_byte(0x78),
            Some(NegativeResponse::ResponsePending)
        );
        assert_eq!(NegativeResponse::from_byte(0xFF), None);
    }

    #[test]
    fn test_nrc_code() {
        assert_eq!(NegativeResponse::GeneralReject.code(), 0x10);
        assert_eq!(NegativeResponse::ResponsePending.code(), 0x78);
    }

    #[test]
    fn test_nrc_unknown_preserves_code() {
        let nrc = NegativeResponse::from_byte_or_unknown(0x94);
        assert_eq!(nrc, NegativeResponse::Unknown(0x94));
        assert_eq!(nrc.code(), 0x94);
    }

    #[test]
    fn test_nrc_display() {
        let nrc = NegativeResponse::SecurityAccessDenied;
        let s = format!("{}", nrc);
        assert!(s.contains("SecurityAccessDenied"));
        assert!(s.contains("0x33"));
    }

    #[test]
    fn test_error_is_send_sync() {
        fn assert_send<T: Send>() {}
        fn assert_sync<T: Sync>() {}
        // Obd2Error must be Send (required for async)
        assert_send::<Obd2Error>();
        assert_sync::<Obd2Error>();
    }
}
