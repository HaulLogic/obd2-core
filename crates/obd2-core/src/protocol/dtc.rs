//! DTC types and status definitions.

/// A Diagnostic Trouble Code read from the vehicle.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Dtc {
    pub code: String,
    pub category: DtcCategory,
    pub status: DtcStatus,
    pub description: Option<String>,
    pub severity: Option<Severity>,
    pub source_module: Option<String>,
    pub notes: Option<String>,
}

impl Default for Dtc {
    fn default() -> Self {
        Self {
            code: String::new(),
            category: DtcCategory::Powertrain,
            status: DtcStatus::Stored,
            description: None,
            severity: None,
            source_module: None,
            notes: None,
        }
    }
}

/// Category prefix of a DTC (P, C, B, U).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DtcCategory {
    Powertrain,
    Chassis,
    Body,
    Network,
}

/// Lifecycle status of a DTC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DtcStatus {
    Stored,
    Pending,
    Permanent,
}

/// Severity level of a DTC.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, serde::Deserialize)]
pub enum Severity {
    Info,
    Low,
    Medium,
    High,
    Critical,
}

/// GM/UDS Mode 19 extended status byte -- 8 flags per DTC.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DtcStatusByte {
    pub test_failed: bool,
    pub test_failed_this_cycle: bool,
    pub pending: bool,
    pub confirmed: bool,
    pub test_not_completed_since_clear: bool,
    pub test_failed_since_clear: bool,
    pub test_not_completed_this_cycle: bool,
    pub warning_indicator_requested: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dtc_default() {
        let dtc = Dtc::default();
        assert_eq!(dtc.category, DtcCategory::Powertrain);
        assert_eq!(dtc.status, DtcStatus::Stored);
        assert!(dtc.description.is_none());
    }

    #[test]
    fn test_severity_ordering() {
        assert!(Severity::Critical > Severity::High);
        assert!(Severity::High > Severity::Medium);
        assert!(Severity::Medium > Severity::Low);
        assert!(Severity::Low > Severity::Info);
    }

    #[test]
    fn test_dtc_status_byte_default() {
        let status = DtcStatusByte::default();
        assert!(!status.test_failed);
        assert!(!status.confirmed);
        assert!(!status.warning_indicator_requested);
    }
}
