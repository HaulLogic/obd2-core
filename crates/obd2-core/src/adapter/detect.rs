//! Chipset detection and capability probing.

use super::{AdapterInfo, Chipset, Capabilities};
use crate::vehicle::Protocol;

impl AdapterInfo {
    /// Detect adapter chipset and capabilities from initialization responses.
    ///
    /// # Arguments
    /// * `atz_response` — Response to ATZ (reset) command
    /// * `sti_response` — Response to STI command (None if not probed or no response)
    pub fn detect(atz_response: &str, sti_response: Option<&str>) -> Self {
        let (chipset, firmware) = if let Some(sti) = sti_response {
            if sti.contains("STN") {
                (Chipset::Stn, sti.trim().to_string())
            } else {
                detect_elm_version(atz_response)
            }
        } else {
            detect_elm_version(atz_response)
        };

        let capabilities = match chipset {
            Chipset::Stn => Capabilities {
                can_clear_dtcs: true,
                dual_can: true,
                enhanced_diag: true,
                battery_voltage: true,
                adaptive_timing: true,
            },
            Chipset::Elm327Genuine => Capabilities {
                can_clear_dtcs: true,
                dual_can: false,
                enhanced_diag: true,
                battery_voltage: true,
                adaptive_timing: true,
            },
            Chipset::Elm327Clone => Capabilities {
                can_clear_dtcs: false,
                dual_can: false,
                enhanced_diag: false,
                battery_voltage: true,
                adaptive_timing: false,
            },
            Chipset::Unknown => Capabilities::default(),
        };

        Self {
            chipset,
            firmware,
            protocol: Protocol::Auto,
            capabilities,
        }
    }
}

fn detect_elm_version(response: &str) -> (Chipset, String) {
    let response = response.trim();
    if let Some(ver_start) = response.find("ELM327") {
        let firmware = response[ver_start..].trim().to_string();
        // Extract version number
        if let Some(v_pos) = firmware.find('v').or_else(|| firmware.find('V')) {
            let ver_str = &firmware[v_pos + 1..];
            let version: f32 = ver_str
                .chars()
                .take_while(|c| c.is_ascii_digit() || *c == '.')
                .collect::<String>()
                .parse()
                .unwrap_or(0.0);

            if version >= 2.0 {
                (Chipset::Elm327Genuine, firmware)
            } else {
                (Chipset::Elm327Clone, firmware)
            }
        } else {
            (Chipset::Elm327Clone, firmware)
        }
    } else {
        (Chipset::Unknown, response.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_elm327_clone() {
        let info = AdapterInfo::detect("ELM327 v1.5", None);
        assert_eq!(info.chipset, Chipset::Elm327Clone);
        assert!(!info.capabilities.can_clear_dtcs);
        assert!(!info.capabilities.dual_can);
    }

    #[test]
    fn test_detect_elm327_genuine() {
        let info = AdapterInfo::detect("ELM327 v2.1", None);
        assert_eq!(info.chipset, Chipset::Elm327Genuine);
        assert!(info.capabilities.can_clear_dtcs);
        assert!(info.capabilities.enhanced_diag);
    }

    #[test]
    fn test_detect_stn() {
        let info = AdapterInfo::detect("ELM327 v1.5", Some("STN2120"));
        assert_eq!(info.chipset, Chipset::Stn);
        assert!(info.capabilities.dual_can);
        assert!(info.capabilities.enhanced_diag);
    }

    #[test]
    fn test_detect_unknown() {
        let info = AdapterInfo::detect("UNKNOWN DEVICE", None);
        assert_eq!(info.chipset, Chipset::Unknown);
    }

    #[test]
    fn test_detect_elm_with_garbage() {
        let info = AdapterInfo::detect("\r\nELM327 v2.2\r\n>", None);
        assert_eq!(info.chipset, Chipset::Elm327Genuine);
    }

    #[test]
    fn test_capabilities_default() {
        let caps = Capabilities::default();
        assert!(!caps.can_clear_dtcs);
        assert!(!caps.dual_can);
    }

    #[test]
    fn test_stn_has_all_capabilities() {
        let info = AdapterInfo::detect("ELM327 v1.5", Some("STN1110 v4.2"));
        assert!(info.capabilities.can_clear_dtcs);
        assert!(info.capabilities.dual_can);
        assert!(info.capabilities.enhanced_diag);
        assert!(info.capabilities.battery_voltage);
        assert!(info.capabilities.adaptive_timing);
    }
}
