//! Threshold evaluation for Session.

use crate::protocol::pid::Pid;
use crate::vehicle::{ThresholdResult, VehicleSpec};

/// Evaluate a standard PID reading against the matched spec's thresholds.
///
/// Returns None if no threshold is defined for this PID, or if no spec is matched.
pub fn evaluate_pid_threshold(
    spec: Option<&VehicleSpec>,
    pid: Pid,
    value: f64,
) -> Option<ThresholdResult> {
    let spec = spec?;
    let thresholds = spec.thresholds.as_ref()?;

    // Map PID code to threshold name
    let name = pid_threshold_name(pid)?;

    // Search engine thresholds first, then transmission
    for named in thresholds.engine.iter().chain(thresholds.transmission.iter()) {
        if named.name == name {
            return named.threshold.evaluate(value, pid.name());
        }
    }
    None
}

/// Evaluate an enhanced PID reading against the matched spec's thresholds.
pub fn evaluate_enhanced_threshold(
    spec: Option<&VehicleSpec>,
    did: u16,
    value: f64,
) -> Option<ThresholdResult> {
    let spec = spec?;
    let thresholds = spec.thresholds.as_ref()?;

    // Search by DID hex string as name
    let did_name = format!("{:#06X}", did);

    for named in thresholds.engine.iter().chain(thresholds.transmission.iter()) {
        if named.name == did_name {
            return named.threshold.evaluate(value, &did_name);
        }
    }
    None
}

/// Map a standard PID to its threshold name in the spec.
fn pid_threshold_name(pid: Pid) -> Option<&'static str> {
    match pid.0 {
        0x05 => Some("coolant_temp_c"),
        0x0C => Some("rpm"),
        0x5C => Some("oil_temp_c"),
        0x0B => Some("intake_map_kpa"),
        0x10 => Some("maf_gs"),
        0x42 => Some("battery_voltage"),
        0x2F => Some("fuel_tank_pct"),
        0x5E => Some("fuel_rate_lh"),
        0x46 => Some("ambient_temp_c"),
        0x0F => Some("intake_air_temp_c"),
        0x33 => Some("barometric_kpa"),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::{
        CommunicationSpec, EngineSpec, NamedThreshold, SpecIdentity, Threshold, ThresholdSet,
        VehicleSpec,
    };

    fn make_spec_with_thresholds() -> VehicleSpec {
        VehicleSpec {
            spec_version: Some("1.0".into()),
            identity: SpecIdentity {
                name: "Test".into(),
                model_years: (2020, 2020),
                makes: vec!["Test".into()],
                models: vec!["Test".into()],
                engine: EngineSpec {
                    code: "TEST".into(),
                    displacement_l: 2.0,
                    cylinders: 4,
                    layout: "I4".into(),
                    aspiration: "NA".into(),
                    fuel_type: "Gasoline".into(),
                    fuel_system: None,
                    compression_ratio: None,
                    max_power_kw: None,
                    max_torque_nm: None,
                    redline_rpm: 6500,
                    idle_rpm_warm: 700,
                    idle_rpm_cold: 900,
                    firing_order: None,
                    ecm_hardware: None,
                },
                transmission: None,
                vin_match: None,
            },
            communication: CommunicationSpec {
                buses: vec![],
                elm327_protocol_code: None,
            },
            thresholds: Some(ThresholdSet {
                engine: vec![
                    NamedThreshold {
                        name: "coolant_temp_c".into(),
                        threshold: Threshold {
                            min: Some(0.0),
                            max: Some(130.0),
                            warning_low: None,
                            warning_high: Some(105.0),
                            critical_low: None,
                            critical_high: Some(115.0),
                            unit: "\u{00B0}C".into(),
                        },
                    },
                    NamedThreshold {
                        name: "rpm".into(),
                        threshold: Threshold {
                            min: Some(0.0),
                            max: Some(7000.0),
                            warning_low: None,
                            warning_high: Some(6000.0),
                            critical_low: None,
                            critical_high: Some(6500.0),
                            unit: "RPM".into(),
                        },
                    },
                ],
                transmission: vec![],
            }),
            dtc_library: None,
            polling_groups: vec![],
            diagnostic_rules: vec![],
            known_issues: vec![],
        }
    }

    #[test]
    fn test_evaluate_normal() {
        let spec = make_spec_with_thresholds();
        let result = evaluate_pid_threshold(Some(&spec), Pid::COOLANT_TEMP, 90.0);
        assert!(result.is_none()); // normal range
    }

    #[test]
    fn test_evaluate_warning() {
        let spec = make_spec_with_thresholds();
        let result = evaluate_pid_threshold(Some(&spec), Pid::COOLANT_TEMP, 110.0);
        assert!(result.is_some());
        assert_eq!(result.unwrap().level, crate::vehicle::AlertLevel::Warning);
    }

    #[test]
    fn test_evaluate_critical() {
        let spec = make_spec_with_thresholds();
        let result = evaluate_pid_threshold(Some(&spec), Pid::COOLANT_TEMP, 118.0);
        assert!(result.is_some());
        assert_eq!(result.unwrap().level, crate::vehicle::AlertLevel::Critical);
    }

    #[test]
    fn test_evaluate_no_spec() {
        let result = evaluate_pid_threshold(None, Pid::COOLANT_TEMP, 110.0);
        assert!(result.is_none());
    }

    #[test]
    fn test_evaluate_no_threshold_for_pid() {
        let spec = make_spec_with_thresholds();
        let result = evaluate_pid_threshold(Some(&spec), Pid::VEHICLE_SPEED, 200.0);
        assert!(result.is_none()); // no speed threshold defined
    }

    #[test]
    fn test_evaluate_rpm_warning() {
        let spec = make_spec_with_thresholds();
        let result = evaluate_pid_threshold(Some(&spec), Pid::ENGINE_RPM, 6200.0);
        assert!(result.is_some());
        assert_eq!(result.unwrap().level, crate::vehicle::AlertLevel::Warning);
    }
}
