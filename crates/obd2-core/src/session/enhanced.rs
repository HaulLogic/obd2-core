//! Enhanced PID spec lookup and formula decoding utilities used by `Session`.

use crate::protocol::enhanced::{EnhancedPid, Value};
use crate::vehicle::{ModuleId, VehicleSpec};

/// List enhanced PIDs available for a module from the spec.
pub fn list_module_pids<'a>(
    spec: Option<&'a VehicleSpec>,
    module: &ModuleId,
) -> Vec<&'a EnhancedPid> {
    let Some(spec) = spec else { return vec![] };
    spec.enhanced_pids
        .iter()
        .filter(|epid| epid.module.eq_ignore_ascii_case(&module.0))
        .collect()
}

/// Find the service ID for an enhanced PID (0x21 for Honda/Toyota, 0x22 default).
fn find_service_id(spec: Option<&VehicleSpec>, did: u16, module: &ModuleId) -> u8 {
    if let Some(epid) = find_enhanced_pid(spec, did, module) {
        epid.service_id
    } else {
        0x22 // Default to Mode 22
    }
}

/// Public version of find_service_id for use by Session.
pub fn find_service_id_from_spec(spec: Option<&VehicleSpec>, did: u16, module: &ModuleId) -> u8 {
    find_service_id(spec, did, module)
}

pub fn decode_enhanced_value(
    spec: Option<&VehicleSpec>,
    did: u16,
    module: &ModuleId,
    data: &[u8],
) -> Value {
    if let Some(epid) = find_enhanced_pid(spec, did, module) {
        decode_with_formula(epid, data)
    } else {
        Value::Raw(data.to_vec())
    }
}

/// Find an EnhancedPid definition in the spec by DID and module.
fn find_enhanced_pid<'a>(
    spec: Option<&'a VehicleSpec>,
    did: u16,
    module: &ModuleId,
) -> Option<&'a EnhancedPid> {
    let spec = spec?;
    spec.enhanced_pids
        .iter()
        .find(|epid| epid.did == did && epid.module.eq_ignore_ascii_case(&module.0))
}

/// Public version of find_enhanced_pid for use by Session.
pub fn find_enhanced_pid_from_spec<'a>(
    spec: Option<&'a VehicleSpec>,
    did: u16,
    module: &ModuleId,
) -> Option<&'a EnhancedPid> {
    find_enhanced_pid(spec, did, module)
}

/// Decode raw bytes using the formula from an EnhancedPid definition.
fn decode_with_formula(epid: &EnhancedPid, data: &[u8]) -> Value {
    use crate::protocol::enhanced::Formula;

    if data.is_empty() {
        return Value::Raw(data.to_vec());
    }

    let a = data[0] as f64;
    let b = if data.len() > 1 { data[1] as f64 } else { 0.0 };

    match &epid.formula {
        Formula::Linear { scale, offset } => Value::Scalar(a * scale + offset),
        Formula::TwoByte { scale, offset } => Value::Scalar((a * 256.0 + b) * scale + offset),
        Formula::Centered { center, divisor } => Value::Scalar((a * 256.0 + b - center) / divisor),
        Formula::Bitmask { bits } => {
            let raw = if data.len() >= 4 {
                u32::from_be_bytes([data[0], data[1], data[2], data[3]])
            } else if data.len() >= 2 {
                u32::from(data[0]) << 8 | u32::from(data[1])
            } else {
                u32::from(data[0])
            };
            let flags = bits
                .iter()
                .map(|(bit, name)| (name.clone(), (raw >> bit) & 1 == 1))
                .collect();
            Value::Bitfield(crate::protocol::enhanced::Bitfield { raw, flags })
        }
        Formula::Enumerated { values } => {
            let key = data[0];
            let label = values
                .iter()
                .find(|(k, _)| *k == key)
                .map(|(_, v)| v.clone())
                .unwrap_or_else(|| format!("Unknown({})", key));
            Value::State(label)
        }
        Formula::Expression(_expr) => {
            // Expression parsing is complex -- fall back to raw for now
            Value::Raw(data.to_vec())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::enhanced::Formula;

    #[test]
    fn test_decode_linear_formula() {
        let epid = EnhancedPid {
            service_id: 0x22,
            did: 0x1234,
            name: "Test".into(),
            unit: "\u{00B0}C".into(),
            formula: Formula::Linear {
                scale: 1.0,
                offset: -40.0,
            },
            bytes: 1,
            module: "ecm".into(),
            value_type: crate::protocol::pid::ValueType::Scalar,
            confidence: crate::protocol::enhanced::Confidence::Verified,
            command_suffix: None,
        };
        let data = [0x7E]; // 126 - 40 = 86
        let value = decode_with_formula(&epid, &data);
        assert_eq!(value.as_f64().unwrap(), 86.0);
    }

    #[test]
    fn test_decode_two_byte_formula() {
        let epid = EnhancedPid {
            service_id: 0x22,
            did: 0x1170,
            name: "Fuel Rail Pressure".into(),
            unit: "kPa".into(),
            formula: Formula::TwoByte {
                scale: 10.0,
                offset: 0.0,
            },
            bytes: 2,
            module: "ecm".into(),
            value_type: crate::protocol::pid::ValueType::Scalar,
            confidence: crate::protocol::enhanced::Confidence::Community,
            command_suffix: None,
        };
        let data = [0x0C, 0x80]; // (12*256 + 128) * 10 = 32000
        let value = decode_with_formula(&epid, &data);
        assert_eq!(value.as_f64().unwrap(), 32000.0);
    }

    #[test]
    fn test_decode_centered_formula() {
        let epid = EnhancedPid {
            service_id: 0x22,
            did: 0x162F,
            name: "Balance Rate".into(),
            unit: "mm3".into(),
            formula: Formula::Centered {
                center: 32768.0,
                divisor: 64.0,
            },
            bytes: 2,
            module: "ecm".into(),
            value_type: crate::protocol::pid::ValueType::Scalar,
            confidence: crate::protocol::enhanced::Confidence::Community,
            command_suffix: None,
        };
        let data = [0x80, 0x00]; // (32768 - 32768) / 64 = 0.0
        let value = decode_with_formula(&epid, &data);
        assert!((value.as_f64().unwrap()).abs() < 0.01);
    }

    #[test]
    fn test_decode_enumerated_formula() {
        let epid = EnhancedPid {
            service_id: 0x22,
            did: 0x1948,
            name: "Current Gear".into(),
            unit: "gear".into(),
            formula: Formula::Enumerated {
                values: vec![
                    (0, "P/N".into()),
                    (1, "1st".into()),
                    (2, "2nd".into()),
                    (3, "3rd".into()),
                ],
            },
            bytes: 1,
            module: "tcm".into(),
            value_type: crate::protocol::pid::ValueType::State,
            confidence: crate::protocol::enhanced::Confidence::Inferred,
            command_suffix: None,
        };
        let data = [0x02]; // 2nd gear
        let value = decode_with_formula(&epid, &data);
        match value {
            Value::State(s) => assert_eq!(s, "2nd"),
            _ => panic!("expected State"),
        }
    }

    fn make_test_spec_with_enhanced_pids() -> VehicleSpec {
        use crate::vehicle::{CommunicationSpec, EngineSpec, SpecIdentity};

        VehicleSpec {
            spec_version: Some("1.0".into()),
            identity: SpecIdentity {
                name: "Test".into(),
                model_years: (2020, 2020),
                makes: vec![],
                models: vec![],
                engine: EngineSpec {
                    code: "T".into(),
                    displacement_l: 2.0,
                    cylinders: 4,
                    layout: "I4".into(),
                    aspiration: "NA".into(),
                    fuel_type: "Gas".into(),
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
            thresholds: None,
            polling_groups: vec![],
            diagnostic_rules: vec![],
            known_issues: vec![],
            dtc_library: None,
            enhanced_pids: vec![
                EnhancedPid {
                    service_id: 0x22,
                    did: 0x1170,
                    name: "Fuel Rail Pressure".into(),
                    unit: "kPa".into(),
                    formula: Formula::TwoByte {
                        scale: 10.0,
                        offset: 0.0,
                    },
                    bytes: 2,
                    module: "ecm".into(),
                    value_type: crate::protocol::pid::ValueType::Scalar,
                    confidence: crate::protocol::enhanced::Confidence::Verified,
                    command_suffix: None,
                },
                EnhancedPid {
                    service_id: 0x21,
                    did: 0x0544,
                    name: "FICM Voltage".into(),
                    unit: "V".into(),
                    formula: Formula::TwoByte {
                        scale: 0.0039,
                        offset: 0.0,
                    },
                    bytes: 2,
                    module: "ficm".into(),
                    value_type: crate::protocol::pid::ValueType::Scalar,
                    confidence: crate::protocol::enhanced::Confidence::Verified,
                    command_suffix: None,
                },
                EnhancedPid {
                    service_id: 0x22,
                    did: 0x162F,
                    name: "Balance Rate".into(),
                    unit: "mm3".into(),
                    formula: Formula::Centered {
                        center: 32768.0,
                        divisor: 64.0,
                    },
                    bytes: 2,
                    module: "ecm".into(),
                    value_type: crate::protocol::pid::ValueType::Scalar,
                    confidence: crate::protocol::enhanced::Confidence::Community,
                    command_suffix: None,
                },
            ],
        }
    }

    #[test]
    fn test_list_module_pids_returns_matching() {
        let spec = make_test_spec_with_enhanced_pids();
        let ecm = ModuleId::new("ecm");
        let pids = list_module_pids(Some(&spec), &ecm);
        assert_eq!(pids.len(), 2, "ECM should have 2 enhanced PIDs");
        assert!(pids.iter().any(|p| p.did == 0x1170));
        assert!(pids.iter().any(|p| p.did == 0x162F));
    }

    #[test]
    fn test_list_module_pids_case_insensitive() {
        let spec = make_test_spec_with_enhanced_pids();
        let ficm = ModuleId::new("FICM");
        let pids = list_module_pids(Some(&spec), &ficm);
        assert_eq!(pids.len(), 1);
        assert_eq!(pids[0].did, 0x0544);
    }

    #[test]
    fn test_list_module_pids_no_spec() {
        let ecm = ModuleId::new("ecm");
        let pids = list_module_pids(None, &ecm);
        assert!(pids.is_empty());
    }

    #[test]
    fn test_list_module_pids_unknown_module() {
        let spec = make_test_spec_with_enhanced_pids();
        let unknown = ModuleId::new("abs");
        let pids = list_module_pids(Some(&spec), &unknown);
        assert!(pids.is_empty());
    }

    #[test]
    fn test_find_enhanced_pid_by_did_and_module() {
        let spec = make_test_spec_with_enhanced_pids();
        let ecm = ModuleId::new("ecm");
        let found = find_enhanced_pid(Some(&spec), 0x1170, &ecm);
        assert!(found.is_some());
        assert_eq!(found.unwrap().name, "Fuel Rail Pressure");
    }

    #[test]
    fn test_find_enhanced_pid_wrong_module() {
        let spec = make_test_spec_with_enhanced_pids();
        let tcm = ModuleId::new("tcm");
        let found = find_enhanced_pid(Some(&spec), 0x1170, &tcm);
        assert!(found.is_none());
    }

    #[test]
    fn test_find_service_id_from_spec_honda_style() {
        let spec = make_test_spec_with_enhanced_pids();
        let ficm = ModuleId::new("ficm");
        // FICM PID has service_id 0x21
        let sid = find_service_id_from_spec(Some(&spec), 0x0544, &ficm);
        assert_eq!(sid, 0x21);
    }

    #[test]
    fn test_find_service_id_defaults_to_0x22() {
        let spec = make_test_spec_with_enhanced_pids();
        let ecm = ModuleId::new("ecm");
        // Unknown DID falls back to 0x22
        let sid = find_service_id_from_spec(Some(&spec), 0xFFFF, &ecm);
        assert_eq!(sid, 0x22);
    }

    #[test]
    fn test_find_enhanced_pid_exposes_command_suffix() {
        let mut spec = make_test_spec_with_enhanced_pids();
        spec.enhanced_pids[2].command_suffix = Some(vec![0x01]);
        let ecm = ModuleId::new("ecm");

        let pid = find_enhanced_pid_from_spec(Some(&spec), 0x162F, &ecm)
            .expect("balance rate PID should exist");

        assert_eq!(pid.command_suffix.as_deref(), Some(&[0x01][..]));
    }

    #[test]
    fn test_decode_enhanced_value_uses_formula() {
        let spec = make_test_spec_with_enhanced_pids();
        let module = ModuleId::new("ecm");
        let value = decode_enhanced_value(Some(&spec), 0x162F, &module, &[0x80, 0x00]);
        assert!(matches!(value, Value::Scalar(_)));
        assert!((value.as_f64().unwrap()).abs() < 0.01);
    }
}
