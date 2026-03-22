//! Enhanced PID reads, multi-module support, and bus switching.

use crate::adapter::Adapter;
use crate::error::Obd2Error;
use crate::protocol::enhanced::{EnhancedPid, Reading, ReadingSource, Value};
use crate::protocol::service::{ServiceRequest, Target};
use crate::vehicle::{BusConfig, ModuleId, VehicleSpec};
use std::time::Instant;

/// Read an enhanced PID from a specific module via the adapter.
pub async fn read_enhanced_pid<A: Adapter>(
    adapter: &mut A,
    did: u16,
    module: &ModuleId,
    spec: Option<&VehicleSpec>,
) -> Result<Reading, Obd2Error> {
    // Look up service ID from spec (default 0x22)
    let service_id = find_service_id(spec, did, module);

    let req = ServiceRequest::enhanced_read(
        service_id,
        did,
        Target::Module(module.0.clone()),
    );
    let data = adapter.request(&req).await?;

    // Try to decode using formula from spec
    let value = if let Some(epid) = find_enhanced_pid(spec, did, module) {
        decode_with_formula(epid, &data)
    } else {
        Value::Raw(data.clone())
    };

    Ok(Reading {
        value,
        // Reading.unit is &'static str; we cannot produce one from EnhancedPid's String field,
        // so default to "" until the Reading type is extended.
        unit: "",
        timestamp: Instant::now(),
        raw_bytes: data,
        source: ReadingSource::Live,
    })
}

/// Read all enhanced PIDs defined for a module in the spec.
pub async fn read_all_enhanced_for_module<A: Adapter>(
    adapter: &mut A,
    module: &ModuleId,
    spec: Option<&VehicleSpec>,
) -> Result<Vec<(EnhancedPid, Reading)>, Obd2Error> {
    let pids = list_module_pids(spec, module);
    let mut results = Vec::new();

    for epid in pids {
        match read_enhanced_pid(adapter, epid.did, module, spec).await {
            Ok(reading) => results.push((epid.clone(), reading)),
            Err(Obd2Error::NoData) => continue, // skip unsupported
            Err(Obd2Error::NegativeResponse { nrc, .. })
                if nrc == crate::error::NegativeResponse::RequestOutOfRange =>
            {
                continue
            }
            Err(e) => return Err(e),
        }
    }

    Ok(results)
}

/// List enhanced PIDs available for a module from the spec.
pub fn list_module_pids<'a>(
    _spec: Option<&'a VehicleSpec>,
    _module: &ModuleId,
) -> Vec<&'a EnhancedPid> {
    // Search spec for PIDs belonging to this module
    // For now this returns empty -- needs spec modules to have enhanced_pids populated
    // This is a placeholder until spec loading populates module PIDs
    vec![]
}

/// Find the service ID for an enhanced PID (0x21 for Honda/Toyota, 0x22 default).
fn find_service_id(spec: Option<&VehicleSpec>, did: u16, module: &ModuleId) -> u8 {
    if let Some(epid) = find_enhanced_pid(spec, did, module) {
        epid.service_id
    } else {
        0x22 // Default to Mode 22
    }
}

/// Find an EnhancedPid definition in the spec by DID and module.
fn find_enhanced_pid<'a>(
    _spec: Option<&'a VehicleSpec>,
    _did: u16,
    _module: &ModuleId,
) -> Option<&'a EnhancedPid> {
    // Search spec modules for matching DID
    // Placeholder -- needs spec data model to include enhanced PIDs per module
    None
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
        Formula::TwoByte { scale, offset } => {
            Value::Scalar((a * 256.0 + b) * scale + offset)
        }
        Formula::Centered { center, divisor } => {
            Value::Scalar((a * 256.0 + b - center) / divisor)
        }
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

/// Get available buses from the spec.
pub fn available_buses(spec: Option<&VehicleSpec>) -> Vec<&BusConfig> {
    spec.map(|s| s.communication.buses.iter().collect())
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::mock::MockAdapter;
    use crate::protocol::enhanced::Formula;

    #[tokio::test]
    async fn test_read_enhanced_default_service_id() {
        let mut adapter = MockAdapter::new();
        adapter.initialize().await.unwrap();
        let module = ModuleId::new("ecm");
        let reading = read_enhanced_pid(&mut adapter, 0x162F, &module, None)
            .await
            .unwrap();
        assert!(matches!(reading.value, Value::Raw(_)));
    }

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

    #[test]
    fn test_available_buses_no_spec() {
        let buses = available_buses(None);
        assert!(buses.is_empty());
    }
}
