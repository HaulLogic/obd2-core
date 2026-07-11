//! Synthetic golden fixtures (no VIN, no partner captures).
//!
//! Path: repo `raw-captures/fixtures/synthetic/`.

use obd2_core::protocol::j1939::{decode_eec1, Pgn};
use serde::Deserialize;
use std::path::PathBuf;

#[derive(Debug, Deserialize)]
struct Expected {
    engine_rpm: f64,
    actual_torque_pct: f64,
    driver_demand_torque_pct: f64,
}

#[derive(Debug, Deserialize)]
struct Golden {
    pgn: u32,
    payload_hex: String,
    expected: Expected,
}

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../raw-captures/fixtures/synthetic")
        .join(name)
}

fn decode_hex(s: &str) -> Vec<u8> {
    assert!(s.len() % 2 == 0, "hex length");
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).expect("hex byte"))
        .collect()
}

#[test]
fn synthetic_eec1_fixture_decodes_rpm() {
    let path = fixture_path("eec1_rpm_680.json");
    let raw = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    let golden: Golden = serde_json::from_str(&raw).expect("parse golden json");

    assert_eq!(golden.pgn, Pgn::EEC1.0);
    let bytes = decode_hex(&golden.payload_hex);
    let eec1 = decode_eec1(&bytes).expect("decode EEC1");

    assert!((eec1.engine_rpm.unwrap() - golden.expected.engine_rpm).abs() < 0.2);
    assert!((eec1.actual_torque_pct.unwrap() - golden.expected.actual_torque_pct).abs() < 0.1);
    assert!(
        (eec1.driver_demand_torque_pct.unwrap() - golden.expected.driver_demand_torque_pct).abs()
            < 0.1
    );
}
