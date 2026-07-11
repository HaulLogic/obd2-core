//! Synthetic golden fixtures (no VIN, no partner captures).
//!
//! Path: repo `raw-captures/fixtures/synthetic/`.

use obd2_core::protocol::j1939::{decode_ccvs, decode_eec1, decode_et1, Pgn};
use serde::Deserialize;
use std::path::PathBuf;

#[derive(Debug, Deserialize)]
struct Eec1Expected {
    engine_rpm: f64,
    actual_torque_pct: f64,
    driver_demand_torque_pct: f64,
}

#[derive(Debug, Deserialize)]
struct Eec1Golden {
    pgn: u32,
    payload_hex: String,
    expected: Eec1Expected,
}

#[derive(Debug, Deserialize)]
struct CcvsExpected {
    vehicle_speed_kmh: f64,
}

#[derive(Debug, Deserialize)]
struct CcvsGolden {
    pgn: u32,
    payload_hex: String,
    expected: CcvsExpected,
}

#[derive(Debug, Deserialize)]
struct Et1Expected {
    coolant_temp_c: f64,
    fuel_temp_c: f64,
}

#[derive(Debug, Deserialize)]
struct Et1Golden {
    pgn: u32,
    payload_hex: String,
    expected: Et1Expected,
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

fn read_json(name: &str) -> String {
    let path = fixture_path(name);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

#[test]
fn synthetic_eec1_fixture_decodes_rpm() {
    let golden: Eec1Golden = serde_json::from_str(&read_json("eec1_rpm_680.json")).unwrap();
    assert_eq!(golden.pgn, Pgn::EEC1.0);
    let eec1 = decode_eec1(&decode_hex(&golden.payload_hex)).unwrap();
    assert!((eec1.engine_rpm.unwrap() - golden.expected.engine_rpm).abs() < 0.2);
    assert!((eec1.actual_torque_pct.unwrap() - golden.expected.actual_torque_pct).abs() < 0.1);
    assert!(
        (eec1.driver_demand_torque_pct.unwrap() - golden.expected.driver_demand_torque_pct).abs()
            < 0.1
    );
}

#[test]
fn synthetic_ccvs_fixture_decodes_speed() {
    let golden: CcvsGolden = serde_json::from_str(&read_json("ccvs_speed_26.json")).unwrap();
    assert_eq!(golden.pgn, Pgn::CCVS.0);
    let ccvs = decode_ccvs(&decode_hex(&golden.payload_hex)).unwrap();
    assert!((ccvs.vehicle_speed.unwrap() - golden.expected.vehicle_speed_kmh).abs() < 0.1);
}

#[test]
fn synthetic_et1_fixture_decodes_temps() {
    let golden: Et1Golden = serde_json::from_str(&read_json("et1_coolant_50.json")).unwrap();
    assert_eq!(golden.pgn, Pgn::ET1.0);
    let et1 = decode_et1(&decode_hex(&golden.payload_hex)).unwrap();
    assert!((et1.coolant_temp.unwrap() - golden.expected.coolant_temp_c).abs() < 0.1);
    assert!((et1.fuel_temp.unwrap() - golden.expected.fuel_temp_c).abs() < 0.1);
}
