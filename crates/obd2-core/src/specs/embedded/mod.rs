//! Embedded vehicle specifications compiled into the binary.

use crate::vehicle::loader::load_spec_from_str;
use crate::vehicle::VehicleSpec;

const DURAMAX_LLY_YAML: &str = include_str!("chevy_duramax_2004_turbo.yaml");

/// Load all embedded vehicle specs.
pub fn load_embedded_specs() -> Vec<VehicleSpec> {
    let mut specs = Vec::new();
    if let Ok(spec) = load_spec_from_str(DURAMAX_LLY_YAML) {
        specs.push(spec);
    }
    specs
}
