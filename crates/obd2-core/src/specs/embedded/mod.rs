//! Embedded vehicle specifications compiled into the binary.

use crate::vehicle::loader::load_spec_from_str;
use crate::vehicle::VehicleSpec;

const DURAMAX_LLY_YAML: &str = include_str!("chevy_duramax_2004_turbo.yaml");
const DURAMAX_LBZ_YAML: &str = include_str!("chevy_duramax_lbz.yaml");
const GENERIC_OBD2_YAML: &str = include_str!("generic_obd2_fallback.yaml");
const FORD_PSTRK67_YAML: &str = include_str!("ford_powerstroke_67.yaml");
const RAM_CUMMINS67_YAML: &str = include_str!("ram_cummins_67.yaml");

const EMBEDDED: &[&str] = &[
    DURAMAX_LLY_YAML,
    DURAMAX_LBZ_YAML,
    GENERIC_OBD2_YAML,
    FORD_PSTRK67_YAML,
    RAM_CUMMINS67_YAML,
];

/// Load all embedded vehicle specs.
///
/// Specs with `vin_match: null` (generic fallback) never participate in VIN
/// uniqueness conflicts — they are available via `match_vehicle` / list APIs.
pub fn load_embedded_specs() -> Vec<VehicleSpec> {
    let mut specs = Vec::new();
    for yaml in EMBEDDED {
        match load_spec_from_str(yaml) {
            Ok(spec) => specs.push(spec),
            Err(e) => {
                // Fail soft at runtime so one bad YAML does not empty the catalog;
                // unit tests assert load counts.
                eprintln!("embedded vehicle spec failed to load: {e}");
            }
        }
    }
    specs
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loads_all_embedded_family_scaffolds() {
        let specs = load_embedded_specs();
        assert!(
            specs.len() >= 5,
            "expected LLY, LBZ, generic, PowerStroke, Cummins; got {}",
            specs.len()
        );
        let codes: Vec<_> = specs
            .iter()
            .map(|s| s.identity.engine.code.as_str())
            .collect();
        for expected in ["LLY", "LBZ", "GENERIC", "PSTRK67", "CUMMINS67"] {
            assert!(
                codes.contains(&expected),
                "missing engine code {expected} in {codes:?}"
            );
        }
    }

    #[test]
    fn generic_fallback_has_no_vin_match() {
        let specs = load_embedded_specs();
        let generic = specs
            .iter()
            .find(|s| s.identity.engine.code == "GENERIC")
            .expect("generic");
        assert!(generic.identity.vin_match.is_none());
    }
}
