//! YAML vehicle spec loading.

use std::path::Path;

use super::VehicleSpec;
use crate::error::Obd2Error;

/// Parse a VehicleSpec from a YAML string.
pub fn load_spec_from_str(yaml: &str) -> Result<VehicleSpec, Obd2Error> {
    serde_yaml::from_str(yaml).map_err(|e| Obd2Error::SpecParse(e.to_string()))
}

/// Load a VehicleSpec from a YAML file path.
pub fn load_spec_from_file(path: &Path) -> Result<VehicleSpec, Obd2Error> {
    let content = std::fs::read_to_string(path)
        .map_err(|e| Obd2Error::SpecParse(format!("failed to read {}: {}", path.display(), e)))?;
    load_spec_from_str(&content)
}
