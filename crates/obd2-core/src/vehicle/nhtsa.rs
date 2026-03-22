//! NHTSA vPIC API client for online VIN decoding.
//!
//! Requires the `nhtsa` feature flag to be enabled.
//! Falls back gracefully if the network is unavailable (BR-14.2).

use crate::error::Obd2Error;
use serde::Deserialize;

const NHTSA_API_URL: &str = "https://vpic.nhtsa.dot.gov/api/vehicles/decodevin";
const TIMEOUT_SECS: u64 = 5;

/// Vehicle information decoded from the NHTSA vPIC database.
#[derive(Debug, Clone, Default)]
pub struct NhtsaVehicle {
    pub make: Option<String>,
    pub model: Option<String>,
    pub year: Option<i32>,
    pub trim: Option<String>,
    pub body_class: Option<String>,
    pub drive_type: Option<String>,
    pub fuel_type: Option<String>,
    pub transmission_type: Option<String>,
    pub engine_model: Option<String>,
    pub displacement_l: Option<f64>,
    pub cylinders: Option<i32>,
}

/// Look up a VIN in the NHTSA vPIC database.
///
/// Returns `Ok(Some(vehicle))` on success, `Ok(None)` if the API has no useful data,
/// or `Err` on network/parse failure. Timeout is 5 seconds (BR-14.2).
pub async fn decode_vin(vin: &str) -> Result<Option<NhtsaVehicle>, Obd2Error> {
    let url = format!("{}/{}?format=json", NHTSA_API_URL, vin);

    tracing::debug!(target: "obd2::nhtsa", "Requesting VIN decode: {}", vin);

    let client = reqwest::Client::builder()
        .timeout(std::time::Duration::from_secs(TIMEOUT_SECS))
        .build()
        .map_err(|e| Obd2Error::Transport(format!("HTTP client error: {}", e)))?;

    let response = client
        .get(&url)
        .send()
        .await
        .map_err(|e| {
            tracing::warn!(target: "obd2::nhtsa", "NHTSA request failed for {}: {}", vin, e);
            Obd2Error::Transport(format!("NHTSA request failed: {}", e))
        })?;

    if !response.status().is_success() {
        tracing::warn!(target: "obd2::nhtsa", "NHTSA returned status {} for VIN {}", response.status(), vin);
        return Err(Obd2Error::Transport(format!(
            "NHTSA returned status {}",
            response.status()
        )));
    }

    let body = response
        .text()
        .await
        .map_err(|e| Obd2Error::Transport(format!("NHTSA response read failed: {}", e)))?;

    parse_nhtsa_response(&body)
}

/// Parse NHTSA JSON response into NhtsaVehicle.
pub fn parse_nhtsa_response(json: &str) -> Result<Option<NhtsaVehicle>, Obd2Error> {
    let response: NhtsaResponse = serde_json::from_str(json)
        .map_err(|e| Obd2Error::ParseError(format!("NHTSA JSON parse error: {}", e)))?;

    let mut vehicle = NhtsaVehicle::default();

    for result in &response.results {
        let value = result
            .value
            .as_deref()
            .filter(|v| !v.is_empty() && *v != "Not Applicable");

        match result.variable_id {
            26 => vehicle.make = value.map(String::from),
            28 => vehicle.model = value.map(String::from),
            29 => vehicle.year = value.and_then(|v| v.parse().ok()),
            38 => vehicle.trim = value.map(String::from),
            5 => vehicle.body_class = value.map(String::from),
            15 => vehicle.drive_type = value.map(String::from),
            24 => vehicle.fuel_type = value.map(String::from),
            37 => vehicle.transmission_type = value.map(String::from),
            18 => vehicle.engine_model = value.map(String::from),
            13 => vehicle.displacement_l = value.and_then(|v| v.parse().ok()),
            9 => vehicle.cylinders = value.and_then(|v| v.parse().ok()),
            _ => {}
        }
    }

    // Return None if we didn't get useful data
    if vehicle.make.is_none() && vehicle.model.is_none() {
        tracing::info!(target: "obd2::nhtsa", "NHTSA returned no useful data");
        Ok(None)
    } else {
        tracing::info!(
            target: "obd2::nhtsa",
            "Decoded: {} {} {} ({:?}, {:?}L, {}cyl)",
            vehicle.year.unwrap_or(0),
            vehicle.make.as_deref().unwrap_or("?"),
            vehicle.model.as_deref().unwrap_or("?"),
            vehicle.fuel_type.as_deref().unwrap_or("?"),
            vehicle.displacement_l.unwrap_or(0.0),
            vehicle.cylinders.unwrap_or(0),
        );
        Ok(Some(vehicle))
    }
}

#[derive(Deserialize)]
struct NhtsaResponse {
    #[serde(rename = "Results")]
    results: Vec<NhtsaResult>,
}

#[derive(Deserialize)]
struct NhtsaResult {
    #[serde(rename = "VariableId")]
    variable_id: u32,
    #[serde(rename = "Value")]
    value: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_nhtsa_response() {
        let json = r#"{
            "Results": [
                {"VariableId": 26, "Value": "CHEVROLET"},
                {"VariableId": 28, "Value": "Silverado 2500 HD"},
                {"VariableId": 29, "Value": "2004"},
                {"VariableId": 9, "Value": "8"},
                {"VariableId": 13, "Value": "6.6"},
                {"VariableId": 24, "Value": "Diesel"},
                {"VariableId": 15, "Value": "4WD/4-Wheel Drive/4x4"},
                {"VariableId": 37, "Value": "Automatic"}
            ]
        }"#;

        let result = parse_nhtsa_response(json).unwrap();
        assert!(result.is_some());
        let v = result.unwrap();
        assert_eq!(v.make.as_deref(), Some("CHEVROLET"));
        assert_eq!(v.model.as_deref(), Some("Silverado 2500 HD"));
        assert_eq!(v.year, Some(2004));
        assert_eq!(v.cylinders, Some(8));
        assert_eq!(v.displacement_l, Some(6.6));
    }

    #[test]
    fn test_parse_empty_response() {
        let json = r#"{"Results": [{"VariableId": 26, "Value": ""}, {"VariableId": 28, "Value": null}]}"#;
        let result = parse_nhtsa_response(json).unwrap();
        assert!(result.is_none()); // No useful data
    }

    #[test]
    fn test_parse_not_applicable() {
        let json = r#"{"Results": [{"VariableId": 26, "Value": "Not Applicable"}]}"#;
        let result = parse_nhtsa_response(json).unwrap();
        assert!(result.is_none());
    }

    #[test]
    fn test_parse_invalid_json() {
        let result = parse_nhtsa_response("not json");
        assert!(result.is_err());
    }
}
