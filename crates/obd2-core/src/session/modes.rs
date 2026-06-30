//! Additional OBD-II diagnostic mode implementations.

use crate::error::Obd2Error;
use crate::protocol::dtc::{Dtc, DtcStatus};
use crate::protocol::service::{MonitorStatus, ReadinessStatus, TestResult};
/// Decode Mode 06 on-board monitoring test results.
pub(crate) fn decode_test_results(data: &[u8]) -> Vec<TestResult> {
    // Mode 06 response: [TID, COMP_ID, test_val_hi, test_val_lo, min_hi, min_lo, max_hi, max_lo]
    let mut results = Vec::new();
    let mut i = 0;
    while i + 7 < data.len() {
        let tid = data[i];
        let _comp_id = data[i + 1];
        let value = u16::from_be_bytes([data[i + 2], data[i + 3]]) as f64;
        let min = u16::from_be_bytes([data[i + 4], data[i + 5]]) as f64;
        let max = u16::from_be_bytes([data[i + 6], data[i + 7]]) as f64;

        let passed = value >= min && value <= max;

        results.push(TestResult {
            test_id: tid,
            name: format!("Test {:#04X}", tid),
            value,
            min: Some(min),
            max: Some(max),
            passed,
            unit: String::new(),
        });
        i += 8;
    }
    results
}

/// Decode readiness status from Mode 01 PID 01 response bytes.
pub(crate) fn decode_readiness(data: &[u8]) -> Result<ReadinessStatus, Obd2Error> {
    if data.len() < 4 {
        return Err(Obd2Error::ParseError(format!(
            "readiness status needs 4 bytes, got {}",
            data.len()
        )));
    }

    let mil_on = (data[0] & 0x80) != 0;
    let dtc_count = data[0] & 0x7F;
    let compression_ignition = (data[1] & 0x08) != 0;

    let mut monitors = Vec::new();

    if compression_ignition {
        // Diesel monitors (bytes C and D)
        let supported = data[2];
        let complete = data[3];
        let diesel_monitors = [
            (0, "EGR/VVT System"),
            (1, "Boost Pressure"),
            (2, "NOx/SCR Monitor"),
            (3, "NMHC Catalyst"),
            (4, "Misfire"),
            (5, "Fuel System"),
            (6, "Comprehensive Component"),
        ];
        for (bit, name) in &diesel_monitors {
            monitors.push(MonitorStatus {
                name: name.to_string(),
                supported: (supported >> bit) & 1 == 1,
                complete: (complete >> bit) & 1 != 1, // bit=1 means NOT complete
            });
        }
    } else {
        // Gasoline monitors
        let supported = data[2];
        let complete = data[3];
        let gas_monitors = [
            (0, "Catalyst"),
            (1, "Heated Catalyst"),
            (2, "EVAP System"),
            (3, "Secondary Air"),
            (4, "A/C Refrigerant"),
            (5, "O2 Sensor"),
            (6, "O2 Sensor Heater"),
            (7, "EGR System"),
        ];
        for (bit, name) in &gas_monitors {
            monitors.push(MonitorStatus {
                name: name.to_string(),
                supported: (supported >> bit) & 1 == 1,
                complete: (complete >> bit) & 1 != 1,
            });
        }
    }

    Ok(ReadinessStatus {
        mil_on,
        dtc_count,
        compression_ignition,
        monitors,
    })
}

/// Decode DTC bytes from a Mode 03/07/0A response.
pub(crate) fn decode_dtc_bytes(data: &[u8], status: DtcStatus) -> Vec<Dtc> {
    let mut dtcs = Vec::new();
    let mut i = 0;
    while i + 1 < data.len() {
        if data[i] == 0 && data[i + 1] == 0 {
            i += 2;
            continue;
        }
        let mut dtc = Dtc::from_bytes(data[i], data[i + 1]);
        dtc.status = status;
        dtcs.push(dtc);
        i += 2;
    }
    dtcs
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_decode_readiness_diesel() {
        // MIL off, 0 DTCs, compression ignition, some monitors complete
        let data = [0x00, 0x08, 0x37, 0x00]; // CI=true, monitors 0-5 supported, all complete
        let status = decode_readiness(&data).unwrap();
        assert!(!status.mil_on);
        assert_eq!(status.dtc_count, 0);
        assert!(status.compression_ignition);
        assert!(!status.monitors.is_empty());
    }

    #[test]
    fn test_decode_readiness_gasoline() {
        let data = [0x82, 0x07, 0xFF, 0x00]; // MIL on, 2 DTCs, gasoline, all complete
        let status = decode_readiness(&data).unwrap();
        assert!(status.mil_on);
        assert_eq!(status.dtc_count, 2);
        assert!(!status.compression_ignition);
    }

    #[test]
    fn test_decode_readiness_mil_on() {
        let data = [0x81, 0x00, 0x00, 0x00]; // MIL on, 1 DTC
        let status = decode_readiness(&data).unwrap();
        assert!(status.mil_on);
        assert_eq!(status.dtc_count, 1);
    }

    #[test]
    fn test_decode_readiness_insufficient_bytes() {
        let data = [0x00, 0x00]; // only 2 bytes
        assert!(decode_readiness(&data).is_err());
    }

    #[test]
    fn test_decode_dtc_bytes() {
        let data = [0x04, 0x20, 0x01, 0x71]; // P0420, P0171
        let dtcs = decode_dtc_bytes(&data, DtcStatus::Stored);
        assert_eq!(dtcs.len(), 2);
        assert_eq!(dtcs[0].code, "P0420");
        assert_eq!(dtcs[1].code, "P0171");
        assert_eq!(dtcs[0].status, DtcStatus::Stored);
    }

    #[test]
    fn test_decode_dtc_bytes_with_padding() {
        let data = [0x04, 0x20, 0x00, 0x00, 0x01, 0x71]; // P0420, padding, P0171
        let dtcs = decode_dtc_bytes(&data, DtcStatus::Pending);
        assert_eq!(dtcs.len(), 2);
        assert_eq!(dtcs[1].status, DtcStatus::Pending);
    }

    #[test]
    fn test_decode_readiness_gasoline_monitors_count() {
        let data = [0x00, 0x00, 0xFF, 0x00]; // All 8 gasoline monitors supported and complete
        let status = decode_readiness(&data).unwrap();
        assert_eq!(status.monitors.len(), 8);
        // All supported and complete
        for m in &status.monitors {
            assert!(m.supported);
            assert!(m.complete);
        }
    }

    #[test]
    fn test_decode_readiness_diesel_monitors_count() {
        let data = [0x00, 0x08, 0x7F, 0x00]; // CI=true, 7 diesel monitors supported, all complete
        let status = decode_readiness(&data).unwrap();
        assert_eq!(status.monitors.len(), 7);
    }

    #[test]
    fn test_decode_readiness_incomplete_monitors() {
        // Gasoline: supported=0x0F (bits 0-3), not-complete=0x03 (bits 0-1 incomplete)
        let data = [0x00, 0x00, 0x0F, 0x03];
        let status = decode_readiness(&data).unwrap();
        // Bits 0 and 1 are supported but NOT complete (complete byte bit=1 means not-complete)
        let catalyst = &status.monitors[0]; // bit 0
        assert!(catalyst.supported);
        assert!(!catalyst.complete); // bit 0 set in complete byte = not complete

        let heated_cat = &status.monitors[1]; // bit 1
        assert!(heated_cat.supported);
        assert!(!heated_cat.complete);

        let evap = &status.monitors[2]; // bit 2
        assert!(evap.supported);
        assert!(evap.complete); // bit 2 NOT set = complete
    }

    #[test]
    fn test_decode_dtc_bytes_empty() {
        let data: [u8; 0] = [];
        let dtcs = decode_dtc_bytes(&data, DtcStatus::Stored);
        assert!(dtcs.is_empty());
    }

    #[test]
    fn test_decode_dtc_bytes_all_padding() {
        let data = [0x00, 0x00, 0x00, 0x00];
        let dtcs = decode_dtc_bytes(&data, DtcStatus::Stored);
        assert!(dtcs.is_empty());
    }

    #[test]
    fn test_decode_test_results() {
        let data = [0x01, 0x10, 0x00, 0x64, 0x00, 0x32, 0x00, 0x96];
        let results = decode_test_results(&data);
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].test_id, 0x01);
        assert_eq!(results[0].value, 100.0);
        assert_eq!(results[0].min, Some(50.0));
        assert_eq!(results[0].max, Some(150.0));
        assert!(results[0].passed);
    }

    #[test]
    fn test_o2_sensor_location_display() {
        use crate::protocol::service::O2SensorLocation;
        assert_eq!(format!("{}", O2SensorLocation::Bank1Sensor1), "B1S1");
        assert_eq!(format!("{}", O2SensorLocation::Bank2Sensor2), "B2S2");
    }

    #[test]
    fn test_o2_sensor_location_from_byte() {
        use crate::protocol::service::O2SensorLocation;
        assert_eq!(
            O2SensorLocation::from_byte(0x01),
            Some(O2SensorLocation::Bank1Sensor1)
        );
        assert_eq!(
            O2SensorLocation::from_byte(0x08),
            Some(O2SensorLocation::Bank4Sensor2)
        );
        assert_eq!(O2SensorLocation::from_byte(0x00), None);
        assert_eq!(O2SensorLocation::from_byte(0x09), None);
    }

    #[test]
    fn test_o2_test_info_voltage_tid() {
        let (name, unit, convert) = crate::protocol::service::o2_test_info(0x01);
        assert_eq!(name, "Rich-to-Lean Threshold Voltage");
        assert_eq!(unit, "V");
        assert!((convert(90) - 0.45).abs() < 0.001);
    }

    #[test]
    fn test_o2_test_info_time_tid() {
        let (name, unit, convert) = crate::protocol::service::o2_test_info(0x05);
        assert_eq!(name, "Rich-to-Lean Switch Time");
        assert_eq!(unit, "s");
        assert!((convert(250) - 1.0).abs() < 0.001);
    }
}
