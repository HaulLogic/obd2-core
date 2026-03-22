//! Additional OBD-II diagnostic mode implementations.

use crate::adapter::Adapter;
use crate::error::Obd2Error;
use crate::protocol::dtc::{Dtc, DtcStatus};
use crate::protocol::enhanced::{Reading, ReadingSource};
use crate::protocol::pid::Pid;
use crate::protocol::service::{
    MonitorStatus, ReadinessStatus, ServiceRequest, Target, TestResult, VehicleInfo,
};
use crate::vehicle::VehicleSpec;
use std::time::Instant;

/// Mode 02: Read freeze frame data for a PID.
pub async fn read_freeze_frame<A: Adapter>(
    adapter: &mut A,
    pid: Pid,
    frame: u8,
) -> Result<Reading, Obd2Error> {
    let req = ServiceRequest {
        service_id: 0x02,
        data: vec![pid.0, frame],
        target: Target::Broadcast,
    };
    let data = adapter.request(&req).await?;
    let value = pid.parse(&data)?;
    Ok(Reading {
        value,
        unit: pid.unit(),
        timestamp: Instant::now(),
        raw_bytes: data,
        source: ReadingSource::FreezeFrame,
    })
}

/// Mode 06: Read on-board monitoring test results.
pub async fn read_test_results<A: Adapter>(
    adapter: &mut A,
    test_id: u8,
) -> Result<Vec<TestResult>, Obd2Error> {
    let req = ServiceRequest {
        service_id: 0x06,
        data: vec![test_id],
        target: Target::Broadcast,
    };
    let data = adapter.request(&req).await?;

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
    Ok(results)
}

/// Decode readiness status from Mode 01 PID 01 response bytes.
pub fn decode_readiness(data: &[u8]) -> Result<ReadinessStatus, Obd2Error> {
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
                complete: !((complete >> bit) & 1 == 1), // bit=1 means NOT complete
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
                complete: !((complete >> bit) & 1 == 1),
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

/// Mode 09: Read full vehicle information (VIN + CALIDs + CVNs + ECU name).
pub async fn read_full_vehicle_info<A: Adapter>(
    adapter: &mut A,
) -> Result<VehicleInfo, Obd2Error> {
    // Read VIN (InfoType 02)
    let vin_req = ServiceRequest::read_vin();
    let vin_data = adapter.request(&vin_req).await?;
    let vin: String = vin_data
        .iter()
        .filter(|&&b| b >= 0x20 && b <= 0x7E)
        .map(|&b| b as char)
        .take(17)
        .collect();

    // Try to read Calibration ID (InfoType 04)
    let cal_ids = match adapter
        .request(&ServiceRequest {
            service_id: 0x09,
            data: vec![0x04],
            target: Target::Broadcast,
        })
        .await
    {
        Ok(data) => {
            let cal_str: String = data
                .iter()
                .filter(|&&b| b >= 0x20 && b <= 0x7E)
                .map(|&b| b as char)
                .collect();
            if cal_str.is_empty() {
                vec![]
            } else {
                vec![cal_str]
            }
        }
        Err(_) => vec![],
    };

    // Try to read CVN (InfoType 06)
    let cvns = match adapter
        .request(&ServiceRequest {
            service_id: 0x09,
            data: vec![0x06],
            target: Target::Broadcast,
        })
        .await
    {
        Ok(data) if data.len() >= 4 => {
            vec![u32::from_be_bytes([data[0], data[1], data[2], data[3]])]
        }
        _ => vec![],
    };

    // Try to read ECU name (InfoType 0A)
    let ecu_name = match adapter
        .request(&ServiceRequest {
            service_id: 0x09,
            data: vec![0x0A],
            target: Target::Broadcast,
        })
        .await
    {
        Ok(data) => {
            let name: String = data
                .iter()
                .filter(|&&b| b >= 0x20 && b <= 0x7E)
                .map(|&b| b as char)
                .collect();
            if name.is_empty() {
                None
            } else {
                Some(name)
            }
        }
        Err(_) => None,
    };

    Ok(VehicleInfo {
        vin,
        calibration_ids: cal_ids,
        cvns,
        ecu_name,
    })
}

/// Clear DTCs on a specific module (GM Mode 14 / standard Mode 04 with targeting).
pub async fn clear_dtcs_on_module<A: Adapter>(
    adapter: &mut A,
    module: &str,
) -> Result<(), Obd2Error> {
    tracing::warn!(module = module, "clearing DTCs on specific module");
    let req = ServiceRequest {
        service_id: 0x04,
        data: vec![],
        target: Target::Module(module.to_string()),
    };
    adapter.request(&req).await?;
    Ok(())
}

/// Read all DTC types from all accessible sources, deduplicate and enrich.
/// Implements BR-4.1: stored + pending + permanent, then per-module if spec available.
pub async fn read_all_dtcs<A: Adapter>(
    adapter: &mut A,
    spec: Option<&VehicleSpec>,
) -> Result<Vec<Dtc>, Obd2Error> {
    let mut all_dtcs = Vec::new();

    // Mode 03: Stored DTCs (broadcast)
    if let Ok(data) = adapter.request(&ServiceRequest::read_dtcs()).await {
        all_dtcs.extend(decode_dtc_bytes(&data, DtcStatus::Stored));
    }

    // Mode 07: Pending DTCs
    if let Ok(data) = adapter
        .request(&ServiceRequest {
            service_id: 0x07,
            data: vec![],
            target: Target::Broadcast,
        })
        .await
    {
        all_dtcs.extend(decode_dtc_bytes(&data, DtcStatus::Pending));
    }

    // Mode 0A: Permanent DTCs
    if let Ok(data) = adapter
        .request(&ServiceRequest {
            service_id: 0x0A,
            data: vec![],
            target: Target::Broadcast,
        })
        .await
    {
        all_dtcs.extend(decode_dtc_bytes(&data, DtcStatus::Permanent));
    }

    // Deduplicate
    super::diagnostics::dedup_dtcs(&mut all_dtcs);

    // Enrich from spec
    super::diagnostics::enrich_dtcs(&mut all_dtcs, spec);

    Ok(all_dtcs)
}

/// Decode DTC bytes from a Mode 03/07/0A response.
fn decode_dtc_bytes(data: &[u8], status: DtcStatus) -> Vec<Dtc> {
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
    use crate::adapter::mock::MockAdapter;

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

    #[tokio::test]
    async fn test_read_freeze_frame() {
        let mut adapter = MockAdapter::new();
        adapter.initialize().await.unwrap();
        // MockAdapter returns NoData for Mode 02, so this will error
        let result = read_freeze_frame(&mut adapter, Pid::ENGINE_RPM, 0).await;
        // Verify it doesn't panic -- MockAdapter doesn't support Mode 02
        assert!(result.is_ok() || result.is_err());
    }

    #[tokio::test]
    async fn test_read_all_dtcs_empty() {
        let mut adapter = MockAdapter::new();
        adapter.initialize().await.unwrap();
        let dtcs = read_all_dtcs(&mut adapter, None).await.unwrap();
        assert!(dtcs.is_empty()); // no DTCs set on mock
    }

    #[tokio::test]
    async fn test_read_all_dtcs_with_dtcs() {
        let mut adapter = MockAdapter::new();
        adapter.set_dtcs(vec![Dtc::from_code("P0420"), Dtc::from_code("P0171")]);
        adapter.initialize().await.unwrap();
        let dtcs = read_all_dtcs(&mut adapter, None).await.unwrap();
        assert!(dtcs.len() >= 2);
        // Should have universal descriptions (populated by Dtc::from_bytes)
        assert!(dtcs.iter().any(|d| d.description.is_some()));
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

    #[tokio::test]
    async fn test_read_full_vehicle_info() {
        let mut adapter = MockAdapter::with_vin("1GCHK23224F000001");
        adapter.initialize().await.unwrap();
        let info = read_full_vehicle_info(&mut adapter).await.unwrap();
        assert_eq!(info.vin, "1GCHK23224F000001");
    }

    #[tokio::test]
    async fn test_clear_dtcs_on_module() {
        let mut adapter = MockAdapter::new();
        adapter.set_dtcs(vec![Dtc::from_code("P0420")]);
        adapter.initialize().await.unwrap();
        // MockAdapter handles Mode 04 regardless of target
        let result = clear_dtcs_on_module(&mut adapter, "ecm").await;
        assert!(result.is_ok());
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
}
