//! Business rule regression tests.
//!
//! These tests enforce invariants that must hold across all changes:
//! 1. Fixing one vehicle cannot break another vehicle
//! 2. Fixing one adapter type cannot break another adapter
//! 3. Fixing one protocol cannot break another protocol
//! 4. Fixing one PID's parsing cannot break another PID's parsing

use obd2_core::adapter::mock::MockAdapter;
use obd2_core::protocol::pid::Pid;
use obd2_core::session::Session;

// ── Rule 1: Vehicle isolation ─────────────────────────────────────────

#[tokio::test]
async fn rule1_sessions_with_different_vins_are_independent() {
    let mut session_a = Session::new(MockAdapter::with_vin("1GCHK23164F000001"));
    let mut session_b = Session::new(MockAdapter::with_vin("WMWRE33546T000001"));

    let profile_a = session_a.identify_vehicle().await.unwrap();
    let profile_b = session_b.identify_vehicle().await.unwrap();

    assert_eq!(profile_a.vin, "1GCHK23164F000001");
    assert_eq!(profile_b.vin, "WMWRE33546T000001");
}

#[tokio::test]
async fn rule1_pid_reads_do_not_bleed_between_sessions() {
    let mut session_a = Session::new(MockAdapter::with_vin("1GCHK23164F000001"));
    let mut session_b = Session::new(MockAdapter::with_vin("WMWRE33546T000001"));

    let rpm_a = session_a.read_pid(Pid::ENGINE_RPM).await.unwrap();
    let rpm_b = session_b.read_pid(Pid::ENGINE_RPM).await.unwrap();

    let val_a = rpm_a.value.as_f64().unwrap();
    let val_b = rpm_b.value.as_f64().unwrap();
    assert!(
        (val_a - val_b).abs() < 0.001,
        "Same mock should give same RPM"
    );
}

#[tokio::test]
async fn rule1_dtcs_do_not_bleed_between_sessions() {
    let mut adapter_a = MockAdapter::with_vin("1GCHK23164F000001");
    adapter_a.set_dtcs(vec![obd2_core::protocol::dtc::Dtc::from_code("P0420")]);
    let mut session_a = Session::new(adapter_a);
    let mut session_b = Session::new(MockAdapter::with_vin("WMWRE33546T000001"));

    let dtcs_a = session_a.read_dtcs().await.unwrap();
    let dtcs_b = session_b.read_dtcs().await.unwrap();

    assert_eq!(dtcs_a.len(), 1);
    assert_eq!(dtcs_b.len(), 0, "Session B should have no DTCs from A");
}

// ── Rule 2: Adapter type isolation ────────────────────────────────────

#[tokio::test]
async fn rule2_adapter_instances_are_independent() {
    let mut session_a = Session::new(MockAdapter::new());
    let mut session_b = Session::new(MockAdapter::new());

    let profile_a = session_a.identify_vehicle().await.unwrap();
    let profile_b = session_b.identify_vehicle().await.unwrap();
    assert_eq!(profile_a.vin, profile_b.vin);

    let rpm_a = session_a
        .read_pid(Pid::ENGINE_RPM)
        .await
        .unwrap()
        .value
        .as_f64()
        .unwrap();
    let rpm_b = session_b
        .read_pid(Pid::ENGINE_RPM)
        .await
        .unwrap()
        .value
        .as_f64()
        .unwrap();
    assert!((rpm_a - rpm_b).abs() < 0.001);
}

#[tokio::test]
async fn rule2_adapter_dtc_state_is_isolated() {
    let mut adapter_with_dtcs = MockAdapter::new();
    adapter_with_dtcs.set_dtcs(vec![
        obd2_core::protocol::dtc::Dtc::from_code("P0171"),
        obd2_core::protocol::dtc::Dtc::from_code("P0300"),
    ]);

    let mut session_dtcs = Session::new(adapter_with_dtcs);
    let mut session_clean = Session::new(MockAdapter::new());

    assert_eq!(session_dtcs.read_dtcs().await.unwrap().len(), 2);
    assert_eq!(session_clean.read_dtcs().await.unwrap().len(), 0);
}

// ── Rule 3: Protocol isolation ────────────────────────────────────────

#[tokio::test]
async fn rule3_mode01_reads_do_not_affect_mode03() {
    let mut adapter = MockAdapter::new();
    adapter.set_dtcs(vec![obd2_core::protocol::dtc::Dtc::from_code("P0420")]);
    let mut session = Session::new(adapter);

    let _ = session.read_pid(Pid::ENGINE_RPM).await.unwrap();
    let _ = session.read_pid(Pid::COOLANT_TEMP).await.unwrap();
    let _ = session.read_pid(Pid::VEHICLE_SPEED).await.unwrap();

    let dtcs = session.read_dtcs().await.unwrap();
    assert_eq!(dtcs.len(), 1);
    assert_eq!(dtcs[0].code, "P0420");
}

#[tokio::test]
async fn rule3_voltage_reads_do_not_affect_pid_reads() {
    let mut session = Session::new(MockAdapter::new());

    let rpm_before = session
        .read_pid(Pid::ENGINE_RPM)
        .await
        .unwrap()
        .value
        .as_f64()
        .unwrap();
    let _ = session.battery_voltage().await.unwrap();
    let rpm_after = session
        .read_pid(Pid::ENGINE_RPM)
        .await
        .unwrap()
        .value
        .as_f64()
        .unwrap();

    assert!((rpm_before - rpm_after).abs() < 0.001);
}

#[tokio::test]
async fn rule3_vin_read_does_not_affect_pid_reads() {
    let mut session = Session::new(MockAdapter::new());

    let rpm1 = session
        .read_pid(Pid::ENGINE_RPM)
        .await
        .unwrap()
        .value
        .as_f64()
        .unwrap();
    let _ = session.read_vin().await.unwrap();
    let rpm2 = session
        .read_pid(Pid::ENGINE_RPM)
        .await
        .unwrap()
        .value
        .as_f64()
        .unwrap();

    assert!((rpm1 - rpm2).abs() < 0.001);
}

// ── Rule 4: PID parsing isolation ─────────────────────────────────────

#[tokio::test]
async fn rule4_pid_parsing_order_does_not_matter() {
    let mut session = Session::new(MockAdapter::new());
    let _ = session.identify_vehicle().await;

    let pids = vec![
        Pid::ENGINE_RPM,
        Pid::VEHICLE_SPEED,
        Pid::COOLANT_TEMP,
        Pid::ENGINE_LOAD,
        Pid::THROTTLE_POSITION,
        Pid::INTAKE_MAP,
        Pid::MAF,
        Pid::BAROMETRIC_PRESSURE,
        Pid::FUEL_TANK_LEVEL,
        Pid::ENGINE_OIL_TEMP,
        Pid::TIMING_ADVANCE,
        Pid::AMBIENT_AIR_TEMP,
        Pid::ENGINE_FUEL_RATE,
    ];

    let mut forward_values = Vec::new();
    for &pid in &pids {
        let reading = session.read_pid(pid).await.unwrap();
        forward_values.push((pid, reading.value.as_f64().unwrap()));
    }

    let mut session2 = Session::new(MockAdapter::new());
    let _ = session2.identify_vehicle().await;
    let mut reverse_values = Vec::new();
    for &pid in pids.iter().rev() {
        let reading = session2.read_pid(pid).await.unwrap();
        reverse_values.push((pid, reading.value.as_f64().unwrap()));
    }
    reverse_values.reverse();

    for (i, ((pid_f, val_f), (pid_r, val_r))) in
        forward_values.iter().zip(reverse_values.iter()).enumerate()
    {
        assert_eq!(pid_f, pid_r, "PID mismatch at index {}", i);
        assert!(
            (val_f - val_r).abs() < 0.001,
            "PID {} value differs: forward={} reverse={}",
            pid_f.name(),
            val_f,
            val_r,
        );
    }
}

#[tokio::test]
async fn rule4_all_two_byte_pids_parse_without_error() {
    let mut session = Session::new(MockAdapter::new());

    let two_byte_pids = [
        Pid::ENGINE_RPM,
        Pid::MAF,
        Pid::RUN_TIME,
        Pid::DISTANCE_WITH_MIL,
        Pid::FUEL_RAIL_GAUGE_PRESSURE,
        Pid::DISTANCE_SINCE_CLEAR,
        Pid::CATALYST_TEMP_B1S1,
        Pid::CATALYST_TEMP_B2S1,
        Pid::CATALYST_TEMP_B1S2,
        Pid::CATALYST_TEMP_B2S2,
        Pid::CONTROL_MODULE_VOLTAGE,
        Pid::ABSOLUTE_LOAD,
        Pid::COMMANDED_EQUIV_RATIO,
        Pid::FUEL_RAIL_ABS_PRESSURE,
        Pid::ENGINE_FUEL_RATE,
        Pid::REFERENCE_TORQUE,
    ];

    for &pid in &two_byte_pids {
        let result = session.read_pid(pid).await;
        assert!(
            result.is_ok(),
            "2-byte PID {} ({:#04X}) failed: {:?}",
            pid.name(),
            pid.0,
            result.err(),
        );
    }
}

#[tokio::test]
async fn rule4_all_one_byte_pids_parse_without_error() {
    let mut session = Session::new(MockAdapter::new());

    let one_byte_pids = [
        Pid::ENGINE_LOAD,
        Pid::COOLANT_TEMP,
        Pid::SHORT_FUEL_TRIM_B1,
        Pid::LONG_FUEL_TRIM_B1,
        Pid::SHORT_FUEL_TRIM_B2,
        Pid::LONG_FUEL_TRIM_B2,
        Pid::FUEL_PRESSURE,
        Pid::INTAKE_MAP,
        Pid::VEHICLE_SPEED,
        Pid::TIMING_ADVANCE,
        Pid::INTAKE_AIR_TEMP,
        Pid::THROTTLE_POSITION,
        Pid::COMMANDED_EGR,
        Pid::FUEL_TANK_LEVEL,
        Pid::BAROMETRIC_PRESSURE,
        Pid::AMBIENT_AIR_TEMP,
        Pid::ENGINE_OIL_TEMP,
    ];

    for &pid in &one_byte_pids {
        let result = session.read_pid(pid).await;
        assert!(
            result.is_ok(),
            "1-byte PID {} ({:#04X}) failed: {:?}",
            pid.name(),
            pid.0,
            result.err(),
        );
    }
}
