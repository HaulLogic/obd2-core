//! Repeated polling regression harness.
//!
//! This is a CI-friendly smoke check for session-owned polling. It exercises
//! the same `execute_poll_cycle()` path the application uses, but runs against
//! `MockAdapter` so it does not require hardware.

use std::time::{Duration, Instant};

use tokio::sync::mpsc;

use obd2_core::adapter::mock::MockAdapter;
use obd2_core::protocol::pid::Pid;
use obd2_core::session::poller::{execute_poll_cycle, PollConfig, PollEvent};
use obd2_core::session::Session;

const DEFAULT_ITERATIONS: usize = 500;
const WARMUP_ITERATIONS: usize = 10;
const MAX_MICROS_PER_CYCLE: u64 = 2_000;

fn iterations() -> usize {
    std::env::var("OBD2_CORE_POLLING_HARNESS_ITERS")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(DEFAULT_ITERATIONS)
}

fn budget_for(iterations: usize) -> Duration {
    Duration::from_micros((iterations as u64) * MAX_MICROS_PER_CYCLE)
}

#[tokio::test]
async fn repeated_pid_polling_harness() {
    let iterations = iterations();
    let config = PollConfig::new(vec![
        Pid::ENGINE_RPM,
        Pid::COOLANT_TEMP,
        Pid::VEHICLE_SPEED,
    ])
    .with_voltage(false);

    let expected_readings = iterations * config.pids.len();
    let (tx, mut rx) = mpsc::channel::<PollEvent>(expected_readings + 16);

    let adapter = MockAdapter::new();
    let mut session = Session::new(adapter);
    session.initialize().await.unwrap();

    for _ in 0..WARMUP_ITERATIONS {
        execute_poll_cycle(&mut session, &config, &tx, None).await;
    }
    while rx.try_recv().is_ok() {}

    let started = Instant::now();
    for _ in 0..iterations {
        execute_poll_cycle(&mut session, &config, &tx, None).await;
    }
    let elapsed = started.elapsed();

    let mut reading_count = 0usize;
    let mut alert_count = 0usize;
    let mut voltage_count = 0usize;
    let mut error_count = 0usize;

    while let Ok(event) = rx.try_recv() {
        match event {
            PollEvent::Reading { .. } => reading_count += 1,
            PollEvent::Alert(_) => alert_count += 1,
            PollEvent::Voltage(_) => voltage_count += 1,
            PollEvent::Error { .. } => error_count += 1,
            PollEvent::EnhancedReading { .. } | PollEvent::RuleFired { .. } => {}
            other => panic!("unexpected poll event in harness: {:?}", other),
        }
    }

    assert_eq!(
        reading_count, expected_readings,
        "expected {} readings across {} cycles, got {}",
        expected_readings, iterations, reading_count
    );
    assert_eq!(alert_count, 0, "mock-backed repeated polling should not alert");
    assert_eq!(voltage_count, 0, "voltage reads are disabled for this harness");
    assert_eq!(error_count, 0, "mock-backed repeated polling should not error");

    let budget = budget_for(iterations);
    eprintln!(
        "polling harness: {} cycles, {} readings, {:?} elapsed, budget {:?}",
        iterations,
        reading_count,
        elapsed,
        budget
    );

    assert!(
        elapsed <= budget,
        "repeated PID polling regressed: {:?} elapsed for {} cycles (budget {:?})",
        elapsed,
        iterations,
        budget
    );
}
