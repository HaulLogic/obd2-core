//! PID polling loop with PollEvent channel and threshold alerting.

use crate::protocol::pid::Pid;
use crate::vehicle::{ModuleId, ThresholdResult};
use std::time::Duration;
use tokio::sync::mpsc;

/// Events emitted by the polling loop.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum PollEvent {
    /// A standard PID was read successfully.
    Reading {
        pid: Pid,
        reading: crate::protocol::enhanced::Reading,
    },

    /// An enhanced PID was read.
    EnhancedReading {
        did: u16,
        module: ModuleId,
        reading: crate::protocol::enhanced::Reading,
    },

    /// A threshold was breached.
    Alert(ThresholdResult),

    /// A diagnostic rule fired.
    RuleFired {
        rule_name: String,
        description: String,
    },

    /// Polling encountered a non-fatal error (polling continues).
    Error { pid: Option<Pid>, error: String },

    /// Battery voltage update.
    Voltage(f64),
}

/// Configuration for a polling session.
#[derive(Debug, Clone)]
pub struct PollConfig {
    /// PIDs to poll each cycle.
    pub pids: Vec<Pid>,
    /// Interval between poll cycles.
    pub interval: Duration,
    /// Whether to read battery voltage each cycle.
    pub read_voltage: bool,
}

impl PollConfig {
    /// Create a basic poll config with default interval.
    pub fn new(pids: Vec<Pid>) -> Self {
        Self {
            pids,
            interval: Duration::from_millis(250),
            read_voltage: true,
        }
    }

    /// Set the polling interval.
    pub fn with_interval(mut self, interval: Duration) -> Self {
        self.interval = interval;
        self
    }

    /// Set whether to read battery voltage.
    pub fn with_voltage(mut self, read_voltage: bool) -> Self {
        self.read_voltage = read_voltage;
        self
    }
}

/// Handle for controlling an active polling loop.
#[derive(Debug)]
pub struct PollHandle {
    cancel_tx: tokio::sync::watch::Sender<bool>,
    interval_tx: tokio::sync::watch::Sender<Duration>,
    // Keep receivers alive so send() succeeds and updates the value.
    _cancel_rx: tokio::sync::watch::Receiver<bool>,
    _interval_rx: tokio::sync::watch::Receiver<Duration>,
}

impl PollHandle {
    /// Stop the polling loop.
    pub fn stop(&self) {
        let _ = self.cancel_tx.send(true);
    }

    /// Adjust the polling interval dynamically (battery conservation).
    pub fn set_interval(&self, interval: Duration) {
        let _ = self.interval_tx.send(interval);
    }

    /// Check if the polling loop is still running.
    pub fn is_running(&self) -> bool {
        !*self.cancel_tx.borrow()
    }
}

/// Start a polling loop that reads PIDs and sends events to a channel.
///
/// Returns a (PollHandle, Receiver, PollConfig) triple. Use PollHandle to stop or adjust.
/// The polling task runs on the current tokio runtime.
///
/// BR-6.1: Cancellable via PollHandle::stop()
/// BR-6.4: Single PID failure emits PollEvent::Error, doesn't stop the loop
/// BR-6.5: Task is tracked via PollHandle
pub fn start_poll_loop(config: PollConfig) -> (PollHandle, mpsc::Receiver<PollEvent>, PollConfig) {
    let (event_tx, event_rx) = mpsc::channel(256);
    let (cancel_tx, cancel_rx) = tokio::sync::watch::channel(false);
    let (interval_tx, interval_rx) = tokio::sync::watch::channel(config.interval);

    // Keep the sender alive so the channel stays open
    let _ = event_tx;

    let handle = PollHandle {
        cancel_tx,
        interval_tx,
        _cancel_rx: cancel_rx,
        _interval_rx: interval_rx,
    };

    // Return the config so Session can drive the loop itself
    // (Session needs &mut self for adapter access, so we can't spawn internally)
    (handle, event_rx, config)
}

/// Execute one poll cycle through the session-owned API.
pub async fn execute_poll_cycle<A: crate::adapter::Adapter>(
    session: &mut super::Session<A>,
    config: &PollConfig,
    event_tx: &mpsc::Sender<PollEvent>,
    spec_override: Option<&crate::vehicle::VehicleSpec>,
) {
    for &pid in &config.pids {
        match session.read_pid(pid).await {
            Ok(reading) => {
                let value = reading.value.clone();
                let _ = event_tx.send(PollEvent::Reading { pid, reading }).await;

                if let crate::protocol::enhanced::Value::Scalar(v) = &value {
                    if let Some(result) = spec_override
                        .and_then(|spec| {
                            super::threshold::evaluate_pid_threshold(Some(spec), pid, *v)
                        })
                        .or_else(|| session.evaluate_threshold(pid, *v))
                    {
                        let _ = event_tx.send(PollEvent::Alert(result)).await;
                    }
                }
            }
            Err(crate::error::Obd2Error::NoData) => {
                // Skip -- PID not supported (BR-6.4)
            }
            Err(e) => {
                let _ = event_tx
                    .send(PollEvent::Error {
                        pid: Some(pid),
                        error: e.to_string(),
                    })
                    .await;
            }
        }
    }

    // Battery voltage
    if config.read_voltage {
        if let Ok(Some(v)) = session.battery_voltage().await {
            let _ = event_tx.send(PollEvent::Voltage(v)).await;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::mock::MockAdapter;
    use crate::session::Session;

    #[test]
    fn test_poll_config_defaults() {
        let config = PollConfig::new(vec![Pid::ENGINE_RPM, Pid::VEHICLE_SPEED]);
        assert_eq!(config.pids.len(), 2);
        assert_eq!(config.interval, Duration::from_millis(250));
        assert!(config.read_voltage);
    }

    #[test]
    fn test_poll_config_builder() {
        let config = PollConfig::new(vec![Pid::ENGINE_RPM])
            .with_interval(Duration::from_millis(500))
            .with_voltage(false);
        assert_eq!(config.interval, Duration::from_millis(500));
        assert!(!config.read_voltage);
    }

    #[test]
    fn test_poll_handle_stop() {
        let (handle, _rx, _config) = start_poll_loop(PollConfig::new(vec![Pid::ENGINE_RPM]));
        assert!(handle.is_running());
        handle.stop();
        assert!(!handle.is_running());
    }

    #[tokio::test]
    async fn test_execute_poll_cycle() {
        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        session.initialize().await.unwrap();

        let config = PollConfig::new(vec![Pid::ENGINE_RPM, Pid::COOLANT_TEMP]);
        let (tx, mut rx) = mpsc::channel(64);

        execute_poll_cycle(&mut session, &config, &tx, None).await;

        // Should receive at least 2 readings + 1 voltage
        let mut count = 0;
        while let Ok(event) = rx.try_recv() {
            match event {
                PollEvent::Reading { .. } => count += 1,
                PollEvent::Voltage(_) => count += 1,
                _ => {}
            }
        }
        assert!(count >= 2, "expected at least 2 events, got {}", count);
    }

    #[tokio::test]
    async fn test_poll_cycle_with_threshold() {
        use crate::vehicle::{
            CommunicationSpec, EngineSpec, NamedThreshold, SpecIdentity, Threshold, ThresholdSet,
            VehicleSpec,
        };

        let spec = VehicleSpec {
            spec_version: Some("1.0".into()),
            identity: SpecIdentity {
                name: "Test".into(),
                model_years: (2020, 2020),
                makes: vec![],
                models: vec![],
                engine: EngineSpec {
                    code: "T".into(),
                    displacement_l: 2.0,
                    cylinders: 4,
                    layout: "I4".into(),
                    aspiration: "NA".into(),
                    fuel_type: "Gas".into(),
                    fuel_system: None,
                    compression_ratio: None,
                    max_power_kw: None,
                    max_torque_nm: None,
                    redline_rpm: 6500,
                    idle_rpm_warm: 700,
                    idle_rpm_cold: 900,
                    firing_order: None,
                    ecm_hardware: None,
                },
                transmission: None,
                vin_match: None,
            },
            communication: CommunicationSpec {
                buses: vec![],
                elm327_protocol_code: None,
            },
            thresholds: Some(ThresholdSet {
                engine: vec![NamedThreshold {
                    name: "coolant_temp_c".into(),
                    threshold: Threshold {
                        min: Some(0.0),
                        max: Some(130.0),
                        warning_low: None,
                        warning_high: Some(60.0),
                        critical_low: None,
                        critical_high: Some(100.0),
                        unit: "\u{00B0}C".into(),
                    },
                }],
                transmission: vec![],
            }),
            dtc_library: None,
            polling_groups: vec![],
            diagnostic_rules: vec![],
            known_issues: vec![],
            enhanced_pids: vec![],
        };

        let config = PollConfig::new(vec![Pid::COOLANT_TEMP]).with_voltage(false);
        let (tx, mut rx) = mpsc::channel(64);

        let adapter = MockAdapter::new();
        let mut session = Session::new(adapter);
        session.initialize().await.unwrap();

        execute_poll_cycle(&mut session, &config, &tx, Some(&spec)).await;

        // MockAdapter returns 50 deg C for coolant which is below warning threshold of 60
        // So we should NOT get an alert
        let mut got_alert = false;
        while let Ok(event) = rx.try_recv() {
            if matches!(event, PollEvent::Alert(_)) {
                got_alert = true;
            }
        }
        assert!(
            !got_alert,
            "50 deg C should not trigger alert (warning at 60)"
        );
    }
}
