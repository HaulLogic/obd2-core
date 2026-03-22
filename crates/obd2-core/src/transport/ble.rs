//! BLE GATT transport using btleplug.
//!
//! Requires the `ble` feature flag.
//! Supports STN-based adapters (OBDLink MX+, etc.) and Nordic UART Service.

use async_trait::async_trait;
use btleplug::api::{CharPropFlags, Central, Manager as _, Peripheral as _, WriteType};
use btleplug::platform::{Manager, Peripheral};
use futures::StreamExt;
use tokio::sync::mpsc;
use tracing::{debug, info, warn};
use uuid::Uuid;

use super::Transport;
use crate::error::Obd2Error;

// ── GATT UUIDs ──────────────────────────────────────────────────────────────

/// OBDLink/STN custom UART service (FFF0).
#[allow(dead_code)]
const STN_SERVICE: Uuid = Uuid::from_u128(0x0000_FFF0_0000_1000_8000_0080_5F9B_34FB);
const STN_NOTIFY_CHAR: Uuid = Uuid::from_u128(0x0000_FFF1_0000_1000_8000_0080_5F9B_34FB);
const STN_WRITE_CHAR: Uuid = Uuid::from_u128(0x0000_FFF2_0000_1000_8000_0080_5F9B_34FB);

/// Nordic UART Service (NUS) — used by some generic ELM327 BLE adapters.
#[allow(dead_code)]
const NUS_SERVICE: Uuid = Uuid::from_u128(0x6E40_0001_B5A3_F393_E0A9_E50E_24DC_CA9E);
const NUS_TX_CHAR: Uuid = Uuid::from_u128(0x6E40_0003_B5A3_F393_E0A9_E50E_24DC_CA9E); // notifications from this
const NUS_RX_CHAR: Uuid = Uuid::from_u128(0x6E40_0002_B5A3_F393_E0A9_E50E_24DC_CA9E); // write to this

/// Known name prefixes for OBD-II BLE adapters.
const ADAPTER_NAME_PATTERNS: &[&str] = &["OBDLink", "OBD", "ELM327", "STN", "OBDII", "Vgate", "vLink", "Veepeak"];

/// Read timeout for BLE responses.
const BLE_READ_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(5);

// ── Characteristic discovery ────────────────────────────────────────────────

struct UartChars {
    write_char: btleplug::api::Characteristic,
    notify_char: btleplug::api::Characteristic,
}

/// Discover the UART write+notify characteristics using a 3-tier strategy:
/// 1. OBDLink/STN UUIDs (FFF0/FFF1/FFF2)
/// 2. Nordic UART Service (NUS)
/// 3. Heuristic: any writable + notifiable pair
fn find_uart_characteristics(
    chars: &[btleplug::api::Characteristic],
) -> Result<UartChars, Obd2Error> {
    // Tier 1: STN/OBDLink
    let stn_write = chars.iter().find(|c| c.uuid == STN_WRITE_CHAR);
    let stn_notify = chars.iter().find(|c| c.uuid == STN_NOTIFY_CHAR);
    if let (Some(w), Some(n)) = (stn_write, stn_notify) {
        info!("using OBDLink/STN GATT UUIDs (FFF0 service)");
        return Ok(UartChars {
            write_char: w.clone(),
            notify_char: n.clone(),
        });
    }

    // Tier 2: Nordic UART
    let nus_write = chars.iter().find(|c| c.uuid == NUS_RX_CHAR);
    let nus_notify = chars.iter().find(|c| c.uuid == NUS_TX_CHAR);
    if let (Some(w), Some(n)) = (nus_write, nus_notify) {
        info!("using Nordic UART Service (NUS) UUIDs");
        return Ok(UartChars {
            write_char: w.clone(),
            notify_char: n.clone(),
        });
    }

    // Tier 3: heuristic — find any writable + notifiable pair
    let writable = chars.iter().find(|c| {
        c.properties.contains(CharPropFlags::WRITE)
            || c.properties.contains(CharPropFlags::WRITE_WITHOUT_RESPONSE)
    });
    let notifiable = chars.iter().find(|c| {
        c.properties.contains(CharPropFlags::NOTIFY)
            || c.properties.contains(CharPropFlags::INDICATE)
    });

    if let (Some(w), Some(n)) = (writable, notifiable) {
        warn!(
            write_uuid = %w.uuid,
            notify_uuid = %n.uuid,
            "using heuristic GATT characteristics (no known service matched)"
        );
        return Ok(UartChars {
            write_char: w.clone(),
            notify_char: n.clone(),
        });
    }

    Err(Obd2Error::Transport(
        "no suitable UART characteristics found on BLE device \
         (expected STN FFF0, Nordic UART, or any writable+notifiable pair)"
            .into(),
    ))
}

// ── Adapter name matching ───────────────────────────────────────────────────

/// Check if a device name matches our adapter filter.
fn is_adapter_match(name: &str, name_filter: Option<&str>) -> bool {
    let lower = name.to_lowercase();
    if let Some(filter) = name_filter {
        lower.contains(&filter.to_lowercase())
    } else {
        ADAPTER_NAME_PATTERNS
            .iter()
            .any(|p| lower.contains(&p.to_lowercase()))
    }
}

// ── BleTransport ────────────────────────────────────────────────────────────

/// BLE GATT transport for OBD-II adapters.
///
/// Connects to a BLE peripheral and communicates via GATT characteristics.
/// Supports OBDLink/STN custom service (FFF0), Nordic UART Service (NUS),
/// and falls back to heuristic characteristic discovery.
pub struct BleTransport {
    peripheral: Peripheral,
    write_char: btleplug::api::Characteristic,
    device_name: String,
    rx: mpsc::Receiver<Vec<u8>>,
    /// Background notification listener handle.
    _listener: tokio::task::JoinHandle<()>,
}

impl BleTransport {
    /// Connect to a BLE peripheral that's already been discovered.
    ///
    /// Performs GATT service discovery, finds UART characteristics,
    /// subscribes to notifications, and spawns a background listener.
    pub async fn connect(peripheral: Peripheral) -> Result<Self, Obd2Error> {
        peripheral
            .connect()
            .await
            .map_err(|e| Obd2Error::Transport(format!("BLE connect failed: {e}")))?;

        peripheral
            .discover_services()
            .await
            .map_err(|e| Obd2Error::Transport(format!("BLE service discovery failed: {e}")))?;

        let chars: Vec<btleplug::api::Characteristic> = peripheral
            .services()
            .iter()
            .flat_map(|s| s.characteristics.clone())
            .collect();

        debug!(count = chars.len(), "BLE characteristics discovered");
        for c in &chars {
            debug!(
                uuid = %c.uuid,
                properties = ?c.properties,
                service = %c.service_uuid,
                "  characteristic"
            );
        }

        let uart = find_uart_characteristics(&chars)?;

        // Subscribe to notifications on the notify characteristic
        peripheral
            .subscribe(&uart.notify_char)
            .await
            .map_err(|e| Obd2Error::Transport(format!("BLE subscribe failed: {e}")))?;

        // Spawn background task to forward notifications into an mpsc channel
        let (tx, rx) = mpsc::channel::<Vec<u8>>(256);
        let notif_peripheral = peripheral.clone();
        let listener = tokio::spawn(async move {
            let Ok(mut stream) = notif_peripheral.notifications().await else {
                return;
            };
            while let Some(notif) = stream.next().await {
                if tx.send(notif.value).await.is_err() {
                    break; // receiver dropped
                }
            }
        });

        let name = peripheral
            .properties()
            .await
            .ok()
            .flatten()
            .and_then(|p| p.local_name)
            .unwrap_or_else(|| "Unknown BLE Device".into());

        info!(device = %name, "BLE transport connected");

        Ok(Self {
            peripheral,
            write_char: uart.write_char,
            device_name: name,
            rx,
            _listener: listener,
        })
    }

    /// Scan for BLE OBD-II adapters and connect to the first matching one.
    ///
    /// If `name_filter` is provided, only peripherals whose name contains that
    /// string (case-insensitive) will match. Otherwise, common OBD adapter name
    /// patterns are used for auto-detection.
    pub async fn scan_and_connect(
        name_filter: Option<&str>,
        scan_duration: std::time::Duration,
    ) -> Result<Self, Obd2Error> {
        let manager = Manager::new()
            .await
            .map_err(|e| Obd2Error::Transport(format!("BLE manager error: {e}")))?;

        let adapters = manager
            .adapters()
            .await
            .map_err(|e| Obd2Error::Transport(format!("no BLE adapters: {e}")))?;

        let central = adapters
            .into_iter()
            .next()
            .ok_or_else(|| Obd2Error::Transport("no BLE adapter found".into()))?;

        info!(duration = ?scan_duration, "scanning for BLE OBD adapters");

        // Subscribe to discovery events for real-time matching
        let mut events = central
            .events()
            .await
            .map_err(|e| Obd2Error::Transport(format!("BLE events error: {e}")))?;

        central
            .start_scan(btleplug::api::ScanFilter::default())
            .await
            .map_err(|e| Obd2Error::Transport(format!("BLE scan failed: {e}")))?;

        // Try to find an adapter via discovery events within the timeout
        let name_filter_owned = name_filter.map(|s| s.to_string());
        let central_clone = central.clone();

        let found = tokio::time::timeout(scan_duration, async {
            use btleplug::api::CentralEvent;
            while let Some(event) = events.next().await {
                if let CentralEvent::DeviceDiscovered(id) = event {
                    if let Ok(peripheral) = central_clone.peripheral(&id).await {
                        if let Ok(Some(props)) = peripheral.properties().await {
                            let name = props.local_name.unwrap_or_default();
                            if !name.is_empty() {
                                info!(device = %name, "BLE device found");
                                if is_adapter_match(
                                    &name,
                                    name_filter_owned.as_deref(),
                                ) {
                                    central_clone.stop_scan().await.ok();
                                    return Ok(peripheral);
                                }
                            }
                        }
                    }
                }
            }
            Err(Obd2Error::Transport("BLE event stream ended".into()))
        })
        .await;

        central.stop_scan().await.ok();

        match found {
            Ok(Ok(peripheral)) => Self::connect(peripheral).await,
            Ok(Err(e)) => Err(e),
            Err(_) => {
                // Timeout — check already-discovered peripherals as fallback
                let peripherals = central
                    .peripherals()
                    .await
                    .map_err(|e| Obd2Error::Transport(format!("BLE peripherals error: {e}")))?;

                for peripheral in peripherals {
                    if let Ok(Some(props)) = peripheral.properties().await {
                        let name = props.local_name.unwrap_or_default();
                        if !name.is_empty() && is_adapter_match(&name, name_filter) {
                            info!(device = %name, "found BLE OBD adapter (from cache)");
                            return Self::connect(peripheral).await;
                        }
                    }
                }

                Err(Obd2Error::Transport(
                    "no BLE OBD adapter found during scan".into(),
                ))
            }
        }
    }
}

impl std::fmt::Debug for BleTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BleTransport")
            .field("device_name", &self.device_name)
            .finish()
    }
}

#[async_trait]
impl Transport for BleTransport {
    async fn write(&mut self, data: &[u8]) -> Result<(), Obd2Error> {
        debug!(device = %self.device_name, data = %String::from_utf8_lossy(data), "BLE write");
        // BLE has a max write size (typically 20 bytes for BLE 4.x, 512 for 5.x).
        // Chunk if necessary, though most OBD commands are short.
        for chunk in data.chunks(20) {
            self.peripheral
                .write(&self.write_char, chunk, WriteType::WithResponse)
                .await
                .map_err(|e| Obd2Error::Transport(format!("BLE write failed: {e}")))?;
        }
        Ok(())
    }

    async fn read(&mut self) -> Result<Vec<u8>, Obd2Error> {
        let mut result = Vec::new();

        loop {
            match tokio::time::timeout(BLE_READ_TIMEOUT, self.rx.recv()).await {
                Ok(Some(data)) => {
                    result.extend_from_slice(&data);
                    // Check for ELM327 prompt character '>'
                    if result.contains(&b'>') {
                        break;
                    }
                }
                Ok(None) => {
                    return Err(Obd2Error::Transport(
                        "BLE notification stream ended".into(),
                    ));
                }
                Err(_) => {
                    if result.is_empty() {
                        return Err(Obd2Error::Timeout);
                    }
                    break; // Return what we have
                }
            }
        }

        debug!(device = %self.device_name, data = %String::from_utf8_lossy(&result), "BLE read");
        Ok(result)
    }

    async fn reset(&mut self) -> Result<(), Obd2Error> {
        // Drain any pending notifications
        while self.rx.try_recv().is_ok() {}
        Ok(())
    }

    fn name(&self) -> &str {
        &self.device_name
    }
}

impl Drop for BleTransport {
    fn drop(&mut self) {
        // Best-effort disconnect
        let peripheral = self.peripheral.clone();
        tokio::spawn(async move {
            if let Err(e) = peripheral.disconnect().await {
                warn!("BLE disconnect error: {e}");
            }
        });
    }
}
