# obd2-core AI Agent Guide

This document helps AI coding assistants (Claude Code, Copilot, Cursor, etc.) work effectively with the obd2-core library. It describes the architecture, key APIs, type signatures, and common patterns so agents can generate correct code without reading every source file.

## Project Identity

- **What**: Cross-platform Rust library for OBD-II vehicle diagnostics
- **Runtime**: Async (tokio). All I/O methods are `async`.
- **MSRV**: Rust 1.75+ (edition 2021)
- **License**: MIT OR Apache-2.0
- **Workspace**: 2 crates — `obd2-core` (library) and `obd2-store-sqlite` (storage backend)

## Architecture at a Glance

```
Session<A: Adapter>        ← consumer API (generic over adapter)
  ├── Adapter trait         ← protocol interpreter (ELM327, mock, custom)
  │     └── Transport trait ← raw byte I/O (serial, BLE, mock, custom)
  ├── SpecRegistry          ← vehicle specs (embedded YAML + runtime loading)
  └── Store traits          ← persistence (VehicleStore, SessionStore)
```

All public types live under `obd2_core::`. The crate re-exports modules, not individual types — always use full paths like `obd2_core::protocol::pid::Pid`.

## Import Map

```rust
// Core types
use obd2_core::error::Obd2Error;
use obd2_core::protocol::pid::Pid;
use obd2_core::protocol::dtc::{Dtc, DtcCategory, DtcStatus, Severity};
use obd2_core::protocol::enhanced::{Value, Reading, ReadingSource, EnhancedPid, Formula, Confidence};
use obd2_core::protocol::service::{ServiceRequest, Target, VehicleInfo, O2TestResult};

// Session
use obd2_core::session::Session;
use obd2_core::session::poller::{PollConfig, PollEvent, PollHandle};
use obd2_core::session::threshold;
use obd2_core::session::DiagnosticSessionState;
use obd2_core::session::diag_session::KeyFunction;
use obd2_core::protocol::service::{ActuatorCommand, DiagSession};

// Vehicle
use obd2_core::vehicle::{VehicleSpec, VehicleProfile, SpecRegistry, ModuleId, Protocol, PhysicalAddress, ThresholdSet, ThresholdResult, AlertLevel};
use obd2_core::vehicle::vin;

// J1939
use obd2_core::protocol::j1939::{Pgn, J1939Dtc, decode_eec1, decode_ccvs, decode_et1, decode_eflp1, decode_lfe, decode_dm1};

// Adapter
use obd2_core::adapter::{Adapter, AdapterInfo, Chipset, Capabilities};
use obd2_core::adapter::elm327::Elm327Adapter;
use obd2_core::adapter::mock::MockAdapter;

// Transport
use obd2_core::transport::Transport;
use obd2_core::transport::mock::MockTransport;
#[cfg(feature = "serial")]
use obd2_core::transport::serial::SerialTransport;
#[cfg(feature = "ble")]
use obd2_core::transport::ble::BleTransport;

// Store
use obd2_core::store::{VehicleStore, SessionStore};

// SQLite (separate crate)
use obd2_store_sqlite::SqliteStore;
```

## Type Signatures (Quick Reference)

### Session<A: Adapter>

```rust
impl<A: Adapter> Session<A> {
    pub fn new(adapter: A) -> Self;
    pub fn load_spec(&mut self, path: &Path) -> Result<(), Obd2Error>;
    pub fn load_spec_dir(&mut self, dir: &Path) -> Result<usize, Obd2Error>;
    pub fn specs(&self) -> &SpecRegistry;
    pub fn vehicle(&self) -> Option<&VehicleProfile>;
    pub fn spec(&self) -> Option<&VehicleSpec>;
    pub fn adapter_info(&self) -> &AdapterInfo;

    // Mode 01
    pub async fn read_pid(&mut self, pid: Pid) -> Result<Reading, Obd2Error>;
    pub async fn read_pids(&mut self, pids: &[Pid]) -> Result<Vec<(Pid, Reading)>, Obd2Error>;
    pub async fn supported_pids(&mut self) -> Result<HashSet<Pid>, Obd2Error>;

    // Mode 03/07/0A
    pub async fn read_dtcs(&mut self) -> Result<Vec<Dtc>, Obd2Error>;
    pub async fn read_pending_dtcs(&mut self) -> Result<Vec<Dtc>, Obd2Error>;
    pub async fn read_permanent_dtcs(&mut self) -> Result<Vec<Dtc>, Obd2Error>;

    // Mode 03/07/0A (aggregated)
    pub async fn read_all_dtcs(&mut self) -> Result<Vec<Dtc>, Obd2Error>;

    // Mode 04
    pub async fn clear_dtcs(&mut self) -> Result<(), Obd2Error>;
    pub async fn clear_dtcs_on_module(&mut self, module: ModuleId) -> Result<(), Obd2Error>;

    // Mode 05
    pub async fn read_o2_monitoring(&mut self, test_id: u8) -> Result<Vec<O2TestResult>, Obd2Error>;
    pub async fn read_all_o2_monitoring(&mut self) -> Result<Vec<O2TestResult>, Obd2Error>;

    // Mode 06
    pub async fn read_test_results(&mut self, test_id: u8) -> Result<Vec<TestResult>, Obd2Error>;

    // Mode 02
    pub async fn read_freeze_frame(&mut self, pid: Pid, frame: u8) -> Result<Reading, Obd2Error>;

    // Mode 01 PID 01
    pub async fn read_readiness(&mut self) -> Result<ReadinessStatus, Obd2Error>;

    // Mode 09
    pub async fn read_vin(&mut self) -> Result<String, Obd2Error>;
    pub async fn read_vehicle_info(&mut self) -> Result<VehicleInfo, Obd2Error>;
    pub async fn identify_vehicle(&mut self) -> Result<VehicleProfile, Obd2Error>;

    // Mode 22 (enhanced)
    pub async fn read_enhanced(&mut self, did: u16, module: ModuleId) -> Result<Reading, Obd2Error>;
    pub fn module_pids(&self, module: ModuleId) -> Vec<&EnhancedPid>;

    // Diagnostic session (Mode 10/27/2F/3E)
    pub async fn enter_diagnostic_session(&mut self, session: DiagSession, module: ModuleId) -> Result<(), Obd2Error>;
    pub async fn security_access(&mut self, module: ModuleId, key_fn: &KeyFunction) -> Result<(), Obd2Error>;
    pub async fn actuator_control(&mut self, did: u16, module: ModuleId, command: &ActuatorCommand) -> Result<(), Obd2Error>;
    pub async fn actuator_release(&mut self, did: u16, module: ModuleId) -> Result<(), Obd2Error>;
    pub async fn tester_present(&mut self, module: ModuleId) -> Result<(), Obd2Error>;
    pub async fn end_diagnostic_session(&mut self, module: ModuleId) -> Result<(), Obd2Error>;

    // J1939 (heavy-duty)
    pub async fn read_j1939_pgn(&mut self, pgn: Pgn) -> Result<Vec<u8>, Obd2Error>;
    pub async fn read_j1939_dtcs(&mut self) -> Result<Vec<J1939Dtc>, Obd2Error>;

    // Thresholds
    pub fn evaluate_threshold(&self, pid: Pid, value: f64) -> Option<ThresholdResult>;
    pub fn evaluate_enhanced_threshold(&self, did: u16, value: f64) -> Option<ThresholdResult>;

    // Adapter
    pub async fn battery_voltage(&mut self) -> Result<Option<f64>, Obd2Error>;

    // State
    pub fn connection_state(&self) -> &ConnectionState;
    pub fn diagnostic_state(&self) -> &DiagnosticSessionState;
    pub fn discovery(&self) -> Option<&DiscoveryProfile>;
    pub fn visible_ecus(&self) -> &[VisibleEcu];

    // Raw capture
    pub fn set_raw_capture_enabled(&mut self, enabled: bool);
    pub fn set_raw_capture_directory(&mut self, dir: impl Into<PathBuf>);
    pub fn raw_capture_path(&self) -> Option<&Path>;
    pub fn start_raw_capture(&mut self, path: &Path, metadata: &CaptureMetadata) -> Result<(), Obd2Error>;
    pub fn stop_raw_capture(&mut self) -> Result<Option<PathBuf>, Obd2Error>;

    // Escape hatch
    pub async fn raw_request(&mut self, service: u8, data: &[u8], target: Target) -> Result<Vec<u8>, Obd2Error>;
}
```

### Pid

```rust
pub struct Pid(pub u8);  // Newtype, Copy + Clone + Hash + Eq

// Named constants (selected — see protocol/pid.rs for full list)
Pid::ENGINE_RPM           // 0x0C
Pid::COOLANT_TEMP         // 0x05
Pid::VEHICLE_SPEED        // 0x0D
Pid::ENGINE_LOAD          // 0x04
Pid::THROTTLE_POSITION    // 0x11
Pid::INTAKE_AIR_TEMP      // 0x0F
Pid::MAF                  // 0x10
Pid::FUEL_TANK_LEVEL      // 0x2F
Pid::ENGINE_OIL_TEMP      // 0x5C
Pid::CONTROL_MODULE_VOLTAGE // 0x42
Pid::AMBIENT_AIR_TEMP     // 0x46

impl Pid {
    pub fn name(&self) -> &'static str;
    pub fn unit(&self) -> &'static str;
    pub fn response_bytes(&self) -> u8;
    pub fn value_type(&self) -> ValueType;
    pub fn parse(&self, data: &[u8]) -> Result<Value, Obd2Error>;
}
```

### Value & Reading

```rust
pub enum Value {
    Scalar(f64),
    Bitfield(Bitfield),
    State(String),
    Raw(Vec<u8>),
}

impl Value {
    pub fn as_f64(&self) -> Result<f64, Obd2Error>;  // Extracts Scalar, errors on others
}

pub struct Reading {
    pub value: Value,
    pub unit: &'static str,
    pub timestamp: Instant,
    pub raw_bytes: Vec<u8>,
    pub source: ReadingSource,  // Live, FreezeFrame, Replay
}
```

### Dtc

```rust
pub struct Dtc {
    pub code: String,                      // "P0420"
    pub category: DtcCategory,             // Powertrain/Chassis/Body/Network
    pub status: DtcStatus,                 // Stored/Pending/Permanent
    pub description: Option<String>,
    pub severity: Option<Severity>,        // Critical/High/Medium/Low/Info
    pub source_module: Option<String>,
    pub notes: Option<String>,
}

impl Dtc {
    pub fn from_bytes(b1: u8, b2: u8) -> Self;
    pub fn from_code(code: &str) -> Self;
}
```

### Obd2Error

```rust
#[non_exhaustive]
pub enum Obd2Error {
    Transport(String),
    Adapter(String),
    AdapterBusy,
    Timeout,
    NoData,
    UnsupportedPid { pid: u8 },
    ModuleNotFound(String),
    NegativeResponse { service: u8, nrc: NegativeResponse },
    SecurityRequired,
    NoSpec,
    BusNotAvailable(String),
    SpecParse(String),
    ParseError(String),
    Io(std::io::Error),
    Other(Box<dyn Error + Send + Sync>),
}
```

### Traits to Implement

```rust
// Transport — physical byte I/O
#[async_trait]
pub trait Transport: Send + Sync {
    async fn write(&mut self, data: &[u8]) -> Result<(), Obd2Error>;
    async fn read(&mut self) -> Result<Vec<u8>, Obd2Error>;
    async fn reset(&mut self) -> Result<(), Obd2Error>;
    fn name(&self) -> &str;
}

// Adapter — protocol interpreter
#[async_trait]
pub trait Adapter: Send {
    async fn initialize(&mut self) -> Result<InitializationReport, Obd2Error>;
    async fn request(&mut self, req: &ServiceRequest) -> Result<Vec<u8>, Obd2Error>;
    async fn routed_request(&mut self, req: &RoutedRequest) -> Result<Vec<u8>, Obd2Error>;
    async fn supported_pids(&mut self) -> Result<HashSet<Pid>, Obd2Error>;
    async fn battery_voltage(&mut self) -> Result<Option<f64>, Obd2Error>;
    fn info(&self) -> &AdapterInfo;
    fn drain_events(&mut self) -> Vec<AdapterEvent>;
    fn transport_mut(&mut self) -> Option<&mut dyn Transport>;
}

// Storage — persistence
#[async_trait]
pub trait VehicleStore: Send + Sync {
    async fn save_vehicle(&self, profile: &VehicleProfile) -> Result<(), Obd2Error>;
    async fn get_vehicle(&self, vin: &str) -> Result<Option<VehicleProfile>, Obd2Error>;
    async fn save_thresholds(&self, vin: &str, ts: &ThresholdSet) -> Result<(), Obd2Error>;
    async fn get_thresholds(&self, vin: &str) -> Result<Option<ThresholdSet>, Obd2Error>;
}

#[async_trait]
pub trait SessionStore: Send + Sync {
    async fn save_reading(&self, vin: &str, pid: Pid, reading: &Reading) -> Result<(), Obd2Error>;
    async fn save_dtc_event(&self, vin: &str, dtcs: &[Dtc]) -> Result<(), Obd2Error>;
}
```

## Common Patterns

### Pattern: Basic Session Setup

```rust
// With real hardware
let transport = SerialTransport::new("/dev/ttyUSB0", 115200).await?;
let mut adapter = Elm327Adapter::new(transport);
adapter.initialize().await?;
let mut session = Session::new(adapter);
let profile = session.identify_vehicle().await?;

// With mock (for tests)
let adapter = MockAdapter::with_vin("1GCHK23224F000001");
let mut session = Session::new(adapter);
```

### Pattern: Read PIDs with Fallback

```rust
let reading = match session.read_pid(Pid::ENGINE_OIL_TEMP).await {
    Ok(r) => Some(r),
    Err(Obd2Error::NoData | Obd2Error::UnsupportedPid { .. }) => None,
    Err(e) => return Err(e),
};
```

### Pattern: MockAdapter for Tests

```rust
#[tokio::test]
async fn test_something() {
    let mut adapter = MockAdapter::with_vin("1GCHK23224F000001");
    adapter.set_dtcs(vec![Dtc::from_code("P0420")]);
    let mut session = Session::new(adapter);

    let profile = session.identify_vehicle().await.unwrap();
    assert!(profile.spec.is_some()); // Matches embedded Duramax spec

    let dtcs = session.read_dtcs().await.unwrap();
    assert_eq!(dtcs.len(), 1);
    assert_eq!(dtcs[0].code, "P0420");
}
```

### Pattern: Polling Loop

```rust
use obd2_core::session::poller::{PollConfig, PollEvent};
use std::time::Duration;

let config = PollConfig {
    pids: vec![Pid::ENGINE_RPM, Pid::COOLANT_TEMP],
    interval: Duration::from_millis(250),
    read_voltage: true,
};

let (handle, mut rx, _) = poller::start_poll_loop(config);

while let Some(event) = rx.recv().await {
    match event {
        PollEvent::Reading { pid, reading } => { /* use data */ }
        PollEvent::Alert(result) => { /* threshold breach */ }
        PollEvent::Voltage(v) => { /* battery */ }
        PollEvent::Error { pid, error } => { /* non-fatal */ }
        _ => {}
    }
}

handle.stop();
```

### Pattern: Enhanced PID Read

```rust
use obd2_core::vehicle::ModuleId;

// Requires a matched vehicle spec
let profile = session.identify_vehicle().await?;
if profile.spec.is_some() {
    let boost = session.read_enhanced(0x0124, ModuleId::new("ecm")).await?;
    let tcm_pids = session.module_pids(ModuleId::new("tcm"));
}
```

### Pattern: Storage

```rust
use obd2_store_sqlite::SqliteStore;

let store = SqliteStore::open(Path::new("data.db"))?;
store.save_vehicle(&profile).await?;
store.save_reading(&profile.vin, Pid::ENGINE_RPM, &reading).await?;

let loaded = store.get_vehicle("1GCHK23224F000001").await?;
```

## File Map

| Path | Purpose | Key Types |
|------|---------|-----------|
| `crates/obd2-core/src/lib.rs` | Crate root, module declarations | — |
| `crates/obd2-core/src/error.rs` | Error types | `Obd2Error`, `NegativeResponse` |
| `crates/obd2-core/src/protocol/pid.rs` | Standard PID definitions | `Pid`, `ValueType` |
| `crates/obd2-core/src/protocol/dtc.rs` | DTC decoding | `Dtc`, `DtcCategory`, `DtcStatus`, `Severity` |
| `crates/obd2-core/src/protocol/enhanced.rs` | Enhanced PID types | `EnhancedPid`, `Value`, `Reading`, `Formula`, `Confidence` |
| `crates/obd2-core/src/protocol/service.rs` | Service request types | `ServiceRequest`, `Target`, `VehicleInfo`, `O2TestResult` |
| `crates/obd2-core/src/transport/mod.rs` | Transport trait | `Transport` |
| `crates/obd2-core/src/transport/serial.rs` | Serial transport | `SerialTransport` |
| `crates/obd2-core/src/transport/ble.rs` | BLE transport | `BleTransport` |
| `crates/obd2-core/src/adapter/mod.rs` | Adapter trait | `Adapter`, `AdapterInfo`, `Chipset`, `Capabilities` |
| `crates/obd2-core/src/adapter/elm327.rs` | ELM327 adapter | `Elm327Adapter` |
| `crates/obd2-core/src/adapter/mock.rs` | Mock adapter | `MockAdapter` |
| `crates/obd2-core/src/adapter/detect.rs` | Chipset detection | `detect_chipset()` |
| `crates/obd2-core/src/vehicle/mod.rs` | Vehicle spec types | `VehicleSpec`, `VehicleProfile`, `SpecRegistry`, `ModuleId`, `Protocol`, `ThresholdSet` |
| `crates/obd2-core/src/vehicle/vin.rs` | Offline VIN decoder | `decode()`, `DecodedVin` |
| `crates/obd2-core/src/vehicle/nhtsa.rs` | NHTSA online VIN lookup | `decode_vin()` |
| `crates/obd2-core/src/vehicle/loader.rs` | YAML spec parser | — |
| `crates/obd2-core/src/session/mod.rs` | Session orchestrator | `Session` |
| `crates/obd2-core/src/session/poller.rs` | Polling engine | `PollConfig`, `PollEvent`, `PollHandle` |
| `crates/obd2-core/src/session/threshold.rs` | Threshold evaluation | `evaluate()`, `ThresholdResult`, `AlertLevel` |
| `crates/obd2-core/src/session/diagnostics.rs` | DTC enrichment & rules | `DiagnosticRule`, `KnownIssue` |
| `crates/obd2-core/src/session/diag_session.rs` | Diag session types | `SessionState`, `KeyFunction` |
| `crates/obd2-core/src/session/enhanced.rs` | Enhanced PID helpers | `find_service_id_from_spec()`, `list_module_pids()` |
| `crates/obd2-core/src/session/modes.rs` | Internal mode decode | `decode_readiness()`, `decode_dtc_bytes()`, `decode_test_results()` |
| `crates/obd2-core/src/protocol/j1939.rs` | J1939 protocol types | `Pgn`, `J1939Dtc`, `Eec1`, `Ccvs`, `Et1`, `Eflp1`, `Lfe` |
| `crates/obd2-core/src/store/mod.rs` | Storage traits | `VehicleStore`, `SessionStore` |
| `crates/obd2-store-sqlite/src/lib.rs` | SQLite storage | `SqliteStore` |

## OBD-II Mode Reference

| Mode | Service ID | Session Method | Description |
|------|-----------|----------------|-------------|
| 01 | 0x01 | `read_pid()`, `read_pids()`, `supported_pids()` | Current data |
| 02 | 0x02 | `read_freeze_frame()` | Freeze frame data |
| 03 | 0x03 | `read_dtcs()`, `read_all_dtcs()` | Stored DTCs |
| 04 | 0x04 | `clear_dtcs()`, `clear_dtcs_on_module()` | Clear DTCs + reset monitors |
| 05 | 0x05 | `read_o2_monitoring()`, `read_all_o2_monitoring()` | O2 sensor monitoring |
| 06 | 0x06 | `read_test_results()` | On-board test results |
| 07 | 0x07 | `read_pending_dtcs()` | Pending DTCs |
| 08 | 0x08 | via `raw_request()` | Control system tests |
| 09 | 0x09 | `read_vin()`, `read_vehicle_info()`, `identify_vehicle()` | Vehicle information |
| 0A | 0x0A | `read_permanent_dtcs()` | Permanent DTCs |
| 10 | 0x10 | `enter_diagnostic_session()`, `end_diagnostic_session()` | Diagnostic session control |
| 22 | 0x22 | `read_enhanced()` | Enhanced PIDs |
| 27 | 0x27 | `security_access()` | Security access |
| 2F | 0x2F | `actuator_control()`, `actuator_release()` | Actuator control |
| 3E | 0x3E | `tester_present()` | Tester present (keep-alive) |
| J1939 | 0xEA | `read_j1939_pgn()`, `read_j1939_dtcs()` | J1939 PGN requests |

## Feature Flags

| Flag | Default | Dependencies Added |
|------|---------|-------------------|
| `serial` | yes | `tokio-serial` |
| `embedded-specs` | yes | (none — compiles YAML into binary) |
| `ble` | no | `btleplug`, `futures`, `uuid` |
| `nhtsa` | no | `reqwest` (with rustls-tls) |
| `full` | no | all of the above |

## Conventions & Gotchas

1. **All I/O is async** — you need a tokio runtime. Use `#[tokio::main]` or `#[tokio::test]`.
2. **Session is generic**: `Session<A: Adapter>`. In tests use `Session<MockAdapter>`. In production use `Session<Elm327Adapter<SerialTransport>>`.
3. **Pid is a newtype**: `Pid(u8)`, not a bare `u8`. Use named constants like `Pid::ENGINE_RPM`.
4. **ModuleId is a newtype**: `ModuleId(String)`. Create with `ModuleId::new("ecm")`. Constants like `ModuleId::ECM` are `&str`, not `ModuleId` — use them with `ModuleId::new(ModuleId::ECM)`.
5. **#[non_exhaustive]**: `Obd2Error`, `Value`, `PollEvent`, `DtcCategory`, `Formula`, `PhysicalAddress`, `Protocol` are all non-exhaustive. Always include a wildcard arm in match expressions.
6. **`read_pids()` skips NoData**: If a PID returns `NoData`, it's silently skipped. Other errors propagate.
7. **`clear_dtcs()` resets monitors**: This is a destructive operation. Readiness monitors must complete drive cycles again.
8. **Enhanced PIDs require a spec**: `read_enhanced()` needs a matched vehicle spec. Call `identify_vehicle()` first.
9. **MockAdapter VINs**: `MockAdapter::new()` uses a default VIN. Use `MockAdapter::with_vin("...")` for spec matching. VIN `"1GCHK23224F000001"` matches the embedded Duramax spec.
10. **Store traits are async**: Even `SqliteStore` uses async traits (with internal `Mutex<Connection>`).
11. **No re-exports at crate root**: Always use full module paths. There is no `obd2_core::Session` — use `obd2_core::session::Session`.

## Testing

```bash
# Run all tests
cargo test --workspace

# Run with all features
cargo test --workspace --all-features

# Run a specific test
cargo test --package obd2-core test_full_session_lifecycle
```

The test suite uses `MockAdapter` exclusively — no hardware required. The mock simulates a realistic vehicle with:
- Configurable VIN (default or custom via `with_vin()`)
- Configurable DTCs (via `set_dtcs()`)
- Realistic PID responses (RPM=680, coolant=50°C, speed=0, throttle~14.9%, etc.)
- Battery voltage (14.4V)
- J1939 PGN responses (EEC1, CCVS, ET1, EFLP1, LFE, DM1)
- All standard mode responses
