# HaulLogic ↔ obd2-core Integration Spec

**Date:** 2026-03-22
**Status:** Ready for implementation
**Library:** `~/Projects/obd2-core` (standalone, v0.1.0)
**Reference implementation:** `~/Projects/obd2-dash` (completed migration)

## Overview

obd2-core is a cross-platform Rust library for OBD-II vehicle diagnostics. HaulLogic should consume it as a dependency — all protocol logic, vehicle specs, adapter communication, and diagnostic intelligence lives in obd2-core. HaulLogic becomes a pure fleet management UI/business-logic shell.

## Current J1939 Gap

HaulLogic's current integration work found one important gap in the shared library surface: downstream consumers need a real session-level J1939 PGN read path, not just decode helpers or protocol claims in documentation.

See:
- `docs/2026-04-03-j1939-support-requirements.md`
- `docs/2026-04-03-j1939-support-design.md`
- `docs/2026-04-03-j1939-support-change-list.md`

## Cargo Dependency

```toml
[dependencies]
obd2-core = { path = "../obd2-core/crates/obd2-core", features = ["ble", "nhtsa", "embedded-specs", "serde"] }
```

Features:
- `ble` — Bluetooth LE transport (btleplug) for wireless OBD adapters
- `nhtsa` — Online VIN lookup via NHTSA API
- `embedded-specs` — Compiled-in vehicle specs (Duramax, etc.)
- `serde` — Serialize/deserialize for all core types

Omit `serial` unless HaulLogic needs USB serial connections. Add it later if needed.

---

## Core API — Session is the Entry Point

Everything goes through `Session<A: Adapter>`. One session per vehicle connection.

### Import Map

```rust
// Session — primary API
use obd2_core::session::Session;

// Adapters — choose one per connection
use obd2_core::adapter::elm327::Elm327Adapter;
use obd2_core::adapter::mock::MockAdapter;
use obd2_core::adapter::{Adapter, AdapterInfo, Chipset, Capabilities};

// Transports — physical connection layer
use obd2_core::transport::ble::BleTransport;
use obd2_core::transport::serial::SerialTransport;  // if "serial" feature enabled
use obd2_core::transport::Transport;

// Protocol types — PIDs, DTCs, readings
use obd2_core::protocol::pid::Pid;                    // Newtype: Pid(u8), constants like Pid::ENGINE_RPM
use obd2_core::protocol::enhanced::{Reading, Value, ReadingSource, EnhancedPid, Confidence};
use obd2_core::protocol::dtc::{Dtc, DtcCategory, DtcStatus, Severity};
use obd2_core::protocol::service::{O2TestResult, O2SensorLocation};

// J1939 heavy-duty protocol (for fleet trucks) — re-exported from protocol::
use obd2_core::protocol::{Pgn, J1939Dtc, decode_eec1, decode_ccvs, decode_et1, decode_eflp1, decode_lfe};

// Vehicle — specs, profiles, modules
use obd2_core::vehicle::{VehicleSpec, VehicleProfile, ModuleId, SpecRegistry};

// Diagnostics helpers
use obd2_core::session::diagnostics::{enrich_dtcs, active_rules, matching_issues};

// Errors
use obd2_core::error::Obd2Error;

// Storage traits (implement these in HaulLogic)
use obd2_core::store::{VehicleStore, SessionStore};
```

### Connection Lifecycle

```rust
// 1. Create transport (BLE for fleet trucks)
let transport = BleTransport::scan_and_connect(
    Some("OBDLink"),                    // adapter name filter (or None)
    Duration::from_secs(30),            // scan timeout
).await?;

// 2. Wrap in adapter
let adapter = Elm327Adapter::new(Box::new(transport));

// 3. Create session
let mut session = Session::new(adapter);

// 4. Identify vehicle (reads VIN, matches spec, discovers supported PIDs)
let profile = session.identify_vehicle().await?;
// profile.vin              — "1GCHK23164F000001"
// profile.info             — Option<VehicleInfo> (calibration IDs, CVNs)
// profile.spec             — Option<VehicleSpec> (thresholds, enhanced PIDs, rules)
// profile.supported_pids   — HashSet<Pid>
```

### Reading Data

```rust
// Standard PIDs (Mode 01)
let reading = session.read_pid(Pid::ENGINE_RPM).await?;
let rpm = reading.value.as_f64()?;  // 680.0

// Multiple PIDs at once
let readings = session.read_pids(&[
    Pid::ENGINE_RPM, Pid::COOLANT_TEMP, Pid::VEHICLE_SPEED,
]).await?;

// Enhanced PIDs (Mode 22 — manufacturer-specific, requires matched spec)
let module = ModuleId::new("ecm");
let pids = session.module_pids(module.clone());  // Vec<&EnhancedPid>
for epid in pids {
    let reading = session.read_enhanced(epid.did, module.clone()).await?;
    // reading.value, reading.unit, reading.raw_bytes
}

// DTCs (Mode 03/07/0A)
let mut dtcs = session.read_dtcs().await?;
enrich_dtcs(&mut dtcs, session.spec());  // add descriptions, severity, notes from spec
let pending = session.read_pending_dtcs().await?;
let permanent = session.read_permanent_dtcs().await?;

// Clear DTCs (Mode 04) — logs tracing::warn, consumer handles confirmation UX
session.clear_dtcs().await?;

// Battery voltage
let voltage = session.battery_voltage().await?;  // Option<f64>

// O2 sensor monitoring (Mode 05)
let o2_results = session.read_all_o2_monitoring().await?;  // Vec<O2TestResult>

// Supported PIDs
let supported = session.supported_pids().await?;  // HashSet<Pid>

// Raw request (escape hatch)
let raw = session.raw_request(0x01, &[0x0C], Target::Broadcast).await?;
```

### J1939 Heavy-Duty Protocol (for fleet trucks)

```rust
use obd2_core::protocol::{Pgn, decode_eec1, decode_ccvs, decode_et1, decode_eflp1, decode_lfe};

// Read engine data (PGN 61444 — EEC1)
let data = session.read_j1939_pgn(Pgn::EEC1).await?;
if let Some(eec1) = decode_eec1(&data) {
    // eec1.engine_rpm (Option<f64> — None if ECU reports "not available")
    // eec1.actual_torque_pct, eec1.driver_demand_torque_pct
}

// Read vehicle speed (PGN 65265 — CCVS)
let data = session.read_j1939_pgn(Pgn::CCVS).await?;
if let Some(ccvs) = decode_ccvs(&data) {
    // ccvs.vehicle_speed (Option<f64>), ccvs.brake_switch (Option<bool>)
}

// Read temperatures (PGN 65262 — ET1)
let data = session.read_j1939_pgn(Pgn::ET1).await?;
if let Some(et1) = decode_et1(&data) {
    // et1.coolant_temp, et1.fuel_temp, et1.oil_temp — all Option<f64>
}

// Read J1939 DTCs (SPN+FMI format, not P-codes)
let j1939_dtcs = session.read_j1939_dtcs().await?;
for dtc in &j1939_dtcs {
    // dtc.spn, dtc.fmi, dtc.fmi_description(), dtc.occurrence_count
}
```

### Diagnostic Intelligence (requires matched spec)

```rust
if let Some(spec) = session.spec() {
    // Find diagnostic rules triggered by current DTCs
    let rules = active_rules(&dtcs, Some(spec));
    for rule in rules {
        // rule.name, rule.description, rule.trigger, rule.action
    }

    // Find known issues matching current symptoms
    let issues = matching_issues(&dtcs, Some(spec));
    for issue in issues {
        // issue.name, issue.rank, issue.root_cause, issue.fix
        // issue.quick_test — Optional diagnostic step
    }

    // Threshold evaluation (returns None if value is in normal range)
    if let Some(result) = session.evaluate_threshold(Pid::COOLANT_TEMP, 110.0) {
        // result.level: Warning | Critical
        // result.message: human-readable description
    }

    // DTC library lookup
    if let Some(entry) = spec.dtc_library.as_ref().and_then(|lib| lib.lookup("P0087")) {
        // entry.meaning, entry.severity, entry.related_pids, entry.notes
    }
}
```

---

## Key Types Reference

### Pid — Newtype over u8

```rust
pub struct Pid(pub u8);

// Named constants (not enum variants — use == not match)
Pid::ENGINE_RPM          // 0x0C
Pid::VEHICLE_SPEED       // 0x0D
Pid::COOLANT_TEMP        // 0x05
Pid::ENGINE_LOAD         // 0x04
Pid::THROTTLE_POSITION   // 0x11
Pid::MAF                 // 0x10
Pid::INTAKE_MAP          // 0x0B
Pid::ENGINE_OIL_TEMP     // 0x5C
Pid::ENGINE_FUEL_RATE    // 0x5E
Pid::BAROMETRIC_PRESSURE // 0x33
// ... ~50 constants total

pid.name()  // "Engine RPM"
pid.unit()  // "RPM"
pid.0       // raw u8 code (0x0C)
```

### Reading — Decoded sensor value

```rust
pub struct Reading {
    pub value: Value,         // Scalar(f64) | Bitfield | State(String) | Raw(Vec<u8>)
    pub unit: &'static str,   // "RPM", "°C", "kPa", "%"
    pub timestamp: Instant,
    pub raw_bytes: Vec<u8>,
    pub source: ReadingSource, // Live | FreezeFrame | Replay
}

// Extract float value:
let rpm: f64 = reading.value.as_f64()?;
```

### Dtc — Diagnostic Trouble Code

```rust
pub struct Dtc {
    pub code: String,                    // "P0087"
    pub category: DtcCategory,          // Powertrain | Chassis | Body | Network
    pub status: DtcStatus,              // Stored | Pending | Permanent
    pub description: Option<String>,    // Populated after enrich_dtcs()
    pub severity: Option<Severity>,     // Critical | High | Medium | Low | Info
    pub source_module: Option<String>,
    pub notes: Option<String>,
}
```

### AdapterInfo — Detected hardware

```rust
pub struct AdapterInfo {
    pub chipset: Chipset,          // Elm327Clone | Elm327Genuine | Stn | Unknown
    pub firmware: String,          // "ELM327 v2.1"
    pub protocol: Protocol,        // Can11Bit500 | J1850Vpw | etc.
    pub capabilities: Capabilities,
}

pub struct Capabilities {
    pub can_clear_dtcs: bool,
    pub dual_can: bool,
    pub enhanced_diag: bool,
    pub battery_voltage: bool,
    pub adaptive_timing: bool,
}
```

### MockAdapter — For testing

```rust
let adapter = MockAdapter::new();              // default Duramax VIN
let adapter = MockAdapter::with_vin("1GC...");  // custom VIN
adapter.set_dtcs(vec![...]);                    // inject fault codes

// Static values only: RPM 680, Coolant 50°C, Speed 0, Load 25%
// No warmup simulation or dynamic behavior
```

---

## Storage Traits — Implement in HaulLogic

obd2-core defines storage traits; HaulLogic provides the implementation (Postgres, SQLite, cloud, etc.).

```rust
#[async_trait]
pub trait VehicleStore: Send + Sync {
    async fn save_vehicle(&self, profile: &VehicleProfile) -> Result<(), Obd2Error>;
    async fn get_vehicle(&self, vin: &str) -> Result<Option<VehicleProfile>, Obd2Error>;
    async fn save_thresholds(&self, vin: &str, thresholds: &ThresholdSet) -> Result<(), Obd2Error>;
    async fn get_thresholds(&self, vin: &str) -> Result<Option<ThresholdSet>, Obd2Error>;
}

#[async_trait]
pub trait SessionStore: Send + Sync {
    async fn save_reading(&self, vin: &str, pid: Pid, reading: &Reading) -> Result<(), Obd2Error>;
    async fn save_dtc_event(&self, vin: &str, dtcs: &[Dtc]) -> Result<(), Obd2Error>;
}
```

---

## Vehicle Specs

Specs are YAML files that define vehicle-specific data: enhanced PIDs, thresholds, DTC descriptions, diagnostic rules, known issues.

### Embedded specs (compiled in)

With the `embedded-specs` feature, the Duramax spec is always available. It matches by VIN 8th digit + WMI prefix + year range.

### Adding fleet-specific specs

```rust
// Load additional specs at runtime
session.load_spec(Path::new("specs/ford_powerstroke_2020.yaml"))?;
session.load_spec_dir(Path::new("specs/"))?;

// Access the registry
let registry = session.specs();
let spec = registry.match_vin("1GCHK23164F000001");
```

### Spec structure (YAML)

```yaml
spec_version: "1.0"
identity:
  name: "2004.5 Duramax LLY"
  model_years: [2004, 2005]
  makes: ["Chevrolet", "GMC"]
  vin_match:
    vin_8th_digit: ['2']
    wmi_prefixes: ['1GC', '1GT']
    year_range: [2004, 2005]
  engine:
    code: "LLY"
    displacement_l: 6.6

enhanced_pids:
  - service_id: 0x22
    did: 0x0146
    name: "FICM Voltage"
    unit: "V"
    formula: { Linear: { scale: 0.1, offset: 0 } }
    bytes: 2
    module: "ficm"
    confidence: "Community"

thresholds:
  coolant_temp:
    warning_high: 105
    critical_high: 115
  engine_oil_temp:
    warning_high: 120
    critical_high: 135

diagnostic_rules:
  - name: "Fuel Rail Pressure Check"
    trigger: { DtcPresent: "P0087" }
    action: { CheckFirst: { pid: 0x0193, module: "ecm", reason: "Verify fuel rail pressure" } }

known_issues:
  - rank: 1
    name: "Injector failure"
    symptoms: ["P0201", "P0202", "P0263"]
    root_cause: "Cracked injector body"
    fix: "Replace affected injector(s)"
```

---

## Concurrency Model

- `Session` is NOT thread-safe (`&mut self` on all methods)
- One session per vehicle/adapter — owned by a single task
- Communicate via channels (mpsc) to UI/business logic
- Polling has exclusive adapter access — no concurrent reads
- All I/O is async (tokio)

### Recommended pattern for fleet (multiple vehicles)

```rust
// One tokio task per vehicle
for truck in fleet {
    let tx = event_tx.clone();
    tokio::spawn(async move {
        let transport = BleTransport::scan_and_connect(Some(&truck.adapter_name), timeout).await?;
        let adapter = Elm327Adapter::new(Box::new(transport));
        let mut session = Session::new(adapter);
        let profile = session.identify_vehicle().await?;

        loop {
            let readings = session.read_pids(&priority_pids).await?;
            tx.send(FleetEvent::Readings { vin: profile.vin.clone(), readings }).await?;
            tokio::time::sleep(poll_interval).await;
        }
    });
}
```

---

## Error Handling

```rust
pub enum Obd2Error {
    Transport(String),       // Connection lost, write failed
    Adapter(String),         // Protocol error, garbled response
    AdapterBusy,             // Stop polling before manual reads
    Timeout,                 // Vehicle didn't respond (retried once with 2x timeout)
    NoData,                  // PID not supported (not retried)
    UnsupportedPid { pid: u8 },
    ModuleNotFound(String),
    NegativeResponse { service: u8, nrc: NegativeResponse },
    SecurityRequired,        // Need security access first
    NoSpec,                  // No spec matched for this vehicle
    BusNotAvailable(String),
    SpecParse(String),
    ParseError(String),
    Io(std::io::Error),
    Other(Box<dyn Error + Send + Sync>),
}
```

Key behaviors:
- `NoData` → PID not supported, mark unsupported for session, don't retry
- `Timeout` → Retried once with doubled timeout
- `NegativeResponse(ResponsePending)` → Waits up to 5s, polling every 100ms
- Single PID failure does NOT stop a polling loop
- On disconnect, all requests return `Transport` error — no auto-reconnect

---

## Business Rules (from obd2-core design)

These are enforced by the library — HaulLogic doesn't need to implement them:

- **BR-5:** Thresholds: VIN-specific > spec > engine family > none
- **BR-7:** Actuator control requires 3-step: session > security > control
- **BR-9:** Session is 1:1 with adapter. Init: initialize → supported_pids → identify_vehicle
- **BR-10:** NoData not retried. Timeout retried once. Garbled data not retried.
- **BR-13:** All values in SI/metric (°C, kPa, km/h, L). Display conversion is consumer's job.
- **BR-14:** NHTSA has 5s timeout, failure never hard error, cached per session.
- **BR-16:** Library uses `tracing` — MUST NOT configure a subscriber. Consumer configures their own.

---

## Lessons from obd2-dash Migration

These patterns worked well during the obd2-dash integration:

1. **Create a VehicleData aggregator** — obd2-core returns individual `Reading` values per PID. If your UI needs a flat struct with all current values, build a `VehicleData` struct with an `apply_reading(pid, &reading)` method that maps `Pid` constants to named fields.

2. **Enrich DTCs at read time** — Call `enrich_dtcs(&mut dtcs, session.spec())` immediately after `session.read_dtcs()` to populate descriptions and severity from the matched spec.

3. **Cache enhanced PID list once** — After `identify_vehicle()`, grab `session.spec().enhanced_pids.clone()` and reuse it. Don't re-query the spec every poll cycle.

4. **Poll enhanced PIDs less frequently** — Standard PIDs every tick, enhanced PIDs every 5th tick, O2 monitoring every 20th tick. Enhanced reads require header switching which is slower.

5. **Mock mode uses static values** — `MockAdapter` returns fixed data (RPM 680, Coolant 50°C). Use `MockAdapter::with_vin("...")` to match a specific vehicle spec for testing. J1939 PGNs are also mocked.

6. **BLE scanning is async** — `BleTransport::scan_and_connect()` blocks for the scan duration. Run in a spawned task, not on the UI thread.

7. **BLE adapter name matching is public** — `obd2_core::transport::ble::is_adapter_match()` and `ADAPTER_NAME_PATTERNS` are available for building custom scanner UIs. Use these to filter discovered BLE devices rather than maintaining a separate pattern list.

8. **VIN decoding is built-in** — After `identify_vehicle()`, `profile.decoded_vin` contains manufacturer name, model year, and vehicle class (diesel-truck, gas-truck-v8, suv, sedan, performance). No need for an external VIN decoder.

9. **DTC descriptions are built-in** — obd2-core includes ~200 SAE J2012 DTC descriptions, auto-populated on `Dtc::from_bytes()` / `Dtc::from_code()`. Call `enrich_dtcs()` to layer vehicle-specific descriptions, severity, and notes from the matched spec. No need for a separate DTC description table.
