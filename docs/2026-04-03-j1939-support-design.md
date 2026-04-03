# J1939 Support Design

**Date:** 2026-04-03  
**Status:** Proposed

## Summary

The missing piece in `obd2-core` is a shared live J1939 I/O path, not just standalone decode helpers.

The design should add J1939 support in three layers:

1. transport / adapter capability
2. session API
3. typed decode + capability reporting

## Proposed Surface

### 1. Capability Types

Add an explicit capability model, for example:

```rust
pub struct ProtocolCapabilities {
    pub supports_standard_obd2: bool,
    pub supports_j1939_reads: bool,
    pub supports_j1939_decoding: bool,
}
```

This should be queryable from the session and/or adapter layer.

### 2. Session API

Add shared J1939 entry points such as:

```rust
impl<A: Adapter> Session<A> {
    pub async fn read_j1939_pgn(&mut self, pgn: Pgn) -> Result<Vec<u8>, Obd2Error>;
    pub async fn read_j1939_pgns(&mut self, pgns: &[Pgn]) -> Result<Vec<(Pgn, Vec<u8>)>, Obd2Error>;
    pub fn protocol_capabilities(&self) -> ProtocolCapabilities;
}
```

The exact method names can change, but downstream consumers need:
- a single-PGN read path
- optional batched reads
- capability introspection

### 3. Adapter / Transport Changes

The current stack appears optimized around OBD-II PID requests. J1939 support should not be bolted onto downstream apps by forcing them to bypass the shared stack.

Preferred approach:

1. add a lower-level raw-frame request surface to the adapter layer
2. implement J1939 request/read behavior centrally
3. keep transport-specific details inside `obd2-core`

If ELM327/STN hardware requires adapter-specific commands for heavy-duty/J1939 reads, that logic should still live in `obd2-core`.

### 4. Decoder Placement

Typed decode helpers should remain in the core library:

```rust
pub fn decode_eec1(data: &[u8]) -> Option<Eec1>;
pub fn decode_ccvs(data: &[u8]) -> Option<Ccvs>;
pub fn decode_et1(data: &[u8]) -> Option<Et1>;
pub fn decode_eflp1(data: &[u8]) -> Option<Eflp1>;
pub fn decode_lfe(data: &[u8]) -> Option<Lfe>;
```

Each decoder should preserve the current “`Option<T>` per field” pattern for unavailable values.

## Phased Implementation

### Phase 1: Shared I/O Surface

- define capability types
- add session-level PGN read methods
- implement one adapter path end-to-end
- add mockable/raw test coverage

### Phase 2: Core Fleet PGNs

- `EEC1`
- `CCVS`
- `ET1`
- `EFL/P1`
- `LFE`

### Phase 3: Heavy-Duty Diagnostics Expansion

- J1939 DTC/DM message support
- heavier capability reporting
- polling helpers / poll-loop integration

## Testing Strategy

1. Unit tests for each decoder
2. Mock session tests for `read_j1939_pgn`
3. Capability tests
4. Live smoke tests with a real heavy-duty-compatible adapter

## Risks

1. Some adapters may claim J1939 compatibility but expose incomplete command support.
2. The current adapter abstraction may need a deeper change than initially expected.
3. If transport/raw-frame behavior remains private to specific adapter implementations, downstream apps will keep re-implementing J1939 support outside the library.

## Recommended Decision

Do the adapter/session expansion in `obd2-core` rather than asking downstream apps to own a private J1939 transport layer. The core problem is shared infrastructure, so the fix should also live in shared infrastructure.
