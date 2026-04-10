# J1939 Support Design

**Date:** 2026-04-09
**Status:** Scoped, incomplete

## Summary

J1939 support in `obd2-core` should be a session-owned capability with explicit routing and explicit limits. The goal is not full SAE J1939 coverage on day one. The goal is a stable shared contract for the heavy-duty telemetry and diagnostics surface that downstream apps can actually rely on.

## Supported Boundary

The supported first-pass boundary is:

1. session-owned live J1939 requests
2. broadcast PGN reads
3. directed PGN reads
4. monitor-flow reads
5. DM1 reads
6. transport-protocol reassembly for the requested PGNs
7. typed decode helpers for the core fleet PGNs

The core fleet PGNs remain:

1. `EEC1`
2. `CCVS`
3. `ET1`
4. `EFL/P1`
5. `LFE`

Explicitly out of scope for the first pass:

1. the full SAE J1939 catalog
2. adapter-first J1939 helper APIs
3. downstream-owned private transport layers
4. assuming every OEM-specific DTC/DM message is implemented

## Session API

The session surface should expose J1939 in the same style as the rest of the library:

```rust
impl<A: Adapter> Session<A> {
    pub async fn read_j1939_pgn(&mut self, pgn: Pgn) -> Result<Vec<u8>, Obd2Error>;
    pub async fn read_j1939_pgns(&mut self, pgns: &[Pgn]) -> Result<Vec<(Pgn, Vec<u8>)>, Obd2Error>;
    pub async fn read_j1939_dm1(&mut self) -> Result<Vec<J1939Dtc>, Obd2Error>;
    pub fn j1939_capabilities(&self) -> J1939Capabilities;
}
```

That surface is intentionally narrow:

1. request bytes come from `Session`
2. capability reporting comes from `Session`
3. decode helpers remain in the core library
4. byte payloads remain available for replay and debugging

## Discovery Profile

`DiscoveryProfile` should grow a J1939 sub-profile rather than scattering J1939 state across adapter internals.

Recommended shape:

```rust
pub struct J1939Discovery {
    pub supported: bool,
    pub active_bus: Option<BusId>,
    pub source_addresses: Vec<u8>,
    pub destination_addresses: Vec<u8>,
    pub transport_protocol_supported: bool,
    pub capabilities: J1939Capabilities,
}
```

This keeps the session answerable for:

1. whether J1939 is available
2. where it is available
3. what source addresses were observed
4. whether directed traffic was resolved
5. whether transport-protocol reassembly is available on this path

## Routing Model

J1939 routing should be explicit and physical, not inferred from adapter names or hidden state.

Required routing inputs:

1. source address
2. destination address
3. broadcast destination
4. PGN

Design rules:

1. `Session` resolves logical names to physical J1939 routing when specs support it.
2. Broadcast and directed traffic should remain distinct in the API.
3. Directed routing must fail explicitly when discovery cannot resolve the path.
4. Raw bytes are preserved regardless of whether a decoder exists yet.

## Timeout And Byte Order Policy

J1939 timing and field interpretation need to be explicit.

Policy:

1. Single-frame requests and transport-protocol reassembly use different timeout handling.
2. Timeout behavior is a session-level policy, not an adapter leak.
3. Transport and routing layers remain byte-oriented.
4. Byte order is decoder/SPN-specific, not globally inferred for all J1939 data.
5. Optional or unavailable values should remain `Option<T>` in typed decoders.

## Adapter / Transport Shape

The current adapter stack should be extended only as far as the shared session contract requires.

Preferred layering:

1. `Session` owns the public J1939 API.
2. The adapter layer performs resolved physical I/O.
3. Transport stays byte-only.
4. Hardware-specific J1939 quirks stay inside `obd2-core`.

If an ELM327/STN path needs special handling for J1939, that logic should remain in the core library rather than leaking into consumers.

## Validation Strategy

1. Unit tests for each decoder.
2. Mock session tests for J1939 PGN reads.
3. Tests for capability reporting.
4. Tests for discovery/profile J1939 visibility.
5. Live smoke tests against a real heavy-duty-compatible adapter before production claims.

## Implementation Phasing

### Phase 1

- define capability types
- add the session-level J1939 API
- add J1939 discovery/profile shape
- add mockable coverage

### Phase 2

- implement core fleet PGNs
- implement DM1 helper surface
- implement directed/broadcast routing behavior

### Phase 3

- expand J1939 diagnostic coverage
- add richer capability reporting
- add polling or replay integration if needed

## Recommended Decision

Keep the J1939 expansion in `obd2-core`. The shared infrastructure should own the shared heavy-duty surface, and downstream apps should consume it through `Session`.
