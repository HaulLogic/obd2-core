# J1939 Support Change List

**Date:** 2026-04-03  
**Status:** Proposed

## Required Code Changes

1. Add a shared protocol capability type that explicitly reports live J1939 support.
2. Add session-level J1939 read methods.
3. Add or expose a lower-level adapter/raw-frame mechanism needed to implement those methods.
4. Keep typed PGN decoders in the core library.
5. Add tests for PGN reads, decoder correctness, unsupported cases, and capabilities.

## Likely Affected Areas

- `src/obd2/mod.rs`
- `src/obd2/elm327.rs`
- `src/obd2/transport.rs`
- `src/obd2/types.rs`
- `src/obd2/mock.rs`
- any session-oriented API module if/when restored or introduced

## Consumer-Facing Changes

Consumers should be able to do something equivalent to:

```rust
let caps = session.protocol_capabilities();
if caps.supports_j1939_reads {
    let data = session.read_j1939_pgn(Pgn::EEC1).await?;
    let decoded = decode_eec1(&data);
}
```

## Documentation Changes

1. Update `README.md` so J1939 claims match the actually shipped API.
2. Update `docs/INTEGRATION.md` with the real J1939 lifecycle and examples once implemented.
3. Keep the heavy-duty support status explicit until live PGN support is genuinely available.

## Validation / Release Gates

1. Mock and unit tests passing
2. At least one live heavy-duty adapter validated
3. Documentation updated to match shipped API
4. Capability reporting available so downstream apps can degrade cleanly
