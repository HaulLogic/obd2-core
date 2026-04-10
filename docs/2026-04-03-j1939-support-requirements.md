# J1939 Support Requirements

**Date:** 2026-04-09
**Status:** Scoped, incomplete

## Problem

`obd2-core` currently provides strong standard OBD-II support, but downstream consumers that need real heavy-duty support are blocked by the lack of a shared J1939 transport and session surface.

The main missing capability is not decode logic by itself. It is the ability to perform live PGN requests/reads through the shared connection stack so higher layers can actually consume J1939 data from Class 7-8 vehicles.

## Goal

Make `obd2-core` the shared authority for live J1939 support in the same way it is already the shared authority for standard OBD-II.

## Functional Requirements

1. The library must expose a first-class J1939 read path from the shared connection/session layer.
2. Consumers must be able to request well-known PGNs without bypassing the core library.
3. The library must support at minimum these fleet-relevant PGNs:
   - `EEC1` (61444): engine RPM and torque
   - `CCVS` (65265): vehicle speed, brake, cruise
   - `ET1` (65262): coolant/fuel/oil temperature
   - `EFL/P1` (65263): oil pressure, coolant pressure
   - `LFE` (65266): fuel rate and fuel economy-related data
4. The library must provide a way to surface raw PGN payload bytes for debugging and incremental decoder rollout.
5. The library must expose typed decode helpers for the PGNs above.
6. The library must allow downstream consumers to distinguish:
   - standard OBD-II support
   - J1939 decode availability
   - live J1939 polling availability
7. The library must support heavy-duty vehicle detection without forcing downstream apps to invent their own protocol heuristics.

## API Requirements

1. Session-level APIs should exist for:
   - reading a single PGN
   - optionally reading a batch of PGNs
   - identifying whether the current connection/session supports live J1939 reads
2. The API should expose an explicit J1939 capability contract rather than requiring downstream apps to infer support from adapter type names.
3. Decode helpers should operate on raw payload bytes and return typed structs with `Option<T>` fields for “not available” values.

## Transport / Adapter Requirements

1. The connection stack must expose enough raw frame access to support J1939 reads through the shared library.
2. This should work through the same adapter abstractions already used for ELM327/STN style hardware where possible.
3. If the current adapter abstraction is too OBD-II-specific, the library should introduce a lower-level capability layer rather than pushing J1939 implementation into downstream apps.

## Validation Requirements

1. Mock coverage must exist for the J1939 session path.
2. Live validation must be performed against at least one real heavy-duty-compatible adapter before the feature is treated as production-ready.
3. Tests must cover:
   - PGN read success
   - unsupported/absent PGN handling
   - partial “not available” fields
   - decode correctness for the core PGN set
   - capability reporting

## Non-Goals

1. This work does not need to cover the full SAE J1939 catalog in the first pass.
2. This work does not need to implement every heavy-duty DTC/DM message up front.
3. This work does not need to solve all fleet/business logic that may consume J1939 data.

## Success Criteria

`obd2-core` becomes sufficient for downstream apps to build real heavy-duty engine telemetry on the shared library without maintaining a separate private J1939 transport layer.
