# VIN scrub convention & capture corpus graduation

**Date:** 2026-07-11  
**Plan phases:** G2 (corpus), privacy for dual-licensed `obd2-core`  
**Status:** Convention + process defined; bulk graduation of partner captures is operator-driven

## Why

Real captures under `obd2-dash/raw-captures/` embed **production VINs in filenames** (Mode 09 and path names).  
`obd2-core` is **published / dual-licensed** — unscrubbed partner fixtures must never land on public remotes.

## Scrub convention

| Field | Rule |
| --- | --- |
| VIN string (17 chars) | Replace with stable fake `1GTESTVINTEST00001` (or other valid-length test VIN) |
| Filenames | Remove real VIN prefix; use archetype + date, e.g. `a1-duramax-2004-j1850-20260627.obd2raw` |
| Session JSON metadata | Scrub any `vin` keys |
| ELM payload Mode 09 | Rewrite VIN ASCII bytes to the same fake VIN (same length) |
| Partner identity | Never commit unscrubbed copies to `obd2-core` public trees |

## Destination layout

```text
obd2-core/raw-captures/
  README.md                 # this process pointer
  fixtures/                 # committed scrubbed golden samples only
  .gitkeep
```

Large partner archives stay in **private** dash workspaces or offline store until scrubbed.

## Graduation checklist (per capture set)

1. Copy candidate into a **private** working directory (not the public tree).  
2. Run scrub (script below or equivalent).  
3. Spot-check: `rg -n '[A-HJ-NPR-Z0-9]{17}' scrubbed/` should only show the fake VIN.  
4. Move scrubbed files into `raw-captures/fixtures/<archetype>/`.  
5. Add a CI replay test that loads the fixture and asserts decoded RPM/speed (or protocol family).  
6. Link fixture + test from hardware matrix / G3 claims table.

## Scrub script (reference)

```bash
# From a private copy of captures (not committed until scrubbed):
# Set REAL_VIN from the private capture source — never write the real VIN
# into this repo, including this doc.
REAL_VIN='<real VIN from private capture source>'
FAKE_VIN='1GTESTVNTEST00001'
find . -type f \( -name '*.obd2raw' -o -name '*.obd2rec' -o -name '*.json' \) -print0 \
  | xargs -0 perl -pi -e "s/\Q$REAL_VIN\E/$FAKE_VIN/g"
# Rename files
for f in *${REAL_VIN}*; do
  mv -- "$f" "${f//$REAL_VIN/a1-duramax-scrubbed}"
done
```

## Known A1 source (do not commit unscrubbed)

`obd2-dash/raw-captures/` (private) — 2004 GMC Sierra 2500HD Duramax Class A1 evidence; capture filenames there embed the real VIN.  
Graduate scrubbed copies only.

## Product claims

Until scrubbed fixtures are in CI **and** live matrix rows exist, do not claim multi-make readiness from dash captures alone.
