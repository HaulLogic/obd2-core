# Embedded vehicle-spec catalog (Phase E1 start)

**Date:** 2026-07-11  
**Location:** `crates/obd2-core/src/specs/embedded/`

| Engine code | Spec file | VIN match | Confidence |
| --- | --- | --- | --- |
| LLY | `chevy_duramax_2004_turbo.yaml` | WMI + 8th=`2` + years 2004–2005 | Existing / field-backed |
| LBZ | `chevy_duramax_lbz.yaml` | WMI + 8th=`D` + years 2006–2007 | Inferred platform |
| GENERIC | `generic_obd2_fallback.yaml` | **none** (never steals VIN matches) | Safe defaults |
| PSTRK67 | `ford_powerstroke_67.yaml` | Ford truck WMI + years 2011–2025 | Inferred scaffold |
| CUMMINS67 | `ram_cummins_67.yaml` | Ram/Dodge WMI + years 2013–2025 | Inferred scaffold |

## Uniqueness rule

`SpecRegistry::match_vin` returns `None` if **two** specs match the same VIN.  
New families must not overlap LLY’s 8th-digit/`year_range` for the default Duramax test VIN.

## Not yet embedded

- LMM / L5P / L5D full communication maps  
- Full Mode 22 catalogs (see program enhanced allowlist)  
- Scrubbed A1 capture fixtures under `raw-captures/fixtures/`
