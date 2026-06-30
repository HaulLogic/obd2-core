//! Diagnostic intelligence — DTC enrichment, rules, known issues.
//!
//! ## DTC Description Pipeline
//!
//! obd2-core provides a two-tier DTC description system:
//!
//! 1. **Universal SAE J2012 table** (~200 codes) — built into [`crate::protocol::dtc`],
//!    auto-populated when DTCs are created via `from_bytes()` or `from_code()`.
//!
//! 2. **Vehicle-specific enrichment** — [`enrich_dtcs`] overlays descriptions,
//!    severity, and technician notes from the matched [`VehicleSpec`]'s DTC library.
//!
//! Consumers do NOT need their own DTC description tables. Call `enrich_dtcs()`
//! after reading DTCs to get the most specific descriptions available.

use crate::protocol::dtc::{universal_dtc_description, Dtc};
use crate::vehicle::{DiagnosticRule, KnownIssue, RuleTrigger, VehicleSpec};

/// Enrich a list of DTCs with descriptions, severity, and notes from the spec.
///
/// Resolution order (BR-4.4):
/// 1. Vehicle spec DTC library (most specific)
/// 2. Universal SAE J2012 descriptions
/// 3. "Unknown DTC" fallback
pub fn enrich_dtcs(dtcs: &mut [Dtc], spec: Option<&VehicleSpec>) {
    for dtc in dtcs.iter_mut() {
        // Try spec DTC library first
        if let Some(spec) = spec {
            if let Some(lib) = &spec.dtc_library {
                if let Some(entry) = lib.lookup(&dtc.code) {
                    dtc.description = Some(entry.meaning.clone());
                    dtc.severity = Some(entry.severity);
                    dtc.notes = entry.notes.clone();
                    continue;
                }
            }
        }

        // Fall back to universal descriptions
        if dtc.description.is_none() {
            dtc.description = universal_dtc_description(&dtc.code).map(String::from);
        }
    }
}

/// Find diagnostic rules that fire for the current set of DTCs.
///
/// Rules fire based on triggers (BR-4.2):
/// - DtcPresent: fires when a specific DTC code is in the list
/// - DtcRange: fires when any DTC in the range is present
pub fn active_rules<'a>(dtcs: &[Dtc], spec: Option<&'a VehicleSpec>) -> Vec<&'a DiagnosticRule> {
    let spec = match spec {
        Some(s) => s,
        None => return vec![],
    };

    spec.diagnostic_rules
        .iter()
        .filter(|rule| match &rule.trigger {
            RuleTrigger::DtcPresent(code) => dtcs.iter().any(|d| d.code == *code),
            RuleTrigger::DtcRange(start, end) => {
                dtcs.iter().any(|d| d.code >= *start && d.code <= *end)
            }
        })
        .collect()
}

/// Find known issues that match current DTCs by symptom codes.
pub fn matching_issues<'a>(dtcs: &[Dtc], spec: Option<&'a VehicleSpec>) -> Vec<&'a KnownIssue> {
    let spec = match spec {
        Some(s) => s,
        None => return vec![],
    };

    let dtc_codes: Vec<&str> = dtcs.iter().map(|d| d.code.as_str()).collect();

    let mut matches: Vec<&KnownIssue> = spec
        .known_issues
        .iter()
        .filter(|issue| {
            issue
                .symptoms
                .iter()
                .any(|symptom| dtc_codes.contains(&symptom.as_str()))
        })
        .collect();

    // Sort by rank (lowest = most common = first)
    matches.sort_by_key(|i| i.rank);
    matches
}

/// Deduplicate DTCs by code, source module, and lifecycle status.
pub fn dedup_dtcs(dtcs: &mut Vec<Dtc>) {
    let mut seen = std::collections::HashSet::new();
    dtcs.retain(|dtc| seen.insert((dtc.code.clone(), dtc.source_module.clone(), dtc.status)));
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::dtc::Severity;
    use crate::vehicle::*;

    fn make_spec_with_dtcs() -> VehicleSpec {
        VehicleSpec {
            spec_version: Some("1.0".into()),
            identity: SpecIdentity {
                name: "Test".into(),
                model_years: (2004, 2005),
                makes: vec!["Chevrolet".into()],
                models: vec!["Silverado".into()],
                engine: EngineSpec {
                    code: "LLY".into(),
                    displacement_l: 6.6,
                    cylinders: 8,
                    layout: "V8".into(),
                    aspiration: "Turbo".into(),
                    fuel_type: "Diesel".into(),
                    fuel_system: None,
                    compression_ratio: None,
                    max_power_kw: None,
                    max_torque_nm: None,
                    redline_rpm: 3250,
                    idle_rpm_warm: 680,
                    idle_rpm_cold: 780,
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
            thresholds: None,
            dtc_library: Some(DtcLibrary {
                ecm: vec![
                    DtcEntry {
                        code: "P0087".into(),
                        meaning: "Fuel Rail Pressure Too Low".into(),
                        severity: Severity::Critical,
                        notes: Some("CP3 pump failure or fuel filter".into()),
                        related_pids: None,
                        category: None,
                    },
                    DtcEntry {
                        code: "P0234".into(),
                        meaning: "Turbo Overboost Condition".into(),
                        severity: Severity::Critical,
                        notes: Some("VGT vanes stuck closed".into()),
                        related_pids: None,
                        category: None,
                    },
                ],
                tcm: vec![],
                bcm: vec![],
                network: vec![],
            }),
            polling_groups: vec![],
            diagnostic_rules: vec![
                DiagnosticRule {
                    name: "P0700 TCM redirect".into(),
                    trigger: RuleTrigger::DtcPresent("P0700".into()),
                    action: RuleAction::QueryModule {
                        module: "tcm".into(),
                        service: 0x03,
                    },
                    description: "P0700 means query TCM directly for real DTCs".into(),
                },
                DiagnosticRule {
                    name: "FICM check".into(),
                    trigger: RuleTrigger::DtcRange("P0201".into(), "P0208".into()),
                    action: RuleAction::CheckFirst {
                        pid: 0x1100,
                        module: "ficm".into(),
                        reason: "Check FICM 48V before condemning injectors".into(),
                    },
                    description: "90% of injector circuit codes are FICM failures".into(),
                },
            ],
            known_issues: vec![
                KnownIssue {
                    rank: 1,
                    name: "turbo_vane_sticking".into(),
                    description: "VGT vanes stick from carbon buildup".into(),
                    symptoms: vec!["P0234".into(), "P0299".into()],
                    root_cause: "Carbon/soot in unison ring".into(),
                    quick_test: Some(QuickTest {
                        description: "Monitor VGT Position Error under load".into(),
                        pass_criteria: "Error < 10%".into(),
                    }),
                    fix: "Remove turbo, clean unison ring".into(),
                },
                KnownIssue {
                    rank: 2,
                    name: "ficm_failure".into(),
                    description: "FICM 48V capacitor bank degradation".into(),
                    symptoms: vec!["P0201".into(), "P0611".into(), "P2146".into()],
                    root_cause: "Internal capacitor failure".into(),
                    quick_test: None,
                    fix: "Replace or rebuild FICM".into(),
                },
            ],
            enhanced_pids: vec![],
        }
    }

    #[test]
    fn test_enrich_from_spec() {
        let spec = make_spec_with_dtcs();
        let mut dtcs = vec![Dtc::from_code("P0087")];
        enrich_dtcs(&mut dtcs, Some(&spec));
        assert_eq!(
            dtcs[0].description.as_deref(),
            Some("Fuel Rail Pressure Too Low")
        );
        assert_eq!(dtcs[0].severity, Some(Severity::Critical));
        assert!(dtcs[0].notes.is_some());
    }

    #[test]
    fn test_enrich_fallback_universal() {
        let spec = make_spec_with_dtcs();
        let mut dtcs = vec![Dtc::from_code("P0420")]; // not in spec, but in universal
        enrich_dtcs(&mut dtcs, Some(&spec));
        assert!(dtcs[0].description.is_some());
        assert!(dtcs[0].description.as_ref().unwrap().contains("Catalyst"));
    }

    #[test]
    fn test_enrich_no_spec() {
        let mut dtcs = vec![Dtc::from_code("P0420")];
        enrich_dtcs(&mut dtcs, None);
        assert!(dtcs[0].description.is_some()); // universal fallback
    }

    #[test]
    fn test_active_rules_p0700() {
        let spec = make_spec_with_dtcs();
        let dtcs = vec![Dtc::from_code("P0700")];
        let rules = active_rules(&dtcs, Some(&spec));
        assert_eq!(rules.len(), 1);
        assert_eq!(rules[0].name, "P0700 TCM redirect");
    }

    #[test]
    fn test_active_rules_injector_range() {
        let spec = make_spec_with_dtcs();
        let dtcs = vec![Dtc::from_code("P0204")]; // within P0201-P0208
        let rules = active_rules(&dtcs, Some(&spec));
        assert_eq!(rules.len(), 1);
        assert_eq!(rules[0].name, "FICM check");
    }

    #[test]
    fn test_active_rules_none() {
        let spec = make_spec_with_dtcs();
        let dtcs = vec![Dtc::from_code("P0420")]; // no rule for this
        let rules = active_rules(&dtcs, Some(&spec));
        assert!(rules.is_empty());
    }

    #[test]
    fn test_matching_issues_turbo() {
        let spec = make_spec_with_dtcs();
        let dtcs = vec![Dtc::from_code("P0234")];
        let issues = matching_issues(&dtcs, Some(&spec));
        assert_eq!(issues.len(), 1);
        assert_eq!(issues[0].name, "turbo_vane_sticking");
    }

    #[test]
    fn test_matching_issues_ficm() {
        let spec = make_spec_with_dtcs();
        let dtcs = vec![Dtc::from_code("P0201")];
        let issues = matching_issues(&dtcs, Some(&spec));
        assert_eq!(issues.len(), 1);
        assert_eq!(issues[0].name, "ficm_failure");
    }

    #[test]
    fn test_matching_issues_multiple() {
        let spec = make_spec_with_dtcs();
        let dtcs = vec![Dtc::from_code("P0234"), Dtc::from_code("P0201")];
        let issues = matching_issues(&dtcs, Some(&spec));
        assert_eq!(issues.len(), 2);
        assert_eq!(issues[0].rank, 1); // sorted by rank
    }

    #[test]
    fn test_matching_issues_no_match() {
        let spec = make_spec_with_dtcs();
        let dtcs = vec![Dtc::from_code("P0420")];
        let issues = matching_issues(&dtcs, Some(&spec));
        assert!(issues.is_empty());
    }

    #[test]
    fn test_dedup_dtcs() {
        let mut dtcs = vec![
            Dtc::from_code("P0420"),
            Dtc::from_code("P0171"),
            Dtc::from_code("P0420"), // duplicate
        ];
        dedup_dtcs(&mut dtcs);
        assert_eq!(dtcs.len(), 2);
    }

    #[test]
    fn test_dedup_dtcs_keeps_distinct_modules() {
        let mut ecm = Dtc::from_code("P0700");
        ecm.source_module = Some("ecm".into());
        let mut tcm = Dtc::from_code("P0700");
        tcm.source_module = Some("tcm".into());
        let mut dtcs = vec![ecm, tcm];

        dedup_dtcs(&mut dtcs);

        assert_eq!(dtcs.len(), 2);
    }
}
