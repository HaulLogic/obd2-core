//! Vehicle specification types and loading.

pub mod loader;
#[cfg(feature = "nhtsa")]
pub mod nhtsa;
pub mod vin;

use serde::Deserialize;

// ── Module Identity (protocol-agnostic) ──

/// Logical identifier for a vehicle module. String-based for extensibility.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Deserialize)]
pub struct ModuleId(pub String);

impl ModuleId {
    pub const ECM: &'static str = "ecm";
    pub const TCM: &'static str = "tcm";
    pub const BCM: &'static str = "bcm";
    pub const ABS: &'static str = "abs";
    pub const IPC: &'static str = "ipc";
    pub const AIRBAG: &'static str = "airbag";
    pub const HVAC: &'static str = "hvac";
    pub const FICM: &'static str = "ficm";

    pub fn new(id: impl Into<String>) -> Self {
        Self(id.into())
    }
}

// ── Physical Addressing ──

#[derive(Debug, Clone, Deserialize)]
#[non_exhaustive]
pub enum PhysicalAddress {
    J1850 { node: u8, header: [u8; 3] },
    Can11Bit { request_id: u16, response_id: u16 },
    Can29Bit { request_id: u32, response_id: u32 },
    J1939 { source_address: u8 },
}

// ── Protocol ──

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[non_exhaustive]
pub enum Protocol {
    J1850Vpw,
    J1850Pwm,
    Iso9141(KLineInit),
    Kwp2000(KLineInit),
    Can11Bit500,
    Can11Bit250,
    Can29Bit500,
    Can29Bit250,
    Auto,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
pub enum KLineInit {
    SlowInit,
    FastInit,
}

// ── Bus Configuration ──

#[derive(Debug, Clone, PartialEq, Eq, Hash, Deserialize)]
pub struct BusId(pub String);

#[derive(Debug, Clone, Deserialize)]
pub struct BusConfig {
    pub id: BusId,
    pub protocol: Protocol,
    pub speed_bps: u32,
    #[serde(default)]
    pub modules: Vec<Module>,
    pub description: Option<String>,
}

// ── Module ──

#[derive(Debug, Clone, Deserialize)]
pub struct Module {
    pub id: ModuleId,
    pub name: String,
    pub address: PhysicalAddress,
    pub bus: BusId,
}

// ── Vehicle Spec ──

#[derive(Debug, Clone, Deserialize)]
pub struct VehicleSpec {
    pub spec_version: Option<String>,
    pub identity: SpecIdentity,
    pub communication: CommunicationSpec,
    pub thresholds: Option<ThresholdSet>,
    #[serde(default)]
    pub polling_groups: Vec<PollingGroup>,
    #[serde(default)]
    pub diagnostic_rules: Vec<DiagnosticRule>,
    #[serde(default)]
    pub known_issues: Vec<KnownIssue>,
    pub dtc_library: Option<DtcLibrary>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SpecIdentity {
    pub name: String,
    pub model_years: (u16, u16),
    pub makes: Vec<String>,
    pub models: Vec<String>,
    pub engine: EngineSpec,
    pub transmission: Option<TransmissionSpec>,
    pub vin_match: Option<VinMatcher>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct VinMatcher {
    pub vin_8th_digit: Option<Vec<char>>,
    #[serde(default)]
    pub wmi_prefixes: Vec<String>,
    pub year_range: Option<(u16, u16)>,
}

impl VinMatcher {
    /// Check if this matcher matches a given VIN.
    pub fn matches(&self, vin: &str) -> bool {
        if vin.len() < 17 {
            return false;
        }
        let chars: Vec<char> = vin.chars().collect();

        // Check WMI prefix (first 3 chars)
        let wmi: String = chars[..3].iter().collect();
        let wmi_ok = self.wmi_prefixes.is_empty()
            || self.wmi_prefixes.iter().any(|p| wmi.eq_ignore_ascii_case(p));

        // Check 8th digit (engine code)
        let digit_ok = self
            .vin_8th_digit
            .as_ref()
            .map(|digits| digits.contains(&chars[7]))
            .unwrap_or(true);

        // Check year range (decode from 10th char)
        let year_ok = self
            .year_range
            .as_ref()
            .map(|(_min, _max)| {
                // Simple year decode from 10th char (would use vin.rs in practice)
                true // placeholder — real impl in Task 7
            })
            .unwrap_or(true);

        wmi_ok && digit_ok && year_ok
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct EngineSpec {
    pub code: String,
    pub displacement_l: f64,
    pub cylinders: u8,
    pub layout: String,
    pub aspiration: String,
    pub fuel_type: String,
    pub fuel_system: Option<String>,
    pub compression_ratio: Option<f64>,
    pub max_power_kw: Option<f64>,
    pub max_torque_nm: Option<f64>,
    pub redline_rpm: u16,
    pub idle_rpm_warm: u16,
    pub idle_rpm_cold: u16,
    pub firing_order: Option<Vec<u8>>,
    pub ecm_hardware: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct TransmissionSpec {
    pub model: String,
    pub transmission_type: TransmissionType,
    pub fluid_capacity_l: Option<f64>,
}

#[derive(Debug, Clone, Deserialize)]
#[non_exhaustive]
pub enum TransmissionType {
    Geared {
        speeds: u8,
        gear_ratios: Vec<(String, f64)>,
    },
    Cvt {
        ratio_range: (f64, f64),
        simulated_steps: Option<u8>,
    },
    Dct {
        speeds: u8,
        gear_ratios: Vec<(String, f64)>,
    },
    Manual {
        speeds: u8,
        gear_ratios: Vec<(String, f64)>,
    },
}

#[derive(Debug, Clone, Deserialize)]
pub struct CommunicationSpec {
    pub buses: Vec<BusConfig>,
    pub elm327_protocol_code: Option<String>,
}

// ── Thresholds ──

#[derive(Debug, Clone, Deserialize)]
pub struct Threshold {
    pub min: Option<f64>,
    pub max: Option<f64>,
    pub warning_low: Option<f64>,
    pub warning_high: Option<f64>,
    pub critical_low: Option<f64>,
    pub critical_high: Option<f64>,
    pub unit: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum AlertLevel {
    Normal,
    Warning,
    Critical,
}

#[derive(Debug, Clone, Copy)]
pub enum AlertDirection {
    Low,
    High,
}

#[derive(Debug, Clone)]
pub struct ThresholdResult {
    pub level: AlertLevel,
    pub reading: f64,
    pub limit: f64,
    pub direction: AlertDirection,
    pub message: String,
}

impl Threshold {
    /// Evaluate a reading against this threshold.
    pub fn evaluate(&self, value: f64, name: &str) -> Option<ThresholdResult> {
        // Check critical first (highest priority)
        if let Some(limit) = self.critical_high {
            if value >= limit {
                return Some(ThresholdResult {
                    level: AlertLevel::Critical,
                    reading: value,
                    limit,
                    direction: AlertDirection::High,
                    message: format!(
                        "{} critically high: {:.1} >= {:.1} {}",
                        name, value, limit, self.unit
                    ),
                });
            }
        }
        if let Some(limit) = self.critical_low {
            if value <= limit {
                return Some(ThresholdResult {
                    level: AlertLevel::Critical,
                    reading: value,
                    limit,
                    direction: AlertDirection::Low,
                    message: format!(
                        "{} critically low: {:.1} <= {:.1} {}",
                        name, value, limit, self.unit
                    ),
                });
            }
        }
        // Then warning
        if let Some(limit) = self.warning_high {
            if value >= limit {
                return Some(ThresholdResult {
                    level: AlertLevel::Warning,
                    reading: value,
                    limit,
                    direction: AlertDirection::High,
                    message: format!(
                        "{} warning high: {:.1} >= {:.1} {}",
                        name, value, limit, self.unit
                    ),
                });
            }
        }
        if let Some(limit) = self.warning_low {
            if value <= limit {
                return Some(ThresholdResult {
                    level: AlertLevel::Warning,
                    reading: value,
                    limit,
                    direction: AlertDirection::Low,
                    message: format!(
                        "{} warning low: {:.1} <= {:.1} {}",
                        name, value, limit, self.unit
                    ),
                });
            }
        }
        None // normal
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct ThresholdSet {
    #[serde(default)]
    pub engine: Vec<NamedThreshold>,
    #[serde(default)]
    pub transmission: Vec<NamedThreshold>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct NamedThreshold {
    pub name: String,
    pub threshold: Threshold,
}

// ── Polling Groups ──

#[derive(Debug, Clone, Deserialize)]
pub struct PollingGroup {
    pub name: String,
    pub description: String,
    pub target_interval_ms: u32,
    pub steps: Vec<PollStep>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct PollStep {
    pub target: String, // "broadcast" or module id
    #[serde(default)]
    pub standard_pids: Vec<u8>, // PID codes
    #[serde(default)]
    pub enhanced_pids: Vec<u16>, // DID values
}

// ── Diagnostic Rules ──

#[derive(Debug, Clone, Deserialize)]
pub struct DiagnosticRule {
    pub name: String,
    pub trigger: RuleTrigger,
    pub action: RuleAction,
    pub description: String,
}

#[derive(Debug, Clone, Deserialize)]
#[non_exhaustive]
pub enum RuleTrigger {
    DtcPresent(String),
    DtcRange(String, String),
}

#[derive(Debug, Clone, Deserialize)]
#[non_exhaustive]
pub enum RuleAction {
    QueryModule { module: String, service: u8 },
    CheckFirst { pid: u16, module: String, reason: String },
    Alert(String),
    MonitorPids(Vec<u16>),
}

// ── Known Issues ──

#[derive(Debug, Clone, Deserialize)]
pub struct KnownIssue {
    pub rank: u8,
    pub name: String,
    pub description: String,
    pub symptoms: Vec<String>,
    pub root_cause: String,
    pub quick_test: Option<QuickTest>,
    pub fix: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct QuickTest {
    pub description: String,
    pub pass_criteria: String,
}

// ── Vehicle Profile (resolved identity + spec) ──

#[derive(Debug, Clone)]
pub struct VehicleProfile {
    pub vin: String,
    pub info: Option<crate::protocol::service::VehicleInfo>,
    pub spec: Option<VehicleSpec>,
    pub supported_pids: std::collections::HashSet<crate::protocol::pid::Pid>,
}

// ── DTC Library ──

#[derive(Debug, Clone, Default, Deserialize)]
pub struct DtcLibrary {
    #[serde(default)]
    pub ecm: Vec<DtcEntry>,
    #[serde(default)]
    pub tcm: Vec<DtcEntry>,
    #[serde(default)]
    pub bcm: Vec<DtcEntry>,
    #[serde(default)]
    pub network: Vec<DtcEntry>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct DtcEntry {
    pub code: String,
    pub meaning: String,
    pub severity: crate::protocol::dtc::Severity,
    pub notes: Option<String>,
    pub related_pids: Option<Vec<u16>>,
    pub category: Option<String>,
}

impl DtcLibrary {
    pub fn lookup(&self, code: &str) -> Option<&DtcEntry> {
        self.ecm
            .iter()
            .chain(self.tcm.iter())
            .chain(self.bcm.iter())
            .chain(self.network.iter())
            .find(|e| e.code == code)
    }
}

// ── Spec Registry ──

/// Registry of loaded vehicle specs. Matches specs to vehicles by VIN.
pub struct SpecRegistry {
    specs: Vec<VehicleSpec>,
}

impl SpecRegistry {
    /// Create an empty registry.
    pub fn new() -> Self {
        Self { specs: Vec::new() }
    }

    /// Create with embedded default specs.
    pub fn with_defaults() -> Self {
        let specs = crate::specs::embedded::load_embedded_specs();
        Self { specs }
    }

    /// Load a spec from a YAML file.
    pub fn load_file(&mut self, path: &std::path::Path) -> Result<(), crate::error::Obd2Error> {
        let spec = loader::load_spec_from_file(path)?;
        self.specs.push(spec);
        Ok(())
    }

    /// Load all YAML specs from a directory.
    pub fn load_directory(
        &mut self,
        dir: &std::path::Path,
    ) -> Result<usize, crate::error::Obd2Error> {
        let mut count = 0;
        if let Ok(entries) = std::fs::read_dir(dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.extension().map_or(false, |e| e == "yaml" || e == "yml") {
                    if self.load_file(&path).is_ok() {
                        count += 1;
                    }
                }
            }
        }
        Ok(count)
    }

    /// Match a spec to a VIN using VinMatcher rules.
    pub fn match_vin(&self, vin: &str) -> Option<&VehicleSpec> {
        self.specs.iter().find(|s| {
            s.identity
                .vin_match
                .as_ref()
                .map_or(false, |m| m.matches(vin))
        })
    }

    /// Match by make, model, and year.
    pub fn match_vehicle(&self, make: &str, model: &str, year: u16) -> Option<&VehicleSpec> {
        self.specs.iter().find(|s| {
            let year_ok = year >= s.identity.model_years.0 && year <= s.identity.model_years.1;
            let make_ok = s
                .identity
                .makes
                .iter()
                .any(|m| m.eq_ignore_ascii_case(make));
            let model_ok = s
                .identity
                .models
                .iter()
                .any(|m| m.eq_ignore_ascii_case(model));
            year_ok && make_ok && model_ok
        })
    }

    /// List all loaded specs.
    pub fn specs(&self) -> &[VehicleSpec] {
        &self.specs
    }

    /// Look up a DTC across all loaded spec DTC libraries.
    pub fn lookup_dtc(&self, code: &str) -> Option<&DtcEntry> {
        for spec in &self.specs {
            if let Some(lib) = &spec.dtc_library {
                if let Some(entry) = lib.lookup(code) {
                    return Some(entry);
                }
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_module_id_constants() {
        let ecm = ModuleId::new(ModuleId::ECM);
        let also_ecm = ModuleId::new("ecm");
        assert_eq!(ecm, also_ecm);
    }

    #[test]
    fn test_module_id_custom() {
        let vsa = ModuleId::new("vsa");
        assert_eq!(vsa.0, "vsa");
    }

    #[test]
    fn test_physical_address_variants() {
        let j1850 = PhysicalAddress::J1850 {
            node: 0x10,
            header: [0x6C, 0x10, 0xF1],
        };
        let can = PhysicalAddress::Can11Bit {
            request_id: 0x7E0,
            response_id: 0x7E8,
        };
        assert!(matches!(j1850, PhysicalAddress::J1850 { .. }));
        assert!(matches!(can, PhysicalAddress::Can11Bit { .. }));
    }

    #[test]
    fn test_transmission_type_cvt() {
        let cvt = TransmissionType::Cvt {
            ratio_range: (0.4, 2.6),
            simulated_steps: Some(7),
        };
        assert!(matches!(cvt, TransmissionType::Cvt { .. }));
    }

    #[test]
    fn test_transmission_type_geared() {
        let geared = TransmissionType::Geared {
            speeds: 5,
            gear_ratios: vec![("1st".into(), 3.10), ("2nd".into(), 1.81)],
        };
        assert!(matches!(geared, TransmissionType::Geared { speeds: 5, .. }));
    }

    #[test]
    fn test_threshold_evaluate_normal() {
        let t = Threshold {
            min: Some(0.0),
            max: Some(120.0),
            warning_low: None,
            warning_high: Some(105.0),
            critical_low: None,
            critical_high: Some(115.0),
            unit: "\u{00B0}C".into(),
        };
        assert!(t.evaluate(90.0, "coolant").is_none());
    }

    #[test]
    fn test_threshold_evaluate_warning_high() {
        let t = Threshold {
            min: Some(0.0),
            max: Some(120.0),
            warning_low: None,
            warning_high: Some(105.0),
            critical_low: None,
            critical_high: Some(115.0),
            unit: "\u{00B0}C".into(),
        };
        let result = t.evaluate(110.0, "coolant");
        assert!(result.is_some());
        assert_eq!(result.unwrap().level, AlertLevel::Warning);
    }

    #[test]
    fn test_threshold_evaluate_critical_high() {
        let t = Threshold {
            min: Some(0.0),
            max: Some(120.0),
            warning_low: None,
            warning_high: Some(105.0),
            critical_low: None,
            critical_high: Some(115.0),
            unit: "\u{00B0}C".into(),
        };
        let result = t.evaluate(118.0, "coolant");
        assert!(result.is_some());
        assert_eq!(result.unwrap().level, AlertLevel::Critical);
    }

    #[test]
    fn test_threshold_evaluate_warning_low() {
        let t = Threshold {
            min: Some(0.0),
            max: Some(500.0),
            warning_low: Some(100.0),
            warning_high: None,
            critical_low: Some(70.0),
            critical_high: None,
            unit: "kPa".into(),
        };
        let result = t.evaluate(90.0, "oil_pressure");
        assert!(result.is_some());
        assert_eq!(result.unwrap().level, AlertLevel::Warning);
    }

    #[test]
    fn test_threshold_evaluate_critical_low() {
        let t = Threshold {
            min: Some(0.0),
            max: Some(500.0),
            warning_low: Some(100.0),
            warning_high: None,
            critical_low: Some(70.0),
            critical_high: None,
            unit: "kPa".into(),
        };
        let result = t.evaluate(60.0, "oil_pressure");
        assert!(result.is_some());
        assert_eq!(result.unwrap().level, AlertLevel::Critical);
    }

    #[test]
    fn test_alert_level_ordering() {
        assert!(AlertLevel::Critical > AlertLevel::Warning);
        assert!(AlertLevel::Warning > AlertLevel::Normal);
    }

    #[test]
    fn test_vin_matcher_matches() {
        let matcher = VinMatcher {
            vin_8th_digit: Some(vec!['2']),
            wmi_prefixes: vec!["1GC".into()],
            year_range: None,
        };
        assert!(matcher.matches("1GCHK23224F000001")); // WMI=1GC, 8th='2'
    }

    #[test]
    fn test_vin_matcher_wrong_digit() {
        let matcher = VinMatcher {
            vin_8th_digit: Some(vec!['2']),
            wmi_prefixes: vec!["1GC".into()],
            year_range: None,
        };
        assert!(!matcher.matches("1GCHK23114F000001")); // 8th='1', not '2'
    }

    #[test]
    fn test_vin_matcher_wrong_wmi() {
        let matcher = VinMatcher {
            vin_8th_digit: Some(vec!['2']),
            wmi_prefixes: vec!["1GC".into()],
            year_range: None,
        };
        assert!(!matcher.matches("1FTHK23124F000001")); // WMI=1FT (Ford)
    }

    #[test]
    fn test_vin_matcher_short_vin() {
        let matcher = VinMatcher {
            vin_8th_digit: None,
            wmi_prefixes: vec![],
            year_range: None,
        };
        assert!(!matcher.matches("SHORT")); // < 17 chars
    }

    #[test]
    fn test_dtc_library_lookup() {
        let lib = DtcLibrary {
            ecm: vec![DtcEntry {
                code: "P0087".into(),
                meaning: "Fuel Rail Pressure Too Low".into(),
                severity: crate::protocol::dtc::Severity::Critical,
                notes: None,
                related_pids: None,
                category: None,
            }],
            tcm: vec![],
            bcm: vec![],
            network: vec![],
        };
        assert!(lib.lookup("P0087").is_some());
        assert_eq!(
            lib.lookup("P0087").unwrap().meaning,
            "Fuel Rail Pressure Too Low"
        );
        assert!(lib.lookup("P9999").is_none());
    }

    #[test]
    fn test_load_embedded_duramax() {
        let registry = SpecRegistry::with_defaults();
        assert!(
            !registry.specs().is_empty(),
            "should have at least one embedded spec"
        );
        let spec = &registry.specs()[0];
        assert_eq!(spec.identity.engine.code, "LLY");
    }

    #[test]
    fn test_registry_match_vin_duramax() {
        let registry = SpecRegistry::with_defaults();
        let matched = registry.match_vin("1GCHK23224F000001");
        assert!(matched.is_some(), "should match Duramax by VIN");
        assert_eq!(matched.unwrap().identity.engine.code, "LLY");
    }

    #[test]
    fn test_registry_no_match() {
        let registry = SpecRegistry::with_defaults();
        let matched = registry.match_vin("JH4KA7660PC000001");
        assert!(matched.is_none(), "Acura should not match any spec");
    }

    #[test]
    fn test_registry_match_vehicle() {
        let registry = SpecRegistry::with_defaults();
        let matched = registry.match_vehicle("Chevrolet", "Silverado 2500HD", 2004);
        assert!(matched.is_some());
    }

    #[test]
    fn test_load_from_str_invalid() {
        let result = loader::load_spec_from_str("not: [valid: yaml: spec");
        assert!(result.is_err());
    }

    #[test]
    fn test_spec_has_thresholds() {
        let registry = SpecRegistry::with_defaults();
        let spec = &registry.specs()[0];
        assert!(
            spec.thresholds.is_some(),
            "Duramax spec should have thresholds"
        );
    }

    #[test]
    fn test_spec_has_known_issues() {
        let registry = SpecRegistry::with_defaults();
        let spec = &registry.specs()[0];
        assert!(
            !spec.known_issues.is_empty(),
            "Duramax spec should have known issues"
        );
    }
}
