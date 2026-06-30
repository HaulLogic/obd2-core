//! Discovery profile resolution for adapter, protocol, bus, and module routing.

use std::collections::HashMap;

use crate::adapter::{AdapterInfo, ProbeAttempt, ProtocolSelectionSource};
use crate::vehicle::{BusConfig, BusId, Module, ModuleId, PhysicalAddress, Protocol, VehicleSpec};

#[derive(Debug, Clone)]
pub struct DiscoveryProfile {
    pub adapter: AdapterInfo,
    pub selected_protocol: Protocol,
    pub protocol_choice_source: ProtocolSelectionSource,
    pub active_bus: Option<ResolvedBus>,
    pub modules: HashMap<ModuleId, ResolvedModule>,
    pub probe_attempts: Vec<ProbeAttempt>,
    pub visible_ecus: Vec<VisibleEcu>,
}

#[derive(Debug, Clone)]
pub struct ResolvedBus {
    pub id: BusId,
    pub protocol: Protocol,
    pub speed_bps: u32,
    pub description: Option<String>,
}

#[derive(Debug, Clone)]
pub struct ResolvedModule {
    pub id: ModuleId,
    pub name: String,
    pub bus: BusId,
    pub address: PhysicalAddress,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VisibleEcu {
    pub id: String,
    pub bus: Option<BusId>,
    pub module: Option<ModuleId>,
    pub address: Option<PhysicalAddress>,
    pub observation_count: u32,
}

pub fn resolve_discovery_profile(
    adapter: &AdapterInfo,
    spec: Option<&VehicleSpec>,
    probe_attempts: &[ProbeAttempt],
    visible_ecus: &[VisibleEcu],
) -> DiscoveryProfile {
    let selected_protocol = adapter.protocol;
    let active_bus = resolve_active_bus(selected_protocol, spec);
    let modules = resolve_modules(spec, active_bus.as_ref());
    let protocol_choice_source = probe_attempts
        .last()
        .map(|attempt| attempt.source)
        .unwrap_or(ProtocolSelectionSource::AutoDetect);

    DiscoveryProfile {
        adapter: adapter.clone(),
        selected_protocol,
        protocol_choice_source,
        active_bus,
        modules,
        probe_attempts: probe_attempts.to_vec(),
        visible_ecus: visible_ecus.to_vec(),
    }
}

fn resolve_active_bus(protocol: Protocol, spec: Option<&VehicleSpec>) -> Option<ResolvedBus> {
    let spec = spec?;

    let matched = spec
        .communication
        .buses
        .iter()
        .find(|bus| bus.protocol == protocol)
        .or_else(|| {
            if spec.communication.buses.len() == 1 {
                spec.communication.buses.first()
            } else {
                None
            }
        })?;

    Some(to_resolved_bus(matched))
}

fn resolve_modules(
    spec: Option<&VehicleSpec>,
    active_bus: Option<&ResolvedBus>,
) -> HashMap<ModuleId, ResolvedModule> {
    let mut modules = HashMap::new();
    let Some(spec) = spec else { return modules };

    match active_bus {
        Some(active_bus) => {
            if let Some(bus) = spec
                .communication
                .buses
                .iter()
                .find(|bus| bus.id == active_bus.id)
            {
                collect_modules(&mut modules, &bus.modules);
            }
        }
        None => {
            for bus in &spec.communication.buses {
                collect_modules(&mut modules, &bus.modules);
            }
        }
    }

    modules
}

fn collect_modules(target: &mut HashMap<ModuleId, ResolvedModule>, modules: &[Module]) {
    for module in modules {
        target.insert(
            module.id.clone(),
            ResolvedModule {
                id: module.id.clone(),
                name: module.name.clone(),
                bus: module.bus.clone(),
                address: module.address.clone(),
            },
        );
    }
}

fn to_resolved_bus(bus: &BusConfig) -> ResolvedBus {
    ResolvedBus {
        id: bus.id.clone(),
        protocol: bus.protocol,
        speed_bps: bus.speed_bps,
        description: bus.description.clone(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::{AdapterInfo, Capabilities, Chipset};
    use crate::vehicle::{
        CommunicationSpec, EngineSpec, KLineInit, Module, SpecIdentity, VinMatcher,
    };

    fn make_spec() -> VehicleSpec {
        VehicleSpec {
            spec_version: Some("1.0".into()),
            identity: SpecIdentity {
                name: "Discovery Test".into(),
                model_years: (2004, 2005),
                makes: vec!["Chevrolet".into()],
                models: vec!["Silverado".into()],
                engine: EngineSpec {
                    code: "LLY".into(),
                    displacement_l: 6.6,
                    cylinders: 8,
                    layout: "V8".into(),
                    aspiration: "turbo".into(),
                    fuel_type: "diesel".into(),
                    fuel_system: None,
                    compression_ratio: None,
                    max_power_kw: None,
                    max_torque_nm: None,
                    redline_rpm: 3200,
                    idle_rpm_warm: 680,
                    idle_rpm_cold: 780,
                    firing_order: None,
                    ecm_hardware: None,
                },
                transmission: None,
                vin_match: Some(VinMatcher {
                    vin_8th_digit: Some(vec!['2']),
                    wmi_prefixes: vec!["1GC".into()],
                    year_range: Some((2004, 2005)),
                }),
            },
            communication: CommunicationSpec {
                buses: vec![
                    BusConfig {
                        id: BusId("j1850vpw".into()),
                        protocol: Protocol::J1850Vpw,
                        speed_bps: 10400,
                        modules: vec![
                            Module {
                                id: ModuleId::new("ecm"),
                                name: "Engine Control Module".into(),
                                address: PhysicalAddress::J1850 {
                                    node: 0x10,
                                    header: [0x6C, 0x10, 0xF1],
                                },
                                bus: BusId("j1850vpw".into()),
                            },
                            Module {
                                id: ModuleId::new("tcm"),
                                name: "Transmission Control Module".into(),
                                address: PhysicalAddress::J1850 {
                                    node: 0x18,
                                    header: [0x6C, 0x18, 0xF1],
                                },
                                bus: BusId("j1850vpw".into()),
                            },
                        ],
                        description: Some("Class 2".into()),
                    },
                    BusConfig {
                        id: BusId("can".into()),
                        protocol: Protocol::Can11Bit500,
                        speed_bps: 500_000,
                        modules: vec![Module {
                            id: ModuleId::new("abs"),
                            name: "ABS".into(),
                            address: PhysicalAddress::Can11Bit {
                                request_id: 0x760,
                                response_id: 0x768,
                            },
                            bus: BusId("can".into()),
                        }],
                        description: Some("CAN".into()),
                    },
                ],
                elm327_protocol_code: Some("2".into()),
            },
            thresholds: None,
            polling_groups: vec![],
            diagnostic_rules: vec![],
            known_issues: vec![],
            dtc_library: None,
            enhanced_pids: vec![],
        }
    }

    fn adapter_info(protocol: Protocol) -> AdapterInfo {
        AdapterInfo {
            chipset: Chipset::Stn,
            firmware: "STN".into(),
            protocol,
            capabilities: Capabilities {
                can_clear_dtcs: true,
                dual_can: true,
                enhanced_diag: true,
                battery_voltage: true,
                adaptive_timing: true,
                kline_init: true,
                kline_wakeup: true,
                can_filtering: true,
                can_flow_control: true,
                can_extended_addressing: true,
                can_silent_mode: true,
            },
        }
    }

    #[test]
    fn test_resolve_profile_without_spec_has_adapter_only() {
        let profile =
            resolve_discovery_profile(&adapter_info(Protocol::Can11Bit500), None, &[], &[]);
        assert_eq!(profile.selected_protocol, Protocol::Can11Bit500);
        assert!(profile.active_bus.is_none());
        assert!(profile.modules.is_empty());
    }

    #[test]
    fn test_resolve_profile_matches_bus_by_protocol() {
        let spec = make_spec();
        let profile =
            resolve_discovery_profile(&adapter_info(Protocol::J1850Vpw), Some(&spec), &[], &[]);
        assert_eq!(profile.active_bus.as_ref().unwrap().id.0, "j1850vpw");
        assert!(profile.modules.contains_key(&ModuleId::new("ecm")));
        assert!(profile.modules.contains_key(&ModuleId::new("tcm")));
        assert!(!profile.modules.contains_key(&ModuleId::new("abs")));
    }

    #[test]
    fn test_resolve_profile_falls_back_to_single_bus() {
        let mut spec = make_spec();
        spec.communication.buses = vec![BusConfig {
            id: BusId("kline".into()),
            protocol: Protocol::Iso9141(KLineInit::SlowInit),
            speed_bps: 10400,
            modules: vec![],
            description: None,
        }];
        let profile =
            resolve_discovery_profile(&adapter_info(Protocol::Auto), Some(&spec), &[], &[]);
        assert_eq!(profile.active_bus.as_ref().unwrap().id.0, "kline");
    }

    #[test]
    fn test_resolve_profile_collects_all_modules_when_bus_unknown() {
        let spec = make_spec();
        let profile =
            resolve_discovery_profile(&adapter_info(Protocol::Auto), Some(&spec), &[], &[]);
        assert!(profile.active_bus.is_none());
        assert_eq!(profile.modules.len(), 3);
    }
}
