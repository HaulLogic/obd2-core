//! STN/OBDLink backend support.
//!
//! STN devices expose the ELM327 byte protocol plus an `ST` command set. This
//! module keeps the STN-specific route setup explicit and testable while still
//! using the existing byte-level [`Link`](crate::transport::Link) boundary.

use super::backend::{
    BackendCaps, CanIdentifier, CanRouteBus, CanRouteConfig, CanRouteFilter, CapabilityMismatch,
    SecondaryBus, STN_BACKEND_CAPS,
};
use crate::error::Obd2Error;
use crate::transport::Link;
use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum StnProtocolPreset {
    /// STP 33: ISO 15765, 11-bit Tx, high-speed CAN, DLC=8.
    PrimaryCan,
    /// STP 53: ISO 15765, 11-bit Tx, Ford MS-CAN transceiver, DLC=8.
    MsCan125,
    /// STP 63: ISO 15765, 11-bit Tx, SW-CAN/GMLAN transceiver, DLC=8.
    SwCanGmlan33,
    /// STP 53 plus STPBR 95000 for GM medium-speed CAN variants.
    MsGmlan95,
}

impl StnProtocolPreset {
    pub fn from_route(route: &CanRouteConfig) -> Self {
        match route.bus {
            CanRouteBus::Primary => Self::PrimaryCan,
            CanRouteBus::Secondary(SecondaryBus::MsCan125) => Self::MsCan125,
            CanRouteBus::Secondary(SecondaryBus::SwCanGmlan33) => Self::SwCanGmlan33,
            CanRouteBus::Secondary(SecondaryBus::MsGmlan95) => Self::MsGmlan95,
        }
    }

    pub fn protocol_number(self) -> u8 {
        match self {
            Self::PrimaryCan => 33,
            Self::MsCan125 => 53,
            Self::SwCanGmlan33 => 63,
            Self::MsGmlan95 => 53,
        }
    }

    fn protocol_default_bitrate_bps(self) -> u32 {
        match self {
            Self::PrimaryCan => 500_000,
            Self::MsCan125 => 125_000,
            Self::SwCanGmlan33 => 33_300,
            Self::MsGmlan95 => 125_000,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum StnCommand {
    CloseProtocol,
    SelectProtocol(StnProtocolPreset),
    SetProtocolBaudRate(u32),
    SetProtocolTimeoutMs(u16),
    UseAutomaticFiltering,
    ClearPassFilters,
    AddPassFilter(CanRouteFilter),
    OpenProtocol,
}

impl StnCommand {
    pub fn encode(self) -> String {
        match self {
            Self::CloseProtocol => "STPC".to_string(),
            Self::SelectProtocol(preset) => format!("STP {:02}", preset.protocol_number()),
            Self::SetProtocolBaudRate(bitrate_bps) => format!("STPBR {bitrate_bps}"),
            Self::SetProtocolTimeoutMs(timeout_ms) => format!("STPTO {timeout_ms}"),
            Self::UseAutomaticFiltering => "STFA".to_string(),
            Self::ClearPassFilters => "STFPC".to_string(),
            Self::AddPassFilter(filter) => {
                let pattern = encode_can_identifier(filter.id);
                let mask = encode_can_identifier(filter.mask);
                format!("STFPA {pattern},{mask}")
            }
            Self::OpenProtocol => "STPO".to_string(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StnRoutePlan {
    pub route: CanRouteConfig,
    pub preset: StnProtocolPreset,
    commands: Vec<StnCommand>,
}

impl StnRoutePlan {
    pub fn commands(&self) -> &[StnCommand] {
        &self.commands
    }

    pub fn encoded_commands(&self) -> Vec<String> {
        self.commands
            .iter()
            .map(|command| command.encode())
            .collect()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StnCommandExchange {
    pub command: StnCommand,
    pub encoded: String,
    pub response: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StnApplyReport {
    pub route: CanRouteConfig,
    pub preset: StnProtocolPreset,
    pub exchanges: Vec<StnCommandExchange>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum StnRouteError {
    Capability(CapabilityMismatch),
    InvalidBitrate(u32),
    InvalidTimeoutMs(u16),
}

impl From<CapabilityMismatch> for StnRouteError {
    fn from(value: CapabilityMismatch) -> Self {
        Self::Capability(value)
    }
}

impl fmt::Display for StnRouteError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Capability(mismatch) => {
                write!(
                    f,
                    "backend {:?} cannot satisfy route: {:?}",
                    mismatch.backend, mismatch.kind
                )
            }
            Self::InvalidBitrate(bitrate) => {
                write!(f, "invalid STN CAN bitrate {bitrate} bps")
            }
            Self::InvalidTimeoutMs(timeout) => {
                write!(f, "invalid STN protocol timeout {timeout} ms")
            }
        }
    }
}

impl std::error::Error for StnRouteError {}

#[derive(Debug, Clone, Copy, Default)]
pub struct StnBackend;

impl StnBackend {
    pub fn new() -> Self {
        Self
    }

    pub fn capabilities(&self) -> BackendCaps {
        STN_BACKEND_CAPS
    }

    pub fn plan_route(
        &self,
        route: CanRouteConfig,
        timeout_ms: Option<u16>,
    ) -> Result<StnRoutePlan, StnRouteError> {
        self.capabilities().negotiate(route.isotp_requirement())?;
        if route.arbitration_bitrate_bps == 0 {
            return Err(StnRouteError::InvalidBitrate(route.arbitration_bitrate_bps));
        }
        if matches!(timeout_ms, Some(0)) {
            return Err(StnRouteError::InvalidTimeoutMs(0));
        }

        let preset = StnProtocolPreset::from_route(&route);
        let mut commands = Vec::with_capacity(5 + route.filters.filters().len());
        commands.push(StnCommand::CloseProtocol);
        commands.push(StnCommand::SelectProtocol(preset));
        if route.arbitration_bitrate_bps != preset.protocol_default_bitrate_bps() {
            commands.push(StnCommand::SetProtocolBaudRate(
                route.arbitration_bitrate_bps,
            ));
        }
        if let Some(timeout_ms) = timeout_ms {
            commands.push(StnCommand::SetProtocolTimeoutMs(timeout_ms));
        }
        if route.filters.is_empty() {
            commands.push(StnCommand::UseAutomaticFiltering);
        } else {
            commands.push(StnCommand::ClearPassFilters);
            for filter in route.filters.filters() {
                commands.push(StnCommand::AddPassFilter(*filter));
            }
        }
        commands.push(StnCommand::OpenProtocol);

        Ok(StnRoutePlan {
            route,
            preset,
            commands,
        })
    }

    pub async fn apply_route(
        &self,
        link: &mut dyn Link,
        route: CanRouteConfig,
        timeout_ms: Option<u16>,
    ) -> Result<StnApplyReport, Obd2Error> {
        let plan = self
            .plan_route(route, timeout_ms)
            .map_err(|err| Obd2Error::Adapter(err.to_string()))?;
        self.apply_route_plan(link, &plan).await
    }

    pub async fn apply_route_plan(
        &self,
        link: &mut dyn Link,
        plan: &StnRoutePlan,
    ) -> Result<StnApplyReport, Obd2Error> {
        let mut exchanges = Vec::with_capacity(plan.commands.len());
        for command in &plan.commands {
            exchanges.push(send_stn_command(link, *command).await?);
        }

        Ok(StnApplyReport {
            route: plan.route.clone(),
            preset: plan.preset,
            exchanges,
        })
    }
}

async fn send_stn_command(
    link: &mut dyn Link,
    command: StnCommand,
) -> Result<StnCommandExchange, Obd2Error> {
    let encoded = command.encode();
    link.annotate_raw_capture(&format!("stn_command={encoded}"));

    let mut frame = Vec::with_capacity(encoded.len() + 1);
    frame.extend_from_slice(encoded.as_bytes());
    frame.push(b'\r');
    link.write(&frame).await?;

    let response_bytes = link.read().await?;
    let response = String::from_utf8_lossy(&response_bytes).into_owned();
    parse_command_response(&response).map_err(|kind| {
        Obd2Error::Adapter(format!(
            "STN command `{encoded}` failed: {kind}; response={:?}",
            response.trim()
        ))
    })?;

    Ok(StnCommandExchange {
        command,
        encoded,
        response,
    })
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum StnResponseErrorKind {
    Empty,
    UnknownCommand,
    OutOfMemory,
    AdapterFault(&'static str),
    Unexpected,
}

impl fmt::Display for StnResponseErrorKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => write!(f, "empty response"),
            Self::UnknownCommand => write!(f, "unknown command"),
            Self::OutOfMemory => write!(f, "adapter filter memory exhausted"),
            Self::AdapterFault(kind) => write!(f, "{kind}"),
            Self::Unexpected => write!(f, "unexpected response"),
        }
    }
}

fn parse_command_response(response: &str) -> Result<(), StnResponseErrorKind> {
    let mut saw_nonempty = false;
    for raw_line in response.split(['\r', '\n']) {
        let line = raw_line
            .trim_matches(|ch: char| ch == '>' || ch == '\0' || ch.is_whitespace())
            .trim();
        if line.is_empty() {
            continue;
        }
        saw_nonempty = true;

        let uppercase = line.to_ascii_uppercase();
        match uppercase.as_str() {
            "OK" => return Ok(()),
            "?" => return Err(StnResponseErrorKind::UnknownCommand),
            "OUT OF MEMORY" => return Err(StnResponseErrorKind::OutOfMemory),
            _ => {}
        }

        for (needle, fault) in [
            ("UNABLE TO CONNECT", "unable to connect"),
            ("BUS INIT", "bus initialization error"),
            ("BUS ERROR", "bus error"),
            ("CAN ERROR", "CAN error"),
            ("DATA ERROR", "data error"),
            ("RX ERROR", "RX error"),
            ("STOPPED", "command stopped"),
            ("LV RESET", "low-voltage reset"),
            ("LP ALERT", "low-power alert"),
        ] {
            if uppercase.contains(needle) {
                return Err(StnResponseErrorKind::AdapterFault(fault));
            }
        }
    }

    if saw_nonempty {
        Err(StnResponseErrorKind::Unexpected)
    } else {
        Err(StnResponseErrorKind::Empty)
    }
}

fn encode_can_identifier(id: CanIdentifier) -> String {
    match id {
        CanIdentifier::Standard(raw) => format!("{raw:04X}"),
        CanIdentifier::Extended(raw) => format!("{raw:08X}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::backend::{
        CanIdentifier, CanRouteConfig, CanRouteFilter, CapabilityMismatchKind, TransportKind,
    };
    use async_trait::async_trait;
    use std::collections::VecDeque;

    #[derive(Debug)]
    struct ScriptedLink {
        responses: VecDeque<Vec<u8>>,
        writes: Vec<Vec<u8>>,
    }

    impl ScriptedLink {
        fn new(responses: impl IntoIterator<Item = &'static [u8]>) -> Self {
            Self {
                responses: responses
                    .into_iter()
                    .map(|response| response.to_vec())
                    .collect(),
                writes: Vec::new(),
            }
        }
    }

    #[async_trait]
    impl Link for ScriptedLink {
        async fn write(&mut self, data: &[u8]) -> Result<(), Obd2Error> {
            self.writes.push(data.to_vec());
            Ok(())
        }

        async fn read(&mut self) -> Result<Vec<u8>, Obd2Error> {
            self.responses.pop_front().ok_or(Obd2Error::Timeout)
        }

        async fn reset(&mut self) -> Result<(), Obd2Error> {
            self.responses.clear();
            self.writes.clear();
            Ok(())
        }

        fn name(&self) -> &str {
            "scripted-stn"
        }
    }

    #[test]
    fn stn_capabilities_include_secondary_can_not_can_fd() {
        let caps = StnBackend::new().capabilities();

        assert!(caps.supports_secondary_bus(SecondaryBus::MsCan125));
        assert!(caps.supports_secondary_bus(SecondaryBus::SwCanGmlan33));
        assert!(caps.supports_transport(TransportKind::IsoTp));
        assert!(!caps.can_fd);
    }

    #[test]
    fn stn_encodes_sw_can_route_with_timeout_and_filter() {
        let filter = CanRouteFilter::exact(CanIdentifier::extended(0x18DA_F110).unwrap());
        let route = CanRouteConfig::sw_can_gmlan_33k().with_filter(filter);
        let plan = StnBackend::new().plan_route(route, Some(25)).unwrap();

        assert_eq!(plan.preset, StnProtocolPreset::SwCanGmlan33);
        assert_eq!(
            plan.commands(),
            [
                StnCommand::CloseProtocol,
                StnCommand::SelectProtocol(StnProtocolPreset::SwCanGmlan33),
                StnCommand::SetProtocolTimeoutMs(25),
                StnCommand::ClearPassFilters,
                StnCommand::AddPassFilter(filter),
                StnCommand::OpenProtocol,
            ]
        );
        assert_eq!(
            plan.encoded_commands(),
            [
                "STPC",
                "STP 63",
                "STPTO 25",
                "STFPC",
                "STFPA 18DAF110,1FFFFFFF",
                "STPO"
            ]
        );
    }

    #[test]
    fn stn_uses_baud_override_for_95k_ms_gmlan() {
        let plan = StnBackend::new()
            .plan_route(CanRouteConfig::ms_gmlan_95k(), None)
            .unwrap();

        assert_eq!(plan.preset, StnProtocolPreset::MsGmlan95);
        assert_eq!(
            plan.encoded_commands(),
            ["STPC", "STP 53", "STPBR 95000", "STFA", "STPO"]
        );
    }

    #[test]
    fn stn_rejects_can_fd_route_plan() {
        let err = StnBackend::new()
            .plan_route(CanRouteConfig::can_fd(500_000, 2_000_000), None)
            .unwrap_err();

        assert!(matches!(
            err,
            StnRouteError::Capability(CapabilityMismatch {
                kind: CapabilityMismatchKind::CanFdUnavailable,
                ..
            })
        ));
    }

    #[test]
    fn stn_rejects_zero_timeout() {
        let err = StnBackend::new()
            .plan_route(CanRouteConfig::primary_classical(500_000), Some(0))
            .unwrap_err();

        assert_eq!(err, StnRouteError::InvalidTimeoutMs(0));
    }

    #[tokio::test]
    async fn stn_applies_route_over_link_in_order() {
        let ok: &[u8] = b"OK\r>";
        let mut link = ScriptedLink::new([ok, ok, ok, ok, ok]);
        let route = CanRouteConfig::primary_classical(500_000);
        let report = StnBackend::new()
            .apply_route(&mut link, route, Some(50))
            .await
            .unwrap();

        assert_eq!(
            link.writes,
            [
                b"STPC\r".to_vec(),
                b"STP 33\r".to_vec(),
                b"STPTO 50\r".to_vec(),
                b"STFA\r".to_vec(),
                b"STPO\r".to_vec(),
            ]
        );
        assert_eq!(report.exchanges.len(), 5);
    }

    #[tokio::test]
    async fn stn_apply_errors_on_malformed_response_without_panic() {
        let mut link = ScriptedLink::new([b"not-ok\r>".as_slice()]);
        let route = CanRouteConfig::primary_classical(500_000);
        let err = StnBackend::new()
            .apply_route(&mut link, route, None)
            .await
            .unwrap_err();

        assert!(
            matches!(err, Obd2Error::Adapter(message) if message.contains("unexpected response"))
        );
    }

    #[test]
    fn stn_response_parser_handles_error_tokens() {
        assert_eq!(
            parse_command_response("OUT OF MEMORY\r>").unwrap_err(),
            StnResponseErrorKind::OutOfMemory
        );
        assert_eq!(
            parse_command_response("?\r>").unwrap_err(),
            StnResponseErrorKind::UnknownCommand
        );
        assert_eq!(parse_command_response("\0OK\r>").unwrap(), ());
    }
}
