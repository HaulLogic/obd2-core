//! ELM327/STN adapter implementation.
//!
//! Translates OBD-II service requests into ELM327 AT commands
//! and hex string format, parses responses back to raw bytes.

use std::collections::HashSet;
use std::fmt::Write as _;
use async_trait::async_trait;
use tracing::debug;
use crate::error::Obd2Error;
use crate::protocol::codec::{self, BusFamily};
use crate::protocol::pid::Pid;
use crate::protocol::service::{ServiceRequest, Target};
use crate::transport::Transport;
use super::{
    Adapter, AdapterEvent, AdapterEventKind, AdapterInfo, Capabilities, Chipset,
    InitializationReport, PhysicalTarget, ProbeAttempt, ProbeResult,
    ProtocolSelectionSource, RoutedRequest,
};
use crate::vehicle::{KLineInit, PhysicalAddress, Protocol};

/// Default adapter info returned before initialization.
fn default_adapter_info() -> AdapterInfo {
    AdapterInfo {
        chipset: Chipset::Unknown,
        firmware: String::new(),
        protocol: Protocol::Auto,
        capabilities: Capabilities::default(),
    }
}

/// ELM327/STN adapter over any Transport.
pub struct Elm327Adapter {
    transport: Box<dyn Transport>,
    info: AdapterInfo,
    initialized: bool,
    current_header: Option<String>,
    events: Vec<AdapterEvent>,
}

impl Elm327Adapter {
    /// Create a new ELM327 adapter wrapping a transport.
    pub fn new(transport: Box<dyn Transport>) -> Self {
        Self {
            transport,
            info: default_adapter_info(),
            initialized: false,
            current_header: None,
            events: Vec::new(),
        }
    }

    /// Mutable access to the underlying transport.
    pub fn transport_mut(&mut self) -> &mut dyn Transport {
        &mut *self.transport
    }

    /// Send an AT command and read the response.
    async fn send_command(&mut self, cmd: &str) -> Result<String, Obd2Error> {
        debug!(cmd = cmd, "ELM327 send");
        let response = self.send_command_raw(cmd).await?;
        self.record_response_events(&response);
        self.handle_fault_recovery(&response).await?;
        debug!(response = response.trim(), "ELM327 recv");
        Ok(response)
    }

    async fn send_command_raw(&mut self, cmd: &str) -> Result<String, Obd2Error> {
        self.transport.annotate_raw_capture(&format!("command={cmd}"));
        let mut framed = String::with_capacity(cmd.len() + 1);
        framed.push_str(cmd);
        framed.push('\r');
        self.transport.write(framed.as_bytes()).await?;
        let response_bytes = self.transport.read().await?;
        Ok(self.sanitize_response_bytes(response_bytes))
    }

    fn push_event(&mut self, kind: AdapterEventKind, detail: impl Into<Option<String>>) {
        let detail = detail.into();
        self.transport.annotate_raw_capture(&format!(
            "adapter_event={kind:?} detail={}",
            detail.clone().unwrap_or_default()
        ));
        self.events.push(AdapterEvent { kind, detail });
    }

    fn sanitize_response_bytes(&mut self, mut response_bytes: Vec<u8>) -> String {
        let original_len = response_bytes.len();
        response_bytes.retain(|&b| b != 0);
        let null_count = original_len - response_bytes.len();
        if null_count > 0 {
            self.push_event(
                AdapterEventKind::NullBytesFiltered { count: null_count },
                Some(format!("filtered {null_count} null bytes from adapter response")),
            );
        }
        String::from_utf8_lossy(&response_bytes).into_owned()
    }

    fn record_response_events(&mut self, response: &str) {
        let trimmed = response.trim();
        if trimmed.contains("SEARCHING...") {
            self.push_event(AdapterEventKind::SearchingDisplayed, None);
        }
        if trimmed.contains("BUS BUSY") {
            self.push_event(AdapterEventKind::BusBusy, Some(trimmed.to_string()));
        }
        if trimmed.contains("LP ALERT") {
            self.push_event(
                AdapterEventKind::RecoveryAction {
                    action: "LP ALERT".into(),
                },
                Some(trimmed.to_string()),
            );
        }
        if trimmed.contains("BUS ERROR") {
            self.push_event(AdapterEventKind::BusError, Some(trimmed.to_string()));
        }
        if trimmed.contains("CAN ERROR") {
            self.push_event(AdapterEventKind::CanError, Some(trimmed.to_string()));
        }
        if trimmed.contains("DATA ERROR") {
            self.push_event(AdapterEventKind::DataError, Some(trimmed.to_string()));
        }
        if trimmed.contains("<RX ERROR") {
            self.push_event(AdapterEventKind::RxError, Some(trimmed.to_string()));
        }
        if trimmed.contains("STOPPED") {
            self.push_event(AdapterEventKind::Stopped, Some(trimmed.to_string()));
        }
        if trimmed.contains("ERR94") {
            self.push_event(AdapterEventKind::Err94, Some(trimmed.to_string()));
        }
        if trimmed.contains("LV RESET") {
            self.push_event(AdapterEventKind::LowVoltageReset, Some(trimmed.to_string()));
        }
        if trimmed == "?" {
            self.push_event(AdapterEventKind::UnknownCommand, None);
        }
    }

    async fn handle_fault_recovery(&mut self, response: &str) -> Result<(), Obd2Error> {
        let trimmed = response.trim();
        if trimmed.contains("ERR94") || trimmed.contains("LV RESET") {
            self.recover_from_adapter_fault().await?;
        }
        Ok(())
    }

    async fn recover_from_adapter_fault(&mut self) -> Result<(), Obd2Error> {
        self.push_event(
            AdapterEventKind::RecoveryAction {
                action: "ATFE".into(),
            },
            Some("recovering from fatal adapter fault".into()),
        );
        let _ = self.send_command_raw("ATFE").await?;
        self.current_header = None;
        self.apply_runtime_defaults().await?;
        Ok(())
    }

    async fn apply_runtime_defaults(&mut self) -> Result<(), Obd2Error> {
        self.send_command_raw("ATE0").await?;
        self.send_command_raw("ATL0").await?;
        self.send_command_raw("ATH0").await?;
        self.send_command_raw("ATS0").await?;
        self.send_command_raw("ATAT1").await?;
        self.send_command_raw("ATCAF1").await?;
        self.send_command_raw("ATCFC1").await?;
        Ok(())
    }

    async fn apply_protocol_runtime_policy(&mut self, protocol: Protocol) -> Result<(), Obd2Error> {
        match protocol {
            Protocol::Iso9141(_) => {
                self.send_command_raw("ATSI").await?;
                self.send_command_raw("ATSW96").await?;
                self.send_command_raw("ATWM686AF10100").await?;
            }
            Protocol::Kwp2000(KLineInit::SlowInit) => {
                self.send_command_raw("ATSI").await?;
                self.send_command_raw("ATSW96").await?;
                self.send_command_raw("ATWMC133F13E").await?;
            }
            Protocol::Kwp2000(KLineInit::FastInit) => {
                self.send_command_raw("ATFI").await?;
                self.send_command_raw("ATSW96").await?;
                self.send_command_raw("ATWMC133F13E").await?;
            }
            Protocol::Can11Bit500 | Protocol::Can11Bit250 | Protocol::Can29Bit500 | Protocol::Can29Bit250 => {
                self.send_command_raw("ATCAF1").await?;
                self.send_command_raw("ATCFC1").await?;
            }
            Protocol::J1850Pwm | Protocol::J1850Vpw | Protocol::Auto => {}
        }
        Ok(())
    }

    fn protocol_family(&self) -> BusFamily {
        match self.info.protocol {
            Protocol::Can11Bit500 | Protocol::Can11Bit250 | Protocol::Can29Bit500 | Protocol::Can29Bit250 => BusFamily::Can,
            Protocol::J1850Pwm | Protocol::J1850Vpw => BusFamily::J1850,
            Protocol::Iso9141(_) => BusFamily::Iso9141,
            Protocol::Kwp2000(_) => BusFamily::Kwp2000,
            Protocol::Auto => BusFamily::Can,
        }
    }

    /// Set the J1850/CAN header for addressing a specific module.
    async fn set_header(&mut self, header: &str) -> Result<(), Obd2Error> {
        if self.current_header.as_deref() == Some(header) {
            return Ok(()); // Already set
        }
        let response = self.send_command(&format!("AT SH {}", header)).await?;
        if !response.contains("OK") {
            return Err(Obd2Error::Adapter(format!("AT SH failed: {}", response.trim())));
        }
        self.current_header = Some(header.to_string());
        self.push_event(
            AdapterEventKind::HeaderChanged {
                header: header.to_string(),
            },
            None,
        );
        Ok(())
    }

    async fn apply_target(&mut self, target: &PhysicalTarget) -> Result<(), Obd2Error> {
        match target {
            PhysicalTarget::Broadcast => self.clear_targeting().await,
            PhysicalTarget::Addressed(address) => match address {
                PhysicalAddress::J1850 { header, .. } => {
                    self.set_header(&format!("{:02X}{:02X}{:02X}", header[0], header[1], header[2])).await
                }
                PhysicalAddress::Can11Bit { request_id, .. } => {
                    self.set_header(&format!("{:03X}", request_id)).await
                }
                PhysicalAddress::Can29Bit { request_id, .. } => {
                    self.set_header(&format!("{:08X}", request_id)).await
                }
                PhysicalAddress::J1939 { .. } => Err(Obd2Error::Adapter(
                    "J1939 addressed routing is not implemented yet".into(),
                )),
            },
        }
    }

    async fn clear_targeting(&mut self) -> Result<(), Obd2Error> {
        if self.current_header.is_none() {
            return Ok(());
        }
        let Some(header) = self.default_broadcast_header() else {
            self.current_header = None;
            return Ok(());
        };
        self.set_header(&header).await?;
        self.push_event(
            AdapterEventKind::HeaderReset {
                header: header.clone(),
            },
            Some("restored broadcast header".into()),
        );
        Ok(())
    }

    fn default_broadcast_header(&self) -> Option<String> {
        match self.info.protocol {
            Protocol::J1850Pwm
            | Protocol::J1850Vpw
            | Protocol::Iso9141(_)
            | Protocol::Kwp2000(_) => Some("686AF1".to_string()),
            Protocol::Can11Bit500 | Protocol::Can11Bit250 => Some("7DF".to_string()),
            Protocol::Can29Bit500 | Protocol::Can29Bit250 => Some("18DB33F1".to_string()),
            Protocol::Auto => None,
        }
    }

    async fn send_routed_command(
        &mut self,
        service_id: u8,
        data: &[u8],
        target: &PhysicalTarget,
    ) -> Result<Vec<u8>, Obd2Error> {
        self.apply_target(target).await?;

        let mut cmd = String::with_capacity(2 + (data.len() * 2));
        write!(&mut cmd, "{:02X}", service_id).expect("write to string");
        for byte in data {
            write!(&mut cmd, "{:02X}", byte).expect("write to string");
        }

        let response = self.send_command(&cmd).await?;
        Self::check_response_error(&response)?;

        let skip = match service_id {
            0x01 | 0x02 => 2,
            0x03 | 0x04 | 0x07 | 0x0A => 1,
            0x09 => 2,
            0x22 | 0x21 => 3,
            _ => 1,
        };

        codec::decode_elm_response_payload(&response, self.protocol_family(), skip)
    }

    /// Parse hex response string into raw bytes.
    /// Input like "41 0C 0A A0\r>" -> vec of all hex bytes, then caller
    /// specifies how many leading bytes to skip (service echo + PID echo).
    ///
    /// Handles echo lines gracefully: if the ELM327 echoes the command
    /// (e.g. "010C\r41 0C 0A A0\r>"), the echo line is skipped because
    /// tokens like "010C" exceed u8 range and fail hex-byte parsing.
    #[cfg(test)]
    fn parse_hex_response(response: &str, skip_bytes: usize) -> Result<Vec<u8>, Obd2Error> {
        let cleaned = response.replace('>', "");
        let mut all_bytes = Vec::new();

        for line in cleaned.split(['\r', '\n']) {
            let line = line.trim();
            if line.is_empty() {
                continue;
            }

            // Try to parse this line as space-separated hex bytes.
            // Echo lines (e.g. "010C") contain tokens that exceed u8
            // range or aren't valid hex, so they naturally fail and
            // are skipped.
            let line_bytes: Result<Vec<u8>, _> = line
                .split_whitespace()
                .map(|s| u8::from_str_radix(s, 16))
                .collect();

            if let Ok(bytes) = line_bytes {
                all_bytes.extend(bytes);
            }
        }

        if all_bytes.is_empty() {
            return Err(Obd2Error::ParseError(format!(
                "no valid hex data in response: {}",
                response.trim()
            )));
        }

        if all_bytes.len() > skip_bytes {
            Ok(all_bytes[skip_bytes..].to_vec())
        } else {
            Ok(vec![])
        }
    }

    /// Parse supported PIDs from a bitmap response.
    /// `data` is the 4 bitmap bytes, `base_pid` is the PID that was queried
    /// (0x00, 0x20, 0x40, 0x60). Bit 31 (MSB of first byte) = base_pid + 1.
    pub fn parse_supported_pids(data: &[u8], base_pid: u8) -> Vec<u8> {
        let mut pids = Vec::new();
        for (byte_idx, &byte) in data.iter().enumerate() {
            for bit in 0..8 {
                if byte & (0x80 >> bit) != 0 {
                    let pid = base_pid + (byte_idx as u8 * 8) + bit + 1;
                    pids.push(pid);
                }
            }
        }
        pids
    }

    /// Check if response indicates an error.
    fn check_response_error(response: &str) -> Result<(), Obd2Error> {
        let trimmed = response.trim();
        if trimmed.contains("NO DATA") {
            return Err(Obd2Error::NoData);
        }
        if trimmed.contains("UNABLE TO CONNECT") {
            return Err(Obd2Error::Adapter("unable to connect to vehicle".into()));
        }
        if trimmed.contains("BUS INIT") && trimmed.contains("ERROR") {
            return Err(Obd2Error::Adapter("bus initialization error".into()));
        }
        if trimmed.contains("CAN ERROR") {
            return Err(Obd2Error::Adapter("CAN bus error".into()));
        }
        if trimmed == "?" {
            return Err(Obd2Error::Adapter("unknown command".into()));
        }
        // Check for negative response (7F xx xx)
        if trimmed.starts_with("7F") {
            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            if parts.len() >= 3 {
                if let (Ok(service), Ok(nrc_byte)) = (
                    u8::from_str_radix(parts[1], 16),
                    u8::from_str_radix(parts[2], 16),
                ) {
                    if let Some(nrc) = crate::error::NegativeResponse::from_byte(nrc_byte) {
                        return Err(Obd2Error::NegativeResponse { service, nrc });
                    }
                }
            }
        }
        Ok(())
    }

    fn classify_probe_result(response: &str) -> ProbeResult {
        let trimmed = response.trim();
        if trimmed.contains("UNABLE TO CONNECT") {
            ProbeResult::UnableToConnect
        } else if trimmed.contains("BUS INIT") && trimmed.contains("ERROR") {
            ProbeResult::BusInitFailure
        } else if trimmed.contains("BUS ERROR") || trimmed.contains("BUS BUSY") {
            ProbeResult::BusError
        } else if trimmed.contains("CAN ERROR") {
            ProbeResult::CanError
        } else if trimmed.contains("NO DATA") {
            ProbeResult::NoResponse
        } else if trimmed == "?" {
            ProbeResult::UnsupportedProtocol
        } else {
            ProbeResult::AdapterFault
        }
    }

    fn parse_protocol_response(&self, protocol_response: &str) -> Protocol {
        let proto_char = protocol_response
            .trim()
            .replace('>', "")
            .trim()
            .chars()
            .last()
            .unwrap_or('0');
        Protocol::from_elm_code(proto_char).unwrap_or(Protocol::Auto)
    }

    async fn probe_protocol(
        &mut self,
        protocol: Protocol,
        source: ProtocolSelectionSource,
        command: &str,
    ) -> Result<ProbeAttempt, Obd2Error> {
        self.send_command(command).await?;
        let response = self.send_command("0100").await?;
        let result = if response.contains("41 00") || response.contains("4100") {
            ProbeResult::Success
        } else {
            Self::classify_probe_result(&response)
        };
        Ok(ProbeAttempt {
            protocol,
            source,
            result,
            detail: Some(response.trim().to_string()),
        })
    }

    fn fallback_probe_order() -> &'static [(Protocol, &'static str)] {
        &[
            (Protocol::Can11Bit500, "ATTP6"),
            (Protocol::Can29Bit500, "ATTP7"),
            (Protocol::Can11Bit250, "ATTP8"),
            (Protocol::Can29Bit250, "ATTP9"),
            (Protocol::J1850Vpw, "ATTP2"),
            (Protocol::J1850Pwm, "ATTP1"),
            (Protocol::Iso9141(KLineInit::SlowInit), "ATTP3"),
            (Protocol::Kwp2000(KLineInit::FastInit), "ATTP5"),
            (Protocol::Kwp2000(KLineInit::SlowInit), "ATTP4"),
        ]
    }
}

impl std::fmt::Debug for Elm327Adapter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Elm327Adapter")
            .field("info", &self.info)
            .field("initialized", &self.initialized)
            .field("current_header", &self.current_header)
            .finish()
    }
}

#[async_trait]
impl Adapter for Elm327Adapter {
    async fn initialize(&mut self) -> Result<InitializationReport, Obd2Error> {
        if self.initialized {
            return Ok(InitializationReport {
                info: self.info.clone(),
                probe_attempts: Vec::new(),
                events: self.drain_events(),
            });
        }

        self.push_event(AdapterEventKind::Reset, Some("initializing adapter".into()));
        // Step 1: Reset
        let atz_response = self.send_command("ATZ").await?;

        // Step 2: Probe for STN
        let sti_response = match self.send_command("STI").await {
            Ok(r) if !r.contains("?") => Some(r),
            _ => None,
        };

        // Detect chipset and capabilities
        let mut info = AdapterInfo::detect(
            &atz_response,
            sti_response.as_deref(),
        );
        let mut probe_attempts = Vec::new();

        // Step 3-6: Configure
        self.send_command("ATE0").await?;   // Echo off
        self.send_command("ATL0").await?;   // Linefeeds off
        self.send_command("ATH0").await?;   // Headers off (for standard queries)
        self.send_command("ATS0").await?;   // Spaces off
        self.send_command("ATAT1").await?;  // Adaptive timing on
        self.send_command("ATSP0").await?;  // Auto-detect protocol
        self.push_event(AdapterEventKind::ProtocolSearching, Some("ATSP0".into()));

        // Step 7: Query supported PIDs to detect protocol
        let response = self.send_command("0100").await?;
        if response.contains("41 00") || response.contains("4100") {
            probe_attempts.push(ProbeAttempt {
                protocol: Protocol::Auto,
                source: ProtocolSelectionSource::AutoDetect,
                result: ProbeResult::Success,
                detail: Some(response.trim().to_string()),
            });
            // Protocol detected successfully
            // Try to detect which protocol was auto-selected
            if let Ok(protocol_response) = self.send_command("ATDPN").await {
                info.protocol = self.parse_protocol_response(&protocol_response);
            }
        } else {
            probe_attempts.push(ProbeAttempt {
                protocol: Protocol::Auto,
                source: ProtocolSelectionSource::AutoDetect,
                result: Self::classify_probe_result(&response),
                detail: Some(response.trim().to_string()),
            });
            for (protocol, command) in Self::fallback_probe_order() {
                let attempt = self
                    .probe_protocol(*protocol, ProtocolSelectionSource::ExplicitProbe, command)
                    .await?;
                let success = attempt.result == ProbeResult::Success;
                probe_attempts.push(attempt);
                if success {
                    info.protocol = *protocol;
                    break;
                }
            }
        }

        if info.protocol == Protocol::Auto {
            self.push_event(
                AdapterEventKind::UnsupportedProtocol,
                Some("no supported protocol could be selected".into()),
            );
            return Err(Obd2Error::Adapter("unable to determine active protocol".into()));
        }
        self.push_event(AdapterEventKind::ProtocolSelected(info.protocol), None);
        self.info = info.clone();
        self.apply_protocol_runtime_policy(info.protocol).await?;

        self.initialized = true;
        Ok(InitializationReport {
            info,
            probe_attempts,
            events: self.drain_events(),
        })
    }

    async fn request(&mut self, req: &ServiceRequest) -> Result<Vec<u8>, Obd2Error> {
        match &req.target {
            Target::Module(module_id) => {
                debug!(module = %module_id, "targeting specific module");
                Err(Obd2Error::Adapter(format!(
                    "logical module target '{module_id}' requires session-side routing resolution"
                )))
            }
            Target::Broadcast => {
                self.send_routed_command(req.service_id, &req.data, &PhysicalTarget::Broadcast).await
            }
        }
    }

    async fn routed_request(&mut self, req: &RoutedRequest) -> Result<Vec<u8>, Obd2Error> {
        self.send_routed_command(req.service_id, &req.data, &req.target).await
    }

    async fn supported_pids(&mut self) -> Result<HashSet<Pid>, Obd2Error> {
        let mut all_supported = HashSet::new();

        // Query PID 0x00, 0x20, 0x40, 0x60
        for base in [0x00u8, 0x20, 0x40, 0x60] {
            let cmd = format!("01{:02X}", base);
            match self.send_command(&cmd).await {
                Ok(response) => {
                    if Self::check_response_error(&response).is_err() {
                        break; // No more supported PIDs
                    }
                    if let Ok(data) = codec::decode_elm_response_payload(&response, self.protocol_family(), 2) {
                        if data.len() >= 4 {
                            for pid_code in Self::parse_supported_pids(&data, base) {
                                all_supported.insert(Pid(pid_code));
                            }
                        }
                    }
                }
                Err(_) => break,
            }
        }

        Ok(all_supported)
    }

    async fn battery_voltage(&mut self) -> Result<Option<f64>, Obd2Error> {
        let response = self.send_command("ATRV").await?;
        let cleaned = response
            .replace(['V', 'v', '>', '\r', '\n'], "");
        let cleaned = cleaned.trim().to_string();
        match cleaned.parse::<f64>() {
            Ok(v) => Ok(Some(v)),
            Err(_) => Ok(None),
        }
    }

    fn info(&self) -> &AdapterInfo {
        &self.info
    }

    fn drain_events(&mut self) -> Vec<AdapterEvent> {
        std::mem::take(&mut self.events)
    }

    fn transport_mut(&mut self) -> Option<&mut dyn Transport> {
        Some(&mut *self.transport)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::mock::MockTransport;

    fn setup_init(transport: &mut MockTransport) {
        transport.expect("ATZ", "ELM327 v2.1\r\r>");
        transport.expect("STI", "?\r>");
        transport.expect("ATE0", "OK\r>");
        transport.expect("ATL0", "OK\r>");
        transport.expect("ATH0", "OK\r>");
        transport.expect("ATS0", "OK\r>");
        transport.expect("ATAT1", "OK\r>");
        transport.expect("ATSP0", "OK\r>");
        transport.expect("0100", "41 00 BE 3E B8 11\r>");
        transport.expect("ATDPN", "A6\r>"); // CAN 11-bit 500kbps
        transport.expect("ATCAF1", "OK\r>");
        transport.expect("ATCFC1", "OK\r>");
    }

    #[tokio::test]
    async fn test_elm327_initialize() {
        let mut transport = MockTransport::new();
        setup_init(&mut transport);

        let mut adapter = Elm327Adapter::new(Box::new(transport));
        let info = adapter.initialize().await.unwrap();
        assert_eq!(info.info.chipset, Chipset::Elm327Genuine);
    }

    #[tokio::test]
    async fn test_elm327_read_pid() {
        let mut transport = MockTransport::new();
        setup_init(&mut transport);
        transport.expect("010C", "41 0C 0A A0\r>");

        let mut adapter = Elm327Adapter::new(Box::new(transport));
        adapter.initialize().await.unwrap();

        let req = ServiceRequest::read_pid(Pid::ENGINE_RPM);
        let response = adapter.request(&req).await.unwrap();
        assert_eq!(response, vec![0x0A, 0xA0]);
    }

    #[tokio::test]
    async fn test_elm327_no_data() {
        let mut transport = MockTransport::new();
        setup_init(&mut transport);
        transport.expect("015C", "NO DATA\r>");

        let mut adapter = Elm327Adapter::new(Box::new(transport));
        adapter.initialize().await.unwrap();

        let req = ServiceRequest::read_pid(Pid::ENGINE_OIL_TEMP);
        let result = adapter.request(&req).await;
        assert!(matches!(result, Err(Obd2Error::NoData)));
    }

    #[tokio::test]
    async fn test_elm327_parse_hex_response() {
        let data = Elm327Adapter::parse_hex_response("41 0C 0A A0\r>", 2).unwrap();
        assert_eq!(data, vec![0x0A, 0xA0]);
    }

    #[tokio::test]
    async fn test_elm327_parse_hex_response_with_echo() {
        // ELM327 echoes "010C" before the actual response when ATE0 hasn't taken effect
        let data = Elm327Adapter::parse_hex_response("010C\r41 0C 0A A0\r>", 2).unwrap();
        assert_eq!(data, vec![0x0A, 0xA0]);
    }

    #[tokio::test]
    async fn test_elm327_parse_hex_response_echo_only() {
        // Only echo, no ECU response — should return a parse error
        let result = Elm327Adapter::parse_hex_response("010C\r>", 2);
        assert!(matches!(result, Err(Obd2Error::ParseError(_))));
    }

    #[tokio::test]
    async fn test_elm327_parse_supported_pids() {
        // BE 3E B8 11 = supported PIDs bitmap
        let pids = Elm327Adapter::parse_supported_pids(&[0xBE, 0x3E, 0xB8, 0x11], 0x00);
        assert!(pids.contains(&0x01)); // Monitor status
        assert!(pids.contains(&0x04)); // Engine load
        assert!(pids.contains(&0x05)); // Coolant temp
        assert!(pids.contains(&0x0C)); // RPM
        assert!(pids.contains(&0x0D)); // Speed
    }

    #[tokio::test]
    async fn test_elm327_read_dtcs() {
        let mut transport = MockTransport::new();
        setup_init(&mut transport);
        transport.expect("03", "43 01 04 20 01 71\r>");

        let mut adapter = Elm327Adapter::new(Box::new(transport));
        adapter.initialize().await.unwrap();

        let req = ServiceRequest::read_dtcs();
        let response = adapter.request(&req).await.unwrap();
        // After skipping the service echo byte (43), we should have the DTC data
        assert!(!response.is_empty());
    }

    #[tokio::test]
    async fn test_elm327_battery_voltage() {
        let mut transport = MockTransport::new();
        setup_init(&mut transport);
        transport.expect("ATRV", "14.4V\r>");

        let mut adapter = Elm327Adapter::new(Box::new(transport));
        adapter.initialize().await.unwrap();

        let voltage = adapter.battery_voltage().await.unwrap();
        assert_eq!(voltage, Some(14.4));
    }

    #[tokio::test]
    async fn test_elm327_negative_response() {
        let result = Elm327Adapter::check_response_error("7F 22 31\r>");
        assert!(matches!(result, Err(Obd2Error::NegativeResponse { service: 0x22, .. })));
    }

    #[tokio::test]
    async fn test_elm327_routed_request_applies_j1850_header() {
        let mut transport = MockTransport::new();
        setup_init(&mut transport);
        transport.expect("AT SH 6C10F1", "OK\r>");
        transport.expect("22162F", "62 16 2F 80 00\r>");

        let mut adapter = Elm327Adapter::new(Box::new(transport));
        adapter.initialize().await.unwrap();

        let response = adapter.routed_request(&RoutedRequest {
            service_id: 0x22,
            data: vec![0x16, 0x2F],
            target: PhysicalTarget::Addressed(PhysicalAddress::J1850 {
                node: 0x10,
                header: [0x6C, 0x10, 0xF1],
            }),
        }).await.unwrap();

        assert_eq!(response, vec![0x80, 0x00]);
    }

    #[tokio::test]
    async fn test_elm327_routed_request_applies_can11_header() {
        let mut transport = MockTransport::new();
        setup_init(&mut transport);
        transport.expect("AT SH 7E0", "OK\r>");
        transport.expect("221234", "62 12 34 12 34\r>");

        let mut adapter = Elm327Adapter::new(Box::new(transport));
        adapter.info.protocol = Protocol::Can11Bit500;
        adapter.initialized = true;

        let response = adapter.routed_request(&RoutedRequest {
            service_id: 0x22,
            data: vec![0x12, 0x34],
            target: PhysicalTarget::Addressed(PhysicalAddress::Can11Bit {
                request_id: 0x7E0,
                response_id: 0x7E8,
            }),
        }).await.unwrap();

        assert_eq!(response, vec![0x12, 0x34]);
    }

    #[tokio::test]
    async fn test_elm327_broadcast_after_targeted_request_resets_header() {
        let mut transport = MockTransport::new();
        setup_init(&mut transport);
        transport.expect("AT SH 7E0", "OK\r>");
        transport.expect("221234", "62 12 34 12 34\r>");
        transport.expect("AT SH 7DF", "OK\r>");
        transport.expect("010C", "41 0C 0A A0\r>");

        let mut adapter = Elm327Adapter::new(Box::new(transport));
        adapter.initialize().await.unwrap();

        let _ = adapter.routed_request(&RoutedRequest {
            service_id: 0x22,
            data: vec![0x12, 0x34],
            target: PhysicalTarget::Addressed(PhysicalAddress::Can11Bit {
                request_id: 0x7E0,
                response_id: 0x7E8,
            }),
        }).await.unwrap();

        let response = adapter.request(&ServiceRequest::read_pid(Pid::ENGINE_RPM)).await.unwrap();
        assert_eq!(response, vec![0x0A, 0xA0]);
    }

    #[tokio::test]
    async fn test_elm327_routed_request_reuses_cached_header() {
        let mut transport = MockTransport::new();
        setup_init(&mut transport);
        transport.expect("AT SH 7E0", "OK\r>");
        transport.expect("221234", "62 12 34 12 34\r>");
        transport.expect("221235", "62 12 35 56 78\r>");

        let mut adapter = Elm327Adapter::new(Box::new(transport));
        adapter.initialize().await.unwrap();

        let _ = adapter.routed_request(&RoutedRequest {
            service_id: 0x22,
            data: vec![0x12, 0x34],
            target: PhysicalTarget::Addressed(PhysicalAddress::Can11Bit {
                request_id: 0x7E0,
                response_id: 0x7E8,
            }),
        }).await.unwrap();

        let response = adapter.routed_request(&RoutedRequest {
            service_id: 0x22,
            data: vec![0x12, 0x35],
            target: PhysicalTarget::Addressed(PhysicalAddress::Can11Bit {
                request_id: 0x7E0,
                response_id: 0x7E8,
            }),
        }).await.unwrap();

        assert_eq!(response, vec![0x56, 0x78]);
    }

    #[tokio::test]
    async fn test_elm327_filters_null_bytes_and_records_event() {
        let mut transport = MockTransport::new();
        setup_init(&mut transport);
        transport.expect("010C", "41 0C \0 0A A0\r>");

        let mut adapter = Elm327Adapter::new(Box::new(transport));
        adapter.initialize().await.unwrap();
        let response = adapter.request(&ServiceRequest::read_pid(Pid::ENGINE_RPM)).await.unwrap();
        let events = adapter.drain_events();

        assert_eq!(response, vec![0x0A, 0xA0]);
        assert!(events.iter().any(|event| matches!(
            event.kind,
            AdapterEventKind::NullBytesFiltered { count: 1 }
        )));
    }

    #[tokio::test]
    async fn test_elm327_records_bus_busy_event() {
        let mut transport = MockTransport::new();
        setup_init(&mut transport);
        transport.expect("010C", "BUS BUSY\r>");

        let mut adapter = Elm327Adapter::new(Box::new(transport));
        adapter.initialize().await.unwrap();
        let result = adapter.request(&ServiceRequest::read_pid(Pid::ENGINE_RPM)).await;
        let events = adapter.drain_events();

        assert!(result.is_err());
        assert!(events.iter().any(|event| matches!(event.kind, AdapterEventKind::BusBusy)));
    }

    #[tokio::test]
    async fn test_elm327_records_stopped_event() {
        let mut transport = MockTransport::new();
        setup_init(&mut transport);
        transport.expect("010C", "STOPPED\r>");

        let mut adapter = Elm327Adapter::new(Box::new(transport));
        adapter.initialize().await.unwrap();
        let result = adapter.request(&ServiceRequest::read_pid(Pid::ENGINE_RPM)).await;
        let events = adapter.drain_events();

        assert!(result.is_err());
        assert!(events.iter().any(|event| matches!(event.kind, AdapterEventKind::Stopped)));
    }

    #[tokio::test]
    async fn test_elm327_records_low_power_alert_event() {
        let mut transport = MockTransport::new();
        setup_init(&mut transport);
        transport.expect("010C", "LP ALERT\r>");

        let mut adapter = Elm327Adapter::new(Box::new(transport));
        adapter.initialize().await.unwrap();
        let result = adapter.request(&ServiceRequest::read_pid(Pid::ENGINE_RPM)).await;
        let events = adapter.drain_events();

        assert!(result.is_err());
        assert!(events.iter().any(|event| matches!(
            &event.kind,
            AdapterEventKind::RecoveryAction { action } if action == "LP ALERT"
        )));
    }

    #[tokio::test]
    async fn test_elm327_recovers_from_lv_reset() {
        let mut transport = MockTransport::new();
        setup_init(&mut transport);
        transport.expect("010C", "LV RESET\r>");
        transport.expect("ATFE", "OK\r>");
        transport.expect("ATE0", "OK\r>");
        transport.expect("ATL0", "OK\r>");
        transport.expect("ATH0", "OK\r>");
        transport.expect("ATS0", "OK\r>");
        transport.expect("ATAT1", "OK\r>");
        transport.expect("ATCAF1", "OK\r>");
        transport.expect("ATCFC1", "OK\r>");

        let mut adapter = Elm327Adapter::new(Box::new(transport));
        adapter.initialize().await.unwrap();
        let result = adapter.request(&ServiceRequest::read_pid(Pid::ENGINE_RPM)).await;
        let events = adapter.drain_events();

        assert!(result.is_err());
        assert!(events.iter().any(|event| matches!(event.kind, AdapterEventKind::LowVoltageReset)));
        assert!(events.iter().any(|event| matches!(event.kind, AdapterEventKind::RecoveryAction { .. })));
    }
}
