//! ELM327/STN adapter implementation.
//!
//! Translates OBD-II service requests into ELM327 AT commands
//! and hex string format, parses responses back to raw bytes.

use std::collections::HashSet;
use async_trait::async_trait;
use tracing::debug;
use crate::error::Obd2Error;
use crate::protocol::pid::Pid;
use crate::protocol::service::{ServiceRequest, Target};
use crate::transport::Transport;
use super::{Adapter, AdapterInfo, Chipset, Capabilities};
use crate::vehicle::Protocol;

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
}

impl Elm327Adapter {
    /// Create a new ELM327 adapter wrapping a transport.
    pub fn new(transport: Box<dyn Transport>) -> Self {
        Self {
            transport,
            info: default_adapter_info(),
            initialized: false,
            current_header: None,
        }
    }

    /// Send an AT command and read the response.
    async fn send_command(&mut self, cmd: &str) -> Result<String, Obd2Error> {
        debug!(cmd = cmd, "ELM327 send");
        self.transport.write(format!("{}\r", cmd).as_bytes()).await?;
        let response_bytes = self.transport.read().await?;
        let response = String::from_utf8_lossy(&response_bytes).to_string();
        debug!(response = response.trim(), "ELM327 recv");
        Ok(response)
    }

    /// Set the J1850/CAN header for addressing a specific module.
    #[allow(dead_code)]
    async fn set_header(&mut self, header: &str) -> Result<(), Obd2Error> {
        if self.current_header.as_deref() == Some(header) {
            return Ok(()); // Already set
        }
        let response = self.send_command(&format!("AT SH {}", header)).await?;
        if !response.contains("OK") {
            return Err(Obd2Error::Adapter(format!("AT SH failed: {}", response.trim())));
        }
        self.current_header = Some(header.to_string());
        Ok(())
    }

    /// Parse hex response string into raw bytes.
    /// Input like "41 0C 0A A0\r>" -> vec of all hex bytes, then caller
    /// specifies how many leading bytes to skip (service echo + PID echo).
    ///
    /// Handles echo lines gracefully: if the ELM327 echoes the command
    /// (e.g. "010C\r41 0C 0A A0\r>"), the echo line is skipped because
    /// tokens like "010C" exceed u8 range and fail hex-byte parsing.
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
    async fn initialize(&mut self) -> Result<AdapterInfo, Obd2Error> {
        if self.initialized {
            return Ok(self.info.clone());
        }

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

        // Step 3-6: Configure
        self.send_command("ATE0").await?;   // Echo off
        self.send_command("ATL0").await?;   // Linefeeds off
        self.send_command("ATH0").await?;   // Headers off (for standard queries)
        self.send_command("ATSP0").await?;  // Auto-detect protocol

        // Step 7: Query supported PIDs to detect protocol
        let response = self.send_command("0100").await?;
        if response.contains("41 00") || response.contains("4100") {
            // Protocol detected successfully
            // Try to detect which protocol was auto-selected
            if let Ok(protocol_response) = self.send_command("ATDPN").await {
                let proto_char = protocol_response
                    .trim()
                    .replace('>', "")
                    .trim()
                    .chars()
                    .last()
                    .unwrap_or('0');
                info.protocol = match proto_char {
                    '1' => Protocol::J1850Pwm,
                    '2' => Protocol::J1850Vpw,
                    '3' => Protocol::Iso9141(crate::vehicle::KLineInit::SlowInit),
                    '4' => Protocol::Kwp2000(crate::vehicle::KLineInit::SlowInit),
                    '5' => Protocol::Kwp2000(crate::vehicle::KLineInit::FastInit),
                    '6' => Protocol::Can11Bit500,
                    '7' => Protocol::Can29Bit500,
                    '8' => Protocol::Can11Bit250,
                    '9' => Protocol::Can29Bit250,
                    _ => Protocol::Auto,
                };
            }
        }

        self.info = info.clone();
        self.initialized = true;
        Ok(info)
    }

    async fn request(&mut self, req: &ServiceRequest) -> Result<Vec<u8>, Obd2Error> {
        // Handle targeting
        match &req.target {
            Target::Module(module_id) => {
                debug!(module = %module_id, "targeting specific module");
            }
            Target::Broadcast => {
                // Use default functional addressing
            }
        }

        // Build the hex command string
        let cmd = if req.data.is_empty() {
            format!("{:02X}", req.service_id)
        } else {
            let data_hex: Vec<String> = req.data.iter().map(|b| format!("{:02X}", b)).collect();
            format!("{:02X}{}", req.service_id, data_hex.join(""))
        };

        let response = self.send_command(&cmd).await?;

        // Check for errors
        Self::check_response_error(&response)?;

        // Determine how many echo bytes to skip
        let skip = match req.service_id {
            0x01 | 0x02 => 2,  // service echo + PID echo
            0x03 | 0x04 | 0x07 | 0x0A => 1,  // service echo only
            0x09 => 2,  // service echo + infotype
            0x22 | 0x21 => 3,  // service echo + 2-byte DID echo
            _ => 1,
        };

        Self::parse_hex_response(&response, skip)
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
                    if let Ok(data) = Self::parse_hex_response(&response, 2) {
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
        transport.expect("ATSP0", "OK\r>");
        transport.expect("0100", "41 00 BE 3E B8 11\r>");
        transport.expect("ATDPN", "A6\r>"); // CAN 11-bit 500kbps
    }

    #[tokio::test]
    async fn test_elm327_initialize() {
        let mut transport = MockTransport::new();
        setup_init(&mut transport);

        let mut adapter = Elm327Adapter::new(Box::new(transport));
        let info = adapter.initialize().await.unwrap();
        assert_eq!(info.chipset, Chipset::Elm327Genuine);
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
}
