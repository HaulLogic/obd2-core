//! Diagnostics over IP (ISO 13400) framing and diagnostic transport support.
//!
//! This module owns deterministic packet encode/decode plus the diagnostic
//! exchange boundary used after a backend has opened a byte stream to a DoIP
//! entity. Socket discovery, TLS policy, and OEM gateway authentication stay in
//! transport/backends.

use crate::error::Obd2Error;
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

pub const DOIP_TCP_PORT: u16 = 13400;
pub const DOIP_UDP_PORT: u16 = 13400;
pub const DOIP_HEADER_LEN: usize = 8;
pub const DOIP_PROTOCOL_VERSION_2012: u8 = 0x02;
pub const DOIP_PROTOCOL_VERSION_2019: u8 = 0x03;
pub const DOIP_VERSION_DEFAULT: u8 = DOIP_PROTOCOL_VERSION_2012;
pub const DOIP_VERSION_ANY: u8 = 0xFF;
pub const DOIP_MAX_PAYLOAD_LEN: u32 = 16 * 1024 * 1024;
pub const DOIP_ROUTING_ACTIVATION_SUCCESS: u8 = 0x10;
const DOIP_MAX_DIAGNOSTIC_EVENTS: usize = 16;

/// DoIP payload type from the generic header.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DoIpPayloadType {
    GenericNegativeAck,
    VehicleIdentificationRequest,
    VehicleIdentificationRequestWithEid,
    VehicleIdentificationRequestWithVin,
    VehicleAnnouncement,
    RoutingActivationRequest,
    RoutingActivationResponse,
    AliveCheckRequest,
    AliveCheckResponse,
    EntityStatusRequest,
    EntityStatusResponse,
    DiagnosticPowerModeRequest,
    DiagnosticPowerModeResponse,
    DiagnosticMessage,
    DiagnosticPositiveAck,
    DiagnosticNegativeAck,
    Unknown(u16),
}

impl DoIpPayloadType {
    pub fn from_u16(value: u16) -> Self {
        match value {
            0x0000 => Self::GenericNegativeAck,
            0x0001 => Self::VehicleIdentificationRequest,
            0x0002 => Self::VehicleIdentificationRequestWithEid,
            0x0003 => Self::VehicleIdentificationRequestWithVin,
            0x0004 => Self::VehicleAnnouncement,
            0x0005 => Self::RoutingActivationRequest,
            0x0006 => Self::RoutingActivationResponse,
            0x0007 => Self::AliveCheckRequest,
            0x0008 => Self::AliveCheckResponse,
            0x4001 => Self::EntityStatusRequest,
            0x4002 => Self::EntityStatusResponse,
            0x4003 => Self::DiagnosticPowerModeRequest,
            0x4004 => Self::DiagnosticPowerModeResponse,
            0x8001 => Self::DiagnosticMessage,
            0x8002 => Self::DiagnosticPositiveAck,
            0x8003 => Self::DiagnosticNegativeAck,
            other => Self::Unknown(other),
        }
    }

    pub fn as_u16(self) -> u16 {
        match self {
            Self::GenericNegativeAck => 0x0000,
            Self::VehicleIdentificationRequest => 0x0001,
            Self::VehicleIdentificationRequestWithEid => 0x0002,
            Self::VehicleIdentificationRequestWithVin => 0x0003,
            Self::VehicleAnnouncement => 0x0004,
            Self::RoutingActivationRequest => 0x0005,
            Self::RoutingActivationResponse => 0x0006,
            Self::AliveCheckRequest => 0x0007,
            Self::AliveCheckResponse => 0x0008,
            Self::EntityStatusRequest => 0x4001,
            Self::EntityStatusResponse => 0x4002,
            Self::DiagnosticPowerModeRequest => 0x4003,
            Self::DiagnosticPowerModeResponse => 0x4004,
            Self::DiagnosticMessage => 0x8001,
            Self::DiagnosticPositiveAck => 0x8002,
            Self::DiagnosticNegativeAck => 0x8003,
            Self::Unknown(value) => value,
        }
    }
}

/// DoIP generic header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DoIpHeader {
    pub protocol_version: u8,
    pub inverse_protocol_version: u8,
    pub payload_type: DoIpPayloadType,
    pub payload_length: u32,
}

impl DoIpHeader {
    pub fn new(payload_type: DoIpPayloadType, payload_length: u32) -> Result<Self, Obd2Error> {
        Self::with_version(DOIP_VERSION_DEFAULT, payload_type, payload_length)
    }

    pub fn with_version(
        protocol_version: u8,
        payload_type: DoIpPayloadType,
        payload_length: u32,
    ) -> Result<Self, Obd2Error> {
        if payload_length > DOIP_MAX_PAYLOAD_LEN {
            return Err(Obd2Error::ParseError(format!(
                "DoIP payload length {payload_length} exceeds configured maximum {DOIP_MAX_PAYLOAD_LEN}"
            )));
        }
        Ok(Self {
            protocol_version,
            inverse_protocol_version: !protocol_version,
            payload_type,
            payload_length,
        })
    }

    pub fn parse(bytes: &[u8]) -> Result<Self, Obd2Error> {
        if bytes.len() < DOIP_HEADER_LEN {
            return Err(Obd2Error::ParseError(format!(
                "DoIP header requires {DOIP_HEADER_LEN} bytes, got {}",
                bytes.len()
            )));
        }
        let protocol_version = bytes[0];
        let inverse_protocol_version = bytes[1];
        if inverse_protocol_version != !protocol_version {
            return Err(Obd2Error::ParseError(format!(
                "DoIP inverse protocol version mismatch: version 0x{protocol_version:02X}, inverse 0x{inverse_protocol_version:02X}"
            )));
        }
        let payload_type = DoIpPayloadType::from_u16(u16::from_be_bytes([bytes[2], bytes[3]]));
        let payload_length = u32::from_be_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
        if payload_length > DOIP_MAX_PAYLOAD_LEN {
            return Err(Obd2Error::ParseError(format!(
                "DoIP payload length {payload_length} exceeds configured maximum {DOIP_MAX_PAYLOAD_LEN}"
            )));
        }
        Ok(Self {
            protocol_version,
            inverse_protocol_version,
            payload_type,
            payload_length,
        })
    }

    pub fn encode(self) -> [u8; DOIP_HEADER_LEN] {
        let mut bytes = [0u8; DOIP_HEADER_LEN];
        bytes[0] = self.protocol_version;
        bytes[1] = self.inverse_protocol_version;
        bytes[2..4].copy_from_slice(&self.payload_type.as_u16().to_be_bytes());
        bytes[4..8].copy_from_slice(&self.payload_length.to_be_bytes());
        bytes
    }
}

/// Complete DoIP packet.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DoIpMessage {
    pub header: DoIpHeader,
    pub payload: Vec<u8>,
}

impl DoIpMessage {
    pub fn new(payload_type: DoIpPayloadType, payload: Vec<u8>) -> Result<Self, Obd2Error> {
        Self::with_version(DOIP_VERSION_DEFAULT, payload_type, payload)
    }

    pub fn with_version(
        protocol_version: u8,
        payload_type: DoIpPayloadType,
        payload: Vec<u8>,
    ) -> Result<Self, Obd2Error> {
        let payload_length = u32::try_from(payload.len()).map_err(|_| {
            Obd2Error::ParseError(format!(
                "DoIP payload length {} exceeds u32 header range",
                payload.len()
            ))
        })?;
        let header = DoIpHeader::with_version(protocol_version, payload_type, payload_length)?;
        Ok(Self { header, payload })
    }

    pub fn diagnostic(
        source_address: u16,
        target_address: u16,
        user_data: Vec<u8>,
    ) -> Result<Self, Obd2Error> {
        Self::new(
            DoIpPayloadType::DiagnosticMessage,
            DoIpDiagnosticMessage {
                source_address,
                target_address,
                user_data,
            }
            .to_payload(),
        )
    }

    pub fn routing_activation_request(
        request: DoIpRoutingActivationRequest,
    ) -> Result<Self, Obd2Error> {
        Self::new(
            DoIpPayloadType::RoutingActivationRequest,
            request.to_payload(),
        )
    }

    pub fn parse(bytes: &[u8]) -> Result<Self, Obd2Error> {
        let header = DoIpHeader::parse(bytes)?;
        let total_len = DOIP_HEADER_LEN
            .checked_add(header.payload_length as usize)
            .ok_or_else(|| Obd2Error::ParseError("DoIP packet length overflow".into()))?;
        if bytes.len() != total_len {
            return Err(Obd2Error::ParseError(format!(
                "DoIP packet length mismatch: header says {total_len} bytes, buffer has {}",
                bytes.len()
            )));
        }
        Ok(Self {
            header,
            payload: bytes[DOIP_HEADER_LEN..].to_vec(),
        })
    }

    /// Parse one complete DoIP frame prefix from a stream buffer.
    ///
    /// Returns `Ok(None)` when the buffer contains a valid prefix but not enough
    /// bytes for the complete packet yet.
    pub fn parse_prefix(bytes: &[u8]) -> Result<Option<(Self, usize)>, Obd2Error> {
        if bytes.len() < DOIP_HEADER_LEN {
            return Ok(None);
        }
        let header = DoIpHeader::parse(bytes)?;
        let total_len = DOIP_HEADER_LEN
            .checked_add(header.payload_length as usize)
            .ok_or_else(|| Obd2Error::ParseError("DoIP packet length overflow".into()))?;
        if bytes.len() < total_len {
            return Ok(None);
        }
        Ok(Some((
            Self {
                header,
                payload: bytes[DOIP_HEADER_LEN..total_len].to_vec(),
            },
            total_len,
        )))
    }

    pub fn encode(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(DOIP_HEADER_LEN + self.payload.len());
        bytes.extend_from_slice(&self.header.encode());
        bytes.extend_from_slice(&self.payload);
        bytes
    }
}

/// Generic DoIP negative acknowledgement payload (payload type 0x0000).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DoIpGenericNegativeAck {
    pub nack_code: u8,
}

impl DoIpGenericNegativeAck {
    pub fn parse(payload: &[u8]) -> Result<Self, Obd2Error> {
        if payload.len() != 1 {
            return Err(Obd2Error::ParseError(format!(
                "DoIP generic negative ACK payload must be 1 byte, got {}",
                payload.len()
            )));
        }
        Ok(Self {
            nack_code: payload[0],
        })
    }

    pub fn to_payload(self) -> [u8; 1] {
        [self.nack_code]
    }
}

/// Diagnostic message payload (payload type 0x8001).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DoIpDiagnosticMessage {
    pub source_address: u16,
    pub target_address: u16,
    pub user_data: Vec<u8>,
}

impl DoIpDiagnosticMessage {
    pub fn parse(payload: &[u8]) -> Result<Self, Obd2Error> {
        if payload.len() < 4 {
            return Err(Obd2Error::ParseError(format!(
                "DoIP diagnostic payload requires at least 4 bytes, got {}",
                payload.len()
            )));
        }
        Ok(Self {
            source_address: u16::from_be_bytes([payload[0], payload[1]]),
            target_address: u16::from_be_bytes([payload[2], payload[3]]),
            user_data: payload[4..].to_vec(),
        })
    }

    pub fn to_payload(&self) -> Vec<u8> {
        let mut payload = Vec::with_capacity(4 + self.user_data.len());
        payload.extend_from_slice(&self.source_address.to_be_bytes());
        payload.extend_from_slice(&self.target_address.to_be_bytes());
        payload.extend_from_slice(&self.user_data);
        payload
    }
}

/// Diagnostic ACK/NACK payload (payload types 0x8002 and 0x8003).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DoIpDiagnosticAck {
    pub source_address: u16,
    pub target_address: u16,
    pub ack_code: u8,
    pub previous_message_data: Vec<u8>,
}

impl DoIpDiagnosticAck {
    pub fn parse(payload: &[u8]) -> Result<Self, Obd2Error> {
        if payload.len() < 5 {
            return Err(Obd2Error::ParseError(format!(
                "DoIP diagnostic ACK payload requires at least 5 bytes, got {}",
                payload.len()
            )));
        }
        Ok(Self {
            source_address: u16::from_be_bytes([payload[0], payload[1]]),
            target_address: u16::from_be_bytes([payload[2], payload[3]]),
            ack_code: payload[4],
            previous_message_data: payload[5..].to_vec(),
        })
    }

    pub fn to_payload(&self) -> Vec<u8> {
        let mut payload = Vec::with_capacity(5 + self.previous_message_data.len());
        payload.extend_from_slice(&self.source_address.to_be_bytes());
        payload.extend_from_slice(&self.target_address.to_be_bytes());
        payload.push(self.ack_code);
        payload.extend_from_slice(&self.previous_message_data);
        payload
    }
}

/// Routing activation request payload (payload type 0x0005).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DoIpRoutingActivationRequest {
    pub source_address: u16,
    pub activation_type: u8,
    pub reserved_iso: [u8; 4],
    pub reserved_oem: Option<[u8; 4]>,
}

impl DoIpRoutingActivationRequest {
    pub fn new(source_address: u16, activation_type: u8) -> Self {
        Self {
            source_address,
            activation_type,
            reserved_iso: [0; 4],
            reserved_oem: None,
        }
    }

    pub fn parse(payload: &[u8]) -> Result<Self, Obd2Error> {
        if payload.len() != 7 && payload.len() != 11 {
            return Err(Obd2Error::ParseError(format!(
                "DoIP routing activation request must be 7 or 11 bytes, got {}",
                payload.len()
            )));
        }
        let mut reserved_iso = [0u8; 4];
        reserved_iso.copy_from_slice(&payload[3..7]);
        let reserved_oem = if payload.len() == 11 {
            let mut bytes = [0u8; 4];
            bytes.copy_from_slice(&payload[7..11]);
            Some(bytes)
        } else {
            None
        };
        Ok(Self {
            source_address: u16::from_be_bytes([payload[0], payload[1]]),
            activation_type: payload[2],
            reserved_iso,
            reserved_oem,
        })
    }

    pub fn to_payload(self) -> Vec<u8> {
        let mut payload = Vec::with_capacity(if self.reserved_oem.is_some() { 11 } else { 7 });
        payload.extend_from_slice(&self.source_address.to_be_bytes());
        payload.push(self.activation_type);
        payload.extend_from_slice(&self.reserved_iso);
        if let Some(reserved_oem) = self.reserved_oem {
            payload.extend_from_slice(&reserved_oem);
        }
        payload
    }
}

/// Routing activation response payload (payload type 0x0006).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DoIpRoutingActivationResponse {
    pub tester_address: u16,
    pub entity_address: u16,
    pub response_code: u8,
    pub reserved_iso: [u8; 4],
    pub reserved_oem: Option<[u8; 4]>,
}

impl DoIpRoutingActivationResponse {
    pub fn parse(payload: &[u8]) -> Result<Self, Obd2Error> {
        if payload.len() != 9 && payload.len() != 13 {
            return Err(Obd2Error::ParseError(format!(
                "DoIP routing activation response must be 9 or 13 bytes, got {}",
                payload.len()
            )));
        }
        let mut reserved_iso = [0u8; 4];
        reserved_iso.copy_from_slice(&payload[5..9]);
        let reserved_oem = if payload.len() == 13 {
            let mut bytes = [0u8; 4];
            bytes.copy_from_slice(&payload[9..13]);
            Some(bytes)
        } else {
            None
        };
        Ok(Self {
            tester_address: u16::from_be_bytes([payload[0], payload[1]]),
            entity_address: u16::from_be_bytes([payload[2], payload[3]]),
            response_code: payload[4],
            reserved_iso,
            reserved_oem,
        })
    }

    pub fn to_payload(self) -> Vec<u8> {
        let mut payload = Vec::with_capacity(if self.reserved_oem.is_some() { 13 } else { 9 });
        payload.extend_from_slice(&self.tester_address.to_be_bytes());
        payload.extend_from_slice(&self.entity_address.to_be_bytes());
        payload.push(self.response_code);
        payload.extend_from_slice(&self.reserved_iso);
        if let Some(reserved_oem) = self.reserved_oem {
            payload.extend_from_slice(&reserved_oem);
        }
        payload
    }
}

/// Vehicle identification response / announcement payload (payload type 0x0004).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DoIpVehicleIdentification {
    pub vin: [u8; 17],
    pub logical_address: u16,
    pub eid: [u8; 6],
    pub gid: [u8; 6],
    pub further_action_required: u8,
    pub vin_gid_sync_status: Option<u8>,
}

impl DoIpVehicleIdentification {
    pub fn parse(payload: &[u8]) -> Result<Self, Obd2Error> {
        if payload.len() != 32 && payload.len() != 33 {
            return Err(Obd2Error::ParseError(format!(
                "DoIP vehicle-identification payload must be 32 or 33 bytes, got {}",
                payload.len()
            )));
        }
        let mut vin = [0u8; 17];
        vin.copy_from_slice(&payload[0..17]);
        let mut eid = [0u8; 6];
        eid.copy_from_slice(&payload[19..25]);
        let mut gid = [0u8; 6];
        gid.copy_from_slice(&payload[25..31]);
        Ok(Self {
            vin,
            logical_address: u16::from_be_bytes([payload[17], payload[18]]),
            eid,
            gid,
            further_action_required: payload[31],
            vin_gid_sync_status: payload.get(32).copied(),
        })
    }

    pub fn to_payload(&self) -> Vec<u8> {
        let mut payload = Vec::with_capacity(if self.vin_gid_sync_status.is_some() {
            33
        } else {
            32
        });
        payload.extend_from_slice(&self.vin);
        payload.extend_from_slice(&self.logical_address.to_be_bytes());
        payload.extend_from_slice(&self.eid);
        payload.extend_from_slice(&self.gid);
        payload.push(self.further_action_required);
        if let Some(sync_status) = self.vin_gid_sync_status {
            payload.push(sync_status);
        }
        payload
    }
}

/// Async DoIP packet transport.
///
/// Implementations exchange complete DoIP messages. Stream-oriented backends
/// should use [`DoIpStreamTransport`] unless they need socket-specific policy.
#[async_trait::async_trait]
pub trait DoIpTransport: Send {
    async fn send(&mut self, message: &DoIpMessage) -> Result<(), Obd2Error>;
    async fn recv(&mut self) -> Result<DoIpMessage, Obd2Error>;

    async fn exchange(&mut self, message: &DoIpMessage) -> Result<DoIpMessage, Obd2Error> {
        self.send(message).await?;
        self.recv().await
    }
}

/// DoIP transport over an already-open async byte stream.
///
/// This type performs exact DoIP header/body framing. Opening TCP sockets is
/// left to the backend because this crate currently does not enable Tokio's
/// `net` feature in its normal dependency set.
pub struct DoIpStreamTransport<S> {
    stream: S,
}

impl<S> DoIpStreamTransport<S> {
    pub fn new(stream: S) -> Self {
        Self { stream }
    }

    pub fn into_inner(self) -> S {
        self.stream
    }

    pub fn stream(&self) -> &S {
        &self.stream
    }

    pub fn stream_mut(&mut self) -> &mut S {
        &mut self.stream
    }
}

#[async_trait::async_trait]
impl<S> DoIpTransport for DoIpStreamTransport<S>
where
    S: AsyncRead + AsyncWrite + Unpin + Send,
{
    async fn send(&mut self, message: &DoIpMessage) -> Result<(), Obd2Error> {
        self.stream.write_all(&message.header.encode()).await?;
        self.stream.write_all(&message.payload).await?;
        self.stream.flush().await?;
        Ok(())
    }

    async fn recv(&mut self) -> Result<DoIpMessage, Obd2Error> {
        let mut header_bytes = [0u8; DOIP_HEADER_LEN];
        self.stream.read_exact(&mut header_bytes).await?;
        let header = DoIpHeader::parse(&header_bytes)?;

        let mut payload = vec![0u8; header.payload_length as usize];
        self.stream.read_exact(&mut payload).await?;
        Ok(DoIpMessage { header, payload })
    }
}

/// Diagnostic client over an activated DoIP transport.
#[derive(Debug)]
pub struct DoIpClient<T: DoIpTransport> {
    transport: T,
    tester_address: u16,
    entity_address: u16,
}

impl<T: DoIpTransport> DoIpClient<T> {
    pub fn new(transport: T, tester_address: u16, entity_address: u16) -> Self {
        Self {
            transport,
            tester_address,
            entity_address,
        }
    }

    pub fn tester_address(&self) -> u16 {
        self.tester_address
    }

    pub fn entity_address(&self) -> u16 {
        self.entity_address
    }

    pub fn set_entity_address(&mut self, entity_address: u16) {
        self.entity_address = entity_address;
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn into_inner(self) -> T {
        self.transport
    }

    pub async fn routing_activation(
        &mut self,
        activation_type: u8,
    ) -> Result<DoIpRoutingActivationResponse, Obd2Error> {
        let request = DoIpRoutingActivationRequest::new(self.tester_address, activation_type);
        let message = DoIpMessage::routing_activation_request(request)?;
        let response = self.transport.exchange(&message).await?;

        if response.header.payload_type == DoIpPayloadType::GenericNegativeAck {
            return doip_generic_nack_error(&response.payload);
        }
        if response.header.payload_type != DoIpPayloadType::RoutingActivationResponse {
            return Err(Obd2Error::ParseError(format!(
                "expected DoIP routing activation response, got payload type 0x{:04X}",
                response.header.payload_type.as_u16()
            )));
        }

        let activation = DoIpRoutingActivationResponse::parse(&response.payload)?;
        if activation.tester_address != self.tester_address {
            return Err(Obd2Error::ParseError(format!(
                "DoIP routing activation response tester address 0x{:04X} does not match request 0x{:04X}",
                activation.tester_address, self.tester_address
            )));
        }
        self.entity_address = activation.entity_address;
        Ok(activation)
    }

    pub async fn activate_routing(
        &mut self,
        activation_type: u8,
    ) -> Result<DoIpRoutingActivationResponse, Obd2Error> {
        let activation = self.routing_activation(activation_type).await?;
        if activation.response_code == DOIP_ROUTING_ACTIVATION_SUCCESS {
            Ok(activation)
        } else {
            Err(Obd2Error::Transport(format!(
                "DoIP routing activation failed with response code 0x{:02X}",
                activation.response_code
            )))
        }
    }

    /// Send one diagnostic PDU and return the diagnostic response PDU.
    ///
    /// The returned bytes are the diagnostic user data, including the response
    /// SID. Positive diagnostic ACKs are consumed. Negative ACKs are surfaced as
    /// transport errors with the DoIP ACK code preserved.
    pub async fn send_diagnostic(&mut self, user_data: &[u8]) -> Result<Vec<u8>, Obd2Error> {
        if user_data.is_empty() {
            return Err(Obd2Error::ParseError(
                "DoIP diagnostic user data is empty".into(),
            ));
        }

        let request =
            DoIpMessage::diagnostic(self.tester_address, self.entity_address, user_data.to_vec())?;
        self.transport.send(&request).await?;

        for _ in 0..DOIP_MAX_DIAGNOSTIC_EVENTS {
            let response = self.transport.recv().await?;
            match response.header.payload_type {
                DoIpPayloadType::DiagnosticMessage => {
                    let diag = DoIpDiagnosticMessage::parse(&response.payload)?;
                    self.validate_response_addresses(
                        diag.source_address,
                        diag.target_address,
                        "diagnostic response",
                    )?;
                    return Ok(diag.user_data);
                }
                DoIpPayloadType::DiagnosticPositiveAck => {
                    let ack = DoIpDiagnosticAck::parse(&response.payload)?;
                    self.validate_response_addresses(
                        ack.source_address,
                        ack.target_address,
                        "diagnostic positive ACK",
                    )?;
                }
                DoIpPayloadType::DiagnosticNegativeAck => {
                    let ack = DoIpDiagnosticAck::parse(&response.payload)?;
                    self.validate_response_addresses(
                        ack.source_address,
                        ack.target_address,
                        "diagnostic negative ACK",
                    )?;
                    return Err(Obd2Error::Transport(format!(
                        "DoIP diagnostic negative ACK 0x{:02X}",
                        ack.ack_code
                    )));
                }
                DoIpPayloadType::GenericNegativeAck => {
                    return doip_generic_nack_error(&response.payload);
                }
                other => {
                    return Err(Obd2Error::ParseError(format!(
                        "unexpected DoIP payload type 0x{:04X} during diagnostic exchange",
                        other.as_u16()
                    )));
                }
            }
        }

        Err(Obd2Error::Timeout)
    }

    fn validate_response_addresses(
        &self,
        source_address: u16,
        target_address: u16,
        label: &str,
    ) -> Result<(), Obd2Error> {
        if source_address != self.entity_address || target_address != self.tester_address {
            return Err(Obd2Error::ParseError(format!(
                "DoIP {label} addresses source=0x{source_address:04X} target=0x{target_address:04X}; expected source=0x{:04X} target=0x{:04X}",
                self.entity_address, self.tester_address
            )));
        }
        Ok(())
    }
}

fn doip_generic_nack_error<T>(payload: &[u8]) -> Result<T, Obd2Error> {
    let nack = DoIpGenericNegativeAck::parse(payload)?;
    Err(Obd2Error::Transport(format!(
        "DoIP generic negative ACK 0x{:02X}",
        nack.nack_code
    )))
}

pub fn encode_doip_message(
    payload_type: DoIpPayloadType,
    payload: &[u8],
) -> Result<Vec<u8>, Obd2Error> {
    DoIpMessage::new(payload_type, payload.to_vec()).map(|message| message.encode())
}

pub fn parse_doip_message(bytes: &[u8]) -> Result<DoIpMessage, Obd2Error> {
    DoIpMessage::parse(bytes)
}

pub fn parse_doip_message_prefix(bytes: &[u8]) -> Result<Option<(DoIpMessage, usize)>, Obd2Error> {
    DoIpMessage::parse_prefix(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};

    #[test]
    fn doip_header_round_trips() {
        let header = DoIpHeader::new(DoIpPayloadType::DiagnosticMessage, 6).unwrap();
        assert_eq!(header.encode(), [0x02, 0xFD, 0x80, 0x01, 0, 0, 0, 6]);

        let parsed = DoIpHeader::parse(&header.encode()).unwrap();
        assert_eq!(parsed, header);
    }

    #[test]
    fn doip_rejects_bad_inverse_version() {
        let err = DoIpHeader::parse(&[0x02, 0x02, 0x80, 0x01, 0, 0, 0, 0]).unwrap_err();
        assert!(err.to_string().contains("inverse protocol version"));
    }

    #[test]
    fn doip_rejects_oversize_payload_len() {
        let mut bytes = [0x02, 0xFD, 0x80, 0x01, 0, 0, 0, 0];
        bytes[4..8].copy_from_slice(&(DOIP_MAX_PAYLOAD_LEN + 1).to_be_bytes());

        let err = DoIpHeader::parse(&bytes).unwrap_err();
        assert!(err.to_string().contains("exceeds configured maximum"));
    }

    #[test]
    fn doip_rejects_packet_length_mismatch() {
        let message = DoIpMessage::diagnostic(0x0E00, 0x1001, vec![0x22, 0xF1, 0x90]).unwrap();
        let mut encoded = message.encode();
        encoded.pop();

        let err = DoIpMessage::parse(&encoded).unwrap_err();
        assert!(err.to_string().contains("packet length mismatch"));
    }

    #[test]
    fn diagnostic_message_round_trips() {
        let message = DoIpMessage::diagnostic(0x0E00, 0x1001, vec![0x22, 0xF1, 0x90]).unwrap();
        let encoded = message.encode();
        let parsed = DoIpMessage::parse(&encoded).unwrap();
        assert_eq!(
            parsed.header.payload_type,
            DoIpPayloadType::DiagnosticMessage
        );

        let diag = DoIpDiagnosticMessage::parse(&parsed.payload).unwrap();
        assert_eq!(diag.source_address, 0x0E00);
        assert_eq!(diag.target_address, 0x1001);
        assert_eq!(diag.user_data, vec![0x22, 0xF1, 0x90]);
    }

    #[test]
    fn diagnostic_ack_round_trips_and_rejects_short_payload() {
        let ack = DoIpDiagnosticAck {
            source_address: 0x1001,
            target_address: 0x0E00,
            ack_code: 0,
            previous_message_data: vec![0x22, 0xF1, 0x90],
        };

        assert_eq!(DoIpDiagnosticAck::parse(&ack.to_payload()).unwrap(), ack);
        assert!(matches!(
            DoIpDiagnosticAck::parse(&[0x10, 0x01, 0x0E, 0x00]),
            Err(Obd2Error::ParseError(_))
        ));
    }

    #[test]
    fn generic_negative_ack_round_trips_and_rejects_bad_len() {
        let nack = DoIpGenericNegativeAck { nack_code: 0x04 };

        assert_eq!(
            DoIpGenericNegativeAck::parse(&nack.to_payload()).unwrap(),
            nack
        );
        assert!(matches!(
            DoIpGenericNegativeAck::parse(&[]),
            Err(Obd2Error::ParseError(_))
        ));
    }

    #[test]
    fn parse_prefix_handles_incomplete_and_coalesced_stream_data() {
        let first = DoIpMessage::diagnostic(0x0E00, 0x1001, vec![0x22, 0xF1, 0x90]).unwrap();
        let second = DoIpMessage::new(DoIpPayloadType::AliveCheckRequest, Vec::new()).unwrap();
        let mut stream = first.encode();
        stream.extend_from_slice(&second.encode());

        assert!(DoIpMessage::parse_prefix(&stream[..10]).unwrap().is_none());

        let (parsed, consumed) = DoIpMessage::parse_prefix(&stream).unwrap().unwrap();
        assert_eq!(parsed, first);
        assert_eq!(consumed, first.encode().len());
        assert_eq!(DoIpMessage::parse(&stream[consumed..]).unwrap(), second);
    }

    #[test]
    fn routing_activation_request_round_trips() {
        let request = DoIpRoutingActivationRequest::new(0x0E00, 0x00);
        let payload = request.to_payload();
        assert_eq!(payload.len(), 7);
        assert_eq!(
            DoIpRoutingActivationRequest::parse(&payload).unwrap(),
            request
        );
    }

    #[test]
    fn routing_activation_response_round_trips() {
        let response = DoIpRoutingActivationResponse {
            tester_address: 0x0E00,
            entity_address: 0x1001,
            response_code: DOIP_ROUTING_ACTIVATION_SUCCESS,
            reserved_iso: [0, 0, 0, 0],
            reserved_oem: Some([1, 2, 3, 4]),
        };

        assert_eq!(
            DoIpRoutingActivationResponse::parse(&response.to_payload()).unwrap(),
            response
        );
    }

    #[test]
    fn vehicle_identification_response_parses() {
        let mut payload = Vec::new();
        payload.extend_from_slice(b"1HGCM82633A004352");
        payload.extend_from_slice(&0x1001u16.to_be_bytes());
        payload.extend_from_slice(&[1, 2, 3, 4, 5, 6]);
        payload.extend_from_slice(&[7, 8, 9, 10, 11, 12]);
        payload.push(0x00);
        payload.push(0x10);

        let vehicle = DoIpVehicleIdentification::parse(&payload).unwrap();
        assert_eq!(&vehicle.vin, b"1HGCM82633A004352");
        assert_eq!(vehicle.logical_address, 0x1001);
        assert_eq!(vehicle.vin_gid_sync_status, Some(0x10));
        assert_eq!(
            DoIpVehicleIdentification::parse(&vehicle.to_payload()).unwrap(),
            vehicle
        );
    }

    #[tokio::test]
    async fn stream_transport_writes_and_reads_exact_frames() {
        let request = DoIpMessage::diagnostic(0x0E00, 0x1001, vec![0x22, 0xF1, 0x90]).unwrap();
        let response = DoIpMessage::diagnostic(0x1001, 0x0E00, vec![0x62, 0xF1, 0x90]).unwrap();
        let expected_request = request.encode();
        let response_bytes = response.encode();
        let (client_io, mut server_io) = tokio::io::duplex(1024);
        let server = tokio::spawn(async move {
            let mut received = vec![0u8; expected_request.len()];
            server_io.read_exact(&mut received).await.unwrap();
            assert_eq!(received, expected_request);
            server_io.write_all(&response_bytes).await.unwrap();
        });

        let mut transport = DoIpStreamTransport::new(client_io);
        transport.send(&request).await.unwrap();
        let parsed = transport.recv().await.unwrap();
        assert_eq!(parsed, response);

        server.await.unwrap();
    }

    #[derive(Debug)]
    struct MockDoIpTransport {
        sent: Vec<DoIpMessage>,
        rx: VecDeque<DoIpMessage>,
    }

    #[async_trait::async_trait]
    impl DoIpTransport for MockDoIpTransport {
        async fn send(&mut self, message: &DoIpMessage) -> Result<(), Obd2Error> {
            self.sent.push(message.clone());
            Ok(())
        }

        async fn recv(&mut self) -> Result<DoIpMessage, Obd2Error> {
            self.rx.pop_front().ok_or(Obd2Error::Timeout)
        }
    }

    fn message(payload_type: DoIpPayloadType, payload: Vec<u8>) -> DoIpMessage {
        DoIpMessage::new(payload_type, payload).unwrap()
    }

    #[tokio::test]
    async fn doip_client_activates_routing_and_updates_entity_address() {
        let activation = DoIpRoutingActivationResponse {
            tester_address: 0x0E00,
            entity_address: 0x1001,
            response_code: DOIP_ROUTING_ACTIVATION_SUCCESS,
            reserved_iso: [0; 4],
            reserved_oem: None,
        };
        let transport = MockDoIpTransport {
            sent: Vec::new(),
            rx: VecDeque::from([message(
                DoIpPayloadType::RoutingActivationResponse,
                activation.to_payload(),
            )]),
        };
        let mut client = DoIpClient::new(transport, 0x0E00, 0);

        let parsed = client.activate_routing(0).await.unwrap();
        assert_eq!(parsed, activation);
        assert_eq!(client.entity_address(), 0x1001);

        let transport = client.into_inner();
        assert_eq!(transport.sent.len(), 1);
        assert_eq!(
            transport.sent[0].header.payload_type,
            DoIpPayloadType::RoutingActivationRequest
        );
    }

    #[tokio::test]
    async fn doip_client_consumes_positive_ack_and_returns_diagnostic_payload() {
        let ack = DoIpDiagnosticAck {
            source_address: 0x1001,
            target_address: 0x0E00,
            ack_code: 0,
            previous_message_data: Vec::new(),
        };
        let response = DoIpDiagnosticMessage {
            source_address: 0x1001,
            target_address: 0x0E00,
            user_data: vec![0x62, 0xF1, 0x90],
        };
        let transport = MockDoIpTransport {
            sent: Vec::new(),
            rx: VecDeque::from([
                message(DoIpPayloadType::DiagnosticPositiveAck, ack.to_payload()),
                message(DoIpPayloadType::DiagnosticMessage, response.to_payload()),
            ]),
        };
        let mut client = DoIpClient::new(transport, 0x0E00, 0x1001);

        let user_data = client.send_diagnostic(&[0x22, 0xF1, 0x90]).await.unwrap();
        assert_eq!(user_data, vec![0x62, 0xF1, 0x90]);
    }

    #[tokio::test]
    async fn doip_client_surfaces_diagnostic_negative_ack() {
        let nack = DoIpDiagnosticAck {
            source_address: 0x1001,
            target_address: 0x0E00,
            ack_code: 0x02,
            previous_message_data: Vec::new(),
        };
        let transport = MockDoIpTransport {
            sent: Vec::new(),
            rx: VecDeque::from([message(
                DoIpPayloadType::DiagnosticNegativeAck,
                nack.to_payload(),
            )]),
        };
        let mut client = DoIpClient::new(transport, 0x0E00, 0x1001);

        let err = client
            .send_diagnostic(&[0x22, 0xF1, 0x90])
            .await
            .unwrap_err();
        assert!(err.to_string().contains("negative ACK 0x02"));
    }
}
