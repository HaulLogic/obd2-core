//! J1939 heavy-duty vehicle protocol support.
//!
//! SAE J1939 is the standard for heavy-duty truck communication over CAN bus
//! (29-bit extended identifiers, 250 kbps). This module provides:
//!
//! - [`Pgn`] type with constants for common parameter groups
//! - Decoder functions for fleet-critical PGNs (engine, vehicle speed, temps)
//! - [`J1939Dtc`] type using SPN+FMI format (distinct from OBD-II P-codes)
//!
//! ## Transport setup
//!
//! J1939 requests require a transport configured for 29-bit CAN at the
//! vehicle's bus speed. This module only defines PGNs and payload decoding;
//! bus selection and request framing stay at the transport/client boundary.
//!
//! ## PGN request format
//!
//! A J1939 request message uses a 29-bit CAN ID:
//! ```text
//! Priority(3) | Reserved(1) | Data Page(1) | PDU Format(8) | PDU Specific(8) | Source Address(8)
//! ```
//! For destination-specific PGNs (PDU Format < 240), PDU Specific = destination address.
//! For broadcast PGNs (PDU Format >= 240), PDU Specific is part of the PGN.

use crate::error::Obd2Error;

/// J1939 global destination address.
pub const GLOBAL_ADDRESS: u8 = 0xFF;

/// J1939 null address. A node uses this when it cannot claim a source address.
pub const NULL_ADDRESS: u8 = 0xFE;

/// Common off-board diagnostic tool source address.
pub const DEFAULT_TOOL_ADDRESS: u8 = 0xF9;

/// Default priority used by request/control traffic.
pub const DEFAULT_PRIORITY: u8 = 6;

/// Maximum 18-bit J1939 Parameter Group Number value.
pub const MAX_PGN: u32 = 0x03_FFFF;

/// Maximum 19-bit J1939 Suspect Parameter Number value.
pub const MAX_SPN: u32 = 0x07_FFFF;

/// Maximum classic CAN payload carried by a single J1939 frame.
pub const MAX_FRAME_DATA_LEN: usize = 8;

/// Bytes carried by each TP.DT packet after the sequence byte.
pub const TP_PACKET_DATA_LEN: usize = 7;

/// Maximum payload length for J1939-21 transport protocol transfers.
pub const TP_MAX_MESSAGE_SIZE: usize = 1785;

/// A J1939 Parameter Group Number.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Pgn(pub u32);

impl Pgn {
    // ── Engine ──

    /// Electronic Engine Controller 1 — engine speed, torque.
    /// 8 bytes, broadcast, 100ms default rate.
    pub const EEC1: Pgn = Pgn(61444);

    /// Engine Temperature 1 — coolant temp, fuel temp.
    /// 8 bytes, broadcast, 1000ms default rate.
    pub const ET1: Pgn = Pgn(65262);

    /// Engine Fluid Level/Pressure 1 — oil pressure, coolant pressure, oil level.
    /// 8 bytes, broadcast, 500ms default rate.
    pub const EFLP1: Pgn = Pgn(65263);

    /// Fuel Economy (Liquid) — fuel rate, instantaneous fuel economy.
    /// 8 bytes, broadcast, 100ms default rate.
    pub const LFE: Pgn = Pgn(65266);

    // ── Vehicle ──

    /// Cruise Control/Vehicle Speed — vehicle speed, brake, cruise control.
    /// 8 bytes, broadcast, 100ms default rate.
    pub const CCVS: Pgn = Pgn(65265);

    // ── Diagnostics ──

    /// DM1 — Active Diagnostic Trouble Codes.
    /// Variable length, broadcast, 1000ms default rate.
    pub const DM1: Pgn = Pgn(65226);

    /// DM2 — Previously Active Diagnostic Trouble Codes.
    /// Variable length, on-request.
    pub const DM2: Pgn = Pgn(65227);

    /// Request — asks another ECU to transmit a PGN.
    pub const REQUEST: Pgn = Pgn(59904);

    /// Acknowledgement — ACK/NACK response for group functions.
    pub const ACKNOWLEDGEMENT: Pgn = Pgn(59392);

    /// Address Claimed / Cannot Claim Address.
    pub const ADDRESS_CLAIMED: Pgn = Pgn(60928);

    /// J1939 Transport Protocol Connection Management.
    pub const TP_CM: Pgn = Pgn(60416);

    /// J1939 Transport Protocol Data Transfer.
    pub const TP_DT: Pgn = Pgn(60160);

    /// DM3 — Diagnostic Data Clear/Reset for Previously Active DTCs.
    pub const DM3: Pgn = Pgn(65228);

    /// DM5 — Diagnostic Readiness 1.
    pub const DM5: Pgn = Pgn(65230);

    /// DM11 — Diagnostic Data Clear/Reset for Active DTCs.
    pub const DM11: Pgn = Pgn(65235);

    /// DM24 — SPNs Supported.
    pub const DM24: Pgn = Pgn(64950);

    /// DM25 — Expanded Freeze Frame.
    pub const DM25: Pgn = Pgn(64951);

    /// Build a PGN from the 3-byte little-endian wire order used in Request/TP.CM data.
    pub fn from_le_bytes(bytes: [u8; 3]) -> Self {
        Self(bytes[0] as u32 | ((bytes[1] as u32) << 8) | ((bytes[2] as u32) << 16))
    }

    /// Return the 3-byte little-endian wire order used in Request/TP.CM data.
    pub fn to_le_bytes(self) -> [u8; 3] {
        [
            (self.0 & 0xFF) as u8,
            ((self.0 >> 8) & 0xFF) as u8,
            ((self.0 >> 16) & 0xFF) as u8,
        ]
    }

    /// Return the PDU Format byte.
    pub fn pdu_format(self) -> u8 {
        ((self.0 >> 8) & 0xFF) as u8
    }

    /// Return the PDU Specific byte.
    pub fn pdu_specific(self) -> u8 {
        (self.0 & 0xFF) as u8
    }

    /// True for destination-specific PDU1 PGNs.
    pub fn is_pdu1(self) -> bool {
        self.pdu_format() < 240
    }

    /// Return the canonical PGN value for CAN identifier encoding.
    ///
    /// PDU1 PGNs do not include the destination address in the PGN low byte.
    pub fn normalized(self) -> Self {
        if self.is_pdu1() {
            Self(self.0 & 0x03_FF00)
        } else {
            Self(self.0 & MAX_PGN)
        }
    }

    /// Check whether this value fits in the 18-bit J1939 PGN field.
    pub fn is_valid(self) -> bool {
        self.0 <= MAX_PGN
    }

    /// Return the PGN name, if known.
    pub fn name(&self) -> &'static str {
        match self.0 {
            61444 => "EEC1 (Electronic Engine Controller 1)",
            65262 => "ET1 (Engine Temperature 1)",
            65263 => "EFLP1 (Engine Fluid Level/Pressure 1)",
            65265 => "CCVS (Cruise Control/Vehicle Speed)",
            65266 => "LFE (Fuel Economy - Liquid)",
            65226 => "DM1 (Active DTCs)",
            65227 => "DM2 (Previously Active DTCs)",
            65228 => "DM3 (Clear Previously Active DTCs)",
            65230 => "DM5 (Diagnostic Readiness 1)",
            65235 => "DM11 (Clear Active DTCs)",
            64950 => "DM24 (SPNs Supported)",
            64951 => "DM25 (Expanded Freeze Frame)",
            59392 => "Acknowledgement",
            59904 => "Request",
            60160 => "TP.DT (Transport Protocol Data Transfer)",
            60416 => "TP.CM (Transport Protocol Connection Management)",
            60928 => "Address Claimed",
            _ => "Unknown PGN",
        }
    }
}

impl std::fmt::Display for Pgn {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "PGN {} ({})", self.0, self.name())
    }
}

/// A J1939 Suspect Parameter Number.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Spn(pub u32);

impl Spn {
    /// Engine Speed.
    pub const ENGINE_SPEED: Spn = Spn(190);
    /// Engine Oil Pressure.
    pub const ENGINE_OIL_PRESSURE: Spn = Spn(100);
    /// Engine Coolant Temperature.
    pub const ENGINE_COOLANT_TEMPERATURE: Spn = Spn(110);

    pub fn try_new(value: u32) -> Result<Self, Obd2Error> {
        if value <= MAX_SPN {
            Ok(Self(value))
        } else {
            Err(Obd2Error::ParseError(format!(
                "SPN {value} exceeds 19-bit J1939 range"
            )))
        }
    }

    pub fn as_u32(self) -> u32 {
        self.0
    }
}

impl std::fmt::Display for Spn {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "SPN {}", self.0)
    }
}

/// A J1939 Failure Mode Identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Fmi(pub u8);

impl Fmi {
    pub fn try_new(value: u8) -> Result<Self, Obd2Error> {
        if value <= 31 {
            Ok(Self(value))
        } else {
            Err(Obd2Error::ParseError(format!(
                "FMI {value} exceeds 5-bit J1939 range"
            )))
        }
    }

    pub fn description(self) -> &'static str {
        fmi_description(self.0)
    }
}

impl std::fmt::Display for Fmi {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "FMI {} ({})", self.0, self.description())
    }
}

/// Parsed 29-bit J1939 CAN identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct J1939CanId {
    pub priority: u8,
    pub pgn: Pgn,
    pub source: u8,
    pub destination: Option<u8>,
}

impl J1939CanId {
    pub fn new(
        priority: u8,
        pgn: Pgn,
        source: u8,
        destination: Option<u8>,
    ) -> Result<Self, Obd2Error> {
        if priority > 7 {
            return Err(Obd2Error::ParseError(format!(
                "J1939 priority {priority} exceeds 3-bit range"
            )));
        }
        if !pgn.is_valid() {
            return Err(Obd2Error::ParseError(format!(
                "PGN {} exceeds 18-bit J1939 range",
                pgn.0
            )));
        }
        if pgn.is_pdu1() && destination.is_none() {
            return Err(Obd2Error::ParseError(format!(
                "PDU1 PGN {} requires a destination address",
                pgn.0
            )));
        }

        Ok(Self {
            priority,
            pgn: pgn.normalized(),
            source,
            destination,
        })
    }

    pub fn encode(self) -> u32 {
        let pgn = self.pgn.normalized();
        let ps = if pgn.is_pdu1() {
            self.destination.unwrap_or(GLOBAL_ADDRESS) as u32
        } else {
            pgn.0 & 0xFF
        };
        let rdp_pf = pgn.0 & 0x03_FF00;
        ((self.priority as u32) << 26) | (rdp_pf << 8) | (ps << 8) | self.source as u32
    }

    pub fn decode(identifier: u32) -> Result<Self, Obd2Error> {
        if identifier > 0x1FFF_FFFF {
            return Err(Obd2Error::ParseError(format!(
                "CAN identifier 0x{identifier:08X} exceeds 29-bit J1939 range"
            )));
        }

        let priority = ((identifier >> 26) & 0x07) as u8;
        let source = (identifier & 0xFF) as u8;
        let pf = ((identifier >> 16) & 0xFF) as u8;
        let ps = ((identifier >> 8) & 0xFF) as u8;
        let raw_pgn = (identifier >> 8) & MAX_PGN;
        let (pgn, destination) = if pf < 240 {
            (Pgn(raw_pgn & 0x03_FF00), Some(ps))
        } else {
            (Pgn(raw_pgn), None)
        };

        Ok(Self {
            priority,
            pgn,
            source,
            destination,
        })
    }
}

/// One classic CAN data frame carrying a J1939 message fragment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct J1939Frame {
    pub id: J1939CanId,
    data: [u8; MAX_FRAME_DATA_LEN],
    len: u8,
}

impl J1939Frame {
    pub fn new(id: J1939CanId, payload: &[u8]) -> Result<Self, Obd2Error> {
        if payload.len() > MAX_FRAME_DATA_LEN {
            return Err(Obd2Error::ParseError(format!(
                "J1939 frame payload is {} bytes; maximum is {MAX_FRAME_DATA_LEN}",
                payload.len()
            )));
        }

        let mut data = [0u8; MAX_FRAME_DATA_LEN];
        data[..payload.len()].copy_from_slice(payload);
        Ok(Self {
            id,
            data,
            len: payload.len() as u8,
        })
    }

    pub fn from_parts(
        priority: u8,
        pgn: Pgn,
        source: u8,
        destination: Option<u8>,
        payload: &[u8],
    ) -> Result<Self, Obd2Error> {
        Self::new(
            J1939CanId::new(priority, pgn, source, destination)?,
            payload,
        )
    }

    pub fn payload(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }

    pub fn can_identifier(&self) -> u32 {
        self.id.encode()
    }
}

/// A complete J1939 application-layer message after any TP reassembly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct J1939Message {
    pub source: u8,
    pub destination: Option<u8>,
    pub pgn: Pgn,
    pub payload: Vec<u8>,
}

impl J1939Message {
    pub fn from_frame(frame: &J1939Frame) -> Self {
        Self {
            source: frame.id.source,
            destination: frame.id.destination,
            pgn: frame.id.pgn,
            payload: frame.payload().to_vec(),
        }
    }
}

/// A J1939 Request PGN command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct J1939Request {
    pub source: u8,
    pub destination: u8,
    pub requested_pgn: Pgn,
}

impl J1939Request {
    pub fn new(source: u8, destination: u8, requested_pgn: Pgn) -> Self {
        Self {
            source,
            destination,
            requested_pgn,
        }
    }

    pub fn global(source: u8, requested_pgn: Pgn) -> Self {
        Self::new(source, GLOBAL_ADDRESS, requested_pgn)
    }

    pub fn to_frame(self) -> Result<J1939Frame, Obd2Error> {
        let mut payload = [0x00; 3];
        payload[..3].copy_from_slice(&self.requested_pgn.to_le_bytes());
        J1939Frame::from_parts(
            DEFAULT_PRIORITY,
            Pgn::REQUEST,
            self.source,
            Some(self.destination),
            &payload,
        )
    }
}

/// SAE J1939-81 NAME value used during address claiming.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct J1939Name(pub u64);

impl J1939Name {
    pub fn from_payload(payload: &[u8]) -> Result<Self, Obd2Error> {
        if payload.len() < 8 {
            return Err(Obd2Error::ParseError(format!(
                "address-claim NAME requires 8 bytes, got {}",
                payload.len()
            )));
        }
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&payload[..8]);
        Ok(Self(u64::from_le_bytes(bytes)))
    }

    pub fn to_payload(self) -> [u8; 8] {
        self.0.to_le_bytes()
    }

    pub fn identity_number(self) -> u32 {
        (self.0 & 0x1F_FFFF) as u32
    }

    pub fn manufacturer_code(self) -> u16 {
        ((self.0 >> 21) & 0x07FF) as u16
    }

    pub fn ecu_instance(self) -> u8 {
        ((self.0 >> 32) & 0x07) as u8
    }

    pub fn function_instance(self) -> u8 {
        ((self.0 >> 35) & 0x1F) as u8
    }

    pub fn function(self) -> u8 {
        ((self.0 >> 40) & 0xFF) as u8
    }

    pub fn vehicle_system(self) -> u8 {
        ((self.0 >> 49) & 0x7F) as u8
    }

    pub fn vehicle_system_instance(self) -> u8 {
        ((self.0 >> 56) & 0x0F) as u8
    }

    pub fn industry_group(self) -> u8 {
        ((self.0 >> 60) & 0x07) as u8
    }

    pub fn arbitrary_address_capable(self) -> bool {
        ((self.0 >> 63) & 0x01) != 0
    }

    /// Lower NAME value wins J1939 address-claim arbitration.
    pub fn wins_arbitration_against(self, other: Self) -> bool {
        self.0 < other.0
    }
}

/// Parsed address-claim frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AddressClaim {
    pub source_address: u8,
    pub name: J1939Name,
}

impl AddressClaim {
    pub fn parse(frame: &J1939Frame) -> Result<Self, Obd2Error> {
        if frame.id.pgn != Pgn::ADDRESS_CLAIMED {
            return Err(Obd2Error::ParseError(format!(
                "expected address-claim PGN {}, got {}",
                Pgn::ADDRESS_CLAIMED.0,
                frame.id.pgn.0
            )));
        }
        Ok(Self {
            source_address: frame.id.source,
            name: J1939Name::from_payload(frame.payload())?,
        })
    }
}

/// J1939 acknowledgement control byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AcknowledgementControl {
    Ack,
    NegativeAck,
    AccessDenied,
    CannotRespond,
    Reserved(u8),
}

impl AcknowledgementControl {
    pub fn from_byte(byte: u8) -> Self {
        match byte {
            0 => Self::Ack,
            1 => Self::NegativeAck,
            2 => Self::AccessDenied,
            3 => Self::CannotRespond,
            other => Self::Reserved(other),
        }
    }

    pub fn as_byte(self) -> u8 {
        match self {
            Self::Ack => 0,
            Self::NegativeAck => 1,
            Self::AccessDenied => 2,
            Self::CannotRespond => 3,
            Self::Reserved(value) => value,
        }
    }
}

/// J1939 acknowledgement PGN payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Acknowledgement {
    pub control: AcknowledgementControl,
    pub group_function_value: u8,
    pub address_acknowledged: u8,
    pub pgn: Pgn,
}

impl Acknowledgement {
    pub fn parse(payload: &[u8]) -> Result<Self, Obd2Error> {
        if payload.len() < 8 {
            return Err(Obd2Error::ParseError(format!(
                "J1939 acknowledgement requires 8 bytes, got {}",
                payload.len()
            )));
        }
        Ok(Self {
            control: AcknowledgementControl::from_byte(payload[0]),
            group_function_value: payload[1],
            address_acknowledged: payload[4],
            pgn: Pgn::from_le_bytes([payload[5], payload[6], payload[7]]),
        })
    }

    pub fn to_payload(self) -> [u8; 8] {
        let mut payload = [0xFF; 8];
        payload[0] = self.control.as_byte();
        payload[1] = self.group_function_value;
        payload[4] = self.address_acknowledged;
        payload[5..8].copy_from_slice(&self.pgn.to_le_bytes());
        payload
    }
}

/// J1939-21 TP.CM control message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TpControlMessage {
    RequestToSend {
        message_size: u16,
        packets: u8,
        max_packets_per_cts: u8,
        pgn: Pgn,
    },
    ClearToSend {
        packets_to_send: u8,
        next_packet: u8,
        pgn: Pgn,
    },
    EndOfMessageAck {
        message_size: u16,
        packets: u8,
        pgn: Pgn,
    },
    BroadcastAnnounce {
        message_size: u16,
        packets: u8,
        pgn: Pgn,
    },
    ConnectionAbort {
        reason: u8,
        pgn: Pgn,
    },
    Unknown {
        control: u8,
        data: [u8; 7],
    },
}

impl TpControlMessage {
    pub const RTS: u8 = 0x10;
    pub const CTS: u8 = 0x11;
    pub const EOM_ACK: u8 = 0x13;
    pub const BAM: u8 = 0x20;
    pub const ABORT: u8 = 0xFF;

    pub fn parse(payload: &[u8]) -> Result<Self, Obd2Error> {
        if payload.len() < 8 {
            return Err(Obd2Error::ParseError(format!(
                "TP.CM payload requires 8 bytes, got {}",
                payload.len()
            )));
        }

        let message_size = u16::from_le_bytes([payload[1], payload[2]]);
        let pgn = Pgn::from_le_bytes([payload[5], payload[6], payload[7]]);
        Ok(match payload[0] {
            Self::RTS => Self::RequestToSend {
                message_size,
                packets: payload[3],
                max_packets_per_cts: payload[4],
                pgn,
            },
            Self::CTS => Self::ClearToSend {
                packets_to_send: payload[1],
                next_packet: payload[2],
                pgn,
            },
            Self::EOM_ACK => Self::EndOfMessageAck {
                message_size,
                packets: payload[3],
                pgn,
            },
            Self::BAM => Self::BroadcastAnnounce {
                message_size,
                packets: payload[3],
                pgn,
            },
            Self::ABORT => Self::ConnectionAbort {
                reason: payload[1],
                pgn,
            },
            control => {
                let mut data = [0u8; 7];
                data.copy_from_slice(&payload[1..8]);
                Self::Unknown { control, data }
            }
        })
    }

    pub fn to_payload(self) -> [u8; 8] {
        let mut payload = [0xFF; 8];
        match self {
            Self::RequestToSend {
                message_size,
                packets,
                max_packets_per_cts,
                pgn,
            } => {
                payload[0] = Self::RTS;
                payload[1..3].copy_from_slice(&message_size.to_le_bytes());
                payload[3] = packets;
                payload[4] = max_packets_per_cts;
                payload[5..8].copy_from_slice(&pgn.to_le_bytes());
            }
            Self::ClearToSend {
                packets_to_send,
                next_packet,
                pgn,
            } => {
                payload[0] = Self::CTS;
                payload[1] = packets_to_send;
                payload[2] = next_packet;
                payload[5..8].copy_from_slice(&pgn.to_le_bytes());
            }
            Self::EndOfMessageAck {
                message_size,
                packets,
                pgn,
            } => {
                payload[0] = Self::EOM_ACK;
                payload[1..3].copy_from_slice(&message_size.to_le_bytes());
                payload[3] = packets;
                payload[5..8].copy_from_slice(&pgn.to_le_bytes());
            }
            Self::BroadcastAnnounce {
                message_size,
                packets,
                pgn,
            } => {
                payload[0] = Self::BAM;
                payload[1..3].copy_from_slice(&message_size.to_le_bytes());
                payload[3] = packets;
                payload[5..8].copy_from_slice(&pgn.to_le_bytes());
            }
            Self::ConnectionAbort { reason, pgn } => {
                payload[0] = Self::ABORT;
                payload[1] = reason;
                payload[5..8].copy_from_slice(&pgn.to_le_bytes());
            }
            Self::Unknown { control, data } => {
                payload[0] = control;
                payload[1..8].copy_from_slice(&data);
            }
        }
        payload
    }

    pub fn pgn(self) -> Option<Pgn> {
        match self {
            Self::RequestToSend { pgn, .. }
            | Self::ClearToSend { pgn, .. }
            | Self::EndOfMessageAck { pgn, .. }
            | Self::BroadcastAnnounce { pgn, .. }
            | Self::ConnectionAbort { pgn, .. } => Some(pgn),
            Self::Unknown { .. } => None,
        }
    }
}

/// One TP.DT packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TpDataTransfer {
    pub sequence: u8,
    pub data: [u8; TP_PACKET_DATA_LEN],
}

impl TpDataTransfer {
    pub fn parse(payload: &[u8]) -> Result<Self, Obd2Error> {
        if payload.len() < 8 {
            return Err(Obd2Error::ParseError(format!(
                "TP.DT payload requires 8 bytes, got {}",
                payload.len()
            )));
        }
        let mut data = [0u8; TP_PACKET_DATA_LEN];
        data.copy_from_slice(&payload[1..8]);
        Ok(Self {
            sequence: payload[0],
            data,
        })
    }

    pub fn to_payload(self) -> [u8; 8] {
        let mut payload = [0xFF; 8];
        payload[0] = self.sequence;
        payload[1..8].copy_from_slice(&self.data);
        payload
    }
}

/// Stateful TP.BAM/TP.CM reassembler for one source/destination/session.
#[derive(Debug, Clone)]
pub struct TpReassembler {
    source: u8,
    destination: Option<u8>,
    pgn: Pgn,
    expected_size: usize,
    expected_packets: u8,
    next_sequence: u8,
    payload: Vec<u8>,
}

impl TpReassembler {
    pub fn start(
        source: u8,
        destination: Option<u8>,
        control: TpControlMessage,
    ) -> Result<Self, Obd2Error> {
        let (message_size, expected_packets, pgn) = match control {
            TpControlMessage::BroadcastAnnounce {
                message_size,
                packets,
                pgn,
            }
            | TpControlMessage::RequestToSend {
                message_size,
                packets,
                pgn,
                ..
            } => (message_size as usize, packets, pgn),
            _ => {
                return Err(Obd2Error::ParseError(
                    "TP reassembly must start with BAM or RTS".into(),
                ))
            }
        };

        validate_tp_size(message_size, expected_packets)?;
        Ok(Self {
            source,
            destination,
            pgn,
            expected_size: message_size,
            expected_packets,
            next_sequence: 1,
            payload: Vec::with_capacity(message_size),
        })
    }

    pub fn accept_dt(&mut self, packet: TpDataTransfer) -> Result<Option<J1939Message>, Obd2Error> {
        if packet.sequence != self.next_sequence {
            return Err(Obd2Error::ParseError(format!(
                "TP.DT sequence mismatch: expected {}, got {}",
                self.next_sequence, packet.sequence
            )));
        }
        if packet.sequence > self.expected_packets {
            return Err(Obd2Error::ParseError(format!(
                "TP.DT sequence {} exceeds expected packet count {}",
                packet.sequence, self.expected_packets
            )));
        }

        let remaining = self.expected_size.saturating_sub(self.payload.len());
        let take = remaining.min(TP_PACKET_DATA_LEN);
        self.payload.extend_from_slice(&packet.data[..take]);
        self.next_sequence = self.next_sequence.saturating_add(1);

        if self.payload.len() == self.expected_size {
            Ok(Some(J1939Message {
                source: self.source,
                destination: self.destination,
                pgn: self.pgn,
                payload: std::mem::take(&mut self.payload),
            }))
        } else {
            Ok(None)
        }
    }
}

fn validate_tp_size(message_size: usize, packets: u8) -> Result<(), Obd2Error> {
    if message_size == 0 || message_size > TP_MAX_MESSAGE_SIZE {
        return Err(Obd2Error::ParseError(format!(
            "TP message size {message_size} outside 1..={TP_MAX_MESSAGE_SIZE}"
        )));
    }
    let required_packets = message_size.div_ceil(TP_PACKET_DATA_LEN);
    if required_packets != packets as usize {
        return Err(Obd2Error::ParseError(format!(
            "TP packet count {packets} does not match {message_size} byte payload"
        )));
    }
    Ok(())
}

// ── Decoded Parameter Groups ──

/// Decoded Electronic Engine Controller 1 (PGN 61444).
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF sentinel).
#[derive(Debug, Clone)]
pub struct Eec1 {
    /// Engine speed in RPM. SPN 190, bytes 4-5.
    pub engine_rpm: Option<f64>,
    /// Driver's demand engine torque as percent. SPN 512, byte 2.
    pub driver_demand_torque_pct: Option<f64>,
    /// Actual engine torque as percent. SPN 513, byte 3.
    pub actual_torque_pct: Option<f64>,
    /// Engine torque mode. SPN 899, byte 1 bits 0-3.
    pub torque_mode: u8,
}

/// Decoded Cruise Control/Vehicle Speed (PGN 65265).
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF sentinel).
#[derive(Debug, Clone)]
pub struct Ccvs {
    /// Vehicle speed in km/h. SPN 84, bytes 2-3.
    pub vehicle_speed: Option<f64>,
    /// Brake switch active. SPN 597, byte 4 bits 2-3. `None` if not available.
    pub brake_switch: Option<bool>,
    /// Cruise control active. SPN 595, byte 1 bits 0-1. `None` if not available.
    pub cruise_active: Option<bool>,
}

/// Decoded Engine Temperature 1 (PGN 65262).
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF sentinel).
#[derive(Debug, Clone)]
pub struct Et1 {
    /// Engine coolant temperature in °C. SPN 110, byte 1.
    pub coolant_temp: Option<f64>,
    /// Fuel temperature in °C. SPN 174, byte 2.
    pub fuel_temp: Option<f64>,
    /// Engine oil temperature in °C. SPN 175, bytes 3-4.
    pub oil_temp: Option<f64>,
}

/// Decoded Engine Fluid Level/Pressure 1 (PGN 65263).
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF sentinel).
#[derive(Debug, Clone)]
pub struct Eflp1 {
    /// Engine oil pressure in kPa. SPN 100, byte 4.
    pub oil_pressure: Option<f64>,
    /// Coolant pressure in kPa. SPN 109, byte 2.
    pub coolant_pressure: Option<f64>,
}

/// Decoded Fuel Economy - Liquid (PGN 65266).
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF sentinel).
#[derive(Debug, Clone)]
pub struct Lfe {
    /// Engine fuel rate in L/h. SPN 183, bytes 1-2.
    pub fuel_rate: Option<f64>,
    /// Instantaneous fuel economy in km/L. SPN 184, bytes 3-4.
    pub instantaneous_fuel_economy: Option<f64>,
}

fn fmi_description(fmi: u8) -> &'static str {
    match fmi {
        0 => "Data Valid But Above Normal Operational Range - Most Severe",
        1 => "Data Valid But Below Normal Operational Range - Most Severe",
        2 => "Data Erratic, Intermittent Or Incorrect",
        3 => "Voltage Above Normal, Or Shorted To High Source",
        4 => "Voltage Below Normal, Or Shorted To Low Source",
        5 => "Current Below Normal Or Open Circuit",
        6 => "Current Above Normal Or Grounded Circuit",
        7 => "Mechanical System Not Responding Or Out Of Adjustment",
        8 => "Abnormal Frequency Or Pulse Width Or Period",
        9 => "Abnormal Update Rate",
        10 => "Abnormal Rate Of Change",
        11 => "Root Cause Not Known",
        12 => "Bad Intelligent Device Or Component",
        13 => "Out Of Calibration",
        14 => "Special Instructions",
        15 => "Data Valid But Above Normal Operating Range - Least Severe",
        16 => "Data Valid But Above Normal Operating Range - Moderately Severe",
        17 => "Data Valid But Below Normal Operating Range - Least Severe",
        18 => "Data Valid But Below Normal Operating Range - Moderately Severe",
        19 => "Received Network Data In Error",
        20 => "Data Drifted High",
        21 => "Data Drifted Low",
        31 => "Condition Exists",
        _ => "Reserved",
    }
}

/// A J1939 Diagnostic Trouble Code (SPN + FMI format).
///
/// Unlike OBD-II P-codes, J1939 uses Suspect Parameter Number (SPN) to identify
/// the faulting parameter and Failure Mode Identifier (FMI) to describe the
/// failure type.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct J1939Dtc {
    /// Suspect Parameter Number — identifies the parameter at fault.
    pub spn: u32,
    /// Failure Mode Identifier — describes the type of failure (0-31).
    pub fmi: u8,
    /// Occurrence count (0-126, 127 = not available).
    pub occurrence_count: u8,
    /// SPN Conversion Method (0 = standard, 1 = extended).
    pub conversion_method: u8,
}

impl J1939Dtc {
    /// Decode a J1939 DTC from the 4-byte DM1/DM2 format.
    ///
    /// Byte layout:
    /// - Bytes 0-1: SPN bits 0-15 (little-endian)
    /// - Byte 2 bits 5-7: SPN bits 16-18
    /// - Byte 2 bits 0-4: FMI
    /// - Byte 3 bit 7: SPN Conversion Method
    /// - Byte 3 bits 0-6: Occurrence Count
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 4 {
            return None;
        }
        let spn_low = u16::from_le_bytes([data[0], data[1]]) as u32;
        let spn_high = ((data[2] >> 5) & 0x07) as u32;
        let spn = spn_low | (spn_high << 16);
        let fmi = data[2] & 0x1F;
        let conversion_method = (data[3] >> 7) & 0x01;
        let occurrence_count = data[3] & 0x7F;

        Some(Self {
            spn,
            fmi,
            occurrence_count,
            conversion_method,
        })
    }

    /// Encode this DTC into the 4-byte DM1/DM2 wire format.
    pub fn to_bytes(&self) -> Result<[u8; 4], Obd2Error> {
        if self.spn > MAX_SPN {
            return Err(Obd2Error::ParseError(format!(
                "SPN {} exceeds 19-bit J1939 range",
                self.spn
            )));
        }
        if self.fmi > 31 {
            return Err(Obd2Error::ParseError(format!(
                "FMI {} exceeds 5-bit J1939 range",
                self.fmi
            )));
        }
        if self.occurrence_count > 127 {
            return Err(Obd2Error::ParseError(format!(
                "DTC occurrence count {} exceeds 7-bit J1939 range",
                self.occurrence_count
            )));
        }
        if self.conversion_method > 1 {
            return Err(Obd2Error::ParseError(format!(
                "DTC conversion method {} exceeds 1-bit J1939 range",
                self.conversion_method
            )));
        }

        Ok([
            (self.spn & 0xFF) as u8,
            ((self.spn >> 8) & 0xFF) as u8,
            (((self.spn >> 16) as u8) << 5) | (self.fmi & 0x1F),
            ((self.conversion_method & 0x01) << 7) | (self.occurrence_count & 0x7F),
        ])
    }

    /// Human-readable FMI description.
    pub fn fmi_description(&self) -> &'static str {
        fmi_description(self.fmi)
    }

    pub fn spn_id(&self) -> Spn {
        Spn(self.spn)
    }

    pub fn fmi_id(&self) -> Fmi {
        Fmi(self.fmi)
    }
}

impl std::fmt::Display for J1939Dtc {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "SPN {} FMI {} ({})",
            self.spn,
            self.fmi,
            self.fmi_description()
        )
    }
}

/// J1939 DM lamp on/off status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LampStatus {
    Off,
    On,
    Reserved,
    NotAvailable,
}

impl LampStatus {
    fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 => Self::Off,
            1 => Self::On,
            2 => Self::Reserved,
            _ => Self::NotAvailable,
        }
    }

    fn bits(self) -> u8 {
        match self {
            Self::Off => 0,
            Self::On => 1,
            Self::Reserved => 2,
            Self::NotAvailable => 3,
        }
    }
}

/// J1939 DM lamp flashing status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LampFlash {
    SlowFlash,
    FastFlash,
    Reserved,
    NotAvailable,
}

impl LampFlash {
    fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 => Self::SlowFlash,
            1 => Self::FastFlash,
            2 => Self::Reserved,
            _ => Self::NotAvailable,
        }
    }

    fn bits(self) -> u8 {
        match self {
            Self::SlowFlash => 0,
            Self::FastFlash => 1,
            Self::Reserved => 2,
            Self::NotAvailable => 3,
        }
    }
}

/// DM1/DM2 lamp bytes decoded into the four legislated lamps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DmLampStatus {
    pub raw_status: u8,
    pub raw_flash: u8,
    pub protect: LampStatus,
    pub amber_warning: LampStatus,
    pub red_stop: LampStatus,
    pub malfunction_indicator: LampStatus,
    pub protect_flash: LampFlash,
    pub amber_warning_flash: LampFlash,
    pub red_stop_flash: LampFlash,
    pub malfunction_indicator_flash: LampFlash,
}

impl DmLampStatus {
    pub fn parse(status: u8, flash: u8) -> Self {
        Self {
            raw_status: status,
            raw_flash: flash,
            protect: LampStatus::from_bits(status),
            amber_warning: LampStatus::from_bits(status >> 2),
            red_stop: LampStatus::from_bits(status >> 4),
            malfunction_indicator: LampStatus::from_bits(status >> 6),
            protect_flash: LampFlash::from_bits(flash),
            amber_warning_flash: LampFlash::from_bits(flash >> 2),
            red_stop_flash: LampFlash::from_bits(flash >> 4),
            malfunction_indicator_flash: LampFlash::from_bits(flash >> 6),
        }
    }

    pub fn to_bytes(self) -> [u8; 2] {
        [
            self.protect.bits()
                | (self.amber_warning.bits() << 2)
                | (self.red_stop.bits() << 4)
                | (self.malfunction_indicator.bits() << 6),
            self.protect_flash.bits()
                | (self.amber_warning_flash.bits() << 2)
                | (self.red_stop_flash.bits() << 4)
                | (self.malfunction_indicator_flash.bits() << 6),
        ]
    }
}

/// Parsed DM1 or DM2 diagnostic message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DmDtcMessage {
    pub lamps: DmLampStatus,
    pub dtcs: Vec<J1939Dtc>,
}

impl DmDtcMessage {
    pub fn to_payload(&self) -> Result<Vec<u8>, Obd2Error> {
        let mut payload = Vec::with_capacity(2 + self.dtcs.len() * 4);
        payload.extend_from_slice(&self.lamps.to_bytes());
        for dtc in &self.dtcs {
            payload.extend_from_slice(&dtc.to_bytes()?);
        }
        Ok(payload)
    }
}

/// Parsed DM5 Diagnostic Readiness 1.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Dm5 {
    pub active_dtc_count: Option<u8>,
    pub previously_active_dtc_count: Option<u8>,
    pub obd_compliance: u8,
    pub monitor_readiness: [u8; 5],
}

impl Dm5 {
    pub fn to_payload(&self) -> [u8; 8] {
        let mut payload = [0u8; 8];
        payload[0] = self.active_dtc_count.unwrap_or(NA_BYTE);
        payload[1] = self.previously_active_dtc_count.unwrap_or(NA_BYTE);
        payload[2] = self.obd_compliance;
        payload[3..8].copy_from_slice(&self.monitor_readiness);
        payload
    }
}

/// One DM24 supported-SPN entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Dm24Entry {
    pub spn: Spn,
    pub support_bits: u8,
    pub data_length: u8,
}

impl Dm24Entry {
    pub fn to_payload(self) -> Result<[u8; 4], Obd2Error> {
        if self.spn.0 > MAX_SPN {
            return Err(Obd2Error::ParseError(format!(
                "SPN {} exceeds 19-bit J1939 range",
                self.spn.0
            )));
        }
        if self.support_bits > 31 {
            return Err(Obd2Error::ParseError(format!(
                "DM24 support bits 0x{:02X} exceed 5-bit J1939 range",
                self.support_bits
            )));
        }

        Ok([
            (self.spn.0 & 0xFF) as u8,
            ((self.spn.0 >> 8) & 0xFF) as u8,
            (((self.spn.0 >> 16) as u8) << 5) | (self.support_bits & 0x1F),
            self.data_length,
        ])
    }
}

/// Parsed DM24 SPNs Supported message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Dm24 {
    pub entries: Vec<Dm24Entry>,
}

impl Dm24 {
    pub fn to_payload(&self) -> Result<Vec<u8>, Obd2Error> {
        let mut payload = Vec::with_capacity(self.entries.len() * 4);
        for entry in &self.entries {
            payload.extend_from_slice(&entry.to_payload()?);
        }
        Ok(payload)
    }
}

/// One DM25 expanded-freeze-frame block.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Dm25FreezeFrame {
    pub length: u8,
    pub dtc: J1939Dtc,
    pub data: Vec<u8>,
}

impl Dm25FreezeFrame {
    pub fn to_payload(&self) -> Result<Vec<u8>, Obd2Error> {
        let actual_len = 4usize
            .checked_add(self.data.len())
            .ok_or_else(|| Obd2Error::ParseError("DM25 freeze-frame length overflow".into()))?;
        if actual_len > u8::MAX as usize {
            return Err(Obd2Error::ParseError(format!(
                "DM25 freeze-frame data length {} exceeds 251 byte maximum",
                self.data.len()
            )));
        }
        if self.length as usize != actual_len {
            return Err(Obd2Error::ParseError(format!(
                "DM25 freeze-frame length {} does not match DTC+data length {}",
                self.length, actual_len
            )));
        }

        let mut payload = Vec::with_capacity(1 + actual_len);
        payload.push(self.length);
        payload.extend_from_slice(&self.dtc.to_bytes()?);
        payload.extend_from_slice(&self.data);
        Ok(payload)
    }
}

/// Parsed DM25 Expanded Freeze Frame message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Dm25 {
    pub frames: Vec<Dm25FreezeFrame>,
}

impl Dm25 {
    pub fn to_payload(&self) -> Result<Vec<u8>, Obd2Error> {
        let mut payload = Vec::new();
        for frame in &self.frames {
            payload.extend_from_slice(&frame.to_payload()?);
        }
        Ok(payload)
    }
}

/// Minimal async raw J1939 frame I/O boundary for CAN backends.
#[async_trait::async_trait]
pub trait J1939FrameIo: Send {
    async fn send_frame(&mut self, frame: J1939Frame) -> Result<(), Obd2Error>;
    async fn recv_frame(&mut self) -> Result<J1939Frame, Obd2Error>;
}

/// Client-side J1939-73 diagnostic transport seam.
///
/// Implementations own CAN, TP.BAM/TP.CM, filtering, timeouts, and address claiming.
/// This trait intentionally exchanges already-typed PGN requests/messages.
#[async_trait::async_trait]
pub trait J1939Transport: Send {
    async fn request_pgn(&mut self, request: &J1939Request) -> Result<J1939Message, Obd2Error>;
}

/// J1939-73 diagnostic client over a J1939 transport.
#[derive(Debug)]
pub struct J1939Client<T: J1939Transport> {
    transport: T,
    source: u8,
    destination: u8,
}

impl<T: J1939Transport> J1939Client<T> {
    pub fn new(transport: T, source: u8) -> Self {
        Self {
            transport,
            source,
            destination: GLOBAL_ADDRESS,
        }
    }

    pub fn with_destination(mut self, destination: u8) -> Self {
        self.destination = destination;
        self
    }

    pub fn into_inner(self) -> T {
        self.transport
    }

    pub async fn request_dm1(&mut self) -> Result<DmDtcMessage, Obd2Error> {
        let msg = self.request(Pgn::DM1).await?;
        parse_dm1(&expect_pgn(msg, Pgn::DM1)?)
    }

    pub async fn request_dm2(&mut self) -> Result<DmDtcMessage, Obd2Error> {
        let msg = self.request(Pgn::DM2).await?;
        parse_dm2(&expect_pgn(msg, Pgn::DM2)?)
    }

    pub async fn clear_dm3(&mut self) -> Result<Acknowledgement, Obd2Error> {
        let msg = self.request(Pgn::DM3).await?;
        parse_dm3_response(&expect_pgn(msg, Pgn::ACKNOWLEDGEMENT)?)
    }

    pub async fn clear_dm11(&mut self) -> Result<Acknowledgement, Obd2Error> {
        let msg = self.request(Pgn::DM11).await?;
        parse_dm11_response(&expect_pgn(msg, Pgn::ACKNOWLEDGEMENT)?)
    }

    pub async fn request_dm5(&mut self) -> Result<Dm5, Obd2Error> {
        let msg = self.request(Pgn::DM5).await?;
        parse_dm5(&expect_pgn(msg, Pgn::DM5)?)
    }

    pub async fn request_dm24(&mut self) -> Result<Dm24, Obd2Error> {
        let msg = self.request(Pgn::DM24).await?;
        parse_dm24(&expect_pgn(msg, Pgn::DM24)?)
    }

    pub async fn request_dm25(&mut self) -> Result<Dm25, Obd2Error> {
        let msg = self.request(Pgn::DM25).await?;
        parse_dm25(&expect_pgn(msg, Pgn::DM25)?)
    }

    async fn request(&mut self, pgn: Pgn) -> Result<J1939Message, Obd2Error> {
        let request = J1939Request::new(self.source, self.destination, pgn);
        self.transport.request_pgn(&request).await
    }
}

pub fn request_dm1(source: u8, destination: u8) -> J1939Request {
    J1939Request::new(source, destination, Pgn::DM1)
}

pub fn request_dm2(source: u8, destination: u8) -> J1939Request {
    J1939Request::new(source, destination, Pgn::DM2)
}

pub fn request_dm3(source: u8, destination: u8) -> J1939Request {
    J1939Request::new(source, destination, Pgn::DM3)
}

pub fn request_dm5(source: u8, destination: u8) -> J1939Request {
    J1939Request::new(source, destination, Pgn::DM5)
}

pub fn request_dm11(source: u8, destination: u8) -> J1939Request {
    J1939Request::new(source, destination, Pgn::DM11)
}

pub fn request_dm24(source: u8, destination: u8) -> J1939Request {
    J1939Request::new(source, destination, Pgn::DM24)
}

pub fn request_dm25(source: u8, destination: u8) -> J1939Request {
    J1939Request::new(source, destination, Pgn::DM25)
}

pub fn parse_dm1(data: &[u8]) -> Result<DmDtcMessage, Obd2Error> {
    parse_dm_dtc_message(data, "DM1")
}

pub fn parse_dm2(data: &[u8]) -> Result<DmDtcMessage, Obd2Error> {
    parse_dm_dtc_message(data, "DM2")
}

pub fn encode_dm1(message: &DmDtcMessage) -> Result<Vec<u8>, Obd2Error> {
    message.to_payload()
}

pub fn encode_dm2(message: &DmDtcMessage) -> Result<Vec<u8>, Obd2Error> {
    message.to_payload()
}

pub fn parse_dm3_response(data: &[u8]) -> Result<Acknowledgement, Obd2Error> {
    let ack = Acknowledgement::parse(data)?;
    if ack.pgn == Pgn::DM3 {
        Ok(ack)
    } else {
        Err(Obd2Error::ParseError(format!(
            "DM3 acknowledgement referenced PGN {}, expected {}",
            ack.pgn.0,
            Pgn::DM3.0
        )))
    }
}

pub fn parse_dm11_response(data: &[u8]) -> Result<Acknowledgement, Obd2Error> {
    let ack = Acknowledgement::parse(data)?;
    if ack.pgn == Pgn::DM11 {
        Ok(ack)
    } else {
        Err(Obd2Error::ParseError(format!(
            "DM11 acknowledgement referenced PGN {}, expected {}",
            ack.pgn.0,
            Pgn::DM11.0
        )))
    }
}

pub fn parse_dm5(data: &[u8]) -> Result<Dm5, Obd2Error> {
    if data.len() < 8 {
        return Err(Obd2Error::ParseError(format!(
            "DM5 requires 8 bytes, got {}",
            data.len()
        )));
    }
    let mut monitor_readiness = [0u8; 5];
    monitor_readiness.copy_from_slice(&data[3..8]);
    Ok(Dm5 {
        active_dtc_count: byte_available(data[0]),
        previously_active_dtc_count: byte_available(data[1]),
        obd_compliance: data[2],
        monitor_readiness,
    })
}

pub fn encode_dm5(message: &Dm5) -> [u8; 8] {
    message.to_payload()
}

pub fn parse_dm24(data: &[u8]) -> Result<Dm24, Obd2Error> {
    let chunks = data.chunks_exact(4);
    if !chunks.remainder().is_empty() {
        return Err(Obd2Error::ParseError(format!(
            "DM24 payload length {} is not divisible by 4",
            data.len()
        )));
    }

    let mut entries = Vec::new();
    for chunk in chunks {
        if chunk == [0xFF, 0xFF, 0xFF, 0xFF] {
            continue;
        }
        let spn_low = u16::from_le_bytes([chunk[0], chunk[1]]) as u32;
        let spn_high = ((chunk[2] >> 5) & 0x07) as u32;
        entries.push(Dm24Entry {
            spn: Spn(spn_low | (spn_high << 16)),
            support_bits: chunk[2] & 0x1F,
            data_length: chunk[3],
        });
    }
    Ok(Dm24 { entries })
}

pub fn encode_dm24(message: &Dm24) -> Result<Vec<u8>, Obd2Error> {
    message.to_payload()
}

pub fn parse_dm25(data: &[u8]) -> Result<Dm25, Obd2Error> {
    let mut frames = Vec::new();
    let mut offset = 0usize;
    while offset < data.len() {
        if data[offset..].iter().all(|byte| *byte == 0xFF) {
            break;
        }
        let length = data[offset] as usize;
        if length == 0 {
            break;
        }
        if length < 4 {
            return Err(Obd2Error::ParseError(format!(
                "DM25 freeze-frame length {length} is too short for a DTC"
            )));
        }
        let end = offset
            .checked_add(1 + length)
            .ok_or_else(|| Obd2Error::ParseError("DM25 length overflow".into()))?;
        if end > data.len() {
            return Err(Obd2Error::ParseError(format!(
                "DM25 freeze-frame length {length} exceeds remaining payload"
            )));
        }
        let dtc = J1939Dtc::from_bytes(&data[offset + 1..offset + 5])
            .ok_or_else(|| Obd2Error::ParseError("DM25 freeze frame missing 4-byte DTC".into()))?;
        frames.push(Dm25FreezeFrame {
            length: length as u8,
            dtc,
            data: data[offset + 5..end].to_vec(),
        });
        offset = end;
    }
    Ok(Dm25 { frames })
}

pub fn encode_dm25(message: &Dm25) -> Result<Vec<u8>, Obd2Error> {
    message.to_payload()
}

fn parse_dm_dtc_message(data: &[u8], label: &str) -> Result<DmDtcMessage, Obd2Error> {
    if data.len() < 2 {
        return Err(Obd2Error::ParseError(format!(
            "{label} requires 2 lamp-status bytes, got {}",
            data.len()
        )));
    }

    let chunks = data[2..].chunks_exact(4);
    if !chunks.remainder().is_empty() && !is_padding_bytes(chunks.remainder()) {
        return Err(Obd2Error::ParseError(format!(
            "{label} DTC payload has {} trailing bytes",
            chunks.remainder().len()
        )));
    }

    let mut dtcs = Vec::new();
    for chunk in chunks {
        if is_padding_bytes(chunk) {
            continue;
        }
        let dtc = J1939Dtc::from_bytes(chunk).ok_or_else(|| {
            Obd2Error::ParseError(format!("{label} contains an invalid DTC chunk"))
        })?;
        dtcs.push(dtc);
    }

    Ok(DmDtcMessage {
        lamps: DmLampStatus::parse(data[0], data[1]),
        dtcs,
    })
}

fn is_padding_bytes(bytes: &[u8]) -> bool {
    bytes.iter().all(|byte| *byte == 0xFF) || bytes.iter().all(|byte| *byte == 0x00)
}

fn expect_pgn(message: J1939Message, expected: Pgn) -> Result<Vec<u8>, Obd2Error> {
    if message.pgn == expected {
        Ok(message.payload)
    } else {
        Err(Obd2Error::ParseError(format!(
            "expected J1939 PGN {}, got {}",
            expected.0, message.pgn.0
        )))
    }
}

// ── PGN Decoders ──

// J1939 "not available" sentinels
const NA_BYTE: u8 = 0xFF;
const NA_WORD: u16 = 0xFFFF;

/// Convert a single-byte J1939 value, returning `None` if the byte is `0xFF` (not available).
fn byte_available(b: u8) -> Option<u8> {
    if b == NA_BYTE {
        None
    } else {
        Some(b)
    }
}

/// Convert a two-byte J1939 value, returning `None` if the word is `0xFFFF` (not available).
fn word_available(w: u16) -> Option<u16> {
    if w == NA_WORD {
        None
    } else {
        Some(w)
    }
}

/// Decode EEC1 (PGN 61444) from 8 raw bytes.
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF).
pub fn decode_eec1(data: &[u8]) -> Option<Eec1> {
    if data.len() < 8 {
        return None;
    }
    // SPN 899: Torque mode (byte 1, bits 0-3)
    let torque_mode = data[0] & 0x0F;

    // SPN 512: Driver's Demand Torque (byte 2) — offset -125, resolution 1%
    let driver_demand_torque_pct = byte_available(data[1]).map(|b| b as f64 - 125.0);

    // SPN 513: Actual Engine Torque (byte 3) — offset -125, resolution 1%
    let actual_torque_pct = byte_available(data[2]).map(|b| b as f64 - 125.0);

    // SPN 190: Engine Speed (bytes 4-5) — resolution 0.125 RPM
    let rpm_raw = u16::from_le_bytes([data[3], data[4]]);
    let engine_rpm = word_available(rpm_raw).map(|w| w as f64 * 0.125);

    Some(Eec1 {
        engine_rpm,
        driver_demand_torque_pct,
        actual_torque_pct,
        torque_mode,
    })
}

/// Decode CCVS (PGN 65265) from 8 raw bytes.
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF).
pub fn decode_ccvs(data: &[u8]) -> Option<Ccvs> {
    if data.len() < 8 {
        return None;
    }
    // SPN 84: Vehicle Speed (bytes 2-3) — resolution 1/256 km/h
    let speed_raw = u16::from_le_bytes([data[1], data[2]]);
    let vehicle_speed = word_available(speed_raw).map(|w| w as f64 / 256.0);

    // SPN 597: Brake Switch (byte 4, bits 2-3) — 0b11 = not available
    let brake_bits = (data[3] >> 2) & 0x03;
    let brake_switch = if brake_bits == 0x03 {
        None
    } else {
        Some(brake_bits == 1)
    };

    // SPN 595: Cruise Control Active (byte 1, bits 0-1) — 0b11 = not available
    let cruise_bits = data[0] & 0x03;
    let cruise_active = if cruise_bits == 0x03 {
        None
    } else {
        Some(cruise_bits == 1)
    };

    Some(Ccvs {
        vehicle_speed,
        brake_switch,
        cruise_active,
    })
}

/// Decode ET1 (PGN 65262) from 8 raw bytes.
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF).
pub fn decode_et1(data: &[u8]) -> Option<Et1> {
    if data.len() < 4 {
        return None;
    }
    // SPN 110: Engine Coolant Temp (byte 1) — offset -40°C
    let coolant_temp = byte_available(data[0]).map(|b| b as f64 - 40.0);

    // SPN 174: Fuel Temp (byte 2) — offset -40°C
    let fuel_temp = byte_available(data[1]).map(|b| b as f64 - 40.0);

    // SPN 175: Engine Oil Temp (bytes 3-4) — resolution 0.03125°C, offset -273°C
    let oil_raw = u16::from_le_bytes([data[2], data[3]]);
    let oil_temp = word_available(oil_raw).map(|w| w as f64 * 0.03125 - 273.0);

    Some(Et1 {
        coolant_temp,
        fuel_temp,
        oil_temp,
    })
}

/// Decode EFLP1 (PGN 65263) from 8 raw bytes.
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF).
pub fn decode_eflp1(data: &[u8]) -> Option<Eflp1> {
    if data.len() < 4 {
        return None;
    }
    // SPN 109: Coolant Pressure (byte 2) — resolution 2 kPa
    let coolant_pressure = byte_available(data[1]).map(|b| b as f64 * 2.0);

    // SPN 100: Engine Oil Pressure (byte 4) — resolution 4 kPa
    let oil_pressure = byte_available(data[3]).map(|b| b as f64 * 4.0);

    Some(Eflp1 {
        oil_pressure,
        coolant_pressure,
    })
}

/// Decode LFE (PGN 65266) from 8 raw bytes.
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF).
pub fn decode_lfe(data: &[u8]) -> Option<Lfe> {
    if data.len() < 4 {
        return None;
    }
    // SPN 183: Engine Fuel Rate (bytes 1-2) — resolution 0.05 L/h
    let rate_raw = u16::from_le_bytes([data[0], data[1]]);
    let fuel_rate = word_available(rate_raw).map(|w| w as f64 * 0.05);

    // SPN 184: Instantaneous Fuel Economy (bytes 3-4) — resolution 1/512 km/L
    let econ_raw = u16::from_le_bytes([data[2], data[3]]);
    let instantaneous_fuel_economy = word_available(econ_raw).map(|w| w as f64 / 512.0);

    Some(Lfe {
        fuel_rate,
        instantaneous_fuel_economy,
    })
}

/// Decode DM1/DM2 active DTCs from a J1939 diagnostic message.
///
/// The first 2 bytes are the lamp status, followed by 4-byte DTC entries.
pub fn decode_dm1(data: &[u8]) -> Vec<J1939Dtc> {
    parse_dm1(data)
        .map(|message| message.dtcs)
        .unwrap_or_default()
}

/// Decode DM2 previously active DTCs from a J1939 diagnostic message.
pub fn decode_dm2(data: &[u8]) -> Vec<J1939Dtc> {
    parse_dm2(data)
        .map(|message| message.dtcs)
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    #[test]
    fn test_pgn_constants() {
        assert_eq!(Pgn::EEC1.0, 61444);
        assert_eq!(Pgn::CCVS.0, 65265);
        assert_eq!(Pgn::ET1.0, 65262);
        assert_eq!(Pgn::DM1.0, 65226);
        assert_eq!(Pgn::DM3.0, 65228);
        assert_eq!(Pgn::DM5.0, 65230);
        assert_eq!(Pgn::DM11.0, 65235);
        assert_eq!(Pgn::DM24.0, 64950);
        assert_eq!(Pgn::DM25.0, 64951);
    }

    #[test]
    fn test_pgn_name() {
        assert!(Pgn::EEC1.name().contains("Electronic Engine"));
        assert!(Pgn::CCVS.name().contains("Vehicle Speed"));
        assert_eq!(Pgn(99999).name(), "Unknown PGN");
    }

    #[test]
    fn test_pgn_display() {
        let s = format!("{}", Pgn::EEC1);
        assert!(s.contains("61444"));
        assert!(s.contains("EEC1"));
    }

    #[test]
    fn test_decode_eec1() {
        // Torque mode 0, demand -125+155=30%, actual -125+155=30%, RPM = 5440*0.125 = 680
        let data = [0x00, 155, 155, 0x40, 0x15, 0xFF, 0xFF, 0xFF];
        let eec1 = decode_eec1(&data).unwrap();
        assert!((eec1.engine_rpm.unwrap() - 680.0).abs() < 0.2);
        assert!((eec1.driver_demand_torque_pct.unwrap() - 30.0).abs() < 0.1);
        assert!((eec1.actual_torque_pct.unwrap() - 30.0).abs() < 0.1);
    }

    #[test]
    fn test_decode_eec1_not_available() {
        // All 0xFF = not available for torque and RPM fields
        let data = [0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        let eec1 = decode_eec1(&data).unwrap();
        assert!(eec1.engine_rpm.is_none());
        assert!(eec1.driver_demand_torque_pct.is_none());
        assert!(eec1.actual_torque_pct.is_none());
    }

    #[test]
    fn test_decode_eec1_too_short() {
        assert!(decode_eec1(&[0x00, 0x01]).is_none());
    }

    #[test]
    fn test_decode_ccvs() {
        // Speed: 0x1A00 / 256 = 26.0 km/h, brake off, cruise off
        let data = [0x00, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00];
        let ccvs = decode_ccvs(&data).unwrap();
        assert!((ccvs.vehicle_speed.unwrap() - 26.0).abs() < 0.1);
        assert_eq!(ccvs.brake_switch, Some(false));
        assert_eq!(ccvs.cruise_active, Some(false));
    }

    #[test]
    fn test_decode_ccvs_not_available() {
        let data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        let ccvs = decode_ccvs(&data).unwrap();
        assert!(ccvs.vehicle_speed.is_none());
        assert!(ccvs.brake_switch.is_none());
        assert!(ccvs.cruise_active.is_none());
    }

    #[test]
    fn test_decode_et1() {
        // Coolant: 90-40 = 50°C, Fuel: 60-40 = 20°C
        let data = [90, 60, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF];
        let et1 = decode_et1(&data).unwrap();
        assert!((et1.coolant_temp.unwrap() - 50.0).abs() < 0.1);
        assert!((et1.fuel_temp.unwrap() - 20.0).abs() < 0.1);
    }

    #[test]
    fn test_decode_et1_not_available() {
        let data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        let et1 = decode_et1(&data).unwrap();
        assert!(et1.coolant_temp.is_none());
        assert!(et1.fuel_temp.is_none());
        assert!(et1.oil_temp.is_none());
    }

    #[test]
    fn test_decode_eflp1() {
        // Coolant pressure: 50*2 = 100 kPa, Oil pressure: 100*4 = 400 kPa
        let data = [0xFF, 50, 0xFF, 100, 0xFF, 0xFF, 0xFF, 0xFF];
        let eflp1 = decode_eflp1(&data).unwrap();
        assert!((eflp1.coolant_pressure.unwrap() - 100.0).abs() < 0.1);
        assert!((eflp1.oil_pressure.unwrap() - 400.0).abs() < 0.1);
    }

    #[test]
    fn test_decode_eflp1_not_available() {
        let data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        let eflp1 = decode_eflp1(&data).unwrap();
        assert!(eflp1.oil_pressure.is_none());
        assert!(eflp1.coolant_pressure.is_none());
    }

    #[test]
    fn test_decode_lfe() {
        // Fuel rate: 100 * 0.05 = 5.0 L/h
        let data = [100, 0x00, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0xFF];
        let lfe = decode_lfe(&data).unwrap();
        assert!((lfe.fuel_rate.unwrap() - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_decode_lfe_not_available() {
        let data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        let lfe = decode_lfe(&data).unwrap();
        assert!(lfe.fuel_rate.is_none());
        assert!(lfe.instantaneous_fuel_economy.is_none());
    }

    #[test]
    fn test_j1939_dtc_from_bytes() {
        // SPN 190 (engine speed), FMI 2 (erratic)
        let data = [0xBE, 0x00, 0x02, 0x01]; // SPN low = 0x00BE = 190, high bits = 0, FMI = 2, OC = 1
        let dtc = J1939Dtc::from_bytes(&data).unwrap();
        assert_eq!(dtc.spn, 190);
        assert_eq!(dtc.fmi, 2);
        assert_eq!(dtc.occurrence_count, 1);
    }

    #[test]
    fn test_j1939_dtc_from_bytes_too_short() {
        assert!(J1939Dtc::from_bytes(&[0x00, 0x01]).is_none());
    }

    #[test]
    fn test_j1939_dtc_display() {
        let dtc = J1939Dtc {
            spn: 190,
            fmi: 2,
            occurrence_count: 1,
            conversion_method: 0,
        };
        let s = format!("{}", dtc);
        assert!(s.contains("SPN 190"));
        assert!(s.contains("FMI 2"));
        assert!(s.contains("Erratic"));
    }

    #[test]
    fn test_j1939_dtc_fmi_descriptions() {
        let dtc = J1939Dtc {
            spn: 0,
            fmi: 0,
            occurrence_count: 0,
            conversion_method: 0,
        };
        assert!(dtc.fmi_description().contains("Above Normal"));
        let dtc = J1939Dtc {
            spn: 0,
            fmi: 11,
            occurrence_count: 0,
            conversion_method: 0,
        };
        assert!(dtc.fmi_description().contains("Root Cause Not Known"));
    }

    #[test]
    fn test_decode_dm1() {
        // 2 bytes lamp status + 1 DTC (4 bytes)
        let data = [0x00, 0x00, 0xBE, 0x00, 0x02, 0x01];
        let dtcs = decode_dm1(&data);
        assert_eq!(dtcs.len(), 1);
        assert_eq!(dtcs[0].spn, 190);
        assert_eq!(dtcs[0].fmi, 2);
    }

    #[test]
    fn test_decode_dm1_empty() {
        assert!(decode_dm1(&[0x00, 0x00]).is_empty());
    }

    #[test]
    fn test_decode_dm1_multiple_dtcs() {
        let data = [
            0x00, 0x00, // lamp status
            0xBE, 0x00, 0x02, 0x01, // SPN 190 FMI 2
            0x64, 0x00, 0x03, 0x02, // SPN 100 FMI 3
        ];
        let dtcs = decode_dm1(&data);
        assert_eq!(dtcs.len(), 2);
        assert_eq!(dtcs[0].spn, 190);
        assert_eq!(dtcs[1].spn, 100);
    }

    #[test]
    fn test_j1939_can_id_round_trips_pdu1_request() {
        let id = J1939CanId::new(
            DEFAULT_PRIORITY,
            Pgn::REQUEST,
            DEFAULT_TOOL_ADDRESS,
            Some(GLOBAL_ADDRESS),
        )
        .unwrap();

        assert_eq!(id.encode(), 0x18EAFFF9);

        let decoded = J1939CanId::decode(0x18EAFFF9).unwrap();
        assert_eq!(decoded.priority, DEFAULT_PRIORITY);
        assert_eq!(decoded.pgn, Pgn::REQUEST);
        assert_eq!(decoded.source, DEFAULT_TOOL_ADDRESS);
        assert_eq!(decoded.destination, Some(GLOBAL_ADDRESS));
    }

    #[test]
    fn test_j1939_can_id_round_trips_pdu2_dm1() {
        let id = J1939CanId::new(DEFAULT_PRIORITY, Pgn::DM1, 0x00, None).unwrap();
        assert_eq!(id.encode(), 0x18FECA00);

        let decoded = J1939CanId::decode(0x18FECA00).unwrap();
        assert_eq!(decoded.pgn, Pgn::DM1);
        assert_eq!(decoded.source, 0x00);
        assert_eq!(decoded.destination, None);
    }

    #[test]
    fn test_j1939_request_frame_encodes_requested_pgn() {
        let frame = request_dm1(DEFAULT_TOOL_ADDRESS, GLOBAL_ADDRESS)
            .to_frame()
            .unwrap();

        assert_eq!(frame.can_identifier(), 0x18EAFFF9);
        assert_eq!(frame.payload(), &[0xCA, 0xFE, 0x00]);
    }

    #[test]
    fn test_address_claim_parses_name() {
        let name = J1939Name(0x8123_4567_89AB_CDEF);
        let frame = J1939Frame::from_parts(
            DEFAULT_PRIORITY,
            Pgn::ADDRESS_CLAIMED,
            0x80,
            Some(GLOBAL_ADDRESS),
            &name.to_payload(),
        )
        .unwrap();

        let claim = AddressClaim::parse(&frame).unwrap();
        assert_eq!(claim.source_address, 0x80);
        assert_eq!(claim.name, name);
        assert!(claim.name.arbitrary_address_capable());
        assert_eq!(claim.name.to_payload(), name.to_payload());
    }

    #[test]
    fn test_tp_bam_control_message_round_trips() {
        let control = TpControlMessage::BroadcastAnnounce {
            message_size: 10,
            packets: 2,
            pgn: Pgn::DM1,
        };

        let payload = control.to_payload();
        assert_eq!(payload, [0x20, 0x0A, 0x00, 0x02, 0xFF, 0xCA, 0xFE, 0x00]);
        assert_eq!(TpControlMessage::parse(&payload).unwrap(), control);
    }

    #[test]
    fn test_tp_reassembler_builds_complete_message() {
        let mut reassembler = TpReassembler::start(
            0x80,
            None,
            TpControlMessage::BroadcastAnnounce {
                message_size: 10,
                packets: 2,
                pgn: Pgn::DM1,
            },
        )
        .unwrap();

        let first = TpDataTransfer {
            sequence: 1,
            data: [1, 2, 3, 4, 5, 6, 7],
        };
        assert!(reassembler.accept_dt(first).unwrap().is_none());

        let second = TpDataTransfer {
            sequence: 2,
            data: [8, 9, 10, 0xFF, 0xFF, 0xFF, 0xFF],
        };
        let message = reassembler.accept_dt(second).unwrap().unwrap();
        assert_eq!(message.source, 0x80);
        assert_eq!(message.pgn, Pgn::DM1);
        assert_eq!(message.payload, vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
    }

    #[test]
    fn test_parse_dm1_decodes_lamps_and_skips_padding() {
        let data = [
            0x40, 0x00, // MIL on, no flash
            0xBE, 0x00, 0x02, 0x01, // SPN 190 FMI 2
            0xFF, 0xFF, 0xFF, 0xFF, // padding
        ];

        let message = parse_dm1(&data).unwrap();
        assert_eq!(message.lamps.malfunction_indicator, LampStatus::On);
        assert_eq!(
            message.lamps.malfunction_indicator_flash,
            LampFlash::SlowFlash
        );
        assert_eq!(message.dtcs.len(), 1);
        assert_eq!(message.dtcs[0].spn, 190);
    }

    #[test]
    fn test_parse_dm1_accepts_zero_padding_without_losing_dtcs() {
        let data = [
            0x40, 0x00, // MIL on, no flash
            0xBE, 0x00, 0x02, 0x01, // SPN 190 FMI 2
            0x00, 0x00, // trailing frame padding from strict-DLC stacks
        ];

        let message = parse_dm1(&data).unwrap();

        assert_eq!(message.dtcs.len(), 1);
        assert_eq!(message.dtcs[0].spn, 190);
        assert_eq!(decode_dm1(&data).len(), 1);
    }

    #[test]
    fn test_parse_dm1_rejects_mixed_trailing_bytes() {
        let data = [
            0x40, 0x00, // MIL on, no flash
            0xBE, 0x00, 0x02, 0x01, // SPN 190 FMI 2
            0x00, 0x7E,
        ];

        assert!(matches!(parse_dm1(&data), Err(Obd2Error::ParseError(_))));
    }

    #[test]
    fn test_parse_dm5_readiness() {
        let dm5 = parse_dm5(&[2, 3, 0x13, 0xAA, 0x55, 0x00, 0x12, 0x34]).unwrap();
        assert_eq!(dm5.active_dtc_count, Some(2));
        assert_eq!(dm5.previously_active_dtc_count, Some(3));
        assert_eq!(dm5.obd_compliance, 0x13);
        assert_eq!(dm5.monitor_readiness, [0xAA, 0x55, 0x00, 0x12, 0x34]);
    }

    #[test]
    fn test_parse_dm24_supported_spns() {
        let dm24 = parse_dm24(&[
            0xBE, 0x00, 0x02, 0x02, // SPN 190, support bits 2, length 2
            0xFF, 0xFF, 0xFF, 0xFF, // padding
        ])
        .unwrap();

        assert_eq!(dm24.entries.len(), 1);
        assert_eq!(dm24.entries[0].spn, Spn(190));
        assert_eq!(dm24.entries[0].support_bits, 0x02);
        assert_eq!(dm24.entries[0].data_length, 2);
    }

    #[test]
    fn test_parse_dm25_freeze_frame() {
        let dm25 = parse_dm25(&[
            6, // DTC + two bytes of freeze-frame data
            0xBE, 0x00, 0x02, 0x01, // SPN 190 FMI 2
            0xAA, 0x55,
        ])
        .unwrap();

        assert_eq!(dm25.frames.len(), 1);
        assert_eq!(dm25.frames[0].length, 6);
        assert_eq!(dm25.frames[0].dtc.spn, 190);
        assert_eq!(dm25.frames[0].data, vec![0xAA, 0x55]);
    }

    #[test]
    fn test_parse_dm11_acknowledgement() {
        let ack = Acknowledgement {
            control: AcknowledgementControl::Ack,
            group_function_value: 0xFF,
            address_acknowledged: DEFAULT_TOOL_ADDRESS,
            pgn: Pgn::DM11,
        };

        let payload = ack.to_payload();
        assert_eq!(payload[2], 0xFF);
        assert_eq!(payload[3], 0xFF);
        assert_eq!(payload[4], DEFAULT_TOOL_ADDRESS);

        let parsed = parse_dm11_response(&payload).unwrap();
        assert_eq!(parsed, ack);
    }

    #[test]
    fn test_acknowledgement_parses_address_from_byte_five() {
        let payload = [0x00, 0xFF, 0x11, 0x22, 0x80, 0xD3, 0xFE, 0x00];

        let parsed = Acknowledgement::parse(&payload).unwrap();

        assert_eq!(parsed.address_acknowledged, 0x80);
        assert_eq!(parsed.pgn, Pgn::DM11);
    }

    #[derive(Debug)]
    struct MockJ1939Transport {
        requests: Vec<J1939Request>,
        responses: VecDeque<J1939Message>,
    }

    #[async_trait::async_trait]
    impl J1939Transport for MockJ1939Transport {
        async fn request_pgn(&mut self, request: &J1939Request) -> Result<J1939Message, Obd2Error> {
            self.requests.push(*request);
            self.responses.pop_front().ok_or_else(|| Obd2Error::NoData)
        }
    }

    #[tokio::test]
    async fn test_j1939_client_requests_dm5() {
        let transport = MockJ1939Transport {
            requests: Vec::new(),
            responses: VecDeque::from([J1939Message {
                source: 0x00,
                destination: Some(DEFAULT_TOOL_ADDRESS),
                pgn: Pgn::DM5,
                payload: vec![1, 0, 0x13, 0, 0, 0, 0, 0],
            }]),
        };
        let mut client = J1939Client::new(transport, DEFAULT_TOOL_ADDRESS);

        let dm5 = client.request_dm5().await.unwrap();
        assert_eq!(dm5.active_dtc_count, Some(1));

        let transport = client.into_inner();
        assert_eq!(transport.requests.len(), 1);
        assert_eq!(transport.requests[0].requested_pgn, Pgn::DM5);
        assert_eq!(transport.requests[0].destination, GLOBAL_ADDRESS);
    }
}
