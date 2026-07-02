//! Host-side ISO-TP transport over raw CAN data frames.
//!
//! This module owns ISO 15765-2 segmentation, reassembly, and flow control.
//! It does not issue adapter-specific text commands and is not wired into the
//! existing ELM/J1979 session path.

use std::time::Duration;

use crate::error::{NegativeResponse, Obd2Error};
use crate::protocol::codec::BusFamily;
use crate::transport::framed::{Transport, TransportRequest};

const PCI_TYPE_SINGLE_FRAME: u8 = 0x0;
const PCI_TYPE_FIRST_FRAME: u8 = 0x1;
const PCI_TYPE_CONSECUTIVE_FRAME: u8 = 0x2;
const PCI_TYPE_FLOW_CONTROL: u8 = 0x3;
const MAX_CLASSIC_ISOTP_PAYLOAD: usize = 4095;

/// Raw classic CAN data frame used by host-side ISO-TP.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanDataFrame {
    pub id: u32,
    pub data: Vec<u8>,
    pub is_extended_id: bool,
}

impl CanDataFrame {
    pub fn new(id: u32, data: impl Into<Vec<u8>>) -> Result<Self, Obd2Error> {
        Self::with_extended_id(id, id > 0x7FF, data)
    }

    pub fn with_extended_id(
        id: u32,
        is_extended_id: bool,
        data: impl Into<Vec<u8>>,
    ) -> Result<Self, Obd2Error> {
        if id > 0x1FFF_FFFF {
            return Err(Obd2Error::ParseError(format!(
                "CAN identifier exceeds 29 bits: 0x{id:X}"
            )));
        }

        let data = data.into();
        if data.len() > 8 {
            return Err(Obd2Error::ParseError(format!(
                "classic CAN frame has {} data bytes; max is 8",
                data.len()
            )));
        }

        Ok(Self {
            id,
            data,
            is_extended_id,
        })
    }
}

/// Minimal async raw CAN I/O required by [`IsoTpTransport`].
#[async_trait::async_trait]
pub trait CanFrameIo: Send {
    async fn send_frame(&mut self, frame: CanDataFrame) -> Result<(), Obd2Error>;
    async fn recv_frame(&mut self) -> Result<CanDataFrame, Obd2Error>;
}

/// ISO-TP connection configuration for one request/response ID pair.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IsoTpConfig {
    pub request_id: u32,
    pub response_id: u32,
    pub block_size: u8,
    pub st_min: u8,
    pub tx_padding: Option<u8>,
    pub max_payload_len: usize,
    pub max_unrelated_frames: usize,
    pub max_flow_control_wait_frames: usize,
}

impl IsoTpConfig {
    pub fn new(request_id: u32, response_id: u32) -> Self {
        Self {
            request_id,
            response_id,
            block_size: 8,
            st_min: 0,
            tx_padding: Some(0x00),
            max_payload_len: MAX_CLASSIC_ISOTP_PAYLOAD,
            max_unrelated_frames: 32,
            max_flow_control_wait_frames: 16,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlowStatus {
    ContinueToSend,
    Wait,
    Overflow,
}

impl FlowStatus {
    fn from_nibble(nibble: u8) -> Result<Self, Obd2Error> {
        match nibble {
            0x0 => Ok(Self::ContinueToSend),
            0x1 => Ok(Self::Wait),
            0x2 => Ok(Self::Overflow),
            _ => Err(Obd2Error::ParseError(format!(
                "unsupported ISO-TP flow status: 0x{nibble:X}"
            ))),
        }
    }

    fn nibble(self) -> u8 {
        match self {
            Self::ContinueToSend => 0x0,
            Self::Wait => 0x1,
            Self::Overflow => 0x2,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FlowControlFrame {
    pub status: FlowStatus,
    pub block_size: u8,
    pub st_min: u8,
}

/// Decode an ISO-TP STmin byte into a concrete separation delay.
pub fn st_min_duration(st_min: u8) -> Result<Duration, Obd2Error> {
    match st_min {
        0x00..=0x7F => Ok(Duration::from_millis(st_min as u64)),
        0xF1..=0xF9 => Ok(Duration::from_micros((st_min as u64 - 0xF0) * 100)),
        _ => Err(Obd2Error::ParseError(format!(
            "reserved ISO-TP STmin byte: 0x{st_min:02X}"
        ))),
    }
}

#[derive(Debug)]
pub struct IsoTpTransport<T: CanFrameIo> {
    io: T,
    config: IsoTpConfig,
}

impl<T: CanFrameIo> IsoTpTransport<T> {
    pub fn new(io: T, config: IsoTpConfig) -> Self {
        Self { io, config }
    }

    pub fn config(&self) -> &IsoTpConfig {
        &self.config
    }

    pub fn io(&self) -> &T {
        &self.io
    }

    pub fn io_mut(&mut self) -> &mut T {
        &mut self.io
    }

    pub fn into_inner(self) -> T {
        self.io
    }

    /// Exchange a raw diagnostic PDU. The returned bytes include the response SID.
    pub async fn exchange_pdu(&mut self, request_pdu: &[u8]) -> Result<Vec<u8>, Obd2Error> {
        if request_pdu.is_empty() {
            return Err(Obd2Error::ParseError("ISO-TP request PDU is empty".into()));
        }
        self.validate_pdu_len(request_pdu.len())?;

        self.send_pdu(request_pdu).await?;
        self.recv_pdu().await
    }

    async fn send_pdu(&mut self, pdu: &[u8]) -> Result<(), Obd2Error> {
        if pdu.len() <= 7 {
            let mut data = Vec::with_capacity(8);
            data.push(pdu.len() as u8);
            data.extend_from_slice(pdu);
            self.send_frame_data(data).await
        } else {
            self.send_multi_frame_pdu(pdu).await
        }
    }

    async fn send_multi_frame_pdu(&mut self, pdu: &[u8]) -> Result<(), Obd2Error> {
        if pdu.len() > MAX_CLASSIC_ISOTP_PAYLOAD {
            return Err(Obd2Error::ParseError(format!(
                "ISO-TP PDU length {} exceeds classic CAN limit {}",
                pdu.len(),
                MAX_CLASSIC_ISOTP_PAYLOAD
            )));
        }

        let mut first = Vec::with_capacity(8);
        first.push(0x10 | (((pdu.len() >> 8) as u8) & 0x0F));
        first.push((pdu.len() & 0xFF) as u8);
        first.extend_from_slice(&pdu[..6]);
        self.send_frame_data(first).await?;

        let mut fc = self.wait_for_continue_to_send().await?;
        let mut offset = 6;
        let mut sequence = 1u8;
        let mut sent_in_block = 0u8;

        while offset < pdu.len() {
            let delay = st_min_duration(fc.st_min)?;
            if !delay.is_zero() {
                tokio::time::sleep(delay).await;
            }

            let chunk_len = (pdu.len() - offset).min(7);
            let mut data = Vec::with_capacity(8);
            data.push(0x20 | (sequence & 0x0F));
            data.extend_from_slice(&pdu[offset..offset + chunk_len]);
            self.send_frame_data(data).await?;

            offset += chunk_len;
            sequence = (sequence + 1) & 0x0F;
            sent_in_block = sent_in_block.saturating_add(1);

            if fc.block_size != 0 && sent_in_block >= fc.block_size && offset < pdu.len() {
                fc = self.wait_for_continue_to_send().await?;
                sent_in_block = 0;
            }
        }

        Ok(())
    }

    async fn recv_pdu(&mut self) -> Result<Vec<u8>, Obd2Error> {
        let frame = self.read_matching_frame(self.config.response_id).await?;
        let pci = first_pci_byte(&frame)?;

        match pci >> 4 {
            PCI_TYPE_SINGLE_FRAME => self.decode_single_frame(&frame),
            PCI_TYPE_FIRST_FRAME => self.recv_multi_frame_pdu(&frame).await,
            frame_type => Err(Obd2Error::ParseError(format!(
                "unexpected ISO-TP frame type while receiving PDU: 0x{frame_type:X}"
            ))),
        }
    }

    async fn recv_multi_frame_pdu(
        &mut self,
        first_frame: &CanDataFrame,
    ) -> Result<Vec<u8>, Obd2Error> {
        if first_frame.data.len() < 2 {
            return Err(Obd2Error::ParseError(
                "ISO-TP first frame missing length byte".into(),
            ));
        }

        let declared_len =
            (((first_frame.data[0] & 0x0F) as usize) << 8) | first_frame.data[1] as usize;
        if declared_len <= 7 {
            return Err(Obd2Error::ParseError(format!(
                "ISO-TP first frame declared single-frame length {declared_len}"
            )));
        }
        self.validate_pdu_len(declared_len)?;

        let mut pdu = Vec::with_capacity(declared_len);
        let initial_len = (declared_len).min(first_frame.data.len().saturating_sub(2));
        pdu.extend_from_slice(&first_frame.data[2..2 + initial_len]);

        self.send_flow_control(FlowStatus::ContinueToSend).await?;

        let mut expected_sequence = 1u8;
        let mut received_in_block = 0u8;
        while pdu.len() < declared_len {
            let frame = self.read_matching_frame(self.config.response_id).await?;
            let pci = first_pci_byte(&frame)?;
            let frame_type = pci >> 4;
            if frame_type != PCI_TYPE_CONSECUTIVE_FRAME {
                return Err(Obd2Error::ParseError(format!(
                    "expected ISO-TP consecutive frame, got type 0x{frame_type:X}"
                )));
            }

            let sequence = pci & 0x0F;
            if sequence != expected_sequence {
                return Err(Obd2Error::ParseError(format!(
                    "ISO-TP sequence mismatch: expected {expected_sequence}, got {sequence}"
                )));
            }

            let remaining = declared_len - pdu.len();
            let available = frame.data.len().saturating_sub(1);
            if available == 0 {
                return Err(Obd2Error::ParseError(
                    "ISO-TP consecutive frame carried no payload".into(),
                ));
            }
            let take = remaining.min(available);
            pdu.extend_from_slice(&frame.data[1..1 + take]);

            expected_sequence = (expected_sequence + 1) & 0x0F;
            received_in_block = received_in_block.saturating_add(1);

            if self.config.block_size != 0
                && received_in_block >= self.config.block_size
                && pdu.len() < declared_len
            {
                self.send_flow_control(FlowStatus::ContinueToSend).await?;
                received_in_block = 0;
            }
        }

        Ok(pdu)
    }

    fn decode_single_frame(&self, frame: &CanDataFrame) -> Result<Vec<u8>, Obd2Error> {
        let len = (first_pci_byte(frame)? & 0x0F) as usize;
        if frame.data.len() < len + 1 {
            return Err(Obd2Error::ParseError(format!(
                "ISO-TP single frame declares {len} bytes but frame has {} payload bytes",
                frame.data.len().saturating_sub(1)
            )));
        }
        self.validate_pdu_len(len)?;
        Ok(frame.data[1..1 + len].to_vec())
    }

    async fn wait_for_continue_to_send(&mut self) -> Result<FlowControlFrame, Obd2Error> {
        let mut wait_frames = 0usize;

        loop {
            let fc = self.read_flow_control().await?;
            match fc.status {
                FlowStatus::ContinueToSend => return Ok(fc),
                FlowStatus::Wait => {
                    wait_frames += 1;
                    if wait_frames > self.config.max_flow_control_wait_frames {
                        return Err(Obd2Error::Timeout);
                    }
                    tokio::time::sleep(Duration::from_millis(1)).await;
                }
                FlowStatus::Overflow => {
                    return Err(Obd2Error::ParseError(
                        "receiver reported ISO-TP flow-control overflow".into(),
                    ));
                }
            }
        }
    }

    async fn read_flow_control(&mut self) -> Result<FlowControlFrame, Obd2Error> {
        let frame = self.read_matching_frame(self.config.response_id).await?;
        if frame.data.len() < 3 {
            return Err(Obd2Error::ParseError(
                "ISO-TP flow-control frame shorter than 3 bytes".into(),
            ));
        }

        let pci = frame.data[0];
        if pci >> 4 != PCI_TYPE_FLOW_CONTROL {
            return Err(Obd2Error::ParseError(format!(
                "expected ISO-TP flow-control frame, got type 0x{:X}",
                pci >> 4
            )));
        }

        Ok(FlowControlFrame {
            status: FlowStatus::from_nibble(pci & 0x0F)?,
            block_size: frame.data[1],
            st_min: frame.data[2],
        })
    }

    async fn read_matching_frame(&mut self, id: u32) -> Result<CanDataFrame, Obd2Error> {
        let mut unrelated = 0usize;

        loop {
            let frame = self.io.recv_frame().await?;
            validate_can_frame(&frame)?;
            if frame.id == id {
                return Ok(frame);
            }

            unrelated += 1;
            if unrelated > self.config.max_unrelated_frames {
                return Err(Obd2Error::ParseError(format!(
                    "too many unrelated CAN frames while waiting for 0x{id:X}"
                )));
            }
        }
    }

    async fn send_flow_control(&mut self, status: FlowStatus) -> Result<(), Obd2Error> {
        let data = vec![
            0x30 | status.nibble(),
            self.config.block_size,
            self.config.st_min,
        ];
        self.send_frame_data(data).await
    }

    async fn send_frame_data(&mut self, mut data: Vec<u8>) -> Result<(), Obd2Error> {
        if data.len() > 8 {
            return Err(Obd2Error::ParseError(format!(
                "ISO-TP CAN payload has {} bytes; max is 8",
                data.len()
            )));
        }
        if let Some(pad) = self.config.tx_padding {
            while data.len() < 8 {
                data.push(pad);
            }
        }
        let frame = CanDataFrame::new(self.config.request_id, data)?;
        self.io.send_frame(frame).await
    }

    fn validate_pdu_len(&self, len: usize) -> Result<(), Obd2Error> {
        if len > self.config.max_payload_len {
            return Err(Obd2Error::ParseError(format!(
                "ISO-TP PDU length {len} exceeds configured max {}",
                self.config.max_payload_len
            )));
        }
        Ok(())
    }
}

#[async_trait::async_trait]
impl<T: CanFrameIo> Transport for IsoTpTransport<T> {
    async fn exchange(&mut self, req: &TransportRequest) -> Result<Vec<u8>, Obd2Error> {
        let mut request_pdu = Vec::with_capacity(1 + req.data.len());
        request_pdu.push(req.service_id);
        request_pdu.extend_from_slice(&req.data);

        let response_pdu = self.exchange_pdu(&request_pdu).await?;
        strip_response_for_request(req.service_id, &response_pdu)
    }

    fn family(&self) -> BusFamily {
        BusFamily::Can
    }
}

fn first_pci_byte(frame: &CanDataFrame) -> Result<u8, Obd2Error> {
    frame
        .data
        .first()
        .copied()
        .ok_or_else(|| Obd2Error::ParseError("ISO-TP CAN frame missing PCI byte".into()))
}

fn validate_can_frame(frame: &CanDataFrame) -> Result<(), Obd2Error> {
    if frame.data.len() > 8 {
        return Err(Obd2Error::ParseError(format!(
            "classic CAN frame has {} data bytes; max is 8",
            frame.data.len()
        )));
    }
    if frame.data.is_empty() {
        return Err(Obd2Error::ParseError(
            "ISO-TP CAN frame missing PCI byte".into(),
        ));
    }
    Ok(())
}

fn strip_response_for_request(service_id: u8, response_pdu: &[u8]) -> Result<Vec<u8>, Obd2Error> {
    let first = response_pdu
        .first()
        .copied()
        .ok_or_else(|| Obd2Error::ParseError("ISO-TP response PDU is empty".into()))?;

    if first == 0x7F {
        if response_pdu.len() < 3 {
            return Err(Obd2Error::ParseError(
                "UDS negative response shorter than 3 bytes".into(),
            ));
        }
        return Err(Obd2Error::NegativeResponse {
            service: response_pdu[1],
            nrc: NegativeResponse::from_byte_or_unknown(response_pdu[2]),
        });
    }

    let expected = service_id.checked_add(0x40).ok_or_else(|| {
        Obd2Error::ParseError(format!(
            "service id 0x{service_id:02X} cannot form a positive-response id"
        ))
    })?;
    if first != expected {
        return Err(Obd2Error::ParseError(format!(
            "unexpected response service 0x{first:02X}; expected 0x{expected:02X}"
        )));
    }

    Ok(response_pdu[1..].to_vec())
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;

    use super::*;

    #[derive(Debug, Default)]
    struct MockCanIo {
        rx: VecDeque<CanDataFrame>,
        tx: Vec<CanDataFrame>,
    }

    impl MockCanIo {
        fn with_rx(frames: Vec<CanDataFrame>) -> Self {
            Self {
                rx: frames.into(),
                tx: Vec::new(),
            }
        }
    }

    #[async_trait::async_trait]
    impl CanFrameIo for MockCanIo {
        async fn send_frame(&mut self, frame: CanDataFrame) -> Result<(), Obd2Error> {
            self.tx.push(frame);
            Ok(())
        }

        async fn recv_frame(&mut self) -> Result<CanDataFrame, Obd2Error> {
            self.rx.pop_front().ok_or(Obd2Error::Timeout)
        }
    }

    fn frame(id: u32, data: &[u8]) -> CanDataFrame {
        CanDataFrame::new(id, data.to_vec()).unwrap()
    }

    #[tokio::test]
    async fn single_frame_exchange_strips_positive_service() {
        let io = MockCanIo::with_rx(vec![frame(
            0x7E8,
            &[0x05, 0x62, 0xF1, 0x90, 0x12, 0x34, 0x00, 0x00],
        )]);
        let config = IsoTpConfig::new(0x7E0, 0x7E8);
        let mut transport = IsoTpTransport::new(io, config);

        let response = transport
            .exchange(&TransportRequest {
                service_id: 0x22,
                data: vec![0xF1, 0x90],
            })
            .await
            .unwrap();

        assert_eq!(response, vec![0xF1, 0x90, 0x12, 0x34]);
        assert_eq!(
            transport.io().tx,
            vec![frame(0x7E0, &[0x03, 0x22, 0xF1, 0x90, 0, 0, 0, 0])]
        );
    }

    #[tokio::test]
    async fn multi_frame_response_sends_flow_control_and_reassembles() {
        let io = MockCanIo::with_rx(vec![
            frame(0x7E8, &[0x10, 0x0A, 0x62, 0xF1, 0x90, 1, 2, 3]),
            frame(0x7E8, &[0x21, 4, 5, 6, 7, 0, 0, 0]),
        ]);
        let config = IsoTpConfig::new(0x7E0, 0x7E8);
        let mut transport = IsoTpTransport::new(io, config);

        let response = transport.exchange_pdu(&[0x22, 0xF1, 0x90]).await.unwrap();

        assert_eq!(response, vec![0x62, 0xF1, 0x90, 1, 2, 3, 4, 5, 6, 7]);
        assert_eq!(
            transport.io().tx,
            vec![
                frame(0x7E0, &[0x03, 0x22, 0xF1, 0x90, 0, 0, 0, 0]),
                frame(0x7E0, &[0x30, 0x08, 0x00, 0, 0, 0, 0, 0]),
            ]
        );
    }

    #[tokio::test]
    async fn multi_frame_request_waits_for_flow_control_before_consecutive_frame() {
        let io = MockCanIo::with_rx(vec![
            frame(0x7E8, &[0x30, 0x00, 0x00, 0, 0, 0, 0, 0]),
            frame(0x7E8, &[0x01, 0x6E, 0, 0, 0, 0, 0, 0]),
        ]);
        let config = IsoTpConfig::new(0x7E0, 0x7E8);
        let mut transport = IsoTpTransport::new(io, config);

        let response = transport
            .exchange_pdu(&[0x2E, 0xF1, 0x90, 1, 2, 3, 4, 5, 6, 7, 8])
            .await
            .unwrap();

        assert_eq!(response, vec![0x6E]);
        assert_eq!(
            transport.io().tx,
            vec![
                frame(0x7E0, &[0x10, 0x0B, 0x2E, 0xF1, 0x90, 1, 2, 3]),
                frame(0x7E0, &[0x21, 4, 5, 6, 7, 8, 0, 0]),
            ]
        );
    }

    #[tokio::test]
    async fn transport_exchange_maps_negative_response() {
        let io = MockCanIo::with_rx(vec![frame(0x7E8, &[0x03, 0x7F, 0x22, 0x31, 0, 0, 0, 0])]);
        let config = IsoTpConfig::new(0x7E0, 0x7E8);
        let mut transport = IsoTpTransport::new(io, config);

        let err = transport
            .exchange(&TransportRequest {
                service_id: 0x22,
                data: vec![0xF1, 0x90],
            })
            .await
            .unwrap_err();

        assert!(matches!(
            err,
            Obd2Error::NegativeResponse {
                service: 0x22,
                nrc: NegativeResponse::RequestOutOfRange
            }
        ));
    }

    #[test]
    fn st_min_decodes_milliseconds_and_microseconds() {
        assert_eq!(st_min_duration(0x05).unwrap(), Duration::from_millis(5));
        assert_eq!(st_min_duration(0xF3).unwrap(), Duration::from_micros(300));
        assert!(matches!(
            st_min_duration(0x80),
            Err(Obd2Error::ParseError(_))
        ));
    }
}
