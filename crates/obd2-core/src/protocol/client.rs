//! Protocol-level diagnostic clients over framed transports.

use crate::error::Obd2Error;
use crate::transport::framed::{Transport, TransportRequest};

#[async_trait::async_trait]
pub trait ProtocolClient: Send {
    fn name(&self) -> &'static str;

    async fn request(&mut self, kind: RequestKind) -> Result<DiagResponse, Obd2Error>;
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RequestKind {
    Mode01Pid(u8),
    Did16 { service: u8, did: u16 },
    Raw { service: u8, data: Vec<u8> },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DiagResponse {
    pub expected_positive_service: u8,
    pub payload: Vec<u8>,
}

#[derive(Debug)]
pub struct J1979Client<T: Transport> {
    transport: T,
}

impl<T: Transport> J1979Client<T> {
    pub fn new(transport: T) -> Self {
        Self { transport }
    }
}

#[async_trait::async_trait]
impl<T: Transport> ProtocolClient for J1979Client<T> {
    fn name(&self) -> &'static str {
        "J1979"
    }

    async fn request(&mut self, kind: RequestKind) -> Result<DiagResponse, Obd2Error> {
        let (service_id, data) = match kind {
            RequestKind::Mode01Pid(pid) => (0x01, vec![pid]),
            RequestKind::Did16 { service, did } => (service, did.to_be_bytes().to_vec()),
            RequestKind::Raw { service, data } => (service, data),
        };
        let expected_positive_service = service_id.checked_add(0x40).ok_or_else(|| {
            Obd2Error::ParseError(format!(
                "service id 0x{service_id:02X} cannot form a positive-response id"
            ))
        })?;
        let payload = self
            .transport
            .exchange(&TransportRequest { service_id, data })
            .await?;
        Ok(DiagResponse {
            expected_positive_service,
            payload,
        })
    }
}
