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
        let request = match kind {
            RequestKind::Mode01Pid(pid) => TransportRequest::broadcast_diagnostic(0x01, [pid]),
            RequestKind::Did16 { service, did } => {
                TransportRequest::broadcast_diagnostic(service, did.to_be_bytes())
            }
            RequestKind::Raw { service, mut data } => {
                data.insert(0, service);
                TransportRequest {
                    target: crate::transport::framed::TransportTarget::Broadcast,
                    pdu: data,
                }
            }
        };
        let payload = self.transport.exchange(request).await?;
        Ok(DiagResponse { payload })
    }
}
