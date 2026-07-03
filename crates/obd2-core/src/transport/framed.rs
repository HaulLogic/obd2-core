//! Transport-neutral diagnostic request/response exchange.
//!
//! This layer deals in already-framed diagnostic PDUs. Adapter-specific
//! command text, byte links, and physical routing stay outside this module.

use crate::error::Obd2Error;
use crate::protocol::codec::BusFamily;

/// Logical target for one framed diagnostic exchange.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportTarget {
    Broadcast,
    Physical(u32),
    Functional(u32),
    Pgn(u32),
}

/// Framed diagnostic request payload.
///
/// `pdu` is already protocol-shaped. Service-byte protocols can use
/// `diagnostic`; PGN protocols can carry their raw message bytes directly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TransportRequest {
    pub target: TransportTarget,
    pub pdu: Vec<u8>,
}

impl TransportRequest {
    pub fn diagnostic(target: TransportTarget, service_id: u8, data: impl AsRef<[u8]>) -> Self {
        let data = data.as_ref();
        let mut pdu = Vec::with_capacity(1 + data.len());
        pdu.push(service_id);
        pdu.extend_from_slice(data);
        Self { target, pdu }
    }

    pub fn broadcast_diagnostic(service_id: u8, data: impl AsRef<[u8]>) -> Self {
        Self::diagnostic(TransportTarget::Broadcast, service_id, data)
    }

    pub fn service_id(&self) -> Option<u8> {
        self.pdu.first().copied()
    }

    pub fn service_data(&self) -> &[u8] {
        self.pdu.get(1..).unwrap_or_default()
    }
}

/// Framed diagnostic transport.
#[async_trait::async_trait]
pub trait Transport: Send {
    /// Exchange one diagnostic request and return the response payload bytes.
    async fn exchange(&mut self, req: TransportRequest) -> Result<Vec<u8>, Obd2Error>;

    /// Active bus family for response decoding or higher-layer decisions.
    fn family(&self) -> BusFamily;
}
