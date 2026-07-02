//! Transport-neutral diagnostic request/response exchange.
//!
//! This layer deals in already-framed diagnostic PDUs. Adapter-specific
//! command text, byte links, and physical routing stay outside this module.

use crate::error::Obd2Error;
use crate::protocol::codec::BusFamily;

/// Broadcast diagnostic request payload for the framed transport seam.
///
/// P0 is broadcast-only. Physical addressing and routing are handled by the
/// existing adapter/session path until the protocol client is wired in later.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TransportRequest {
    pub service_id: u8,
    pub data: Vec<u8>,
}

/// Framed diagnostic transport.
#[async_trait::async_trait]
pub trait Transport: Send {
    /// Exchange one diagnostic request and return the response payload bytes.
    async fn exchange(&mut self, req: &TransportRequest) -> Result<Vec<u8>, Obd2Error>;

    /// Active bus family for response decoding or higher-layer decisions.
    fn family(&self) -> BusFamily;
}
