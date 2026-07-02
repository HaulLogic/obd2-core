//! ELM327-backed implementation of the neutral framed transport.

use crate::adapter::{elm327::Elm327Adapter, Adapter};
use crate::error::Obd2Error;
use crate::protocol::codec::BusFamily;
use crate::protocol::service::{ServiceRequest, Target};
use crate::transport::framed::{Transport, TransportRequest};

#[derive(Debug)]
pub struct ElmTransport {
    adapter: Elm327Adapter,
}

impl ElmTransport {
    pub fn new(adapter: Elm327Adapter) -> Self {
        Self { adapter }
    }
}

#[async_trait::async_trait]
impl Transport for ElmTransport {
    async fn exchange(&mut self, req: &TransportRequest) -> Result<Vec<u8>, Obd2Error> {
        self.adapter
            .request(&ServiceRequest {
                service_id: req.service_id,
                data: req.data.clone(),
                target: Target::Broadcast,
            })
            .await
    }

    fn family(&self) -> BusFamily {
        self.adapter.protocol_family()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::client::{J1979Client, ProtocolClient, RequestKind};
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
        transport.expect("ATDPN", "A6\r>");
        transport.expect("ATCAF1", "OK\r>");
        transport.expect("ATCFC1", "OK\r>");
    }

    #[tokio::test]
    async fn j1979_client_reads_pid_over_elm_backed_transport() {
        let mut mock = MockTransport::new();
        setup_init(&mut mock);
        mock.expect("0105", "41 05 7B\r\r>");

        let mut adapter = Elm327Adapter::new(Box::new(mock));
        adapter.initialize().await.unwrap();

        let mut client = J1979Client::new(ElmTransport::new(adapter));
        let resp = client.request(RequestKind::Mode01Pid(0x05)).await.unwrap();

        assert_eq!(resp.expected_positive_service, 0x41);
        assert_eq!(resp.payload, vec![0x7B]);
    }
}
