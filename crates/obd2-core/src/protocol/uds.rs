//! UDS request/response helpers over framed diagnostic transports.

use crate::error::{NegativeResponse, Obd2Error};
use crate::protocol::client::{DiagResponse, ProtocolClient, RequestKind};
use crate::protocol::dtc::{Dtc, DtcStatusByte};
use crate::transport::framed::{Transport, TransportRequest};

pub const SID_DIAGNOSTIC_SESSION_CONTROL: u8 = 0x10;
pub const SID_ECU_RESET: u8 = 0x11;
pub const SID_CLEAR_DIAGNOSTIC_INFORMATION: u8 = 0x14;
pub const SID_READ_DTC_INFORMATION: u8 = 0x19;
pub const SID_READ_DATA_BY_IDENTIFIER: u8 = 0x22;
pub const SID_SECURITY_ACCESS: u8 = 0x27;
pub const SID_AUTHENTICATION: u8 = 0x29;
pub const SID_WRITE_DATA_BY_IDENTIFIER: u8 = 0x2E;
pub const SID_INPUT_OUTPUT_CONTROL_BY_IDENTIFIER: u8 = 0x2F;
pub const SID_ROUTINE_CONTROL: u8 = 0x31;
pub const SID_TESTER_PRESENT: u8 = 0x3E;
pub const SID_CONTROL_DTC_SETTING: u8 = 0x85;

pub const DID_OBDONUDS: u16 = 0xF810;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UdsPositiveResponse<'a> {
    pub service_id: u8,
    pub data: &'a [u8],
}

pub fn positive_response_sid(service_id: u8) -> Result<u8, Obd2Error> {
    service_id.checked_add(0x40).ok_or_else(|| {
        Obd2Error::ParseError(format!(
            "service id 0x{service_id:02X} cannot form a positive-response id"
        ))
    })
}

/// Parse a raw UDS response PDU. Positive data is returned without the response SID.
pub fn parse_uds_response<'a>(
    request_service_id: u8,
    response_pdu: &'a [u8],
) -> Result<UdsPositiveResponse<'a>, Obd2Error> {
    let service_id = response_pdu
        .first()
        .copied()
        .ok_or_else(|| Obd2Error::ParseError("UDS response PDU is empty".into()))?;

    if service_id == 0x7F {
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

    let expected = positive_response_sid(request_service_id)?;
    if service_id != expected {
        return Err(Obd2Error::ParseError(format!(
            "unexpected UDS response SID 0x{service_id:02X}; expected 0x{expected:02X}"
        )));
    }

    Ok(UdsPositiveResponse {
        service_id,
        data: &response_pdu[1..],
    })
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UdsDiagnosticSessionResponse {
    pub session_type: u8,
    pub timing: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UdsReadDtcResponse {
    pub subfunction: u8,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UdsEcuResetResponse {
    pub reset_type: u8,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UdsSubfunctionResponse {
    pub subfunction: u8,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UdsDidResponse {
    pub did: u16,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UdsRoutineResponse {
    pub control_type: u8,
    pub routine_id: u16,
    pub data: Vec<u8>,
}

/// Three-byte UDS/J1979-2 DTC: two-byte SAE code plus Failure Type Byte.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UdsDtc {
    pub raw: [u8; 3],
    pub code: Dtc,
    pub failure_type: u8,
    pub status: Option<DtcStatusByte>,
}

impl UdsDtc {
    pub fn from_three_bytes(raw: [u8; 3]) -> Self {
        Self::from_three_bytes_and_status(raw, None)
    }

    pub fn from_three_bytes_and_status(raw: [u8; 3], status: Option<DtcStatusByte>) -> Self {
        Self {
            raw,
            code: Dtc::from_bytes(raw[0], raw[1]),
            failure_type: raw[2],
            status,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UdsDtcReport {
    pub subfunction: u8,
    pub status_availability_mask: u8,
    pub dtcs: Vec<UdsDtc>,
}

/// Decode UDS DTC-and-status records: DTC high, DTC low, FTB, status.
pub fn decode_dtc_and_status_records(records: &[u8]) -> Result<Vec<UdsDtc>, Obd2Error> {
    let chunks = records.chunks_exact(4);
    if !chunks.remainder().is_empty() {
        return Err(Obd2Error::ParseError(format!(
            "UDS DTC status records length {} is not divisible by 4",
            records.len()
        )));
    }

    let mut dtcs = Vec::with_capacity(records.len() / 4);
    for chunk in chunks {
        dtcs.push(UdsDtc::from_three_bytes_and_status(
            [chunk[0], chunk[1], chunk[2]],
            Some(DtcStatusByte::from_byte(chunk[3])),
        ));
    }
    Ok(dtcs)
}

#[derive(Debug)]
pub struct UdsClient<T: Transport> {
    transport: T,
}

impl<T: Transport> UdsClient<T> {
    pub fn new(transport: T) -> Self {
        Self { transport }
    }

    pub fn transport(&self) -> &T {
        &self.transport
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn into_inner(self) -> T {
        self.transport
    }

    pub async fn request_service(
        &mut self,
        service_id: u8,
        data: impl Into<Vec<u8>>,
    ) -> Result<Vec<u8>, Obd2Error> {
        let payload = self
            .transport
            .exchange(&TransportRequest {
                service_id,
                data: data.into(),
            })
            .await?;
        normalize_transport_payload(service_id, payload)
    }

    pub async fn diagnostic_session_control(
        &mut self,
        session_type: u8,
    ) -> Result<UdsDiagnosticSessionResponse, Obd2Error> {
        let payload = self
            .request_service(SID_DIAGNOSTIC_SESSION_CONTROL, vec![session_type])
            .await?;
        let (echo, timing) = split_required_echo(&payload, session_type, "session type")?;
        Ok(UdsDiagnosticSessionResponse {
            session_type: echo,
            timing: timing.to_vec(),
        })
    }

    pub async fn read_data_by_identifier(&mut self, did: u16) -> Result<Vec<u8>, Obd2Error> {
        let did_bytes = did.to_be_bytes();
        let payload = self
            .request_service(SID_READ_DATA_BY_IDENTIFIER, did_bytes.to_vec())
            .await?;

        if payload.starts_with(&did_bytes) {
            Ok(payload[did_bytes.len()..].to_vec())
        } else {
            Ok(payload)
        }
    }

    pub async fn read_obdonuds_identifier(&mut self) -> Result<Vec<u8>, Obd2Error> {
        self.read_data_by_identifier(DID_OBDONUDS).await
    }

    pub async fn write_data_by_identifier(
        &mut self,
        did: u16,
        data: &[u8],
    ) -> Result<UdsDidResponse, Obd2Error> {
        let mut request = Vec::with_capacity(2 + data.len());
        request.extend_from_slice(&did.to_be_bytes());
        request.extend_from_slice(data);

        let payload = self
            .request_service(SID_WRITE_DATA_BY_IDENTIFIER, request)
            .await?;
        split_required_did(payload, did, "writeDataByIdentifier")
    }

    pub async fn read_dtc_information(
        &mut self,
        subfunction: u8,
        data: &[u8],
    ) -> Result<UdsReadDtcResponse, Obd2Error> {
        let mut request = Vec::with_capacity(1 + data.len());
        request.push(subfunction);
        request.extend_from_slice(data);

        let payload = self
            .request_service(SID_READ_DTC_INFORMATION, request)
            .await?;
        let (echo, rest) = split_required_echo(&payload, subfunction, "DTC subfunction")?;
        Ok(UdsReadDtcResponse {
            subfunction: echo,
            data: rest.to_vec(),
        })
    }

    pub async fn report_dtcs_by_status_mask(
        &mut self,
        status_mask: u8,
    ) -> Result<UdsDtcReport, Obd2Error> {
        let response = self.read_dtc_information(0x02, &[status_mask]).await?;
        let status_availability_mask = response.data.first().copied().ok_or_else(|| {
            Obd2Error::ParseError("UDS DTC report missing status availability mask".into())
        })?;
        let dtcs = decode_dtc_and_status_records(&response.data[1..])?;
        Ok(UdsDtcReport {
            subfunction: response.subfunction,
            status_availability_mask,
            dtcs,
        })
    }

    pub async fn clear_diagnostic_information(&mut self, group: u32) -> Result<(), Obd2Error> {
        if group > 0xFF_FF_FF {
            return Err(Obd2Error::ParseError(format!(
                "UDS DTC group exceeds 24 bits: 0x{group:X}"
            )));
        }

        let request = vec![
            ((group >> 16) & 0xFF) as u8,
            ((group >> 8) & 0xFF) as u8,
            (group & 0xFF) as u8,
        ];
        let payload = self
            .request_service(SID_CLEAR_DIAGNOSTIC_INFORMATION, request)
            .await?;
        if !payload.is_empty() {
            return Err(Obd2Error::ParseError(format!(
                "UDS clearDiagnosticInformation returned unexpected payload: {payload:02X?}"
            )));
        }
        Ok(())
    }

    pub async fn security_access(
        &mut self,
        subfunction: u8,
        data: &[u8],
    ) -> Result<UdsSubfunctionResponse, Obd2Error> {
        let mut request = Vec::with_capacity(1 + data.len());
        request.push(subfunction);
        request.extend_from_slice(data);

        let payload = self.request_service(SID_SECURITY_ACCESS, request).await?;
        let (echo, rest) =
            split_required_echo(&payload, subfunction, "security-access subfunction")?;
        Ok(UdsSubfunctionResponse {
            subfunction: echo,
            data: rest.to_vec(),
        })
    }

    pub async fn request_seed(&mut self, security_level: u8) -> Result<Vec<u8>, Obd2Error> {
        Ok(self.security_access(security_level, &[]).await?.data)
    }

    pub async fn send_key(
        &mut self,
        security_level: u8,
        key: &[u8],
    ) -> Result<UdsSubfunctionResponse, Obd2Error> {
        self.security_access(security_level, key).await
    }

    pub async fn authentication(
        &mut self,
        subfunction: u8,
        data: &[u8],
    ) -> Result<UdsSubfunctionResponse, Obd2Error> {
        let mut request = Vec::with_capacity(1 + data.len());
        request.push(subfunction);
        request.extend_from_slice(data);

        let payload = self.request_service(SID_AUTHENTICATION, request).await?;
        let (echo, rest) =
            split_required_echo(&payload, subfunction, "authentication subfunction")?;
        Ok(UdsSubfunctionResponse {
            subfunction: echo,
            data: rest.to_vec(),
        })
    }

    pub async fn input_output_control_by_identifier(
        &mut self,
        did: u16,
        control_option_record: &[u8],
    ) -> Result<UdsDidResponse, Obd2Error> {
        let mut request = Vec::with_capacity(2 + control_option_record.len());
        request.extend_from_slice(&did.to_be_bytes());
        request.extend_from_slice(control_option_record);

        let payload = self
            .request_service(SID_INPUT_OUTPUT_CONTROL_BY_IDENTIFIER, request)
            .await?;
        split_required_did(payload, did, "inputOutputControlByIdentifier")
    }

    pub async fn routine_control(
        &mut self,
        control_type: u8,
        routine_id: u16,
        option_record: &[u8],
    ) -> Result<UdsRoutineResponse, Obd2Error> {
        let mut request = Vec::with_capacity(3 + option_record.len());
        request.push(control_type);
        request.extend_from_slice(&routine_id.to_be_bytes());
        request.extend_from_slice(option_record);

        let payload = self.request_service(SID_ROUTINE_CONTROL, request).await?;
        let (echo, rest) = split_required_echo(&payload, control_type, "routine-control type")?;
        if rest.len() < 2 {
            return Err(Obd2Error::ParseError(
                "routineControl response missing routine identifier".into(),
            ));
        }
        let echoed_routine_id = u16::from_be_bytes([rest[0], rest[1]]);
        if echoed_routine_id != routine_id {
            return Err(Obd2Error::ParseError(format!(
                "routineControl response routine id mismatch: expected 0x{routine_id:04X}, got 0x{echoed_routine_id:04X}"
            )));
        }
        Ok(UdsRoutineResponse {
            control_type: echo,
            routine_id: echoed_routine_id,
            data: rest[2..].to_vec(),
        })
    }

    pub async fn control_dtc_setting(
        &mut self,
        setting_type: u8,
        data: &[u8],
    ) -> Result<UdsSubfunctionResponse, Obd2Error> {
        let mut request = Vec::with_capacity(1 + data.len());
        request.push(setting_type);
        request.extend_from_slice(data);

        let payload = self
            .request_service(SID_CONTROL_DTC_SETTING, request)
            .await?;
        let (echo, rest) = split_required_echo(&payload, setting_type, "control-DTC-setting type")?;
        Ok(UdsSubfunctionResponse {
            subfunction: echo,
            data: rest.to_vec(),
        })
    }

    pub async fn tester_present(&mut self) -> Result<(), Obd2Error> {
        let payload = self.request_service(SID_TESTER_PRESENT, vec![0x00]).await?;
        let _ = split_required_echo(&payload, 0x00, "tester-present subfunction")?;
        Ok(())
    }

    pub async fn ecu_reset(&mut self, reset_type: u8) -> Result<UdsEcuResetResponse, Obd2Error> {
        let payload = self
            .request_service(SID_ECU_RESET, vec![reset_type])
            .await?;
        let (echo, rest) = split_required_echo(&payload, reset_type, "reset type")?;
        Ok(UdsEcuResetResponse {
            reset_type: echo,
            data: rest.to_vec(),
        })
    }
}

#[async_trait::async_trait]
impl<T: Transport> ProtocolClient for UdsClient<T> {
    fn name(&self) -> &'static str {
        "UDS"
    }

    async fn request(&mut self, kind: RequestKind) -> Result<DiagResponse, Obd2Error> {
        let (service_id, data) = match kind {
            RequestKind::Did16 { service, did } => (service, did.to_be_bytes().to_vec()),
            RequestKind::Raw { service, data } => (service, data),
            RequestKind::Mode01Pid(pid) => {
                return Err(Obd2Error::ParseError(format!(
                    "UDS client does not support Mode 01 PID request 0x{pid:02X}"
                )));
            }
        };
        let expected_positive_service = positive_response_sid(service_id)?;
        let payload = self.request_service(service_id, data).await?;
        Ok(DiagResponse {
            expected_positive_service,
            payload,
        })
    }
}

fn normalize_transport_payload(service_id: u8, payload: Vec<u8>) -> Result<Vec<u8>, Obd2Error> {
    match payload.first().copied() {
        Some(0x7F) => parse_uds_response(service_id, &payload).map(|resp| resp.data.to_vec()),
        Some(sid) if sid == positive_response_sid(service_id)? => Ok(payload[1..].to_vec()),
        _ => Ok(payload),
    }
}

fn split_required_echo<'a>(
    payload: &'a [u8],
    expected: u8,
    label: &str,
) -> Result<(u8, &'a [u8]), Obd2Error> {
    let echo = payload
        .first()
        .copied()
        .ok_or_else(|| Obd2Error::ParseError(format!("UDS response missing {label} echo byte")))?;
    if echo != expected {
        return Err(Obd2Error::ParseError(format!(
            "UDS response {label} echo mismatch: expected 0x{expected:02X}, got 0x{echo:02X}"
        )));
    }
    Ok((echo, &payload[1..]))
}

fn split_required_did(
    payload: Vec<u8>,
    expected_did: u16,
    label: &str,
) -> Result<UdsDidResponse, Obd2Error> {
    if payload.len() < 2 {
        return Err(Obd2Error::ParseError(format!(
            "UDS {label} response missing DID echo"
        )));
    }
    let echoed_did = u16::from_be_bytes([payload[0], payload[1]]);
    if echoed_did != expected_did {
        return Err(Obd2Error::ParseError(format!(
            "UDS {label} DID echo mismatch: expected 0x{expected_did:04X}, got 0x{echoed_did:04X}"
        )));
    }
    Ok(UdsDidResponse {
        did: echoed_did,
        data: payload[2..].to_vec(),
    })
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;

    use super::*;
    use crate::protocol::codec::BusFamily;

    #[derive(Debug, Default)]
    struct MockFramedTransport {
        expectations: VecDeque<(TransportRequest, Vec<u8>)>,
    }

    impl MockFramedTransport {
        fn expect(&mut self, service_id: u8, data: &[u8], response: &[u8]) {
            self.expectations.push_back((
                TransportRequest {
                    service_id,
                    data: data.to_vec(),
                },
                response.to_vec(),
            ));
        }

        fn is_empty(&self) -> bool {
            self.expectations.is_empty()
        }
    }

    #[async_trait::async_trait]
    impl Transport for MockFramedTransport {
        async fn exchange(&mut self, req: &TransportRequest) -> Result<Vec<u8>, Obd2Error> {
            let (expected, response) = self
                .expectations
                .pop_front()
                .ok_or_else(|| Obd2Error::Transport("unexpected UDS request".into()))?;
            assert_eq!(&expected, req);
            Ok(response)
        }

        fn family(&self) -> BusFamily {
            BusFamily::Can
        }
    }

    #[test]
    fn parse_uds_response_accepts_positive_and_maps_negative_unknown_nrc() {
        let positive = parse_uds_response(0x22, &[0x62, 0xF1, 0x90, 0x12]).unwrap();
        assert_eq!(positive.service_id, 0x62);
        assert_eq!(positive.data, &[0xF1, 0x90, 0x12]);

        let err = parse_uds_response(0x22, &[0x7F, 0x22, 0x94]).unwrap_err();
        assert!(matches!(
            err,
            Obd2Error::NegativeResponse {
                service: 0x22,
                nrc: NegativeResponse::Unknown(0x94)
            }
        ));
    }

    #[test]
    fn uds_dtc_model_decodes_two_byte_code_and_failure_type() {
        let dtc = UdsDtc::from_three_bytes([0x03, 0x01, 0x11]);
        assert_eq!(dtc.raw, [0x03, 0x01, 0x11]);
        assert_eq!(dtc.code.code, "P0301");
        assert_eq!(dtc.failure_type, 0x11);
        assert!(dtc.status.is_none());
    }

    #[test]
    fn decode_dtc_status_records_rejects_partial_record() {
        assert!(matches!(
            decode_dtc_and_status_records(&[0x03, 0x01, 0x11]),
            Err(Obd2Error::ParseError(_))
        ));
    }

    #[tokio::test]
    async fn uds_client_builds_core_service_requests() {
        let mut transport = MockFramedTransport::default();
        transport.expect(0x10, &[0x03], &[0x03, 0x00, 0x32, 0x01, 0xF4]);
        transport.expect(0x22, &[0xF1, 0x90], &[0xF1, 0x90, 0x12, 0x34]);
        transport.expect(0x19, &[0x02, 0xFF], &[0x02, 0x8F, 0x03, 0x01, 0x11, 0x0B]);
        transport.expect(0x14, &[0xFF, 0xFF, 0xFF], &[]);
        transport.expect(0x27, &[0x01], &[0x01, 0xAA, 0x55]);
        transport.expect(0x27, &[0x02, 0x12, 0x34], &[0x02]);
        transport.expect(0x29, &[0x01, 0x99], &[0x01, 0x00]);
        transport.expect(0x2E, &[0xF1, 0x90, 0x12, 0x34], &[0xF1, 0x90]);
        transport.expect(0x2F, &[0x12, 0x34, 0x03, 0x7F], &[0x12, 0x34, 0x03]);
        transport.expect(0x31, &[0x01, 0xFF, 0x00, 0x42], &[0x01, 0xFF, 0x00, 0x7E]);
        transport.expect(0x3E, &[0x00], &[0x00]);
        transport.expect(0x85, &[0x02], &[0x02]);
        transport.expect(0x11, &[0x01], &[0x01, 0x05]);

        let mut client = UdsClient::new(transport);

        let session = client.diagnostic_session_control(0x03).await.unwrap();
        assert_eq!(session.session_type, 0x03);
        assert_eq!(session.timing, vec![0x00, 0x32, 0x01, 0xF4]);

        let did = client.read_data_by_identifier(0xF190).await.unwrap();
        assert_eq!(did, vec![0x12, 0x34]);

        let report = client.report_dtcs_by_status_mask(0xFF).await.unwrap();
        assert_eq!(report.subfunction, 0x02);
        assert_eq!(report.status_availability_mask, 0x8F);
        assert_eq!(report.dtcs.len(), 1);
        assert_eq!(report.dtcs[0].code.code, "P0301");
        assert_eq!(report.dtcs[0].failure_type, 0x11);
        assert_eq!(report.dtcs[0].status, Some(DtcStatusByte::from_byte(0x0B)));

        client
            .clear_diagnostic_information(0xFF_FF_FF)
            .await
            .unwrap();

        assert_eq!(client.request_seed(0x01).await.unwrap(), vec![0xAA, 0x55]);
        let key = client.send_key(0x02, &[0x12, 0x34]).await.unwrap();
        assert_eq!(key.subfunction, 0x02);
        assert!(key.data.is_empty());

        let auth = client.authentication(0x01, &[0x99]).await.unwrap();
        assert_eq!(auth.subfunction, 0x01);
        assert_eq!(auth.data, vec![0x00]);

        let write = client
            .write_data_by_identifier(0xF190, &[0x12, 0x34])
            .await
            .unwrap();
        assert_eq!(write.did, 0xF190);
        assert!(write.data.is_empty());

        let io_control = client
            .input_output_control_by_identifier(0x1234, &[0x03, 0x7F])
            .await
            .unwrap();
        assert_eq!(io_control.did, 0x1234);
        assert_eq!(io_control.data, vec![0x03]);

        let routine = client.routine_control(0x01, 0xFF00, &[0x42]).await.unwrap();
        assert_eq!(routine.control_type, 0x01);
        assert_eq!(routine.routine_id, 0xFF00);
        assert_eq!(routine.data, vec![0x7E]);

        client.tester_present().await.unwrap();
        let dtc_setting = client.control_dtc_setting(0x02, &[]).await.unwrap();
        assert_eq!(dtc_setting.subfunction, 0x02);
        assert!(dtc_setting.data.is_empty());

        let reset = client.ecu_reset(0x01).await.unwrap();
        assert_eq!(reset.reset_type, 0x01);
        assert_eq!(reset.data, vec![0x05]);

        assert!(client.into_inner().is_empty());
    }

    #[tokio::test]
    async fn uds_client_accepts_raw_positive_pdu_from_transport() {
        let mut transport = MockFramedTransport::default();
        transport.expect(0x22, &[0xF8, 0x10], &[0x62, 0xF8, 0x10, 0x01]);
        let mut client = UdsClient::new(transport);

        assert_eq!(client.read_obdonuds_identifier().await.unwrap(), vec![0x01]);
    }
}
