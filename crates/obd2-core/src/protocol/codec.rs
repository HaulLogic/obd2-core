//! Protocol-aware framing and response decoding helpers.

use crate::error::Obd2Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusFamily {
    Can,
    J1850,
    Iso9141,
    Kwp2000,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecodedFrame {
    pub family: BusFamily,
    pub source: Option<u32>,
    pub target: Option<u32>,
    pub service: Option<u8>,
    pub identifier: Option<u32>,
    pub payload: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CanFrameKind {
    Single,
    First,
    Consecutive,
    FlowControl,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanFrame {
    pub identifier: u32,
    pub source: Option<u32>,
    pub target: Option<u32>,
    pub kind: CanFrameKind,
    pub payload: Vec<u8>,
    pub declared_length: Option<usize>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct J1850Frame {
    pub priority: u8,
    pub target: u8,
    pub source: u8,
    pub payload: Vec<u8>,
    pub checksum: Option<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IsoKLineFrame {
    pub format: u8,
    pub target: u8,
    pub source: u8,
    pub payload: Vec<u8>,
    pub checksum: u8,
    pub checksum_valid: bool,
}

pub fn decode_can_headers_on(line: &str) -> Result<CanFrame, Obd2Error> {
    let tokens: Vec<&str> = line.split_whitespace().collect();
    if tokens.len() < 3 {
        return Err(Obd2Error::ParseError(format!("invalid CAN frame: {line}")));
    }

    let identifier = u32::from_str_radix(tokens[0], 16)
        .map_err(|_| Obd2Error::ParseError(format!("invalid CAN identifier: {}", tokens[0])))?;
    let bytes = parse_hex_tokens(&tokens[1..])?;
    if bytes.is_empty() {
        return Err(Obd2Error::ParseError("CAN frame missing PCI byte".into()));
    }

    let pci = bytes[0];
    let kind = match pci >> 4 {
        0x0 => CanFrameKind::Single,
        0x1 => CanFrameKind::First,
        0x2 => CanFrameKind::Consecutive,
        0x3 => CanFrameKind::FlowControl,
        _ => {
            return Err(Obd2Error::ParseError(format!(
                "unsupported CAN PCI type: {:02X}",
                pci
            )))
        }
    };

    let (payload, declared_length) = match kind {
        CanFrameKind::Single => {
            let len = (pci & 0x0F) as usize;
            (bytes[1..].iter().copied().take(len).collect(), Some(len))
        }
        CanFrameKind::First => {
            let len =
                (((pci & 0x0F) as usize) << 8) | bytes.get(1).copied().unwrap_or_default() as usize;
            (bytes[2..].to_vec(), Some(len))
        }
        CanFrameKind::Consecutive => (bytes[1..].to_vec(), None),
        CanFrameKind::FlowControl => (bytes[1..].to_vec(), None),
    };

    let (source, target) = if identifier <= 0x7FF {
        let source = if (0x7E8..=0x7EF).contains(&identifier) {
            Some(identifier - 0x08)
        } else {
            None
        };
        let target = if (0x7E0..=0x7E7).contains(&identifier) {
            Some(identifier)
        } else {
            None
        };
        (source, target)
    } else {
        let source = Some(identifier & 0xFF);
        let pdu_format = (identifier >> 16) & 0xFF;
        let target = if pdu_format < 0xF0 {
            Some((identifier >> 8) & 0xFF)
        } else {
            None
        };
        (source, target)
    };

    Ok(CanFrame {
        identifier,
        source,
        target,
        kind,
        payload,
        declared_length,
    })
}

pub fn decode_can_headers_off(bytes: &[u8]) -> Result<DecodedFrame, Obd2Error> {
    if bytes.len() < 2 {
        return Err(Obd2Error::ParseError("CAN payload too short".into()));
    }
    Ok(DecodedFrame {
        family: BusFamily::Can,
        source: None,
        target: None,
        service: bytes.first().copied(),
        identifier: None,
        payload: bytes.to_vec(),
    })
}

pub fn decode_j1850_headers_on(line: &str) -> Result<J1850Frame, Obd2Error> {
    let bytes = parse_hex_line(line)?;
    if bytes.len() < 4 {
        return Err(Obd2Error::ParseError(format!(
            "invalid J1850 frame: {line}"
        )));
    }
    let checksum = bytes.last().copied();
    Ok(J1850Frame {
        priority: bytes[0],
        target: bytes[1],
        source: bytes[2],
        payload: bytes[3..bytes.len() - 1].to_vec(),
        checksum,
    })
}

pub fn decode_iso_kline_headers_on(line: &str) -> Result<IsoKLineFrame, Obd2Error> {
    let bytes = parse_hex_line(line)?;
    if bytes.len() < 5 {
        return Err(Obd2Error::ParseError(format!(
            "invalid ISO/KWP frame: {line}"
        )));
    }
    let checksum = *bytes
        .last()
        .ok_or_else(|| Obd2Error::ParseError("missing checksum".into()))?;
    let computed = bytes[..bytes.len() - 1]
        .iter()
        .fold(0u8, |sum, byte| sum.wrapping_add(*byte));
    Ok(IsoKLineFrame {
        format: bytes[0],
        target: bytes[1],
        source: bytes[2],
        payload: bytes[3..bytes.len() - 1].to_vec(),
        checksum,
        checksum_valid: checksum == computed,
    })
}

pub fn decode_frame(line: &str, family: BusFamily) -> Result<DecodedFrame, Obd2Error> {
    match family {
        BusFamily::Can => {
            let frame = decode_can_headers_on(line)?;
            Ok(DecodedFrame {
                family,
                source: frame.source,
                target: frame.target,
                service: frame.payload.first().copied(),
                identifier: Some(frame.identifier),
                payload: frame.payload,
            })
        }
        BusFamily::J1850 => {
            let frame = decode_j1850_headers_on(line)?;
            Ok(DecodedFrame {
                family,
                source: Some(frame.source as u32),
                target: Some(frame.target as u32),
                service: frame.payload.first().copied(),
                identifier: None,
                payload: frame.payload,
            })
        }
        BusFamily::Iso9141 | BusFamily::Kwp2000 => {
            let frame = decode_iso_kline_headers_on(line)?;
            Ok(DecodedFrame {
                family,
                source: Some(frame.source as u32),
                target: Some(frame.target as u32),
                service: frame.payload.first().copied(),
                identifier: None,
                payload: frame.payload,
            })
        }
    }
}

pub fn decode_elm_response_payload(
    response: &str,
    family: BusFamily,
    skip_bytes: usize,
) -> Result<Vec<u8>, Obd2Error> {
    decode_elm_response_payload_for_command(response, family, skip_bytes, None)
}

pub fn decode_elm_response_payload_for_command(
    response: &str,
    family: BusFamily,
    skip_bytes: usize,
    echo_command: Option<&str>,
) -> Result<Vec<u8>, Obd2Error> {
    let expected_prefix = echo_command.and_then(|cmd| expected_response_prefix(cmd, skip_bytes));
    let mut matched_expected = expected_prefix.is_none();
    let mut payload = Vec::new();

    for raw_line in response.split(['\r', '\n']) {
        let line = raw_line.trim().trim_end_matches('>');
        if line.is_empty() || line == "SEARCHING..." {
            continue;
        }
        if echo_command.is_some_and(|cmd| line_matches_command_echo(line, cmd)) {
            continue;
        }

        let tokens: Vec<&str> = line.split_whitespace().collect();
        if tokens.is_empty() {
            continue;
        }

        let allow_status_noise = matched_expected && expected_prefix.is_some();
        let looks_like_headers_on = match family {
            BusFamily::Can => tokens.first().is_some_and(|t| t.len() > 2),
            BusFamily::J1850 | BusFamily::Iso9141 | BusFamily::Kwp2000 => tokens.len() >= 6,
        };

        let decoded = if looks_like_headers_on {
            match family {
                BusFamily::Can => decode_can_headers_on(line)
                    .map(|frame| frame.payload)
                    .or_else(|_| parse_hex_line(line)),
                BusFamily::J1850 => decode_j1850_headers_on(line)
                    .map(|frame| {
                        let mut bytes = Vec::with_capacity(3 + frame.payload.len());
                        bytes.push(frame.priority);
                        bytes.push(frame.target);
                        bytes.push(frame.source);
                        bytes.extend(frame.payload);
                        bytes
                    })
                    .or_else(|_| parse_hex_line(line)),
                BusFamily::Iso9141 | BusFamily::Kwp2000 => decode_iso_kline_headers_on(line)
                    .map(|frame| {
                        let mut bytes = Vec::with_capacity(3 + frame.payload.len());
                        bytes.push(frame.format);
                        bytes.push(frame.target);
                        bytes.push(frame.source);
                        bytes.extend(frame.payload);
                        bytes
                    })
                    .or_else(|_| parse_hex_line(line)),
            }
        } else {
            parse_hex_line(line)
        };

        let decoded = match decoded {
            Ok(decoded) => decoded,
            Err(e) if allow_status_noise => {
                continue;
            }
            Err(e) => return Err(e),
        };

        if let Some(prefix) = &expected_prefix {
            if decoded.starts_with(prefix) {
                payload.extend_from_slice(&decoded[prefix.len()..]);
                matched_expected = true;
            } else if matched_expected {
                payload.extend(decoded);
            }
        } else {
            payload.extend(decoded);
        }
    }

    if payload.is_empty() {
        return Err(Obd2Error::ParseError(format!(
            "no valid payload in response: {}",
            response.trim()
        )));
    }

    if expected_prefix.is_some() {
        return Ok(payload);
    }

    if skip_bytes >= payload.len() {
        return Ok(Vec::new());
    }
    Ok(payload.split_off(skip_bytes))
}

fn parse_hex_line(line: &str) -> Result<Vec<u8>, Obd2Error> {
    let tokens: Vec<&str> = line.split_whitespace().collect();
    parse_hex_tokens(&tokens)
}

fn parse_hex_tokens(tokens: &[&str]) -> Result<Vec<u8>, Obd2Error> {
    let mut bytes = Vec::new();
    for token in tokens {
        let raw = token.as_bytes();
        if raw.is_empty() {
            continue;
        }
        if raw.len() % 2 != 0 {
            return Err(invalid_hex_byte(token));
        }

        for pair in raw.chunks_exact(2) {
            let high = hex_nibble(pair[0]).ok_or_else(|| invalid_hex_byte(token))?;
            let low = hex_nibble(pair[1]).ok_or_else(|| invalid_hex_byte(token))?;
            bytes.push((high << 4) | low);
        }
    }
    Ok(bytes)
}

fn hex_nibble(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        b'A'..=b'F' => Some(byte - b'A' + 10),
        _ => None,
    }
}

fn invalid_hex_byte(token: &str) -> Obd2Error {
    Obd2Error::ParseError(format!("invalid hex byte: {token}"))
}

fn expected_response_prefix(command: &str, skip_bytes: usize) -> Option<Vec<u8>> {
    if skip_bytes == 0 {
        return None;
    }
    let request = parse_compact_hex(command).ok()?;
    if request.len() < skip_bytes {
        return None;
    }

    let mut prefix = Vec::with_capacity(skip_bytes);
    prefix.push(request[0].wrapping_add(0x40));
    prefix.extend_from_slice(&request[1..skip_bytes]);
    Some(prefix)
}

fn parse_compact_hex(input: &str) -> Result<Vec<u8>, Obd2Error> {
    let mut bytes = Vec::new();
    let mut high = None;

    for byte in input.bytes().filter(|byte| !byte.is_ascii_whitespace()) {
        let nibble = hex_nibble(byte).ok_or_else(|| invalid_hex_byte(input))?;
        if let Some(high_nibble) = high.take() {
            bytes.push((high_nibble << 4) | nibble);
        } else {
            high = Some(nibble);
        }
    }

    if high.is_some() {
        return Err(invalid_hex_byte(input));
    }
    Ok(bytes)
}

fn line_matches_command_echo(line: &str, command: &str) -> bool {
    let mut command_bytes = command.bytes().filter(|byte| !byte.is_ascii_whitespace());
    let mut saw_byte = false;

    for byte in line.bytes().filter(|byte| !byte.is_ascii_whitespace()) {
        saw_byte = true;
        let Some(expected) = command_bytes.next() else {
            return false;
        };
        if byte.to_ascii_uppercase() != expected.to_ascii_uppercase() {
            return false;
        }
    }

    saw_byte && command_bytes.next().is_none()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_decode_can_single_frame_headers_on() {
        let frame = decode_can_headers_on("7E8 06 41 0C 1A F8 00 00").unwrap();
        assert_eq!(frame.identifier, 0x7E8);
        assert_eq!(frame.source, Some(0x7E0));
        assert_eq!(frame.kind, CanFrameKind::Single);
        assert_eq!(frame.payload, vec![0x41, 0x0C, 0x1A, 0xF8, 0x00, 0x00]);
        assert_eq!(frame.declared_length, Some(6));
    }

    #[test]
    fn test_decode_can_first_frame_headers_on() {
        let frame = decode_can_headers_on("7E8 10 14 49 02 01 31 47 31").unwrap();
        assert_eq!(frame.kind, CanFrameKind::First);
        assert_eq!(frame.declared_length, Some(20));
        assert_eq!(frame.payload, vec![0x49, 0x02, 0x01, 0x31, 0x47, 0x31]);
    }

    #[test]
    fn test_decode_j1850_headers_on() {
        let frame = decode_j1850_headers_on("48 6B 10 41 0C 1A F8 C4").unwrap();
        assert_eq!(frame.target, 0x6B);
        assert_eq!(frame.source, 0x10);
        assert_eq!(frame.payload, vec![0x41, 0x0C, 0x1A, 0xF8]);
        assert_eq!(frame.checksum, Some(0xC4));
    }

    #[test]
    fn test_decode_iso_kline_headers_on() {
        let frame = decode_iso_kline_headers_on("68 6A F1 01 00 C4").unwrap();
        assert_eq!(frame.target, 0x6A);
        assert_eq!(frame.source, 0xF1);
        assert_eq!(frame.payload, vec![0x01, 0x00]);
        assert!(frame.checksum_valid);
    }

    #[test]
    fn test_decode_generic_frame() {
        let frame = decode_frame("48 6B 10 41 0C 1A F8 C4", BusFamily::J1850).unwrap();
        assert_eq!(frame.service, Some(0x41));
        assert_eq!(frame.source, Some(0x10));
    }

    #[test]
    fn test_decode_can_headers_off_payload() {
        let frame = decode_can_headers_off(&[0x41, 0x0C, 0x1A, 0xF8]).unwrap();
        assert_eq!(frame.family, BusFamily::Can);
        assert_eq!(frame.service, Some(0x41));
        assert_eq!(frame.payload, vec![0x41, 0x0C, 0x1A, 0xF8]);
    }

    #[test]
    fn test_decode_elm_response_payload_compact_headers_off() {
        let payload = decode_elm_response_payload("410C0A9B\r>", BusFamily::Can, 2).unwrap();
        assert_eq!(payload, vec![0x0A, 0x9B]);
    }

    #[test]
    fn test_decode_elm_response_payload_skips_compact_echo() {
        let payload = decode_elm_response_payload_for_command(
            "010C\r410C0A9B\r>",
            BusFamily::Can,
            2,
            Some("010C"),
        )
        .unwrap();
        assert_eq!(payload, vec![0x0A, 0x9B]);
    }

    #[test]
    fn test_decode_elm_response_payload_skips_spaced_echo() {
        let payload = decode_elm_response_payload_for_command(
            "01 0C\r41 0C 0A 9B\r>",
            BusFamily::Can,
            2,
            Some("010C"),
        )
        .unwrap();
        assert_eq!(payload, vec![0x0A, 0x9B]);
    }

    #[test]
    fn test_decode_elm_response_payload_rejects_odd_hex_token() {
        let result = decode_elm_response_payload("410C0A9\r>", BusFamily::Can, 2);
        assert!(matches!(result, Err(Obd2Error::ParseError(_))));
    }

    #[test]
    fn test_decode_elm_response_payload_ignores_status_after_matching_payload() {
        let payload = decode_elm_response_payload_for_command(
            "41057786\rNO DATA\r>",
            BusFamily::Can,
            2,
            Some("0105"),
        )
        .unwrap();
        assert_eq!(payload, vec![0x77, 0x86]);
    }

    #[test]
    fn test_decode_elm_response_payload_rejects_wrong_pid_response() {
        let result =
            decode_elm_response_payload_for_command("410C0A9B\r>", BusFamily::Can, 2, Some("0105"));
        assert!(matches!(result, Err(Obd2Error::ParseError(_))));
    }
}
