//! ELM327 ASCII response decoding.

use crate::error::Obd2Error;
use crate::protocol::codec::{
    decode_can_headers_on, decode_iso_kline_headers_on, decode_j1850_headers_on, BusFamily,
};

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
