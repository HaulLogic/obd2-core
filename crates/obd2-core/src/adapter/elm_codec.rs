//! ELM327 ASCII response decoding.

use crate::error::Obd2Error;
use crate::protocol::codec::{
    decode_can_headers_on, decode_iso_kline_headers_on, decode_j1850_headers_on, BusFamily,
};
use crate::protocol::hex::{parse_compact_hex, parse_hex_line};

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
    let mut payloads =
        decode_elm_response_payloads_for_command(response, family, skip_bytes, echo_command)?;
    if is_mode09_vin_request(echo_command) {
        return assemble_mode09_vin_payload(&payloads);
    }
    if expected_prefix_is_used(echo_command, skip_bytes) {
        return Ok(payloads.remove(0));
    }

    let mut payload = payloads.remove(0);
    if skip_bytes >= payload.len() {
        return Ok(Vec::new());
    }
    Ok(payload.split_off(skip_bytes))
}

pub fn decode_elm_response_payloads_for_command(
    response: &str,
    family: BusFamily,
    skip_bytes: usize,
    echo_command: Option<&str>,
) -> Result<Vec<Vec<u8>>, Obd2Error> {
    let expected_prefix = echo_command.and_then(|cmd| expected_response_prefix(cmd, skip_bytes));
    let mut matched_expected = expected_prefix.is_none();
    let mut payloads = Vec::new();
    let mut current = Vec::new();

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

        if matched_expected && expected_prefix.is_some() && is_status_noise_line(line) {
            continue;
        }

        let decoded = decode_line(line, family, skip_bytes, expected_prefix.as_deref())?;

        if let Some(prefix) = &expected_prefix {
            if decoded.starts_with(prefix) {
                if matched_expected && !current.is_empty() {
                    payloads.push(std::mem::take(&mut current));
                }
                current.extend_from_slice(&decoded[prefix.len()..]);
                matched_expected = true;
            } else if matched_expected {
                current.extend(decoded);
            }
        } else {
            current.extend(decoded);
        }
    }

    if !current.is_empty() {
        payloads.push(current);
    }

    if payloads.is_empty() {
        return Err(Obd2Error::ParseError(format!(
            "no valid payload in response: {}",
            response.trim()
        )));
    }

    Ok(payloads)
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

fn expected_prefix_is_used(echo_command: Option<&str>, skip_bytes: usize) -> bool {
    echo_command.is_some() && skip_bytes > 0
}

fn is_mode09_vin_request(echo_command: Option<&str>) -> bool {
    echo_command
        .and_then(|command| parse_compact_hex(command).ok())
        .is_some_and(|request| request == [0x09, 0x02])
}

fn assemble_mode09_vin_payload(payloads: &[Vec<u8>]) -> Result<Vec<u8>, Obd2Error> {
    const VIN_LENGTH: usize = 17;

    // SAE J1979 Mode 09 responses carry a sequence byte after 49 02. ELM
    // adapters with headers disabled commonly repeat 49 02 on every physical
    // frame, so the generic decoder intentionally returns one payload per
    // frame. Reassemble only this sequenced response; repeated prefixes for
    // ordinary PID queries may be replies from multiple ECUs and must remain
    // separate.
    let uses_sequence_frames = payloads
        .first()
        .and_then(|payload| payload.first())
        .is_some_and(|sequence| *sequence == 1);
    let mut vin = Vec::with_capacity(VIN_LENGTH);

    if uses_sequence_frames {
        let mut expected_sequence = 1u8;
        for payload in payloads {
            let Some((&sequence, frame_data)) = payload.split_first() else {
                return Err(Obd2Error::ParseError("empty Mode 09 VIN frame".to_string()));
            };
            if sequence != expected_sequence {
                return Err(Obd2Error::ParseError(format!(
                    "out-of-sequence Mode 09 VIN frame: expected {expected_sequence}, got {sequence}"
                )));
            }

            vin.extend(
                frame_data
                    .iter()
                    .copied()
                    .filter(|byte| (0x20..=0x7e).contains(byte)),
            );
            if vin.len() >= VIN_LENGTH {
                vin.truncate(VIN_LENGTH);
                return Ok(vin);
            }
            expected_sequence = expected_sequence.checked_add(1).ok_or_else(|| {
                Obd2Error::ParseError("Mode 09 VIN frame sequence overflow".to_string())
            })?;
        }
    } else {
        vin.extend(
            payloads
                .iter()
                .flatten()
                .copied()
                .filter(|byte| (0x20..=0x7e).contains(byte)),
        );
        if vin.len() >= VIN_LENGTH {
            vin.truncate(VIN_LENGTH);
            return Ok(vin);
        }
    }

    Err(Obd2Error::ParseError(format!(
        "incomplete Mode 09 VIN response: {} of {VIN_LENGTH} characters",
        vin.len()
    )))
}

fn decode_line(
    line: &str,
    family: BusFamily,
    skip_bytes: usize,
    expected_prefix: Option<&[u8]>,
) -> Result<Vec<u8>, Obd2Error> {
    if let Ok(bytes) = parse_hex_line(line) {
        if should_treat_as_headers_off(&bytes, skip_bytes, expected_prefix) {
            return Ok(bytes);
        }
    }

    let tokens: Vec<&str> = line.split_whitespace().collect();
    let looks_like_headers_on = match family {
        BusFamily::Can => tokens.first().is_some_and(|t| t.len() > 2),
        BusFamily::J1850 | BusFamily::Iso9141 | BusFamily::Kwp2000 => tokens.len() >= 6,
    };

    if !looks_like_headers_on {
        return parse_hex_line(line);
    }

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
        BusFamily::Iso9141 | BusFamily::Kwp2000 => match decode_iso_kline_headers_on(line) {
            Ok(frame) if frame.checksum_valid => {
                let mut bytes = Vec::with_capacity(3 + frame.payload.len());
                bytes.push(frame.format);
                bytes.push(frame.target);
                bytes.push(frame.source);
                bytes.extend(frame.payload);
                Ok(bytes)
            }
            _ => parse_hex_line(line),
        },
    }
}

fn should_treat_as_headers_off(
    bytes: &[u8],
    skip_bytes: usize,
    expected_prefix: Option<&[u8]>,
) -> bool {
    if expected_prefix.is_some_and(|prefix| bytes.starts_with(prefix)) {
        return true;
    }
    skip_bytes > 0
        && bytes
            .first()
            .is_some_and(|service| is_likely_response_service(*service))
}

fn is_likely_response_service(service: u8) -> bool {
    matches!(
        service,
        0x41 | 0x42 | 0x43 | 0x44 | 0x45 | 0x46 | 0x47 | 0x49 | 0x4A | 0x61 | 0x62 | 0x7F
    )
}

fn is_status_noise_line(line: &str) -> bool {
    matches!(line, "NO DATA" | "STOPPED" | "BUS BUSY")
}

fn line_matches_command_echo(line: &str, command: &str) -> bool {
    let mut command_bytes = command.bytes().filter(|byte| !byte.is_ascii_whitespace());
    let mut saw_byte = false;

    for byte in line.bytes().filter(|byte| !byte.is_ascii_whitespace()) {
        saw_byte = true;
        let Some(expected) = command_bytes.next() else {
            return false;
        };
        if !byte.eq_ignore_ascii_case(&expected) {
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

    #[test]
    fn headers_off_j1850_six_byte_response_keeps_all_payload_bytes() {
        let payload = decode_elm_response_payload_for_command(
            "41 00 BE 3E B8 11\r>",
            BusFamily::J1850,
            2,
            Some("0100"),
        )
        .unwrap();
        assert_eq!(payload, vec![0xBE, 0x3E, 0xB8, 0x11]);
    }

    #[test]
    fn headers_off_kline_six_byte_response_keeps_all_payload_bytes() {
        let payload = decode_elm_response_payload_for_command(
            "41 00 BE 3E B8 11\r>",
            BusFamily::Iso9141,
            2,
            Some("0100"),
        )
        .unwrap();
        assert_eq!(payload, vec![0xBE, 0x3E, 0xB8, 0x11]);
    }

    #[test]
    fn headers_off_kline_without_command_keeps_all_payload_bytes() {
        let payload =
            decode_elm_response_payload("41 00 BE 3E B8 11\r>", BusFamily::Iso9141, 2).unwrap();
        assert_eq!(payload, vec![0xBE, 0x3E, 0xB8, 0x11]);
    }

    #[test]
    fn corrupted_continuation_after_match_returns_error() {
        let result = decode_elm_response_payload_for_command(
            "49 02 01 31 47\rXYZ\r>",
            BusFamily::Can,
            2,
            Some("0902"),
        );
        assert!(matches!(result, Err(Obd2Error::ParseError(_))));
    }

    #[test]
    fn repeated_prefix_responses_remain_separate() {
        let payloads = decode_elm_response_payloads_for_command(
            "41 00 BE 3E B8 11\r41 00 80 00 00 04\r>",
            BusFamily::Can,
            2,
            Some("0100"),
        )
        .unwrap();
        assert_eq!(
            payloads,
            vec![vec![0xBE, 0x3E, 0xB8, 0x11], vec![0x80, 0x00, 0x00, 0x04],]
        );
    }

    #[test]
    fn headers_off_j1850_mode09_vin_frames_are_reassembled() {
        let payload = decode_elm_response_payload_for_command(
            concat!(
                "49020100000031\r",
                "4902024743484B\r",
                "49020332333232\r",
                "49020434463030\r",
                "49020530303031\r>"
            ),
            BusFamily::J1850,
            2,
            Some("0902"),
        )
        .unwrap();

        assert_eq!(payload, b"1GCHK23224F000001");
    }

    #[test]
    fn incomplete_mode09_vin_frames_are_rejected() {
        let result = decode_elm_response_payload_for_command(
            "49020100000031\r4902024743484B\r>",
            BusFamily::J1850,
            2,
            Some("0902"),
        );

        assert!(matches!(result, Err(Obd2Error::ParseError(_))));
    }

    #[test]
    fn out_of_sequence_mode09_vin_frames_are_rejected() {
        let result = decode_elm_response_payload_for_command(
            concat!(
                "49020100000031\r",
                "49020332333232\r",
                "49020434463030\r",
                "49020530303031\r>"
            ),
            BusFamily::J1850,
            2,
            Some("0902"),
        );

        assert!(matches!(result, Err(Obd2Error::ParseError(_))));
    }
}
