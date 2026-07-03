use crate::error::Obd2Error;

pub(crate) fn parse_hex_line(line: &str) -> Result<Vec<u8>, Obd2Error> {
    let tokens: Vec<&str> = line.split_whitespace().collect();
    parse_hex_tokens(&tokens)
}

pub(crate) fn parse_hex_tokens(tokens: &[&str]) -> Result<Vec<u8>, Obd2Error> {
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

pub(crate) fn parse_compact_hex(input: &str) -> Result<Vec<u8>, Obd2Error> {
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

pub(crate) fn hex_nibble(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        b'A'..=b'F' => Some(byte - b'A' + 10),
        _ => None,
    }
}

pub(crate) fn invalid_hex_byte(token: &str) -> Obd2Error {
    Obd2Error::ParseError(format!("invalid hex byte: {token}"))
}
