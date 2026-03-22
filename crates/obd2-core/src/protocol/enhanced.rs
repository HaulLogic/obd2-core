//! Enhanced manufacturer-specific PID types.

use std::time::Instant;
use crate::error::Obd2Error;

/// Confidence level of diagnostic data from a vehicle spec.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Confidence {
    Verified,
    Community,
    Inferred,
    Unverified,
}

/// How to decode raw response bytes into a Value.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum Formula {
    Linear { scale: f64, offset: f64 },
    TwoByte { scale: f64, offset: f64 },
    Centered { center: f64, divisor: f64 },
    Bitmask { bits: Vec<(u8, String)> },
    Enumerated { values: Vec<(u8, String)> },
    Expression(String),
}

/// Enhanced manufacturer-specific PID (Mode 21/22).
#[derive(Debug, Clone)]
pub struct EnhancedPid {
    pub service_id: u8,
    pub did: u16,
    pub name: String,
    pub unit: String,
    pub formula: Formula,
    pub bytes: u8,
    pub module: String,
    pub value_type: super::pid::ValueType,
    pub confidence: Confidence,
    pub command_suffix: Option<Vec<u8>>,
}

/// A decoded value from an OBD-II response.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum Value {
    Scalar(f64),
    Bitfield(Bitfield),
    State(String),
    Raw(Vec<u8>),
}

impl Value {
    pub fn as_f64(&self) -> Result<f64, Obd2Error> {
        match self {
            Value::Scalar(v) => Ok(*v),
            _ => Err(Obd2Error::ParseError("expected scalar value".into())),
        }
    }

    pub fn as_bitfield(&self) -> Result<&Bitfield, Obd2Error> {
        match self {
            Value::Bitfield(b) => Ok(b),
            _ => Err(Obd2Error::ParseError("expected bitfield value".into())),
        }
    }
}

/// A set of named boolean flags decoded from raw bytes.
#[derive(Debug, Clone)]
pub struct Bitfield {
    pub raw: u32,
    pub flags: Vec<(String, bool)>,
}

/// Source of a reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadingSource {
    Live,
    FreezeFrame,
    Replay,
}

/// A single reading from the vehicle.
#[derive(Debug, Clone)]
pub struct Reading {
    pub value: Value,
    pub unit: &'static str,
    pub timestamp: Instant,
    pub raw_bytes: Vec<u8>,
    pub source: ReadingSource,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_value_as_f64() {
        let v = Value::Scalar(42.0);
        assert_eq!(v.as_f64().unwrap(), 42.0);
    }

    #[test]
    fn test_value_as_f64_error_on_bitfield() {
        let v = Value::Bitfield(Bitfield { raw: 0xFF, flags: vec![] });
        assert!(v.as_f64().is_err());
    }

    #[test]
    fn test_value_as_bitfield() {
        let bf = Bitfield { raw: 0xAB, flags: vec![("test".into(), true)] };
        let v = Value::Bitfield(bf);
        let result = v.as_bitfield().unwrap();
        assert_eq!(result.raw, 0xAB);
    }

    #[test]
    fn test_reading_source_equality() {
        assert_ne!(ReadingSource::Live, ReadingSource::FreezeFrame);
        assert_ne!(ReadingSource::Live, ReadingSource::Replay);
    }
}
