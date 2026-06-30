//! Standard OBD-II PID definitions.

use super::enhanced::{Bitfield, Value};
use crate::error::Obd2Error;

/// The type of value a PID returns.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Deserialize)]
pub enum ValueType {
    /// Numeric measurement (temperature, pressure, RPM, %)
    Scalar,
    /// Bitfield with named flags (readiness monitors, solenoid state)
    Bitfield,
    /// Enumerated state (gear position, key position)
    State,
}

/// Standard OBD-II PID (Mode 01/02). Newtype over u8 for type safety.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Pid(pub u8);

impl Pid {
    // Status & readiness
    pub const SUPPORTED_PIDS_01_20: Pid = Pid(0x00);
    pub const MONITOR_STATUS: Pid = Pid(0x01);
    pub const FUEL_SYSTEM_STATUS: Pid = Pid(0x03);

    // Engine performance
    pub const ENGINE_LOAD: Pid = Pid(0x04);
    pub const COOLANT_TEMP: Pid = Pid(0x05);
    pub const SHORT_FUEL_TRIM_B1: Pid = Pid(0x06);
    pub const LONG_FUEL_TRIM_B1: Pid = Pid(0x07);
    pub const SHORT_FUEL_TRIM_B2: Pid = Pid(0x08);
    pub const LONG_FUEL_TRIM_B2: Pid = Pid(0x09);
    pub const FUEL_PRESSURE: Pid = Pid(0x0A);
    pub const INTAKE_MAP: Pid = Pid(0x0B);
    pub const ENGINE_RPM: Pid = Pid(0x0C);
    pub const VEHICLE_SPEED: Pid = Pid(0x0D);
    pub const TIMING_ADVANCE: Pid = Pid(0x0E);
    pub const INTAKE_AIR_TEMP: Pid = Pid(0x0F);
    pub const MAF: Pid = Pid(0x10);
    pub const THROTTLE_POSITION: Pid = Pid(0x11);

    // OBD standards
    pub const OBD_STANDARD: Pid = Pid(0x1C);
    pub const RUN_TIME: Pid = Pid(0x1F);

    // Supported PIDs bitmaps
    pub const SUPPORTED_PIDS_21_40: Pid = Pid(0x20);
    pub const DISTANCE_WITH_MIL: Pid = Pid(0x21);
    pub const FUEL_RAIL_GAUGE_PRESSURE: Pid = Pid(0x23);
    pub const COMMANDED_EGR: Pid = Pid(0x2C);
    pub const EGR_ERROR: Pid = Pid(0x2D);
    pub const COMMANDED_EVAP_PURGE: Pid = Pid(0x2E);
    pub const FUEL_TANK_LEVEL: Pid = Pid(0x2F);
    pub const WARMUPS_SINCE_CLEAR: Pid = Pid(0x30);
    pub const DISTANCE_SINCE_CLEAR: Pid = Pid(0x31);
    pub const BAROMETRIC_PRESSURE: Pid = Pid(0x33);

    // Catalysts
    pub const CATALYST_TEMP_B1S1: Pid = Pid(0x3C);
    pub const CATALYST_TEMP_B2S1: Pid = Pid(0x3D);
    pub const CATALYST_TEMP_B1S2: Pid = Pid(0x3E);
    pub const CATALYST_TEMP_B2S2: Pid = Pid(0x3F);

    // Supported PIDs bitmap
    pub const SUPPORTED_PIDS_41_60: Pid = Pid(0x40);
    pub const CONTROL_MODULE_VOLTAGE: Pid = Pid(0x42);
    pub const ABSOLUTE_LOAD: Pid = Pid(0x43);
    pub const COMMANDED_EQUIV_RATIO: Pid = Pid(0x44);
    pub const RELATIVE_THROTTLE_POS: Pid = Pid(0x45);
    pub const AMBIENT_AIR_TEMP: Pid = Pid(0x46);
    pub const ABS_THROTTLE_POS_B: Pid = Pid(0x47);
    pub const ACCEL_PEDAL_POS_D: Pid = Pid(0x49);
    pub const ACCEL_PEDAL_POS_E: Pid = Pid(0x4A);
    pub const COMMANDED_THROTTLE_ACTUATOR: Pid = Pid(0x4C);

    // Engine oil and fuel
    pub const ENGINE_OIL_TEMP: Pid = Pid(0x5C);
    pub const ENGINE_FUEL_RATE: Pid = Pid(0x5E);
    pub const FUEL_RAIL_ABS_PRESSURE: Pid = Pid(0x59);

    // Supported PIDs bitmap
    pub const SUPPORTED_PIDS_61_80: Pid = Pid(0x60);
    pub const DEMANDED_TORQUE: Pid = Pid(0x61);
    pub const ACTUAL_TORQUE: Pid = Pid(0x62);
    pub const REFERENCE_TORQUE: Pid = Pid(0x63);

    /// Human-readable name for this PID.
    pub fn name(&self) -> &'static str {
        match self.0 {
            0x00 => "Supported PIDs [01-20]",
            0x01 => "Monitor Status",
            0x03 => "Fuel System Status",
            0x04 => "Engine Load",
            0x05 => "Coolant Temperature",
            0x06 => "Short Term Fuel Trim (Bank 1)",
            0x07 => "Long Term Fuel Trim (Bank 1)",
            0x08 => "Short Term Fuel Trim (Bank 2)",
            0x09 => "Long Term Fuel Trim (Bank 2)",
            0x0A => "Fuel Pressure",
            0x0B => "Intake MAP",
            0x0C => "Engine RPM",
            0x0D => "Vehicle Speed",
            0x0E => "Timing Advance",
            0x0F => "Intake Air Temperature",
            0x10 => "MAF Air Flow Rate",
            0x11 => "Throttle Position",
            0x1C => "OBD Standard",
            0x1F => "Run Time Since Start",
            0x20 => "Supported PIDs [21-40]",
            0x21 => "Distance with MIL On",
            0x23 => "Fuel Rail Gauge Pressure",
            0x2C => "Commanded EGR",
            0x2D => "EGR Error",
            0x2E => "Commanded Evaporative Purge",
            0x2F => "Fuel Tank Level",
            0x30 => "Warm-ups Since Clear",
            0x31 => "Distance Since DTC Clear",
            0x33 => "Barometric Pressure",
            0x3C => "Catalyst Temp B1S1",
            0x3D => "Catalyst Temp B2S1",
            0x3E => "Catalyst Temp B1S2",
            0x3F => "Catalyst Temp B2S2",
            0x40 => "Supported PIDs [41-60]",
            0x42 => "Control Module Voltage",
            0x43 => "Absolute Load",
            0x44 => "Commanded Equivalence Ratio",
            0x45 => "Relative Throttle Position",
            0x46 => "Ambient Air Temperature",
            0x47 => "Absolute Throttle Position B",
            0x49 => "Accelerator Pedal Position D",
            0x4A => "Accelerator Pedal Position E",
            0x4C => "Commanded Throttle Actuator",
            0x59 => "Fuel Rail Absolute Pressure",
            0x5C => "Engine Oil Temperature",
            0x5E => "Engine Fuel Rate",
            0x60 => "Supported PIDs [61-80]",
            0x61 => "Demanded Torque",
            0x62 => "Actual Torque",
            0x63 => "Reference Torque",
            _ => "Unknown PID",
        }
    }

    /// Measurement unit for this PID.
    pub fn unit(&self) -> &'static str {
        match self.0 {
            0x00 | 0x01 | 0x03 | 0x20 | 0x40 | 0x60 => "bitfield",
            0x04
            | 0x06..=0x09
            | 0x11
            | 0x2C
            | 0x2D
            | 0x2E
            | 0x2F
            | 0x43
            | 0x45
            | 0x47
            | 0x49
            | 0x4A
            | 0x4C
            | 0x61
            | 0x62 => "%",
            0x05 | 0x0F | 0x3C..=0x3F | 0x46 | 0x5C => "\u{00B0}C",
            0x0A | 0x0B | 0x23 | 0x33 | 0x59 => "kPa",
            0x0C => "RPM",
            0x0D => "km/h",
            0x0E => "\u{00B0}",
            0x10 => "g/s",
            0x1F => "s",
            0x21 | 0x31 => "km",
            0x30 => "count",
            0x42 => "V",
            0x44 => "\u{03BB}",
            0x5E => "L/h",
            0x63 => "Nm",
            _ => "",
        }
    }

    /// Number of response data bytes expected for this PID.
    pub fn response_bytes(&self) -> u8 {
        match self.0 {
            0x00 | 0x01 | 0x20 | 0x40 | 0x60 => 4, // bitmaps
            0x0C
            | 0x10
            | 0x1F
            | 0x21
            | 0x23
            | 0x31
            | 0x3C..=0x3F
            | 0x42
            | 0x43
            | 0x44
            | 0x59
            | 0x5E
            | 0x63 => 2,
            _ => 1, // most single-byte PIDs
        }
    }

    /// The type of value this PID returns.
    pub fn value_type(&self) -> ValueType {
        match self.0 {
            0x00 | 0x01 | 0x03 | 0x20 | 0x40 | 0x60 => ValueType::Bitfield,
            0x1C => ValueType::State,
            _ => ValueType::Scalar,
        }
    }

    /// Returns a slice of all known standard PIDs.
    pub fn all() -> &'static [Pid] {
        &[
            Self::SUPPORTED_PIDS_01_20,
            Self::MONITOR_STATUS,
            Self::FUEL_SYSTEM_STATUS,
            Self::ENGINE_LOAD,
            Self::COOLANT_TEMP,
            Self::SHORT_FUEL_TRIM_B1,
            Self::LONG_FUEL_TRIM_B1,
            Self::SHORT_FUEL_TRIM_B2,
            Self::LONG_FUEL_TRIM_B2,
            Self::FUEL_PRESSURE,
            Self::INTAKE_MAP,
            Self::ENGINE_RPM,
            Self::VEHICLE_SPEED,
            Self::TIMING_ADVANCE,
            Self::INTAKE_AIR_TEMP,
            Self::MAF,
            Self::THROTTLE_POSITION,
            Self::OBD_STANDARD,
            Self::RUN_TIME,
            Self::SUPPORTED_PIDS_21_40,
            Self::DISTANCE_WITH_MIL,
            Self::FUEL_RAIL_GAUGE_PRESSURE,
            Self::COMMANDED_EGR,
            Self::EGR_ERROR,
            Self::COMMANDED_EVAP_PURGE,
            Self::FUEL_TANK_LEVEL,
            Self::WARMUPS_SINCE_CLEAR,
            Self::DISTANCE_SINCE_CLEAR,
            Self::BAROMETRIC_PRESSURE,
            Self::CATALYST_TEMP_B1S1,
            Self::CATALYST_TEMP_B2S1,
            Self::CATALYST_TEMP_B1S2,
            Self::CATALYST_TEMP_B2S2,
            Self::SUPPORTED_PIDS_41_60,
            Self::CONTROL_MODULE_VOLTAGE,
            Self::ABSOLUTE_LOAD,
            Self::COMMANDED_EQUIV_RATIO,
            Self::RELATIVE_THROTTLE_POS,
            Self::AMBIENT_AIR_TEMP,
            Self::ABS_THROTTLE_POS_B,
            Self::ACCEL_PEDAL_POS_D,
            Self::ACCEL_PEDAL_POS_E,
            Self::COMMANDED_THROTTLE_ACTUATOR,
            Self::FUEL_RAIL_ABS_PRESSURE,
            Self::ENGINE_OIL_TEMP,
            Self::ENGINE_FUEL_RATE,
            Self::SUPPORTED_PIDS_61_80,
            Self::DEMANDED_TORQUE,
            Self::ACTUAL_TORQUE,
            Self::REFERENCE_TORQUE,
        ]
    }

    /// Convert a raw byte code to a Pid.
    pub fn from_code(code: u8) -> Pid {
        Pid(code)
    }

    /// Parse raw response bytes into a decoded Value.
    /// Bytes should NOT include the service ID or PID echo byte —
    /// just the data bytes (A, B, C, D).
    pub fn parse(&self, data: &[u8]) -> Result<Value, Obd2Error> {
        // Check byte count
        let expected = self.response_bytes() as usize;
        if data.len() < expected {
            return Err(Obd2Error::ParseError(format!(
                "PID {:#04X} expects {} bytes, got {}",
                self.0,
                expected,
                data.len()
            )));
        }

        let a = data[0] as f64;
        let b = if data.len() > 1 { data[1] as f64 } else { 0.0 };
        let _c = if data.len() > 2 { data[2] as f64 } else { 0.0 };
        let _d = if data.len() > 3 { data[3] as f64 } else { 0.0 };

        match self.0 {
            // Bitmaps (4 bytes)
            0x00 | 0x20 | 0x40 | 0x60 => {
                let raw = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
                Ok(Value::Bitfield(Bitfield { raw, flags: vec![] }))
            }

            // Monitor Status (special bitfield)
            0x01 => {
                let raw = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
                let mil_on = (data[0] & 0x80) != 0;
                let dtc_count = data[0] & 0x7F;
                let flags = vec![
                    ("MIL".into(), mil_on),
                    (format!("{} DTCs", dtc_count), dtc_count > 0),
                    ("Compression Ignition".into(), (data[1] & 0x08) != 0),
                ];
                Ok(Value::Bitfield(Bitfield { raw, flags }))
            }

            // Fuel system status
            0x03 => {
                let raw = u32::from(data[0]);
                Ok(Value::Bitfield(Bitfield { raw, flags: vec![] }))
            }

            // Percentage: A * 100 / 255
            0x04 | 0x11 | 0x2C | 0x2E | 0x2F | 0x45 | 0x47 | 0x49 | 0x4A | 0x4C => {
                Ok(Value::Scalar(a * 100.0 / 255.0))
            }

            // Temperature: A - 40
            0x05 | 0x0F | 0x46 | 0x5C => Ok(Value::Scalar(a - 40.0)),

            // Fuel trim: (A - 128) * 100 / 128
            0x06 | 0x07 | 0x08 | 0x09 | 0x2D => Ok(Value::Scalar((a - 128.0) * 100.0 / 128.0)),

            // Fuel pressure: A * 3
            0x0A => Ok(Value::Scalar(a * 3.0)),

            // Intake MAP: A (direct kPa)
            0x0B | 0x33 => Ok(Value::Scalar(a)),

            // RPM: (256*A + B) / 4
            0x0C => Ok(Value::Scalar((256.0 * a + b) / 4.0)),

            // Speed: A (direct km/h)
            0x0D => Ok(Value::Scalar(a)),

            // Timing advance: A/2 - 64
            0x0E => Ok(Value::Scalar(a / 2.0 - 64.0)),

            // MAF: (256*A + B) / 100
            0x10 => Ok(Value::Scalar((256.0 * a + b) / 100.0)),

            // OBD Standard (state)
            0x1C => Ok(Value::State(format!("Standard {}", data[0]))),

            // Run time / distance: 256*A + B (seconds or km)
            0x1F | 0x21 | 0x31 => Ok(Value::Scalar(256.0 * a + b)),

            // Fuel rail gauge pressure: (256*A + B) * 10
            0x23 => Ok(Value::Scalar((256.0 * a + b) * 10.0)),

            // Warm-ups since clear: A (direct count)
            0x30 => Ok(Value::Scalar(a)),

            // Catalyst temps: (256*A + B) / 10 - 40
            0x3C..=0x3F => Ok(Value::Scalar((256.0 * a + b) / 10.0 - 40.0)),

            // Control module voltage: (256*A + B) / 1000
            0x42 => Ok(Value::Scalar((256.0 * a + b) / 1000.0)),

            // Absolute load: (256*A + B) * 100 / 255
            0x43 => Ok(Value::Scalar((256.0 * a + b) * 100.0 / 255.0)),

            // Commanded equivalence ratio: (256*A + B) / 32768
            0x44 => Ok(Value::Scalar((256.0 * a + b) / 32768.0)),

            // Fuel rail absolute pressure: (256*A + B) * 10
            0x59 => Ok(Value::Scalar((256.0 * a + b) * 10.0)),

            // Engine fuel rate: (256*A + B) / 20
            0x5E => Ok(Value::Scalar((256.0 * a + b) / 20.0)),

            // Torque percentage: A - 125
            0x61 | 0x62 => Ok(Value::Scalar(a - 125.0)),

            // Reference torque: 256*A + B (Nm)
            0x63 => Ok(Value::Scalar(256.0 * a + b)),

            _ => Err(Obd2Error::ParseError(format!(
                "no parse formula for PID {:#04X}",
                self.0
            ))),
        }
    }
}

impl std::fmt::Display for Pid {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} ({:#04X})", self.name(), self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pid_constants() {
        assert_eq!(Pid::ENGINE_RPM.0, 0x0C);
        assert_eq!(Pid::COOLANT_TEMP.0, 0x05);
        assert_eq!(Pid::ENGINE_OIL_TEMP.0, 0x5C);
    }

    #[test]
    fn test_pid_names() {
        assert_eq!(Pid::ENGINE_RPM.name(), "Engine RPM");
        assert_eq!(Pid::COOLANT_TEMP.name(), "Coolant Temperature");
    }

    #[test]
    fn test_pid_units() {
        assert_eq!(Pid::ENGINE_RPM.unit(), "RPM");
        assert_eq!(Pid::COOLANT_TEMP.unit(), "\u{00B0}C");
        assert_eq!(Pid::VEHICLE_SPEED.unit(), "km/h");
    }

    #[test]
    fn test_pid_response_bytes() {
        assert_eq!(Pid::ENGINE_RPM.response_bytes(), 2);
        assert_eq!(Pid::COOLANT_TEMP.response_bytes(), 1);
        assert_eq!(Pid::MONITOR_STATUS.response_bytes(), 4);
    }

    #[test]
    fn test_pid_value_types() {
        assert_eq!(Pid::ENGINE_RPM.value_type(), ValueType::Scalar);
        assert_eq!(Pid::MONITOR_STATUS.value_type(), ValueType::Bitfield);
    }

    #[test]
    fn test_all_pids_have_names() {
        for pid in Pid::all() {
            assert_ne!(pid.name(), "Unknown PID", "PID {:#04x} has no name", pid.0);
        }
    }

    #[test]
    fn test_pid_display() {
        let s = format!("{}", Pid::ENGINE_RPM);
        assert!(s.contains("Engine RPM"));
        assert!(s.contains("0x0C"));
    }

    #[test]
    fn test_parse_rpm() {
        let data = [0x0C, 0x00]; // (0x0C * 256 + 0) / 4 = 768 RPM
        let val = Pid::ENGINE_RPM.parse(&data).unwrap();
        assert_eq!(val.as_f64().unwrap(), 768.0);
    }

    #[test]
    fn test_parse_rpm_idle() {
        let data = [0x0A, 0xA0]; // (10 * 256 + 160) / 4 = 680 RPM
        let val = Pid::ENGINE_RPM.parse(&data).unwrap();
        assert_eq!(val.as_f64().unwrap(), 680.0);
    }

    #[test]
    fn test_parse_speed() {
        let data = [0x3C]; // 60 km/h
        let val = Pid::VEHICLE_SPEED.parse(&data).unwrap();
        assert_eq!(val.as_f64().unwrap(), 60.0);
    }

    #[test]
    fn test_parse_coolant_temp() {
        let data = [0x7E]; // 126 - 40 = 86°C
        let val = Pid::COOLANT_TEMP.parse(&data).unwrap();
        assert_eq!(val.as_f64().unwrap(), 86.0);
    }

    #[test]
    fn test_parse_coolant_temp_freezing() {
        let data = [0x00]; // 0 - 40 = -40°C
        let val = Pid::COOLANT_TEMP.parse(&data).unwrap();
        assert_eq!(val.as_f64().unwrap(), -40.0);
    }

    #[test]
    fn test_parse_fuel_trim_zero() {
        let data = [0x80]; // (128 - 128) * 100 / 128 = 0%
        let val = Pid::SHORT_FUEL_TRIM_B1.parse(&data).unwrap();
        assert!((val.as_f64().unwrap()).abs() < 0.01);
    }

    #[test]
    fn test_parse_fuel_trim_rich() {
        let data = [0x90]; // (144 - 128) * 100 / 128 = 12.5%
        let val = Pid::SHORT_FUEL_TRIM_B1.parse(&data).unwrap();
        assert!((val.as_f64().unwrap() - 12.5).abs() < 0.01);
    }

    #[test]
    fn test_parse_control_module_voltage() {
        let data = [0x38, 0x5C]; // 14428 / 1000 = 14.428V
        let val = Pid(0x42).parse(&data).unwrap();
        assert!((val.as_f64().unwrap() - 14.428).abs() < 0.001);
    }

    #[test]
    fn test_parse_maf() {
        let data = [0x01, 0xF4]; // (256 + 244) / 100 = 5.00 g/s
        let val = Pid::MAF.parse(&data).unwrap();
        assert!((val.as_f64().unwrap() - 5.0).abs() < 0.01);
    }

    #[test]
    fn test_parse_catalyst_temp() {
        let data = [0x11, 0x0E]; // (17*256 + 14) / 10 - 40 = 4366/10 - 40 = 396.6°C
        let val = Pid::CATALYST_TEMP_B1S1.parse(&data).unwrap();
        assert!((val.as_f64().unwrap() - 396.6).abs() < 0.1);
    }

    #[test]
    fn test_parse_fuel_rate() {
        let data = [0x00, 0xC8]; // 200 / 20 = 10.0 L/h
        let val = Pid::ENGINE_FUEL_RATE.parse(&data).unwrap();
        assert!((val.as_f64().unwrap() - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_parse_monitor_status_bitfield() {
        let data = [0x00, 0x07, 0x65, 0x00];
        let val = Pid::MONITOR_STATUS.parse(&data).unwrap();
        let bf = val.as_bitfield().unwrap();
        assert!(!bf.flags.iter().find(|(n, _)| n == "MIL").unwrap().1); // MIL off
    }

    #[test]
    fn test_parse_insufficient_bytes() {
        let data = [0x0C]; // RPM needs 2 bytes
        let result = Pid::ENGINE_RPM.parse(&data);
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_timing_advance() {
        let data = [0x8C]; // 140/2 - 64 = 6°
        let val = Pid::TIMING_ADVANCE.parse(&data).unwrap();
        assert!((val.as_f64().unwrap() - 6.0).abs() < 0.01);
    }

    #[test]
    fn test_parse_torque_percent() {
        let data = [0xAF]; // 175 - 125 = 50%
        let val = Pid::ACTUAL_TORQUE.parse(&data).unwrap();
        assert_eq!(val.as_f64().unwrap(), 50.0);
    }

    #[test]
    fn test_parse_reference_torque() {
        let data = [0x03, 0x7F]; // 256*3 + 127 = 895 Nm
        let val = Pid::REFERENCE_TORQUE.parse(&data).unwrap();
        assert_eq!(val.as_f64().unwrap(), 895.0);
    }
}
