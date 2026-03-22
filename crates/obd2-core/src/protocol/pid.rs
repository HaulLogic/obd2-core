//! Standard OBD-II PID definitions.

/// The type of value a PID returns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
            0x04 | 0x06..=0x09 | 0x11 | 0x2C | 0x2D | 0x2E | 0x2F
            | 0x43 | 0x45 | 0x47 | 0x49 | 0x4A | 0x4C | 0x61 | 0x62 => "%",
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
            0x0C | 0x10 | 0x1F | 0x21 | 0x23 | 0x31 | 0x3C..=0x3F
            | 0x42 | 0x43 | 0x44 | 0x59 | 0x5E | 0x63 => 2,
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
            Self::SUPPORTED_PIDS_01_20, Self::MONITOR_STATUS, Self::FUEL_SYSTEM_STATUS,
            Self::ENGINE_LOAD, Self::COOLANT_TEMP,
            Self::SHORT_FUEL_TRIM_B1, Self::LONG_FUEL_TRIM_B1,
            Self::SHORT_FUEL_TRIM_B2, Self::LONG_FUEL_TRIM_B2,
            Self::FUEL_PRESSURE, Self::INTAKE_MAP, Self::ENGINE_RPM,
            Self::VEHICLE_SPEED, Self::TIMING_ADVANCE, Self::INTAKE_AIR_TEMP,
            Self::MAF, Self::THROTTLE_POSITION, Self::OBD_STANDARD, Self::RUN_TIME,
            Self::SUPPORTED_PIDS_21_40, Self::DISTANCE_WITH_MIL,
            Self::FUEL_RAIL_GAUGE_PRESSURE, Self::COMMANDED_EGR, Self::EGR_ERROR,
            Self::COMMANDED_EVAP_PURGE, Self::FUEL_TANK_LEVEL, Self::WARMUPS_SINCE_CLEAR,
            Self::DISTANCE_SINCE_CLEAR, Self::BAROMETRIC_PRESSURE,
            Self::CATALYST_TEMP_B1S1, Self::CATALYST_TEMP_B2S1,
            Self::CATALYST_TEMP_B1S2, Self::CATALYST_TEMP_B2S2,
            Self::SUPPORTED_PIDS_41_60, Self::CONTROL_MODULE_VOLTAGE,
            Self::ABSOLUTE_LOAD, Self::COMMANDED_EQUIV_RATIO,
            Self::RELATIVE_THROTTLE_POS, Self::AMBIENT_AIR_TEMP,
            Self::ABS_THROTTLE_POS_B, Self::ACCEL_PEDAL_POS_D,
            Self::ACCEL_PEDAL_POS_E, Self::COMMANDED_THROTTLE_ACTUATOR,
            Self::FUEL_RAIL_ABS_PRESSURE, Self::ENGINE_OIL_TEMP,
            Self::ENGINE_FUEL_RATE, Self::SUPPORTED_PIDS_61_80,
            Self::DEMANDED_TORQUE, Self::ACTUAL_TORQUE, Self::REFERENCE_TORQUE,
        ]
    }

    /// Convert a raw byte code to a Pid.
    pub fn from_code(code: u8) -> Pid {
        Pid(code)
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
}
