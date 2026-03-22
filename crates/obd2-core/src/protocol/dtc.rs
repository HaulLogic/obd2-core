//! DTC types and status definitions.

/// A Diagnostic Trouble Code read from the vehicle.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Dtc {
    pub code: String,
    pub category: DtcCategory,
    pub status: DtcStatus,
    pub description: Option<String>,
    pub severity: Option<Severity>,
    pub source_module: Option<String>,
    pub notes: Option<String>,
}

impl Default for Dtc {
    fn default() -> Self {
        Self {
            code: String::new(),
            category: DtcCategory::Powertrain,
            status: DtcStatus::Stored,
            description: None,
            severity: None,
            source_module: None,
            notes: None,
        }
    }
}

impl Dtc {
    /// Decode a DTC from two raw bytes (Mode 03/07/0A response).
    /// Bits 15-14 = category, bits 13-12 = second char, bits 11-8 = third, 7-4 = fourth, 3-0 = fifth.
    pub fn from_bytes(a: u8, b: u8) -> Self {
        let category = match (a >> 6) & 0x03 {
            0 => DtcCategory::Powertrain,
            1 => DtcCategory::Chassis,
            2 => DtcCategory::Body,
            _ => DtcCategory::Network,
        };
        let prefix = match category {
            DtcCategory::Powertrain => 'P',
            DtcCategory::Chassis => 'C',
            DtcCategory::Body => 'B',
            DtcCategory::Network => 'U',
        };
        let d2 = (a >> 4) & 0x03;
        let d3 = a & 0x0F;
        let d4 = (b >> 4) & 0x0F;
        let d5 = b & 0x0F;
        let code = format!("{}{}{:X}{:X}{:X}", prefix, d2, d3, d4, d5);

        let description = universal_dtc_description(&code).map(String::from);

        Self {
            code,
            category,
            status: DtcStatus::Stored,
            description,
            severity: None,
            source_module: None,
            notes: None,
        }
    }

    /// Create a DTC from a code string (e.g., "P0420").
    pub fn from_code(code: &str) -> Self {
        let category = match code.chars().next() {
            Some('P') | Some('p') => DtcCategory::Powertrain,
            Some('C') | Some('c') => DtcCategory::Chassis,
            Some('B') | Some('b') => DtcCategory::Body,
            Some('U') | Some('u') => DtcCategory::Network,
            _ => DtcCategory::Powertrain,
        };
        let upper = code.to_uppercase();
        let description = universal_dtc_description(&upper).map(String::from);
        Self {
            code: upper,
            category,
            status: DtcStatus::Stored,
            description,
            severity: None,
            source_module: None,
            notes: None,
        }
    }
}

/// Category prefix of a DTC (P, C, B, U).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DtcCategory {
    Powertrain,
    Chassis,
    Body,
    Network,
}

/// Lifecycle status of a DTC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DtcStatus {
    Stored,
    Pending,
    Permanent,
}

/// Severity level of a DTC.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, serde::Deserialize)]
pub enum Severity {
    Info,
    Low,
    Medium,
    High,
    Critical,
}

/// GM/UDS Mode 19 extended status byte -- 8 flags per DTC.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DtcStatusByte {
    pub test_failed: bool,
    pub test_failed_this_cycle: bool,
    pub pending: bool,
    pub confirmed: bool,
    pub test_not_completed_since_clear: bool,
    pub test_failed_since_clear: bool,
    pub test_not_completed_this_cycle: bool,
    pub warning_indicator_requested: bool,
}

impl DtcStatusByte {
    /// Decode a Mode 19 DTC status byte into individual flags.
    pub fn from_byte(b: u8) -> Self {
        Self {
            test_failed: (b & 0x01) != 0,
            test_failed_this_cycle: (b & 0x02) != 0,
            pending: (b & 0x04) != 0,
            confirmed: (b & 0x08) != 0,
            test_not_completed_since_clear: (b & 0x10) != 0,
            test_failed_since_clear: (b & 0x20) != 0,
            test_not_completed_this_cycle: (b & 0x40) != 0,
            warning_indicator_requested: (b & 0x80) != 0,
        }
    }

    /// Encode the status flags back into a single byte.
    pub fn to_byte(&self) -> u8 {
        let mut b = 0u8;
        if self.test_failed {
            b |= 0x01;
        }
        if self.test_failed_this_cycle {
            b |= 0x02;
        }
        if self.pending {
            b |= 0x04;
        }
        if self.confirmed {
            b |= 0x08;
        }
        if self.test_not_completed_since_clear {
            b |= 0x10;
        }
        if self.test_failed_since_clear {
            b |= 0x20;
        }
        if self.test_not_completed_this_cycle {
            b |= 0x40;
        }
        if self.warning_indicator_requested {
            b |= 0x80;
        }
        b
    }
}

/// Look up the universal (SAE J2012) description for a DTC code.
///
/// Covers ~200 common OBD-II codes across powertrain, chassis, body, and network categories.
pub fn universal_dtc_description(code: &str) -> Option<&'static str> {
    match code {
        // VVT / Camshaft
        "P0010" => Some("Intake Camshaft Position Actuator Circuit (Bank 1)"),
        "P0011" => Some("Intake Camshaft Position Timing Over-Advanced (Bank 1)"),
        "P0012" => Some("Intake Camshaft Position Timing Over-Retarded (Bank 1)"),
        "P0014" => Some("Exhaust Camshaft Position Timing Over-Advanced (Bank 1)"),
        "P0016" => Some("Crankshaft Position / Camshaft Position Correlation (Bank 1 Sensor A)"),
        // O2 Sensor Heaters
        "P0030" => Some("HO2S Heater Control Circuit (Bank 1 Sensor 1)"),
        "P0036" => Some("HO2S Heater Control Circuit (Bank 1 Sensor 2)"),
        // Diesel / Fuel Rail
        "P0087" => Some("Fuel Rail/System Pressure Too Low"),
        "P0088" => Some("Fuel Rail/System Pressure Too High"),
        "P0093" => Some("Fuel System Leak Detected (Large)"),
        // Fuel & Air Metering
        "P0100" => Some("MAF/VAF Circuit Malfunction"),
        "P0101" => Some("MAF/VAF Circuit Range/Performance"),
        "P0102" => Some("MAF/VAF Circuit Low Input"),
        "P0103" => Some("MAF/VAF Circuit High Input"),
        "P0106" => Some("MAP/Barometric Pressure Circuit Range/Performance"),
        "P0107" => Some("MAP/Barometric Pressure Circuit Low Input"),
        "P0108" => Some("MAP/Barometric Pressure Circuit High Input"),
        "P0110" => Some("Intake Air Temperature Sensor 1 Circuit"),
        "P0111" => Some("Intake Air Temperature Sensor 1 Circuit Range/Performance"),
        "P0112" => Some("Intake Air Temperature Sensor 1 Circuit Low"),
        "P0113" => Some("Intake Air Temperature Sensor 1 Circuit High"),
        "P0115" => Some("Engine Coolant Temperature Circuit"),
        "P0116" => Some("Engine Coolant Temperature Circuit Range/Performance"),
        "P0117" => Some("Engine Coolant Temperature Circuit Low"),
        "P0118" => Some("Engine Coolant Temperature Circuit High"),
        "P0120" => Some("Throttle Position Sensor A Circuit"),
        "P0121" => Some("Throttle Position Sensor A Circuit Range/Performance"),
        "P0122" => Some("Throttle Position Sensor A Circuit Low"),
        "P0123" => Some("Throttle Position Sensor A Circuit High"),
        "P0125" => Some("Insufficient Coolant Temperature for Closed Loop"),
        "P0128" => Some("Coolant Thermostat Below Operating Temperature"),
        "P0130" => Some("O2 Sensor Circuit Bank 1 Sensor 1"),
        "P0131" => Some("O2 Sensor Circuit Low Voltage B1S1"),
        "P0132" => Some("O2 Sensor Circuit High Voltage B1S1"),
        "P0133" => Some("O2 Sensor Circuit Slow Response B1S1"),
        "P0134" => Some("O2 Sensor Circuit No Activity B1S1"),
        "P0135" => Some("O2 Sensor Heater Circuit B1S1"),
        "P0136" => Some("O2 Sensor Circuit Bank 1 Sensor 2"),
        "P0137" => Some("O2 Sensor Circuit Low Voltage B1S2"),
        "P0138" => Some("O2 Sensor Circuit High Voltage B1S2"),
        "P0139" => Some("O2 Sensor Circuit Slow Response B1S2"),
        "P0140" => Some("O2 Sensor Circuit No Activity B1S2"),
        "P0141" => Some("O2 Sensor Heater Circuit B1S2"),
        "P0150" => Some("O2 Sensor Circuit Bank 2 Sensor 1"),
        "P0151" => Some("O2 Sensor Circuit Low Voltage B2S1"),
        "P0152" => Some("O2 Sensor Circuit High Voltage B2S1"),
        "P0153" => Some("O2 Sensor Circuit Slow Response B2S1"),
        "P0154" => Some("O2 Sensor Circuit No Activity B2S1"),
        "P0155" => Some("O2 Sensor Heater Circuit B2S1"),
        "P0156" => Some("O2 Sensor Circuit Bank 2 Sensor 2"),
        "P0157" => Some("O2 Sensor Circuit Low Voltage B2S2"),
        "P0158" => Some("O2 Sensor Circuit High Voltage B2S2"),
        "P0159" => Some("O2 Sensor Circuit Slow Response B2S2"),
        "P0160" => Some("O2 Sensor Circuit No Activity B2S2"),
        "P0161" => Some("O2 Sensor Heater Circuit B2S2"),
        "P0171" => Some("System Too Lean Bank 1"),
        "P0172" => Some("System Too Rich Bank 1"),
        "P0174" => Some("System Too Lean Bank 2"),
        "P0175" => Some("System Too Rich Bank 2"),
        // Fuel Rail Pressure Sensor
        "P0192" => Some("Fuel Rail Pressure Sensor Circuit Low"),
        "P0193" => Some("Fuel Rail Pressure Sensor Circuit High"),
        // Injector Circuits
        "P0201" => Some("Injector Circuit/Open Cylinder 1"),
        "P0202" => Some("Injector Circuit/Open Cylinder 2"),
        "P0203" => Some("Injector Circuit/Open Cylinder 3"),
        "P0204" => Some("Injector Circuit/Open Cylinder 4"),
        "P0205" => Some("Injector Circuit/Open Cylinder 5"),
        "P0206" => Some("Injector Circuit/Open Cylinder 6"),
        "P0207" => Some("Injector Circuit/Open Cylinder 7"),
        "P0208" => Some("Injector Circuit/Open Cylinder 8"),
        // Turbo / Boost
        "P0234" => Some("Turbo/Supercharger Overboost Condition"),
        "P0236" => Some("Turbocharger Boost Sensor A Circuit Range/Performance"),
        "P0299" => Some("Turbo/Supercharger Underboost"),
        // Ignition / Misfire
        "P0300" => Some("Random/Multiple Cylinder Misfire Detected"),
        "P0301" => Some("Cylinder 1 Misfire Detected"),
        "P0302" => Some("Cylinder 2 Misfire Detected"),
        "P0303" => Some("Cylinder 3 Misfire Detected"),
        "P0304" => Some("Cylinder 4 Misfire Detected"),
        "P0305" => Some("Cylinder 5 Misfire Detected"),
        "P0306" => Some("Cylinder 6 Misfire Detected"),
        "P0307" => Some("Cylinder 7 Misfire Detected"),
        "P0308" => Some("Cylinder 8 Misfire Detected"),
        "P0325" => Some("Knock Sensor 1 Circuit Bank 1"),
        "P0335" => Some("Crankshaft Position Sensor A Circuit"),
        "P0336" => Some("Crankshaft Position Sensor A Range/Performance"),
        "P0340" => Some("Camshaft Position Sensor A Circuit Bank 1"),
        "P0341" => Some("Camshaft Position Sensor A Range/Performance Bank 1"),
        // Camshaft Position Sensors
        "P0365" => Some("Camshaft Position Sensor B Circuit (Bank 1)"),
        "P0366" => Some("Camshaft Position Sensor B Circuit Range/Performance (Bank 1)"),
        // Glow Plug
        "P0380" => Some("Glow Plug/Heater Circuit A Malfunction"),
        // Emission Controls
        "P0400" => Some("EGR Flow Malfunction"),
        "P0401" => Some("EGR Flow Insufficient"),
        "P0402" => Some("EGR Flow Excessive"),
        "P0404" => Some("EGR System Range/Performance"),
        "P0405" => Some("EGR Position Sensor A Circuit Low"),
        "P0406" => Some("EGR Position Sensor A Circuit High"),
        "P0420" => Some("Catalyst System Efficiency Below Threshold Bank 1"),
        "P0430" => Some("Catalyst System Efficiency Below Threshold Bank 2"),
        "P0440" => Some("Evaporative Emission System Malfunction"),
        "P0441" => Some("Evaporative Emission System Incorrect Purge Flow"),
        "P0442" => Some("Evaporative Emission System Leak Detected (Small)"),
        "P0443" => Some("Evaporative Emission System Purge Control Valve Circuit"),
        "P0446" => Some("Evaporative Emission System Vent Control Circuit"),
        "P0449" => Some("Evaporative Emission System Vent Valve/Solenoid Circuit"),
        "P0451" => Some("Evaporative Emission System Pressure Sensor Range/Performance"),
        "P0455" => Some("Evaporative Emission System Leak Detected (Large)"),
        "P0456" => Some("Evaporative Emission System Leak Detected (Very Small)"),
        "P0496" => Some("Evaporative Emission System High Purge Flow"),
        // Speed / Idle
        "P0500" => Some("Vehicle Speed Sensor A Malfunction"),
        "P0503" => Some("Vehicle Speed Sensor Intermittent/Erratic/High"),
        "P0505" => Some("Idle Air Control System Malfunction"),
        "P0506" => Some("Idle Air Control System RPM Lower Than Expected"),
        "P0507" => Some("Idle Air Control System RPM Higher Than Expected"),
        // Electrical
        "P0562" => Some("System Voltage Low"),
        "P0563" => Some("System Voltage High"),
        // Thermostat
        "P0597" => Some("Thermostat Heater Control Circuit Open"),
        "P0598" => Some("Thermostat Heater Control Circuit Low"),
        "P0599" => Some("Thermostat Heater Control Circuit High"),
        // ECM
        "P0600" => Some("Serial Communication Link Malfunction"),
        "P0601" => Some("Internal Control Module Memory Check Sum Error"),
        "P0602" => Some("Control Module Programming Error"),
        "P0606" => Some("ECM/PCM Processor Fault"),
        // Transmission
        "P0700" => Some("Transmission Control System Malfunction"),
        "P0705" => Some("Transmission Range Sensor Circuit Malfunction"),
        "P0710" => Some("Transmission Fluid Temperature Sensor Circuit"),
        "P0711" => Some("Transmission Fluid Temperature Sensor Circuit Range/Performance"),
        "P0715" => Some("Input/Turbine Speed Sensor Circuit"),
        "P0717" => Some("Input/Turbine Speed Sensor Circuit No Signal"),
        "P0720" => Some("Output Speed Sensor Circuit"),
        "P0725" => Some("Engine Speed Input Circuit"),
        "P0730" => Some("Incorrect Gear Ratio"),
        "P0731" => Some("Gear 1 Incorrect Ratio"),
        "P0732" => Some("Gear 2 Incorrect Ratio"),
        "P0733" => Some("Gear 3 Incorrect Ratio"),
        "P0734" => Some("Gear 4 Incorrect Ratio"),
        "P0735" => Some("Gear 5 Incorrect Ratio"),
        "P0740" => Some("Torque Converter Clutch Circuit Malfunction"),
        "P0741" => Some("Torque Converter Clutch System Stuck Off"),
        "P0742" => Some("Torque Converter Clutch System Stuck On"),
        "P0743" => Some("Torque Converter Clutch System Electrical"),
        "P0747" => Some("Pressure Control Solenoid A Stuck On"),
        "P0748" => Some("Pressure Control Solenoid A Electrical"),
        "P0750" => Some("Shift Solenoid A Malfunction"),
        "P0751" => Some("Shift Solenoid A Performance/Stuck Off"),
        "P0752" => Some("Shift Solenoid A Stuck On"),
        "P0753" => Some("Shift Solenoid A Electrical"),
        "P0755" => Some("Shift Solenoid B Malfunction"),
        "P0756" => Some("Shift Solenoid B Performance/Stuck Off"),
        "P0757" => Some("Shift Solenoid B Stuck On"),
        "P0758" => Some("Shift Solenoid B Electrical"),
        // GM-Specific / Manufacturer
        "P1101" => Some("Intake Airflow System Performance"),
        "P2097" => Some("Post Catalyst Fuel Trim System Too Rich (Bank 1)"),
        "P2227" => Some("Barometric Pressure Circuit Range/Performance"),
        "P2270" => Some("O2 Sensor Signal Stuck Lean (Bank 1 Sensor 2)"),
        "P2271" => Some("O2 Sensor Signal Stuck Rich (Bank 1 Sensor 2)"),
        "P2797" => Some("Auxiliary Transmission Fluid Pump Performance"),
        // Body (B) Codes
        "B0083" => Some("Left Side/Front Impact Sensor Circuit"),
        "B0092" => Some("Left Side/Rear Impact Sensor Circuit"),
        "B0096" => Some("Right Side/Rear Impact Sensor Circuit"),
        "B0408" => Some("Temperature Control A Circuit"),
        "B1325" => Some("Control Module General Memory Failure"),
        "B1517" => Some("Steering Wheel Controls Switch 1 Circuit"),
        // Chassis (C) Codes
        "C0035" => Some("Left Front Wheel Speed Sensor Circuit"),
        "C0040" => Some("Right Front Wheel Speed Sensor Circuit"),
        "C0045" => Some("Left Rear Wheel Speed Sensor Circuit"),
        "C0050" => Some("Right Rear Wheel Speed Sensor Circuit"),
        "C0110" => Some("Pump Motor Circuit Malfunction"),
        "C0161" => Some("ABS/TCS Brake Switch Circuit Malfunction"),
        "C0186" => Some("Lateral Accelerometer Circuit"),
        "C0196" => Some("Yaw Rate Sensor Circuit"),
        "C0550" => Some("ECU Malfunction (Stability System)"),
        "C0899" => Some("Device Voltage Low"),
        "C0900" => Some("Device Voltage High"),
        // Network (U) Codes
        "U0001" => Some("High Speed CAN Communication Bus"),
        "U0073" => Some("Control Module Communication Bus Off"),
        "U0100" => Some("Lost Communication with ECM/PCM"),
        "U0101" => Some("Lost Communication with TCM"),
        "U0121" => Some("Lost Communication with ABS"),
        "U0140" => Some("Lost Communication with BCM"),
        "U0146" => Some("Lost Communication with Gateway A"),
        "U0151" => Some("Lost Communication with SIR Module"),
        "U0155" => Some("Lost Communication with Instrument Panel Cluster"),
        "U0168" => Some("Lost Communication with HVAC Control Module"),
        "U0184" => Some("Lost Communication with Radio"),
        "U0401" => Some("Invalid Data Received from ECM/PCM A"),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dtc_default() {
        let dtc = Dtc::default();
        assert_eq!(dtc.category, DtcCategory::Powertrain);
        assert_eq!(dtc.status, DtcStatus::Stored);
        assert!(dtc.description.is_none());
    }

    #[test]
    fn test_severity_ordering() {
        assert!(Severity::Critical > Severity::High);
        assert!(Severity::High > Severity::Medium);
        assert!(Severity::Medium > Severity::Low);
        assert!(Severity::Low > Severity::Info);
    }

    #[test]
    fn test_dtc_status_byte_default() {
        let status = DtcStatusByte::default();
        assert!(!status.test_failed);
        assert!(!status.confirmed);
        assert!(!status.warning_indicator_requested);
    }

    #[test]
    fn test_dtc_from_bytes_powertrain() {
        let dtc = Dtc::from_bytes(0x04, 0x20);
        assert_eq!(dtc.code, "P0420");
        assert_eq!(dtc.category, DtcCategory::Powertrain);
        assert!(dtc.description.is_some());
        assert!(dtc.description.unwrap().contains("Catalyst"));
    }

    #[test]
    fn test_dtc_from_bytes_chassis() {
        let dtc = Dtc::from_bytes(0x40, 0x35);
        assert_eq!(dtc.code, "C0035");
        assert_eq!(dtc.category, DtcCategory::Chassis);
    }

    #[test]
    fn test_dtc_from_bytes_body() {
        let dtc = Dtc::from_bytes(0x80, 0x83);
        assert_eq!(dtc.code, "B0083");
        assert_eq!(dtc.category, DtcCategory::Body);
    }

    #[test]
    fn test_dtc_from_bytes_network() {
        let dtc = Dtc::from_bytes(0xC1, 0x00);
        assert_eq!(dtc.code, "U0100");
        assert_eq!(dtc.category, DtcCategory::Network);
    }

    #[test]
    fn test_dtc_from_code() {
        let dtc = Dtc::from_code("P0420");
        assert_eq!(dtc.code, "P0420");
        assert_eq!(dtc.category, DtcCategory::Powertrain);
        assert!(dtc.description.is_some());
    }

    #[test]
    fn test_dtc_from_code_lowercase() {
        let dtc = Dtc::from_code("p0171");
        assert_eq!(dtc.code, "P0171");
    }

    #[test]
    fn test_universal_description_known() {
        assert!(universal_dtc_description("P0420").is_some());
        assert!(universal_dtc_description("P0300").unwrap().contains("Misfire"));
    }

    #[test]
    fn test_universal_description_unknown() {
        assert!(universal_dtc_description("P9999").is_none());
    }

    #[test]
    fn test_status_byte_decode() {
        let status = DtcStatusByte::from_byte(0x0B); // bits 0,1,3
        assert!(status.test_failed);
        assert!(status.test_failed_this_cycle);
        assert!(!status.pending);
        assert!(status.confirmed);
    }

    #[test]
    fn test_status_byte_roundtrip() {
        let original: u8 = 0xAF;
        let status = DtcStatusByte::from_byte(original);
        assert_eq!(status.to_byte(), original);
    }

    #[test]
    fn test_status_byte_all_flags() {
        let status = DtcStatusByte::from_byte(0xFF);
        assert!(status.test_failed);
        assert!(status.test_failed_this_cycle);
        assert!(status.pending);
        assert!(status.confirmed);
        assert!(status.test_not_completed_since_clear);
        assert!(status.test_failed_since_clear);
        assert!(status.test_not_completed_this_cycle);
        assert!(status.warning_indicator_requested);
    }

    #[test]
    fn test_status_byte_mil_on() {
        let status = DtcStatusByte::from_byte(0x80);
        assert!(status.warning_indicator_requested); // MIL bit
        assert!(!status.test_failed);
    }
}
