//! J1939 heavy-duty vehicle protocol support.
//!
//! SAE J1939 is the standard for heavy-duty truck communication over CAN bus
//! (29-bit extended identifiers, 250 kbps). This module provides:
//!
//! - [`Pgn`] type with constants for common parameter groups
//! - Decoder functions for fleet-critical PGNs (engine, vehicle speed, temps)
//! - [`J1939Dtc`] type using SPN+FMI format (distinct from OBD-II P-codes)
//!
//! ## Usage with ELM327/STN adapters
//!
//! Most OBD-II adapters support J1939 via CAN 29-bit mode (`AT SP A` for
//! 29-bit 250 kbps). Use [`Session::read_j1939_pgn`] which handles the
//! CAN addressing internally via `raw_request()`.
//!
//! ## PGN request format
//!
//! A J1939 request message uses a 29-bit CAN ID:
//! ```text
//! Priority(3) | Reserved(1) | Data Page(1) | PDU Format(8) | PDU Specific(8) | Source Address(8)
//! ```
//! For destination-specific PGNs (PDU Format < 240), PDU Specific = destination address.
//! For broadcast PGNs (PDU Format >= 240), PDU Specific is part of the PGN.

/// A J1939 Parameter Group Number.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Pgn(pub u32);

impl Pgn {
    // ── Engine ──

    /// Electronic Engine Controller 1 — engine speed, torque.
    /// 8 bytes, broadcast, 100ms default rate.
    pub const EEC1: Pgn = Pgn(61444);

    /// Engine Temperature 1 — coolant temp, fuel temp.
    /// 8 bytes, broadcast, 1000ms default rate.
    pub const ET1: Pgn = Pgn(65262);

    /// Engine Fluid Level/Pressure 1 — oil pressure, coolant pressure, oil level.
    /// 8 bytes, broadcast, 500ms default rate.
    pub const EFLP1: Pgn = Pgn(65263);

    /// Fuel Economy (Liquid) — fuel rate, instantaneous fuel economy.
    /// 8 bytes, broadcast, 100ms default rate.
    pub const LFE: Pgn = Pgn(65266);

    // ── Vehicle ──

    /// Cruise Control/Vehicle Speed — vehicle speed, brake, cruise control.
    /// 8 bytes, broadcast, 100ms default rate.
    pub const CCVS: Pgn = Pgn(65265);

    // ── Diagnostics ──

    /// DM1 — Active Diagnostic Trouble Codes.
    /// Variable length, broadcast, 1000ms default rate.
    pub const DM1: Pgn = Pgn(65226);

    /// DM2 — Previously Active Diagnostic Trouble Codes.
    /// Variable length, on-request.
    pub const DM2: Pgn = Pgn(65227);

    /// Return the PGN name, if known.
    pub fn name(&self) -> &'static str {
        match self.0 {
            61444 => "EEC1 (Electronic Engine Controller 1)",
            65262 => "ET1 (Engine Temperature 1)",
            65263 => "EFLP1 (Engine Fluid Level/Pressure 1)",
            65265 => "CCVS (Cruise Control/Vehicle Speed)",
            65266 => "LFE (Fuel Economy - Liquid)",
            65226 => "DM1 (Active DTCs)",
            65227 => "DM2 (Previously Active DTCs)",
            _ => "Unknown PGN",
        }
    }
}

impl std::fmt::Display for Pgn {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "PGN {} ({})", self.0, self.name())
    }
}

// ── Decoded Parameter Groups ──

/// Decoded Electronic Engine Controller 1 (PGN 61444).
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF sentinel).
#[derive(Debug, Clone)]
pub struct Eec1 {
    /// Engine speed in RPM. SPN 190, bytes 4-5.
    pub engine_rpm: Option<f64>,
    /// Driver's demand engine torque as percent. SPN 512, byte 2.
    pub driver_demand_torque_pct: Option<f64>,
    /// Actual engine torque as percent. SPN 513, byte 3.
    pub actual_torque_pct: Option<f64>,
    /// Engine torque mode. SPN 899, byte 1 bits 0-3.
    pub torque_mode: u8,
}

/// Decoded Cruise Control/Vehicle Speed (PGN 65265).
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF sentinel).
#[derive(Debug, Clone)]
pub struct Ccvs {
    /// Vehicle speed in km/h. SPN 84, bytes 2-3.
    pub vehicle_speed: Option<f64>,
    /// Brake switch active. SPN 597, byte 4 bits 2-3. `None` if not available.
    pub brake_switch: Option<bool>,
    /// Cruise control active. SPN 595, byte 1 bits 0-1. `None` if not available.
    pub cruise_active: Option<bool>,
}

/// Decoded Engine Temperature 1 (PGN 65262).
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF sentinel).
#[derive(Debug, Clone)]
pub struct Et1 {
    /// Engine coolant temperature in °C. SPN 110, byte 1.
    pub coolant_temp: Option<f64>,
    /// Fuel temperature in °C. SPN 174, byte 2.
    pub fuel_temp: Option<f64>,
    /// Engine oil temperature in °C. SPN 175, bytes 3-4.
    pub oil_temp: Option<f64>,
}

/// Decoded Engine Fluid Level/Pressure 1 (PGN 65263).
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF sentinel).
#[derive(Debug, Clone)]
pub struct Eflp1 {
    /// Engine oil pressure in kPa. SPN 100, byte 4.
    pub oil_pressure: Option<f64>,
    /// Coolant pressure in kPa. SPN 109, byte 2.
    pub coolant_pressure: Option<f64>,
}

/// Decoded Fuel Economy - Liquid (PGN 65266).
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF sentinel).
#[derive(Debug, Clone)]
pub struct Lfe {
    /// Engine fuel rate in L/h. SPN 183, bytes 1-2.
    pub fuel_rate: Option<f64>,
    /// Instantaneous fuel economy in km/L. SPN 184, bytes 3-4.
    pub instantaneous_fuel_economy: Option<f64>,
}

/// A J1939 Diagnostic Trouble Code (SPN + FMI format).
///
/// Unlike OBD-II P-codes, J1939 uses Suspect Parameter Number (SPN) to identify
/// the faulting parameter and Failure Mode Identifier (FMI) to describe the
/// failure type.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct J1939Dtc {
    /// Suspect Parameter Number — identifies the parameter at fault.
    pub spn: u32,
    /// Failure Mode Identifier — describes the type of failure (0-31).
    pub fmi: u8,
    /// Occurrence count (0-126, 127 = not available).
    pub occurrence_count: u8,
    /// SPN Conversion Method (0 = standard, 1 = extended).
    pub conversion_method: u8,
}

impl J1939Dtc {
    /// Decode a J1939 DTC from the 4-byte DM1/DM2 format.
    ///
    /// Byte layout:
    /// - Bytes 0-1: SPN bits 0-15 (little-endian)
    /// - Byte 2 bits 5-7: SPN bits 16-18
    /// - Byte 2 bits 0-4: FMI
    /// - Byte 3 bit 7: SPN Conversion Method
    /// - Byte 3 bits 0-6: Occurrence Count
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 4 {
            return None;
        }
        let spn_low = u16::from_le_bytes([data[0], data[1]]) as u32;
        let spn_high = ((data[2] >> 5) & 0x07) as u32;
        let spn = spn_low | (spn_high << 16);
        let fmi = data[2] & 0x1F;
        let conversion_method = (data[3] >> 7) & 0x01;
        let occurrence_count = data[3] & 0x7F;

        Some(Self {
            spn,
            fmi,
            occurrence_count,
            conversion_method,
        })
    }

    /// Human-readable FMI description.
    pub fn fmi_description(&self) -> &'static str {
        match self.fmi {
            0 => "Data Valid But Above Normal Operational Range - Most Severe",
            1 => "Data Valid But Below Normal Operational Range - Most Severe",
            2 => "Data Erratic, Intermittent Or Incorrect",
            3 => "Voltage Above Normal, Or Shorted To High Source",
            4 => "Voltage Below Normal, Or Shorted To Low Source",
            5 => "Current Below Normal Or Open Circuit",
            6 => "Current Above Normal Or Grounded Circuit",
            7 => "Mechanical System Not Responding Or Out Of Adjustment",
            8 => "Abnormal Frequency Or Pulse Width Or Period",
            9 => "Abnormal Update Rate",
            10 => "Abnormal Rate Of Change",
            11 => "Root Cause Not Known",
            12 => "Bad Intelligent Device Or Component",
            13 => "Out Of Calibration",
            14 => "Special Instructions",
            15 => "Data Valid But Above Normal Operating Range - Least Severe",
            16 => "Data Valid But Above Normal Operating Range - Moderately Severe",
            17 => "Data Valid But Below Normal Operating Range - Least Severe",
            18 => "Data Valid But Below Normal Operating Range - Moderately Severe",
            19 => "Received Network Data In Error",
            20 => "Data Drifted High",
            21 => "Data Drifted Low",
            31 => "Condition Exists",
            _ => "Reserved",
        }
    }
}

impl std::fmt::Display for J1939Dtc {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "SPN {} FMI {} ({})",
            self.spn,
            self.fmi,
            self.fmi_description()
        )
    }
}

// ── PGN Decoders ──

// J1939 "not available" sentinels
const NA_BYTE: u8 = 0xFF;
const NA_WORD: u16 = 0xFFFF;

/// Convert a single-byte J1939 value, returning `None` if the byte is `0xFF` (not available).
fn byte_available(b: u8) -> Option<u8> {
    if b == NA_BYTE {
        None
    } else {
        Some(b)
    }
}

/// Convert a two-byte J1939 value, returning `None` if the word is `0xFFFF` (not available).
fn word_available(w: u16) -> Option<u16> {
    if w == NA_WORD {
        None
    } else {
        Some(w)
    }
}

/// Decode EEC1 (PGN 61444) from 8 raw bytes.
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF).
pub fn decode_eec1(data: &[u8]) -> Option<Eec1> {
    if data.len() < 8 {
        return None;
    }
    // SPN 899: Torque mode (byte 1, bits 0-3)
    let torque_mode = data[0] & 0x0F;

    // SPN 512: Driver's Demand Torque (byte 2) — offset -125, resolution 1%
    let driver_demand_torque_pct = byte_available(data[1]).map(|b| b as f64 - 125.0);

    // SPN 513: Actual Engine Torque (byte 3) — offset -125, resolution 1%
    let actual_torque_pct = byte_available(data[2]).map(|b| b as f64 - 125.0);

    // SPN 190: Engine Speed (bytes 4-5) — resolution 0.125 RPM
    let rpm_raw = u16::from_le_bytes([data[3], data[4]]);
    let engine_rpm = word_available(rpm_raw).map(|w| w as f64 * 0.125);

    Some(Eec1 {
        engine_rpm,
        driver_demand_torque_pct,
        actual_torque_pct,
        torque_mode,
    })
}

/// Decode CCVS (PGN 65265) from 8 raw bytes.
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF).
pub fn decode_ccvs(data: &[u8]) -> Option<Ccvs> {
    if data.len() < 8 {
        return None;
    }
    // SPN 84: Vehicle Speed (bytes 2-3) — resolution 1/256 km/h
    let speed_raw = u16::from_le_bytes([data[1], data[2]]);
    let vehicle_speed = word_available(speed_raw).map(|w| w as f64 / 256.0);

    // SPN 597: Brake Switch (byte 4, bits 2-3) — 0b11 = not available
    let brake_bits = (data[3] >> 2) & 0x03;
    let brake_switch = if brake_bits == 0x03 {
        None
    } else {
        Some(brake_bits == 1)
    };

    // SPN 595: Cruise Control Active (byte 1, bits 0-1) — 0b11 = not available
    let cruise_bits = data[0] & 0x03;
    let cruise_active = if cruise_bits == 0x03 {
        None
    } else {
        Some(cruise_bits == 1)
    };

    Some(Ccvs {
        vehicle_speed,
        brake_switch,
        cruise_active,
    })
}

/// Decode ET1 (PGN 65262) from 8 raw bytes.
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF).
pub fn decode_et1(data: &[u8]) -> Option<Et1> {
    if data.len() < 4 {
        return None;
    }
    // SPN 110: Engine Coolant Temp (byte 1) — offset -40°C
    let coolant_temp = byte_available(data[0]).map(|b| b as f64 - 40.0);

    // SPN 174: Fuel Temp (byte 2) — offset -40°C
    let fuel_temp = byte_available(data[1]).map(|b| b as f64 - 40.0);

    // SPN 175: Engine Oil Temp (bytes 3-4) — resolution 0.03125°C, offset -273°C
    let oil_raw = u16::from_le_bytes([data[2], data[3]]);
    let oil_temp = word_available(oil_raw).map(|w| w as f64 * 0.03125 - 273.0);

    Some(Et1 {
        coolant_temp,
        fuel_temp,
        oil_temp,
    })
}

/// Decode EFLP1 (PGN 65263) from 8 raw bytes.
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF).
pub fn decode_eflp1(data: &[u8]) -> Option<Eflp1> {
    if data.len() < 4 {
        return None;
    }
    // SPN 109: Coolant Pressure (byte 2) — resolution 2 kPa
    let coolant_pressure = byte_available(data[1]).map(|b| b as f64 * 2.0);

    // SPN 100: Engine Oil Pressure (byte 4) — resolution 4 kPa
    let oil_pressure = byte_available(data[3]).map(|b| b as f64 * 4.0);

    Some(Eflp1 {
        oil_pressure,
        coolant_pressure,
    })
}

/// Decode LFE (PGN 65266) from 8 raw bytes.
///
/// Fields are `None` when the ECU reports "not available" (0xFF/0xFFFF).
pub fn decode_lfe(data: &[u8]) -> Option<Lfe> {
    if data.len() < 4 {
        return None;
    }
    // SPN 183: Engine Fuel Rate (bytes 1-2) — resolution 0.05 L/h
    let rate_raw = u16::from_le_bytes([data[0], data[1]]);
    let fuel_rate = word_available(rate_raw).map(|w| w as f64 * 0.05);

    // SPN 184: Instantaneous Fuel Economy (bytes 3-4) — resolution 1/512 km/L
    let econ_raw = u16::from_le_bytes([data[2], data[3]]);
    let instantaneous_fuel_economy = word_available(econ_raw).map(|w| w as f64 / 512.0);

    Some(Lfe {
        fuel_rate,
        instantaneous_fuel_economy,
    })
}

/// Decode DM1/DM2 active DTCs from a J1939 diagnostic message.
///
/// The first 2 bytes are the lamp status, followed by 4-byte DTC entries.
pub fn decode_dm1(data: &[u8]) -> Vec<J1939Dtc> {
    if data.len() < 6 {
        return vec![];
    }
    // Skip 2 bytes of lamp status (MIL, RSL, AWL, PL)
    let dtc_data = &data[2..];
    dtc_data
        .chunks(4)
        .filter_map(J1939Dtc::from_bytes)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pgn_constants() {
        assert_eq!(Pgn::EEC1.0, 61444);
        assert_eq!(Pgn::CCVS.0, 65265);
        assert_eq!(Pgn::ET1.0, 65262);
        assert_eq!(Pgn::DM1.0, 65226);
    }

    #[test]
    fn test_pgn_name() {
        assert!(Pgn::EEC1.name().contains("Electronic Engine"));
        assert!(Pgn::CCVS.name().contains("Vehicle Speed"));
        assert_eq!(Pgn(99999).name(), "Unknown PGN");
    }

    #[test]
    fn test_pgn_display() {
        let s = format!("{}", Pgn::EEC1);
        assert!(s.contains("61444"));
        assert!(s.contains("EEC1"));
    }

    #[test]
    fn test_decode_eec1() {
        // Torque mode 0, demand -125+155=30%, actual -125+155=30%, RPM = 5440*0.125 = 680
        let data = [0x00, 155, 155, 0x40, 0x15, 0xFF, 0xFF, 0xFF];
        let eec1 = decode_eec1(&data).unwrap();
        assert!((eec1.engine_rpm.unwrap() - 680.0).abs() < 0.2);
        assert!((eec1.driver_demand_torque_pct.unwrap() - 30.0).abs() < 0.1);
        assert!((eec1.actual_torque_pct.unwrap() - 30.0).abs() < 0.1);
    }

    #[test]
    fn test_decode_eec1_not_available() {
        // All 0xFF = not available for torque and RPM fields
        let data = [0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        let eec1 = decode_eec1(&data).unwrap();
        assert!(eec1.engine_rpm.is_none());
        assert!(eec1.driver_demand_torque_pct.is_none());
        assert!(eec1.actual_torque_pct.is_none());
    }

    #[test]
    fn test_decode_eec1_too_short() {
        assert!(decode_eec1(&[0x00, 0x01]).is_none());
    }

    #[test]
    fn test_decode_ccvs() {
        // Speed: 0x1A00 / 256 = 26.0 km/h, brake off, cruise off
        let data = [0x00, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00];
        let ccvs = decode_ccvs(&data).unwrap();
        assert!((ccvs.vehicle_speed.unwrap() - 26.0).abs() < 0.1);
        assert_eq!(ccvs.brake_switch, Some(false));
        assert_eq!(ccvs.cruise_active, Some(false));
    }

    #[test]
    fn test_decode_ccvs_not_available() {
        let data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        let ccvs = decode_ccvs(&data).unwrap();
        assert!(ccvs.vehicle_speed.is_none());
        assert!(ccvs.brake_switch.is_none());
        assert!(ccvs.cruise_active.is_none());
    }

    #[test]
    fn test_decode_et1() {
        // Coolant: 90-40 = 50°C, Fuel: 60-40 = 20°C
        let data = [90, 60, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF];
        let et1 = decode_et1(&data).unwrap();
        assert!((et1.coolant_temp.unwrap() - 50.0).abs() < 0.1);
        assert!((et1.fuel_temp.unwrap() - 20.0).abs() < 0.1);
    }

    #[test]
    fn test_decode_et1_not_available() {
        let data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        let et1 = decode_et1(&data).unwrap();
        assert!(et1.coolant_temp.is_none());
        assert!(et1.fuel_temp.is_none());
        assert!(et1.oil_temp.is_none());
    }

    #[test]
    fn test_decode_eflp1() {
        // Coolant pressure: 50*2 = 100 kPa, Oil pressure: 100*4 = 400 kPa
        let data = [0xFF, 50, 0xFF, 100, 0xFF, 0xFF, 0xFF, 0xFF];
        let eflp1 = decode_eflp1(&data).unwrap();
        assert!((eflp1.coolant_pressure.unwrap() - 100.0).abs() < 0.1);
        assert!((eflp1.oil_pressure.unwrap() - 400.0).abs() < 0.1);
    }

    #[test]
    fn test_decode_eflp1_not_available() {
        let data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        let eflp1 = decode_eflp1(&data).unwrap();
        assert!(eflp1.oil_pressure.is_none());
        assert!(eflp1.coolant_pressure.is_none());
    }

    #[test]
    fn test_decode_lfe() {
        // Fuel rate: 100 * 0.05 = 5.0 L/h
        let data = [100, 0x00, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0xFF];
        let lfe = decode_lfe(&data).unwrap();
        assert!((lfe.fuel_rate.unwrap() - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_decode_lfe_not_available() {
        let data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        let lfe = decode_lfe(&data).unwrap();
        assert!(lfe.fuel_rate.is_none());
        assert!(lfe.instantaneous_fuel_economy.is_none());
    }

    #[test]
    fn test_j1939_dtc_from_bytes() {
        // SPN 190 (engine speed), FMI 2 (erratic)
        let data = [0xBE, 0x00, 0x02, 0x01]; // SPN low = 0x00BE = 190, high bits = 0, FMI = 2, OC = 1
        let dtc = J1939Dtc::from_bytes(&data).unwrap();
        assert_eq!(dtc.spn, 190);
        assert_eq!(dtc.fmi, 2);
        assert_eq!(dtc.occurrence_count, 1);
    }

    #[test]
    fn test_j1939_dtc_from_bytes_too_short() {
        assert!(J1939Dtc::from_bytes(&[0x00, 0x01]).is_none());
    }

    #[test]
    fn test_j1939_dtc_display() {
        let dtc = J1939Dtc {
            spn: 190,
            fmi: 2,
            occurrence_count: 1,
            conversion_method: 0,
        };
        let s = format!("{}", dtc);
        assert!(s.contains("SPN 190"));
        assert!(s.contains("FMI 2"));
        assert!(s.contains("Erratic"));
    }

    #[test]
    fn test_j1939_dtc_fmi_descriptions() {
        let dtc = J1939Dtc {
            spn: 0,
            fmi: 0,
            occurrence_count: 0,
            conversion_method: 0,
        };
        assert!(dtc.fmi_description().contains("Above Normal"));
        let dtc = J1939Dtc {
            spn: 0,
            fmi: 11,
            occurrence_count: 0,
            conversion_method: 0,
        };
        assert!(dtc.fmi_description().contains("Root Cause Not Known"));
    }

    #[test]
    fn test_decode_dm1() {
        // 2 bytes lamp status + 1 DTC (4 bytes)
        let data = [0x00, 0x00, 0xBE, 0x00, 0x02, 0x01];
        let dtcs = decode_dm1(&data);
        assert_eq!(dtcs.len(), 1);
        assert_eq!(dtcs[0].spn, 190);
        assert_eq!(dtcs[0].fmi, 2);
    }

    #[test]
    fn test_decode_dm1_empty() {
        assert!(decode_dm1(&[0x00, 0x00]).is_empty());
    }

    #[test]
    fn test_decode_dm1_multiple_dtcs() {
        let data = [
            0x00, 0x00, // lamp status
            0xBE, 0x00, 0x02, 0x01, // SPN 190 FMI 2
            0x64, 0x00, 0x03, 0x02, // SPN 100 FMI 3
        ];
        let dtcs = decode_dm1(&data);
        assert_eq!(dtcs.len(), 2);
        assert_eq!(dtcs[0].spn, 190);
        assert_eq!(dtcs[1].spn, 100);
    }
}
