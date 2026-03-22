//! Offline VIN (Vehicle Identification Number) decoding.
//!
//! Decodes model year, manufacturer, and vehicle class from the 17-character VIN
//! without any network calls. Uses the 10th character for year and WMI (first 3
//! characters) for manufacturer identification.

/// Decoded VIN information.
#[derive(Debug, Clone)]
pub struct DecodedVin {
    pub year: Option<i32>,
    pub year_alt: Option<i32>,
    pub manufacturer: Option<String>,
    pub truck_class: Option<String>,
}

/// Decode the model year from the VIN's 10th character.
pub fn decode_year(vin: &str) -> Option<i32> {
    // Returns the most likely year (2010-2039 cycle)
    // 10th char: A=2010, B=2011, ..., J=2018 (skip I), K=2019, L=2020, M=2021, N=2022
    // 1=2031, 2=2032, ..., 9=2039
    // Also 1=2001, 2=2002, ..., 9=2009 (previous cycle)
    decode_year_candidates(vin).0
}

/// Decode both possible year cycles (current 2010-2039, previous 1980-2009).
pub fn decode_year_candidates(vin: &str) -> (Option<i32>, Option<i32>) {
    let ch = match vin.as_bytes().get(9) {
        Some(c) => c,
        None => return (None, None),
    };
    match ch {
        b'A' => (Some(2010), Some(1980)),
        b'B' => (Some(2011), Some(1981)),
        b'C' => (Some(2012), Some(1982)),
        b'D' => (Some(2013), Some(1983)),
        b'E' => (Some(2014), Some(1984)),
        b'F' => (Some(2015), Some(1985)),
        b'G' => (Some(2016), Some(1986)),
        b'H' => (Some(2017), Some(1987)),
        b'J' => (Some(2018), Some(1988)),
        b'K' => (Some(2019), Some(1989)),
        b'L' => (Some(2020), Some(1990)),
        b'M' => (Some(2021), Some(1991)),
        b'N' => (Some(2022), Some(1992)),
        b'P' => (Some(2023), Some(1993)),
        b'R' => (Some(2024), Some(1994)),
        b'S' => (Some(2025), Some(1995)),
        b'T' => (Some(2026), Some(1996)),
        b'V' => (Some(2027), Some(1997)),
        b'W' => (Some(2028), Some(1998)),
        b'X' => (Some(2029), Some(1999)),
        b'Y' => (Some(2030), Some(2000)),
        b'1' => (Some(2031), Some(2001)),
        b'2' => (Some(2032), Some(2002)),
        b'3' => (Some(2033), Some(2003)),
        b'4' => (Some(2034), Some(2004)),
        b'5' => (Some(2035), Some(2005)),
        b'6' => (Some(2036), Some(2006)),
        b'7' => (Some(2037), Some(2007)),
        b'8' => (Some(2038), Some(2008)),
        b'9' => (Some(2039), Some(2009)),
        _ => (None, None),
    }
}

/// Decode manufacturer from the World Manufacturer Identifier (first 3 chars).
pub fn decode_manufacturer(wmi: &str) -> Option<&'static str> {
    // Match specific 3-char WMIs first, then 2-char prefixes
    let wmi_upper = wmi.to_uppercase();
    let wmi3 = if wmi_upper.len() >= 3 { &wmi_upper[..3] } else { return None; };

    match wmi3 {
        // GM
        "1GC" | "1GT" | "2GC" => Some("Chevrolet"),
        "1G1" => Some("Chevrolet"),
        "1G2" => Some("Pontiac"),
        "1GK" | "1GD" => Some("GMC"),
        "1G6" => Some("Cadillac"),
        "1GB" => Some("GM Bus/Incomplete"),
        // Ford
        "1FA" | "1FT" | "1FM" | "1FD" | "3FA" => Some("Ford"),
        "1LN" => Some("Lincoln"),
        "2FA" | "2FM" | "2FT" => Some("Ford (Canada)"),
        // Chrysler/Stellantis
        "1C3" | "1C6" | "2C3" | "2C4" => Some("Chrysler"),
        "1B3" | "1B7" | "3B7" | "3D7" => Some("Dodge/Ram"),
        "1C4" | "1J4" | "1J8" => Some("Jeep"),
        // Toyota
        "1TM" | "4T1" | "5TD" | "JTD" | "JTE" | "JTN" => Some("Toyota"),
        "2T1" | "2T3" => Some("Toyota (Canada)"),
        // Honda
        "1HG" | "2HG" | "JHM" | "5FN" | "5J6" => Some("Honda"),
        "JH4" | "19U" => Some("Acura"),
        // Nissan
        "1N4" | "1N6" | "3N1" | "5N1" | "JN8" => Some("Nissan"),
        "JN1" => Some("Infiniti"),
        // Subaru
        "JF1" | "JF2" | "4S3" | "4S4" => Some("Subaru"),
        // Mazda
        "JM1" | "JM3" | "3MZ" => Some("Mazda"),
        // BMW
        "WBA" | "WBS" | "WBY" | "5UX" | "5UJ" => Some("BMW"),
        "WMW" | "WME" => Some("MINI"),
        // Mercedes
        "WDB" | "WDC" | "WDD" | "4JG" | "55S" => Some("Mercedes-Benz"),
        // VW/Audi
        "WVW" | "WV1" | "WV2" | "3VW" => Some("Volkswagen"),
        "WAU" | "WA1" => Some("Audi"),
        "WP0" | "WP1" => Some("Porsche"),
        // Hyundai/Kia
        "KMH" | "5NP" | "5NM" => Some("Hyundai"),
        "KNA" | "KND" | "5XY" => Some("Kia"),
        // Tesla
        "5YJ" | "7SA" => Some("Tesla"),
        // Volvo
        "YV1" | "YV4" => Some("Volvo"),
        _ => {
            // Fall back to country of origin from first character
            match wmi_upper.as_bytes().first()? {
                b'1' | b'4' | b'5' => Some("United States"),
                b'2' => Some("Canada"),
                b'3' => Some("Mexico"),
                b'J' => Some("Japan"),
                b'K' => Some("South Korea"),
                b'S' => Some("United Kingdom"),
                b'W' => Some("Germany"),
                b'Z' => Some("Italy"),
                b'Y' => Some("Sweden/Finland"),
                _ => None,
            }
        }
    }
}

/// Detect vehicle class/type from VIN patterns.
pub fn detect_truck_class(vin: &str) -> Option<&'static str> {
    if vin.len() < 17 { return None; }
    let wmi = &vin[..3].to_uppercase();

    match wmi.as_str() {
        // GM HD trucks (diesel likely)
        "1GC" | "1GT" | "2GC" => {
            let eighth = vin.as_bytes().get(7)?;
            match eighth {
                b'1' | b'2' | b'5' | b'6' => Some("diesel-truck"),
                _ => Some("gas-truck-v8"),
            }
        }
        // Ford trucks
        "1FT" | "1FD" | "3FT" => Some("gas-truck-v8"),
        // Ram trucks
        "3D7" | "1B7" | "3B7" | "3C6" => Some("gas-truck-v8"),
        // SUVs
        "1FM" | "5FN" | "5J6" | "5TD" => Some("suv"),
        // Sedans
        "1G1" | "1HG" | "2HG" | "1FA" | "3FA" | "4T1" => Some("sedan"),
        // Sports/performance
        "WBA" | "WBS" | "WP0" => Some("performance"),
        _ => None,
    }
}

/// Decode all available information from a VIN.
pub fn decode(vin: &str) -> DecodedVin {
    let (year, year_alt) = decode_year_candidates(vin);
    let manufacturer = if vin.len() >= 17 {
        decode_manufacturer(&vin[..3]).map(String::from)
    } else {
        None
    };
    let truck_class = detect_truck_class(vin).map(String::from);

    DecodedVin { year, year_alt, manufacturer, truck_class }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_decode_year_2004() {
        assert_eq!(decode_year("1GCHK23124F000001"), Some(2034));
        // 10th char is '4' => 2034 in current cycle
        // For 2004 vehicle, year_alt would be 2004
    }

    #[test]
    fn test_decode_year_candidates() {
        let (current, previous) = decode_year_candidates("1GCHK23124F000001");
        assert_eq!(current, Some(2034));
        assert_eq!(previous, Some(2004));
    }

    #[test]
    fn test_decode_year_2006() {
        assert_eq!(decode_year("WMWRE33546T000001"), Some(2036));
        // 10th char '6' => 2036 current, 2006 previous
    }

    #[test]
    fn test_decode_manufacturer_chevy() {
        assert_eq!(decode_manufacturer("1GC"), Some("Chevrolet"));
    }

    #[test]
    fn test_decode_manufacturer_ford() {
        assert_eq!(decode_manufacturer("1FT"), Some("Ford"));
    }

    #[test]
    fn test_decode_manufacturer_mini() {
        assert_eq!(decode_manufacturer("WMW"), Some("MINI"));
    }

    #[test]
    fn test_decode_manufacturer_toyota() {
        assert_eq!(decode_manufacturer("4T1"), Some("Toyota"));
    }

    #[test]
    fn test_decode_manufacturer_honda() {
        assert_eq!(decode_manufacturer("1HG"), Some("Honda"));
    }

    #[test]
    fn test_decode_manufacturer_tesla() {
        assert_eq!(decode_manufacturer("5YJ"), Some("Tesla"));
    }

    #[test]
    fn test_decode_manufacturer_fallback_country() {
        // Unknown WMI but known country prefix
        assert_eq!(decode_manufacturer("1ZZ"), Some("United States"));
        assert_eq!(decode_manufacturer("JXX"), Some("Japan"));
    }

    #[test]
    fn test_detect_truck_class_diesel() {
        assert_eq!(detect_truck_class("1GCHK23124F000001"), Some("diesel-truck"));
        // 8th digit '2' maps to diesel
    }

    #[test]
    fn test_detect_truck_class_sedan() {
        assert_eq!(detect_truck_class("1G1YY22G465000001"), Some("sedan"));
    }

    #[test]
    fn test_decode_full() {
        let decoded = decode("1GCHK23124F000001");
        assert!(decoded.manufacturer.is_some());
        assert_eq!(decoded.manufacturer.as_deref(), Some("Chevrolet"));
        assert!(decoded.truck_class.is_some());
    }

    #[test]
    fn test_decode_short_vin() {
        let decoded = decode("SHORT");
        assert!(decoded.year.is_none());
        assert!(decoded.manufacturer.is_none());
    }

    #[test]
    fn test_decode_invalid_year_char() {
        assert_eq!(decode_year("1GCHK2312IF000001"), None); // 'I' is not used
    }
}
