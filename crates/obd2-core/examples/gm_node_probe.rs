use obd2_core::adapter::elm327::Elm327Adapter;
use obd2_core::adapter::{Adapter, PhysicalTarget, RoutedRequest};
use obd2_core::transport::serial::SerialTransport;
use obd2_core::vehicle::PhysicalAddress;
use std::env;

// (node address, human label). The label comes from the GM U-code decimal->hex map
// (e.g. U1024 = lost comm w/ TCM -> 24 dec -> 0x18). 0x10 vs 0x11 split is unverified.
const NODES: &[(u8, &str)] = &[
    (0x10, "ECM/PCM"),
    (0x11, "ECM (engine node 2)"),
    (0x18, "TCM"),
    (0x1A, "TCCM (transfer case)"),
    (0x20, "IPC (cluster)"),
    (0x29, "EBCM (brakes/ABS)"),
    (0x40, "BCM (body)"),
    (0x58, "SDM (airbag)"),
    (0x60, "HVAC / unknown"),
    (0x80, "Radio/IRC"),
    (0xA0, "DDM (driver door)"),
];

const PROBES: &[Probe] = &[
    // Generic emissions DTCs (normally only the ECM answers) -- kept for comparison.
    Probe {
        name: "generic stored (03)",
        service: 0x03,
        data: &[],
    },
    // GM Class 2 (SAE J2190) enhanced DTCs: request = 19 <statusMask> <groupHi> <groupLo>.
    // FF / FF 00 = "all DTCs, any status". Positive reply service = 59. Clear = 14.
    Probe {
        name: "Class2 $19 ALL DTCs (FF FF 00)",
        service: 0x19,
        data: &[0xFF, 0xFF, 0x00],
    },
    // Tech 2 filters with status mask 0x92 (MIL + history + current).
    Probe {
        name: "Class2 $19 active (92 FF 00)",
        service: 0x19,
        data: &[0x92, 0xFF, 0x00],
    },
    // GMLAN/CAN-era successor service; fallback in case a module only speaks A9.
    Probe {
        name: "GMLAN $A9 byStatusMask (fallback)",
        service: 0xA9,
        data: &[0x81, 0xFF],
    },
];

struct Probe {
    name: &'static str,
    service: u8,
    data: &'static [u8],
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port = env::args()
        .nth(1)
        .unwrap_or_else(|| "/dev/cu.usbserial-223230360830".to_string());
    let baud = env::args()
        .nth(2)
        .and_then(|arg| arg.parse::<u32>().ok())
        .unwrap_or(115_200);

    let transport = SerialTransport::new(&port, baud)?;
    let mut adapter = Elm327Adapter::new(Box::new(transport));
    let report = adapter.initialize().await?;
    println!("protocol: {:?}", report.info.protocol);

    for (node, label) in NODES {
        println!("\n[node {node:02X}  {label}]");
        for probe in PROBES {
            let request = RoutedRequest {
                service_id: probe.service,
                data: probe.data.to_vec(),
                target: PhysicalTarget::Addressed(PhysicalAddress::J1850 {
                    node: *node,
                    header: [0x6C, *node, 0xF1],
                }),
            };
            match adapter.routed_request(&request).await {
                Ok(bytes) => {
                    println!(
                        "  {:02X}{} {:<32} -> {}",
                        probe.service,
                        hex_bytes(probe.data),
                        probe.name,
                        hex_bytes(&bytes)
                    );
                }
                Err(err) => {
                    println!(
                        "  {:02X}{} {:<32} -> ERR {}",
                        probe.service,
                        hex_bytes(probe.data),
                        probe.name,
                        err
                    );
                }
            }
        }
    }

    Ok(())
}

fn hex_bytes(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        use std::fmt::Write;
        write!(&mut out, "{byte:02X}").expect("write to string");
    }
    out
}
