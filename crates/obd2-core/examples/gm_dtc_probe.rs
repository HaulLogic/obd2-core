use obd2_core::adapter::elm327::Elm327Adapter;
use obd2_core::protocol::service::Target;
use obd2_core::session::Session;
use obd2_core::transport::serial::SerialTransport;
use std::env;

const MODULES: &[&str] = &["ecm", "tcm", "ficm", "bcm", "abs"];

const PROBES: &[Probe] = &[
    Probe {
        name: "UDS/J2190 report DTC count by status mask",
        service: 0x19,
        data: &[0x01, 0xFF],
    },
    Probe {
        name: "UDS/J2190 report DTCs by status mask",
        service: 0x19,
        data: &[0x02, 0xFF],
    },
    Probe {
        name: "UDS report supported DTCs",
        service: 0x19,
        data: &[0x0A],
    },
    Probe {
        name: "KWP/J2190 candidate read DTC status",
        service: 0x18,
        data: &[0x00],
    },
    Probe {
        name: "KWP/J2190 candidate read DTC status mask",
        service: 0x18,
        data: &[0x00, 0xFF],
    },
    Probe {
        name: "KWP/J2190 candidate read DTC status group",
        service: 0x18,
        data: &[0x00, 0xFF, 0x00],
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
    let adapter = Elm327Adapter::new(Box::new(transport));
    let mut session = Session::new(adapter);

    let profile = session.identify_vehicle().await?;
    println!("VIN: {}", profile.vin);
    if let Some(spec) = &profile.spec {
        println!("Spec: {}", spec.identity.name);
    } else {
        println!("Spec: none");
    }

    for module in MODULES {
        println!("\n[{module}]");
        for probe in PROBES {
            match session
                .raw_request(
                    probe.service,
                    probe.data,
                    Target::Module((*module).to_string()),
                )
                .await
            {
                Ok(bytes) => {
                    println!(
                        "  {:02X}{} {:<45} -> {}",
                        probe.service,
                        hex_bytes(probe.data),
                        probe.name,
                        hex_bytes(&bytes)
                    );
                }
                Err(err) => {
                    println!(
                        "  {:02X}{} {:<45} -> ERR {}",
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
