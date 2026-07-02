mod compare;
mod report;
mod runner;
mod tests;
mod vehicles;

use std::path::{Path, PathBuf};
use std::time::Duration;

use clap::{Parser, Subcommand, ValueEnum};
use serde_json::json;

use obd2_core::adapter::elm327::Elm327Adapter;
use obd2_core::transport::Link;

use crate::report::{Report, ReportMeta, TestGroupResult, TestStatus};

#[derive(Parser)]
#[command(
    name = "obd2-hw-test",
    about = "Hardware parity test harness for obd2-core"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Run the test matrix against real hardware
    Run {
        #[arg(long)]
        transport: TransportArg,
        #[arg(long)]
        port: Option<String>,
        #[arg(long)]
        vehicle: String,
        #[arg(long, default_value = "results/report.json")]
        output: String,
        #[arg(long)]
        only: Option<String>,
        #[arg(long)]
        interactive: bool,
    },
    /// Compare two JSON reports for parity
    Compare { report_a: String, report_b: String },
    /// List known vehicle definitions
    Vehicles,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum TransportArg {
    Usb,
    Ble,
}

impl TransportArg {
    fn as_str(self) -> &'static str {
        match self {
            Self::Usb => "usb",
            Self::Ble => "ble",
        }
    }
}

#[tokio::main]
async fn main() {
    let cli = Cli::parse();
    let exit_code = match cli.command {
        Command::Run {
            transport,
            port,
            vehicle,
            output,
            only,
            interactive,
        } => run_command(transport, port, vehicle, output, only, interactive).await,
        Command::Compare { report_a, report_b } => compare_command(&report_a, &report_b),
        Command::Vehicles => {
            vehicles::list_vehicles();
            0
        }
    };

    std::process::exit(exit_code);
}

async fn run_command(
    transport: TransportArg,
    port: Option<String>,
    vehicle_id: String,
    output: String,
    only: Option<String>,
    interactive: bool,
) -> i32 {
    let output_path = PathBuf::from(&output);
    let Some(vehicle) = vehicles::find_vehicle(&vehicle_id) else {
        eprintln!("Unknown vehicle: {vehicle_id}. Use `vehicles` to list known IDs.");
        return 2;
    };

    let only_groups = match parse_only_groups(only.as_deref()) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("{error}");
            return 2;
        }
    };

    let meta = ReportMeta {
        timestamp: chrono::Utc::now().to_rfc3339(),
        harness_version: env!("CARGO_PKG_VERSION").to_string(),
        vehicle_id: vehicle.id.to_string(),
        transport: transport.as_str().to_string(),
        port: port.clone(),
        adapter_chipset: None,
        adapter_firmware: None,
        protocol_detected: None,
        raw_capture_path: None,
    };

    println!("=== obd2-hw-test ===");
    println!("Vehicle:   {} ({})", vehicle.display_name, vehicle.id);
    println!("Transport: {}", transport.as_str());
    println!("Output:    {}", output_path.display());
    if let Some(groups) = &only_groups {
        println!("Only:      {}", groups.join(","));
    }
    println!();

    let transport_box = match build_transport(transport, port.as_deref()).await {
        Ok(transport) => transport,
        Err(error) => {
            eprintln!("{error}");
            let _ = write_fatal_report(
                meta,
                &output_path,
                "startup",
                &error,
                json!({ "transport": transport.as_str() }),
            );
            return 1;
        }
    };

    let adapter = Elm327Adapter::new(transport_box);
    let mut session = obd2_core::session::Session::new(adapter);
    session.set_raw_capture_enabled(true);
    session.set_raw_capture_directory(capture_dir_for_output(&output_path));

    let mut report = runner::run_matrix(
        &mut session,
        vehicle,
        meta,
        only_groups.as_deref(),
        interactive,
    )
    .await;
    hydrate_report_meta(&session, &mut report);

    if let Err(error) = report.write_to_file(&output_path) {
        eprintln!(
            "Failed to write report to {}: {error}",
            output_path.display()
        );
        return 1;
    }

    println!();
    println!("=== Summary ===");
    println!(
        "Total: {} | Passed: {} | Failed: {} | Skipped: {}",
        report.summary.total, report.summary.passed, report.summary.failed, report.summary.skipped
    );
    println!("Duration: {:.1}s", report.summary.duration_secs);
    if let Some(path) = &report.meta.raw_capture_path {
        println!("Capture:  {path}");
    }
    println!("Report:   {}", output_path.display());

    if report.summary.failed > 0 || report.fatal_error.is_some() {
        1
    } else {
        0
    }
}

fn compare_command(report_a: &str, report_b: &str) -> i32 {
    let report_a = match Report::read_from_file(report_a) {
        Ok(report) => report,
        Err(error) => {
            eprintln!("Failed to read {report_a}: {error}");
            return 1;
        }
    };
    let report_b = match Report::read_from_file(report_b) {
        Ok(report) => report,
        Err(error) => {
            eprintln!("Failed to read {report_b}: {error}");
            return 1;
        }
    };

    let diffs = compare::compare_reports(&report_a, &report_b);
    compare::print_comparison(&report_a, &report_b, &diffs);
    if compare::parity_ok(&diffs) {
        0
    } else {
        1
    }
}

async fn build_transport(
    transport: TransportArg,
    port: Option<&str>,
) -> Result<Box<dyn Link>, String> {
    match transport {
        TransportArg::Usb => {
            let port = port.ok_or_else(|| {
                let ports = obd2_core::transport::serial::list_ports();
                if ports.is_empty() {
                    "--port is required for USB transport".to_string()
                } else {
                    format!(
                        "--port is required for USB transport. Available ports: {}",
                        ports.join(", ")
                    )
                }
            })?;
            let transport = obd2_core::transport::serial::SerialTransport::with_defaults(port)
                .map_err(|error| format!("Failed to open serial port {port}: {error}"))?;
            Ok(Box::new(transport))
        }
        TransportArg::Ble => {
            let transport = obd2_core::transport::ble::BleTransport::scan_and_connect(
                None,
                Duration::from_secs(10),
            )
            .await
            .map_err(|error| format!("BLE scan/connect failed: {error}"))?;
            Ok(Box::new(transport))
        }
    }
}

fn parse_only_groups(value: Option<&str>) -> Result<Option<Vec<String>>, String> {
    let Some(value) = value else {
        return Ok(None);
    };

    let groups = value
        .split(',')
        .map(str::trim)
        .filter(|item| !item.is_empty())
        .map(ToOwned::to_owned)
        .collect::<Vec<_>>();

    if groups.is_empty() {
        return Ok(None);
    }

    let known = tests::group_names();
    let unknown = groups
        .iter()
        .filter(|group| !known.iter().any(|known| known == &group.as_str()))
        .cloned()
        .collect::<Vec<_>>();

    if !unknown.is_empty() {
        return Err(format!(
            "Unknown group(s): {}. Known groups: {}",
            unknown.join(", "),
            known.join(", ")
        ));
    }

    Ok(Some(groups))
}

fn capture_dir_for_output(output: &Path) -> PathBuf {
    output
        .parent()
        .unwrap_or_else(|| Path::new("results"))
        .join("captures")
}

fn hydrate_report_meta(session: &obd2_core::session::Session<Elm327Adapter>, report: &mut Report) {
    let info = session.adapter_info();
    report.meta.adapter_chipset = Some(format!("{:?}", info.chipset));
    report.meta.adapter_firmware = Some(info.firmware.clone());
    report.meta.protocol_detected = Some(format!("{:?}", info.protocol));
    report.meta.raw_capture_path = session
        .raw_capture_path()
        .map(|path| path.display().to_string());
}

fn write_fatal_report(
    meta: ReportMeta,
    output_path: &Path,
    group: &str,
    message: &str,
    details: serde_json::Value,
) -> std::io::Result<()> {
    let mut report = Report::new(meta);
    report.fatal_error = Some(message.to_string());
    report.tests.insert(
        group.to_string(),
        TestGroupResult {
            status: TestStatus::Fail,
            duration_ms: 0,
            reason: Some(message.to_string()),
            details: Some(details),
        },
    );
    report.compute_summary();
    report.write_to_file(output_path)
}
