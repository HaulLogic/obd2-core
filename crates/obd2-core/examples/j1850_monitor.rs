use obd2_core::transport::serial::SerialTransport;
use obd2_core::transport::Transport;
use std::env;
use std::sync::{Arc, Mutex};
use std::time::Duration;

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port = env::args()
        .nth(1)
        .unwrap_or_else(|| "/dev/cu.usbserial-223230360830".to_string());
    let baud = env::args()
        .nth(2)
        .and_then(|arg| arg.parse::<u32>().ok())
        .unwrap_or(115_200);

    let mut transport = SerialTransport::new(&port, baud)?;

    for cmd in ["ATZ", "ATE0", "ATL0", "ATS0", "ATH1", "ATSP2"] {
        let response = send(&mut transport, cmd).await?;
        println!("{cmd:<5} {}", sanitize(&response));
    }

    println!("AT MA listening for 10 seconds...");
    let captured = Arc::new(Mutex::new(Vec::<u8>::new()));
    let observer_capture = Arc::clone(&captured);
    transport.set_chunk_observer(Some(Arc::new(Mutex::new(move |chunk: &[u8]| {
        if let Ok(mut bytes) = observer_capture.lock() {
            bytes.extend_from_slice(chunk);
        }
    }))));

    transport.write(b"AT MA\r").await?;
    let monitor = tokio::time::timeout(Duration::from_secs(10), transport.read()).await;
    let _ = transport.write(b"\r").await;

    match monitor {
        Ok(Ok(bytes)) => {
            println!("{}", String::from_utf8_lossy(&bytes));
        }
        Ok(Err(err)) => {
            println!("ERR {err}");
        }
        Err(_) => {}
    }

    if let Ok(bytes) = captured.lock() {
        if !bytes.is_empty() {
            println!("{}", String::from_utf8_lossy(&bytes));
        }
    }

    Ok(())
}

async fn send(
    transport: &mut SerialTransport,
    cmd: &str,
) -> Result<String, Box<dyn std::error::Error>> {
    let mut bytes = Vec::with_capacity(cmd.len() + 1);
    bytes.extend_from_slice(cmd.as_bytes());
    bytes.push(b'\r');
    transport.write(&bytes).await?;
    let response = transport.read().await?;
    Ok(String::from_utf8_lossy(&response).into_owned())
}

fn sanitize(response: &str) -> String {
    response.replace('\r', "\\r").replace('\n', "\\n")
}
