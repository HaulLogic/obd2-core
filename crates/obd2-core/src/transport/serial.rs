//! Serial port transport using tokio-serial.
//!
//! Requires the `serial` feature flag.

use async_trait::async_trait;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio_serial::SerialPortBuilderExt;
use tracing::debug;

use super::Transport;
use crate::error::Obd2Error;

/// Serial port transport for ELM327/STN adapters.
///
/// Connects to a serial port (USB or RS-232) at the specified baud rate.
/// Default baud rate is 115200 for modern ELM327 adapters, 38400 for
/// older ones or emulators.
pub struct SerialTransport {
    port: tokio_serial::SerialStream,
    port_name: String,
    read_buf: Vec<u8>,
}

impl SerialTransport {
    /// Open a serial port with the specified baud rate.
    pub fn new(port_name: &str, baud_rate: u32) -> Result<Self, Obd2Error> {
        let port = tokio_serial::new(port_name, baud_rate)
            .open_native_async()
            .map_err(|e| Obd2Error::Transport(format!("failed to open {}: {}", port_name, e)))?;

        Ok(Self {
            port,
            port_name: port_name.to_string(),
            read_buf: vec![0u8; 4096],
        })
    }

    /// Open with default baud rate (115200).
    pub fn with_defaults(port_name: &str) -> Result<Self, Obd2Error> {
        Self::new(port_name, 115200)
    }
}

impl std::fmt::Debug for SerialTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SerialTransport")
            .field("port_name", &self.port_name)
            .finish()
    }
}

#[async_trait]
impl Transport for SerialTransport {
    async fn write(&mut self, data: &[u8]) -> Result<(), Obd2Error> {
        debug!(port = %self.port_name, data = %String::from_utf8_lossy(data), "serial write");
        self.port
            .write_all(data)
            .await
            .map_err(|e| Obd2Error::Transport(format!("serial write error: {}", e)))?;
        self.port
            .flush()
            .await
            .map_err(|e| Obd2Error::Transport(format!("serial flush error: {}", e)))?;
        Ok(())
    }

    async fn read(&mut self) -> Result<Vec<u8>, Obd2Error> {
        let mut result = Vec::new();
        let timeout = tokio::time::Duration::from_secs(5);

        loop {
            match tokio::time::timeout(timeout, self.port.read(&mut self.read_buf)).await {
                Ok(Ok(0)) => {
                    return Err(Obd2Error::Transport("serial port closed".into()));
                }
                Ok(Ok(n)) => {
                    result.extend_from_slice(&self.read_buf[..n]);
                    // Check for ELM327 prompt character '>'
                    if result.contains(&b'>') {
                        break;
                    }
                    // Check for common terminators
                    let as_str = String::from_utf8_lossy(&result);
                    if as_str.ends_with('\r') || as_str.ends_with('\n') {
                        // Give a tiny bit more time for additional data
                        match tokio::time::timeout(
                            tokio::time::Duration::from_millis(50),
                            self.port.read(&mut self.read_buf),
                        )
                        .await
                        {
                            Ok(Ok(n)) if n > 0 => {
                                result.extend_from_slice(&self.read_buf[..n]);
                            }
                            _ => break,
                        }
                    }
                }
                Ok(Err(e)) => {
                    return Err(Obd2Error::Transport(format!("serial read error: {}", e)));
                }
                Err(_) => {
                    if result.is_empty() {
                        return Err(Obd2Error::Timeout);
                    }
                    break; // Return what we have
                }
            }
        }

        debug!(port = %self.port_name, data = %String::from_utf8_lossy(&result), "serial read");
        Ok(result)
    }

    async fn reset(&mut self) -> Result<(), Obd2Error> {
        // Clear any pending data
        let _ = tokio::time::timeout(tokio::time::Duration::from_millis(100), async {
            loop {
                match self.port.read(&mut self.read_buf).await {
                    Ok(0) | Err(_) => break,
                    Ok(_) => continue,
                }
            }
        })
        .await;
        Ok(())
    }

    fn name(&self) -> &str {
        &self.port_name
    }
}

/// List available serial ports on this system.
pub fn list_ports() -> Vec<String> {
    tokio_serial::available_ports()
        .unwrap_or_default()
        .into_iter()
        .map(|p| p.port_name)
        .collect()
}
