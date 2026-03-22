//! Mock transport for testing.

use async_trait::async_trait;
use std::collections::VecDeque;
use crate::error::Obd2Error;
use super::Transport;

/// A mock transport that returns pre-configured responses.
///
/// Use `expect()` to queue command/response pairs. When `write()` is called,
/// the command is matched and the corresponding response is queued for `read()`.
#[derive(Debug)]
pub struct MockTransport {
    expectations: Vec<(String, String)>,
    pending_response: VecDeque<String>,
}

impl MockTransport {
    /// Create a new empty MockTransport.
    pub fn new() -> Self {
        Self {
            expectations: Vec::new(),
            pending_response: VecDeque::new(),
        }
    }

    /// Add an expected command/response pair.
    /// When write() receives data matching `command`, the `response` is queued.
    pub fn expect(&mut self, command: &str, response: &str) {
        self.expectations.push((command.to_string(), response.to_string()));
    }
}

impl Default for MockTransport {
    fn default() -> Self { Self::new() }
}

#[async_trait]
impl Transport for MockTransport {
    async fn write(&mut self, data: &[u8]) -> Result<(), Obd2Error> {
        let cmd = String::from_utf8_lossy(data).trim().to_string();
        // Find matching expectation
        for (expected_cmd, response) in &self.expectations {
            if cmd == *expected_cmd || cmd.contains(expected_cmd.as_str()) {
                self.pending_response.push_back(response.clone());
                return Ok(());
            }
        }
        // No match — return a generic "NO DATA"
        self.pending_response.push_back("NO DATA\r>".to_string());
        Ok(())
    }

    async fn read(&mut self) -> Result<Vec<u8>, Obd2Error> {
        match self.pending_response.pop_front() {
            Some(response) => Ok(response.into_bytes()),
            None => Err(Obd2Error::Timeout),
        }
    }

    async fn reset(&mut self) -> Result<(), Obd2Error> {
        self.pending_response.clear();
        Ok(())
    }

    fn name(&self) -> &str { "mock" }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_mock_transport_expect() {
        let mut t = MockTransport::new();
        t.expect("ATZ", "ELM327 v2.1\r>");
        t.write(b"ATZ").await.unwrap();
        let response = t.read().await.unwrap();
        assert!(String::from_utf8_lossy(&response).contains("ELM327"));
    }

    #[tokio::test]
    async fn test_mock_transport_no_match() {
        let mut t = MockTransport::new();
        t.write(b"UNKNOWN").await.unwrap();
        let response = t.read().await.unwrap();
        assert!(String::from_utf8_lossy(&response).contains("NO DATA"));
    }

    #[tokio::test]
    async fn test_mock_transport_reset() {
        let mut t = MockTransport::new();
        t.expect("ATZ", "OK\r>");
        t.write(b"ATZ").await.unwrap();
        t.reset().await.unwrap();
        // After reset, no pending response
        let result = t.read().await;
        assert!(result.is_err()); // Timeout
    }
}
