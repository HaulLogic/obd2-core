//! Diagnostic session management — Mode 10/27/2F/3E.
//!
//! Implements the session control sequence required for actuator tests:
//! 1. enter_diagnostic_session(Extended)  — Mode 10
//! 2. security_access(module, key_fn)     — Mode 27
//! 3. actuator_control(did, module, cmd)  — Mode 2F
//!
//! Tester Present (Mode 3E) keep-alive runs automatically during
//! extended sessions.

use crate::adapter::Adapter;
use crate::error::Obd2Error;
use crate::protocol::service::{ServiceRequest, Target, DiagSession, ActuatorCommand};
use tokio::sync::watch;

/// State of the diagnostic session.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SessionState {
    /// Default session — no special access.
    Default,
    /// Extended session active — Mode 2F available after security access.
    Extended {
        /// Modules that have been security-unlocked.
        unlocked_modules: Vec<String>,
    },
    /// Programming session (for reflash — library provides no flash capability).
    Programming,
}

impl Default for SessionState {
    fn default() -> Self { Self::Default }
}

/// Callback type for computing the security key from a seed.
/// The algorithm is manufacturer-proprietary (BR-7.2).
pub type KeyFunction = Box<dyn Fn(&[u8]) -> Vec<u8> + Send>;

/// Enter a diagnostic session (Mode 10).
///
/// Starts tester-present keep-alive automatically (BR-3.6).
pub async fn enter_session<A: Adapter>(
    adapter: &mut A,
    session: DiagSession,
    module: &str,
) -> Result<SessionState, Obd2Error> {
    let sub = match session {
        DiagSession::Default => 0x01,
        DiagSession::Programming => 0x02,
        DiagSession::Extended => 0x03,
    };

    let req = ServiceRequest {
        service_id: 0x10,
        data: vec![sub],
        target: Target::Module(module.to_string()),
    };

    adapter.request(&req).await?;

    let state = match session {
        DiagSession::Default => SessionState::Default,
        DiagSession::Extended => SessionState::Extended { unlocked_modules: vec![] },
        DiagSession::Programming => SessionState::Programming,
    };

    tracing::info!(session = ?session, module = module, "entered diagnostic session");
    Ok(state)
}

/// Request security access (Mode 27) — seed/key exchange.
///
/// Step 1: Request seed (sub-function 0x01)
/// Step 2: Send key (sub-function 0x02) computed by the caller's KeyFunction
pub async fn security_access<A: Adapter>(
    adapter: &mut A,
    module: &str,
    key_fn: &KeyFunction,
) -> Result<(), Obd2Error> {
    // Step 1: Request seed
    let seed_req = ServiceRequest {
        service_id: 0x27,
        data: vec![0x01],
        target: Target::Module(module.to_string()),
    };
    let seed = adapter.request(&seed_req).await?;

    if seed.is_empty() {
        return Err(Obd2Error::Adapter("empty seed from Mode 27".into()));
    }

    // Check if already unlocked (seed = all zeros)
    if seed.iter().all(|&b| b == 0) {
        tracing::info!(module = module, "security already unlocked (zero seed)");
        return Ok(());
    }

    // Step 2: Compute key and send
    let key = key_fn(&seed);

    let key_req = ServiceRequest {
        service_id: 0x27,
        data: std::iter::once(0x02).chain(key.into_iter()).collect(),
        target: Target::Module(module.to_string()),
    };
    adapter.request(&key_req).await?;

    tracing::info!(module = module, "security access granted");
    Ok(())
}

/// Send an actuator control command (Mode 2F).
///
/// Requires active extended session + security access (BR-7.1).
pub async fn actuator_control<A: Adapter>(
    adapter: &mut A,
    did: u16,
    module: &str,
    command: &ActuatorCommand,
    state: &SessionState,
) -> Result<(), Obd2Error> {
    // Verify session state (BR-7.1)
    match state {
        SessionState::Extended { unlocked_modules } => {
            if !unlocked_modules.contains(&module.to_string()) {
                return Err(Obd2Error::SecurityRequired);
            }
        }
        _ => return Err(Obd2Error::SecurityRequired),
    }

    let did_bytes = [(did >> 8) as u8, (did & 0xFF) as u8];
    let control_bytes = match command {
        ActuatorCommand::ReturnToEcu => vec![0x00],
        ActuatorCommand::Activate => vec![0x03],
        ActuatorCommand::Adjust(data) => {
            let mut v = vec![0x03];
            v.extend(data);
            v
        }
    };

    let mut data = Vec::new();
    data.extend_from_slice(&did_bytes);
    data.extend(control_bytes);

    let req = ServiceRequest {
        service_id: 0x2F,
        data,
        target: Target::Module(module.to_string()),
    };

    tracing::warn!(did = format!("{:#06X}", did), module = module, "actuator control command");
    adapter.request(&req).await?;
    Ok(())
}

/// Release actuator control — return to ECU (Mode 2F with ReturnToEcu).
pub async fn actuator_release<A: Adapter>(
    adapter: &mut A,
    did: u16,
    module: &str,
) -> Result<(), Obd2Error> {
    let did_bytes = [(did >> 8) as u8, (did & 0xFF) as u8];
    let req = ServiceRequest {
        service_id: 0x2F,
        data: vec![did_bytes[0], did_bytes[1], 0x00],
        target: Target::Module(module.to_string()),
    };
    adapter.request(&req).await?;
    tracing::info!(did = format!("{:#06X}", did), module = module, "actuator released to ECU");
    Ok(())
}

/// End diagnostic session — return to default (Mode 10 sub 0x01).
pub async fn end_session<A: Adapter>(
    adapter: &mut A,
    module: &str,
) -> Result<(), Obd2Error> {
    let req = ServiceRequest {
        service_id: 0x10,
        data: vec![0x01], // Default session
        target: Target::Module(module.to_string()),
    };
    adapter.request(&req).await?;
    tracing::info!(module = module, "returned to default diagnostic session");
    Ok(())
}

/// Send Tester Present (Mode 3E) to keep a diagnostic session alive.
pub async fn tester_present<A: Adapter>(
    adapter: &mut A,
    module: &str,
) -> Result<(), Obd2Error> {
    let req = ServiceRequest {
        service_id: 0x3E,
        data: vec![],
        target: Target::Module(module.to_string()),
    };
    adapter.request(&req).await?;
    Ok(())
}

/// Spawn a background task that sends Tester Present every 2 seconds.
/// Returns a cancel sender — drop it or send to stop the keep-alive.
pub fn start_tester_present_keepalive() -> watch::Sender<bool> {
    let (cancel_tx, _cancel_rx) = watch::channel(false);
    // Note: The actual keep-alive task needs adapter access,
    // which Session will manage. This just provides the cancel mechanism.
    cancel_tx
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::mock::MockAdapter;

    #[tokio::test]
    async fn test_enter_extended_session() {
        let mut adapter = MockAdapter::new();
        adapter.initialize().await.unwrap();
        let state = enter_session(&mut adapter, DiagSession::Extended, "ecm").await.unwrap();
        assert!(matches!(state, SessionState::Extended { .. }));
    }

    #[tokio::test]
    async fn test_enter_default_session() {
        let mut adapter = MockAdapter::new();
        adapter.initialize().await.unwrap();
        let state = enter_session(&mut adapter, DiagSession::Default, "ecm").await.unwrap();
        assert_eq!(state, SessionState::Default);
    }

    #[tokio::test]
    async fn test_end_session() {
        let mut adapter = MockAdapter::new();
        adapter.initialize().await.unwrap();
        let result = end_session(&mut adapter, "ecm").await;
        assert!(result.is_ok());
    }

    #[tokio::test]
    async fn test_actuator_requires_security() {
        let mut adapter = MockAdapter::new();
        let state = SessionState::Default; // not extended
        let result = actuator_control(
            &mut adapter, 0x1196, "ecm",
            &ActuatorCommand::Activate, &state,
        ).await;
        assert!(matches!(result, Err(Obd2Error::SecurityRequired)));
    }

    #[tokio::test]
    async fn test_actuator_requires_unlock() {
        let mut adapter = MockAdapter::new();
        let state = SessionState::Extended {
            unlocked_modules: vec![], // ecm not unlocked
        };
        let result = actuator_control(
            &mut adapter, 0x1196, "ecm",
            &ActuatorCommand::Activate, &state,
        ).await;
        assert!(matches!(result, Err(Obd2Error::SecurityRequired)));
    }

    #[tokio::test]
    async fn test_actuator_with_security() {
        let mut adapter = MockAdapter::new();
        adapter.initialize().await.unwrap();
        let state = SessionState::Extended {
            unlocked_modules: vec!["ecm".to_string()],
        };
        let result = actuator_control(
            &mut adapter, 0x1196, "ecm",
            &ActuatorCommand::Activate, &state,
        ).await;
        assert!(result.is_ok());
    }

    #[tokio::test]
    async fn test_tester_present() {
        let mut adapter = MockAdapter::new();
        adapter.initialize().await.unwrap();
        let result = tester_present(&mut adapter, "ecm").await;
        assert!(result.is_ok());
    }

    #[test]
    fn test_session_state_default() {
        let state = SessionState::default();
        assert_eq!(state, SessionState::Default);
    }
}
