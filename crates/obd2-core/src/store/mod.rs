//! Storage traits for persisting vehicle and session data.
//!
//! obd2-core defines traits only. Implementations live in separate crates
//! (e.g., `obd2-store-sqlite`) or are provided by consumers (e.g., HaulLogic
//! might implement `VehicleStore` against its own PostgreSQL database).
//!
//! # Design Rationale
//!
//! Keeping storage out of the core library means:
//! - Core has no SQLite/database dependency
//! - Consumers choose their own storage backend
//! - Mobile apps can use platform-native storage (Core Data, Room)
//! - The library works without any persistence at all

use async_trait::async_trait;
use crate::error::Obd2Error;
use crate::protocol::pid::Pid;
use crate::protocol::enhanced::Reading;
use crate::protocol::dtc::Dtc;
use crate::vehicle::{VehicleProfile, ThresholdSet};

/// Persist and retrieve vehicle profiles and threshold overrides.
///
/// A vehicle profile includes the VIN, decoded vehicle info, matched spec,
/// and supported PID set. Threshold overrides allow per-VIN customization
/// of alert limits beyond what the spec defines.
#[async_trait]
pub trait VehicleStore: Send + Sync {
    /// Save or update a vehicle profile.
    ///
    /// If a profile with the same VIN already exists, it is updated.
    async fn save_vehicle(&self, profile: &VehicleProfile) -> Result<(), Obd2Error>;

    /// Retrieve a vehicle profile by VIN.
    ///
    /// Returns `None` if the VIN is not found in storage.
    async fn get_vehicle(&self, vin: &str) -> Result<Option<VehicleProfile>, Obd2Error>;

    /// Save per-VIN threshold overrides.
    ///
    /// These take priority over spec-defined thresholds (BR-5.1).
    async fn save_thresholds(&self, vin: &str, thresholds: &ThresholdSet) -> Result<(), Obd2Error>;

    /// Retrieve per-VIN threshold overrides.
    ///
    /// Returns `None` if no overrides exist for this VIN.
    async fn get_thresholds(&self, vin: &str) -> Result<Option<ThresholdSet>, Obd2Error>;
}

/// Persist diagnostic session data for history and analysis.
///
/// Session data includes PID readings and DTC events captured during
/// a diagnostic session. This enables historical trending, baseline
/// learning, and post-session review.
#[async_trait]
pub trait SessionStore: Send + Sync {
    /// Save a PID reading to the session history.
    async fn save_reading(&self, vin: &str, pid: Pid, reading: &Reading) -> Result<(), Obd2Error>;

    /// Save a DTC scan event to the session history.
    async fn save_dtc_event(&self, vin: &str, dtcs: &[Dtc]) -> Result<(), Obd2Error>;
}
