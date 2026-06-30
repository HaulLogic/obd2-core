//! SQLite storage backend for obd2-core.
//!
//! Implements `VehicleStore` and `SessionStore` traits using rusqlite.
//!
//! # Example
//!
//! ```rust,no_run
//! use obd2_store_sqlite::SqliteStore;
//! use std::path::Path;
//!
//! let store = SqliteStore::open(Path::new("obd2.db")).unwrap();
//! ```

use async_trait::async_trait;
use obd2_core::error::Obd2Error;
use obd2_core::protocol::dtc::Dtc;
use obd2_core::protocol::enhanced::Reading;
use obd2_core::protocol::pid::Pid;
use obd2_core::store::{SessionStore, VehicleStore};
use obd2_core::vehicle::{ThresholdSet, VehicleProfile};
use rusqlite::{params, Connection};
use std::path::Path;
use std::sync::Mutex;

/// SQLite storage backend.
pub struct SqliteStore {
    conn: Mutex<Connection>,
}

impl SqliteStore {
    /// Open or create a SQLite database at the given path.
    pub fn open(path: &Path) -> Result<Self, Obd2Error> {
        let conn = Connection::open(path).map_err(|e| Obd2Error::Other(Box::new(e)))?;
        let store = Self {
            conn: Mutex::new(conn),
        };
        store.create_tables()?;
        Ok(store)
    }

    /// Create an in-memory SQLite database (for testing).
    pub fn in_memory() -> Result<Self, Obd2Error> {
        let conn = Connection::open_in_memory().map_err(|e| Obd2Error::Other(Box::new(e)))?;
        let store = Self {
            conn: Mutex::new(conn),
        };
        store.create_tables()?;
        Ok(store)
    }

    fn create_tables(&self) -> Result<(), Obd2Error> {
        let conn = self.conn.lock().map_err(|e| {
            Obd2Error::Other(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                e.to_string(),
            )))
        })?;
        conn.execute_batch(
            "CREATE TABLE IF NOT EXISTS vehicles (
                vin TEXT PRIMARY KEY,
                data TEXT NOT NULL,
                updated_at TEXT NOT NULL DEFAULT (datetime('now'))
            );

            CREATE TABLE IF NOT EXISTS thresholds (
                vin TEXT PRIMARY KEY,
                data TEXT NOT NULL,
                updated_at TEXT NOT NULL DEFAULT (datetime('now'))
            );

            CREATE TABLE IF NOT EXISTS readings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                vin TEXT NOT NULL,
                pid_code INTEGER NOT NULL,
                value REAL,
                unit TEXT,
                timestamp TEXT NOT NULL DEFAULT (datetime('now'))
            );

            CREATE TABLE IF NOT EXISTS dtc_events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                vin TEXT NOT NULL,
                dtc_codes TEXT NOT NULL,
                timestamp TEXT NOT NULL DEFAULT (datetime('now'))
            );

            CREATE INDEX IF NOT EXISTS idx_readings_vin ON readings(vin);
            CREATE INDEX IF NOT EXISTS idx_dtc_events_vin ON dtc_events(vin);",
        )
        .map_err(|e| Obd2Error::Other(Box::new(e)))?;
        Ok(())
    }
}

impl std::fmt::Debug for SqliteStore {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SqliteStore").finish()
    }
}

#[async_trait]
impl VehicleStore for SqliteStore {
    async fn save_vehicle(&self, profile: &VehicleProfile) -> Result<(), Obd2Error> {
        let vin = &profile.vin;
        let data = serde_json::to_string(&SerializableProfile::from(profile))
            .map_err(|e| Obd2Error::Other(Box::new(e)))?;

        let conn = self.conn.lock().map_err(|e| {
            Obd2Error::Other(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                e.to_string(),
            )))
        })?;
        conn.execute(
            "INSERT OR REPLACE INTO vehicles (vin, data) VALUES (?1, ?2)",
            params![vin, data],
        )
        .map_err(|e| Obd2Error::Other(Box::new(e)))?;

        Ok(())
    }

    async fn get_vehicle(&self, vin: &str) -> Result<Option<VehicleProfile>, Obd2Error> {
        let conn = self.conn.lock().map_err(|e| {
            Obd2Error::Other(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                e.to_string(),
            )))
        })?;
        let mut stmt = conn
            .prepare("SELECT data FROM vehicles WHERE vin = ?1")
            .map_err(|e| Obd2Error::Other(Box::new(e)))?;

        let result = stmt.query_row(params![vin], |row| {
            let data: String = row.get(0)?;
            Ok(data)
        });

        match result {
            Ok(data) => {
                let sp: SerializableProfile =
                    serde_json::from_str(&data).map_err(|e| Obd2Error::Other(Box::new(e)))?;
                Ok(Some(sp.into()))
            }
            Err(rusqlite::Error::QueryReturnedNoRows) => Ok(None),
            Err(e) => Err(Obd2Error::Other(Box::new(e))),
        }
    }

    async fn save_thresholds(&self, vin: &str, thresholds: &ThresholdSet) -> Result<(), Obd2Error> {
        let data = serde_json::to_string(thresholds).map_err(|e| Obd2Error::Other(Box::new(e)))?;

        let conn = self.conn.lock().map_err(|e| {
            Obd2Error::Other(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                e.to_string(),
            )))
        })?;
        conn.execute(
            "INSERT OR REPLACE INTO thresholds (vin, data) VALUES (?1, ?2)",
            params![vin, data],
        )
        .map_err(|e| Obd2Error::Other(Box::new(e)))?;

        Ok(())
    }

    async fn get_thresholds(&self, vin: &str) -> Result<Option<ThresholdSet>, Obd2Error> {
        let conn = self.conn.lock().map_err(|e| {
            Obd2Error::Other(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                e.to_string(),
            )))
        })?;
        let mut stmt = conn
            .prepare("SELECT data FROM thresholds WHERE vin = ?1")
            .map_err(|e| Obd2Error::Other(Box::new(e)))?;

        let result = stmt.query_row(params![vin], |row| {
            let data: String = row.get(0)?;
            Ok(data)
        });

        match result {
            Ok(data) => {
                let ts: ThresholdSet =
                    serde_json::from_str(&data).map_err(|e| Obd2Error::Other(Box::new(e)))?;
                Ok(Some(ts))
            }
            Err(rusqlite::Error::QueryReturnedNoRows) => Ok(None),
            Err(e) => Err(Obd2Error::Other(Box::new(e))),
        }
    }
}

#[async_trait]
impl SessionStore for SqliteStore {
    async fn save_reading(&self, vin: &str, pid: Pid, reading: &Reading) -> Result<(), Obd2Error> {
        let value = reading.value.as_f64().ok();
        let unit = reading.unit;

        let conn = self.conn.lock().map_err(|e| {
            Obd2Error::Other(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                e.to_string(),
            )))
        })?;
        conn.execute(
            "INSERT INTO readings (vin, pid_code, value, unit) VALUES (?1, ?2, ?3, ?4)",
            params![vin, pid.0, value, unit],
        )
        .map_err(|e| Obd2Error::Other(Box::new(e)))?;

        Ok(())
    }

    async fn save_dtc_event(&self, vin: &str, dtcs: &[Dtc]) -> Result<(), Obd2Error> {
        let codes: Vec<&str> = dtcs.iter().map(|d| d.code.as_str()).collect();
        let codes_str = codes.join(",");

        let conn = self.conn.lock().map_err(|e| {
            Obd2Error::Other(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                e.to_string(),
            )))
        })?;
        conn.execute(
            "INSERT INTO dtc_events (vin, dtc_codes) VALUES (?1, ?2)",
            params![vin, codes_str],
        )
        .map_err(|e| Obd2Error::Other(Box::new(e)))?;

        Ok(())
    }
}

/// Simplified serializable version of VehicleProfile for JSON storage.
#[derive(serde::Serialize, serde::Deserialize)]
struct SerializableProfile {
    vin: String,
    make: Option<String>,
    model: Option<String>,
    year: Option<i32>,
    engine_code: Option<String>,
    manufacturer: Option<String>,
    truck_class: Option<String>,
}

impl From<&VehicleProfile> for SerializableProfile {
    fn from(p: &VehicleProfile) -> Self {
        Self {
            vin: p.vin.clone(),
            make: None,
            model: None,
            year: p.decoded_vin.as_ref().and_then(|d| d.year),
            engine_code: p.spec.as_ref().map(|s| s.identity.engine.code.clone()),
            manufacturer: p.decoded_vin.as_ref().and_then(|d| d.manufacturer.clone()),
            truck_class: p.decoded_vin.as_ref().and_then(|d| d.truck_class.clone()),
        }
    }
}

impl From<SerializableProfile> for VehicleProfile {
    fn from(sp: SerializableProfile) -> Self {
        let decoded_vin =
            if sp.manufacturer.is_some() || sp.year.is_some() || sp.truck_class.is_some() {
                Some(obd2_core::vehicle::vin::DecodedVin {
                    year: sp.year,
                    year_alt: None,
                    manufacturer: sp.manufacturer,
                    truck_class: sp.truck_class,
                })
            } else {
                None
            };
        VehicleProfile {
            vin: sp.vin,
            decoded_vin,
            info: None,
            spec: None,
            supported_pids: std::collections::HashSet::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use obd2_core::protocol::enhanced::{ReadingSource, Value};
    use std::time::Instant;

    #[tokio::test]
    async fn test_create_store() {
        let store = SqliteStore::in_memory().unwrap();
        let conn = store.conn.lock().unwrap();
        assert!(conn.is_autocommit());
    }

    #[tokio::test]
    async fn test_save_and_get_vehicle() {
        let store = SqliteStore::in_memory().unwrap();
        let profile = VehicleProfile {
            vin: "1GCHK23224F000001".into(),
            decoded_vin: None,
            info: None,
            spec: None,
            supported_pids: std::collections::HashSet::new(),
        };

        store.save_vehicle(&profile).await.unwrap();
        let retrieved = store.get_vehicle("1GCHK23224F000001").await.unwrap();
        assert!(retrieved.is_some());
        assert_eq!(retrieved.unwrap().vin, "1GCHK23224F000001");
    }

    #[tokio::test]
    async fn test_get_vehicle_not_found() {
        let store = SqliteStore::in_memory().unwrap();
        let result = store.get_vehicle("NONEXISTENT").await.unwrap();
        assert!(result.is_none());
    }

    #[tokio::test]
    async fn test_save_reading() {
        let store = SqliteStore::in_memory().unwrap();
        let reading = Reading {
            value: Value::Scalar(680.0),
            unit: "RPM",
            timestamp: Instant::now(),
            raw_bytes: vec![0x0A, 0xA0],
            source: ReadingSource::Live,
        };

        store
            .save_reading("1GCHK23224F000001", Pid::ENGINE_RPM, &reading)
            .await
            .unwrap();

        // Verify it was saved
        let conn = store.conn.lock().unwrap();
        let count: i64 = conn
            .query_row(
                "SELECT COUNT(*) FROM readings WHERE vin = '1GCHK23224F000001'",
                [],
                |row| row.get(0),
            )
            .unwrap();
        assert_eq!(count, 1);
    }

    #[tokio::test]
    async fn test_save_dtc_event() {
        let store = SqliteStore::in_memory().unwrap();
        let dtcs = vec![Dtc::from_code("P0420"), Dtc::from_code("P0171")];

        store
            .save_dtc_event("1GCHK23224F000001", &dtcs)
            .await
            .unwrap();

        let conn = store.conn.lock().unwrap();
        let codes: String = conn
            .query_row(
                "SELECT dtc_codes FROM dtc_events WHERE vin = '1GCHK23224F000001'",
                [],
                |row| row.get(0),
            )
            .unwrap();
        assert!(codes.contains("P0420"));
        assert!(codes.contains("P0171"));
    }

    #[tokio::test]
    async fn test_save_and_get_thresholds() {
        let store = SqliteStore::in_memory().unwrap();
        let ts = ThresholdSet {
            engine: vec![],
            transmission: vec![],
        };

        store
            .save_thresholds("TEST_VIN_12345678", &ts)
            .await
            .unwrap();
        let retrieved = store.get_thresholds("TEST_VIN_12345678").await.unwrap();
        assert!(retrieved.is_some());
    }

    #[tokio::test]
    async fn test_upsert_vehicle() {
        let store = SqliteStore::in_memory().unwrap();
        let profile = VehicleProfile {
            vin: "1GCHK23224F000001".into(),
            decoded_vin: None,
            info: None,
            spec: None,
            supported_pids: std::collections::HashSet::new(),
        };

        // Save twice -- should upsert, not error
        store.save_vehicle(&profile).await.unwrap();
        store.save_vehicle(&profile).await.unwrap();

        let conn = store.conn.lock().unwrap();
        let count: i64 = conn
            .query_row("SELECT COUNT(*) FROM vehicles", [], |row| row.get(0))
            .unwrap();
        assert_eq!(count, 1);
    }
}
