//! Native CAN backend support.
//!
//! This module owns backend capability negotiation and the adapter-neutral
//! channel wrapper used by host-side CAN transports. Platform-specific handles
//! such as SocketCAN sockets or gs_usb devices plug in through the
//! [`CanFrameIo`](crate::transport::isotp::CanFrameIo) boundary; this module
//! does not open OS file descriptors itself.

use super::backend::{
    BackendCaps, CanFrameMode, CanIdentifier, CanRouteConfig, CapabilityMismatch,
    NegotiatedBackend, NATIVE_CAN_CLASSICAL_BACKEND_CAPS, NATIVE_CAN_FD_BACKEND_CAPS,
};
use crate::error::Obd2Error;
use crate::transport::isotp::{CanDataFrame, CanFrameIo};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum NativeCanDriver {
    SocketCan,
    GsUsb,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NativeCanBackend {
    driver: NativeCanDriver,
    interface: String,
    channels: u8,
    can_fd: bool,
}

impl NativeCanBackend {
    pub fn socketcan(interface: impl Into<String>) -> Self {
        Self {
            driver: NativeCanDriver::SocketCan,
            interface: interface.into(),
            channels: 1,
            can_fd: false,
        }
    }

    pub fn socketcan_fd(interface: impl Into<String>) -> Self {
        Self {
            can_fd: true,
            ..Self::socketcan(interface)
        }
    }

    pub fn gs_usb(interface: impl Into<String>) -> Self {
        Self {
            driver: NativeCanDriver::GsUsb,
            interface: interface.into(),
            channels: 1,
            can_fd: false,
        }
    }

    pub fn with_channels(mut self, channels: u8) -> Self {
        self.channels = channels;
        self
    }

    pub fn with_can_fd(mut self, can_fd: bool) -> Self {
        self.can_fd = can_fd;
        self
    }

    pub fn driver(&self) -> NativeCanDriver {
        self.driver
    }

    pub fn interface(&self) -> &str {
        &self.interface
    }

    pub fn capabilities(&self) -> BackendCaps {
        let mut caps = if self.can_fd {
            NATIVE_CAN_FD_BACKEND_CAPS
        } else {
            NATIVE_CAN_CLASSICAL_BACKEND_CAPS
        };
        caps.channels = self.channels;
        caps
    }

    pub fn plan_route(
        &self,
        route: CanRouteConfig,
    ) -> Result<NativeCanRoutePlan, CapabilityMismatch> {
        let negotiated = self.capabilities().negotiate(route.raw_can_requirement())?;
        Ok(NativeCanRoutePlan {
            driver: self.driver,
            interface: self.interface.clone(),
            negotiated,
            route,
        })
    }

    /// Bind a platform-specific classic CAN channel to a negotiated route.
    ///
    /// The returned wrapper filters inbound frames using the route filter set
    /// before handing them to ISO-TP or J1939 code. CAN-FD routes are rejected
    /// here because the current [`CanDataFrame`] type intentionally models
    /// classic 8-byte CAN frames only.
    pub fn bind_classical_io<T: CanFrameIo>(
        &self,
        route: CanRouteConfig,
        io: T,
    ) -> Result<NativeCanChannel<T>, Obd2Error> {
        if route.frame_mode.is_fd() {
            return Err(Obd2Error::Adapter(
                "native CAN classic channel cannot bind a CAN-FD route".into(),
            ));
        }
        let plan = self
            .plan_route(route)
            .map_err(|err| Obd2Error::Adapter(format!("native CAN route rejected: {:?}", err)))?;
        Ok(NativeCanChannel::new(plan, io))
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NativeCanRoutePlan {
    pub driver: NativeCanDriver,
    pub interface: String,
    pub negotiated: NegotiatedBackend,
    pub route: CanRouteConfig,
}

/// Adapter-neutral native CAN channel over a platform-specific frame source.
#[derive(Debug)]
pub struct NativeCanChannel<T> {
    plan: NativeCanRoutePlan,
    io: T,
    max_unrelated_frames: usize,
}

impl<T> NativeCanChannel<T> {
    pub fn new(plan: NativeCanRoutePlan, io: T) -> Self {
        Self {
            plan,
            io,
            max_unrelated_frames: 32,
        }
    }

    pub fn plan(&self) -> &NativeCanRoutePlan {
        &self.plan
    }

    pub fn io(&self) -> &T {
        &self.io
    }

    pub fn io_mut(&mut self) -> &mut T {
        &mut self.io
    }

    pub fn into_inner(self) -> T {
        self.io
    }

    pub fn with_max_unrelated_frames(mut self, max_unrelated_frames: usize) -> Self {
        self.max_unrelated_frames = max_unrelated_frames;
        self
    }
}

#[async_trait::async_trait]
impl<T: CanFrameIo> CanFrameIo for NativeCanChannel<T> {
    async fn send_frame(&mut self, frame: CanDataFrame) -> Result<(), Obd2Error> {
        if matches!(self.plan.route.frame_mode, CanFrameMode::Fd(_)) {
            return Err(Obd2Error::Adapter(
                "native CAN classic channel cannot send CAN-FD frames".into(),
            ));
        }
        validate_frame_identifier(&frame)?;
        self.io.send_frame(frame).await
    }

    async fn recv_frame(&mut self) -> Result<CanDataFrame, Obd2Error> {
        for _ in 0..=self.max_unrelated_frames {
            let frame = self.io.recv_frame().await?;
            let identifier = frame_identifier(&frame)?;
            if self.plan.route.accepts(identifier) {
                return Ok(frame);
            }
        }
        Err(Obd2Error::Timeout)
    }
}

fn validate_frame_identifier(frame: &CanDataFrame) -> Result<(), Obd2Error> {
    let _ = frame_identifier(frame)?;
    Ok(())
}

fn frame_identifier(frame: &CanDataFrame) -> Result<CanIdentifier, Obd2Error> {
    if frame.is_extended_id {
        CanIdentifier::extended(frame.id)
            .map_err(|err| Obd2Error::Adapter(format!("invalid extended CAN id: {:?}", err)))
    } else if frame.id <= 0x7FF {
        CanIdentifier::standard(frame.id as u16)
            .map_err(|err| Obd2Error::Adapter(format!("invalid standard CAN id: {:?}", err)))
    } else {
        Err(Obd2Error::Adapter(format!(
            "standard CAN frame id 0x{:X} exceeds 11 bits",
            frame.id
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::backend::{
        BackendKind, CanIdentifier, CanRouteConfig, CanRouteFilter, CapabilityMismatchKind,
        TransportKind,
    };
    use std::collections::VecDeque;

    #[derive(Debug)]
    struct ScriptedCanIo {
        rx: VecDeque<CanDataFrame>,
        tx: Vec<CanDataFrame>,
    }

    impl ScriptedCanIo {
        fn new(rx: impl IntoIterator<Item = CanDataFrame>) -> Self {
            Self {
                rx: rx.into_iter().collect(),
                tx: Vec::new(),
            }
        }
    }

    #[async_trait::async_trait]
    impl CanFrameIo for ScriptedCanIo {
        async fn send_frame(&mut self, frame: CanDataFrame) -> Result<(), Obd2Error> {
            self.tx.push(frame);
            Ok(())
        }

        async fn recv_frame(&mut self) -> Result<CanDataFrame, Obd2Error> {
            self.rx.pop_front().ok_or(Obd2Error::Timeout)
        }
    }

    #[test]
    fn classical_socketcan_reports_raw_can_without_fd() {
        let backend = NativeCanBackend::socketcan("can0");
        let caps = backend.capabilities();

        assert_eq!(caps.backend, BackendKind::NativeCan);
        assert!(caps.supports_transport(TransportKind::RawCan));
        assert!(caps.supports_transport(TransportKind::RawCanFd));
        assert!(!caps.can_fd);
    }

    #[test]
    fn socketcan_fd_plans_can_fd_route() {
        let plan = NativeCanBackend::socketcan_fd("can0")
            .plan_route(CanRouteConfig::can_fd(500_000, 5_000_000))
            .unwrap();

        assert_eq!(plan.driver, NativeCanDriver::SocketCan);
        assert_eq!(plan.interface, "can0");
        assert_eq!(plan.negotiated.transport, TransportKind::RawCanFd);
        assert!(plan.negotiated.can_fd);
    }

    #[test]
    fn classical_socketcan_rejects_can_fd_route() {
        let err = NativeCanBackend::socketcan("can0")
            .plan_route(CanRouteConfig::can_fd(500_000, 5_000_000))
            .unwrap_err();

        assert_eq!(err.kind, CapabilityMismatchKind::CanFdUnavailable);
    }

    #[test]
    fn native_can_plan_keeps_filters() {
        let filter = CanRouteFilter::new(
            CanIdentifier::standard(0x7E8).unwrap(),
            CanIdentifier::standard(0x7F8).unwrap(),
        )
        .unwrap();
        let route = CanRouteConfig::primary_classical(500_000).with_filter(filter);
        let plan = NativeCanBackend::gs_usb("can1").plan_route(route).unwrap();

        assert_eq!(plan.driver, NativeCanDriver::GsUsb);
        assert!(plan.route.accepts(CanIdentifier::standard(0x7EF).unwrap()));
        assert!(!plan.route.accepts(CanIdentifier::standard(0x7D0).unwrap()));
    }

    #[tokio::test]
    async fn native_can_channel_filters_unrelated_frames() {
        let filter = CanRouteFilter::exact(CanIdentifier::standard(0x7E8).unwrap());
        let route = CanRouteConfig::primary_classical(500_000).with_filter(filter);
        let io = ScriptedCanIo::new([
            CanDataFrame::new(0x123, [0x01]).unwrap(),
            CanDataFrame::new(0x7E8, [0x02]).unwrap(),
        ]);
        let mut channel = NativeCanBackend::socketcan("can0")
            .bind_classical_io(route, io)
            .unwrap()
            .with_max_unrelated_frames(4);

        let frame = channel.recv_frame().await.unwrap();
        assert_eq!(frame.id, 0x7E8);

        let sent = CanDataFrame::new(0x7E0, [0x03]).unwrap();
        channel.send_frame(sent.clone()).await.unwrap();
        assert_eq!(channel.io().tx, vec![sent]);
    }

    #[test]
    fn native_can_channel_rejects_fd_route_for_classic_io() {
        let io = ScriptedCanIo::new([]);
        let err = NativeCanBackend::socketcan_fd("can0")
            .bind_classical_io(CanRouteConfig::can_fd(500_000, 2_000_000), io)
            .unwrap_err();

        assert!(matches!(err, Obd2Error::Adapter(message) if message.contains("CAN-FD")));
    }

    #[tokio::test]
    async fn native_can_channel_rejects_standard_id_overflow() {
        let route = CanRouteConfig::primary_classical(500_000);
        let io = ScriptedCanIo::new([]);
        let mut channel = NativeCanBackend::socketcan("can0")
            .bind_classical_io(route, io)
            .unwrap();
        let frame = CanDataFrame::with_extended_id(0x800, false, [0x01]).unwrap();
        let err = channel.send_frame(frame).await.unwrap_err();

        assert!(matches!(err, Obd2Error::Adapter(message) if message.contains("exceeds 11 bits")));
    }
}
