//! Backend capability negotiation and CAN route configuration.
//!
//! These types describe what a backend can realize before any protocol client
//! commits to a route. They intentionally do not perform I/O.

use super::{AdapterInfo, Chipset};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum BackendKind {
    Elm327,
    Stn,
    NativeCan,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum LinkKind {
    Serial,
    Ble,
    Usb,
    Tcp,
    NativeCan,
    Mock,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum TransportKind {
    IsoTp,
    J1850Vpw,
    J1850Pwm,
    KLine,
    J1939,
    DoIp,
    RawCan,
    RawCanFd,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum SecondaryBus {
    MsCan125,
    SwCanGmlan33,
    MsGmlan95,
}

impl SecondaryBus {
    pub fn bitrate_bps(self) -> u32 {
        match self {
            Self::MsCan125 => 125_000,
            Self::SwCanGmlan33 => 33_300,
            Self::MsGmlan95 => 95_000,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct QuirkFlags {
    pub elm_prompt_framing: bool,
    pub elm_clone_version_lie: bool,
    pub small_rx_buffer: bool,
    pub stn_command_set: bool,
    pub native_can_interface: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BackendCaps {
    pub backend: BackendKind,
    pub links: &'static [LinkKind],
    pub transports: &'static [TransportKind],
    pub can_fd: bool,
    pub channels: u8,
    pub secondary_can: &'static [SecondaryBus],
    pub max_pdu: usize,
    pub quirks: QuirkFlags,
}

impl BackendCaps {
    pub fn supports_link(self, link: LinkKind) -> bool {
        self.links.contains(&link)
    }

    pub fn supports_transport(self, transport: TransportKind) -> bool {
        self.transports.contains(&transport)
    }

    pub fn supports_secondary_bus(self, bus: SecondaryBus) -> bool {
        self.secondary_can.contains(&bus)
    }

    pub fn negotiate(
        self,
        request: CapabilityRequest,
    ) -> Result<NegotiatedBackend, CapabilityMismatch> {
        if let Some(link) = request.link {
            if !self.supports_link(link) {
                return Err(CapabilityMismatch::new(
                    self.backend,
                    CapabilityMismatchKind::UnsupportedLink(link),
                ));
            }
        }

        if !self.supports_transport(request.transport) {
            return Err(CapabilityMismatch::new(
                self.backend,
                CapabilityMismatchKind::UnsupportedTransport(request.transport),
            ));
        }

        if request.can_fd && !self.can_fd {
            return Err(CapabilityMismatch::new(
                self.backend,
                CapabilityMismatchKind::CanFdUnavailable,
            ));
        }

        if let Some(bus) = request.secondary_bus {
            if !self.supports_secondary_bus(bus) {
                return Err(CapabilityMismatch::new(
                    self.backend,
                    CapabilityMismatchKind::SecondaryCanUnavailable(bus),
                ));
            }
        }

        if self.channels < request.min_channels {
            return Err(CapabilityMismatch::new(
                self.backend,
                CapabilityMismatchKind::InsufficientChannels {
                    required: request.min_channels,
                    available: self.channels,
                },
            ));
        }

        if self.max_pdu < request.min_pdu {
            return Err(CapabilityMismatch::new(
                self.backend,
                CapabilityMismatchKind::InsufficientPdu {
                    required: request.min_pdu,
                    available: self.max_pdu,
                },
            ));
        }

        Ok(NegotiatedBackend {
            backend: self.backend,
            link: request.link.or_else(|| self.links.first().copied()),
            transport: request.transport,
            secondary_bus: request.secondary_bus,
            can_fd: request.can_fd,
            channels: self.channels,
            max_pdu: self.max_pdu,
            quirks: self.quirks,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CapabilityRequest {
    pub link: Option<LinkKind>,
    pub transport: TransportKind,
    pub secondary_bus: Option<SecondaryBus>,
    pub can_fd: bool,
    pub min_channels: u8,
    pub min_pdu: usize,
}

impl CapabilityRequest {
    pub fn new(transport: TransportKind) -> Self {
        Self {
            link: None,
            transport,
            secondary_bus: None,
            can_fd: false,
            min_channels: 1,
            min_pdu: 0,
        }
    }

    pub fn with_link(mut self, link: LinkKind) -> Self {
        self.link = Some(link);
        self
    }

    pub fn with_secondary_bus(mut self, bus: SecondaryBus) -> Self {
        self.secondary_bus = Some(bus);
        self
    }

    pub fn require_can_fd(mut self) -> Self {
        self.can_fd = true;
        self
    }

    pub fn with_min_channels(mut self, min_channels: u8) -> Self {
        self.min_channels = min_channels;
        self
    }

    pub fn with_min_pdu(mut self, min_pdu: usize) -> Self {
        self.min_pdu = min_pdu;
        self
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NegotiatedBackend {
    pub backend: BackendKind,
    pub link: Option<LinkKind>,
    pub transport: TransportKind,
    pub secondary_bus: Option<SecondaryBus>,
    pub can_fd: bool,
    pub channels: u8,
    pub max_pdu: usize,
    pub quirks: QuirkFlags,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CapabilityMismatch {
    pub backend: BackendKind,
    pub kind: CapabilityMismatchKind,
}

impl CapabilityMismatch {
    fn new(backend: BackendKind, kind: CapabilityMismatchKind) -> Self {
        Self { backend, kind }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum CapabilityMismatchKind {
    UnsupportedLink(LinkKind),
    UnsupportedTransport(TransportKind),
    SecondaryCanUnavailable(SecondaryBus),
    CanFdUnavailable,
    InsufficientChannels { required: u8, available: u8 },
    InsufficientPdu { required: usize, available: usize },
}

const ELM_AT_LINKS: [LinkKind; 2] = [LinkKind::Serial, LinkKind::Ble];
const ELM_AT_TRANSPORTS: [TransportKind; 5] = [
    TransportKind::IsoTp,
    TransportKind::J1850Vpw,
    TransportKind::J1850Pwm,
    TransportKind::KLine,
    TransportKind::J1939,
];
const STN_SECONDARY_CAN: [SecondaryBus; 3] = [
    SecondaryBus::MsCan125,
    SecondaryBus::SwCanGmlan33,
    SecondaryBus::MsGmlan95,
];
const NATIVE_CAN_LINKS: [LinkKind; 2] = [LinkKind::NativeCan, LinkKind::Usb];
const NATIVE_CAN_TRANSPORTS: [TransportKind; 4] = [
    TransportKind::IsoTp,
    TransportKind::J1939,
    TransportKind::RawCan,
    TransportKind::RawCanFd,
];
const NATIVE_CAN_SECONDARY: [SecondaryBus; 3] = [
    SecondaryBus::MsCan125,
    SecondaryBus::SwCanGmlan33,
    SecondaryBus::MsGmlan95,
];
const NO_LINKS: [LinkKind; 0] = [];
const NO_TRANSPORTS: [TransportKind; 0] = [];
const NO_SECONDARY_CAN: [SecondaryBus; 0] = [];

pub const ELM327_CLONE_BACKEND_CAPS: BackendCaps = BackendCaps {
    backend: BackendKind::Elm327,
    links: &ELM_AT_LINKS,
    transports: &ELM_AT_TRANSPORTS,
    can_fd: false,
    channels: 1,
    secondary_can: &NO_SECONDARY_CAN,
    max_pdu: 512,
    quirks: QuirkFlags {
        elm_prompt_framing: true,
        elm_clone_version_lie: true,
        small_rx_buffer: true,
        stn_command_set: false,
        native_can_interface: false,
    },
};

pub const ELM327_GENUINE_BACKEND_CAPS: BackendCaps = BackendCaps {
    backend: BackendKind::Elm327,
    links: &ELM_AT_LINKS,
    transports: &ELM_AT_TRANSPORTS,
    can_fd: false,
    channels: 1,
    secondary_can: &NO_SECONDARY_CAN,
    max_pdu: 512,
    quirks: QuirkFlags {
        elm_prompt_framing: true,
        elm_clone_version_lie: false,
        small_rx_buffer: true,
        stn_command_set: false,
        native_can_interface: false,
    },
};

pub const STN_BACKEND_CAPS: BackendCaps = BackendCaps {
    backend: BackendKind::Stn,
    links: &ELM_AT_LINKS,
    transports: &ELM_AT_TRANSPORTS,
    can_fd: false,
    channels: 1,
    secondary_can: &STN_SECONDARY_CAN,
    max_pdu: 4096,
    quirks: QuirkFlags {
        elm_prompt_framing: true,
        elm_clone_version_lie: false,
        small_rx_buffer: false,
        stn_command_set: true,
        native_can_interface: false,
    },
};

pub const NATIVE_CAN_CLASSICAL_BACKEND_CAPS: BackendCaps = BackendCaps {
    backend: BackendKind::NativeCan,
    links: &NATIVE_CAN_LINKS,
    transports: &NATIVE_CAN_TRANSPORTS,
    can_fd: false,
    channels: 1,
    secondary_can: &NATIVE_CAN_SECONDARY,
    max_pdu: 4096,
    quirks: QuirkFlags {
        elm_prompt_framing: false,
        elm_clone_version_lie: false,
        small_rx_buffer: false,
        stn_command_set: false,
        native_can_interface: true,
    },
};

pub const NATIVE_CAN_FD_BACKEND_CAPS: BackendCaps = BackendCaps {
    can_fd: true,
    ..NATIVE_CAN_CLASSICAL_BACKEND_CAPS
};

pub const UNKNOWN_BACKEND_CAPS: BackendCaps = BackendCaps {
    backend: BackendKind::Unknown,
    links: &NO_LINKS,
    transports: &NO_TRANSPORTS,
    can_fd: false,
    channels: 0,
    secondary_can: &NO_SECONDARY_CAN,
    max_pdu: 0,
    quirks: QuirkFlags {
        elm_prompt_framing: false,
        elm_clone_version_lie: false,
        small_rx_buffer: false,
        stn_command_set: false,
        native_can_interface: false,
    },
};

impl AdapterInfo {
    pub fn backend_caps(&self) -> BackendCaps {
        match self.chipset {
            Chipset::Elm327Clone => ELM327_CLONE_BACKEND_CAPS,
            Chipset::Elm327Genuine => ELM327_GENUINE_BACKEND_CAPS,
            Chipset::Stn => STN_BACKEND_CAPS,
            Chipset::Unknown => UNKNOWN_BACKEND_CAPS,
        }
    }

    pub fn negotiate_backend(
        &self,
        request: CapabilityRequest,
    ) -> Result<NegotiatedBackend, CapabilityMismatch> {
        self.backend_caps().negotiate(request)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanIdentifier {
    Standard(u16),
    Extended(u32),
}

impl CanIdentifier {
    pub fn standard(id: u16) -> Result<Self, CanRouteError> {
        if id <= 0x7FF {
            Ok(Self::Standard(id))
        } else {
            Err(CanRouteError::IdentifierOutOfRange {
                id: id as u32,
                bits: 11,
            })
        }
    }

    pub fn extended(id: u32) -> Result<Self, CanRouteError> {
        if id <= 0x1FFF_FFFF {
            Ok(Self::Extended(id))
        } else {
            Err(CanRouteError::IdentifierOutOfRange { id, bits: 29 })
        }
    }

    pub fn raw(self) -> u32 {
        match self {
            Self::Standard(id) => id as u32,
            Self::Extended(id) => id,
        }
    }

    pub fn max_mask(self) -> Self {
        match self {
            Self::Standard(_) => Self::Standard(0x7FF),
            Self::Extended(_) => Self::Extended(0x1FFF_FFFF),
        }
    }

    fn same_width(self, other: Self) -> bool {
        matches!(
            (self, other),
            (Self::Standard(_), Self::Standard(_)) | (Self::Extended(_), Self::Extended(_))
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanRouteFilter {
    pub id: CanIdentifier,
    pub mask: CanIdentifier,
}

impl CanRouteFilter {
    pub fn new(id: CanIdentifier, mask: CanIdentifier) -> Result<Self, CanRouteError> {
        if !id.same_width(mask) {
            return Err(CanRouteError::MixedIdentifierWidth);
        }
        Ok(Self { id, mask })
    }

    pub fn exact(id: CanIdentifier) -> Self {
        Self {
            id,
            mask: id.max_mask(),
        }
    }

    pub fn matches(self, frame_id: CanIdentifier) -> bool {
        if !self.id.same_width(frame_id) {
            return false;
        }
        (frame_id.raw() & self.mask.raw()) == (self.id.raw() & self.mask.raw())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct CanFilterSet {
    filters: Vec<CanRouteFilter>,
}

impl CanFilterSet {
    pub fn new() -> Self {
        Self {
            filters: Vec::new(),
        }
    }

    pub fn from_filters(filters: Vec<CanRouteFilter>) -> Self {
        Self { filters }
    }

    pub fn push(&mut self, filter: CanRouteFilter) {
        self.filters.push(filter);
    }

    pub fn filters(&self) -> &[CanRouteFilter] {
        &self.filters
    }

    pub fn is_empty(&self) -> bool {
        self.filters.is_empty()
    }

    pub fn accepts(&self, frame_id: CanIdentifier) -> bool {
        self.filters.is_empty() || self.filters.iter().any(|filter| filter.matches(frame_id))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFdConfig {
    pub data_bitrate_bps: u32,
    pub bitrate_switch: bool,
    pub iso_mode: bool,
}

impl CanFdConfig {
    pub fn iso(data_bitrate_bps: u32) -> Self {
        Self {
            data_bitrate_bps,
            bitrate_switch: true,
            iso_mode: true,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanFrameMode {
    Classical,
    Fd(CanFdConfig),
}

impl CanFrameMode {
    pub fn is_fd(self) -> bool {
        matches!(self, Self::Fd(_))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanRouteBus {
    Primary,
    Secondary(SecondaryBus),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanRouteConfig {
    pub bus: CanRouteBus,
    pub arbitration_bitrate_bps: u32,
    pub frame_mode: CanFrameMode,
    pub filters: CanFilterSet,
}

impl CanRouteConfig {
    pub fn primary_classical(arbitration_bitrate_bps: u32) -> Self {
        Self {
            bus: CanRouteBus::Primary,
            arbitration_bitrate_bps,
            frame_mode: CanFrameMode::Classical,
            filters: CanFilterSet::new(),
        }
    }

    pub fn ms_can_125k() -> Self {
        Self::secondary_classical(SecondaryBus::MsCan125)
    }

    pub fn sw_can_gmlan_33k() -> Self {
        Self::secondary_classical(SecondaryBus::SwCanGmlan33)
    }

    pub fn ms_gmlan_95k() -> Self {
        Self::secondary_classical(SecondaryBus::MsGmlan95)
    }

    pub fn can_fd(arbitration_bitrate_bps: u32, data_bitrate_bps: u32) -> Self {
        Self {
            bus: CanRouteBus::Primary,
            arbitration_bitrate_bps,
            frame_mode: CanFrameMode::Fd(CanFdConfig::iso(data_bitrate_bps)),
            filters: CanFilterSet::new(),
        }
    }

    pub fn with_filter(mut self, filter: CanRouteFilter) -> Self {
        self.filters.push(filter);
        self
    }

    pub fn accepts(&self, frame_id: CanIdentifier) -> bool {
        self.filters.accepts(frame_id)
    }

    pub fn secondary_bus(&self) -> Option<SecondaryBus> {
        match self.bus {
            CanRouteBus::Primary => None,
            CanRouteBus::Secondary(bus) => Some(bus),
        }
    }

    pub fn raw_can_requirement(&self) -> CapabilityRequest {
        let mut request = if self.frame_mode.is_fd() {
            CapabilityRequest::new(TransportKind::RawCanFd).require_can_fd()
        } else {
            CapabilityRequest::new(TransportKind::RawCan)
        };

        if let Some(bus) = self.secondary_bus() {
            request = request.with_secondary_bus(bus);
        }
        request.with_min_pdu(if self.frame_mode.is_fd() { 64 } else { 8 })
    }

    pub fn isotp_requirement(&self) -> CapabilityRequest {
        let mut request = CapabilityRequest::new(TransportKind::IsoTp);
        if self.frame_mode.is_fd() {
            request = request.require_can_fd();
        }
        if let Some(bus) = self.secondary_bus() {
            request = request.with_secondary_bus(bus);
        }
        request
    }

    fn secondary_classical(bus: SecondaryBus) -> Self {
        Self {
            bus: CanRouteBus::Secondary(bus),
            arbitration_bitrate_bps: bus.bitrate_bps(),
            frame_mode: CanFrameMode::Classical,
            filters: CanFilterSet::new(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum CanRouteError {
    IdentifierOutOfRange { id: u32, bits: u8 },
    MixedIdentifierWidth,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::{Capabilities, Chipset};
    use crate::vehicle::Protocol;

    fn adapter_info(chipset: Chipset) -> AdapterInfo {
        AdapterInfo {
            chipset,
            firmware: String::new(),
            protocol: Protocol::Auto,
            capabilities: Capabilities::default(),
        }
    }

    #[test]
    fn stn_negotiates_ms_can_isotp() {
        let route = CanRouteConfig::ms_can_125k();
        let negotiated = adapter_info(Chipset::Stn)
            .negotiate_backend(route.isotp_requirement())
            .unwrap();

        assert_eq!(negotiated.backend, BackendKind::Stn);
        assert_eq!(negotiated.transport, TransportKind::IsoTp);
        assert_eq!(negotiated.secondary_bus, Some(SecondaryBus::MsCan125));
        assert!(!negotiated.can_fd);
    }

    #[test]
    fn elm_rejects_secondary_can_route() {
        let route = CanRouteConfig::sw_can_gmlan_33k();
        let err = adapter_info(Chipset::Elm327Genuine)
            .negotiate_backend(route.isotp_requirement())
            .unwrap_err();

        assert_eq!(
            err.kind,
            CapabilityMismatchKind::SecondaryCanUnavailable(SecondaryBus::SwCanGmlan33)
        );
    }

    #[test]
    fn stn_rejects_can_fd_route() {
        let route = CanRouteConfig::can_fd(500_000, 2_000_000);
        let err = adapter_info(Chipset::Stn)
            .negotiate_backend(route.isotp_requirement())
            .unwrap_err();

        assert_eq!(err.kind, CapabilityMismatchKind::CanFdUnavailable);
    }

    #[test]
    fn native_can_fd_negotiates_raw_fd() {
        let route = CanRouteConfig::can_fd(500_000, 5_000_000);
        let negotiated = NATIVE_CAN_FD_BACKEND_CAPS
            .negotiate(route.raw_can_requirement())
            .unwrap();

        assert_eq!(negotiated.backend, BackendKind::NativeCan);
        assert_eq!(negotiated.transport, TransportKind::RawCanFd);
        assert!(negotiated.can_fd);
    }

    #[test]
    fn standard_filter_accepts_masked_range() {
        let filter = CanRouteFilter::new(
            CanIdentifier::standard(0x7E8).unwrap(),
            CanIdentifier::standard(0x7F8).unwrap(),
        )
        .unwrap();
        let set = CanFilterSet::from_filters(vec![filter]);

        assert!(set.accepts(CanIdentifier::standard(0x7E8).unwrap()));
        assert!(set.accepts(CanIdentifier::standard(0x7EF).unwrap()));
        assert!(!set.accepts(CanIdentifier::standard(0x7D8).unwrap()));
    }

    #[test]
    fn extended_filter_does_not_match_standard_id() {
        let filter = CanRouteFilter::exact(CanIdentifier::extended(0x18DA_F110).unwrap());

        assert!(filter.matches(CanIdentifier::extended(0x18DA_F110).unwrap()));
        assert!(!filter.matches(CanIdentifier::standard(0x7E8).unwrap()));
    }

    #[test]
    fn mixed_filter_width_is_rejected() {
        let err = CanRouteFilter::new(
            CanIdentifier::standard(0x7E8).unwrap(),
            CanIdentifier::extended(0x1FFF_FFFF).unwrap(),
        )
        .unwrap_err();

        assert_eq!(err, CanRouteError::MixedIdentifierWidth);
    }

    #[test]
    fn identifier_range_is_checked() {
        let err = CanIdentifier::standard(0x800).unwrap_err();

        assert_eq!(
            err,
            CanRouteError::IdentifierOutOfRange {
                id: 0x800,
                bits: 11
            }
        );
    }
}
