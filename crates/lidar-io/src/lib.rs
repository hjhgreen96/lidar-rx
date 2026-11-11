//! 입력 출처 구현 (경계 1의 구체 어댑터들).
//!
//! ```text
//!  pcap 파일 ─► pcap::PcapReader ─► 레코드(캡처 시각 + Ethernet 프레임)
//!                                      │ net::parse_udp     Ethernet → IPv4 → UDP 헤더를 벗김
//!                                      ▼
//!                               PcapSource ─► lidar_core::PacketSource
//!
//!  UDP 소켓 ─► udp::UdpSource ─► lidar_core::PacketSource   (리눅스 전용: 커널 수신 시각·소켓 드롭 수)
//!
//!  어떤 출처든 ─► forward::ForwardSource ─► lidar_core::PacketSource   받은 패킷을 다른 주소로도 보냄 (보드 → PC)
//! ```
//!
//! 자세한 설명: `docs/modules/lidar-io.md`

pub mod forward;
pub mod net;
pub mod pcap;
mod pcap_source;
#[cfg(target_os = "linux")]
pub mod udp;

pub use forward::{ForwardCounts, ForwardSource};
pub use pcap_source::PcapSource;
#[cfg(target_os = "linux")]
pub use udp::{UdpOptions, UdpSource};
