//! 센서와 무관한 공통 타입과 경계(인터페이스).
//!
//! 이 크레이트는 **무엇을 주고받는가**만 정하고, **어떻게 하는가**는 다른 크레이트가 구현한다.
//! 그래서 의존 방향이 항상 이 크레이트를 향한다 (`vlp16`, `lidar-io`, `rx` → `lidar-core`).
//!
//! ```text
//!   경계 1: 입력 출처          경계 2: 센서 모델             경계 3: 소비자
//!  ┌──────────────┐  Packet  ┌───────────────┐  Frame   ┌──────────────┐
//!  │ PacketSource │ ───────► │ SensorDecoder │ ───────► │ ROS 2, 통계…  │
//!  └──────────────┘          └───────────────┘          └──────────────┘
//!   UDP 소켓 / pcap 파일        VLP-16 / (다른 모델)        (rx에서 채널로 연결)
//! ```
//!
//! 자세한 설명: `docs/modules/lidar-core.md`

mod decoder;
mod frame;
mod source;
mod time;

pub use decoder::{DecodeError, DecoderCounters, SensorDecoder};
pub use frame::{Frame, FrameStats, PacketTime, Point};
pub use source::{Packet, PacketSource, Received};
pub use time::{HostTime, SensorTime};
