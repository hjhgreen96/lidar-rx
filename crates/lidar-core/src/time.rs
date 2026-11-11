//! 이 프로젝트에 나오는 두 시계.
//!
//! ```text
//!  센서 시계  ── SensorTime ──  센서 안의 시계. 패킷마다 "정시 이후 µs"로 찍혀 옴. 아주 일정하게 감
//!  호스트 시계 ── HostTime  ──  로봇 컴퓨터의 CLOCK_REALTIME. ROS 2 메시지 시각, 커널 수신 시각이 이 시계
//! ```
//!
//! 둘은 서로 다른 시계라 기준점과 빠르기가 다르다. 둘 사이의 관계(기울기·오프셋)를
//! 추정하는 것이 `clock` 크레이트의 일이다.

/// 호스트 시계의 시각: 1970-01-01 00:00:00 UTC부터의 나노초 (`CLOCK_REALTIME`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct HostTime(pub i64);

impl HostTime {
    pub fn from_secs_nanos(secs: i64, nanos: u32) -> Self {
        HostTime(secs * 1_000_000_000 + nanos as i64)
    }

    pub fn as_secs_f64(self) -> f64 {
        self.0 as f64 * 1e-9
    }
}

/// 센서 시계의 시각, 마이크로초.
///
/// 센서는 "정시 이후 µs"(1시간마다 0으로 돌아감)만 보낸다. 정시 넘김을 이어 붙여
/// 계속 증가하는 값으로 만든 것이 이 타입이다. 0이 언제인지는 의미가 없고, **차이만** 의미가 있다.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SensorTime(pub i64);

impl SensorTime {
    /// `self - earlier`, 마이크로초.
    pub fn micros_since(self, earlier: SensorTime) -> i64 {
        self.0 - earlier.0
    }
}
