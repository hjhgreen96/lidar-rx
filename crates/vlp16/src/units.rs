//! 패킷에 들어 있는 원시값의 단위를 타입으로 구분한다 (Newtype 패턴).
//!
//! 방위각(0.01° 단위)과 거리(2 mm 단위)는 둘 다 `u16`이라, `u16` 그대로 다루면 서로 바꿔 넣어도
//! 컴파일러가 모른다. 각각 다른 타입으로 감싸면 잘못 넣는 순간 컴파일 에러가 난다.

/// 방위각 원시값, 0.01° 단위 (0..=35999). 센서 정면(+y)에서 위에서 보아 시계방향.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Azimuth(pub u16);

impl Azimuth {
    /// 한 바퀴 = 36000 (0.01° 단위)
    pub const FULL_TURN: u16 = 36000;

    pub fn degrees(self) -> f64 {
        self.0 as f64 / 100.0
    }

    /// `self`에서 `later`까지 정방향으로 돈 각도 (0.01° 단위, 0..36000). 0°를 넘어가도 맞게 계산.
    pub fn forward_to(self, later: Azimuth) -> u16 {
        (later.0 as i32 - self.0 as i32).rem_euclid(Self::FULL_TURN as i32) as u16
    }
}

/// 거리 원시값, 2 mm 단위. 0이면 "반사 없음" (측정 실패).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct RawDistance(pub u16);

impl RawDistance {
    /// 원시값 1 = 2 mm
    pub const UNIT_M: f32 = 0.002;

    pub fn is_no_return(self) -> bool {
        self.0 == 0
    }

    pub fn meters(self) -> f32 {
        self.0 as f32 * Self::UNIT_M
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn forward_handles_wrap_at_zero() {
        assert_eq!(Azimuth(35990).forward_to(Azimuth(30)), 40);
        assert_eq!(Azimuth(100).forward_to(Azimuth(140)), 40);
        assert_eq!(Azimuth(100).forward_to(Azimuth(100)), 0);
    }

    #[test]
    fn units() {
        assert_eq!(Azimuth(12345).degrees(), 123.45);
        assert_eq!(RawDistance(500).meters(), 1.0);
        assert!(RawDistance(0).is_no_return());
    }
}
