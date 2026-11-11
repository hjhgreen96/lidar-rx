//! 발사 타이밍과 센서 시계.
//!
//! ```text
//!  발사(firing) 1회 = 레이저 16개를 2.304 µs 간격으로 차례로 쏨 (16 × 2.304 = 36.864 µs)
//!                    + 다음 발사 전 충전 18.432 µs  →  주기 55.296 µs
//!
//!  ──┬─┬─┬─ … ─┬────────────┬─┬─┬─ … ─┬────────────┬── 시간
//!    0 1 2     15  (충전)    0 1 2     15  (충전)        ← 레이저 번호
//!    ├────── 발사 0 ────────┤├────── 발사 1 ────────┤
//!    ↑ 패킷 타임스탬프 = 발사 0의 레이저 0 시각
//!
//!  블록 b (단일 반환) = 발사 2b (채널 0..15) + 발사 2b+1 (채널 16..31)
//!  패킷 = 블록 12개 = 발사 24회 = 24 × 55.296 = 1327.104 µs
//! ```

use lidar_core::SensorTime;

pub const LASERS: usize = 16;
pub const FIRINGS_PER_BLOCK: usize = 2;
pub const FIRINGS_PER_PACKET: usize = 24;

/// 같은 발사 안에서 레이저 사이 간격, µs
pub const LASER_INTERVAL_US: f64 = 2.304;
/// 발사 주기, µs
pub const FIRING_PERIOD_US: f64 = 55.296;
/// 단일 반환 모드에서 패킷 간격, µs. 이중 반환이면 같은 발사를 블록 두 개가 나눠 담으므로 절반.
pub const PACKET_INTERVAL_US: f64 = FIRING_PERIOD_US * FIRINGS_PER_PACKET as f64;

/// 패킷 타임스탬프(발사 0의 레이저 0)로부터, 발사 `firing`의 레이저 `laser`가 쏜 순간까지 µs.
pub fn firing_offset_us(firing: usize, laser: usize) -> f64 {
    firing as f64 * FIRING_PERIOD_US + laser as f64 * LASER_INTERVAL_US
}

/// 센서의 "정시 이후 µs"는 1시간마다 0으로 돌아간다.
const HOUR_US: i64 = 3_600_000_000;

/// "정시 이후 µs"를 계속 증가하는 [`SensorTime`]으로 펼친다.
///
/// ```text
///  원시값:  … 3_599_998_000, 3_599_999_327, │ 654, 1_981, …
///  펼친 값: … 3_599_998_000, 3_599_999_327, │ 3_600_000_654, 3_600_001_981, …
///                                       정시 넘김 → 1시간을 더해 감
/// ```
///
/// 30분 넘게 뒤로 간 값만 정시 넘김으로 본다. 그보다 작게 뒤로 간 값(재생 반복, 순서 바뀜)은
/// 그대로 돌려주고, 판단은 호출한 쪽(프레임 조립)이 한다.
///
/// 정시 직후에 정시 직전 패킷이 늦게 도착하면(순서 바뀜) 30분 넘게 **앞으로** 간 것처럼 보인다.
/// 이것은 지난 시간의 패킷이므로 1시간을 빼서 돌려주고, 상태는 바꾸지 않는다 → 호출한 쪽은 조금 뒤로 간
/// 패킷(순서 바뀜)으로 본다.
#[derive(Clone, Debug, Default)]
pub struct SensorClock {
    last_raw: Option<u32>,
    hours: i64,
}

impl SensorClock {
    pub fn unwrap(&mut self, raw: u32) -> SensorTime {
        if let Some(last) = self.last_raw {
            let step = raw as i64 - last as i64;
            if step < -HOUR_US / 2 {
                self.hours += 1;
            } else if step > HOUR_US / 2 && self.hours > 0 {
                return SensorTime((self.hours - 1) * HOUR_US + raw as i64);
            }
        }
        self.last_raw = Some(raw);
        SensorTime(self.hours * HOUR_US + raw as i64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn offsets_follow_the_spec_table() {
        assert_eq!(firing_offset_us(0, 0), 0.0);
        assert!((firing_offset_us(0, 15) - 34.56).abs() < 1e-9);
        assert!((firing_offset_us(1, 0) - 55.296).abs() < 1e-9);
        assert!((firing_offset_us(23, 15) - (23.0 * 55.296 + 34.56)).abs() < 1e-9);
        assert!((PACKET_INTERVAL_US - 1327.104).abs() < 1e-9);
    }

    #[test]
    fn clock_unwraps_top_of_hour() {
        let mut c = SensorClock::default();
        assert_eq!(c.unwrap(3_599_999_327), SensorTime(3_599_999_327));
        assert_eq!(c.unwrap(654), SensorTime(3_600_000_654));
        assert_eq!(c.unwrap(1_981), SensorTime(3_600_001_981));
    }

    #[test]
    fn a_late_packet_from_before_the_hour_is_not_a_jump_forward() {
        let mut c = SensorClock::default();
        c.unwrap(3_599_998_000);
        c.unwrap(654); // 정시 넘김
                       // 정시 직전 패킷이 늦게 옴 → 지난 시간의 값 (조금 뒤로 감)
        assert_eq!(c.unwrap(3_599_999_327), SensorTime(3_599_999_327));
        // 상태는 그대로라 다음 패킷도 맞게 펼쳐짐
        assert_eq!(c.unwrap(1_981), SensorTime(3_600_001_981));
    }

    #[test]
    fn clock_leaves_small_backward_steps_alone() {
        let mut c = SensorClock::default();
        c.unwrap(10_000_000);
        // 9.4 s짜리 파일을 반복 재생하면 시각이 9.4 s 뒤로 간다 → 정시 넘김이 아님
        assert_eq!(c.unwrap(600_000), SensorTime(600_000));
    }
}
