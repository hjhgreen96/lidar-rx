//! 레이저별 보정값: 수직각, 수직 오프셋, ring 번호.
//!
//! ```text
//!  레이저(발사 순서)  0    1   2    3   4    5   6   7   8   9   10  11  12  13  14  15
//!  수직각 (°)        -15   1  -13   3  -11   5  -9   7  -7   9  -5   11  -3  13  -1  15
//!  ring (각도 순)     0    8   1    9   2   10   3  11   4  12   5   13   6  14   7  15
//! ```
//!
//! 레이저를 위아래로 번갈아 쏘는 이유: 이웃한 각도의 레이저를 연달아 쏘면 서로의 반사를 받을 수 있어서.
//! 소비자는 각도 순서가 편하므로 수직각 오름차순 번호(ring)를 따로 붙인다.
//!
//! ## 수직 오프셋
//!
//! 레이저들은 센서 앞쪽의 한 점(초점)을 지나도록 배치돼 있어, 빔이 나오는 위치가 레이저마다
//! 위아래로 조금씩 다르다 (옆에서 본 그림):
//!
//! ```text
//!        ↗ +15° 빔                  오프셋 = 초점 거리 × tan(−수직각)
//!   ●───────────● 초점 (41.91 mm 앞)    +15° 레이저: −11.23 mm (아래에서 나와 위로)
//!        ↘ −15° 빔                      −15° 레이저: +11.23 mm (위에서 나와 아래로)
//! ```
//!
//! 이 값(41.91 mm, tan 식)과 "z에 더하는" 적용 방식은 참조 구현 `velodyne_decoder` 3.1.0과 같다.
//! 공식 ROS 드라이버는 사양서 표 값(±11.2 mm 등)을 빔 각도만큼 돌려 적용해 xy가 최대 약 2.9 mm 다르다.
//! (`docs/modules/vlp16.md`의 "참조 구현과의 차이")

use crate::timing::LASERS;

/// 레이저 번호 순서의 수직각 (사양서)
pub const VERTICAL_ANGLES_DEG: [f32; LASERS] = [
    -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0,
];

/// 센서 중심에서 빔들이 모이는 초점까지의 거리, m
pub const FOCAL_DISTANCE_M: f32 = 0.04191;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Laser {
    pub sin_vertical: f32,
    pub cos_vertical: f32,
    /// 빔이 나오는 위치의 높이 차이, m
    pub vertical_offset_m: f32,
    pub ring: u16,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Calibration {
    pub lasers: [Laser; LASERS],
}

impl Calibration {
    /// VLP-16 기본 보정값.
    pub fn vlp16() -> Self {
        let rings = rings_by_angle(&VERTICAL_ANGLES_DEG);
        let lasers = std::array::from_fn(|i| {
            let angle = (VERTICAL_ANGLES_DEG[i] as f64).to_radians() as f32;
            Laser {
                sin_vertical: angle.sin(),
                cos_vertical: angle.cos(),
                vertical_offset_m: FOCAL_DISTANCE_M * (-angle).tan(),
                ring: rings[i],
            }
        });
        Calibration { lasers }
    }
}

/// 수직각 오름차순으로 0, 1, 2, …를 매긴다.
fn rings_by_angle(angles: &[f32; LASERS]) -> [u16; LASERS] {
    let mut order: [usize; LASERS] = std::array::from_fn(|i| i);
    order.sort_by(|&a, &b| angles[a].total_cmp(&angles[b]));
    let mut rings = [0u16; LASERS];
    for (ring, &laser) in order.iter().enumerate() {
        rings[laser] = ring as u16;
    }
    rings
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rings_follow_vertical_angle() {
        let c = Calibration::vlp16();
        let rings: Vec<u16> = c.lasers.iter().map(|l| l.ring).collect();
        assert_eq!(
            rings,
            [0, 8, 1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15]
        );
    }

    #[test]
    fn offsets_match_spec_table_within_0_05_mm() {
        // 사양서 표의 수직 오프셋 (mm)
        let spec = [
            11.2, -0.7, 9.7, -2.2, 8.1, -3.7, 6.6, -5.1, 5.1, -6.6, 3.7, -8.1, 2.2, -9.7, 0.7,
            -11.2,
        ];
        for (laser, mm) in Calibration::vlp16().lasers.iter().zip(spec) {
            assert!(
                (laser.vertical_offset_m * 1000.0 - mm).abs() < 0.05,
                "{laser:?} vs {mm}"
            );
        }
    }
}
