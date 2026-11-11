//! 방위각 보간과 좌표 변환.
//!
//! ## 레이저마다 방위각이 다르다
//!
//! 블록에는 방위각이 하나만 있지만(블록 첫 발사 시작 순간), 센서는 계속 돌고 있으므로 같은 블록 안에서도
//! 나중에 쏜 레이저일수록 조금 더 돌아간 방향을 본다. 그래서 쏜 시각에 비례해 방위각을 더해 준다.
//!
//! ```text
//!  방위각 = 블록 방위각 + 회전 속도 × (블록 첫 발사로부터 경과 시간)
//!
//!  회전 속도는 패킷의 첫 블록과 마지막 블록 방위각 차이 ÷ 그 사이 시간(발사 22회)으로 구한다.
//!    블록 방위각은 0.01° 단위로 잘려 있어, 블록 하나 간격(약 0.4°)으로 구하면 ±2.5% 흔들리지만
//!    패킷 전체(약 4.4°)로 구하면 ±0.23%로 줄어든다.
//! ```
//!
//! ## 좌표계
//!
//! ```text
//!  센서 좌표 (사양서)                    ROS 좌표 (REP-103)
//!        +y (방위각 0°, 정면)                 +x (앞)
//!         ↑                                    ↑
//!         │  방위각은 위에서 보아 시계방향       │
//!   ──────┼──────► +x                  +y ◄───┼──────
//!         │                                    │
//!  x_s = d·cosω·sinα                   x = y_s =  d·cosω·cosα
//!  y_s = d·cosω·cosα                   y = −x_s = −d·cosω·sinα
//!  z_s = d·sinω (+ 수직 오프셋)         z = z_s
//!  (d: 거리, ω: 수직각, α: 방위각)
//! ```

use crate::calibration::Laser;
use crate::timing::FIRING_PERIOD_US;
use crate::units::Azimuth;

/// 패킷 안 첫 블록 → 마지막 블록 사이 시간 = 발사 22회
const FIRST_TO_LAST_BLOCK_US: f64 = 22.0 * FIRING_PERIOD_US;

/// 회전 속도, 0.01° / µs (단일 반환 패킷 기준).
pub fn rotation_rate(first_block: Azimuth, last_block: Azimuth) -> f64 {
    first_block.forward_to(last_block) as f64 / FIRST_TO_LAST_BLOCK_US
}

/// 블록 방위각에서 `dt_us`만큼 돈 뒤의 방위각, 도 (0 ≤ 값 < 360).
pub fn azimuth_after(block: Azimuth, rate: f64, dt_us: f64) -> f64 {
    ((block.0 as f64 + rate * dt_us) / 100.0).rem_euclid(360.0)
}

/// 거리·방위각·레이저 보정값 → ROS 좌표 (x 앞, y 왼쪽, z 위), m.
pub fn to_ros_xyz(distance_m: f32, azimuth_deg: f64, laser: &Laser) -> [f32; 3] {
    let (sin_a, cos_a) = (azimuth_deg.to_radians() as f32).sin_cos();
    to_ros_xyz_sincos(distance_m, sin_a, cos_a, laser)
}

/// [`to_ros_xyz`]와 같은 계산에 방위각의 sin·cos를 받는 판. 해석기는 포인트마다 삼각함수를 부르지 않고
/// 덧셈 공식으로 sin·cos를 만들어 이것을 부른다 (`decode.rs`).
pub fn to_ros_xyz_sincos(distance_m: f32, sin_a: f32, cos_a: f32, laser: &Laser) -> [f32; 3] {
    let horizontal = distance_m * laser.cos_vertical;
    [
        horizontal * cos_a,
        -horizontal * sin_a,
        distance_m * laser.sin_vertical + laser.vertical_offset_m,
    ]
}

/// `last`에서 `current`로 정방향으로 돌면서 `cut`을 지났는가 (모두 도).
///
/// `current`에 정확히 도착한 경우도 지난 것으로 본다. 반대로 돌았거나 반 바퀴 넘게 건너뛴 경우는
/// 회전이 아니라 이상한 값이므로 지나지 않은 것으로 본다.
pub fn crossed(last: f64, current: f64, cut: f64) -> bool {
    let step = (current - last).rem_euclid(360.0);
    let to_cut = (cut - last).rem_euclid(360.0);
    step < 180.0 && to_cut > 0.0 && to_cut <= step
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calibration::Calibration;

    #[test]
    fn rate_and_interpolation_across_zero() {
        // 블록 간격 0.40°(40) → 첫 블록 359.90°, 마지막 블록 4.30°
        let rate = rotation_rate(Azimuth(35_990), Azimuth(430));
        assert!((rate - 440.0 / (22.0 * 55.296)).abs() < 1e-12);
        // 블록의 두 번째 발사(55.296 µs 뒤)는 블록 간격의 절반만큼 더 돈 곳
        let az = azimuth_after(Azimuth(35_990), rate, 55.296);
        assert!((az - 0.1).abs() < 1e-9, "{az}");
    }

    #[test]
    fn axes() {
        let laser = Calibration::vlp16().lasers[14]; // −1°
        let [x, y, _] = to_ros_xyz(10.0, 0.0, &laser); // 정면
        assert!(x > 9.9 && y.abs() < 1e-6);
        let [x, y, _] = to_ros_xyz(10.0, 90.0, &laser); // 위에서 보아 시계방향 90° = 오른쪽 = −y
        assert!(x.abs() < 1e-5 && y < -9.9);
    }

    #[test]
    fn crossing() {
        assert!(crossed(359.8, 0.2, 0.0));
        assert!(crossed(359.8, 0.0, 0.0)); // 정확히 도착
        assert!(!crossed(0.0, 0.2, 0.0)); // 이미 지난 뒤
        assert!(crossed(89.9, 90.1, 90.0));
        assert!(!crossed(10.0, 5.0, 7.0)); // 뒤로 감
    }
}
