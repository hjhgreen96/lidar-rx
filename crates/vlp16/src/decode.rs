//! 패킷 하나 → 포인트들. 프레임(한 바퀴)은 모른다. 그것은 `assemble`의 일.
//!
//! ```text
//!  PacketView ──┬─ 블록 12개 × 채널 32개
//!               │    채널 c → 발사 = 2·블록 + c/16, 레이저 = c%16
//!               │    거리 0(반사 없음) 또는 범위 밖 → 버림
//!               │    방위각 = 블록 방위각 + 회전 속도 × 경과 시간      (geometry)
//!               │    xyz    = 거리·수직각·방위각·수직 오프셋 → ROS 좌표 (geometry)
//!               │    시각   = 패킷 타임스탬프 + 발사 오프셋            (timing)
//!               └─► DecodedPacket { 발사 24회의 방위각, 포인트들(발사 순) }
//! ```

use crate::calibration::Calibration;
use crate::geometry::{azimuth_after, rotation_rate, to_ros_xyz_sincos};
use crate::packet::{PacketView, ReturnMode, BLOCKS, CHANNELS_PER_BLOCK};
use crate::timing::{
    firing_offset_us, FIRINGS_PER_BLOCK, FIRINGS_PER_PACKET, LASERS, LASER_INTERVAL_US,
};
use lidar_core::DecodeError;

/// 이 범위 밖의 거리는 버린다 (양 끝 포함). 거리 0(반사 없음)은 범위와 상관없이 버린다.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RangeFilter {
    pub min_m: f32,
    pub max_m: f32,
}

impl RangeFilter {
    fn keeps(&self, distance_m: f32) -> bool {
        distance_m >= self.min_m && distance_m <= self.max_m
    }
}

/// 해석된 포인트 하나. 프레임에 들어가기 전이라 패킷 안의 위치(발사·레이저)를 그대로 갖고 있다.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RawPoint {
    /// 패킷 안 발사 번호 0..24
    pub firing: u8,
    /// 레이저 번호 0..16 (발사 순서)
    pub laser: u8,
    pub ring: u16,
    pub azimuth_deg: f64,
    pub distance_m: f32,
    pub reflectivity: u8,
    /// ROS 좌표, m
    pub xyz: [f32; 3],
    /// 패킷 타임스탬프로부터의 측정 시각, µs
    pub offset_us: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DecodedPacket {
    /// 센서 시계, 정시 이후 µs
    pub timestamp_us: u32,
    pub product_id: u8,
    /// 발사마다 레이저 0이 쏜 순간의 방위각 (도). 포인트가 하나도 안 남은 발사도 있으므로 따로 둔다
    /// → 프레임 경계는 이 값으로 정한다.
    pub firing_azimuths: [f64; FIRINGS_PER_PACKET],
    /// 이 패킷에서 잰 회전 속도, 도/µs. 손실 구간 동안 얼마나 돌았는지 어림하는 데 쓴다.
    pub rotation_deg_per_us: f64,
    /// 발사 순서대로
    pub points: Vec<RawPoint>,
}

pub fn decode_packet(
    packet: PacketView<'_>,
    calibration: &Calibration,
    range: RangeFilter,
) -> Result<DecodedPacket, DecodeError> {
    decode_packet_reusing(packet, calibration, range, Vec::new())
}

/// `decode_packet`과 같되 포인트를 `points`(비워서 씀)에 담는다. 수신 스레드가 패킷마다(초당 754번) 배열을
/// 새로 할당하지 않게, 해석기는 지난 패킷의 배열을 넘긴다.
pub fn decode_packet_reusing(
    packet: PacketView<'_>,
    calibration: &Calibration,
    range: RangeFilter,
    mut points: Vec<RawPoint>,
) -> Result<DecodedPacket, DecodeError> {
    if packet.return_mode() == ReturnMode::Dual {
        return Err(DecodeError::Unsupported(
            "dual return mode is not implemented".into(),
        ));
    }

    let rate = rotation_rate(
        packet.block(0).azimuth(),
        packet.block(BLOCKS - 1).azimuth(),
    );
    let mut firing_azimuths = [0.0; FIRINGS_PER_PACKET];
    points.clear();
    points.reserve(BLOCKS * CHANNELS_PER_BLOCK);

    // 한 발사 안에서 레이저 l은 레이저 0보다 l × 2.304 µs 뒤에 쏘므로 방위각이 δ_l = 회전 속도 × 그 시간만큼 크다.
    // δ_l은 이 패킷 안에서 발사와 상관없이 같다 → sin·cos를 레이저마다 한 번(16번), 발사마다 한 번(24번)만 구하고
    // 포인트마다는 덧셈 공식으로 합친다: sin(a+δ) = sin a·cos δ + cos a·sin δ, cos(a+δ) = cos a·cos δ − sin a·sin δ.
    // 포인트마다 삼각함수를 부르면 삼각함수 하드웨어가 없는 보드(Cortex-A9)에서 패킷 하나에 약 0.2 ms가 들었다.
    let mut laser_turn = [(0.0, 0.0, 1.0); LASERS]; // (δ 도, sin δ, cos δ)
    for (laser, turn) in laser_turn.iter_mut().enumerate() {
        let delta_deg = rate * laser as f64 * LASER_INTERVAL_US / 100.0;
        let (sin_d, cos_d) = delta_deg.to_radians().sin_cos();
        *turn = (delta_deg, sin_d, cos_d);
    }

    for b in 0..BLOCKS {
        let block = packet.block(b);
        let first_firing = b * FIRINGS_PER_BLOCK;
        // 블록 방위각은 블록 첫 발사(레이저 0)의 시각 기준 → 경과 시간은 그 시각부터 잼
        let block_start_us = firing_offset_us(first_firing, 0);

        for f in 0..FIRINGS_PER_BLOCK {
            let firing = first_firing + f;
            let firing_deg = azimuth_after(
                block.azimuth(),
                rate,
                firing_offset_us(firing, 0) - block_start_us,
            );
            firing_azimuths[firing] = firing_deg;
            let (sin_f, cos_f) = firing_deg.to_radians().sin_cos();

            for (laser, &(delta_deg, sin_d, cos_d)) in laser_turn.iter().enumerate() {
                let channel = block.channel(f * LASERS + laser);
                if channel.distance.is_no_return() {
                    continue;
                }
                let distance_m = channel.distance.meters();
                if !range.keeps(distance_m) {
                    continue;
                }
                let mut azimuth_deg = firing_deg + delta_deg; // δ는 0.1° 미만이라 넘어도 한 번만 뺌
                if azimuth_deg >= 360.0 {
                    azimuth_deg -= 360.0;
                }
                let sin_a = sin_f * cos_d + cos_f * sin_d;
                let cos_a = cos_f * cos_d - sin_f * sin_d;
                let cal = &calibration.lasers[laser];
                points.push(RawPoint {
                    firing: firing as u8,
                    laser: laser as u8,
                    ring: cal.ring,
                    azimuth_deg,
                    distance_m,
                    reflectivity: channel.reflectivity,
                    xyz: to_ros_xyz_sincos(distance_m, sin_a as f32, cos_a as f32, cal),
                    offset_us: firing_offset_us(firing, laser),
                });
            }
        }
    }

    Ok(DecodedPacket {
        timestamp_us: packet.timestamp_us(),
        product_id: packet.product_id(),
        firing_azimuths,
        rotation_deg_per_us: rate / 100.0,
        points,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::packet::test_packet;

    const ALL: RangeFilter = RangeFilter {
        min_m: 0.1,
        max_m: 200.0,
    };

    #[test]
    fn every_channel_becomes_a_point_in_firing_order() {
        let bytes = test_packet::build(1_000, 0, 40, 5_000); // 10 m
        let d = decode_packet(
            PacketView::parse(&bytes).unwrap(),
            &Calibration::vlp16(),
            ALL,
        )
        .unwrap();
        assert_eq!(d.points.len(), 12 * 32);
        assert!(d.points.windows(2).all(|w| w[0].firing <= w[1].firing));
        let p = d.points[16 + 3]; // 블록 0, 두 번째 발사, 레이저 3
        assert_eq!((p.firing, p.laser, p.ring), (1, 3, 9));
        assert!((p.offset_us - (55.296 + 3.0 * 2.304)).abs() < 1e-9);
        assert_eq!(p.reflectivity, 19); // 합성 패킷의 반사율 = 채널 번호
    }

    #[test]
    fn decoding_into_a_used_array_gives_the_same_points_in_the_same_array() {
        let decode = |start, points| {
            let bytes = test_packet::build(1_000, start, 40, 5_000);
            decode_packet_reusing(
                PacketView::parse(&bytes).unwrap(),
                &Calibration::vlp16(),
                ALL,
                points,
            )
            .unwrap()
        };
        let used = decode(9_000, Vec::new()).points; // 다른 패킷의 포인트가 든 배열
        let at = used.as_ptr();
        let again = decode(0, used);
        assert_eq!(again.points.as_ptr(), at);
        assert_eq!(again.points, decode(0, Vec::new()).points);
    }

    #[test]
    fn range_filter_and_no_return() {
        let bytes = test_packet::build(1_000, 0, 40, 0);
        let d = decode_packet(
            PacketView::parse(&bytes).unwrap(),
            &Calibration::vlp16(),
            ALL,
        )
        .unwrap();
        assert!(d.points.is_empty());
        assert_eq!(d.firing_azimuths[2], 0.4); // 포인트가 없어도 발사 방위각은 있음

        let bytes = test_packet::build(1_000, 0, 40, 40); // 0.08 m
        let d = decode_packet(
            PacketView::parse(&bytes).unwrap(),
            &Calibration::vlp16(),
            ALL,
        )
        .unwrap();
        assert!(d.points.is_empty());
    }

    #[test]
    fn matches_trigonometry_per_point_including_the_wrap_at_360() {
        // 359.9°에서 시작해 패킷 안에서 0°를 넘음
        let bytes = test_packet::build(1_000, 35_990, 40, 5_000);
        let view = PacketView::parse(&bytes).unwrap();
        let cal = Calibration::vlp16();
        let d = decode_packet(view, &cal, ALL).unwrap();
        let rate = rotation_rate(view.block(0).azimuth(), view.block(BLOCKS - 1).azimuth());
        assert_eq!(d.points.len(), 12 * 32);
        for p in &d.points {
            // 바꾸기 전 방식: 포인트마다 방위각을 보간하고 삼각함수를 부름
            let b = p.firing as usize / FIRINGS_PER_BLOCK;
            let dt = p.offset_us - firing_offset_us(b * FIRINGS_PER_BLOCK, 0);
            let az = azimuth_after(view.block(b).azimuth(), rate, dt);
            let xyz = crate::geometry::to_ros_xyz(p.distance_m, az, &cal.lasers[p.laser as usize]);
            assert!(
                (p.azimuth_deg - az).abs() < 1e-9,
                "{} vs {az}",
                p.azimuth_deg
            );
            for k in 0..3 {
                assert!((p.xyz[k] - xyz[k]).abs() < 1e-5, "{:?} vs {xyz:?}", p.xyz);
            }
        }
    }
}
