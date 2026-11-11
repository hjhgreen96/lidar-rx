//! 참조 구현(velodyne_decoder 3.1.0)의 출력과 포인트 단위로 대조한다.
//!
//! 골든 파일: `testdata/golden_vlp16.csv` (`tools/make_golden.py`로 생성, 패킷 0..152)
//! 입력: `data/vlp16.pcap` (원본, `tools/fetch_data.sh`로 받음)
//!
//! 프레임 자르는 방식이 다르므로(참조는 패킷 단위, 여기는 발사 단위) 프레임이 아니라
//! (패킷 번호, 발사 번호, ring)으로 짝을 지어 비교한다. 허용 오차의 근거:
//!
//! | 값     | 허용 오차  | 근거                                                                 |
//! |--------|-----------|----------------------------------------------------------------------|
//! | 수평 거리, z | 0.1 mm | 같은 식·같은 f32 계산. 남는 차이는 sin/cos 구현의 마지막 자리           |
//! | 방위각 | 0.006°    | 참조는 방위각을 0.01° 정수로 반올림해 cos·sin 표를 찾음 → 최대 0.005°     |
//! | 시각   | 0.25 µs   | 참조는 절대 시각(약 14억 초)을 double로 다뤄 해상도가 약 0.24 µs        |

use lidar_core::{PacketSource, Received};
use lidar_io::PcapSource;
use std::collections::HashMap;
use std::path::PathBuf;
use vlp16::calibration::Calibration;
use vlp16::decode::{decode_packet, RangeFilter, RawPoint};
use vlp16::packet::PacketView;

const RANGE: RangeFilter = RangeFilter {
    min_m: 0.1,
    max_m: 200.0,
};

struct Golden {
    xyz: [f64; 3],
    intensity: u8,
    offset_us: f64,
}

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

/// (packet, firing, ring) → 참조 포인트
fn load_golden() -> HashMap<(usize, u8, u16), Golden> {
    let text = std::fs::read_to_string(repo_root().join("testdata/golden_vlp16.csv"))
        .expect("testdata/golden_vlp16.csv (generate with tools/make_golden.py)");
    text.lines()
        .filter(|l| !l.starts_with('#'))
        .map(|line| {
            let v: Vec<f64> = line.split(',').map(|s| s.parse().unwrap()).collect();
            let key = (v[0] as usize, v[1] as u8, v[2] as u16);
            (
                key,
                Golden {
                    xyz: [v[3], v[4], v[5]],
                    intensity: v[6] as u8,
                    offset_us: v[8],
                },
            )
        })
        .collect()
}

fn azimuth_deg(x: f64, y: f64) -> f64 {
    // ROS 좌표에서 센서 방위각(정면에서 시계방향) = atan2(−y, x)
    (-y).atan2(x).to_degrees().rem_euclid(360.0)
}

fn angle_diff(a: f64, b: f64) -> f64 {
    ((a - b + 180.0).rem_euclid(360.0) - 180.0).abs()
}

#[derive(Default, Debug)]
struct MaxDiff {
    horizontal_m: f64,
    z_m: f64,
    azimuth_deg: f64,
    time_us: f64,
}

#[test]
fn matches_velodyne_decoder_point_by_point() {
    let golden = load_golden();
    let last_packet = golden.keys().map(|k| k.0).max().unwrap();

    let pcap = repo_root().join("data/vlp16.pcap");
    let mut source = PcapSource::open(&pcap, 2368).unwrap_or_else(|e| {
        panic!(
            "{}: {e} (download with tools/fetch_data.sh)",
            pcap.display()
        )
    });
    let calibration = Calibration::vlp16();

    let mut mine: HashMap<(usize, u8, u16), RawPoint> = HashMap::new();
    for index in 0..=last_packet {
        let Received::Packet(packet) = source.recv().unwrap() else {
            panic!("pcap ended early")
        };
        let view = PacketView::parse(&packet.payload).unwrap();
        for p in decode_packet(view, &calibration, RANGE).unwrap().points {
            mine.insert((index, p.firing, p.ring), p);
        }
    }

    // 같은 포인트들이 남았는가 (거리 필터와 '반사 없음' 처리가 같은가)
    assert_eq!(mine.len(), golden.len(), "point count");
    let mut max = MaxDiff::default();
    for (key, g) in &golden {
        let p = mine
            .get(key)
            .unwrap_or_else(|| panic!("missing point {key:?}"));
        let [x, y, z] = p.xyz.map(f64::from);

        max.horizontal_m = max
            .horizontal_m
            .max((x.hypot(y) - g.xyz[0].hypot(g.xyz[1])).abs());
        max.z_m = max.z_m.max((z - g.xyz[2]).abs());
        max.azimuth_deg = max
            .azimuth_deg
            .max(angle_diff(p.azimuth_deg, azimuth_deg(g.xyz[0], g.xyz[1])));
        max.time_us = max.time_us.max((p.offset_us - g.offset_us).abs());
        assert_eq!(p.reflectivity, g.intensity, "intensity {key:?}");
    }

    println!("{} points, max diff {max:?}", golden.len());
    assert!(max.horizontal_m < 1e-4, "{max:?}");
    assert!(max.z_m < 1e-4, "{max:?}");
    assert!(max.azimuth_deg < 0.006, "{max:?}");
    assert!(max.time_us < 0.25, "{max:?}");
}
