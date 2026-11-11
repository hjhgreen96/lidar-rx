//! Velodyne VLP-16 해석기: UDP 페이로드 → 포인트 → 한 바퀴 프레임.
//!
//! 소켓도 파일도 모른다. 바이트를 받아 계산만 하므로 바이트 배열만으로 테스트할 수 있다.
//!
//! ```text
//!   Packet.payload (1206 B)
//!        │  packet::PacketView::parse      길이·플래그·방위각 범위 검증, 복사 없이 읽는 창
//!        ▼
//!   PacketView
//!        │  decode::decode_packet          채널 → 포인트 (거리 필터, 방위각 보간, 좌표, 시각)
//!        │     ├ calibration               레이저별 수직각·수직 오프셋·ring
//!        │     ├ geometry                  회전 속도, 방위각 보간, ROS 좌표
//!        │     └ timing                    발사 오프셋
//!        ▼
//!   DecodedPacket (발사 24회의 방위각 + 포인트들)
//!        │  assemble::FrameAssembler       센서 시각 펼치기·손실 판정, 발사 단위로 한 바퀴씩 자름
//!        ▼
//!   lidar_core::Frame
//! ```
//!
//! [`Vlp16Decoder`]가 이 전체를 묶어 `lidar_core::SensorDecoder`를 구현한다 (Strategy의 구체 전략).
//! 자세한 설명: `docs/modules/vlp16.md`

pub mod assemble;
pub mod calibration;
pub mod decode;
pub mod geometry;
pub mod packet;
pub mod timing;
pub mod units;

use assemble::{AssemblerCounters, FrameAssembler, StampPolicy};
use calibration::Calibration;
use decode::{decode_packet_reusing, RangeFilter, RawPoint};
use lidar_core::{DecodeError, DecoderCounters, Frame, Packet, SensorDecoder};
use packet::{PacketView, PRODUCT_ID_VLP16};

#[derive(Clone, Debug, PartialEq)]
pub struct Config {
    /// 거리 필터, m (양 끝 포함). 기본 0.1 ~ 200 m는 `velodyne_decoder`와 같다. 공식 드라이버는 0.9 ~ 130 m.
    pub min_range_m: f32,
    pub max_range_m: f32,
    /// 프레임을 자르는 방위각, 도
    pub cut_azimuth_deg: f64,
    pub stamp: StampPolicy,
    pub frame_id: String,
    /// 패킷의 제품 ID가 이 값과 다르면 세기만 한다 (해석은 계속). 샘플 pcap은 0x21로 찍혀 있다.
    pub expected_product_id: u8,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            min_range_m: 0.1,
            max_range_m: 200.0,
            cut_azimuth_deg: 0.0,
            stamp: StampPolicy::LastPacket,
            frame_id: "velodyne".into(),
            expected_product_id: PRODUCT_ID_VLP16,
        }
    }
}

/// 해석기가 지금까지 센 것들. 라이브러리는 로그를 찍지 않고 숫자로 알리며, 찍는 것은 쓰는 쪽(rx)이 한다.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DecoderStats {
    pub packets: u64,
    pub malformed_packets: u64,
    pub product_id_mismatches: u64,
    pub assembler: AssemblerCounters,
}

pub struct Vlp16Decoder {
    calibration: Calibration,
    range: RangeFilter,
    expected_product_id: u8,
    assembler: FrameAssembler,
    /// 지난 패킷의 포인트 배열. 다음 패킷을 여기에 해석한다 (패킷마다 새로 할당하지 않게)
    packet_points: Vec<RawPoint>,
    stats: DecoderStats,
}

impl Vlp16Decoder {
    pub fn new(config: Config) -> Self {
        Vlp16Decoder {
            calibration: Calibration::vlp16(),
            range: RangeFilter {
                min_m: config.min_range_m,
                max_m: config.max_range_m,
            },
            expected_product_id: config.expected_product_id,
            assembler: FrameAssembler::new(config.cut_azimuth_deg, config.stamp, config.frame_id),
            packet_points: Vec::new(),
            stats: DecoderStats::default(),
        }
    }

    pub fn stats(&self) -> DecoderStats {
        DecoderStats {
            assembler: self.assembler.counters(),
            ..self.stats
        }
    }
}

impl SensorDecoder for Vlp16Decoder {
    fn push(&mut self, packet: &Packet) -> Result<Option<Frame>, DecodeError> {
        self.stats.packets += 1;
        let view = PacketView::parse(&packet.payload).map_err(|e| {
            self.stats.malformed_packets += 1;
            DecodeError::Malformed(e.to_string())
        })?;
        if view.product_id() != self.expected_product_id {
            self.stats.product_id_mismatches += 1;
        }
        let points = std::mem::take(&mut self.packet_points);
        let decoded = decode_packet_reusing(view, &self.calibration, self.range, points)?;
        let frame = self.assembler.push(&decoded, packet.received_at);
        self.packet_points = decoded.points;
        Ok(frame)
    }

    fn flush(&mut self) -> Option<Frame> {
        self.assembler.flush()
    }

    fn recycle(&mut self, frame: Frame) {
        self.assembler.give_back(frame);
    }

    fn counters(&self) -> DecoderCounters {
        let a = self.assembler.counters();
        DecoderCounters {
            packets: self.stats.packets,
            malformed_packets: self.stats.malformed_packets,
            missing_packets: a.missing_packets,
            reordered_packets: a.reordered_packets,
            restarts: a.restarts,
            frames: a.frames,
        }
    }
}
