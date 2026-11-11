//! 해석된 패킷들 → 한 바퀴씩 묶은 프레임.
//!
//! ## 어디서 자르나: 발사 단위
//!
//! 방위각이 자르는 각도(기본 0°)를 지나는 **발사**에서 자른다. 한 패킷은 약 4.8°라 자르는 각도는
//! 대개 패킷 중간에 오고, 그 패킷의 앞쪽 발사는 이전 프레임, 뒤쪽 발사는 새 프레임에 들어간다.
//!
//! ```text
//!   패킷 n-1          패킷 n                     패킷 n+1
//!  ┌──────────┐┌────────────────────────────┐┌──────────┐
//!  │ … 359.5° ││ 359.7° 359.9° │ 0.1° 0.3° …││ …        │
//!  └──────────┘└───────────────┼────────────┘└──────────┘
//!       ──── 프레임 k ────────►│◄──────── 프레임 k+1 ────
//!                         0°를 지남
//! ```
//!
//! 참조 구현 두 개(공식 드라이버, velodyne_decoder)는 패킷 단위로 잘라 한 프레임에 최대 약 4.8°가
//! 더 들어간다. 여기서는 발사 단위(약 0.2°)라 한 바퀴에 더 가깝다.
//!
//! ## 패킷 사이를 센서 시각으로 확인한다
//!
//! ```text
//!  간격 Δ = 이번 패킷 시각 − 지난 패킷 시각
//!   Δ ≈ 1327 µs              정상
//!   Δ ≈ k × 1327 µs          패킷 k−1개 손실 → 프레임의 missing_packets에 더함
//!                            빠진 구간에 자르는 각도가 있으면 앞뒤 프레임에 각도로 나눔 (`split_missing`)
//!                            그동안 반 바퀴 넘게 돌았으면(회전 속도 × Δ ≥ 180°) 그 자리에서 끊음:
//!                            방위각은 한 바퀴마다 같은 값으로 돌아오므로, 그만큼 건너뛰면 자르는 각도를
//!                            지났는지 방위각만으로는 알 수 없다 → 두 바퀴가 한 프레임에 섞이는 것을 막음
//!   −10 ms < Δ ≤ 0           순서 바뀜·중복 → 이미 지난 자리라 버리고 셈
//!   Δ ≤ −10 ms 또는 Δ > 1 s   흐름 재시작 (재생 반복, 센서 재시작) → 모으던 프레임을 버리고 새로 시작
//! ```
//!
//! ## 대표 시각
//!
//! 기본은 프레임의 **마지막 패킷** 시각(공식 드라이버·velodyne_decoder와 같음). 포인트 시각은 이 값
//! 기준 상대값이라 대부분 음수다 (샘플에서 약 −0.1 ~ +0.0013 s).
//!
//! ## 배열은 돌려받아 다시 쓴다
//!
//! ```text
//!  수신 스레드: 조립 ─ 프레임(점 배열 A) ─► stage·소비자 스레드 ─ 다 쓰면 ─┐
//!       ▲  다음 프레임은 돌려받은 A에 모음                                 │ give_back
//!       └────────────────────────────────────────────────────────────────┘
//! ```
//!
//! 점 배열(약 0.7 MB)을 프레임마다 새로 할당하면, 큰 할당이라 C 라이브러리가 mmap으로 새 페이지를 받아 오고
//! 수신 스레드가 그 페이지를 처음 쓸 때마다 page fault가 난다 (보드에서 초당 약 1,700번, 잠깐씩 커널에 들어감).
//! 돌려받은 배열은 이미 쓴 페이지라 그렇지 않다. 새로 할당하는 것은 돌려받은 것이 없을 때뿐이라, 돌고 있는
//! 배열의 수는 동시에 처리 중인 프레임 수 + 1을 넘지 않는다.

use crate::decode::DecodedPacket;
use crate::geometry::crossed;
use crate::timing::{SensorClock, FIRINGS_PER_PACKET, PACKET_INTERVAL_US};
use lidar_core::{Frame, FrameStats, HostTime, PacketTime, Point, SensorTime};

/// 이보다 작게 뒤로 간 패킷은 순서 바뀜으로, 크게 뒤로 간 패킷은 흐름 재시작으로 본다.
const REORDER_WINDOW_US: i64 = 10_000;
/// 이보다 오래 끊겼다 다시 오면 흐름 재시작으로 본다.
const RESTART_GAP_US: i64 = 1_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StampPolicy {
    /// 프레임 마지막 패킷의 시각 (기본, 참조 구현들과 같음)
    LastPacket,
    /// 프레임 첫 패킷의 시각 (참조 구현의 `timestamp_first_packet` 옵션)
    FirstPacket,
}

/// 프레임 조립 중에 센 이상 상황.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AssemblerCounters {
    pub frames: u64,
    pub missing_packets: u64,
    pub reordered_packets: u64,
    pub restarts: u64,
    /// 손실 구간 동안 반 바퀴 넘게 돌아서 그 자리에서 끊은 횟수
    pub gap_cuts: u64,
    /// 다음 프레임의 점 배열을 새로 할당한 횟수 (돌려받은 것이 없어서). 돌려받으면 시작 직후 몇 번에서 멈춘다
    pub new_point_arrays: u64,
}

pub struct FrameAssembler {
    cut_deg: f64,
    stamp: StampPolicy,
    frame_id: String,
    clock: SensorClock,
    last_packet: Option<SensorTime>,
    last_firing_azimuth: Option<f64>,
    partial: Partial,
    /// 돌려받은 프레임의 배열 (비운 것). 다음 프레임을 모을 때 이것부터 쓴다
    spares: Vec<(Vec<Point>, Vec<PacketTime>)>,
    counters: AssemblerCounters,
}

impl FrameAssembler {
    pub fn new(cut_deg: f64, stamp: StampPolicy, frame_id: String) -> Self {
        FrameAssembler {
            cut_deg,
            stamp,
            frame_id,
            clock: SensorClock::default(),
            last_packet: None,
            last_firing_azimuth: None,
            partial: Partial::default(),
            spares: Vec::with_capacity(4),
            counters: AssemblerCounters::default(),
        }
    }

    /// 다 쓴 프레임을 돌려받는다. 그 배열들을 비워 두었다가 다음 프레임을 모을 때 쓴다.
    pub fn give_back(&mut self, frame: Frame) {
        let (mut points, mut packet_times) = (frame.points, frame.packet_times);
        points.clear();
        packet_times.clear();
        self.spares.push((points, packet_times));
    }

    pub fn counters(&self) -> AssemblerCounters {
        self.counters
    }

    /// 패킷 하나를 넣는다. 이 패킷에서 자르는 각도를 지나면 완성된 이전 프레임을 돌려준다.
    /// `received_at`은 호스트가 이 패킷을 받은 시각으로, 프레임의 `packet_times`에 그대로 실린다.
    pub fn push(&mut self, packet: &DecodedPacket, received_at: Option<HostTime>) -> Option<Frame> {
        let t = self.clock.unwrap(packet.timestamp_us);
        let mut finished = None;
        let mut gap_missing = 0; // 이 패킷 바로 앞에서 빠진 패킷 수 (그 자리에서 끊지 않았을 때)

        if let Some(last) = self.last_packet {
            let dt = t.micros_since(last);
            if dt <= 0 && dt > -REORDER_WINDOW_US {
                self.counters.reordered_packets += 1;
                return None;
            }
            if dt <= -REORDER_WINDOW_US || dt > RESTART_GAP_US {
                self.counters.restarts += 1;
                self.partial.clear();
                self.last_firing_azimuth = None;
            } else {
                let missing = ((dt as f64 / PACKET_INTERVAL_US).round() as u32).saturating_sub(1);
                self.partial.missing_packets += missing;
                self.counters.missing_packets += missing as u64;
                // 빠진 동안 반 바퀴 넘게 돌았으면, 방위각만으로는 몇 바퀴를 돌았는지(자르는 각도를 지났는지)
                // 알 수 없다 → 여기서 끊는다. 두 바퀴가 한 프레임에 섞이는 것을 막음.
                if missing > 0 && packet.rotation_deg_per_us * dt as f64 >= 180.0 {
                    self.counters.gap_cuts += 1;
                    if !self.partial.is_empty() {
                        finished = Some(self.take_frame(false));
                    }
                    self.last_firing_azimuth = None;
                } else {
                    gap_missing = missing;
                }
            }
        }
        self.last_packet = Some(t);

        let mut points = packet.points.iter().peekable();
        for (firing, &azimuth) in packet.firing_azimuths.iter().enumerate() {
            if let Some(last) = self.last_firing_azimuth {
                if crossed(last, azimuth, self.cut_deg) {
                    if !self.partial.is_empty() {
                        if finished.is_none() {
                            let mut frame = self.take_frame(true);
                            if firing == 0 && gap_missing > 0 {
                                // 자르는 각도가 빠진 구간 안에 있었다 → 빠진 패킷을 앞뒤 프레임에 나눔.
                                // 안 나누면 뒤 프레임은 시작 부분이 비었는데도 빠진 패킷 0인 한 바퀴로 보인다.
                                let (before, after) =
                                    split_missing(last, azimuth, self.cut_deg, gap_missing);
                                frame.stats.missing_packets -= gap_missing - before;
                                self.partial.missing_packets += after;
                            }
                            finished = Some(frame);
                        } else {
                            // 이 패킷에서 이미 프레임을 하나 내보냄 (손실로 끊은 직후 곧바로 자르는 각도를 지남).
                            // 그 사이 조각은 한 바퀴의 일부일 뿐이라 버린다.
                            self.partial.clear();
                        }
                    }
                    self.partial.starts_at_cut = true;
                }
            }
            self.last_firing_azimuth = Some(azimuth);

            self.partial.add_packet_once(PacketTime {
                sensor: t,
                received_at,
            });
            // 시각은 우선 이 프레임 첫 패킷 기준으로 넣는다. 대표 시각은 프레임이 끝나야 정해지므로 take()에서 옮김
            let since_first_us = t.micros_since(self.partial.first_packet()) as f64;
            while let Some(p) = points.next_if(|p| p.firing as usize == firing) {
                self.partial.points.push(Point {
                    x: p.xyz[0],
                    y: p.xyz[1],
                    z: p.xyz[2],
                    intensity: p.reflectivity as f32,
                    ring: p.ring,
                    time: ((since_first_us + p.offset_us) * 1e-6) as f32,
                });
            }
        }
        finished
    }

    /// 입력이 끝났을 때 모으던 프레임을 내보낸다.
    pub fn flush(&mut self) -> Option<Frame> {
        (!self.partial.is_empty()).then(|| self.take_frame(false))
    }

    fn take_frame(&mut self, ends_at_cut: bool) -> Frame {
        self.counters.frames += 1;
        let next = self.next_partial();
        let done = std::mem::replace(&mut self.partial, next);
        let mut frame = done.into_frame(self.stamp, &self.frame_id);
        frame.stats.ends_at_cut = ends_at_cut;
        frame
    }

    /// 다음 프레임을 모을 빈 자리. 돌려받은 배열이 있으면 그것을 쓴다. 없으면 새로 할당하되 이번 크기만큼
    /// 미리 잡는다 (빈 배열에서 두 배씩 늘리며 복사하지 않게).
    fn next_partial(&mut self) -> Partial {
        let (mut points, mut packet_times) = self.spares.pop().unwrap_or_else(|| {
            self.counters.new_point_arrays += 1;
            Default::default()
        });
        points.reserve(self.partial.points.len());
        packet_times.reserve(self.partial.packet_times.len());
        Partial {
            points,
            packet_times,
            ..Partial::default()
        }
    }
}

/// 빠진 구간 안에 자르는 각도가 있을 때, 빠진 패킷 `missing`개를 자르는 각도 앞(닫는 프레임)과 뒤(새 프레임)로 나눈다.
///
/// ```text
///  a0 = 빠지기 전 마지막 발사, a1 = 다시 받은 첫 발사. 그 사이에 빠진 발사가 missing × 24개, 간격은 같다
///  a0 │ · · · · · · · · · · · · · · · · · · · │ a1
///          패킷 1       패킷 2  ┊   패킷 3
///                        자르는 각도 → 앞 2개, 뒤 2개 (걸친 패킷 2는 양쪽 프레임 모두에서 빠졌으므로 양쪽에 셈)
/// ```
fn split_missing(a0: f64, a1: f64, cut: f64, missing: u32) -> (u32, u32) {
    let per_packet = FIRINGS_PER_PACKET as u32;
    let intervals = missing * per_packet + 1; // a0 → a1 사이 발사 간격 수
    let spacing = (a1 - a0).rem_euclid(360.0) / intervals as f64;
    // 자르는 각도 뒤의 첫 발사가 a0에서 몇 번째인가 (1 ..= intervals, intervals면 a1 자신)
    let first_after = ((cut - a0).rem_euclid(360.0) / spacing - 1e-6).ceil() as u32;
    let lost_before = first_after.clamp(1, intervals) - 1; // 자르는 각도 앞에서 빠진 발사 수
    (
        lost_before.div_ceil(per_packet),
        missing - lost_before / per_packet,
    )
}

/// 모으는 중인 프레임.
#[derive(Default)]
struct Partial {
    /// 포인트의 `time`은 아직 이 프레임 첫 패킷 기준 (대표 시각이 정해지기 전이라)
    points: Vec<Point>,
    packet_times: Vec<PacketTime>,
    missing_packets: u32,
    starts_at_cut: bool,
}

impl Partial {
    fn is_empty(&self) -> bool {
        self.packet_times.is_empty()
    }

    fn first_packet(&self) -> SensorTime {
        self.packet_times[0].sensor
    }

    /// 이 패킷의 발사가 처음 들어올 때 한 번만 넣는다 (한 패킷의 발사 24개가 각각 부르므로).
    fn add_packet_once(&mut self, packet: PacketTime) {
        if self.packet_times.last().map(|p| p.sensor) != Some(packet.sensor) {
            self.packet_times.push(packet);
        }
    }

    /// 모으던 것을 버린다. 배열의 자리는 그대로 두고 다시 쓴다 (새로 할당하지 않게).
    fn clear(&mut self) {
        self.points.clear();
        self.packet_times.clear();
        self.missing_packets = 0;
        self.starts_at_cut = false;
    }

    /// 프레임으로 만든다.
    ///
    /// 수신 스레드에서 프레임마다(초당 10번) 불리므로 점 배열(약 27,000개)을 만들거나 복사하거나 훑지 않는다:
    /// 모은 배열을 그대로 프레임에 넘기고, 시각 기준을 옮길 값은 `pending_time_shift`에 적어 두기만 한다
    /// (수신 루프가 다른 스레드에서 `Frame::settle_point_times`로 옮김). 보드(Cortex-A9)에서 변환·복사하던 때는
    /// 이 함수가 약 7 ms, 그 자리에서 옮기던 때는 1.9 ms 걸려 그동안 도착한 패킷이 소켓 큐에서 기다렸다
    /// (docs/modules/vlp16.md).
    fn into_frame(self, policy: StampPolicy, frame_id: &str) -> Frame {
        let first = self.first_packet();
        let stamp = match policy {
            StampPolicy::LastPacket => self.packet_times.last(),
            StampPolicy::FirstPacket => self.packet_times.first(),
        }
        .expect("into_frame() is called only on a non-empty partial")
        .sensor;

        Frame {
            sensor_stamp: stamp,
            stamp: None,
            frame_id: frame_id.to_owned(),
            points: self.points,
            packet_times: self.packet_times,
            stats: FrameStats {
                missing_packets: self.missing_packets,
                starts_at_cut: self.starts_at_cut,
                ends_at_cut: false, // take_frame이 정함
            },
            // 포인트 시각은 지금 첫 패킷 기준 → 대표 시각 기준으로 만들려면 이만큼 더 뺀다
            pending_time_shift: (stamp.micros_since(first) as f64 * 1e-6) as f32,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calibration::Calibration;
    use crate::decode::{decode_packet, RangeFilter};
    use crate::packet::{test_packet, PacketView};

    const RANGE: RangeFilter = RangeFilter {
        min_m: 0.1,
        max_m: 200.0,
    };
    /// 블록 간격 40 (0.4°) → 패킷 간격 480 (4.8°) → 75패킷이 한 바퀴
    const BLOCK_STEP: u16 = 40;

    fn packet(i: u32) -> DecodedPacket {
        let start = ((i * 12 * BLOCK_STEP as u32) % 36000) as u16;
        let ts = 1_000_000 + (i as f64 * PACKET_INTERVAL_US).round() as u32;
        let bytes = test_packet::build(ts, start, BLOCK_STEP, 5_000);
        decode_packet(
            PacketView::parse(&bytes).unwrap(),
            &Calibration::vlp16(),
            RANGE,
        )
        .unwrap()
    }

    fn assembler() -> FrameAssembler {
        FrameAssembler::new(90.0, StampPolicy::LastPacket, "velodyne".into())
    }

    #[test]
    fn cuts_at_the_firing_that_passes_the_cut_angle() {
        let mut a = assembler();
        let frames: Vec<Frame> = (0..200).filter_map(|i| a.push(&packet(i), None)).collect();

        // 첫 프레임: 시작(0°)부터 90°까지라 한 바퀴가 아님
        assert!(!frames[0].stats.starts_at_cut);
        // 그 뒤는 90° → 90°, 정확히 한 바퀴 = 발사 1800회
        for f in &frames[1..] {
            assert!(f.stats.is_full_turn());
            assert_eq!(f.points.len(), 1800 * 16);
            assert_eq!(f.stats.missing_packets, 0);
        }
        // 발사 하나 = 0.2°, 패킷 18은 86.4°에서 시작 → 발사 18이 정확히 90° → 패킷 18이 두 프레임에 걸침
        assert_eq!(frames[0].packet_times.len(), 19);
        assert_eq!(frames[1].packet_times.len(), 76);
        let shared = frames[0].packet_times.last().unwrap();
        assert_eq!(shared, frames[1].packet_times.first().unwrap());
    }

    #[test]
    fn point_times_are_relative_to_the_last_packet() {
        let mut a = assembler();
        let mut f = (0..200)
            .filter_map(|i| a.push(&packet(i), None))
            .nth(1)
            .unwrap();
        // 조립기는 옮길 값만 적어 두고(수신 스레드에서 점 배열을 훑지 않음), 옮기는 것은 settle_point_times
        assert!(f.pending_time_shift > 0.09, "{}", f.pending_time_shift);
        f.settle_point_times();
        assert_eq!(f.pending_time_shift, 0.0);
        let before = f.points[0].time;
        f.settle_point_times(); // 두 번 불러도 같다
        assert_eq!(f.points[0].time, before);
        let min = f
            .points
            .iter()
            .map(|p| p.time)
            .fold(f32::INFINITY, f32::min);
        let max = f
            .points
            .iter()
            .map(|p| p.time)
            .fold(f32::NEG_INFINITY, f32::max);
        assert!(min < -0.09 && min > -0.11, "{min}");
        assert!(max > 0.0 && max < 0.0014, "{max}");
    }

    #[test]
    fn counts_missing_packets_and_restarts() {
        let mut a = assembler();
        let mut frames = Vec::new();
        for i in (0..200).filter(|&i| i != 50 && i != 51) {
            frames.extend(a.push(&packet(i), None));
        }
        let lost: u32 = frames.iter().map(|f| f.stats.missing_packets).sum();
        assert_eq!(lost, 2);

        // 재생 반복처럼 시각이 크게 되돌아감
        a.push(&packet(0), None);
        assert_eq!(a.counters().restarts, 1);
    }

    #[test]
    fn a_long_loss_burst_cuts_instead_of_merging_two_turns() {
        // 패킷 85..124 손실 (40패킷 = 192°, 반 바퀴 넘음) → 자르는 각도(90°)를 지났는지 방위각만으로 알 수 없음
        let mut a = assembler();
        let frames: Vec<Frame> = (0..300)
            .filter(|i| !(85..125).contains(i))
            .filter_map(|i| a.push(&packet(i), None))
            .collect();
        assert_eq!(a.counters().gap_cuts, 1);
        // 어떤 프레임도 한 바퀴(발사 1800회 × 16)를 넘지 않음
        assert!(
            frames.iter().all(|f| f.points.len() <= 1800 * 16),
            "a frame spans more than one turn"
        );
        // 손실 직전 프레임은 끝이, 직후 프레임은 시작이 자르는 각도가 아님 → 온전한 한 바퀴로 세지 않음
        let cut = frames.iter().position(|f| !f.stats.ends_at_cut).unwrap();
        assert!(!frames[cut + 1].stats.starts_at_cut);
        assert_eq!(
            frames.iter().filter(|f| f.stats.is_full_turn()).count(),
            frames.len() - 3
        );
    }

    #[test]
    fn a_short_loss_across_the_cut_still_cuts_there() {
        // 패킷 15..20 손실 (28.8°): 90°가 들어 있는 패킷 18도 빠졌지만 반 바퀴 미만이라 정상적으로 자름
        let mut a = assembler();
        let frames: Vec<Frame> = (0..200)
            .filter(|i| !(15..21).contains(i))
            .filter_map(|i| a.push(&packet(i), None))
            .collect();
        assert_eq!(a.counters().gap_cuts, 0);
        assert!(frames[1].stats.is_full_turn());
        // 90°는 패킷 18 안: 15·16·17·18은 앞 프레임에서, 18·19·20은 뒤 프레임에서 빠졌다
        assert_eq!(frames[0].stats.missing_packets, 4);
        assert_eq!(frames[1].stats.missing_packets, 3);
        assert!(!frames[1].stats.is_complete());
        assert_eq!(a.counters().missing_packets, 6);
    }

    #[test]
    fn missing_packets_are_split_at_the_cut_by_angle() {
        // 위 시험의 빠진 구간: 패킷 14의 마지막 발사 71.8° → 패킷 21의 첫 발사 100.8°, 발사 간격 0.2°
        let split = |cut| split_missing(71.8, 100.8, cut, 6);
        assert_eq!(split(90.0), (4, 3)); // 패킷 18에 걸침 → 양쪽에 셈
        assert_eq!(split(76.8), (1, 5)); // 패킷 16의 첫 발사에서 자름 → 걸친 패킷 없음
        assert_eq!(split(71.9), (0, 6)); // 빠진 첫 발사보다 앞 → 앞 프레임은 잃은 것 없음
        assert_eq!(split(100.7), (6, 0)); // 빠진 마지막 발사보다 뒤 → 뒤 프레임은 잃은 것 없음
    }

    #[test]
    fn reuses_the_arrays_of_frames_given_back() {
        // 돌려주지 않으면 프레임마다 새 배열
        let mut a = assembler();
        let kept: Vec<Frame> = (0..300).filter_map(|i| a.push(&packet(i), None)).collect();
        assert_eq!(a.counters().new_point_arrays, kept.len() as u64);

        // 받자마자 돌려주면 처음 하나만 새로 할당하고, 배열 두 개가 번갈아 돈다
        let mut a = assembler();
        let mut arrays = Vec::new();
        for i in 0..300 {
            if let Some(f) = a.push(&packet(i), None) {
                arrays.push(f.points.as_ptr());
                a.give_back(f);
            }
        }
        assert_eq!(a.counters().new_point_arrays, 1);
        // 첫 프레임은 시작(0°)부터라 작고, 그 배열은 처음 다시 쓸 때 한 바퀴 크기로 한 번 늘어난다 → 그 뒤부터
        let mut later = arrays[2..].to_vec();
        later.sort();
        later.dedup();
        assert_eq!(later.len(), 2);

        // 다시 쓴 배열에도 같은 프레임이 모인다 (비워서 돌려받음)
        let f = (300..400).find_map(|i| a.push(&packet(i), None)).unwrap();
        assert!(f.stats.is_full_turn());
        assert_eq!(f.points.len(), 1800 * 16);
        assert_eq!(f.packet_times.len(), 76);
    }

    #[test]
    fn drops_reordered_packets() {
        let mut a = assembler();
        a.push(&packet(10), None);
        a.push(&packet(11), None);
        assert!(a.push(&packet(10), None).is_none());
        assert_eq!(a.counters().reordered_packets, 1);
    }
}
