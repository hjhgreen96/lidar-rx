//! 패킷마다 "시작으로부터 몇 ns 뒤에 보낼지"를 정한다. 시계를 읽지 않는 순수 계산이라 테스트할 수 있다.
//!
//! ```text
//!  --pace capture   원래 캡처 시각 간격대로.  캡처 PC의 수신 흔들림(최대 약 1 ms)까지 그대로 재현
//!  --pace sensor    센서 타임스탬프 간격 × (1 + 드리프트).  흔들림 없는 깨끗한 간격
//!                   → "정답을 아는 시계 테스트": 받는 쪽 시계 추정기가 이 드리프트를 되찾아야 함
//!
//!  반복 재생 (--loops N): 9.4 s 파일을 이어 붙인다. 센서 타임스탬프도 이어지게 다시 씀
//!    반복 k의 패킷 i:  센서 시각 = 원래 시각 + k × 주기,  주기 = 파일 길이(센서 시각) + 패킷 간격 1327 µs
//! ```

use lidar_core::HostTime;
use std::borrow::Cow;
use vlp16::packet::PacketView;
use vlp16::timing::{SensorClock, PACKET_INTERVAL_US};

const HOUR_US: i64 = 3_600_000_000;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Pace {
    Capture,
    Sensor { drift_ppm: f64 },
}

/// 녹화된 패킷 하나.
struct Recorded {
    payload: Vec<u8>,
    /// 첫 패킷부터 캡처 시각 차이, ns
    capture_ns: i64,
    /// 첫 패킷부터 센서 시각 차이, µs (정시 넘김을 펼친 값)
    sensor_us: i64,
}

pub struct Schedule {
    packets: Vec<Recorded>,
    first_timestamp_us: u32,
    pace: Pace,
    speed: f64,
    /// 반복 한 번의 길이
    period_sensor_us: i64,
    period_capture_ns: i64,
}

impl Schedule {
    /// `records`: (캡처 시각, VLP-16 페이로드), 캡처 순서대로.
    pub fn new(records: Vec<(HostTime, Vec<u8>)>, pace: Pace, speed: f64) -> Result<Self, String> {
        if records.is_empty() {
            return Err("no packets".into());
        }
        if speed.is_nan() || speed <= 0.0 {
            return Err("speed must be positive".into());
        }
        let mut clock = SensorClock::default();
        let mut packets = Vec::with_capacity(records.len());
        let mut first: Option<(HostTime, i64)> = None;
        let mut first_timestamp_us = 0;
        for (i, (captured_at, payload)) in records.into_iter().enumerate() {
            let view = PacketView::parse(&payload).map_err(|e| format!("packet {i}: {e}"))?;
            let sensor = clock.unwrap(view.timestamp_us()).0;
            let (cap0, sensor0) = *first.get_or_insert((captured_at, sensor));
            if i == 0 {
                first_timestamp_us = view.timestamp_us();
            }
            packets.push(Recorded {
                capture_ns: captured_at.0 - cap0.0,
                sensor_us: sensor - sensor0,
                payload,
            });
        }
        let last = packets.last().unwrap();
        let interval_us = PACKET_INTERVAL_US.round() as i64;
        Ok(Schedule {
            period_sensor_us: last.sensor_us + interval_us,
            period_capture_ns: last.capture_ns + interval_us * 1000,
            packets,
            first_timestamp_us,
            pace,
            speed,
        })
    }

    pub fn packets_per_loop(&self) -> usize {
        self.packets.len()
    }

    /// 반복 한 번의 재생 시간, s
    pub fn loop_duration_s(&self) -> f64 {
        self.send_at_ns(1, 0) as f64 * 1e-9
    }

    /// 반복 `k`의 패킷 `i`를 보낼 시각: 시작으로부터 ns.
    pub fn send_at_ns(&self, k: u64, i: usize) -> u64 {
        let p = &self.packets[i];
        let ns = match self.pace {
            Pace::Capture => (p.capture_ns + k as i64 * self.period_capture_ns) as f64,
            Pace::Sensor { drift_ppm } => {
                let sensor_us = p.sensor_us + k as i64 * self.period_sensor_us;
                sensor_us as f64 * 1000.0 * (1.0 + drift_ppm * 1e-6)
            }
        };
        (ns / self.speed) as u64
    }

    /// 반복 `k`의 패킷 `i` 페이로드. 첫 반복은 원본 그대로, 그 뒤는 타임스탬프만 이어지게 바꾼 사본.
    pub fn payload(&self, k: u64, i: usize) -> Cow<'_, [u8]> {
        let p = &self.packets[i];
        if k == 0 {
            return Cow::Borrowed(&p.payload);
        }
        let sensor_us =
            self.first_timestamp_us as i64 + p.sensor_us + k as i64 * self.period_sensor_us;
        let mut copy = p.payload.clone();
        PacketView::set_timestamp_us(&mut copy, sensor_us.rem_euclid(HOUR_US) as u32);
        Cow::Owned(copy)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 센서 간격은 정확히 1327 µs, 캡처 간격은 짝수 번째만 300 µs 늦게 (수신 지터 모사)
    fn records(n: usize, first_ts: u32) -> Vec<(HostTime, Vec<u8>)> {
        (0..n)
            .map(|i| {
                let mut payload = vec![0u8; 1206];
                for b in 0..12 {
                    payload[b * 100] = 0xFF;
                    payload[b * 100 + 1] = 0xEE;
                }
                payload[1204] = 0x37;
                let ts = ((first_ts as u64 + i as u64 * 1327) % HOUR_US as u64) as u32;
                PacketView::set_timestamp_us(&mut payload, ts);
                let jitter = if i % 2 == 0 { 300_000 } else { 0 };
                (
                    HostTime(5_000_000_000 + i as i64 * 1_327_000 + jitter),
                    payload,
                )
            })
            .collect()
    }

    fn ts(p: &[u8]) -> u32 {
        PacketView::parse(p).unwrap().timestamp_us()
    }

    #[test]
    fn capture_pace_keeps_original_jitter() {
        let s = Schedule::new(records(10, 1000), Pace::Capture, 1.0).unwrap();
        assert_eq!(s.send_at_ns(0, 0), 0);
        assert_eq!(s.send_at_ns(0, 1), 1_327_000 - 300_000);
        assert_eq!(s.send_at_ns(0, 2), 2 * 1_327_000);
    }

    #[test]
    fn sensor_pace_applies_drift() {
        let s = Schedule::new(records(10, 1000), Pace::Sensor { drift_ppm: 100.0 }, 1.0).unwrap();
        let expected = 9.0 * 1_327_000.0 * 1.0001;
        assert!((s.send_at_ns(0, 9) as f64 - expected).abs() <= 1.0);
        let s = Schedule::new(records(10, 1000), Pace::Sensor { drift_ppm: 0.0 }, 2.0).unwrap();
        assert_eq!(s.send_at_ns(0, 2), 1_327_000);
    }

    #[test]
    fn loops_continue_time_and_timestamps_across_the_hour() {
        // 정시 직전에서 시작하는 파일
        let s = Schedule::new(
            records(10, 3_599_990_000),
            Pace::Sensor { drift_ppm: 0.0 },
            1.0,
        )
        .unwrap();
        assert_eq!(s.send_at_ns(1, 0), 10 * 1_327_000); // 마지막 패킷 다음 간격
        assert_eq!(ts(&s.payload(0, 3)), 3_599_990_000 + 3 * 1327);
        assert_eq!(
            ts(&s.payload(1, 0)),
            (3_599_990_000u64 + 10 * 1327 - 3_600_000_000) as u32
        );
    }
}
