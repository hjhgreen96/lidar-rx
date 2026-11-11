//! 센서 시계 → 호스트 시계 관계 추정.
//!
//! ```text
//!  패킷마다 (센서 시각 s, 커널 수신 시각 h)
//!        │
//!        ▼  y = h − s  (두 시계의 차이 + 수신 지연)
//!  ┌───────────────────────────────────────────────────────────┐
//!  │ ClockEstimator                                            │
//!  │   창(window, 기본 30 s) 안의 점들 (x = s, y = h − s)        │
//!  │   1초마다 다시 맞춤: 하한선(fit::lower_envelope)            │
//!  │                     + 비교용 최소제곱(fit::least_squares)    │
//!  │   계단 감지: 직선보다 크게 아래 → 즉시, 크게 위 1초 지속 → 리셋 │
//!  └───────────────────────────────────────────────────────────┘
//!        │
//!        ▼  ClockModel: h ≈ s + 기울기·(s − s₀) + 오프셋
//!  Frame::stamp = to_host(Frame::sensor_stamp)
//! ```
//!
//! - **기울기**(skew, ppm): 두 시계의 빠르기 차이. 1 ppm = 1초에 1 µs씩 벌어짐.
//! - **오프셋**: 하한선은 "가장 빨리 온 패킷"을 따라가므로, 최소 지연(전송 + 커널 처리)만큼은 여전히
//!   들어 있다. 한쪽 방향 측정으로는 이 상수를 알 수 없다 - PTP가 왕복 메시지로 푸는 문제.
//!
//! 자세한 설명: `docs/modules/clock.md`

pub mod fit;

use fit::{least_squares, lower_envelope, Line};
use lidar_core::{Frame, HostTime, SensorTime};
use std::collections::VecDeque;

#[derive(Clone, Debug, PartialEq)]
pub struct Config {
    /// 추정에 쓰는 최근 구간 (센서 시각 기준), µs
    pub window_us: i64,
    /// 이만큼 점이 쌓일 때마다 다시 맞춘다 (약 1초 분량)
    pub refit_every: usize,
    /// 첫 추정을 내기 전에 필요한 최소 구간, µs
    pub min_span_us: i64,
    /// 직선에서 이만큼 벗어나면 시계가 계단처럼 뛴 것으로 본다, ns
    pub step_threshold_ns: f64,
    /// 직선 위로 벗어난 상태가 이만큼 계속되면 계단으로 본다 (잠깐 붐빈 것과 구분), µs
    pub step_persist_us: i64,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            window_us: 30_000_000,
            refit_every: 754,
            min_span_us: 2_000_000,
            step_threshold_ns: 5_000_000.0,
            step_persist_us: 1_000_000,
        }
    }
}

/// 추정 결과. 하한선을 실제 변환에 쓰고, 최소제곱은 비교용으로 함께 들고 있다.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClockModel {
    origin: Origin,
    /// x = 센서 시각 − 기준 (µs), y = (수신 시각 − 센서 시각) − 기준 차이 (ns)
    pub envelope: Line,
    pub least_squares: Line,
    /// 맞출 때 쓴 점 수와 구간
    pub samples: usize,
    pub span_us: i64,
    /// 하한선 위로 떨어진 거리(= 최소 지연을 넘는 추가 지연)의 분포, ns
    pub residual_p50_ns: f64,
    pub residual_p99_ns: f64,
    pub residual_max_ns: f64,
    /// 창 안 점들의 x 평균 (최소제곱과 하한선을 비교하는 위치)
    mid_x: f64,
}

impl ClockModel {
    /// 두 시계의 빠르기 차이 (ppm). 양수면 호스트 시계가 센서 시계보다 빠르게 간다.
    pub fn skew_ppm(&self) -> f64 {
        self.envelope.slope * 1000.0 // ns/µs → ppm
    }

    pub fn least_squares_skew_ppm(&self) -> f64 {
        self.least_squares.slope * 1000.0
    }

    /// 창 가운데에서 최소제곱 직선이 하한선보다 얼마나 위에 있는가 (ns) = 대략 평균 추가 지연.
    pub fn least_squares_bias_ns(&self) -> f64 {
        self.least_squares.at(self.mid_x) - self.envelope.at(self.mid_x)
    }

    /// 센서 시각 → 호스트 시각.
    pub fn to_host(&self, sensor: SensorTime) -> HostTime {
        let (x, _) = self.origin.relative(sensor, HostTime(0));
        let diff_ns = self.origin.diff_ns + self.envelope.at(x).round() as i64;
        HostTime(sensor.0 * 1000 + diff_ns)
    }

    /// 이 점이 하한선보다 얼마나 위에 있는가 (ns). 음수면 직선 아래 (있을 수 없는 일 → 계단).
    pub fn residual_ns(&self, sensor: SensorTime, host: HostTime) -> f64 {
        let (x, y) = self.origin.relative(sensor, host);
        y - self.envelope.at(x)
    }
}

/// 계산을 f64로 하면서 정밀도를 잃지 않도록, 첫 점을 0으로 하는 상대값으로 바꾼다.
/// (호스트 시각 ns는 약 1.7 × 10¹⁸ - f64의 정확한 정수 범위 9 × 10¹⁵를 넘는다)
#[derive(Clone, Copy, Debug, PartialEq)]
struct Origin {
    sensor_us: i64,
    diff_ns: i64,
}

impl Origin {
    fn new(sensor: SensorTime, host: HostTime) -> Self {
        Origin {
            sensor_us: sensor.0,
            diff_ns: host.0 - sensor.0 * 1000,
        }
    }

    fn relative(&self, sensor: SensorTime, host: HostTime) -> (f64, f64) {
        let x = (sensor.0 - self.sensor_us) as f64;
        let y = (host.0 - sensor.0 * 1000 - self.diff_ns) as f64;
        (x, y)
    }
}

/// 추정기가 지금까지 센 것.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ClockStats {
    pub samples: u64,
    /// 센서 시각이 같거나 조금 뒤로 가 버린 점 (경계 패킷 중복, 순서 바뀜)
    pub ignored: u64,
    /// 다시 시작한 횟수 (계단, 흐름 재시작)
    pub resets: u64,
}

pub struct ClockEstimator {
    config: Config,
    origin: Option<Origin>,
    points: VecDeque<(f64, f64)>,
    last_sensor_us: Option<i64>,
    since_refit: usize,
    above_since_us: Option<i64>,
    model: Option<ClockModel>,
    stats: ClockStats,
    /// 다시 맞출 때 잔차를 담는 자리. 매번 새로 할당하지 않으려고 들고 있다
    residuals: Vec<f64>,
}

/// 이보다 크게 센서 시각이 뒤로 가면 흐름 재시작으로 본다.
const RESTART_BACKWARD_US: i64 = 10_000;

impl ClockEstimator {
    pub fn new(config: Config) -> Self {
        ClockEstimator {
            config,
            origin: None,
            points: VecDeque::new(),
            last_sensor_us: None,
            since_refit: 0,
            above_since_us: None,
            model: None,
            stats: ClockStats::default(),
            residuals: Vec::new(),
        }
    }

    pub fn model(&self) -> Option<&ClockModel> {
        self.model.as_ref()
    }

    pub fn stats(&self) -> ClockStats {
        self.stats
    }

    /// 프레임의 패킷 시각들을 모두 넣고, 프레임 대표 시각을 호스트 시각으로 바꿔 채운다.
    pub fn stamp_frame(&mut self, frame: &mut Frame) {
        for p in &frame.packet_times {
            if let Some(host) = p.received_at {
                self.observe(p.sensor, host);
            }
        }
        frame.stamp = self.model.map(|m| m.to_host(frame.sensor_stamp));
    }

    /// 점 하나를 넣는다.
    pub fn observe(&mut self, sensor: SensorTime, host: HostTime) {
        if let Some(last) = self.last_sensor_us {
            let back = last - sensor.0;
            if back >= RESTART_BACKWARD_US {
                self.reset();
            } else if back >= 0 {
                self.stats.ignored += 1;
                return;
            }
        }
        if self.is_step(sensor, host) {
            self.reset();
        }

        let origin = *self.origin.get_or_insert_with(|| Origin::new(sensor, host));
        self.points.push_back(origin.relative(sensor, host));
        self.last_sensor_us = Some(sensor.0);
        self.stats.samples += 1;

        self.since_refit += 1;
        if self.since_refit >= self.config.refit_every || self.model.is_none() {
            self.refit();
        }
    }

    /// 지금 모델로 보아 시계가 계단처럼 뛰었는가.
    fn is_step(&mut self, sensor: SensorTime, host: HostTime) -> bool {
        let Some(model) = self.model else {
            return false;
        };
        let r = model.residual_ns(sensor, host);
        if r < -self.config.step_threshold_ns {
            return true; // 가장 빨리 온 패킷보다도 일찍 옴 → 호스트 시계가 뒤로 감
        }
        if r > self.config.step_threshold_ns {
            let since = *self.above_since_us.get_or_insert(sensor.0);
            return sensor.0 - since >= self.config.step_persist_us; // 계속 늦음 → 앞으로 감
        }
        self.above_since_us = None;
        false
    }

    fn reset(&mut self) {
        self.stats.resets += 1;
        self.origin = None;
        self.points.clear();
        self.last_sensor_us = None;
        self.since_refit = 0;
        self.above_since_us = None;
        self.model = None;
    }

    fn refit(&mut self) {
        self.since_refit = 0;
        let newest = self.points.back().map_or(0.0, |p| p.0);
        while self
            .points
            .front()
            .is_some_and(|p| newest - p.0 > self.config.window_us as f64)
        {
            self.points.pop_front();
        }
        let (Some(first), Some(origin)) = (self.points.front(), self.origin) else {
            return;
        };
        let span_us = (newest - first.0) as i64;
        if span_us < self.config.min_span_us {
            return;
        }

        let points = self.points.make_contiguous();
        let (Some(envelope), Some(ls)) = (lower_envelope(points), least_squares(points)) else {
            return;
        };
        // 분위수 두 개와 최대값만 필요하므로 전부 정렬(O(N log N))하지 않고 그 순위만 고른다(O(N)).
        // 이 함수는 수신 스레드에서 1초마다 창 전체(30 s ≈ 22,600점)를 다루므로, 보드(Cortex-A9)에서는
        // 정렬만으로 수 ms가 걸려 그동안 도착한 패킷이 소켓 큐에서 기다렸다 (docs/modules/clock.md).
        let residuals = &mut self.residuals;
        residuals.clear();
        residuals.extend(points.iter().map(|&(x, y)| y - envelope.at(x)));
        let max = residuals.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let rank = |q: f64| ((residuals.len() - 1) as f64 * q) as usize;
        let (p50_at, p99_at) = (rank(0.5), rank(0.99));
        let p99 = *residuals.select_nth_unstable_by(p99_at, f64::total_cmp).1;
        let p50 = *residuals.select_nth_unstable_by(p50_at, f64::total_cmp).1;
        let mid_x = points.iter().map(|p| p.0).sum::<f64>() / points.len() as f64;

        self.model = Some(ClockModel {
            origin,
            envelope,
            least_squares: ls,
            samples: points.len(),
            span_us,
            residual_p50_ns: p50,
            residual_p99_ns: p99,
            residual_max_ns: max,
            mid_x,
        });
    }
}

#[cfg(test)]
mod tests;
