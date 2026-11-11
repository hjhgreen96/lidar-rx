//! 정해진 시각까지 기다리기.
//!
//! ```text
//!  나쁜 방법: 매번 sleep(간격)        → sleep은 늘 조금 늦게 깸 → 늦음이 쌓여 점점 밀림
//!  이 방법:   시작 시각 + 예정 오프셋 = 절대 마감 시각
//!            마감 − 여유까지 sleep  → 남은 짧은 구간은 시계를 계속 보며 바쁜 대기(spin)
//!
//!   ──────── sleep ────────┬─ spin ─┐
//!                          │ 여유    │ 마감
//!  ────────────────────────┴────────┴────► 시간
//! ```
//!
//! 늦게 깨도 다음 마감은 원래 계획대로라 늦음이 쌓이지 않는다. 여유는 OS의 sleep 정밀도에 맞춘다:
//! 리눅스는 고해상도 타이머라 200 µs, Windows는 기본 타이머가 거칠어 2 ms.
//!
//! 늦음은 1 µs 단위 구간(0 ~ 10 ms, 그 위는 한 칸)에 세어 둔다 → `--loops 0`으로 끝없이 돌려도 메모리가 늘지 않음.

use std::time::{Duration, Instant};

#[cfg(windows)]
const SPIN_MARGIN: Duration = Duration::from_millis(2);
#[cfg(not(windows))]
const SPIN_MARGIN: Duration = Duration::from_micros(200);

/// 늦음을 1 µs 단위로 세는 범위. 이보다 늦은 것은 마지막 칸에 모음.
const LATENESS_RANGE_US: usize = 10_000;

pub struct Pacer {
    start: Instant,
    lateness_counts: Vec<u64>,
    total: u64,
    max_us: u64,
}

impl Pacer {
    pub fn start() -> Self {
        Pacer {
            start: Instant::now(),
            lateness_counts: vec![0; LATENESS_RANGE_US + 1],
            total: 0,
            max_us: 0,
        }
    }

    /// 시작으로부터 `at_ns`까지 기다린다. 이미 지났으면 바로 돌아온다.
    pub fn wait_until(&mut self, at_ns: u64) {
        let deadline = self.start + Duration::from_nanos(at_ns);
        let now = Instant::now();
        if let Some(remaining) = deadline.checked_duration_since(now) {
            if remaining > SPIN_MARGIN {
                std::thread::sleep(remaining - SPIN_MARGIN);
            }
            while Instant::now() < deadline {
                std::hint::spin_loop();
            }
        }
        let late_us = Instant::now()
            .saturating_duration_since(deadline)
            .as_micros() as u64;
        self.lateness_counts[(late_us as usize).min(LATENESS_RANGE_US)] += 1;
        self.total += 1;
        self.max_us = self.max_us.max(late_us);
    }

    /// 마감 시각들의 기준 (`send_at_ns` = 0인 순간)
    pub fn started_at(&self) -> Instant {
        self.start
    }

    pub fn elapsed(&self) -> Duration {
        self.start.elapsed()
    }

    /// 늦음 분포: (중앙값, p99, 최대, 100 µs 넘게 늦은 수), µs. 10 ms 넘는 분위수는 10000으로 보임.
    pub fn lateness(&self) -> (u64, u64, u64, u64) {
        let quantile = |q: f64| {
            let rank = ((self.total.saturating_sub(1)) as f64 * q) as u64;
            let mut seen = 0;
            for (us, &n) in self.lateness_counts.iter().enumerate() {
                seen += n;
                if seen > rank {
                    return us as u64;
                }
            }
            0
        };
        let over_100: u64 = self.lateness_counts[101..].iter().sum();
        (quantile(0.5), quantile(0.99), self.max_us, over_100)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lateness_summary_from_buckets() {
        let mut p = Pacer::start();
        for late in [0u64, 0, 5, 150, 20_000] {
            p.lateness_counts[(late as usize).min(LATENESS_RANGE_US)] += 1;
            p.total += 1;
            p.max_us = p.max_us.max(late);
        }
        let (p50, _p99, max, over_100) = p.lateness();
        assert_eq!((p50, max, over_100), (5, 20_000, 2));
    }
}
