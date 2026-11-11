//! 정답을 아는 합성 데이터로 추정기를 확인한다.
//!
//! 실제 관계: 호스트 = 센서 × (1 + 100 ppm) + 상수.  관측 = 실제 관계 + 최소 지연 50 µs + 추가 지연.
//! 추가 지연은 한쪽으로만 치우침: 평균 200 µs 지수 분포 + 1% 확률로 2 ms (붐빔).

use super::*;

const PACKET_US: f64 = 1327.104;
const SKEW: f64 = 100e-6;
const BASE_HOST_NS: i64 = 1_700_000_000_000_000_000;
const MIN_DELAY_NS: i64 = 50_000;

/// 재현 가능한 의사 난수 (xorshift64)
struct Rng(u64);
impl Rng {
    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn extra_delay_ns(&mut self) -> i64 {
        let exp = -200_000.0 * (1.0 - self.uniform()).ln();
        let spike = if self.uniform() < 0.01 {
            2_000_000.0
        } else {
            0.0
        };
        (exp + spike) as i64
    }
}

fn sensor(i: usize) -> SensorTime {
    SensorTime(1_000_000_000 + (i as f64 * PACKET_US) as i64)
}

/// 지연 없는 실제 관계
fn true_host(s: SensorTime) -> i64 {
    let since_start_ns = (s.0 - sensor(0).0) as f64 * 1000.0;
    BASE_HOST_NS + (since_start_ns * (1.0 + SKEW)) as i64
}

fn observed(s: SensorTime, rng: &mut Rng) -> HostTime {
    HostTime(true_host(s) + MIN_DELAY_NS + rng.extra_delay_ns())
}

fn run(packets: usize, mut host_shift_ns: impl FnMut(usize) -> i64) -> ClockEstimator {
    let mut est = ClockEstimator::new(Config::default());
    let mut rng = Rng(0x2545_F491_4F6C_DD1D);
    for i in 0..packets {
        let s = sensor(i);
        let h = observed(s, &mut rng);
        est.observe(s, HostTime(h.0 + host_shift_ns(i)));
    }
    est
}

const TWENTY_S: usize = 15_070; // 20 s / 1327.104 µs

#[test]
fn recovers_known_skew_and_offset() {
    let est = run(TWENTY_S, |_| 0);
    let m = est.model().unwrap();
    assert!((m.skew_ppm() - 100.0).abs() < 0.5, "skew {}", m.skew_ppm());

    // 변환 결과 = 실제 관계 + 최소 지연 (한쪽 방향 측정으로는 최소 지연을 뺄 수 없음)
    let s = sensor(TWENTY_S / 2);
    let err_us = (m.to_host(s).0 - (true_host(s) + MIN_DELAY_NS)) as f64 / 1000.0;
    assert!(err_us.abs() < 10.0, "offset error {err_us} us");

    // 최소제곱은 평균 추가 지연(약 220 µs)만큼 위로 밀림
    let bias_us = m.least_squares_bias_ns() / 1000.0;
    assert!(
        bias_us > 150.0 && bias_us < 300.0,
        "least squares bias {bias_us} us"
    );
}

#[test]
fn host_clock_stepping_back_resets_immediately() {
    let step_at = TWENTY_S / 2;
    let est = run(TWENTY_S, |i| if i >= step_at { -50_000_000 } else { 0 });
    assert_eq!(est.stats().resets, 1);
    // 계단 뒤 구간(약 10 s)만으로 다시 추정
    assert!((est.model().unwrap().skew_ppm() - 100.0).abs() < 1.0);
}

#[test]
fn host_clock_stepping_forward_resets_after_it_persists() {
    let step_at = TWENTY_S / 2;
    let est = run(TWENTY_S, |i| if i >= step_at { 50_000_000 } else { 0 });
    assert_eq!(est.stats().resets, 1);
    assert!((est.model().unwrap().skew_ppm() - 100.0).abs() < 1.0);
}

#[test]
fn short_congestion_is_not_a_step() {
    // 0.5 s 동안 모든 패킷이 20 ms 늦게 옴 (지속 기준 1 s보다 짧음)
    let est = run(TWENTY_S, |i| {
        if (8000..8377).contains(&i) {
            20_000_000
        } else {
            0
        }
    });
    assert_eq!(est.stats().resets, 0);
    assert!((est.model().unwrap().skew_ppm() - 100.0).abs() < 0.5);
}

#[test]
fn duplicates_are_ignored_and_a_big_backward_jump_restarts() {
    let mut est = ClockEstimator::new(Config::default());
    let mut rng = Rng(7);
    for i in 0..3000 {
        est.observe(sensor(i), observed(sensor(i), &mut rng));
        if i % 76 == 0 {
            est.observe(sensor(i), observed(sensor(i), &mut rng)); // 경계 패킷 중복
        }
    }
    assert_eq!(est.stats().ignored, 40);
    assert!(est.model().is_some());

    est.observe(sensor(0), observed(sensor(0), &mut rng)); // 재생 반복
    assert_eq!(est.stats().resets, 1);
    assert!(est.model().is_none());
}

#[test]
fn needs_two_seconds_before_the_first_estimate() {
    let est = run(1000, |_| 0); // 1.3 s
    assert!(est.model().is_none());
    let est = run(2000, |_| 0); // 2.7 s
    assert!(est.model().is_some());
}
