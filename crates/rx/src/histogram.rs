//! 지연(µs) 분포를 고정 크기로 모으는 히스토그램.
//!
//! 값을 전부 저장하면 오래 돌수록 메모리가 는다 (초당 754개 → 하루 6500만 개). 그래서 구간(bucket)별 개수만 센다.
//!
//! ```text
//!  0 ~ 1023 µs        1 µs 단위 구간 1024개          (정확)
//!  1024 µs ~ 약 71분   2의 거듭제곱 구간마다 64등분     (상대 오차 1/64 ≈ 1.6% 이하, 위 끝 2^32 µs)
//!                     예: 1024~2047 µs는 16 µs 단위, 1~2 s는 약 16 ms 단위
//! ```

const LINEAR: u64 = 1024;
const SUB_BITS: u32 = 6; // 2의 거듭제곱 구간을 2^6 = 64등분
const LINEAR_BITS: u32 = 10; // LINEAR = 2^10
const BUCKETS: usize = LINEAR as usize + (32 - LINEAR_BITS as usize) * (1 << SUB_BITS);

#[derive(Clone)]
pub struct LatencyHistogram {
    counts: Vec<u64>,
    total: u64,
    max: u64,
}

impl Default for LatencyHistogram {
    fn default() -> Self {
        LatencyHistogram {
            counts: vec![0; BUCKETS],
            total: 0,
            max: 0,
        }
    }
}

impl LatencyHistogram {
    pub fn record(&mut self, micros: u64) {
        let micros = micros.min(u32::MAX as u64);
        self.counts[bucket(micros)] += 1;
        self.total += 1;
        self.max = self.max.max(micros);
    }

    pub fn count(&self) -> u64 {
        self.total
    }

    pub fn max(&self) -> u64 {
        self.max
    }

    /// q 분위수 (0 ≤ q ≤ 1), 그 구간의 아래 경계. 값이 없으면 0.
    pub fn quantile(&self, q: f64) -> u64 {
        if self.total == 0 {
            return 0;
        }
        let rank = ((self.total - 1) as f64 * q).round() as u64;
        let mut seen = 0;
        for (i, &n) in self.counts.iter().enumerate() {
            seen += n;
            if seen > rank {
                return lower_bound(i).min(self.max);
            }
        }
        self.max
    }
}

fn bucket(v: u64) -> usize {
    if v < LINEAR {
        return v as usize;
    }
    let exp = 63 - v.leading_zeros(); // floor(log2 v), 10 이상
    let sub = (v >> (exp - SUB_BITS)) & ((1 << SUB_BITS) - 1);
    LINEAR as usize + ((exp - LINEAR_BITS) as usize) * (1 << SUB_BITS) + sub as usize
}

fn lower_bound(i: usize) -> u64 {
    if i < LINEAR as usize {
        return i as u64;
    }
    let k = i - LINEAR as usize;
    let exp = (k >> SUB_BITS) as u32 + LINEAR_BITS;
    let sub = (k & ((1 << SUB_BITS) - 1)) as u64;
    (1u64 << exp) + (sub << (exp - SUB_BITS))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn small_values_are_exact() {
        let mut h = LatencyHistogram::default();
        for v in 0..1000 {
            h.record(v);
        }
        assert_eq!(h.quantile(0.5), 500);
        assert_eq!(h.quantile(0.0), 0);
        assert_eq!(h.quantile(1.0), 999);
        assert_eq!(h.max(), 999);
    }

    #[test]
    fn large_values_within_1_6_percent() {
        for v in [1024u64, 1500, 33_000, 1_000_000, 2_000_000_000] {
            let lb = lower_bound(bucket(v));
            assert!(lb <= v && (v - lb) as f64 <= v as f64 / 64.0, "{v} -> {lb}");
        }
    }

    #[test]
    fn buckets_are_monotonic() {
        for i in 1..BUCKETS {
            assert!(lower_bound(i) > lower_bound(i - 1));
        }
    }
}
