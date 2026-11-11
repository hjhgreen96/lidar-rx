//! 점들에 직선을 맞추는 두 방법. 시계와 무관한 순수 계산이다.
//!
//! ```text
//!  y (지연 포함 시각 차)
//!  │      ·        ·                ·            ·  : 패킷 하나 (x = 센서 시각, y = 수신 시각 − 센서 시각)
//!  │   ·    ·  ·      ·    ·   ·        ·   ·
//!  │ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─   최소제곱: 점들 한가운데 → 평균 지연만큼 위로 밀림
//!  │  ·  ·   ·  · ·  ·   ·  ·  ·  ·  ·   ·  ·
//!  │━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━   하한선: 점들 아래를 받치는 직선 → 가장 빨리 온 패킷들
//!  └────────────────────────────────────────────── x
//! ```
//!
//! 수신 지연은 음수가 될 수 없다(패킷은 늦게 올 수는 있어도 일찍 올 수는 없다). 그래서 점들은 "실제
//! 관계" 직선 위쪽에만 흩어지고, 그 직선에 가장 가까운 것은 아래쪽 경계다.

/// y = slope · x + intercept
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Line {
    pub slope: f64,
    pub intercept: f64,
}

impl Line {
    pub fn at(&self, x: f64) -> f64 {
        self.slope * x + self.intercept
    }
}

/// 최소제곱 직선: 세로 오차 제곱합이 가장 작은 직선. x가 모두 같으면 `None`.
pub fn least_squares(points: &[(f64, f64)]) -> Option<Line> {
    let n = points.len() as f64;
    let mean_x = points.iter().map(|p| p.0).sum::<f64>() / n;
    let mean_y = points.iter().map(|p| p.1).sum::<f64>() / n;
    let (mut sxx, mut sxy) = (0.0, 0.0);
    for &(x, y) in points {
        sxx += (x - mean_x) * (x - mean_x);
        sxy += (x - mean_x) * (y - mean_y);
    }
    (sxx > 0.0).then(|| {
        let slope = sxy / sxx;
        Line {
            slope,
            intercept: mean_y - slope * mean_x,
        }
    })
}

/// 하한선 (Moon, Skelly, Towsley, "Estimation and Removal of Clock Skew from Network Delay
/// Measurements", INFOCOM 1999).
///
/// 문제: 모든 점 아래(이하)에 있으면서 점들과의 세로 거리 합 Σ(yᵢ − a·xᵢ − b)가 가장 작은 직선.
///
/// ```text
///  Σ(yᵢ − a·xᵢ − b) = n·(ȳ − (a·x̄ + b))   → 직선이 x̄에서 가장 높을수록 작아진다
///  "모든 점 아래"이면서 x̄에서 가장 높은 직선 = 아래쪽 볼록 껍질에서 x̄를 가로지르는 변
///
///   ·        ·     ·
///      ·  ●───────────●  ·        ● : 아래쪽 볼록 껍질의 꼭짓점
///  ●──/        x̄       \───●      x̄를 가로지르는 변을 늘인 직선이 답
/// ```
///
/// `points`는 x 오름차순이어야 한다. 껍질의 변이 x̄를 가로지르지 못하면(점이 하나뿐 등) `None`.
pub fn lower_envelope(points: &[(f64, f64)]) -> Option<Line> {
    debug_assert!(
        points.windows(2).all(|w| w[0].0 <= w[1].0),
        "x must be sorted"
    );
    let hull = lower_hull(points);
    let mean_x = points.iter().map(|p| p.0).sum::<f64>() / points.len() as f64;
    hull.windows(2)
        .find(|e| e[0].0 <= mean_x && mean_x <= e[1].0 && e[0].0 < e[1].0)
        .map(|e| {
            let slope = (e[1].1 - e[0].1) / (e[1].0 - e[0].0);
            Line {
                slope,
                intercept: e[0].1 - slope * e[0].0,
            }
        })
}

/// 아래쪽 볼록 껍질 (Andrew의 monotone chain). x 오름차순 점들을 왼쪽부터 보며,
/// 마지막 두 꼭짓점과 새 점이 "왼쪽으로 꺾이지" 않으면 가운데 꼭짓점을 버린다.
fn lower_hull(points: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut hull: Vec<(f64, f64)> = Vec::new();
    for &p in points {
        while hull.len() >= 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0 {
            hull.pop();
        }
        hull.push(p);
    }
    hull
}

/// o→a와 o→b의 외적. 양수면 o→a→b가 왼쪽(반시계)으로 꺾인다.
fn cross(o: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_line_is_found_by_both() {
        let pts: Vec<(f64, f64)> = (0..100).map(|i| (i as f64, 2.0 * i as f64 + 5.0)).collect();
        for line in [least_squares(&pts).unwrap(), lower_envelope(&pts).unwrap()] {
            assert!((line.slope - 2.0).abs() < 1e-12 && (line.intercept - 5.0).abs() < 1e-9);
        }
    }

    #[test]
    fn envelope_ignores_positive_noise_least_squares_does_not() {
        // 실제 관계 y = 0.5x, 그 위로만 흩어진 지연 (점 셋 중 하나만 지연 0)
        let pts: Vec<(f64, f64)> = (0..300)
            .map(|i| {
                let delay = [0.0, 7.0, 3.0][i % 3];
                (i as f64, 0.5 * i as f64 + delay)
            })
            .collect();
        let env = lower_envelope(&pts).unwrap();
        let ls = least_squares(&pts).unwrap();
        assert!(
            (env.slope - 0.5).abs() < 1e-9 && env.intercept.abs() < 1e-6,
            "{env:?}"
        );
        // 최소제곱은 평균 지연(10/3)만큼 위로 밀림
        assert!((ls.intercept - 10.0 / 3.0).abs() < 0.1, "{ls:?}");
        // 하한선은 모든 점 아래
        assert!(pts.iter().all(|&(x, y)| y >= env.at(x) - 1e-9));
    }

    #[test]
    fn hull_keeps_only_lower_corners() {
        let pts = [(0.0, 0.0), (1.0, 5.0), (2.0, 1.0), (3.0, 4.0), (4.0, 0.0)];
        assert_eq!(lower_hull(&pts), vec![(0.0, 0.0), (4.0, 0.0)]);
    }
}
