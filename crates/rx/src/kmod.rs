//! 커널 모듈 `lidar_mon`의 카운터 읽기 (`/proc/lidar_mon`, "이름 값" 줄들).
//!
//! 모듈이 적재돼 있으면 lidar-rx가 시작할 때와 끝날 때 읽어 차이를 요약에 보인다:
//!
//! ```text
//!  모듈이 IP 계층에서 본 수 − lidar-rx가 받은 수 = IP 계층과 사용자 공간 사이에서 사라진 수
//!                                               (대부분 소켓 수신 버퍼 넘침 = SO_RXQ_OVFL)
//! ```

use std::collections::HashMap;

pub const PROC_PATH: &str = "/proc/lidar_mon";

pub type Snapshot = HashMap<String, u64>;

/// 모듈이 없으면 `None`.
pub fn read() -> Option<Snapshot> {
    let text = std::fs::read_to_string(PROC_PATH).ok()?;
    Some(parse(&text))
}

fn parse(text: &str) -> Snapshot {
    text.lines()
        .filter_map(|line| {
            let (key, value) = line.split_once(' ')?;
            Some((key.to_owned(), value.trim().parse().ok()?))
        })
        .collect()
}

/// `after − before` (키가 없으면 0)
pub fn delta(before: &Snapshot, after: &Snapshot, key: &str) -> u64 {
    after
        .get(key)
        .copied()
        .unwrap_or(0)
        .saturating_sub(before.get(key).copied().unwrap_or(0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_proc_format() {
        let before = parse("port 2368\nseen 10\nlost 0\n");
        let after = parse("port 2368\nseen 7103\nlost 2\nlast_timestamp_us 277884689\n");
        assert_eq!(delta(&before, &after, "seen"), 7093);
        assert_eq!(delta(&before, &after, "lost"), 2);
        assert_eq!(after["port"], 2368);
    }
}
