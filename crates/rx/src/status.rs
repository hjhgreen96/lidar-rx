//! 정해진 간격마다 상태 한 줄. 지난 줄 이후의 변화량(초당 값)과 누적값을 함께 보인다.
//!
//! ```text
//!  rx: 754 pkt/s 10.0 frm/s | missing 0 reorder 0 restart 0 kernel-drops 0 wait-p99 61 us | clock +100.02 ppm (lsq +99.71) p99 85 us | queue-drops stats 0
//! ```
//!
//! `wait-p99`: 커널이 패킷을 받은 뒤 이 프로그램이 꺼내기까지 기다린 시간의 99% 지점 (지금까지 누적).

use clock::ClockModel;
use lidar_core::DecoderCounters;
use std::time::{Duration, Instant};

pub struct Tick {
    pub decoder: DecoderCounters,
    pub kernel_drops: Option<u64>,
    pub clock: Option<ClockModel>,
    pub consumer_drops: Vec<(&'static str, u64)>,
    pub queue_wait_p99_us: Option<u64>,
}

pub struct Status {
    every: Duration,
    last_at: Instant,
    last: DecoderCounters,
    quiet: bool,
}

impl Status {
    pub fn new(every: Duration) -> Self {
        Status {
            every,
            last_at: Instant::now(),
            last: DecoderCounters::default(),
            quiet: false,
        }
    }

    /// 입력 출처가 타임아웃을 알림 (센서가 조용함)
    pub fn quiet(&mut self) {
        self.quiet = true;
    }

    /// 줄을 찍을 때가 됐나
    pub fn due(&self) -> bool {
        self.last_at.elapsed() >= self.every
    }

    /// 한 줄을 찍고 그 줄을 돌려준다 (systemd 상태로도 보내도록)
    pub fn print(&mut self, tick: Tick) -> String {
        let secs = self.last_at.elapsed().as_secs_f64();
        let d = tick.decoder;
        let pkt_rate = (d.packets - self.last.packets) as f64 / secs;
        let frame_rate = (d.frames - self.last.frames) as f64 / secs;

        let mut line = format!("rx: {pkt_rate:.0} pkt/s {frame_rate:.1} frm/s");
        if self.quiet && d.packets == self.last.packets {
            line += " (sensor quiet)";
        }
        line += &format!(
            " | missing {} reorder {} restart {}",
            d.missing_packets, d.reordered_packets, d.restarts
        );
        if let Some(drops) = tick.kernel_drops {
            line += &format!(" kernel-drops {drops}");
        }
        if let Some(p99) = tick.queue_wait_p99_us {
            line += &format!(" wait-p99 {p99} us");
        }
        match tick.clock {
            Some(m) => {
                line += &format!(
                    " | clock {:+.2} ppm (lsq {:+.2}) p99 {:.0} us",
                    m.skew_ppm(),
                    m.least_squares_skew_ppm(),
                    m.residual_p99_ns / 1000.0
                )
            }
            None => line += " | clock (collecting)",
        }
        if !tick.consumer_drops.is_empty() {
            line += " | queue-drops";
            for (name, n) in &tick.consumer_drops {
                line += &format!(" {name} {n}");
            }
        }
        eprintln!("{line}");

        self.last_at = Instant::now();
        self.last = d;
        self.quiet = false;
        line
    }
}
