//! 시계 재추정(refit) 한 번에 걸리는 시간을 잰다 - 보드(Cortex-A9)와 PC를 비교하려고.
//!
//! ```text
//!  cargo run --release -p clock --example refit_cost                                   PC
//!  cargo build --release -p clock --example refit_cost --target armv7-unknown-linux-musleabihf
//!      → target/armv7-unknown-linux-musleabihf/release/examples/refit_cost를 보드에서 실행
//! ```
//!
//! 754 pkt/s로 40초 분량의 (센서 시각, 호스트 시각)을 넣으며 `observe` 한 번에 걸린 시간을 잰다.
//! 평소에는 점 하나를 넣기만 하고, 754개(1초)마다 한 번 창(기본 30 s) 전체를 다시 맞춘다 → 그 순간만 오래 걸린다.
//! lidar-rx에서는 이 일이 수신 스레드에서 일어나므로, 그동안 도착한 패킷은 소켓 큐에서 기다린다.

use clock::{ClockEstimator, Config};
use lidar_core::{HostTime, SensorTime};
use std::time::Instant;

const PACKET_US: f64 = 1327.104;
const PER_SECOND: usize = 754;
const SECONDS: usize = 40;

fn main() {
    let config = Config::default();
    let window_s = config.window_us / 1_000_000;
    let mut clock = ClockEstimator::new(config);
    let base_ns: i64 = 1_700_000_000_000_000_000;
    let mut seed: u64 = 1;
    let mut slowest_per_second = vec![0u128; SECONDS];
    for i in 0..SECONDS * PER_SECOND {
        let sensor_us = (i as f64 * PACKET_US) as i64;
        // 지연 = 50 µs + 0 ~ 200 µs (결정적 의사 난수: 실행마다 같은 입력)
        seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let delay_ns = 50_000 + ((seed >> 33) % 200_000) as i64;
        let host_ns = base_ns + sensor_us * 1000 + delay_ns;

        let started = Instant::now();
        clock.observe(SensorTime(sensor_us), HostTime(host_ns));
        let ns = started.elapsed().as_nanos();
        let second = i / PER_SECOND;
        slowest_per_second[second] = slowest_per_second[second].max(ns);
    }
    println!("window {window_s} s; slowest observe() in each second (= the refit over the window)");
    println!(
        "{:>6} {:>14} {:>12}",
        "second", "window points", "slowest ms"
    );
    for (second, ns) in slowest_per_second.iter().enumerate().skip(2).step_by(4) {
        let points = (second + 1).min(window_s as usize) * PER_SECOND;
        println!("{second:>6} {points:>14} {:>12.2}", *ns as f64 / 1e6);
    }
}
