//! 해석기 `push` 한 번에 걸리는 시간을 잰다 - 보드(Cortex-A9)와 PC를 비교하려고.
//!
//! ```text
//!  cargo run --release -p rx --example push_cost -- data/vlp16.pcap                   PC
//!  cargo build --release -p rx --example push_cost --target armv7-unknown-linux-musleabihf
//!      → 보드에서 ./push_cost vlp16.pcap
//! ```
//!
//! 녹화 파일의 패킷을 차례로 `decoder.push`에 넣고, 프레임을 내보낸 push(초당 10번)와 나머지 push의 시간을
//! 따로 모은다. lidar-rx에서는 이 일이 수신 스레드에서 일어나므로, 오래 걸리면 그동안 도착한 패킷이 기다린다.

use lidar_core::{PacketSource, Received, SensorDecoder};
use lidar_io::PcapSource;
use std::time::Instant;
use vlp16::{Config, Vlp16Decoder};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).unwrap_or("data/vlp16.pcap".into());
    let mut source = PcapSource::open(&path, 2368)?;
    let mut decoder = Vlp16Decoder::new(Config::default());
    let (mut normal, mut emitting) = (Vec::new(), Vec::new());
    while let Received::Packet(packet) = source.recv()? {
        let started = Instant::now();
        let frame = decoder.push(&packet)?;
        let us = started.elapsed().as_secs_f64() * 1e6;
        if frame.is_some() {
            emitting.push(us)
        } else {
            normal.push(us)
        }
        drop(frame); // 소비자 쪽에서 일어날 해제는 재지 않음
    }
    for (name, v) in [
        ("push without a frame", &mut normal),
        ("push that emits a frame", &mut emitting),
    ] {
        v.sort_by(f64::total_cmp);
        let q = |p: f64| v[((v.len() - 1) as f64 * p) as usize];
        println!(
            "{name:>24}: {:>5} pushes, p50 {:>8.1} us, p99 {:>8.1} us, max {:>8.1} us",
            v.len(),
            q(0.5),
            q(0.99),
            v[v.len() - 1]
        );
    }
    Ok(())
}
