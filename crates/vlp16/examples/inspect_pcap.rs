//! pcap 파일을 해석해 프레임마다 한 줄씩 요약한다.
//!
//!   cargo run -p vlp16 --example inspect_pcap -- data/vlp16.pcap
//!
//! 입력 출처(PcapSource)와 센서 모델(Vlp16Decoder)을 trait 객체로 받아 돌리는 모습이
//! 실제 수신 프로그램(rx)의 뼈대와 같다: 출처나 모델을 바꿔도 이 루프는 그대로다.

use lidar_core::{PacketSource, Received, SensorDecoder};
use lidar_io::PcapSource;
use vlp16::{Config, Vlp16Decoder};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .ok_or("usage: inspect_pcap <file.pcap>")?;
    let mut source: Box<dyn PacketSource> = Box::new(PcapSource::open(&path, 2368)?);
    let mut decoder = Vlp16Decoder::new(Config::default());

    println!("frame  points  packets  missing  full  stamp(s past hour)  time span(s)");
    let mut index = 0;
    let mut print = |f: lidar_core::Frame| {
        let (lo, hi) = f.points.iter().fold((f32::MAX, f32::MIN), |(lo, hi), p| {
            (lo.min(p.time), hi.max(p.time))
        });
        println!(
            "{index:>5}  {:>6}  {:>7}  {:>7}  {:>4}  {:>18.6}  {lo:+.4} .. {hi:+.4}",
            f.points.len(),
            f.packet_times.len(),
            f.stats.missing_packets,
            if f.stats.is_full_turn() { "yes" } else { "no" },
            (f.sensor_stamp.0 % 3_600_000_000) as f64 * 1e-6,
        );
        index += 1;
    };

    loop {
        match source.recv()? {
            Received::Packet(packet) => {
                if let Some(frame) = decoder.push(&packet)? {
                    print(frame);
                }
            }
            Received::Timeout => continue,
            Received::End => break,
        }
    }
    if let Some(frame) = decoder.flush() {
        print(frame);
    }
    println!("{:?}", decoder.stats());
    Ok(())
}
