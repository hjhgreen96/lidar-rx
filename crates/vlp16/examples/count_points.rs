//! 패킷 구간별 포인트 수. 프레임 자르는 방식이 다른 구현(공식 드라이버: 76패킷마다)과 개수를 맞춰 볼 때 쓴다.
//!
//!   cargo run -p vlp16 --example count_points -- data/vlp16.pcap 76 [저장할 파일]
//!
//! 패킷 0부터 `group`개씩 묶어, 묶음마다 거리 필터(0.1 ~ 200 m)를 통과한 포인트 수를 출력한다.
//! 마지막의 모자란 묶음은 뺀다 (공식 드라이버도 내보내지 않음). 파일을 주면 묶음마다 한 줄씩 쓴다.

use lidar_core::{PacketSource, Received};
use lidar_io::PcapSource;
use vlp16::calibration::Calibration;
use vlp16::decode::{decode_packet, RangeFilter};
use vlp16::packet::PacketView;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .ok_or("usage: count_points <file.pcap> [group=76]")?;
    let group: usize = args.next().map_or(Ok(76), |s| s.parse())?;
    if group == 0 {
        return Err("group must be at least 1".into());
    }
    let save = args.next();
    let range = RangeFilter {
        min_m: 0.1,
        max_m: 200.0,
    };
    let calibration = Calibration::vlp16();

    let mut source = PcapSource::open(&path, 2368)?;
    let mut per_packet = Vec::new();
    while let Received::Packet(p) = source.recv()? {
        let decoded = decode_packet(PacketView::parse(&p.payload)?, &calibration, range)?;
        per_packet.push(decoded.points.len());
    }
    let groups: Vec<usize> = per_packet
        .chunks_exact(group)
        .map(|c| c.iter().sum())
        .collect();
    if groups.is_empty() {
        return Err(format!("fewer than {group} packets in the file").into());
    }
    let mut sorted = groups.clone();
    sorted.sort_unstable();
    println!(
        "{} packets → {} groups of {group}: min {}, median {}, max {}, total {}",
        per_packet.len(),
        groups.len(),
        sorted[0],
        sorted[sorted.len() / 2],
        sorted[sorted.len() - 1],
        groups.iter().sum::<usize>()
    );
    let edge = groups.len().min(3);
    println!(
        "first {edge}: {:?}, last {edge}: {:?}",
        &groups[..edge],
        &groups[groups.len() - edge..]
    );
    if let Some(path) = save {
        let lines: Vec<String> = groups.iter().map(|g| g.to_string()).collect();
        std::fs::write(path, lines.join("\n") + "\n")?;
    }
    Ok(())
}
