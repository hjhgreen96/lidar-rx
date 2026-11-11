//! pcap에 녹화된 VLP-16 패킷을 UDP로 다시 보낸다 (센서 대신).
//!
//! ```text
//!  pcap-replay <file.pcap> [--to 127.0.0.1:2368] [--pace capture|sensor] [--drift-ppm 0]
//!                          [--speed 1.0] [--loops 1 (0 = 끝없이)] [--gap AT:FOR]
//!                          [--echo ADDR:PORT [--echo-relay ADDR:PORT] [--echo-csv FILE]]
//!
//!  pcap ──PcapSource──► (캡처 시각, 페이로드) 전부 메모리로 ──Schedule──► 보낼 시각
//!                                                                │ Pacer: 절대 마감까지 기다림
//!                                                                ▼
//!                                                        UdpSocket::send_to
//! ```
//!
//! `--gap AT:FOR`: 시작 AT 초부터 FOR 초 동안 보낼 패킷을 보내지 않는다 (선을 뽑은 것처럼: 센서는 계속 돌고 그동안의
//! 패킷만 사라짐). `--echo`: 받는 쪽이 되돌려 보낸 패킷으로 왕복 시간을 잰다 (`echo.rs`).
//!
//! 자세한 설명: `docs/modules/pcap-replay.md`

mod echo;
mod pacer;
mod schedule;

use echo::Echo;
use lidar_core::{PacketSource, Received};
use lidar_io::PcapSource;
use pacer::Pacer;
use schedule::{Pace, Schedule};
use std::fmt::Write as _;
use std::net::{SocketAddr, UdpSocket};
use std::time::Duration;

struct Args {
    file: String,
    to: SocketAddr,
    pace: Pace,
    speed: f64,
    /// `None` = 끝없이
    loops: Option<u64>,
    /// 보내지 않을 구간: 시작으로부터 [from, from + len) ns
    gap: Option<(u64, u64)>,
    echo: Option<SocketAddr>,
    echo_relay: Option<SocketAddr>,
    echo_csv: Option<String>,
}

const USAGE: &str = "usage: pcap-replay <file.pcap> [--to ADDR:PORT] [--pace capture|sensor] \
                     [--drift-ppm X] [--speed X] [--loops N (0 = forever)] [--gap AT:FOR (s)] \
                     [--echo ADDR:PORT [--echo-relay ADDR:PORT] [--echo-csv FILE]]";

fn parse_args() -> Result<Args, String> {
    let mut args = std::env::args().skip(1);
    let mut file = None;
    let mut to: SocketAddr = "127.0.0.1:2368".parse().unwrap();
    let mut pace_name = "capture".to_string();
    let mut drift_ppm = 0.0;
    let mut speed = 1.0;
    let mut loops = Some(1);
    let mut gap = None;
    let (mut echo, mut echo_relay, mut echo_csv) = (None, None, None);
    let addr = |name: &str, v: String| -> Result<SocketAddr, String> {
        v.parse().map_err(|e| format!("{name}: {e}"))
    };
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or(format!("{arg} needs a value"));
        match arg.as_str() {
            "--to" => to = value()?.parse().map_err(|e| format!("--to: {e}"))?,
            "--pace" => pace_name = value()?,
            "--drift-ppm" => {
                drift_ppm = value()?.parse().map_err(|e| format!("--drift-ppm: {e}"))?
            }
            "--speed" => speed = value()?.parse().map_err(|e| format!("--speed: {e}"))?,
            "--loops" => {
                let n: u64 = value()?.parse().map_err(|e| format!("--loops: {e}"))?;
                loops = (n > 0).then_some(n);
            }
            "--gap" => {
                let v = value()?;
                let (at, len) = v.split_once(':').ok_or("--gap: AT:FOR in seconds")?;
                let ns = |s: &str| {
                    s.parse::<f64>()
                        .map(|s| (s * 1e9) as u64)
                        .map_err(|e| format!("--gap: {e}"))
                };
                gap = Some((ns(at)?, ns(len)?));
            }
            "--echo" => echo = Some(addr("--echo", value()?)?),
            "--echo-relay" => echo_relay = Some(addr("--echo-relay", value()?)?),
            "--echo-csv" => echo_csv = Some(value()?),
            "-h" | "--help" => return Err(USAGE.into()),
            _ if file.is_none() && !arg.starts_with('-') => file = Some(arg),
            _ => return Err(format!("unknown argument {arg}\n{USAGE}")),
        }
    }
    if echo.is_none() && (echo_relay.is_some() || echo_csv.is_some()) {
        return Err("--echo-relay and --echo-csv need --echo".into());
    }
    if echo.is_some() && loops.is_none() {
        return Err("--echo keeps a record per packet: give --loops N".into());
    }
    let pace = match pace_name.as_str() {
        "capture" if drift_ppm != 0.0 => return Err("--drift-ppm needs --pace sensor".into()),
        "capture" => Pace::Capture,
        "sensor" => Pace::Sensor { drift_ppm },
        other => return Err(format!("unknown pace {other}")),
    };
    Ok(Args {
        file: file.ok_or(USAGE)?,
        to,
        pace,
        speed,
        loops,
        gap,
        echo,
        echo_relay,
        echo_csv,
    })
}

fn main() {
    if let Err(e) = run() {
        eprintln!("pcap-replay: {e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args()?;

    let mut source = PcapSource::open(&args.file, 2368)?;
    let mut records = Vec::new();
    while let Received::Packet(p) = source.recv()? {
        records.push((p.received_at.ok_or("pcap record without time")?, p.payload));
    }
    let schedule = Schedule::new(records, args.pace, args.speed)?;
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    // VLP-16의 기본 목적지는 브로드캐스트(255.255.255.255:2368)라 그런 주소로도 보낼 수 있게 (없으면 EACCES)
    socket.set_broadcast(true)?;

    eprintln!(
        "pcap-replay: {} packets/loop, {:.2} s/loop, pace {:?}, speed {}, loops {}, to {}",
        schedule.packets_per_loop(),
        schedule.loop_duration_s(),
        args.pace,
        args.speed,
        args.loops.map_or("forever".into(), |n| n.to_string()),
        args.to,
    );

    let echo = args
        .echo
        .map(|listen| Echo::start(listen, args.echo_relay))
        .transpose()?;
    if let Some((from, len)) = args.gap {
        eprintln!(
            "pcap-replay: gap: nothing sent from {:.3} s for {:.3} s",
            from as f64 * 1e-9,
            len as f64 * 1e-9
        );
    }
    if let Some(echo) = &echo {
        eprintln!(
            "pcap-replay: echo: listening on {}{}",
            echo.local_addr(),
            args.echo_relay
                .map_or(String::new(), |to| format!(", relaying to {to}"))
        );
    }

    let mut pacer = Pacer::start();
    let (mut sent, mut skipped): (u64, u64) = (0, 0);
    for k in (0..).take_while(|&k| args.loops.is_none_or(|n| k < n)) {
        for i in 0..schedule.packets_per_loop() {
            let at = schedule.send_at_ns(k, i);
            if args
                .gap
                .is_some_and(|(from, len)| at >= from && at - from < len)
            {
                skipped += 1;
                continue;
            }
            pacer.wait_until(at);
            let payload = schedule.payload(k, i);
            if let Some(echo) = &echo {
                echo.note_sent(&payload);
            }
            socket.send_to(&payload, args.to)?;
            sent += 1;
        }
    }

    let planned_s =
        schedule.send_at_ns(args.loops.unwrap_or(1) - 1, schedule.packets_per_loop() - 1) as f64
            * 1e-9;
    let (p50, p99, max, over_100) = pacer.lateness();
    eprintln!(
        "pcap-replay: sent {sent} packets in {:.3} s (planned {planned_s:.3} s){}; \
         lateness p50 {p50} us, p99 {p99} us, max {max} us, >100 us: {over_100}",
        pacer.elapsed().as_secs_f64(),
        if args.gap.is_some() {
            format!(", skipped {skipped} in the gap")
        } else {
            String::new()
        },
    );

    if let Some(echo) = echo {
        let report = echo.finish(Duration::from_millis(500))?;
        if let Some(path) = &args.echo_csv {
            let mut csv = String::from("send_s,rtt_us\n");
            for e in &report.echoed {
                let send_s = e
                    .sent_at
                    .saturating_duration_since(pacer.started_at())
                    .as_secs_f64();
                writeln!(csv, "{send_s:.6},{}", e.round_trip.as_micros())?;
            }
            std::fs::write(path, csv)?;
        }
        let mut rtt_us: Vec<u128> = report
            .echoed
            .iter()
            .map(|e| e.round_trip.as_micros())
            .collect();
        rtt_us.sort_unstable();
        let q = |p: f64| {
            let rank = (rtt_us.len().saturating_sub(1) as f64 * p) as usize;
            rtt_us.get(rank).copied().unwrap_or(0)
        };
        eprintln!(
            "pcap-replay: echo: {} of {sent} came back ({} unknown); \
             round trip p50 {} us, p99 {} us, max {} us",
            rtt_us.len(),
            report.unknown,
            q(0.5),
            q(0.99),
            rtt_us.last().copied().unwrap_or(0),
        );
    }
    Ok(())
}
