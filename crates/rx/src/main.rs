//! LiDAR 수신 프로그램. 출처·해석기·시계 추정·소비자를 조립해 돌린다 - 크레이트들을 엮는 유일한 곳.
//!
//! ```text
//!  lidar-rx (--udp ADDR:PORT | --pcap FILE) [--rcvbuf BYTES] [--mode threaded|inline] [--queue N]
//!           [--duration S] [--status S] [--ros2] [--stall-ms T --stall-after N]
//!           [--work-ms D] [--cpu C] [--rt-prio P] [--lock-memory] [--forward ADDR:PORT [--forward-tx-log FILE]]
//!
//!   ┌ PacketSource ┐   ┌ SensorDecoder ┐   ┌ ClockEstimator ┐   ┌ FrameConsumer들 ┐
//!   │ UdpSource    │──►│ Vlp16Decoder  │──►│ stamp_frame    │──►│ StatsConsumer   │
//!   │ PcapSource   │   └───────────────┘   └────────────────┘   │ Ros2Publisher   │
//!   └──────────────┘                                            └─────────────────┘
//!   --forward: 출처를 ForwardSource로 감싸 받은 패킷을 그대로 다른 주소로도 보냄 (보드 → PC의 RViz)
//!   --forward-tx-log: 진단. 넘긴 패킷마다 커널 수신 시각과 NIC 드라이버에 넘긴 시각을 파일로 (길이가 정해진 실행에만)
//! ```
//!
//! 자세한 설명: `docs/modules/rx.md`

mod consumer;
mod histogram;
mod kmod;
mod notify;
mod pipeline;
#[cfg(feature = "ros2")]
mod ros2;
mod sched;
mod signal;
mod status;

use consumer::{FrameConsumer, StatsConsumer, WorkConsumer};
use lidar_core::{PacketSource, SensorDecoder};
use lidar_io::PcapSource;
use notify::Notifier;
use pipeline::{LoopHooks, Mode, Outcome, RunOptions, Stall};
use sched::ThreadSetup;
use std::time::Duration;
use vlp16::{Config, Vlp16Decoder};

enum Input {
    Udp(std::net::SocketAddr),
    Pcap(String),
}

struct Args {
    input: Input,
    rcvbuf: Option<usize>,
    run: RunOptions,
    ros2: bool,
    /// 실험 3: 프레임마다 이만큼 CPU를 쓰는 소비자를 붙임
    work: Option<Duration>,
    /// 받은 패킷을 그대로 이 주소로도 보냄
    forward: Option<std::net::SocketAddr>,
    /// 진단: 넘긴 패킷마다 (커널 수신 시각, NIC 드라이버에 넘긴 시각)을 끝날 때 이 파일에
    forward_tx_log: Option<String>,
}

const USAGE: &str = "usage: lidar-rx (--udp ADDR:PORT | --pcap FILE) [--rcvbuf BYTES] \
                     [--mode threaded|inline] [--queue N] [--duration S] [--status S] [--ros2] \
                     [--stall-ms T [--stall-after N]] [--work-ms D] [--cpu C] [--rt-prio P] \
                     [--lock-memory] [--forward ADDR:PORT [--forward-tx-log FILE]]";

fn parse_args() -> Result<Args, String> {
    let mut args = std::env::args().skip(1);
    let mut input = None;
    let mut rcvbuf = None;
    let mut mode_name = "threaded".to_string();
    let mut queue = 4;
    let mut duration = None;
    let mut status_every = Duration::from_secs(1);
    let mut ros2 = false;
    let mut stall_ms: Option<u64> = None;
    let mut stall_after = 2000;
    let mut work_ms: Option<u64> = None;
    let mut receive_thread = ThreadSetup::default();
    let mut forward = None;
    let mut forward_tx_log = None;
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or(format!("{arg} needs a value"));
        // 음수·NaN·너무 큰 값은 Duration으로 못 바꾼다 (from_secs_f64는 그때 패닉)
        let secs = |v: String| {
            v.parse::<f64>()
                .map_err(|e| e.to_string())
                .and_then(|s| Duration::try_from_secs_f64(s).map_err(|e| e.to_string()))
                .map_err(|e| format!("{arg}: {e}"))
        };
        let num = |v: String, name: &str| v.parse::<u64>().map_err(|e| format!("{name}: {e}"));
        match arg.as_str() {
            "--udp" => {
                input = Some(Input::Udp(
                    value()?.parse().map_err(|e| format!("--udp: {e}"))?,
                ))
            }
            "--pcap" => input = Some(Input::Pcap(value()?)),
            "--rcvbuf" => rcvbuf = Some(num(value()?, "--rcvbuf")? as usize),
            "--mode" => mode_name = value()?,
            "--queue" => queue = num(value()?, "--queue")? as usize,
            "--duration" => duration = Some(secs(value()?)?),
            "--status" => status_every = secs(value()?)?,
            "--ros2" => ros2 = true,
            "--stall-ms" => stall_ms = Some(num(value()?, "--stall-ms")?),
            "--stall-after" => stall_after = num(value()?, "--stall-after")?,
            "--work-ms" => work_ms = Some(num(value()?, "--work-ms")?),
            "--cpu" => receive_thread.cpu = Some(num(value()?, "--cpu")? as usize),
            "--rt-prio" => match num(value()?, "--rt-prio")? {
                p @ 1..=99 => receive_thread.rt_priority = Some(p as i32),
                p => return Err(format!("--rt-prio: {p} is outside 1..=99")),
            },
            "--lock-memory" => receive_thread.lock_memory = true,
            "--forward" => forward = Some(value()?.parse().map_err(|e| format!("--forward: {e}"))?),
            "--forward-tx-log" => forward_tx_log = Some(value()?),
            _ => return Err(format!("unknown argument {arg}\n{USAGE}")),
        }
    }
    if forward_tx_log.is_some() && forward.is_none() {
        return Err("--forward-tx-log needs --forward".into());
    }
    let from_file = matches!(input, Some(Input::Pcap(_)));
    let mode = match mode_name.as_str() {
        "threaded" => Mode::Threaded { queue },
        "inline" => Mode::Inline,
        other => return Err(format!("unknown mode {other}")),
    };
    Ok(Args {
        input: input.ok_or(USAGE)?,
        rcvbuf,
        run: RunOptions {
            mode,
            duration,
            status_every,
            stall: stall_ms.map(|ms| Stall {
                after_packets: stall_after,
                duration: Duration::from_millis(ms),
            }),
            receive_thread,
            from_file,
        },
        ros2,
        work: work_ms.map(Duration::from_millis),
        forward,
        forward_tx_log,
    })
}

fn main() {
    if let Err(e) = run() {
        eprintln!("lidar-rx: {e}");
        std::process::exit(1);
    }
}

fn open_source(args: &Args) -> Result<Box<dyn PacketSource>, Box<dyn std::error::Error>> {
    Ok(match &args.input {
        Input::Pcap(path) => Box::new(PcapSource::open(path, 2368)?),
        #[cfg(target_os = "linux")]
        Input::Udp(addr) => {
            let options = lidar_io::UdpOptions {
                recv_buffer_bytes: args.rcvbuf,
                ..Default::default()
            };
            let source = lidar_io::UdpSource::bind(*addr, &options)?;
            eprintln!(
                "lidar-rx: listening on {}, socket receive buffer {} B (requested {:?})",
                source.local_addr()?,
                source.recv_buffer_bytes()?,
                args.rcvbuf
            );
            Box::new(source)
        }
        #[cfg(not(target_os = "linux"))]
        Input::Udp(_) => return Err("UDP input is implemented for Linux only".into()),
    })
}

fn consumers(args: &Args) -> Result<Vec<Box<dyn FrameConsumer>>, Box<dyn std::error::Error>> {
    let mut list: Vec<Box<dyn FrameConsumer>> = vec![Box::new(StatsConsumer::default())];
    if let Some(work) = args.work {
        list.push(Box::new(WorkConsumer::new(work)));
    }
    if args.ros2 {
        #[cfg(feature = "ros2")]
        list.push(Box::new(ros2::Ros2Publisher::new(
            "lidar_rx",
            "velodyne_points",
        )?));
        #[cfg(not(feature = "ros2"))]
        return Err("built without the ros2 feature (cargo build --features ros2)".into());
    }
    Ok(list)
}

/// 수신 루프의 순간들을 systemd·종료 신호·커널 모듈 카운터에 잇는다.
struct Hooks {
    notify: Notifier,
    kmod_after: Option<kmod::Snapshot>,
}

impl LoopHooks for Hooks {
    fn ready(&mut self) {
        self.notify.send("READY=1\nSTATUS=receiving");
    }
    fn alive(&mut self) {
        self.notify.alive();
    }
    fn status(&mut self, line: &str) {
        self.notify.send(&format!("STATUS={line}"));
    }
    fn should_stop(&self) -> bool {
        signal::stop_requested()
    }
    fn loop_end(&mut self) {
        self.kmod_after = kmod::read();
        self.notify.send("STOPPING=1");
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args()?;
    signal::install()?;
    let mut hooks = Hooks {
        notify: Notifier::from_env()?,
        kmod_after: None,
    };
    if let Some(every) = hooks.notify.watchdog_every() {
        eprintln!("lidar-rx: systemd watchdog: notifying every {every:?}");
    }
    let mut source = open_source(&args)?;
    let forwarded = match args.forward {
        Some(to) => {
            let mut wrapped = lidar_io::ForwardSource::new(source, to)?;
            if args.forward_tx_log.is_some() {
                wrapped = wrapped.with_tx_timestamps()?;
            }
            let counts = wrapped.counts();
            source = Box::new(wrapped);
            eprintln!("lidar-rx: forwarding every received packet to {to}");
            Some((to, counts))
        }
        None => None,
    };
    let mut decoder = Vlp16Decoder::new(Config::default());
    let clock = clock::ClockEstimator::new(clock::Config::default());

    let kmod_before = kmod::read();
    let outcome = pipeline::run(
        source.as_mut(),
        &mut decoder,
        clock,
        consumers(&args)?,
        &args.run,
        &mut hooks,
    )?;
    if signal::stop_requested() {
        eprintln!("lidar-rx: stopped by signal");
    }

    print_summary(
        &decoder,
        source.as_ref(),
        &outcome,
        kmod_before.zip(hooks.kmod_after),
    );
    if let Some((to, counts)) = forwarded {
        eprintln!(
            "forward: {} packets to {to}, {} failed",
            counts.forwarded.get(),
            counts.failed.get()
        );
        if let (Some(path), Some(log)) = (&args.forward_tx_log, counts.tx_log.borrow().as_ref()) {
            write_tx_log(path, log)?;
        }
    }
    Ok(())
}

/// `--forward-tx-log`: 패킷마다 한 줄 + 요약 (커널 도착 → NIC 드라이버에 넘김)
fn write_tx_log(path: &str, log: &[(i64, Option<i64>)]) -> std::io::Result<()> {
    use std::fmt::Write as _;
    let mut csv = String::from("index,rx_ns,tx_ns\n");
    let mut turnaround_us: Vec<i64> = Vec::with_capacity(log.len());
    for (i, &(rx, tx)) in log.iter().enumerate() {
        let _ = writeln!(
            csv,
            "{i},{rx},{}",
            tx.map_or(String::new(), |t| t.to_string())
        );
        if let Some(tx) = tx {
            turnaround_us.push((tx - rx) / 1000);
        }
    }
    std::fs::write(path, csv)?;
    turnaround_us.sort_unstable();
    let q = |p: f64| turnaround_us[((turnaround_us.len() - 1) as f64 * p) as usize];
    if turnaround_us.is_empty() {
        eprintln!("forward: no transmit timestamps came back ({path})");
    } else {
        eprintln!(
            "forward: transmit timestamps for {} of {} packets -> {path}; kernel arrival -> driver p50 {} us, p99 {} us, max {} us",
            turnaround_us.len(),
            log.len(),
            q(0.5),
            q(0.99),
            turnaround_us[turnaround_us.len() - 1],
        );
    }
    Ok(())
}

fn print_summary(
    decoder: &Vlp16Decoder,
    source: &dyn PacketSource,
    outcome: &Outcome,
    kmod: Option<(kmod::Snapshot, kmod::Snapshot)>,
) {
    let clock = &outcome.clock;
    let s = decoder.stats();
    let a = s.assembler;
    eprintln!("--- summary");
    if let Some(thread) = &outcome.receive_thread {
        eprintln!("receive thread: {thread} (read back from the kernel)");
    }
    eprintln!(
        "decoder: {} packets, {} frames, malformed {}, product-id mismatches {}, missing {}, reordered {}, restarts {}, new point arrays {}",
        s.packets, a.frames, s.malformed_packets, s.product_id_mismatches, a.missing_packets, a.reordered_packets, a.restarts,
        a.new_point_arrays
    );
    if let Some(drops) = source.kernel_drops() {
        eprintln!("socket: kernel drops (SO_RXQ_OVFL) {drops}");
    }
    let w = &outcome.queue_wait;
    if w.count() > 0 {
        // 최대값을 꺼낸 시각(CLOCK_REALTIME 초)도 보임: 같은 순간의 다른 측정과 맞춰 볼 수 있게
        let max_at = outcome.queue_wait_max_at_ns.map_or(String::new(), |ns| {
            format!("; max at {:.6} s", ns as f64 * 1e-9)
        });
        eprintln!(
            "queue wait (kernel receive → read here): p50 {} us, p99 {} us, max {} us over {} packets{max_at}",
            w.quantile(0.5),
            w.quantile(0.99),
            w.max(),
            w.count()
        );
    }
    if let Some(r) = outcome.stall {
        eprintln!(
            "stall: paused {} ms after {} packets; {} packets were queued when reading resumed (oldest waited {} us); \
             kernel drops during the stall {}",
            r.stall.duration.as_millis(),
            r.stall.after_packets,
            r.queued,
            r.oldest_wait_us,
            r.kernel_drops_during().map_or("n/a".into(), |d| d.to_string())
        );
    }
    if let Some((before, after)) = kmod {
        let seen = kmod::delta(&before, &after, "seen");
        eprintln!(
            "kernel module (IP layer, {}): seen {seen}, lost {}, reordered {}, restarts {}; \
             seen − received here = {}",
            kmod::PROC_PATH,
            kmod::delta(&before, &after, "lost"),
            kmod::delta(&before, &after, "reordered"),
            kmod::delta(&before, &after, "restarts"),
            seen as i64 - decoder.counters().packets as i64,
        );
    }
    let cs = clock.stats();
    match clock.model() {
        Some(m) => eprintln!(
            "clock: skew {:+.3} ppm (least squares {:+.3}), least squares sits {:.1} us above the envelope; \
             residual above envelope p50 {:.1} us, p99 {:.1} us, max {:.1} us; {} samples over {:.2} s; resets {}",
            m.skew_ppm(),
            m.least_squares_skew_ppm(),
            m.least_squares_bias_ns() / 1000.0,
            m.residual_p50_ns / 1000.0,
            m.residual_p99_ns / 1000.0,
            m.residual_max_ns / 1000.0,
            m.samples,
            m.span_us as f64 * 1e-6,
            cs.resets,
        ),
        None => eprintln!("clock: no estimate ({} samples, resets {})", cs.samples, cs.resets),
    }
    if outcome.stage_drops > 0 {
        eprintln!(
            "stage: queue drops {} (frames not stamped or delivered)",
            outcome.stage_drops
        );
    }
    for (consumer, dropped) in &outcome.consumers {
        eprintln!(
            "consumer {}: queue drops {dropped}; {}",
            consumer.name(),
            consumer.summary()
        );
    }
}
