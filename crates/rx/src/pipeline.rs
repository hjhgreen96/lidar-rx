//! 수신 루프: 출처 → 해석기 → 시계 추정 → 소비자들.
//!
//! ```text
//!  threaded (기본)                                                 inline
//!  ┌─ 수신 스레드 ───────────┐   ┌─ stage 스레드 ──────────────┐    ┌─ 한 스레드 ─────────────────────────┐
//!  │ recv → decoder.push    │   │ clock.stamp_frame          │    │ recv → push → stamp → 소비자 직접 호출 │
//!  │  └ 프레임 완성 ─► 채널 ──┼──►│  └ Arc ─► 소비자별 채널 ────┼─►  │   소비자가 느리면 그동안 recv가 멈춤   │
//!  └────────────────────────┘   └────────────────────────────┘    │   → 커널 소켓 버퍼가 차서 커널이 버림  │
//!   패킷마다 일정한 일만          창 전체를 다시 맞추는 일(1초마다)  └─────────────────────────────────────┘
//! ```
//!
//! threaded에서는 수신 스레드가 소켓을 계속 비우고, 밀리면 **완성된 프레임 단위로** 버린다 (어디서 버릴지를
//! 커널에 맡기지 않음). inline은 비교용 (실험 3).
//!
//! 시계 추정을 stage 스레드로 뺀 이유: 1초마다 창 전체(30 s ≈ 22,600점)를 다시 맞추는 일이 보드(Cortex-A9)에서
//! 10 ms 넘게 걸려, 수신 스레드에서 하면 그동안 도착한 패킷이 소켓 큐에서 기다렸다 (docs/modules/clock.md).
//! 수신 스레드(실시간 정책을 받는 스레드)에는 패킷마다 상한이 정해진 일만 남긴다.
//!
//! 다 쓴 프레임은 마지막으로 놓는 스레드(stage 또는 소비자)가 수신 스레드로 돌려보내고, 해석기가 그 배열에 다음
//! 프레임을 모은다 (`Recycle`, `SensorDecoder::recycle`). 수신 스레드가 프레임마다 큰 배열을 새로 받지 않게.
//!
//! 패킷마다 **큐 대기 시간**(커널이 받은 시각 → 이 루프가 꺼낸 시각)을 잰다 (실험 2). 실험 1을 위해 한 번 일부러
//! 쉬는 기능(`Stall`)이 있고, 실험 4를 위해 수신 스레드에만 CPU 고정·실시간 정책을 줄 수 있다 (`sched`).

use crate::consumer::{FrameConsumer, Recycle, Subscriber};
use crate::histogram::LatencyHistogram;
use crate::sched::{self, ThreadSetup};
use crate::status::{Status, Tick};
use clock::{ClockEstimator, ClockModel};
use lidar_core::{DecodeError, Frame, PacketSource, Received, SensorDecoder};
use std::sync::mpsc::{sync_channel, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Threaded { queue: usize },
    Inline,
}

/// 수신 루프를 어떻게 돌릴지.
pub struct RunOptions {
    pub mode: Mode,
    /// 이만큼 지나면 끝냄 (`None` = 출처가 끝날 때까지)
    pub duration: Option<Duration>,
    /// 상태 한 줄을 찍는 간격
    pub status_every: Duration,
    /// 실험 1: 한 번 일부러 쉬기
    pub stall: Option<Stall>,
    /// 실험 4: 수신 스레드의 CPU 고정·실시간 정책
    pub receive_thread: ThreadSetup,
    /// 출처가 파일이면 true: stage가 밀려도 수신 쪽이 기다린다 (버리지 않음 → 결과가 실행마다 같음).
    /// 실시간 소켓이면 false: 수신 스레드는 기다리지 않고, 밀리면 프레임을 버리고 센다.
    pub from_file: bool,
}

/// 실험 1: 패킷을 `after_packets`개 받은 뒤 한 번, `duration` 동안 소켓을 읽지 않는다.
///
/// ```text
///  ─ 받는 중 ─┬──────── 쉼 (duration) ────────┬─ 다시 받음 ─────────────────►
///            │ 도착한 패킷은 소켓 큐에 쌓임      │ 처음 꺼내는 패킷들 중 커널 수신 시각이 "다시 받은 순간"보다
///            │ 큐가 차면 커널이 버림            │ 앞선 것 = 쉬는 동안 큐에 들어 있던 패킷 = 큐 용량
/// ```
#[derive(Clone, Copy, Debug)]
pub struct Stall {
    pub after_packets: u64,
    pub duration: Duration,
}

#[derive(Clone, Copy, Debug)]
pub struct StallReport {
    pub stall: Stall,
    /// 다시 받기 시작할 때 큐에 들어 있던 패킷 수
    pub queued: u64,
    /// 그중 가장 오래 기다린 패킷의 대기 시간
    pub oldest_wait_us: u64,
    /// 쉬기 직전의 소켓 누적 드롭 수 (SO_RXQ_OVFL)
    pub kernel_drops_before: Option<u64>,
    /// 큐에 남아 있던 패킷을 다 꺼낸 직후의 누적 드롭 수. 넘친 뒤 처음 큐에 들어간 패킷에 이 값이 붙어 오므로
    /// 그 패킷을 받은 순간에 읽는다 (실행이 끝날 때 읽으면 그 뒤의 드롭까지 섞임).
    pub kernel_drops_after: Option<u64>,
}

impl StallReport {
    /// 쉬는 동안 커널이 버린 수. 커널 카운터는 u32라 넘어가며 돌 수 있어 u32 뺄셈으로 계산.
    pub fn kernel_drops_during(&self) -> Option<u64> {
        let (before, after) = (self.kernel_drops_before?, self.kernel_drops_after?);
        Some((after as u32).wrapping_sub(before as u32) as u64)
    }
}

enum StallState {
    Waiting(Stall),
    Draining {
        report: StallReport,
        resumed_ns: i64,
    },
    Done(StallReport),
}

/// 끝난 소비자와, 그 소비자 채널이 차서 버린 프레임 수
pub type ConsumerResults = Vec<(Box<dyn FrameConsumer>, u64)>;

/// 상태 줄에 필요한, stage 스레드가 가진 값들의 사본.
#[derive(Clone, Default)]
struct StageView {
    clock: Option<ClockModel>,
    consumer_drops: Vec<(&'static str, u64)>,
}

/// threaded: 시계 추정 + 소비자 전달을 맡는 스레드와, 그 앞의 bounded 채널.
struct Stage {
    tx: SyncSender<Frame>,
    view: Arc<Mutex<StageView>>,
    /// 마지막으로 읽은 사본. stage가 잠금을 쥐고 있으면 이것을 쓴다 (수신 스레드는 잠금을 기다리지 않음)
    last_view: StageView,
    handle: JoinHandle<(ClockEstimator, ConsumerResults)>,
    /// true면 채널이 차 있을 때 기다린다 (파일 입력)
    wait_when_full: bool,
    /// stage 채널이 차서 버린 프레임
    dropped: u64,
}

impl Stage {
    /// 소비자 스레드들과 stage 스레드를 만든다. 둘 다 이 스레드(아직 실시간 정책 전)의 정책을 물려받는다.
    fn spawn(
        mut clock: ClockEstimator,
        consumers: Vec<Box<dyn FrameConsumer>>,
        queue: usize,
        wait_when_full: bool,
        recycle: Recycle,
    ) -> std::io::Result<Self> {
        let mut subs = consumers
            .into_iter()
            .map(|c| Subscriber::spawn(c, queue, recycle.clone()))
            .collect::<Result<Vec<_>, _>>()?;
        // 첫 프레임 전에도 상태 줄에 소비자 이름이 보이게 미리 채움
        let initial = StageView {
            clock: None,
            consumer_drops: subs.iter().map(|s| (s.name(), 0)).collect(),
        };
        let view = Arc::new(Mutex::new(initial.clone()));
        let shared = Arc::clone(&view);
        let (tx, rx) = sync_channel::<Frame>(queue);
        let handle = std::thread::Builder::new()
            .name("stage".into())
            .spawn(move || {
                for mut frame in rx {
                    frame.settle_point_times(); // 해석기가 수신 스레드에서 미뤄 둔 점 시각 옮기기
                    clock.stamp_frame(&mut frame);
                    let frame = Arc::new(frame);
                    subs.iter_mut().for_each(|s| s.offer(&frame));
                    recycle.release(frame);
                    let mut v = shared.lock().unwrap();
                    v.clock = clock.model().copied();
                    v.consumer_drops = subs.iter().map(|s| (s.name(), s.dropped())).collect();
                }
                (clock, subs.into_iter().map(Subscriber::finish).collect())
            })?;
        Ok(Stage {
            tx,
            view,
            last_view: initial,
            handle,
            wait_when_full,
            dropped: 0,
        })
    }

    /// 실시간: 기다리지 않고 넣어 본다. 차 있으면 이 프레임을 버리고 세고, 돌려준다 (해석기가 배열을 다시 쓰게).
    /// 파일: 자리가 날 때까지 기다린다.
    fn offer(&mut self, frame: Frame) -> Option<Frame> {
        let rejected = if self.wait_when_full {
            self.tx.send(frame).err().map(|e| e.0)
        } else {
            match self.tx.try_send(frame) {
                Ok(()) => None,
                Err(TrySendError::Full(f)) | Err(TrySendError::Disconnected(f)) => Some(f),
            }
        };
        if rejected.is_some() {
            self.dropped += 1;
        }
        rejected
    }

    fn view(&mut self) -> &StageView {
        if let Ok(v) = self.view.try_lock() {
            self.last_view = v.clone();
        }
        &self.last_view
    }
}

enum Outputs {
    Threaded(Stage),
    Inline {
        clock: Box<ClockEstimator>,
        consumers: Vec<Box<dyn FrameConsumer>>,
    },
}

impl Outputs {
    /// 이 자리에서 다 쓴 프레임(inline, 또는 stage 채널이 차서 버린 것)은 돌려준다 → 해석기에 바로 돌려줌.
    fn emit(&mut self, mut frame: Frame) -> Option<Frame> {
        match self {
            Outputs::Threaded(stage) => stage.offer(frame),
            Outputs::Inline { clock, consumers } => {
                frame.settle_point_times();
                clock.stamp_frame(&mut frame);
                consumers.iter_mut().for_each(|c| c.consume(&frame));
                Some(frame)
            }
        }
    }

    /// 상태 줄용: 시계 모델, 채널별 버린 프레임 수
    fn status(&mut self) -> (Option<ClockModel>, Vec<(&'static str, u64)>) {
        match self {
            Outputs::Threaded(stage) => {
                let dropped = stage.dropped;
                let v = stage.view();
                let mut drops = vec![("stage", dropped)];
                drops.extend_from_slice(&v.consumer_drops);
                (v.clock, drops)
            }
            Outputs::Inline { clock, .. } => (clock.model().copied(), Vec::new()),
        }
    }

    /// 채널을 닫고(남은 프레임은 마저 처리) 스레드들이 끝나길 기다린다.
    fn finish(self) -> (ClockEstimator, ConsumerResults, u64) {
        match self {
            Outputs::Threaded(stage) => {
                drop(stage.tx);
                let (clock, consumers) = stage.handle.join().expect("stage thread panicked");
                (clock, consumers, stage.dropped)
            }
            Outputs::Inline { clock, consumers } => {
                (*clock, consumers.into_iter().map(|c| (c, 0)).collect(), 0)
            }
        }
    }
}

/// 수신 루프가 바깥(main)과 주고받는 순간들. systemd 알림, 종료 신호, 커널 모듈 카운터 읽기에 쓴다.
/// 루프는 이것이 무엇에 쓰이는지 모른다.
pub trait LoopHooks {
    /// 소비자 스레드·수신 스레드 설정까지 끝나 패킷을 받을 준비가 된 순간 (systemd: READY=1)
    fn ready(&mut self) {}
    /// 루프가 한 바퀴 돌 때마다. 패킷이 없어도 출처의 타임아웃마다 돈다 → 루프가 살아 있다는 신호 (watchdog)
    fn alive(&mut self) {}
    /// 상태 한 줄을 찍을 때 (systemd: STATUS=)
    fn status(&mut self, _line: &str) {}
    /// true면 루프를 끝낸다 (SIGTERM 등)
    fn should_stop(&self) -> bool {
        false
    }
    /// 수신을 멈춘 바로 그 순간 (소비자 스레드를 정리하기 전). 커널 모듈 카운터의 끝 값은 여기서 읽어야 대조가 맞다
    fn loop_end(&mut self) {}
}

/// 끝난 뒤의 결과 (요약 출력용).
pub struct Outcome {
    /// 넘겨받았던 시계 추정기 (threaded에서는 stage 스레드가 쓰다가 돌려준 것)
    pub clock: ClockEstimator,
    pub consumers: ConsumerResults,
    /// threaded: 수신 스레드 → stage 채널이 차서 버린 프레임
    pub stage_drops: u64,
    /// 커널 수신 시각 → 이 루프가 꺼낸 시각 (실시간 소켓일 때만 의미 있음)
    pub queue_wait: LatencyHistogram,
    /// 가장 오래 기다린 패킷을 꺼낸 순간 (CLOCK_REALTIME ns). 같은 순간 다른 측정(예: CPU가 멈춘 구간)과 맞춰 보려고
    pub queue_wait_max_at_ns: Option<i64>,
    pub stall: Option<StallReport>,
    /// 수신 스레드에 적용한 실행 조건을 커널에서 다시 읽은 것 (`RunOptions::receive_thread`를 줬을 때)
    pub receive_thread: Option<String>,
}

pub fn now_realtime_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as i64
}

/// 녹화 파일의 캡처 시각처럼 지금과 동떨어진 수신 시각은 대기 시간으로 치지 않음
const MAX_PLAUSIBLE_WAIT_NS: i64 = 10_000_000_000;
/// 수신 스레드로 돌아오는 프레임 채널. 수신 스레드가 패킷마다 비우므로 작아도 된다 (차면 돌려보내는 쪽에서 해제)
const RECYCLE_QUEUE: usize = 4;

pub fn run(
    source: &mut dyn PacketSource,
    decoder: &mut dyn SensorDecoder,
    clock: ClockEstimator,
    consumers: Vec<Box<dyn FrameConsumer>>,
    options: &RunOptions,
    hooks: &mut dyn LoopHooks,
) -> Result<Outcome, Box<dyn std::error::Error>> {
    let (recycle, returned) = Recycle::channel(RECYCLE_QUEUE);
    let mut outputs = match options.mode {
        Mode::Threaded { queue } => Outputs::Threaded(Stage::spawn(
            clock,
            consumers,
            queue,
            options.from_file,
            recycle,
        )?),
        Mode::Inline => Outputs::Inline {
            clock: Box::new(clock),
            consumers,
        },
    };
    // stage·소비자 스레드를 다 만든 뒤에: 새 스레드는 만든 스레드의 정책을 물려받으므로, 먼저 바꾸면 그 스레드들까지 바뀜
    let receive_thread = sched::apply(&options.receive_thread)?;
    if let Some(applied) = &receive_thread {
        eprintln!("lidar-rx: receive thread: {applied} (read back from the kernel)");
    }

    let started = Instant::now();
    let mut status = Status::new(options.status_every);
    let mut queue_wait = LatencyHistogram::default();
    let mut max_wait: Option<(i64, i64)> = None; // (기다린 ns, 꺼낸 시각 ns)
    let mut stall = options.stall.map(StallState::Waiting);
    let mut received: u64 = 0;
    hooks.ready();
    loop {
        if hooks.should_stop() || options.duration.is_some_and(|d| started.elapsed() >= d) {
            break;
        }
        match source.recv()? {
            Received::Packet(packet) => {
                received += 1;
                let now = now_realtime_ns();
                if let Some(at) = packet.received_at {
                    let wait = now - at.0;
                    if (0..MAX_PLAUSIBLE_WAIT_NS).contains(&wait) {
                        queue_wait.record(wait as u64 / 1000);
                        if max_wait.is_none_or(|(w, _)| wait > w) {
                            max_wait = Some((wait, now));
                        }
                    }
                    stall = stall.map(|s| s.observe(at.0, now, source.kernel_drops()));
                }
                // 돌아온 프레임을 해석기에 넘긴다 (다음 프레임을 새 배열이 아니라 이것에 모으게)
                while let Ok(frame) = returned.try_recv() {
                    decoder.recycle(frame);
                }
                match decoder.push(&packet) {
                    Ok(Some(frame)) => {
                        if let Some(done) = outputs.emit(frame) {
                            decoder.recycle(done);
                        }
                    }
                    Ok(None) => {}
                    Err(DecodeError::Malformed(_)) => {} // 해석기가 셈. 다음 패킷으로
                    Err(e @ DecodeError::Unsupported(_)) => return Err(e.into()),
                }
                source.recycle(packet); // 다음 패킷을 이 배열에 받게
                stall = stall.map(|s| s.maybe_pause(received, source.kernel_drops()));
            }
            Received::Timeout => status.quiet(),
            Received::End => break,
        }
        hooks.alive();
        // 줄을 찍을 때만 값을 모은다 (분위수 계산·목록 할당을 패킷마다 하지 않게)
        if status.due() {
            let (clock, consumer_drops) = outputs.status();
            let line = status.print(Tick {
                decoder: decoder.counters(),
                kernel_drops: source.kernel_drops(),
                clock,
                consumer_drops,
                queue_wait_p99_us: (queue_wait.count() > 0).then(|| queue_wait.quantile(0.99)),
            });
            hooks.status(&line);
        }
    }
    hooks.loop_end();

    if let Some(frame) = decoder.flush() {
        outputs.emit(frame);
    }
    let (clock, consumers, stage_drops) = outputs.finish();
    Ok(Outcome {
        clock,
        consumers,
        stage_drops,
        queue_wait,
        queue_wait_max_at_ns: max_wait.map(|(_, at)| at),
        stall: stall.and_then(StallState::report),
        receive_thread,
    })
}

impl StallState {
    /// 정해진 수를 받았으면 쉰다.
    fn maybe_pause(self, received: u64, kernel_drops: Option<u64>) -> Self {
        match self {
            StallState::Waiting(stall) if received >= stall.after_packets => {
                std::thread::sleep(stall.duration);
                StallState::Draining {
                    report: StallReport {
                        stall,
                        queued: 0,
                        oldest_wait_us: 0,
                        kernel_drops_before: kernel_drops,
                        kernel_drops_after: None,
                    },
                    resumed_ns: now_realtime_ns(),
                }
            }
            other => other,
        }
    }

    /// 쉰 뒤 꺼내는 패킷이 다시 받기 시작한 순간보다 먼저 커널에 도착했으면 큐에 들어 있던 것.
    /// 다 꺼낸 뒤 처음 오는 패킷(넘친 뒤 처음 큐에 들어간 패킷)에 붙은 누적 드롭 수를 "쉰 뒤" 값으로 기록.
    fn observe(self, received_at_ns: i64, now_ns: i64, kernel_drops: Option<u64>) -> Self {
        match self {
            StallState::Draining {
                mut report,
                resumed_ns,
            } => {
                if received_at_ns <= resumed_ns {
                    report.queued += 1;
                    report.oldest_wait_us = report
                        .oldest_wait_us
                        .max(((now_ns - received_at_ns) / 1000) as u64);
                    StallState::Draining { report, resumed_ns }
                } else {
                    report.kernel_drops_after = kernel_drops;
                    StallState::Done(report)
                }
            }
            other => other,
        }
    }

    fn report(self) -> Option<StallReport> {
        match self {
            StallState::Waiting(_) => None,
            StallState::Draining { report, .. } | StallState::Done(report) => Some(report),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::consumer::StatsConsumer;
    use lidar_core::{DecoderCounters, FrameStats, Packet, SensorTime};

    /// 빈 패킷 개를 1 ms 간격으로 낸다 (그동안 stage·소비자 스레드가 앞 프레임을 다 씀)
    struct Packets {
        left: u32,
    }

    impl PacketSource for Packets {
        fn recv(&mut self) -> std::io::Result<Received> {
            if self.left == 0 {
                return Ok(Received::End);
            }
            self.left -= 1;
            std::thread::sleep(Duration::from_millis(1));
            Ok(Received::Packet(Packet {
                payload: Vec::new(),
                received_at: None,
            }))
        }
    }

    /// 패킷마다 빈 프레임 하나를 내고, 돌려받은 프레임을 센다
    #[derive(Default)]
    struct Counting {
        frames: u64,
        recycled: u64,
    }

    impl SensorDecoder for Counting {
        fn push(&mut self, _: &Packet) -> Result<Option<Frame>, DecodeError> {
            self.frames += 1;
            Ok(Some(Frame {
                sensor_stamp: SensorTime(0),
                stamp: None,
                frame_id: "test".into(),
                points: Vec::new(),
                packet_times: Vec::new(),
                stats: FrameStats::default(),
                pending_time_shift: 0.0,
            }))
        }
        fn flush(&mut self) -> Option<Frame> {
            None
        }
        fn counters(&self) -> DecoderCounters {
            DecoderCounters::default()
        }
        fn recycle(&mut self, _: Frame) {
            self.recycled += 1;
        }
    }

    struct NoHooks;
    impl LoopHooks for NoHooks {}

    fn run_with(mode: Mode) -> Counting {
        let mut decoder = Counting::default();
        let consumers: Vec<Box<dyn FrameConsumer>> = vec![
            Box::new(StatsConsumer::default()),
            Box::new(StatsConsumer::default()),
        ];
        let options = RunOptions {
            mode,
            duration: None,
            status_every: Duration::from_secs(3600),
            stall: None,
            receive_thread: ThreadSetup::default(),
            from_file: true,
        };
        let clock = ClockEstimator::new(clock::Config::default());
        run(
            &mut Packets { left: 50 },
            &mut decoder,
            clock,
            consumers,
            &options,
            &mut NoHooks,
        )
        .unwrap();
        decoder
    }

    #[test]
    fn used_frames_come_back_to_the_decoder() {
        // inline: 소비자를 부른 직후 그 자리에서 돌려줌
        let d = run_with(Mode::Inline);
        assert_eq!((d.frames, d.recycled), (50, 50));
        // threaded: 소비자 둘 중 나중에 놓는 스레드가 돌려보내고, 수신 루프가 다음 패킷 전에 해석기에 넘김.
        // 마지막 패킷 뒤에 돌아온 것은 세지 못하고, 느린 CI에서는 스레드가 늦게 깨어 몰려 돌아오면 채널(4)이 차
        // 그 자리에서 해제되기도 하므로 전부는 아님 → 절반 넘게 돌아오는지만 본다 (보통 49 ~ 50)
        let d = run_with(Mode::Threaded { queue: 2 });
        assert_eq!(d.frames, 50);
        assert!(d.recycled > 25, "{} of 50 came back", d.recycled);
    }
}
