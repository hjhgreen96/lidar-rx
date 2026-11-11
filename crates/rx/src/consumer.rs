//! 경계 3 - 소비자 (Observer 패턴).
//!
//! ```text
//!                         ┌─ bounded 채널 (N) ─► [스레드] StatsConsumer
//!  stage 스레드 ── Frame ─┤
//!   (발행, pipeline.rs)   └─ bounded 채널 (N) ─► [스레드] Ros2Publisher
//!
//!  발행하는 쪽은 누가 받는지 모른다. 소비자를 붙이거나 빼도 수신 코드는 그대로다.
//!  채널이 차면(그 소비자가 느리면) 그 소비자 몫의 프레임만 버리고 센다 → 느린 소비자가 수신을 막지 않는다.
//! ```
//!
//! 프레임은 `Arc`로 공유한다: 소비자가 여럿이어도 포인트 배열(약 0.6 MB)을 복사하지 않는다.
//! 마지막으로 놓는 스레드가 그 프레임을 수신 스레드로 돌려보낸다 (`Recycle`).

use crate::histogram::LatencyHistogram;
use crate::pipeline::now_realtime_ns;
use lidar_core::Frame;
use std::sync::mpsc::{sync_channel, Receiver, SyncSender, TrySendError};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

pub trait FrameConsumer: Send {
    fn name(&self) -> &'static str;
    fn consume(&mut self, frame: &Frame);
    /// 끝날 때 한 줄 요약
    fn summary(&self) -> String {
        String::new()
    }
}

/// 다 쓴 프레임을 수신 스레드로 돌려보내는 길. 수신 스레드는 그 배열을 다음 프레임에 다시 쓴다
/// (`SensorDecoder::recycle`) → 프레임마다 점 배열을 새로 할당하지 않음 (vlp16 `assemble`의 설명).
#[derive(Clone)]
pub struct Recycle(SyncSender<Frame>);

impl Recycle {
    pub fn channel(queue: usize) -> (Self, Receiver<Frame>) {
        let (tx, rx) = sync_channel(queue);
        (Recycle(tx), rx)
    }

    /// 이 스레드가 프레임을 놓는다. 마지막으로 놓는 쪽이면 돌려보낸다 (자리가 없으면 여기서 해제 - 수신 스레드가 아니라).
    pub fn release(&self, frame: Arc<Frame>) {
        if let Some(frame) = Arc::into_inner(frame) {
            let _ = self.0.try_send(frame);
        }
    }
}

/// 소비자 하나를 자기 스레드에서 돌리고, 앞에 bounded 채널을 둔 것.
pub struct Subscriber {
    name: &'static str,
    tx: SyncSender<Arc<Frame>>,
    handle: JoinHandle<Box<dyn FrameConsumer>>,
    dropped: u64,
}

impl Subscriber {
    pub fn spawn(
        mut consumer: Box<dyn FrameConsumer>,
        queue: usize,
        recycle: Recycle,
    ) -> std::io::Result<Self> {
        let name = consumer.name();
        let (tx, rx) = sync_channel::<Arc<Frame>>(queue);
        let handle = std::thread::Builder::new()
            .name(format!("consumer-{name}"))
            .spawn(move || {
                for frame in rx {
                    consumer.consume(&frame);
                    recycle.release(frame);
                }
                consumer
            })?;
        Ok(Subscriber {
            name,
            tx,
            handle,
            dropped: 0,
        })
    }

    pub fn name(&self) -> &'static str {
        self.name
    }

    /// 기다리지 않고 넣어 본다. 차 있으면 이 프레임은 이 소비자에게 가지 않는다.
    pub fn offer(&mut self, frame: &Arc<Frame>) {
        match self.tx.try_send(Arc::clone(frame)) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) => self.dropped += 1,
        }
    }

    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    /// 채널을 닫고(남은 프레임은 소비자가 마저 처리) 스레드가 끝나길 기다린다.
    pub fn finish(self) -> (Box<dyn FrameConsumer>, u64) {
        drop(self.tx);
        let consumer = self.handle.join().expect("consumer thread panicked");
        (consumer, self.dropped)
    }
}

/// 프레임 품질과 지연을 센다.
#[derive(Default)]
pub struct StatsConsumer {
    frames: u64,
    full_frames: u64,
    full_frame_points: u64,
    /// 받은 프레임들의 `missing_packets` 합. 자르는 각도에 걸쳐 빠진 패킷은 앞뒤 두 프레임에 다 들어 있어
    /// 해석기의 누적 손실보다 클 수 있다
    missing_packets: u64,
    /// 프레임 대표 시각(호스트 시계로 변환한 값) → 이 소비자가 받은 순간, µs
    age: LatencyHistogram,
}

impl FrameConsumer for StatsConsumer {
    fn name(&self) -> &'static str {
        "stats"
    }

    fn consume(&mut self, frame: &Frame) {
        self.frames += 1;
        self.missing_packets += frame.stats.missing_packets as u64;
        if !frame.stats.is_full_turn() {
            // 시작 직후의 조각, 끝날 때 flush한 조각: 점 수와 지연 통계에서 뺀다
            return;
        }
        self.full_frames += 1;
        self.full_frame_points += frame.points.len() as u64;
        if let Some(stamp) = frame.stamp {
            let age_ns = now_realtime_ns() - stamp.0;
            // 녹화 파일을 읽을 때는 대표 시각이 과거(캡처한 날)라 의미가 없으므로 10 s 넘는 값은 뺌
            if (0..10_000_000_000).contains(&age_ns) {
                self.age.record(age_ns as u64 / 1000);
            }
        }
    }

    fn summary(&self) -> String {
        let mut s = format!(
            "{} frames ({} full turns, {:.0} points each), missing packets {}",
            self.frames,
            self.full_frames,
            self.full_frame_points as f64 / self.full_frames.max(1) as f64,
            self.missing_packets,
        );
        if self.age.count() > 0 {
            s += &format!(
                "; frame age at consumer p50 {} us, p99 {} us, max {} us",
                self.age.quantile(0.5),
                self.age.quantile(0.99),
                self.age.max()
            );
        }
        s
    }
}

/// 실험 3: 프레임마다 정해진 시간 동안 CPU를 쓰는 처리(뒤따르는 인식 알고리즘 같은 것)를 모사한다.
///
/// 받은 프레임 중 온전한 한 바퀴(자르는 각도에서 자르는 각도까지, 빠진 패킷 없음)의 수와, 처리를 시작할 때 그 프레임이
/// 얼마나 오래된 데이터였는지를 센다. inline이면 이 처리 동안 수신이 멈추고, threaded면 이 소비자의 채널만 찬다.
pub struct WorkConsumer {
    work: Duration,
    frames: u64,
    complete: u64,
    /// 프레임 대표 시각(호스트 시계) → 처리를 시작한 순간, µs
    age: LatencyHistogram,
}

impl WorkConsumer {
    pub fn new(work: Duration) -> Self {
        WorkConsumer {
            work,
            frames: 0,
            complete: 0,
            age: LatencyHistogram::default(),
        }
    }
}

impl FrameConsumer for WorkConsumer {
    fn name(&self) -> &'static str {
        "work"
    }

    fn consume(&mut self, frame: &Frame) {
        self.frames += 1;
        // 빠진 패킷만 보면 안 된다: 반 바퀴 넘게 빠져 그 자리에서 끊은 프레임은 빠진 패킷 0이지만 한 바퀴가 아니다
        if frame.stats.is_complete() {
            self.complete += 1;
        }
        // 시계 추정 전(처음 2초)에는 stamp가 없으므로 마지막 패킷의 커널 수신 시각으로
        let stamp = frame
            .stamp
            .or_else(|| frame.packet_times.last().and_then(|p| p.received_at));
        if let Some(stamp) = stamp {
            let age_ns = now_realtime_ns() - stamp.0;
            if (0..10_000_000_000).contains(&age_ns) {
                self.age.record(age_ns as u64 / 1000);
            }
        }
        // sleep이 아니라 바쁜 대기: 실제 처리처럼 그동안 CPU를 쓴다
        let end = Instant::now() + self.work;
        while Instant::now() < end {
            std::hint::spin_loop();
        }
    }

    fn summary(&self) -> String {
        let mut s = format!(
            "{} frames processed ({} complete: full turn, no missing packets)",
            self.frames, self.complete
        );
        if self.age.count() > 0 {
            s += &format!(
                "; data age when processing started p50 {} us, p99 {} us, max {} us",
                self.age.quantile(0.5),
                self.age.quantile(0.99),
                self.age.max()
            );
        }
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use lidar_core::{FrameStats, SensorTime};

    fn frame(missing: u32, starts_at_cut: bool) -> Frame {
        Frame {
            sensor_stamp: SensorTime(0),
            stamp: None,
            frame_id: "velodyne".into(),
            points: Vec::new(),
            packet_times: Vec::new(),
            stats: FrameStats {
                missing_packets: missing,
                starts_at_cut,
                ends_at_cut: true,
            },
            pending_time_shift: 0.0,
        }
    }

    #[test]
    fn only_the_last_holder_sends_the_frame_back() {
        let (recycle, returned) = Recycle::channel(4);
        let stage = Arc::new(frame(0, true));
        let consumer = Arc::clone(&stage);
        recycle.release(stage); // 소비자가 아직 쥐고 있음
        assert!(returned.try_recv().is_err());
        recycle.release(consumer);
        assert!(returned.try_recv().is_ok());
    }

    #[test]
    fn work_consumer_spends_the_time_and_counts_complete_turns() {
        let mut c = WorkConsumer::new(Duration::from_millis(5));
        let start = Instant::now();
        c.consume(&frame(0, true));
        c.consume(&frame(3, true)); // 구멍 남
        c.consume(&frame(0, false)); // 빠진 패킷은 없지만 손실로 끊은 뒤의 조각 → 한 바퀴가 아님
        assert!(start.elapsed() >= Duration::from_millis(15));
        assert_eq!((c.frames, c.complete), (3, 1));
        assert_eq!(
            c.summary(),
            "3 frames processed (1 complete: full turn, no missing packets)"
        );
    }
}
