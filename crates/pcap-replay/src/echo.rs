//! 보드가 되돌려 보낸 패킷으로 왕복 시간 재기 (`--echo`).
//!
//! ```text
//!  송신 루프: 보내기 직전에 (내용 해시 → 보낸 시각)을 표에 적음 ──send_to──► 보드 lidar-rx
//!                                                                          │ --forward (받은 그대로)
//!  Echo 스레드: recv ◄─────────────────────────────────────────────────────┘
//!              해시로 표에서 찾아 왕복 = 지금 − 보낸 시각
//!              [--echo-relay: 받은 것을 그대로 다시 보냄 → WSL의 lidar-rx --ros2 → RViz]
//! ```
//!
//! 보낸 시각과 받은 시각을 이 PC의 한 시계(`Instant`)로 재므로 두 기기의 시계를 맞출 필요가 없다.
//! 보드는 패킷을 바꾸지 않고 넘기므로 내용이 곧 이름이다 (반복마다 센서 타임스탬프를 다시 쓰므로 반복이 달라도 겹치지 않음).
//! 패킷마다 결과를 쌓으므로 녹화처럼 길이가 정해진 실행에만 쓴다 (`--loops 0`과 같이 못 씀).

use std::collections::HashMap;
use std::hash::{DefaultHasher, Hash, Hasher};
use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// 돌아온 패킷 하나
pub struct Echoed {
    pub sent_at: Instant,
    pub round_trip: Duration,
}

pub struct EchoReport {
    /// 돌아온 순서대로
    pub echoed: Vec<Echoed>,
    /// 받았지만 보낸 표에 없는 것 (다른 곳에서 온 패킷, 두 번 온 패킷)
    pub unknown: u64,
}

pub struct Echo {
    addr: SocketAddr,
    sent: Arc<Mutex<HashMap<u64, Instant>>>,
    stop: Arc<AtomicBool>,
    handle: JoinHandle<io::Result<EchoReport>>,
}

impl Echo {
    pub fn start(listen: SocketAddr, relay: Option<SocketAddr>) -> io::Result<Self> {
        let socket = UdpSocket::bind(listen)?;
        socket.set_read_timeout(Some(Duration::from_millis(100)))?; // 끝낼 때인지 보려고 가끔 깸
        let addr = socket.local_addr()?;
        let sent = Arc::new(Mutex::new(HashMap::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let handle = {
            let (sent, stop) = (Arc::clone(&sent), Arc::clone(&stop));
            std::thread::spawn(move || receive(&socket, relay, &sent, &stop))
        };
        Ok(Echo {
            addr,
            sent,
            stop,
            handle,
        })
    }

    pub fn local_addr(&self) -> SocketAddr {
        self.addr
    }

    /// 보내기 직전에 부른다 (돌아온 패킷을 찾을 때 보낸 기록이 이미 있도록)
    pub fn note_sent(&self, payload: &[u8]) {
        self.sent
            .lock()
            .unwrap()
            .insert(key(payload), Instant::now());
    }

    /// `linger` 동안 늦게 오는 것을 더 기다린 뒤 멈추고 결과를 돌려준다
    pub fn finish(self, linger: Duration) -> io::Result<EchoReport> {
        std::thread::sleep(linger);
        self.stop.store(true, Ordering::Relaxed);
        self.handle.join().expect("echo thread panicked")
    }
}

fn key(payload: &[u8]) -> u64 {
    let mut h = DefaultHasher::new();
    payload.hash(&mut h);
    h.finish()
}

fn receive(
    socket: &UdpSocket,
    relay: Option<SocketAddr>,
    sent: &Mutex<HashMap<u64, Instant>>,
    stop: &AtomicBool,
) -> io::Result<EchoReport> {
    let mut report = EchoReport {
        echoed: Vec::new(),
        unknown: 0,
    };
    let mut buf = [0u8; 2048];
    while !stop.load(Ordering::Relaxed) {
        let n = match socket.recv(&mut buf) {
            Ok(n) => n,
            // TimedOut: Windows의 읽기 시한. ConnectionReset: Windows에서는 이 소켓으로 보낸 곳(--echo-relay)이
            // 닫혀 있으면 돌아온 ICMP가 다음 recv의 오류로 나옴 → 재기와 상관없으니 넘어감
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock
                        | io::ErrorKind::TimedOut
                        | io::ErrorKind::ConnectionReset
                ) =>
            {
                continue
            }
            Err(e) => return Err(e),
        };
        let now = Instant::now();
        match sent.lock().unwrap().remove(&key(&buf[..n])) {
            Some(sent_at) => report.echoed.push(Echoed {
                sent_at,
                round_trip: now - sent_at,
            }),
            None => report.unknown += 1,
        }
        if let Some(to) = relay {
            let _ = socket.send_to(&buf[..n], to); // RViz 쪽이 없어도 재기는 계속
        }
    }
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_returned_packets_and_relays_them() {
        // 보드 대신: 받은 것을 그대로 Echo로 되돌려 보내고, 끝에 모르는 패킷 하나를 더 보냄
        let board = UdpSocket::bind("127.0.0.1:0").unwrap();
        let viewer = UdpSocket::bind("127.0.0.1:0").unwrap();
        viewer
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        let echo = Echo::start(
            "127.0.0.1:0".parse().unwrap(),
            Some(viewer.local_addr().unwrap()),
        )
        .unwrap();
        let echo_addr = echo.local_addr();
        let board_addr = board.local_addr().unwrap();
        let forwarder = std::thread::spawn(move || {
            let mut buf = [0u8; 64];
            for _ in 0..3 {
                let n = board.recv(&mut buf).unwrap();
                board.send_to(&buf[..n], echo_addr).unwrap();
            }
            board.send_to(b"not sent by us", echo_addr).unwrap();
        });

        let sender = UdpSocket::bind("127.0.0.1:0").unwrap();
        for payload in [&b"one"[..], b"two", b"three"] {
            echo.note_sent(payload);
            sender.send_to(payload, board_addr).unwrap();
        }
        forwarder.join().unwrap();
        let mut relayed = 0;
        let mut buf = [0u8; 64];
        while relayed < 4 && viewer.recv(&mut buf).is_ok() {
            relayed += 1;
        }
        let report = echo.finish(Duration::from_millis(50)).unwrap();

        assert_eq!(report.echoed.len(), 3);
        assert_eq!(report.unknown, 1);
        assert!(report
            .echoed
            .iter()
            .all(|e| e.round_trip < Duration::from_secs(1)));
        assert_eq!(
            relayed, 4,
            "every packet goes on to the viewer, known or not"
        );
    }
}
