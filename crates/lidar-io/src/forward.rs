//! 받은 패킷을 그대로 돌려주면서, 같은 바이트를 다른 주소로도 보내는 출처 (Decorator).
//!
//! ```text
//!  UdpSource ──► ForwardSource::recv() ──► 수신 루프 (받은 것 그대로)
//!                      └─ send_to(같은 페이로드) ──► 다른 컴퓨터 (예: 보드 → PC의 lidar-rx --ros2 → RViz)
//! ```
//!
//! 보드(32비트 ARM)에는 ROS 2가 없어서, 보드가 실제 NIC로 받은 패킷을 PC로 넘겨 PC에서 그린다.
//! 수신 루프는 이것이 감싼 출처인지 모른다 (`PacketSource` 그대로).
//! 보내기는 막히지 않게(non-blocking) 하고 실패는 세기만 한다: 보여 주기용 경로가 끊겨도 수신은 멈추지 않는다.
//!
//! 진단 (`with_tx_timestamps`, lidar-rx `--forward-tx-log`): 넘긴 패킷마다 커널이 NIC 드라이버에 넘긴 시각을 받는다.
//!
//! ```text
//!  setsockopt(SO_TIMESTAMPING, TX_SOFTWARE | SOFTWARE | OPT_ID | OPT_TSONLY)
//!  send_to ─► … ─► 드라이버 xmit의 skb_tx_timestamp() ─► 그 시각이 이 소켓의 오류 큐로 (OPT_ID: 몇 번째 보낸 것인지)
//!  recvmsg(MSG_ERRQUEUE | MSG_DONTWAIT) ─► (번호, 시각)  ← recv()를 부를 때마다 비움
//! ```
//!
//! 받은 패킷의 커널 수신 시각과 같은 보드 시계라, 패킷마다 "커널 도착 → 드라이버에 넘김"이 바로 나온다.
//! 패킷마다 기록을 쌓고 수신 스레드에서 recvmsg를 한 번 더 부르므로 길이가 정해진 진단 실행에만 쓴다.

use lidar_core::{Packet, PacketSource, Received};
use std::cell::{Cell, RefCell};
use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::rc::Rc;

/// 넘긴 패킷마다 (커널 수신 시각 ns, 드라이버에 넘긴 시각 ns - 아직 안 왔으면 `None`), 보낸 순서대로
pub type TxLog = Vec<(i64, Option<i64>)>;

/// 보낸 수와 못 보낸 수. 출처를 `Box<dyn PacketSource>`로 넘긴 뒤에도 요약에서 읽을 수 있게 따로 둔다.
#[derive(Debug, Default)]
pub struct ForwardCounts {
    pub forwarded: Cell<u64>,
    pub failed: Cell<u64>,
    /// `with_tx_timestamps`일 때만
    pub tx_log: RefCell<Option<TxLog>>,
}

pub struct ForwardSource {
    inner: Box<dyn PacketSource>,
    socket: UdpSocket,
    to: SocketAddr,
    counts: Rc<ForwardCounts>,
}

impl ForwardSource {
    pub fn new(inner: Box<dyn PacketSource>, to: SocketAddr) -> io::Result<Self> {
        let any: SocketAddr = if to.is_ipv4() { "0.0.0.0:0" } else { "[::]:0" }
            .parse()
            .unwrap();
        let socket = UdpSocket::bind(any)?;
        socket.set_nonblocking(true)?;
        Ok(Self {
            inner,
            socket,
            to,
            counts: Rc::default(),
        })
    }

    pub fn counts(&self) -> Rc<ForwardCounts> {
        Rc::clone(&self.counts)
    }

    /// 넘긴 패킷마다 드라이버에 넘긴 시각을 기록한다 (모듈 설명의 진단)
    pub fn with_tx_timestamps(self) -> io::Result<Self> {
        sys::enable_tx_timestamps(&self.socket)?;
        *self.counts.tx_log.borrow_mut() = Some(Vec::new());
        Ok(self)
    }

    /// 오류 큐에 온 송신 시각을 모두 꺼내 기록에 채운다
    fn collect_tx_timestamps(&self) {
        let mut log = self.counts.tx_log.borrow_mut();
        let Some(log) = log.as_mut() else { return };
        while let Some(stamp) = sys::read_tx_timestamp(&self.socket) {
            if let Some((id, ns)) = stamp {
                if let Some(entry) = log.get_mut(id as usize) {
                    entry.1 = Some(ns);
                }
            }
        }
    }
}

impl PacketSource for ForwardSource {
    fn recv(&mut self) -> io::Result<Received> {
        self.collect_tx_timestamps(); // 패킷이 없을 때(타임아웃)도 불리므로 마지막 것까지 모임
        let received = self.inner.recv()?;
        if let Received::Packet(packet) = &received {
            let counter = match self.socket.send_to(&packet.payload, self.to) {
                Ok(_) => {
                    // OPT_ID 번호는 성공한 보내기마다 0부터 하나씩 → 기록의 위치와 같다
                    if let Some(log) = self.counts.tx_log.borrow_mut().as_mut() {
                        log.push((packet.received_at.map_or(0, |t| t.0), None));
                    }
                    &self.counts.forwarded
                }
                Err(_) => &self.counts.failed, // 보내기 버퍼가 참(WouldBlock) 등: 수신은 계속
            };
            counter.set(counter.get() + 1);
            self.collect_tx_timestamps();
        }
        Ok(received)
    }

    fn kernel_drops(&self) -> Option<u64> {
        self.inner.kernel_drops()
    }

    fn recycle(&mut self, packet: Packet) {
        self.inner.recycle(packet);
    }
}

#[cfg(target_os = "linux")]
mod sys {
    use std::io;
    use std::mem;
    use std::net::UdpSocket;
    use std::os::fd::AsRawFd;
    use std::ptr::read_unaligned;

    /// 32비트에서 64비트 timespec으로 오는 형식 (커널 5.1+). 옛 형식은 `libc::SO_TIMESTAMPING`(37)
    const SO_TIMESTAMPING_NEW: libc::c_int = 65;

    pub fn enable_tx_timestamps(socket: &UdpSocket) -> io::Result<()> {
        let flags = (libc::SOF_TIMESTAMPING_TX_SOFTWARE
            | libc::SOF_TIMESTAMPING_SOFTWARE
            | libc::SOF_TIMESTAMPING_OPT_ID
            | libc::SOF_TIMESTAMPING_OPT_TSONLY) as libc::c_int;
        // SAFETY: c_int 하나의 주소와 크기를 넘긴다.
        let rc = unsafe {
            libc::setsockopt(
                socket.as_raw_fd(),
                libc::SOL_SOCKET,
                libc::SO_TIMESTAMPING,
                (&flags as *const libc::c_int).cast(),
                mem::size_of::<libc::c_int>() as libc::socklen_t,
            )
        };
        if rc != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }

    /// 오류 큐에서 하나. 큐가 비었으면 `None`, 시각이나 번호가 없는 메시지면 `Some(None)`.
    pub fn read_tx_timestamp(socket: &UdpSocket) -> Option<Option<(u32, i64)>> {
        let mut control = [0u64; 16];
        // SAFETY: msghdr는 모든 필드가 0이어도 유효하다. OPT_TSONLY라 패킷 내용이 없으므로 버퍼(iov)도 없다.
        let mut msg: libc::msghdr = unsafe { mem::zeroed() };
        msg.msg_control = control.as_mut_ptr().cast();
        msg.msg_controllen = mem::size_of_val(&control) as _;
        // SAFETY: control은 호출 동안 살아 있고 길이를 정확히 알린다.
        let n = unsafe {
            libc::recvmsg(
                socket.as_raw_fd(),
                &mut msg,
                libc::MSG_ERRQUEUE | libc::MSG_DONTWAIT,
            )
        };
        if n < 0 {
            return None;
        }
        let (mut id, mut ns) = (None, None);
        // SAFETY: udp.rs의 recv_with_meta와 같음: 커널이 채운 control 안에서만 움직이고, 길이를 확인한 뒤 읽는다.
        unsafe {
            let mut c = libc::CMSG_FIRSTHDR(&msg);
            while !c.is_null() {
                let header = &*c;
                let data = libc::CMSG_DATA(c);
                let Some(len) = (header.cmsg_len as usize).checked_sub(libc::CMSG_LEN(0) as usize)
                else {
                    break;
                };
                let timestamping = header.cmsg_level == libc::SOL_SOCKET
                    && (header.cmsg_type == libc::SO_TIMESTAMPING
                        || header.cmsg_type == SO_TIMESTAMPING_NEW);
                // scm_timestamping: timespec 세 개 중 첫째가 소프트웨어 시각. 32비트 옛 형식은 (i32, i32)
                if timestamping && len >= 48 {
                    ns = Some(
                        read_unaligned(data.cast::<i64>()) * 1_000_000_000
                            + read_unaligned(data.add(8).cast::<i64>()),
                    );
                } else if timestamping && len >= 24 {
                    ns = Some(
                        read_unaligned(data.cast::<i32>()) as i64 * 1_000_000_000
                            + read_unaligned(data.add(4).cast::<i32>()) as i64,
                    );
                }
                // sock_extended_err: ee_origin(4번째 바이트), ee_data(12번째부터) = OPT_ID 번호
                let error = (header.cmsg_level == libc::SOL_IP
                    && header.cmsg_type == libc::IP_RECVERR)
                    || (header.cmsg_level == libc::SOL_IPV6
                        && header.cmsg_type == libc::IPV6_RECVERR);
                if error && len >= 16 && *data.add(4) == libc::SO_EE_ORIGIN_TIMESTAMPING {
                    id = Some(read_unaligned(data.add(12).cast::<u32>()));
                }
                c = libc::CMSG_NXTHDR(&msg, c);
            }
        }
        Some(id.zip(ns))
    }
}

#[cfg(not(target_os = "linux"))]
mod sys {
    use std::io;
    use std::net::UdpSocket;

    pub fn enable_tx_timestamps(_: &UdpSocket) -> io::Result<()> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "transmit timestamps are implemented for Linux only",
        ))
    }
    pub fn read_tx_timestamp(_: &UdpSocket) -> Option<Option<(u32, i64)>> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use lidar_core::{HostTime, Packet};
    use std::time::Duration;

    /// 정해 둔 결과를 차례로 돌려주는 출처
    struct Script(Vec<Received>);

    impl PacketSource for Script {
        fn recv(&mut self) -> io::Result<Received> {
            Ok(if self.0.is_empty() {
                Received::End
            } else {
                self.0.remove(0)
            })
        }
        fn kernel_drops(&self) -> Option<u64> {
            Some(7)
        }
    }

    #[test]
    fn passes_packets_through_and_sends_the_same_bytes() {
        let rx = UdpSocket::bind("127.0.0.1:0").unwrap();
        rx.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
        let packet = Packet {
            payload: vec![0xFF, 0xEE, 1, 2, 3],
            received_at: Some(HostTime(42)),
        };
        let script = Script(vec![Received::Packet(packet.clone()), Received::Timeout]);
        let mut source = ForwardSource::new(Box::new(script), rx.local_addr().unwrap()).unwrap();
        let counts = source.counts();

        assert_eq!(source.recv().unwrap(), Received::Packet(packet)); // 수신 루프에는 그대로
        let mut buf = [0u8; 16];
        let n = rx.recv(&mut buf).unwrap();
        assert_eq!(&buf[..n], &[0xFF, 0xEE, 1, 2, 3]); // 같은 바이트가 나감
        assert_eq!(source.recv().unwrap(), Received::Timeout); // 패킷이 아니면 보내지 않음
        assert_eq!(source.recv().unwrap(), Received::End);
        assert_eq!((counts.forwarded.get(), counts.failed.get()), (1, 0));
        assert_eq!(source.kernel_drops(), Some(7)); // 감싼 출처의 값
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn records_when_each_forwarded_packet_was_handed_to_the_driver() {
        // loopback 드라이버도 xmit에서 skb_tx_timestamp를 부른다
        let rx = UdpSocket::bind("127.0.0.1:0").unwrap();
        let packets: Vec<Received> = (0..3)
            .map(|i| {
                Received::Packet(Packet {
                    payload: vec![i; 8],
                    received_at: Some(HostTime(1_000 + i as i64)),
                })
            })
            .chain([Received::Timeout])
            .collect();
        let mut source = ForwardSource::new(Box::new(Script(packets)), rx.local_addr().unwrap())
            .unwrap()
            .with_tx_timestamps()
            .unwrap();
        let counts = source.counts();
        let now_ns = || {
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as i64
        };
        let before = now_ns();
        for _ in 0..4 {
            source.recv().unwrap(); // 마지막 Timeout 때 남은 시각까지 모임
        }
        let after = now_ns();

        let log = counts.tx_log.borrow();
        let log = log.as_ref().unwrap();
        assert_eq!(log.len(), 3);
        for (i, &(received, sent)) in log.iter().enumerate() {
            assert_eq!(received, 1_000 + i as i64); // 넘긴 패킷의 수신 시각, 보낸 순서대로
            let sent = sent.expect("transmit timestamp");
            assert!(
                (before..=after).contains(&sent),
                "{sent} not in {before}..={after}"
            );
        }
    }
}
