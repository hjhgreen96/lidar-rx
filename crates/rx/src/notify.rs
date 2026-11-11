//! systemd에 알리기 (sd_notify 프로토콜, 4단계). 라이브러리 없이 직접 한다 - 유닉스 데이터그램 하나를 보내는 것이 전부다.
//!
//! ```text
//!  systemd ── 환경 변수 ──► lidar-rx
//!    NOTIFY_SOCKET=/run/systemd/notify       알림을 보낼 곳 (유닉스 데이터그램 소켓, '@'로 시작하면 추상 이름)
//!    WATCHDOG_USEC=2000000                   WatchdogSec=2s → 이 안에 WATCHDOG=1을 못 보내면 systemd가 죽이고 재시작
//!    WATCHDOG_PID=<pid>                      이 프로세스에게 준 값인지 확인용
//!
//!  lidar-rx ── "KEY=VALUE\n…" 데이터그램 ──► systemd
//!    READY=1        소켓·버퍼·소비자 스레드·수신 스레드 설정까지 끝남 (Type=notify: 이걸 받아야 "시작됨")
//!    WATCHDOG=1     수신 루프가 돌고 있음. 간격의 절반마다
//!    STATUS=…       systemctl status에 보이는 한 줄 (상태 줄과 같은 내용)
//!    STOPPING=1     끝내는 중
//! ```
//!
//! systemd 밖에서 돌리면(`NOTIFY_SOCKET`이 없으면) 아무것도 하지 않는다.

use std::io;
use std::time::{Duration, Instant};

pub struct Notifier {
    #[cfg(target_os = "linux")]
    target: Option<sys::Target>,
    /// WATCHDOG=1을 보낼 간격 (systemd가 준 값의 절반)
    watchdog_every: Option<Duration>,
    last_ping: Option<Instant>,
    reported_error: bool,
}

impl Notifier {
    /// systemd가 넘겨준 환경 변수를 읽는다.
    pub fn from_env() -> io::Result<Self> {
        let usec = std::env::var("WATCHDOG_USEC").ok();
        let pid = std::env::var("WATCHDOG_PID").ok();
        Ok(Notifier {
            #[cfg(target_os = "linux")]
            target: match std::env::var_os("NOTIFY_SOCKET") {
                Some(path) => Some(sys::Target::new(&path)?),
                None => None,
            },
            watchdog_every: watchdog_every(usec.as_deref(), pid.as_deref(), std::process::id()),
            last_ping: None,
            reported_error: false,
        })
    }

    pub fn watchdog_every(&self) -> Option<Duration> {
        self.watchdog_every
    }

    /// 상태를 보낸다. 실패해도 수신은 계속한다 (처음 한 번만 알림).
    pub fn send(&mut self, state: &str) {
        #[cfg(target_os = "linux")]
        if let Some(target) = &self.target {
            if let Err(e) = target.send(state) {
                if !self.reported_error {
                    eprintln!("lidar-rx: notifying systemd failed: {e}");
                    self.reported_error = true;
                }
            }
        }
        #[cfg(not(target_os = "linux"))]
        let _ = state;
    }

    /// 수신 루프가 한 바퀴 돌 때마다 부른다. 간격이 지났으면 WATCHDOG=1.
    pub fn alive(&mut self) {
        let Some(every) = self.watchdog_every else {
            return;
        };
        if self.last_ping.is_none_or(|t| t.elapsed() >= every) {
            self.send("WATCHDOG=1");
            self.last_ping = Some(Instant::now());
        }
    }
}

/// WATCHDOG_USEC의 절반. WATCHDOG_PID가 있는데 이 프로세스가 아니면 (부모가 받은 값을 물려받은 것) 쓰지 않는다.
fn watchdog_every(usec: Option<&str>, pid: Option<&str>, my_pid: u32) -> Option<Duration> {
    if pid.is_some_and(|p| p.parse() != Ok(my_pid)) {
        return None;
    }
    let usec: u64 = usec?.parse().ok().filter(|&u| u > 0)?;
    Some(Duration::from_micros(usec / 2))
}

#[cfg(target_os = "linux")]
mod sys {
    use std::ffi::OsStr;
    use std::io;
    use std::os::linux::net::SocketAddrExt;
    use std::os::unix::ffi::OsStrExt;
    use std::os::unix::net::{SocketAddr, UnixDatagram};

    pub struct Target {
        socket: UnixDatagram,
        addr: SocketAddr,
    }

    impl Target {
        pub fn new(path: &OsStr) -> io::Result<Self> {
            Ok(Target {
                socket: UnixDatagram::unbound()?,
                addr: address(path)?,
            })
        }

        pub fn send(&self, state: &str) -> io::Result<()> {
            self.socket.send_to_addr(state.as_bytes(), &self.addr)?;
            Ok(())
        }
    }

    /// "/run/systemd/notify" → 파일 경로, "@name" → 추상 이름 (파일 시스템에 없는 소켓)
    pub fn address(path: &OsStr) -> io::Result<SocketAddr> {
        match path.as_bytes() {
            [b'@', name @ ..] => SocketAddr::from_abstract_name(name),
            _ => SocketAddr::from_pathname(path),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn watchdog_interval_is_half_of_what_systemd_gives() {
        let me = 4242;
        assert_eq!(
            watchdog_every(Some("2000000"), Some("4242"), me),
            Some(Duration::from_secs(1))
        );
        assert_eq!(
            watchdog_every(Some("2000000"), None, me),
            Some(Duration::from_secs(1))
        );
        // 다른 프로세스에게 준 값, 없음, 0, 숫자 아님 → watchdog 안 함
        assert_eq!(watchdog_every(Some("2000000"), Some("1"), me), None);
        assert_eq!(watchdog_every(None, None, me), None);
        assert_eq!(watchdog_every(Some("0"), None, me), None);
        assert_eq!(watchdog_every(Some("2s"), None, me), None);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn sends_one_datagram_per_state_to_the_notify_socket() {
        use std::os::unix::net::UnixDatagram;
        let dir = std::env::temp_dir().join(format!("lidar-rx-notify-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("notify");
        let _ = std::fs::remove_file(&path);
        let systemd = UnixDatagram::bind(&path).unwrap();

        let mut n = Notifier {
            target: Some(sys::Target::new(path.as_os_str()).unwrap()),
            watchdog_every: Some(Duration::from_secs(3600)),
            last_ping: None,
            reported_error: false,
        };
        n.send("READY=1\nSTATUS=receiving");
        n.alive();
        n.alive(); // 간격 안이라 보내지 않음

        let mut buf = [0u8; 256];
        let got = systemd.recv(&mut buf).unwrap();
        assert_eq!(&buf[..got], b"READY=1\nSTATUS=receiving");
        let got = systemd.recv(&mut buf).unwrap();
        assert_eq!(&buf[..got], b"WATCHDOG=1");
        systemd.set_nonblocking(true).unwrap();
        assert!(systemd.recv(&mut buf).is_err());
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn abstract_socket_names_start_with_at() {
        use std::os::linux::net::SocketAddrExt;
        let addr = sys::address(std::ffi::OsStr::new("@lidar-test")).unwrap();
        assert_eq!(addr.as_abstract_name(), Some(&b"lidar-test"[..]));
        let addr = sys::address(std::ffi::OsStr::new("/run/systemd/notify")).unwrap();
        assert_eq!(
            addr.as_pathname(),
            Some(std::path::Path::new("/run/systemd/notify"))
        );
    }
}
