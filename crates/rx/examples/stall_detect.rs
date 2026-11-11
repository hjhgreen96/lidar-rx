//! 가상 CPU가 리눅스 밖의 이유로 멈추는지 재는 도구 (실험 4 재검). 커널의 hwlat 트레이서와 같은 생각.
//!
//! ```text
//!   cargo build --release -p rx --example stall_detect
//!   sudo target/release/examples/stall_detect --cpu 2 --seconds 60 [--prio 50] [--duty 0.5] [--top 10]
//!                                             --prio 0: 일반 정책(SCHED_OTHER)으로 - 그 CPU의 "부하" 역할을 겸할 때
//! ```
//!
//! 지정한 CPU에 고정한 SCHED_FIFO 스레드가 시계를 쉬지 않고 읽으며, 연달아 읽은 두 값 사이가 벌어진 구간(gap)을 센다.
//!
//! ```text
//!  t₀ t₁ t₂ t₃ …………………………… t₄ t₅ …        읽기 한 번 ≈ 수십 ns
//!             └── gap = t₄ − t₃ ──┘          이 동안 이 스레드는 돌지 못했다
//! ```
//!
//! FIFO 스레드는 그 CPU의 일반 작업(부하 프로세스 포함)에게 밀리지 않는다. 그래서 gap의 원인은 둘뿐이다:
//! 인터럽트·softirq 처리(보통 수 µs), 그리고 **가상 CPU 자체가 멈춘 시간**(하이퍼바이저가 다른 일을 함 - 리눅스 안에서는
//! 아무 흔적이 없다). ms 단위 gap은 사실상 뒤쪽이다.
//!
//! 1초마다 `duty` 비율만큼만 돈다: 실시간 작업은 1초에 0.95초까지만 CPU를 쓸 수 있고(`sched_rt_runtime_us`),
//! 이를 넘기면 커널이 강제로 쉬게 해 그것이 gap으로 잡히기 때문이다. 쉬는 동안의 시간은 재지 않는다.

#[cfg(target_os = "linux")]
fn main() {
    if let Err(e) = linux::run() {
        eprintln!("stall_detect: {e}");
        std::process::exit(1);
    }
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("stall_detect: Linux only");
    std::process::exit(1);
}

#[cfg(target_os = "linux")]
mod linux {
    use std::io;
    use std::mem::{size_of, zeroed};
    use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

    /// 센 gap 크기 구간 (µs 이상)
    const THRESHOLDS_US: [u64; 5] = [10, 100, 500, 1_000, 5_000];

    struct Args {
        cpu: usize,
        seconds: u64,
        prio: i32,
        duty: f64,
        /// 큰 순서로 몇 개를 시각과 함께 보일지
        top: usize,
    }

    fn parse() -> Result<Args, String> {
        let mut a = Args {
            cpu: 0,
            seconds: 30,
            prio: 50,
            duty: 0.5,
            top: 10,
        };
        let mut it = std::env::args().skip(1);
        while let Some(arg) = it.next() {
            let v = it.next().ok_or(format!("{arg} needs a value"))?;
            let bad = |e: &dyn std::fmt::Display| format!("{arg}: {e}");
            match arg.as_str() {
                "--cpu" => a.cpu = v.parse().map_err(|e| bad(&e))?,
                "--seconds" => a.seconds = v.parse().map_err(|e| bad(&e))?,
                "--prio" => a.prio = v.parse().map_err(|e| bad(&e))?,
                "--duty" => a.duty = v.parse().map_err(|e| bad(&e))?,
                "--top" => a.top = v.parse().map_err(|e| bad(&e))?,
                _ => return Err(format!("unknown argument {arg}")),
            }
        }
        if !(0.05..=0.9).contains(&a.duty) {
            return Err("--duty must be within 0.05..=0.9".into());
        }
        if a.top == 0 {
            return Err("--top must be at least 1".into());
        }
        Ok(a)
    }

    fn setup_thread(cpu: usize, prio: i32) -> io::Result<()> {
        // SAFETY: rx의 sched.rs와 같음 - 0으로 채운 cpu_set_t·sched_param은 유효하고, 커널은 넘긴 크기만큼만 읽는다.
        // 순서도 sched.rs와 같다: 정책을 먼저 바꾸고 그다음 CPU를 옮김
        unsafe {
            if prio > 0 {
                let mut param: libc::sched_param = zeroed();
                param.sched_priority = prio;
                let rc =
                    libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_FIFO, &param);
                if rc != 0 {
                    return Err(io::Error::from_raw_os_error(rc));
                }
            }
            let mut set: libc::cpu_set_t = zeroed();
            libc::CPU_SET(cpu, &mut set);
            if libc::sched_setaffinity(0, size_of::<libc::cpu_set_t>(), &set) != 0 {
                return Err(io::Error::last_os_error());
            }
        }
        Ok(())
    }

    pub fn run() -> Result<(), Box<dyn std::error::Error>> {
        let a = parse()?;
        setup_thread(a.cpu, a.prio)
            .map_err(|e| format!("CPU {} SCHED_FIFO {} (needs root): {e}", a.cpu, a.prio))?;
        let policy = if a.prio > 0 {
            format!("SCHED_FIFO {}", a.prio)
        } else {
            "SCHED_OTHER".to_string()
        };

        let mut counts = [0u64; THRESHOLDS_US.len()];
        let mut top: Vec<(u64, f64)> = Vec::new(); // (gap µs, 그때의 CLOCK_REALTIME 초)
        let mut measured = Duration::ZERO;
        let mut reads: u64 = 0;
        let spin = Duration::from_secs_f64(a.duty);
        let end = Instant::now() + Duration::from_secs(a.seconds);

        while Instant::now() < end {
            let window = Instant::now();
            let mut last = window;
            while last.duration_since(window) < spin {
                let now = Instant::now();
                reads += 1;
                let gap_us = now.duration_since(last).as_micros() as u64;
                if gap_us >= THRESHOLDS_US[0] {
                    for (c, &t) in counts.iter_mut().zip(&THRESHOLDS_US) {
                        if gap_us >= t {
                            *c += 1;
                        }
                    }
                    if top.len() < a.top || gap_us > top[a.top - 1].0 {
                        let wall = SystemTime::now().duration_since(UNIX_EPOCH)?.as_secs_f64();
                        top.push((gap_us, wall));
                        top.sort_by(|x, y| y.0.cmp(&x.0));
                        top.truncate(a.top);
                    }
                }
                last = now;
            }
            measured += last.duration_since(window);
            // 남은 시간은 쉼 (실시간 작업 제한에 걸리지 않게)
            std::thread::sleep(Duration::from_secs(1).saturating_sub(window.elapsed()));
        }

        let secs = measured.as_secs_f64();
        println!(
            "stall_detect: CPU {}, {policy}, spinning {:.1} s ({:.0}% of each second), {} clock reads",
            a.cpu,
            secs,
            a.duty * 100.0,
            reads
        );
        let line: Vec<String> = THRESHOLDS_US
            .iter()
            .zip(counts)
            .map(|(t, c)| format!(">={t} us: {c} ({:.1}/s)", c as f64 / secs))
            .collect();
        println!("gaps {}", line.join(", "));
        let max = top.first().map_or(0, |g| g.0);
        println!("max gap {max} us; largest at (CLOCK_REALTIME s):");
        for (gap, wall) in &top {
            println!("  {gap:>7} us  at {wall:.6}");
        }
        Ok(())
    }
}
