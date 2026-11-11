## 샘플 데이터

ROS 공식 서버의 VLP-16 캡처를 받아 해시를 확인하고, 참조 디코더 비교용으로 제품 ID 바이트만 고친 사본을 만든다.

```console
$ tools/fetch_data.sh 2>&1 | grep -v "%"
data/vlp16.pcap: OK
data packets 7093, patched 7093 → data/vlp16_pid22.pcap
```

## 단위 테스트

```console
$ cargo test -q -p vlp16 2>&1 | grep -E "^(running|test result)"
running 21 tests
test result: ok. 21 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.02s
running 1 test
test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.16s
running 0 tests
test result: ok. 0 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
```

## 참조 구현(velodyne_decoder 3.1.0)과 포인트 단위 대조

골든 파일(패킷 0..152, 54,952 포인트)과 (패킷, 발사, ring)으로 짝을 지어 비교한다. 허용 오차: 수평 거리·z 0.1 mm, 방위각 0.006°(참조의 0.01° 반올림), 시각 0.25 µs(참조의 double 해상도).

```console
$ cargo test -q -p vlp16 --test golden -- --nocapture 2>&1 | grep -E "points, max diff|test result"
54952 points, max diff MaxDiff { horizontal_m: 1.2780117604194174e-6, z_m: 4.9890136466501644e-9, azimuth_deg: 0.005071153110179694, time_us: 0.23220000000014807 }
test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.15s
```

## 샘플 전체를 프레임으로

pcap 전체(7,093 패킷)를 해석해 프레임마다 한 줄. 첫 프레임은 한 바퀴 중간부터, 마지막 프레임은 파일 끝에서 끊긴 조각(full = no).

```console
$ cargo run -q --release -p vlp16 --example inspect_pcap -- data/vlp16.pcap | sed -n "1,5p"
frame  points  packets  missing  full  stamp(s past hour)  time span(s)
    0    1833        5        0    no          259.064360  -0.0053 .. +0.0012
    1   27044       77        0   yes          259.165220  -0.0996 .. +0.0003
    2   27038       76        0   yes          259.264753  -0.0992 .. +0.0008
    3   27030       76        0   yes          259.364286  -0.0988 .. +0.0011
```

```console
$ cargo run -q --release -p vlp16 --example inspect_pcap -- data/vlp16.pcap | tail -n 3
   94   27055       76        0   yes          268.459263  -0.0995 .. +0.0004
   95    3665       10        0    no          268.471207  -0.0116 .. +0.0013
DecoderStats { packets: 7093, malformed_packets: 0, product_id_mismatches: 7093, assembler: AssemblerCounters { frames: 96, missing_packets: 0, reordered_packets: 0, restarts: 0, gap_cuts: 0 } }
```

_기록한 날: 2026-11-09 13:34, 커널 6.18.54.1-microsoft-standard-WSL2_
