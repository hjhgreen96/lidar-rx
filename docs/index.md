# 문서

## 읽는 순서

1. [architecture.md](architecture.md) - 전체 구조, 크레이트 의존 방향, 경계와 디자인 패턴, 데이터 형태 변화, 스레드, 시간, 이상 상황 처리, 테스트와 검증
2. [modules/lidar-core.md](modules/lidar-core.md) - 경계 trait과 공통 타입
3. [modules/vlp16.md](modules/vlp16.md) - 패킷 구조, 발사 타이밍, 보정값, 방위각 보간, 좌표 변환, 프레임 조립, 참조 구현과 공식 드라이버와의 차이
4. [modules/lidar-io.md](modules/lidar-io.md) - pcap 파일 형식, Ethernet/IPv4/UDP 헤더, `recvmsg`와 제어 메시지, `SO_RXQ_OVFL`
5. [modules/clock.md](modules/clock.md) - 하한선 추정(아래쪽 볼록 껍질), 계단 감지, 결과
6. [modules/pcap-replay.md](modules/pcap-replay.md) - 송신 간격, 반복 재생, 제때 보내기, 왕복 시간 측정
7. [modules/rx.md](modules/rx.md) - 수신 루프, threaded/inline, 소비자(Observer), 배열 재사용, ROS 2 발행, RViz
8. [modules/kmod.md](modules/kmod.md) - 커널 모듈: netfilter 훅 위치, softirq 문맥, CPU별 카운터, 잠금, 해제 순서와 RCU
9. [modules/experiments.md](modules/experiments.md) - 수신 경로 실험: 소켓 버퍼 용량, 버퍼 = 지연, 어디서 버릴지(inline vs threaded), 스케줄링 정책
10. [modules/deploy.md](modules/deploy.md) - systemd 서비스 2개: 준비 알림, watchdog, 재시작, 의존 관계, 커널 쪽 설정(모듈 자동 적재, 소켓 버퍼 상한)
11. [modules/tools.md](modules/tools.md) - 데이터와 골든 파일, 검증과 실험 스크립트, 실행 기록, WSL 커널 빌드
12. [runs/](runs/README.md) - 단계별 실행 기록 (명령과 실제 출력, 터미널 영상)

## 정리

- [receive_path.md](receive_path.md) - 수신 경로 전체: 단계별 복사, 버려지는 곳, 확인 방법
- [rust_notes.md](rust_notes.md) - C라면 사람이 챙길 부분을 Rust에서 언어가 챙기는 지점 (C 커널 모듈과 비교)
- [boot_flow.md](boot_flow.md) - 보드 부팅 흐름: BootROM, FSBL, U-Boot, FIT 이미지, device tree와 드라이버, systemd (실물 로그로 확인)
- [runs/5-board.md](runs/5-board.md) - 실제 보드(Arty Z7)에서 한 것 전부: 부팅, 모듈 빌드, 설치, 실제 Ethernet 수신, 찾고 고친 실행 비용, 실험 1~4, 장시간 실행과 장애 시험, 실시간 그래프

## 그림, 영상, 측정 원본

- [media/](media/) - README와 문서의 그림, 영상, 측정 원본 CSV
