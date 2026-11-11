# 단계별 실행 기록

- 단계별로 실제 실행한 명령(입력)과 터미널 출력 그대로
- 같은 기록으로 만든 터미널 영상

| 단계 | 내용 | 실행 기록 | 영상 |
|---|---|---|---|
| 0 | 실행 중인 WSL 커널용 모듈 빌드 환경, 확인용 모듈 | [0-kernel.md](0-kernel.md) | [터미널](../media/run0_kernel.mp4) |
| 1 | VLP-16 해석기, 참조 구현과 포인트 단위 대조 | [1-decoder.md](1-decoder.md) | [터미널](../media/run1_decoder.mp4) |
| 2 | 시계 추정, 정답을 아는 드리프트 시험, ROS 2 발행, 공식 드라이버 비교 | [2-rx-clock-ros2.md](2-rx-clock-ros2.md) | [터미널](../media/run2_rx.mp4) · [RViz 데모](../media/demo.mp4) |
| 3 | 커널 모듈 lidar_mon, 실험 1·2 (소켓 버퍼 용량, 버퍼 = 지연) | [3-kmod.md](3-kmod.md) | [터미널](../media/run3_kmod.mp4) |
| 3 | 실험 3·4 (느린 소비자: inline vs threaded, 부하 아래 스케줄링 정책) | [3-exp34.md](3-exp34.md) | [터미널](../media/run3_exp34.mp4) |
| 4 | systemd 서비스 두 개: 준비 알림, 센서 조용함 vs 멈춤(watchdog), 재시작, 녹화, 모듈 대조 | [4-systemd.md](4-systemd.md) | [터미널](../media/run4_systemd.mp4) |
| 5 | 실제 보드(Arty Z7-20): 부팅 흐름, 보드에서 모듈 빌드, 서비스 설치, 실제 Ethernet 수신, 보드에서 찾은 실행 비용과 고친 것 | [5-board.md](5-board.md) | [RViz + 보드 터미널](../media/board_demo.mp4) |

- 각 기록 끝에 기록한 날짜와 커널 버전
- 숫자는 실행마다 조금씩 다름 (예: 송신 지연, 캡처 간격으로 보낼 때의 기울기)
