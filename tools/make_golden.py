"""참조 디코더(velodyne_decoder 3.1.0)의 출력을 골든 파일(CSV)로 저장한다.

프레임 자르는 방식은 구현마다 다르므로(참조는 패킷 단위, lidar-rx 해석기는 발사 단위), 포인트마다
(패킷 번호, 발사 번호, ring)을 붙여 프레임과 무관하게 비교할 수 있게 한다.

- 입력은 제품 ID를 고친 사본(tools/patch_product_id.py). 패킷 시각은 원본과 같다.
- 프레임마다 어느 패킷들인지는 가정하지 않고 데이터로 구한다:
  마지막 패킷 = 프레임 대표 시각(정시 이후 µs)과 센서 타임스탬프가 같은 패킷,
  패킷 수 = 발사 번호(column) 개수 ÷ 24. 프레임끼리 패킷이 빈틈없이 이어지는지도 확인한다.
  (이 파일에서는 첫 프레임만 77패킷, 그 뒤는 76패킷)

열
  packet           data 패킷 번호 (pcap 안의 순서, 0부터)
  firing           패킷 안 발사 번호 0..23 (블록 b의 두 발사 = 2b, 2b+1)
  ring             수직각 오름차순 레이저 번호 0..15
  x y z            m, ROS 좌표계(x 앞, y 왼쪽, z 위)
  intensity        반사율 원시값
  t_rel_frame_s    프레임 대표 시각(마지막 패킷)으로부터의 상대 시각, s (참조 출력 그대로)
  t_from_packet_us 그 패킷의 센서 타임스탬프로부터의 오프셋, µs (위 값에서 계산)

사용: python make_golden.py <vlp16_pid22.pcap> <출력.csv> [프레임 수=2]
"""
import struct
import sys
from importlib.metadata import version

import numpy as np
import velodyne_decoder as vd

FIRINGS_PER_PACKET = 24
ETH_IP_UDP = 14 + 20 + 8
PAYLOAD = 1206
TS_OFFSET = 1200


def sensor_timestamps(path: str) -> np.ndarray:
    """data 패킷마다 센서 타임스탬프(정시 이후 µs)."""
    data = open(path, 'rb').read()
    off, ts = 24, []
    while off + 16 <= len(data):
        incl = struct.unpack('<I', data[off + 8:off + 12])[0]
        off += 16
        if incl == ETH_IP_UDP + PAYLOAD:
            p = off + ETH_IP_UDP + TS_OFFSET
            ts.append(struct.unpack('<I', data[p:p + 4])[0])
        off += incl
    return np.array(ts, dtype=np.int64)


def main(pcap: str, out: str, n_frames: int) -> None:
    ts = sensor_timestamps(pcap)
    index_of_ts = {int(t): i for i, t in enumerate(ts)}
    cfg = vd.Config(model=vd.Model.VLP16, cut_angle=None, timestamp_first_packet=False)

    rows, ranges, next_first = [], [], 0
    for k, frame in enumerate(vd.read_pcap(pcap, cfg)):
        if k == n_frames:
            break
        p = frame.points
        assert p.shape[1] == 8, p.shape          # x y z intensity time column ring return_type
        col = p[:, 5].astype(np.int64)

        stamp_us = round((frame.stamp.device % 3600.0) * 1e6)
        last = index_of_ts.get(stamp_us)
        n_cols = int(col.max()) + 1
        if last is None or n_cols % FIRINGS_PER_PACKET:
            sys.exit(f'frame {k}: stamp {stamp_us} us → packet {last}, columns {n_cols}')
        first = last - n_cols // FIRINGS_PER_PACKET + 1
        if first != next_first:
            sys.exit(f'frame {k}: packets {first}..{last}, expected to start at {next_first}')
        next_first = last + 1
        ranges.append(f'{first}..{last}')

        packet = first + col // FIRINGS_PER_PACKET
        firing = col % FIRINGS_PER_PACKET
        t_rel = p[:, 4].astype(np.float64)
        t_from_packet_us = t_rel * 1e6 + (ts[last] - ts[packet])
        rows.append(np.column_stack([packet, firing, p[:, 6], p[:, 0], p[:, 1], p[:, 2], p[:, 3],
                                     p[:, 4], t_from_packet_us]))

    g = np.vstack(rows)
    header = '\n'.join([
        f'velodyne_decoder {version("velodyne_decoder")}, model VLP16, '
        f'min_range {cfg.min_range:.2f} m, max_range {cfg.max_range:.1f} m, cut_angle None, '
        f'timestamp_first_packet False, frames {n_frames} (packets {", ".join(ranges)})',
        'packet,firing,ring,x,y,z,intensity,t_rel_frame_s,t_from_packet_us',
    ])
    fmt = ['%d', '%d', '%d', '%.9g', '%.9g', '%.9g', '%d', '%.9g', '%.4f']
    np.savetxt(out, g, fmt=fmt, delimiter=',', header=header)
    print(f'{len(g)} points, frames {", ".join(ranges)} → {out}')


if __name__ == '__main__':
    if len(sys.argv) not in (3, 4):
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2], int(sys.argv[3]) if len(sys.argv) == 4 else 2)
