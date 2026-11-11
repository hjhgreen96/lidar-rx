"""공식 velodyne 드라이버가 버리는 포인트를 예측하고, 실제 차이와 비교한다.

공식 드라이버(velodyne_pointcloud rawdata.cpp, unpack_vlp16)는 블록마다 "다음 블록 방위각 − 이 블록 방위각"으로
회전 간격을 구한다. 그 값이 음수(0°를 넘음)면 직전 블록의 간격을 대신 쓰는데, 직전 값(last_azimuth_diff)은
패킷마다 0으로 시작한다. 그래서 0°를 넘는 지점이 블록 0과 블록 1 사이에 오면 쓸 값이 없어 블록 0 전체를
건너뛴다(continue).

사용: python3 official_drop_check.py <vlp16.pcap> <lidar-rx 76패킷 묶음별 수> <공식 메시지별 수>
"""
import struct
import sys

ETH_IP_UDP, PAYLOAD = 42, 1206


def predicted_drops_per_packet(pcap):
    data = open(pcap, 'rb').read()
    off, drops = 24, []
    while off + 16 <= len(data):
        incl = struct.unpack('<I', data[off + 8:off + 12])[0]
        off += 16
        if incl == ETH_IP_UDP + PAYLOAD:
            p = off + ETH_IP_UDP
            az0 = struct.unpack('<H', data[p + 2:p + 4])[0]
            az1 = struct.unpack('<H', data[p + 102:p + 104])[0]
            n = 0
            if az1 < az0:  # 블록 0 → 1 사이에서 0°를 넘음
                for c in range(32):
                    raw = struct.unpack('<H', data[p + 4 + 3 * c:p + 6 + 3 * c])[0]
                    if raw and 0.1 <= raw * 0.002 <= 200.0:  # 거리 필터를 통과했을 포인트만
                        n += 1
            drops.append(n)
        off += incl
    return drops


def main(pcap, mine_path, official_path):
    per_packet = predicted_drops_per_packet(pcap)
    predicted = [sum(per_packet[i:i + 76]) for i in range(0, len(per_packet) - 75, 76)]
    mine = [int(x) for x in open(mine_path)]
    official = [int(x) for x in open(official_path)]
    if not official or len(official) != len(mine):
        # 공식 드라이버 메시지를 못 받았거나 일부만 받음 → 비교 자체가 성립하지 않음 (zip이 짧은 쪽에서 멈춤)
        sys.exit(f'MISMATCH: received {len(official)} official messages, expected {len(mine)} '
                 '(driver not running, or messages were missed)')
    actual = [m - o for m, o in zip(mine, official)]
    print(f'groups: ours {len(mine)}, official {len(official)}')
    print(f'packets where 0° falls between block 0 and 1: {sum(1 for d in per_packet if d)}')
    print(f'predicted total drop {sum(predicted)}, actual total difference {sum(actual)}')
    print('groups with a difference (index, predicted, actual):',
          [(i, p, a) for i, (p, a) in enumerate(zip(predicted, actual)) if p or a])
    if predicted[:len(actual)] != actual:
        sys.exit('MISMATCH: the differences are not explained by the block-0 skip')
    print('all groups match the prediction')


if __name__ == '__main__':
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    main(*sys.argv[1:4])
