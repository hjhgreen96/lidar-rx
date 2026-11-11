"""VLP-16 데이터 패킷의 제품 ID 바이트만 0x21(HDL-32E) → 0x22(VLP-16)로 바꾼 사본을 만든다.

샘플 pcap은 구조와 타이밍이 VLP-16인데 제품 ID가 0x21로 찍혀 있어, velodyne_decoder 3.0.0이
HDL-32E로 잘못 해석하거나(자동 감지) VLP-16 지정을 거부한다. 참조 출력을 만들 때만 이 사본을 쓴다.

사용: python3 patch_product_id.py <원본.pcap> <사본.pcap>
"""
import struct
import sys

PCAP_GLOBAL_HEADER = 24
PCAP_RECORD_HEADER = 16
ETH_IP_UDP = 14 + 20 + 8
PAYLOAD = 1206
PRODUCT_ID_OFFSET = 1205      # 페이로드 끝 factory 2 B 중 두 번째 바이트
OLD, NEW = 0x21, 0x22


def main(src: str, dst: str) -> None:
    data = bytearray(open(src, 'rb').read())
    magic = struct.unpack('<I', data[:4])[0]
    if magic != 0xA1B2C3D4:
        sys.exit(f'little-endian µs pcap만 처리함 (magic=0x{magic:08x})')

    off = PCAP_GLOBAL_HEADER
    n_data = n_patched = 0
    while off + PCAP_RECORD_HEADER <= len(data):
        incl = struct.unpack('<I', data[off + 8:off + 12])[0]
        off += PCAP_RECORD_HEADER
        if incl == ETH_IP_UDP + PAYLOAD:
            n_data += 1
            pos = off + ETH_IP_UDP + PRODUCT_ID_OFFSET
            if data[pos] == OLD:
                data[pos] = NEW
                n_patched += 1
        off += incl

    open(dst, 'wb').write(data)
    print(f'data packets {n_data}, patched {n_patched} → {dst}')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
