#!/bin/bash
# 샘플 데이터를 data/에 받는다.
#   data/vlp16.pcap        ROS 공식 서버의 VLP-16 캡처 (원본, 해시로 확인)
#   data/vlp16_pid22.pcap  제품 ID 바이트만 0x21 → 0x22로 고친 사본 (참조 디코더 비교용)
set -euo pipefail
cd "$(dirname "$0")/.."

URL=http://download.ros.org/data/velodyne/vlp16.pcap
SHA256=84727cd4645ec78bd8a1bb270447d70e9e6c61bf6c726d7d040c81c5c8ca12a6

mkdir -p data
if [ ! -f data/vlp16.pcap ]; then
    curl -L --fail -o data/vlp16.pcap "$URL"
fi
echo "$SHA256  data/vlp16.pcap" | sha256sum -c -

python3 tools/patch_product_id.py data/vlp16.pcap data/vlp16_pid22.pcap
