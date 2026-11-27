"""팔 모터 응답 확인 (읽기 전용 — 모터에 아무것도 쓰지 않음).

  python steps/bus_scan.py [--port /dev/ttyACM0]

여러 통신 속도로 ID 1~6에 ping을 보내 응답하는지 봄.
  - 1000000에서 응답 → 정상 (lerobot 기본)
  - 다른 속도에서만 응답 → 모터 통신 속도가 바뀐 것 (직접 고치지 말고 알려 주기)
  - 어디서도 응답 없음 → 전원(12V)·케이블·보드 점퍼 문제
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot import devices as DEV  # noqa: E402

import scservo_sdk as scs  # noqa: E402

BAUDS = [1000000, 500000, 250000, 128000, 115200, 57600, 38400]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEV.ARM_PORT)
    ap.add_argument("--ids", default="1-6", help='예: "1-6" 또는 "1-20"')
    args = ap.parse_args()
    a, b = (int(v) for v in args.ids.split("-"))
    ids = range(a, b + 1)

    ph = scs.PortHandler(args.port)
    if not ph.openPort():
        sys.exit(f"포트 열기 실패: {args.port}")
    pk = scs.PacketHandler(0)
    print(f"포트 {args.port}")
    try:
        any_found = False
        for baud in BAUDS:
            ph.setBaudRate(baud)
            found = []
            for i in ids:
                model, res, err = pk.ping(ph, i)
                if res == scs.COMM_SUCCESS:
                    found.append((i, model, err))
            any_found |= bool(found)
            print(f"  {baud:>7} bps: " + (", ".join(f"ID{i}(모델 {m}{', 오류비트 ' + hex(e) if e else ''})"
                                                    for i, m, e in found) if found else "응답 없음"))
        if not any_found:
            print("\n어떤 속도에서도 응답 없음 → 12V 전원, 보드→첫 모터 케이블, 보드의 USB/모드 점퍼 확인")
    finally:
        ph.closePort()


if __name__ == "__main__":
    main()
