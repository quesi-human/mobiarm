#!/usr/bin/env python3
"""0단계: 모터에 기록된 영점(Homing_Offset)에 맞춘 '내 PC 전용' 캘리브레이션 파일 만들기.

- 모터에는 아무것도 쓰지 않는다 (읽기 전용, 토크 상태도 바꾸지 않음).
- 기존 파일(~/.cache/.../my_follower_arm.json)도 건드리지 않는다.
- 내가 측정한 range_min/max는 그대로 쓰되, 영점 차이만큼 카운트를 이동시킨다.
  (Feetech: Present_Position = 엔코더값 - Homing_Offset
   → 같은 물리 위치의 값은 (H_file - H_motor)만큼 이동)

실행 (프로젝트 루트):
  python steps/s0_make_local_calibration.py
결과:
  calibration/motor_matched/my_follower_arm.json
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot import devices as DEV  # noqa: E402

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
SRC_DEFAULT = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_follower_arm.json"
MAX_RES = 4095


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default=DEV.ARM_PORT, help="팔 USB (기본: 장치 고유 이름, 없으면 /dev/ttyACM0)")
    p.add_argument("--robot-id", default="my_follower_arm")
    p.add_argument("--source", default=str(SRC_DEFAULT), help="내가 측정한 min/max가 있는 파일")
    p.add_argument("--out-dir", default="calibration/motor_matched")
    args = p.parse_args()

    src = json.loads(Path(args.source).read_text())

    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.robot_id, use_degrees=True, cameras={}))
    robot.bus.connect()
    try:
        motor = robot.bus.read_calibration()   # Homing_Offset, Min/Max_Position_Limit 읽기
    finally:
        try:
            robot.bus.disconnect(disable_torque=False)   # 토크 상태 유지
        except TypeError:
            robot.bus.disconnect()

    out, rows = {}, []
    score_plus = score_minus = 0
    for j in JOINTS:
        s = src[j]
        m = motor[j]
        d = int(s["homing_offset"]) - int(m.homing_offset)
        lo, hi = int(s["range_min"]) + d, int(s["range_max"]) + d
        if j != "wrist_roll":  # wrist_roll은 0~4095 전체라 비교 불가
            # 부호 검증: 다른 사람이 손으로 잰 범위(모터 레지스터)와 더 가까운 쪽이어야 한다
            score_plus += abs(lo - m.range_min) + abs(hi - m.range_max)
            score_minus += abs(int(s["range_min"]) - d - m.range_min) + abs(int(s["range_max"]) - d - m.range_max)
        if j == "wrist_roll":
            lo, hi = int(s["range_min"]), int(s["range_max"])
        lo, hi = max(0, lo), min(MAX_RES, hi)
        out[j] = {"id": s["id"], "drive_mode": s["drive_mode"], "homing_offset": int(m.homing_offset),
                  "range_min": lo, "range_max": hi}
        rows.append((j, s["homing_offset"], m.homing_offset, d, s["range_min"], s["range_max"],
                     lo, hi, m.range_min, m.range_max))

    print(f"{'관절':14} {'H파일':>6} {'H모터':>6} {'이동':>5} | {'내 범위(원본)':>13} | "
          f"{'내 범위(이동)':>13} | {'모터 레지스터':>13}")
    for j, hf, hm, d, a, b, lo, hi, ma, mb in rows:
        print(f"{j:14} {hf:6} {hm:6} {d:+5} | {a:5}~{b:<6} | {lo:5}~{hi:<6} | {ma:5}~{mb:<6}")

    if score_plus > score_minus:
        sys.exit(f"[STOP] 부호 검증 실패(+:{score_plus} / -:{score_minus}). 파일을 만들지 않았습니다. 결과를 공유해 주세요.")
    print(f"[OK] 부호 검증 통과 (모터 레지스터와의 차이 합: 채택 {score_plus} / 반대 {score_minus})")

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{args.robot_id}.json"
    path.write_text(json.dumps(out, indent=4), encoding="utf-8")
    print(f"[SAVED] {path}")


if __name__ == "__main__":
    main()
