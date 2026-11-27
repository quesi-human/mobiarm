"""5단계(준비): 접힌 자세(park) 저장 — 한 번만. 이후 m4·미션이 이 자세에서 보는 자세로 스스로 펴고, 끝나면 여기로 접음.

  python steps/m5_teach_park.py

토크를 켜지 않고 읽기만 함. 팔을 평소 쉬는 '접힌 자세'(전원을 끄면 내려앉는 자세)에 둔 채 Enter.
저장 후 접힌 자세 → 보는 자세 경로를 미리 계산해 부딪힐 곳이 없는지 확인해 줌 (팔은 안 움직임).
"""
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot import devices as DEV  # noqa: E402
from robot import park as P  # noqa: E402
from robot import wristcam as WC  # noqa: E402
from robot.arm import ARM, Arm, ArmStop  # noqa: E402
from robot.kinematics import SO101Kinematics  # noqa: E402


def main():
    arm = Arm(port=DEV.ARM_PORT).connect()
    try:
        if any(arm.torque_was.values()):
            print("[주의] 토크가 켜져 있음 — 다른 프로그램이 팔을 쓰는 중이면 끝내고 다시")
            return
        input("팔을 평소 쉬는 접힌 자세에 두고 Enter > ")
        q = arm.read_deg()
        print("  " + ", ".join(f"{j} {q[j]:+.1f}°" for j in ARM))
        out = {"q_deg": {j: q[j] for j in ARM}, "saved": time.strftime("%Y-%m-%d %H:%M:%S")}
        P.PARK_FILE.write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"[저장] {P.PARK_FILE.relative_to(ROOT)}")
        tgt = P.park_target(arm, out["q_deg"])
        moved = {j: round(tgt[j] - q[j], 1) for j in ARM if abs(tgt[j] - q[j]) > 0.05}
        if moved:
            print(f"  측정 범위 끝에 붙은 관절은 목표를 안쪽으로 당겨 씀: {moved} (끝낼 때 토크 OFF하면 내려앉음)")
        try:
            kin = SO101Kinematics()
            wps = P.plan(kin, tgt, WC.load_look_q())
            print(f"  접힌 자세 → 보는 자세 경로 확인: 경유점 {len(wps)}개, 부딪힘 검사 통과")
        except ArmStop as e:
            print(f"  [경로] {e}")
    finally:
        arm.close(disable_torque=False)


if __name__ == "__main__":
    main()
