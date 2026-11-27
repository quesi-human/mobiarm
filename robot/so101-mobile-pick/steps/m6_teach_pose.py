"""6단계(준비): 운반 자세(carry)·내려놓는 자세(place) 저장 — 팔 서버가 씀.

  python steps/m6_teach_pose.py carry    # 블록을 든 채 터틀봇이 이동할 자세
  python steps/m6_teach_pose.py place    # 목적지에서 이 자세로 가서 그리퍼를 열어 떨어뜨림

토크를 켜지 않고 읽기만 함. 팔을 손으로 원하는 자세에 두고 Enter.
  carry: 팔을 몸 쪽으로 당겨 무게중심을 낮추고, **라이다(맨 위 LDS) 시야를 가리지 않게**. 블록이 그리퍼 아래로 매달림을 생각.
  place: 그리퍼가 통(쓰레기통) 입구 바로 위에 오게. 터틀봇이 목적지에서 서는 방향 기준.
저장 후 보는 자세 ↔ 이 자세 경로가 상자·터틀봇 윗판에 부딪히지 않는지 계산해 알려 줌 (팔은 안 움직임).
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
from robot.tasks import POSE_FILES  # noqa: E402


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in POSE_FILES:
        sys.exit(f"사용법: python steps/m6_teach_pose.py {{{'|'.join(POSE_FILES)}}}")
    name = sys.argv[1]
    arm = Arm(port=DEV.ARM_PORT).connect()
    try:
        if any(arm.torque_was.values()):
            print("[주의] 토크가 켜져 있음 — 팔 서버 등 다른 프로그램을 끄고 다시")
            return
        input(f"팔을 '{name}' 자세에 두고(손으로 받친 채) Enter > ")
        q = arm.read_deg()
        print("  " + ", ".join(f"{j} {q[j]:+.1f}°" for j in ARM))
        prob = arm.check_targets_deg({j: q[j] for j in ARM})
        if prob:
            print(f"  [다시] 관절 범위 끝에 너무 가까움: {prob} — 조금 안쪽으로")
            return
        kin = SO101Kinematics()
        p, R = kin.fk([q[j] for j in ARM])
        print(f"  손끝 위치: 팔 ({p[0]*100:.1f}, {p[1]*100:.1f}, {p[2]*100:.1f}) cm")
        out = {"q_deg": {j: q[j] for j in ARM}, "tip_m": p.tolist(), "saved": time.strftime("%Y-%m-%d %H:%M:%S")}
        POSE_FILES[name].write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"[저장] {POSE_FILES[name].relative_to(ROOT)}")
        try:
            look = WC.load_look_q()
            n1 = len(P.plan(kin, look, out["q_deg"]))
            n2 = len(P.plan(kin, out["q_deg"], look))
            print(f"  보는 자세 ↔ {name} 경로 확인: 경유점 {n1}/{n2}개, 부딪힘 검사 통과")
        except ArmStop as e:
            print(f"  [경로] {e}")
    finally:
        arm.close(disable_torque=False)


if __name__ == "__main__":
    main()
