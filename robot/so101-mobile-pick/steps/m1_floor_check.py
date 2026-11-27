"""1단계: 팔을 터틀봇에 단 뒤, 물건을 올려 둘 작업면(상자 윗면)이 팔 기준으로 얼마나 아래 있는지 재기 (토크 OFF, 손으로).

11/24: 바닥은 팔을 거의 끝까지 뻗어야 해서(어깨 86~92°/104°) 16cm 상자 위를 작업면으로 함.
팔 손끝(고정 집게 끝)을 상자 윗면에 살짝 닿게 해서 Enter → 관절 각도로 손끝 높이 계산.
상자 윗면의 가까운 곳·먼 곳, 왼쪽·오른쪽 등 4~6곳 → 작업면 평면(높이·기울기) 맞춤.
터틀봇 몸체 앞 끝까지 거리도 함께 입력 → 집을 수 있는 범위 계산.

python steps/m1_floor_check.py [--port /dev/ttyACM0]
→ calibration/mount.json
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot import devices as DEV  # noqa: E402
from robot.arm import ARM, Arm  # noqa: E402
from robot.kinematics import SO101Kinematics  # noqa: E402
from robot import mount as M  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEV.ARM_PORT, help="팔 USB (기본: 장치 고유 이름, 없으면 /dev/ttyACM0)")
    args = ap.parse_args()

    arm = Arm(port=args.port).connect()
    kin = SO101Kinematics()               # 기본 접촉점 = 고정 집게 손끝 (v1에서 검증)
    limits = {m: arm.range_deg(m) for m in ARM}
    pts = []
    try:
        input("팔을 손으로 받치고 Enter → 토크 OFF > ")
        arm.torque_off()
        print("\n손끝(고정 집게 끝)을 상자 윗면에 살짝 닿게 하고 Enter. 4~6곳 (가까운·먼, 왼쪽·오른쪽). 끝나면 q + Enter")
        while True:
            a = input(f"  [{len(pts)+1}] 상자 윗면에 닿게 하고 Enter (q = 끝) > ").strip().lower()
            if a == "q":
                break
            q = arm.read_deg()
            p, _ = kin.fk([q[j] for j in ARM])
            pts.append(p.tolist())
            print(f"      손끝 ({p[0]*100:.1f}, {p[1]*100:.1f}, {p[2]*100:.1f}) cm")
        if len(pts) < 3:
            print("[중단] 3곳 이상 필요")
            return
        P = np.array(pts)
        A = np.c_[np.ones(len(P)), P[:, 0], P[:, 1]]
        coef, *_ = np.linalg.lstsq(A, P[:, 2], rcond=None)
        resid = P[:, 2] - A @ coef
        z0 = coef[0] + coef[1] * M.PAN_AXIS_XY[0]
        tilt = np.degrees(np.arctan(np.hypot(coef[1], coef[2])))
        print(f"\n작업면 높이(팔 기준) {z0*100:.1f} cm, 기울기 {tilt:.1f}°, 점별 오차 최대 {np.abs(resid).max()*1000:.1f} mm")
        if np.abs(resid).max() > 0.005:
            print("  [주의] 점별 오차 5mm 초과 — 손끝이 덜 닿았거나 떠 있던 점이 있을 수 있음")
        if tilt > 3:
            print("  [주의] 기울기 3° 초과 — 상자나 팔 고정이 기울었는지 확인 (v2 계산은 작업면을 수평으로 가정)")

        b = input("\n팔 회전축(베이스 중심)에서 터틀봇 몸체 앞 끝까지 거리 cm (모르면 Enter) > ").strip()
        bumper = float(b) / 100 if b else None

        # 위에서 수직으로 집을 수 있는 거리 (v1 IK 그대로)
        kin_g = SO101Kinematics(contact=M.grasp_point())
        zc = z0 + M.BLOCK_M / 2
        ok = [x for x in np.arange(0.04, 0.36, 0.005)
              if kin_g.ik_topdown(M.PAN_AXIS_XY[0] + x, 0.0, zc, limits=limits).ok]
        if ok:
            near = max(min(ok), (bumper + 0.02) if bumper else 0)   # 몸체 앞 끝 + 2cm 여유
            print(f"위에서 집을 수 있는 거리(정면, 팬 축 기준): {min(ok)*100:.0f}~{max(ok)*100:.0f} cm"
                  + (f" → 몸체 앞만 치면 {near*100:.0f}~{max(ok)*100:.0f} cm" if bumper else ""))
        else:
            print("[주의] 이 작업면 높이에서는 위에서 수직으로 집을 수 있는 거리가 없음")

        d = {"surface_z_m": float(z0), "floor_plane_coef": coef.tolist(), "tilt_deg": float(tilt),
             "points_m": pts, "saved": time.strftime("%Y-%m-%d %H:%M:%S")}
        if bumper:
            d["bumper_x_m"] = bumper
        if ok:
            d["topdown_reach_m"] = [float(min(ok)), float(max(ok))]
        M.MOUNT_FILE.parent.mkdir(parents=True, exist_ok=True)
        M.MOUNT_FILE.write_text(json.dumps(d, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"[저장] {M.MOUNT_FILE}")
    finally:
        arm.close(disable_torque=True)


if __name__ == "__main__":
    main()
