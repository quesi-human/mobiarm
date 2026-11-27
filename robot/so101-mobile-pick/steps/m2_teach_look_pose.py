"""2단계: 손목 카메라로 작업면(상자 윗면)을 내려다보는 '보는 자세' 정하기 (토크 OFF, 손으로).

팔을 옮기며 Enter를 누를 때마다 손목 카메라 사진을 저장 → 사진을 보고 마음에 들면 l + Enter.
좋은 보는 자세: 그리퍼가 수직(출력 '아래 방향'이 1에 가까움)이고 팔이 '터틀봇 정면'을 향하며,
집을 수 있는 범위(몸체 앞 ~15cm)가 화면에 넓게 들어오고, 그리퍼·몸체가 화면을 덜 가림.
(그리퍼 수직: travel()로 바로 이어지게 / 정면: 이 자세의 방향을 '터틀봇 정면'으로 씀(mount.forward_heading_deg).
 이 팔은 팬 0°가 터틀봇 정면이 아님 — 11/24 정면 = 팬 약 -29.5°. 팬만 돌린 자세는 보정을 회전해서 확장)
보는 자세는 매번 정확히 같은 관절 각도로 돌아가므로, 이 자세에서 한 번 보정하면 계속 쓸 수 있음(3단계).

python steps/m2_teach_look_pose.py [--cam /dev/video0]
→ calibration/look_pose.json, 사진은 steps/logs/look_<번호>.png
"""
import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot import devices as DEV  # noqa: E402
from robot.arm import ARM, Arm  # noqa: E402
from robot.kinematics import SO101Kinematics  # noqa: E402
from robot import mount as M  # noqa: E402
from vision.camera import UsbCamera  # noqa: E402

LOGS = ROOT / "steps/logs"
KO = {"ㅣ": "l", "ㅂ": "q", "ㅛ": "y"}   # 한글 입력 상태로 눌러도 되게 (ㅣ=l, ㅂ=q, ㅛ=y)


def ask(prompt):
    a = input(prompt).strip().lower()
    return KO.get(a[:1], a[:1]) if a else ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEV.ARM_PORT, help="팔 USB (기본: 장치 고유 이름, 없으면 /dev/ttyACM0)")
    ap.add_argument("--cam", default=DEV.WRIST_CAM, help="손목 카메라 (기본: 장치 고유 이름, 없으면 /dev/video0)")
    args = ap.parse_args()

    arm = Arm(port=args.port).connect()
    kin = SO101Kinematics()
    cam = UsbCamera(args.cam).start()
    try:
        input("팔을 손으로 받치고 Enter → 토크 OFF > ")
        arm.torque_off()
        print("\n그리퍼가 상자 윗면을 내려다보게 팔을 옮기고 Enter → 사진 저장. 마음에 들면 l + Enter, 그만두려면 q")
        n = 0
        while True:
            a = ask(f"  [{n+1}] Enter = 사진, l = 이 자세 저장, q = 끝 > ")
            if a not in ("", "l", "q"):
                print("      Enter(빈 칸), l, q 중 하나만 입력")
                continue
            if a == "q":
                return
            q = arm.read_deg()
            p, R = kin.fk([q[j] for j in ARM])
            if a == "l":
                down = float((-R[:, 2])[2])
                if down < 0.97:
                    print(f"  [확인] 그리퍼 아래 방향 {down:.2f}(권장 0.97 이상)")
                    if ask("  l 또는 y = 그래도 저장, 그냥 Enter = 취소 > ") not in ("l", "y"):
                        print("      저장 취소 — 자세를 고친 뒤 다시 l")
                        continue
                prob = arm.check_targets_deg({j: q[j] for j in ARM})
                if prob:
                    print(f"  [다시] 관절이 범위 끝 5° 안: {prob}")
                    continue
                img = LOGS / "look_pose_saved.png"
                cam.snapshot(img)
                d = {"q_deg": {j: q[j] for j in ARM}, "tip_m": p.tolist(), "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
                     "camera": args.cam, "image": str(img.relative_to(ROOT))}
                M.LOOK_FILE.parent.mkdir(parents=True, exist_ok=True)
                M.LOOK_FILE.write_text(json.dumps(d, indent=2, ensure_ascii=False), encoding="utf-8")
                print(f"[저장] {M.LOOK_FILE} (사진 {img})")
                print(f"       이 자세 방향(팬 {q['shoulder_pan']:+.1f}°)을 '터틀봇 정면'으로 씀 — 팔이 터틀봇 정면을 향했는지 확인")
                return
            n += 1
            img = LOGS / f"look_{n}.png"
            cam.snapshot(img)
            try:
                fz = M.surface_z()
                h = f", 작업면 위 {(p[2]-fz)*100:.1f} cm"
            except RuntimeError:
                h = ""
            print(f"      손끝 ({p[0]*100:.1f}, {p[1]*100:.1f}) cm{h}, 팬 {q['shoulder_pan']:+.1f}°, "
                  f"그리퍼 아래 방향 {(-R[:, 2])[2]:.2f} (1이면 수직)"
                  f" → 사진 {img}")
    finally:
        cam.stop()
        arm.close(disable_torque=True)


if __name__ == "__main__":
    main()
