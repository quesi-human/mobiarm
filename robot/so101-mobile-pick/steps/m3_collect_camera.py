"""3단계: 손목 카메라 보정 수집 — 블록을 '화면에 보이는 곳'에 놓고, 손끝으로 짚어 위치를 알려 주는 방식.

  python steps/m3_collect_camera.py --color yellow

11/24 첫 시도(로봇이 정한 격자 점에 블록 놓기): 손목 카메라가 그리퍼 오른쪽에 달려 있어 화면이 오른쪽으로 치우쳐
16점 중 7점만 보였음 → 이제는 화면을 보고 블록을 놓은 뒤, 그 블록을 손끝으로 짚는다(m1처럼 토크 OFF).

점마다:
  1) 팔은 '보는 자세'. 블록을 화면에 보이는 새 위치에 놓고 Enter → 화면 어디에 보이는지 출력
  2) 팔을 받치고 Enter(토크 OFF) → 손끝(고정 집게 끝)을 블록 윗면 '중앙'에 살짝 대고 Enter → 위치 기록
  3) 팔을 블록 위 3~5cm로 들어 그리퍼를 대충 수직으로 받치고 Enter(토크 ON) → 손 떼고 Enter
     → 로봇이 위로 빠져 보는 자세로 돌아가 다시 찍음 (짚다가 블록이 밀렸어도 이 사진을 씀)
화면 9칸(3×3)에 고르게 12~15점 모으면 좋음 (그리퍼에 가려지지 않는 곳, 가장자리에 걸치지 않게). q로 끝.

결과: steps/logs/m3_<시각>.jsonl, 사진 steps/logs/m3_<시각>_p<번호>.png
다음: python steps/m3_solve_camera.py steps/logs/m3_<시각>.jsonl --save
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
from robot.arm import ARM, Arm, ArmStop  # noqa: E402
from robot.kinematics import SO101Kinematics  # noqa: E402
from robot.motion import go, plan_arc, plan_line  # noqa: E402
from robot import mount as M  # noqa: E402
from vision.camera import COLOR_RANGES, UsbCamera  # noqa: E402

START_MARGIN_DEG = 1.0
LOOK_TOL_COUNTS = 20
MAX_SPREAD_PX = 3.0
TOUCH_TOL_M = 0.008      # 손끝 높이가 블록 윗면과 이보다 많이 다르면 다시 짚기
MOVED_WARN_PX = 8.0      # 짚기 전후 사진에서 블록이 이만큼 넘게 움직였으면 알려 줌
LIFT_DOWN_MIN = 0.85     # 토크 ON 전 그리퍼 수직 정도(1=수직) — 이보다 기울면 다시
KO = {"ㅂ": "q", "ㅏ": "k"}
W, H = 640, 480


def ask(prompt):
    a = input(prompt).strip().lower()
    return KO.get(a[:1], a[:1]) if a else ""


def cell(px):
    c, r = min(2, int(px[0] // (W / 3))), min(2, int(px[1] // (H / 3)))
    return r, c


def coverage(cells):
    names = [["왼위", "가운위", "오위"], ["왼중", "가운데", "오중"], ["왼아래", "가운아래", "오아래"]]
    rows = []
    for r in range(3):
        rows.append(" ".join(f"{names[r][c]}:{cells.get((r, c), 0)}" for c in range(3)))
    return "  화면 칸별 점 수 → " + " | ".join(rows)


def measure(cam, color, save=None):
    m, diag = cam.measure(color, n_frames=12, save_image=save)
    if m is not None and diag.get("spread_px", 0) <= MAX_SPREAD_PX:
        return m, diag
    return None, diag


def back_to_look(arm, kin, limits, look_q, p_look):
    """지금 자세(손으로 들어 둔 곳) → 위로 → 보는 자세."""
    q = arm.read_deg()
    p, _ = kin.fk([q[j] for j in ARM])
    above = np.array([p[0], p[1], max(p[2], M.surface_z(*p[:2]) + M.TRAVEL_ABOVE_SURFACE_M)])
    if above[2] - p[2] > 0.005:
        wps, q_ref = plan_line(kin, limits, p, above, q)
        go(arm, wps, precise=False)
    else:
        q_ref = q
    wps, _ = plan_arc(kin, limits, above, p_look, q_ref)
    go(arm, wps, precise=False)
    # 보는 자세는 매번 같은 관절값으로. 어깨가 무게로 0.6~0.9° 처지는 건 매번 같은 방향이라 괜찮음
    # (11/24: 오차 9~10카운트로 '오차 남음' STOP → 허용 20카운트 ≈ 1.8°, 넘으면 멈춤)
    res = arm.move_deg(look_q, result_tol=LOOK_TOL_COUNTS)
    if not res.status.startswith("도착"):
        raise ArmStop(res.status)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--color", default="yellow", choices=sorted(COLOR_RANGES))
    ap.add_argument("--port", default=DEV.ARM_PORT, help="팔 USB (기본: 장치 고유 이름, 없으면 /dev/ttyACM0)")
    ap.add_argument("--cam", default=DEV.WRIST_CAM, help="손목 카메라 (기본: 장치 고유 이름, 없으면 /dev/video0)")
    ap.add_argument("--resume", help="이전 수집 로그(.jsonl)에 이어서 모으기")
    args = ap.parse_args()

    if not M.LOOK_FILE.is_file():
        sys.exit("calibration/look_pose.json 없음 — 먼저 python steps/m2_teach_look_pose.py")
    look = json.loads(M.LOOK_FILE.read_text(encoding="utf-8"))
    look_q = {j: float(look["q_deg"][j]) for j in ARM}
    kin = SO101Kinematics()
    p_look, _ = kin.fk([look_q[j] for j in ARM])

    arm = Arm(port=args.port).connect()
    limits = {m: arm.range_deg(m) for m in ARM}
    cam = None
    stamp = time.strftime("%Y%m%d_%H%M%S")
    log, log_path = [], ROOT / f"steps/logs/m3_{stamp}.jsonl"
    if args.resume:
        log_path = Path(args.resume).resolve()
        log = [json.loads(l) for l in log_path.read_text(encoding="utf-8").splitlines() if l.strip()]
        log = [r for r in log if r.get("mode") == "touch"]
        print(f"[이어서] {log_path.name}: {len(log)}점")
    cells = {}
    for r in log:
        k = cell(r["camera"]["pixel"])
        cells[k] = cells.get(k, 0) + 1
    try:
        prob = arm.check_targets_deg(look_q)
        if prob:
            raise ArmStop(f"보는 자세가 관절 범위 끝에 너무 가까움: {prob} — m2 다시")
        cam = UsbCamera(args.cam).start()

        print("팔을 손으로 '보는 자세' 근처(상자 위, 터틀봇 정면)로 옮겨 받쳐 주세요.")
        while True:
            if ask("받친 채 Enter (토크 ON), q = 끝 > ") == "q":
                return
            now = arm.read_deg()
            bad = [f"{j}={now[j]:.1f}°" for j in ("shoulder_lift", "elbow_flex", "wrist_flex")
                   if not (arm.range_deg(j)[0] + START_MARGIN_DEG < now[j] < arm.range_deg(j)[1] - START_MARGIN_DEG)]
            if not bad:
                break
            print(f"  [다시] 아직 측정 범위 밖 {bad} — 팔을 접힌 자세에서 더 펴서 상자 위로 들어 주세요")
        start_q = arm.read_deg()
        arm.torque_on()
        arm.gripper_torque_on()
        time.sleep(0.3)
        ask("자세가 유지되면 손을 떼고 Enter (그리퍼를 닫고 보는 자세로 감) > ")
        arm.gripper_move_raw(arm._gripper_raw(0))
        back_to_look(arm, kin, limits, look_q, p_look)

        n = len(log)
        while True:
            print(f"\n[점 {n+1}]" + (f"\n{coverage(cells)}" if cells else ""))
            a = ask(f"  '{args.color}' 블록 하나를 화면에 보이는 새 위치에 놓고 손을 치운 뒤 Enter (q = 끝) > ")
            if a == "q":
                break
            miss = log_path.with_name(f"{log_path.stem}_miss.png")
            m0, diag = measure(cam, args.color, save=miss)
            if m0 is None:
                print(f"  [카메라] 못 찾음 {diag} — 그리퍼에 가렸거나 화면 가장자리에 걸침. 위치를 바꿔 주세요"
                      f" (그 순간 사진: {miss.relative_to(ROOT)})")
                continue
            r0, c0 = cell(m0["pixel"])
            print(f"  보임: 픽셀 {np.round(m0['pixel'], 0)} ({['위', '가운데', '아래'][r0]}-{['왼쪽', '가운데', '오른쪽'][c0]} 칸)")

            ask("  팔을 받치고 Enter → 토크 OFF > ")
            arm.torque_off()
            while True:
                ask("  손끝(고정 집게 끝)을 블록 윗면 '중앙'에 살짝 대고(밀지 말고) Enter > ")
                q_touch = arm.read_deg()
                tip, _ = kin.fk([q_touch[j] for j in ARM])
                dz = tip[2] - (M.surface_z(*tip[:2]) + M.BLOCK_M)
                if abs(dz) <= TOUCH_TOL_M:
                    break
                print(f"  [다시] 손끝이 블록 윗면보다 {dz*100:+.1f} cm — 윗면에 닿게 다시 대 주세요")
            f, l = M.to_robot(*tip[:2])
            print(f"  손끝 위치: 팔 ({tip[0]*100:.1f}, {tip[1]*100:.1f}) cm = 터틀봇 기준 앞 {f*100:.1f}, 왼 {l*100:+.1f} cm"
                  f" (블록 윗면과 높이 차 {dz*1000:+.0f} mm)")
            while True:
                ask("  팔을 블록 위 3~5cm로 들고, 그리퍼를 대충 수직으로 받친 채 Enter (토크 ON) > ")
                q = arm.read_deg()
                _, R = kin.fk([q[j] for j in ARM])
                if (-R[:, 2])[2] >= LIFT_DOWN_MIN:
                    break
                print(f"  [다시] 그리퍼가 기울어 있음 (아래 방향 {(-R[:, 2])[2]:.2f}, {LIFT_DOWN_MIN} 이상으로)")
            arm.torque_on()
            time.sleep(0.3)
            ask("  자세가 유지되면 손을 떼고 Enter (로봇이 위로 빠져 보는 자세로 감 — 블록은 건드리지 말기) > ")
            back_to_look(arm, kin, limits, look_q, p_look)
            q_look_act = arm.read_deg()
            time.sleep(0.4)
            img = log_path.with_name(f"{log_path.stem}_p{n+1}.png")
            m1, diag = measure(cam, args.color, save=img)
            if m1 is None:
                print(f"  [카메라] 돌아와서 못 찾음 {diag} — 이 점은 버림")
                continue
            moved = float(np.linalg.norm(np.subtract(m1["pixel"], m0["pixel"])))
            if moved > MOVED_WARN_PX:
                print(f"  [주의] 짚는 동안 블록이 화면에서 {moved:.0f}px 움직임 — 짚을 때 밀린 듯. "
                      "이 점은 빼고 같은 위치에서 다시 하는 게 좋음")
                if ask("  k = 그래도 저장, Enter = 버림 > ") != "k":
                    continue
            n += 1
            k = cell(m1["pixel"])
            cells[k] = cells.get(k, 0) + 1
            log.append({"index": n, "mode": "touch", "robot_fl_m": [f, l], "target_xy_m": tip[:2].tolist(),
                        "q_touch_deg": q_touch, "tip_xy_m": tip[:2].tolist(), "tip_z_m": float(tip[2]),
                        "touch_dz_m": float(dz), "q_look_deg": q_look_act, "camera": {"pixel": m1["pixel"]},
                        "camera_before": {"pixel": m0["pixel"]}, "moved_px": moved, "camera_diag": diag,
                        "color": args.color, "surface_z_m": M.surface_z(*tip[:2]), "time": time.time()})
            log_path.parent.mkdir(parents=True, exist_ok=True)
            log_path.write_text("".join(json.dumps(r, ensure_ascii=False) + "\n" for r in log), encoding="utf-8")
            print(f"  저장: 픽셀 {np.round(m1['pixel'], 1)} → 지금까지 {n}점 (12점 이상 권장)")

        print(f"\n[완료] {len(log)}점 → {log_path}")
        if ask("팔이 처음 받쳤던 자세로 돌아갈까요? Enter = 예, k = 그대로 > ") != "k":
            res = arm.move_deg({j: start_q[j] for j in ARM}, result_tol=40)
            print(f"[복귀] {res.summary()}")
    except ArmStop as e:
        print(f"\n[STOP] {e} — 현재 위치에서 정지(유지 중)")
    finally:
        if cam is not None:
            cam.stop()
        if log:
            print(f"[LOG] {log_path}\n  다음: python steps/m3_solve_camera.py {log_path.relative_to(ROOT)} --save")
        try:
            torque_on = any(arm._read("Torque_Enable").values())
        except Exception as e:
            print(f"[WARN] 토크 상태를 읽지 못함({e}) — USB 연결 확인")
            return
        if torque_on and not any(arm.torque_was.values()):
            try:
                ask("\n팔을 받치고 Enter → 토크 OFF > ")
            except (KeyboardInterrupt, EOFError):
                print("\n[주의] 입력 없이 종료 — 토크를 끕니다")
            arm.close(disable_torque=True)
        else:
            arm.close(disable_torque=False)


if __name__ == "__main__":
    main()
