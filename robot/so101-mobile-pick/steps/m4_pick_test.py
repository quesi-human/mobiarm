"""4단계: 손목 카메라로 블록을 찾아 집고 → 들어 올렸다 → 같은 자리에 내려놓고 → 보는 자세로. 반복 (터틀봇은 정지).

  python steps/m4_pick_test.py --color yellow

한 번에:
  1) 보는 자세에서 사진 → 블록 위치(팔 좌표)와 방향 → 팬만 돌려 블록을 화면 가운데로 오게 해서 다시 찍기
  2) 블록 위 8cm → 그리퍼 열고 수직으로 내려가 → 쥐기 → 들어 올리기 (잡았는지 출력)
  3) Enter → 같은 자리에 내려놓고 위로 빠져 보는 자세로
  4) 다시 찍어서 '놓은 자리(팔이 안다)'와 '카메라가 본 자리'의 차이를 출력 → 카메라 보정 확인용
결과: steps/logs/m4_<시각>.jsonl, 사진 steps/logs/m4_<시각>_c<번호>.png
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot import devices as DEV  # noqa: E402
from robot.arm import ARM, Arm, ArmStop  # noqa: E402
from robot.kinematics import SO101Kinematics  # noqa: E402
from robot.motion import go  # noqa: E402
from robot import mount as M  # noqa: E402
from robot import wristcam as WC  # noqa: E402
from robot import park as P  # noqa: E402
from robot.pick import (PICK_CLEAR_M, PICK_RAISE_M, OPEN_PICK_PCT,  # noqa: E402
                        START_MARGIN_DEG, line)
from robot.pickflow import go_down, plan_pick, unwind_roll  # noqa: E402
from vision.camera import COLOR_RANGES, UsbCamera  # noqa: E402

CLOSED_PCT = 3
RELEASE_PCT = 25           # 11/24: 놓을 때 60%로 크게 벌리면 빠지면서 블록을 쳐서 밀림 → 조금만 벌리고 수직으로 빠짐
PLACE_ABOVE_M = 0.0        # 내려놓을 때 쥔 높이에서 더 올릴 양 (쥔 높이 자체가 이미 1cm 위)
KO = {"ㅂ": "q", "ㅏ": "k"}
def ask(prompt):
    a = input(prompt).strip().lower()
    return KO.get(a[:1], a[:1]) if a else ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--color", default="yellow", choices=sorted(COLOR_RANGES))
    ap.add_argument("--port", default=DEV.ARM_PORT, help="팔 USB (기본: 장치 고유 이름, 없으면 /dev/ttyACM0)")
    ap.add_argument("--cam", default=DEV.WRIST_CAM, help="손목 카메라 (기본: 장치 고유 이름, 없으면 /dev/video0)")
    ap.add_argument("--clear-mm", type=float, default=PICK_CLEAR_M * 1000)
    ap.add_argument("--open-pct", type=float, default=OPEN_PICK_PCT)
    ap.add_argument("--raise-mm", type=float, default=PICK_RAISE_M * 1000, help="블록 중심보다 이만큼 위에서 쥐고 놓기")
    args = ap.parse_args()

    cal = WC.load_calib()
    look_q = WC.load_look_q()
    G = M.grasp_point()
    kin = SO101Kinematics()
    kin_g = SO101Kinematics(contact=G)
    kin_pick = SO101Kinematics(contact=G + np.array([-args.clear_mm / 1000, 0, 0]))
    print(f"[보정] {cal.get('model', 'homography')}, 검증 오차 평균 {cal['loo_mean_mm']:.1f} mm ({cal['created']})")

    arm = Arm(port=args.port).connect()
    limits = {m: arm.range_deg(m) for m in ARM}
    cam = None
    stamp = time.strftime("%Y%m%d_%H%M%S")
    log, log_path = [], ROOT / f"steps/logs/m4_{stamp}.jsonl"
    parked = False
    try:
        prob = arm.check_targets_deg(look_q)
        if prob:
            raise ArmStop(f"보는 자세가 관절 범위 끝: {prob}")
        cam = UsbCamera(args.cam).start()

        park = P.load_park()
        now = arm.read_deg()
        auto = P.near_park(now, park)
        if auto:
            # 11/25: 접힌 자세에서 시작하면 손으로 옮길 필요 없이 스스로 폄 (m5_teach_park.py로 저장한 자세)
            wps = P.plan(kin, now, look_q)
            if ask("접힌 자세에서 시작 — Enter = 팔을 펴서 보는 자세로 (팔 주변을 비워 주세요), q = 끝 > ") == "q":
                return
            arm.torque_on()
            arm.gripper_torque_on()
            time.sleep(0.3)
            arm.gripper_open(CLOSED_PCT)
            P.go_joint(arm, wps, last_tol=WC.LOOK_TOL_COUNTS)
        else:
            if park is None:
                print("(steps/m5_teach_park.py로 접힌 자세를 한 번 저장하면, 다음부터 손으로 옮기지 않아도 됨)")
            else:
                print("(지금 자세가 저장한 접힌 자세와 달라서 손으로 시작합니다)")
            print("팔을 손으로 '보는 자세' 근처(상자 위, 터틀봇 정면)로 옮겨 받쳐 주세요.")
        while not auto:
            if ask("받친 채 Enter (토크 ON), q = 끝 > ") == "q":
                return
            now = arm.read_deg()
            bad = [f"{j}={now[j]:.1f}°" for j in ("shoulder_lift", "elbow_flex", "wrist_flex")
                   if not (limits[j][0] + START_MARGIN_DEG < now[j] < limits[j][1] - START_MARGIN_DEG)]
            if not bad:
                break
            print(f"  [다시] 아직 측정 범위 밖 {bad} — 팔을 접힌 자세에서 더 펴서 상자 위로 들어 주세요")
        if not auto:
            arm.torque_on()
            arm.gripper_torque_on()
            time.sleep(0.3)
            ask("자세가 유지되면 손을 떼고 Enter (그리퍼를 닫고 보는 자세로 감) > ")
            unwind_roll(arm, look_q["wrist_roll"])
            arm.gripper_open(CLOSED_PCT)
            WC.go_look(arm, kin, limits, look_q)

        cycle = 0
        placed = None             # 직전에 내려놓은 자리 (카메라 확인용)
        placed_yaw = 0.0
        while True:
            a = ask(f"\n[{cycle+1}] '{args.color}' 블록을 상자 위 아무 데나(팔이 닿는 곳) 놓고 Enter (q = 끝) > " if placed is None
                    else f"\n[{cycle+1}] 내려놓은 블록을 다시 집으려면 Enter, 옮기고 싶으면 옮긴 뒤 Enter (q = 끝) > ")
            if a == "q":
                if park is not None:
                    print("  - 접힌 자세로")
                    WC.go_look(arm, kin, limits, look_q)
                    P.go_joint(arm, P.plan(kin, arm.read_deg(), P.park_target(arm, park)))
                    parked = True
                break
            cycle += 1
            t0 = time.monotonic()
            img = log_path.with_name(f"{log_path.stem}_c{cycle}.png")
            b, q_seen, phi0, diag = WC.search(arm, cam, cal, look_q, args.color, save_image=img)
            if b is not None and abs(phi0) >= 1.0:          # 11/25: 정면(-0.1°)인데 '오른쪽 0°에서 찾음'으로 나옴
                side = "왼쪽" if phi0 > 0 else "오른쪽"
                print(f"  [둘러보기] 정면에선 안 보여서 {side} {abs(phi0):.0f}°에서 찾음")
            if b is None:
                WC.go_look(arm, kin, limits, look_q)
                cnt = diag.get("detections_per_frame", {})
                why = "블록이 없거나 화면 가장자리·그리퍼에 걸침" if cnt.get(0) else (diag.get("error") or cnt)
                print(f"  [카메라] 좌우 {WC.SCAN_SPAN_DEG:.0f}°까지 둘러봤지만 못 찾음: {why} (사진 {img.relative_to(ROOT)}, _pan±N.png)")
                cycle -= 1
                continue
            x, y = b["xy_m"]
            f, l = M.to_robot(x, y)
            yaw_s = "?" if b["yaw_deg"] is None else f"{b['yaw_deg']:.0f}°"
            print(f"  블록: 팔 ({x*100:.1f}, {y*100:.1f}) cm = 터틀봇 앞 {f*100:.1f}, 왼 {l*100:+.1f} cm, 방향 {yaw_s}, "
                  f"윗면 크기 {diag.get('top_size_mm')} mm")
            rec = {"cycle": cycle, "color": args.color, "xy_m": [x, y], "yaw_deg": b["yaw_deg"], "pixel": b["pixel"],
                   "diag": {k: v for k, v in diag.items() if k != "detections_per_frame"}}
            if placed is not None:
                err = (np.array([x, y]) - placed) * 1000
                rec["placed_xy_m"], rec["camera_err_mm"] = placed.tolist(), err.tolist()
                t = np.radians(placed_yaw)
                along = err @ np.array([np.cos(t), np.sin(t)])     # + = 고정 집게 쪽
                rec["camera_err_fixed_jaw_mm"] = float(along)
                print(f"  [확인] 놓은 자리와 카메라 값 차이: ({err[0]:+.1f}, {err[1]:+.1f}) mm, 거리 {np.linalg.norm(err):.1f} mm"
                      f" — 그중 고정 집게 방향 {along:+.1f} mm (+면 카메라가 블록을 고정 집게 쪽으로 더 멀리 봄)")
            placed = None

            # 블록을 화면 가운데로 오게 팬만 돌려 다시 찍기 (화면 가장자리는 보정 오차가 큼)
            b2, q_view, phi = WC.relook(arm, cam, cal, look_q, b, args.color,
                                        save_image=img.with_name(img.stem + "_relook.png"))
            if b2 is None:
                print(f"  [다시 보기] 팬 {phi:+.0f}° 돌렸더니 블록을 못 찾음 → 처음 값으로 집음")
                q_view = arm.read_deg()
            elif phi:
                d = (np.array(b2["xy_m"]) - np.array([x, y])) * 1000
                print(f"  [다시 보기] 팬 {phi:+.0f}° 돌려 다시 찍음 → 위치 보정 ({d[0]:+.0f}, {d[1]:+.0f}) mm")
                x, y = b2["xy_m"]
                b = b2
                rec.update({"relook_phi_deg": phi, "relook_xy_m": [x, y], "relook_pixel": b2["pixel"]})
            plan = plan_pick(kin_g, kin_pick, limits, q_view, (x, y), b["yaw_deg"], args.raise_mm / 1000)
            if isinstance(plan, str):
                print(f"  [건너뜀] {plan}")
                rec["skip"] = plan
                log.append(rec)
                log_path.parent.mkdir(parents=True, exist_ok=True)
                log_path.write_text("".join(json.dumps(r, ensure_ascii=False) + "\n" for r in log), encoding="utf-8")
                WC.go_look(arm, kin, limits, look_q)
                continue
            print(f"  - 블록 위로 (그리퍼 방향 {plan['yaw']:.0f}°)")
            go(arm, plan["to_above"], precise=False)
            arm.gripper_open(args.open_pct)
            print("  - 내려가 집기")
            go_down(arm, plan["down"])
            held, pct = arm.grip()
            go(arm, plan["up"], precise=False)
            rec.update({"held": held, "gripper_pct": pct, "pick_yaw_deg": plan["yaw"]})
            if not held:
                print(f"  [실패] 못 잡음 (그리퍼 {pct:.0f}%) — 사진 {img.relative_to(ROOT)}")
                arm.gripper_open(CLOSED_PCT)
            else:
                print(f"  [성공] 집음 (그리퍼 {pct:.0f}%, 블록 25mm면 약 10~14%) — {time.monotonic()-t0:.1f}s")
                ask("  Enter → 같은 자리에 내려놓기 > ")
                z_place = plan["z_c"] + PLACE_ABOVE_M
                q_ref = arm.read_deg()
                wps, q_ref = line(kin_g, limits, plan["above"], (x, y, z_place), q_ref, plan["yaw"])
                go_down(arm, wps)
                arm.gripper_open(RELEASE_PCT)
                time.sleep(0.3)
                q_now = arm.read_deg()
                gp, _ = kin_g.fk([q_now[j] for j in ARM])
                wps, _ = line(kin_g, limits, (x, y, z_place), plan["above"], q_now, plan["yaw"])
                go(arm, wps, precise=False)
                arm.gripper_open(CLOSED_PCT)
                # 블록 중앙 = 쥔 점(손가락 사이 가운데)의 XY. 카메라 기준(윗면 중앙)과 같은 수평 위치
                placed = gp[:2].copy()
                placed_yaw = plan["yaw"]
            print("  - 보는 자세로")
            WC.go_look(arm, kin, limits, look_q)
            rec["seconds"] = round(time.monotonic() - t0, 1)
            log.append(rec)
            log_path.parent.mkdir(parents=True, exist_ok=True)
            log_path.write_text("".join(json.dumps(r, ensure_ascii=False) + "\n" for r in log), encoding="utf-8")
    except ArmStop as e:
        print(f"\n[STOP] {e} — 현재 위치에서 정지(유지 중)")
    except KeyboardInterrupt:
        print("\n[중단] Ctrl+C")
        try:
            arm.hold()
        except Exception:
            pass
    finally:
        if cam is not None:
            cam.stop()
        if log:
            n_ok = sum(1 for r in log if r.get("held"))
            n_try = sum(1 for r in log if "held" in r)
            errs = [np.linalg.norm(r["camera_err_mm"]) for r in log if "camera_err_mm" in r]
            print(f"\n[결과] 집기 {n_ok}/{n_try}" + (f", 카메라 확인 오차 평균 {np.mean(errs):.1f} mm (최대 {np.max(errs):.1f})"
                                                if errs else ""))
            print(f"[LOG] {log_path}")
        try:
            torque_on = any(arm._read("Torque_Enable").values())
        except Exception as e:
            print(f"[WARN] 토크 상태를 읽지 못함({e}) — USB 연결 확인")
            return
        if torque_on and parked:
            print("\n접힌 자세 — 토크 OFF (몇 도 내려앉을 수 있음)")
            arm.close(disable_torque=True)
        elif torque_on:
            try:
                ask("\n팔을 받치고 Enter → 토크 OFF > ")
            except (KeyboardInterrupt, EOFError):
                print("\n[주의] 입력 없이 종료 — 토크를 끕니다")
            arm.close(disable_torque=True)
        else:
            arm.close(disable_torque=False)


if __name__ == "__main__":
    main()
