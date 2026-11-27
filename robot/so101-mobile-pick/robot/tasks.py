"""팔 작업 묶음 (팔 서버가 씀): 준비 → 찾아 집기 → 운반 자세 → 내려놓기 → 접힌 자세.

자세 4개 (관절값, calibration/*.json):
  park   접힌 자세 — 쉬거나 터틀봇이 빈손으로 이동할 때 (m5_teach_park.py)
  look   보는 자세 — 둘러보기 (m2)
  carry  운반 자세 — 블록을 든 채 터틀봇이 이동할 때. 라이다를 가리지 않게 (m6_teach_pose.py carry)
  place  내려놓는 자세 — 목적지에 도착한 뒤 이 자세에서 그리퍼를 열어 떨어뜨림 (m6_teach_pose.py place)
자세 사이 이동은 park.plan(관절 보간 + 상자·터틀봇 윗판 충돌 검사).

stop: 다른 스레드에서 request_stop() → 다음 단계 사이에서 멈춤(그 자리 유지). 한 동작(수백 ms~2초) 중간에는 못 끊음.
"""
import json
import threading
import time
from pathlib import Path

import numpy as np

from robot import mount as M
from robot import park as P
from robot import wristcam as WC
from robot.arm import ARM, Arm, ArmStop
from robot.kinematics import SO101Kinematics
from robot.motion import PASS_TOL, TRAVEL_TOL, go
from robot.pick import OPEN_PICK_PCT, PICK_CLEAR_M, PICK_RAISE_M
from robot.pickflow import go_down, plan_pick, unwind_roll
from vision.camera import UsbCamera

ROOT = Path(__file__).resolve().parents[1]
POSE_FILES = {"carry": ROOT / "calibration/carry_pose.json", "place": ROOT / "calibration/place_pose.json"}
LOGS = ROOT / "steps/logs"
CLOSED_PCT = 3
DROP_OPEN_PCT = 45         # 내려놓을 때 벌림 (떨어뜨리기)
HELD_MAX_PCT = 25          # 운반 중 그리퍼가 이보다 닫혀 있으면(블록 25mm ≈ 12%) 쥐고 있는 것


class Stopped(ArmStop):
    pass


def load_pose(name):
    f = POSE_FILES[name]
    if not f.is_file():
        return None
    q = json.loads(f.read_text(encoding="utf-8"))["q_deg"]
    return {j: float(q[j]) for j in ARM}


class ArmTasks:
    def __init__(self, port, cam):
        self.cal = WC.load_calib()
        self.look_q = WC.load_look_q()
        self.park_q = P.load_park()
        G = M.grasp_point()
        self.kin = SO101Kinematics()
        self.kin_g = SO101Kinematics(contact=G)
        self.kin_pick = SO101Kinematics(contact=G + np.array([-PICK_CLEAR_M, 0, 0]))
        print(f"[준비] 팔 연결 중 ({port})", flush=True)
        self.arm = Arm(port=port).connect()
        self.limits = {m: self.arm.range_deg(m) for m in ARM}
        self.cam_path = cam
        self.cam = None                 # 집을 때만 엶 (place·park는 카메라가 필요 없음 → 더 빨리 시작)
        print("[준비] 완료", flush=True)
        self.holding = False
        self.pose = "unknown"
        self._stop = threading.Event()
        self.deadline = None            # time.monotonic() 기준. 넘으면 다음 단계 사이에서 멈춤 (워커 제한 시간)
        self.stage_cb = lambda s: None

    # ------------------------------------------------------------ 공통
    def request_stop(self):
        self._stop.set()

    def _check(self, stage=None):
        if self._stop.is_set():
            self._stop.clear()
            self.halt()
            raise Stopped("중지 요청")
        if self.deadline is not None and time.monotonic() > self.deadline:
            self.halt()
            raise Stopped("제한 시간 초과")
        if stage:
            self.stage_cb(stage)

    def halt(self):
        """그 자리 정지 — 팔 관절만. 그리퍼는 건드리지 않음 (arm.hold()는 그리퍼 목표도 현재값으로 바꿔 쥔 힘이 풀림)."""
        try:
            pos = self.arm.read_raw()
            for m in ARM:
                self.arm.bus.write("Goal_Position", m, pos[m], num_retry=2)
        except Exception as e:
            print(f"[정지] 목표 고정 실패: {e}", flush=True)

    def _torque_is_on(self):
        return any(self.arm._read("Torque_Enable").values())

    def _goto_joint(self, q_to, name, last_tol=None):
        self._check(f"{name} 자세로")
        wps = P.plan(self.kin, self.arm.read_deg(), q_to)
        for i, q in enumerate(wps):
            self._check()
            last = i == len(wps) - 1
            res = self.arm.move_deg(q, correct=last, result_tol=last_tol if last else TRAVEL_TOL,
                                    pass_tol=None if last else PASS_TOL)
            if not res.status.startswith("도착"):
                raise ArmStop(res.status)
        self.pose = name

    def ready(self):
        """토크 켜고 보는 자세까지. 토크가 꺼져 있으면 지금 자세에서 그대로 토크를 켜고(제자리 유지) 충돌 검사한 경로로 폄."""
        self._stop.clear()
        if not self._torque_is_on():
            now = self.arm.read_deg()
            if not P.near_park(now, self.park_q):
                # 11/26: 매번 손으로 접힌 자세에 두지 않아도 되게 — 지금 자세에서 토크를 켜고(그 자리 유지) 충돌 검사한 경로로 펌.
                # 경로를 못 찾으면(상자에 걸린 자세 등) P.plan이 ArmStop을 냄.
                print("[준비] 접힌 자세가 아님 — 지금 자세에서 바로 보는 자세로 갑니다 "
                      + ", ".join(f"{j} {now[j]:+.0f}°" for j in ARM), flush=True)
                P.plan(self.kin, now, self.look_q)        # 움직이기 전에 경로부터 확인
            self.arm.torque_on()
            self.arm.gripper_torque_on()
            time.sleep(0.3)
            self.arm.gripper_open(CLOSED_PCT)
            self.pose = "park"
        if self.pose != "look":
            self._goto_joint(self.look_q, "look", last_tol=WC.LOOK_TOL_COUNTS)

    # ------------------------------------------------------------ 찾아 집기
    def pick(self, color):
        """반환 dict: result = picked / not_found / too_far / missed. too_far면 블록 위치(터틀봇 기준) 포함."""
        if self.holding:
            raise ArmStop("이미 블록을 들고 있음 — 먼저 place")
        if self.cam is None:             # 11/26: 팔을 움직이기 전에 카메라부터 (카메라가 안 되면 팔은 그대로)
            print(f"[준비] 손목 카메라 여는 중 ({self.cam_path})", flush=True)
            self.cam = UsbCamera(self.cam_path).start()
            print("[준비] 카메라 열림", flush=True)
        self.ready()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        img = LOGS / f"task_{stamp}_pick.png"
        LOGS.mkdir(parents=True, exist_ok=True)
        self._check("둘러보기")
        b, _, phi0, diag = WC.search(self.arm, self.cam, self.cal, self.look_q, color, save_image=img)
        self.pose = "look"
        if b is None:
            self._goto_joint(self.look_q, "look", last_tol=WC.LOOK_TOL_COUNTS)
            return {"result": "not_found", "scan_deg": WC.SCAN_SPAN_DEG}
        self._check("화면 가운데로 다시 보기")
        b2, q_view, _ = WC.relook(self.arm, self.cam, self.cal, self.look_q, b, color,
                                  save_image=img.with_name(img.stem + "_relook.png"))
        if b2 is not None:
            b = b2
        else:
            q_view = self.arm.read_deg()
        x, y = b["xy_m"]
        info = self._where(x, y)
        plan = plan_pick(self.kin_g, self.kin_pick, self.limits, q_view, (x, y), b["yaw_deg"], PICK_RAISE_M)
        if isinstance(plan, str):
            self._goto_joint(self.look_q, "look", last_tol=WC.LOOK_TOL_COUNTS)
            return {"result": "too_far", "reason": plan, **info}
        self._check("블록 위로")
        go(self.arm, plan["to_above"], precise=False)
        self.arm.gripper_open(OPEN_PICK_PCT)
        self._check("내려가 집기")
        go_down(self.arm, plan["down"])
        held, pct = self.arm.grip()
        go(self.arm, plan["up"], precise=False)
        self.pose = "above"
        if not held:
            self.arm.gripper_open(CLOSED_PCT)
            WC.go_look(self.arm, self.kin, self.limits, self.look_q)
            self.pose = "look"
            return {"result": "missed", "gripper_pct": round(pct, 1), **info}
        self.holding = True
        WC.go_look(self.arm, self.kin, self.limits, self.look_q)
        self.pose = "look"
        unwind_roll(self.arm, self.look_q["wrist_roll"])
        carry = load_pose("carry") or self.look_q
        self._goto_joint(carry, "carry")
        return {"result": "picked", "gripper_pct": round(pct, 1), **info}

    def _where(self, x, y):
        fwd, left = M.to_robot(x, y)
        out = {"arm_xy_m": [round(x, 4), round(y, 4)], "fwd_m": round(fwd, 4), "left_m": round(left, 4),
               "dist_m": round(float(np.hypot(fwd, left)), 4),
               "bearing_deg": round(float(np.degrees(np.arctan2(left, fwd))), 1)}
        off = M._mount().get("pan_axis_in_base_m")      # 터틀봇 중심 → 팔 회전축 (앞, 왼) m, 재서 넣으면 함께 줌
        if off:
            bf, bl = fwd + off[0], left + off[1]
            out.update({"base_fwd_m": round(bf, 4), "base_left_m": round(bl, 4),
                        "base_bearing_deg": round(float(np.degrees(np.arctan2(bl, bf))), 1)})
        return out

    # ------------------------------------------------------------ 내려놓기·접기
    def place(self):
        if not self.holding:
            raise ArmStop("들고 있는 블록이 없음")
        place_q = load_pose("place")
        if place_q is None:
            raise ArmStop("내려놓는 자세가 없음 — python steps/m6_teach_pose.py place")
        self._stop.clear()
        self._goto_joint(place_q, "place")
        self._check("그리퍼 열기")
        self.arm.gripper_open(DROP_OPEN_PCT)
        time.sleep(0.5)
        self.holding = False
        self.arm.gripper_open(CLOSED_PCT)
        self.park()
        return {"result": "placed"}

    def park(self):
        self._stop.clear()
        if self.park_q is None:
            raise ArmStop("접힌 자세가 없음 — python steps/m5_teach_park.py")
        if not self._torque_is_on():
            return {"result": "parked", "note": "토크 꺼져 있음"}
        if self.pose in ("above", "unknown"):
            WC.go_look(self.arm, self.kin, self.limits, self.look_q)
        self._goto_joint(P.park_target(self.arm, self.park_q), "park")
        return {"result": "parked"}

    def gripper_pct(self):
        try:
            return round(self.arm.gripper_open_pct(), 1)
        except Exception:
            return None

    def shutdown(self):
        try:
            if self._torque_is_on():
                if not self.holding:
                    try:
                        self.park()
                    except Exception as e:
                        print(f"[종료] 접힌 자세로 못 감: {e}")
                    self.arm.close(disable_torque=True)
                else:
                    print("[종료] 블록을 들고 있어 토크를 켠 채 둡니다 — 팔을 받치고 전원을 끄거나 다시 실행해 place")
                    self.arm.close(disable_torque=False)
            else:
                self.arm.close(disable_torque=False)
        finally:
            if self.cam is not None:
                self.cam.stop()

    def detach(self):
        """움직이지 않고 연결만 끊음 — 토크는 지금 상태 그대로 (블록을 든 채 다음 명령을 기다릴 때)."""
        try:
            self.arm.close(disable_torque=False)
        finally:
            if self.cam is not None:
                self.cam.stop()
