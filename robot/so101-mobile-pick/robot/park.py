"""접힌 자세(park) ↔ 보는 자세 자동 이동 — 시작할 때마다 팔을 손으로 옮기지 않도록 (11/25).

  park: 전원을 끄면 팔이 내려앉는 접힌 자세. steps/m5_teach_park.py로 한 번 저장 (calibration/park_pose.json).
  저장값이 측정 범위 끝(어깨 -103°처럼)에 붙어 있으면 목표로는 끝에서 PARK_MARGIN_DEG 안쪽 값을 씀
  → 끝낼 때 그 자세에서 토크를 끄면 몇 도 내려앉아 원래 접힌 자세가 됨.

경로: 관절 공간 직선 보간(관절을 동시에 조금씩). 손끝이 터틀봇 윗판·작업 상자에 닿지 않는지 FK로 미리 검사하고,
안 되면 '들어 올린 중간 자세'를 거쳐 감. 이동 중 막히면 arm.move_deg의 안전 정지(부하·막힘 감시)가 그대로 동작.
"""
import json
from pathlib import Path

import numpy as np

from robot import mount as M
from robot.arm import ARM, ArmStop
from robot.motion import PASS_TOL, TRAVEL_TOL

ROOT = Path(__file__).resolve().parents[1]
PARK_FILE = ROOT / "calibration/park_pose.json"
PARK_MARGIN_DEG = 6.0        # arm.TARGET_MARGIN_DEG(5°)보다 조금 더 안쪽
NEAR_PARK_DEG = 8.0          # 지금 자세가 저장한 park와 모든 관절이 이 안이면 '접힌 자세에서 시작'
BOX_CLEAR_M = 0.03           # 작업 상자 위에서는 손끝이 상자 윗면보다 이만큼 위
BODY_CLEAR_M = 0.005         # 터틀봇 윗판 위(팬 축 앞 5cm 안)에서는 팔 바닥면 높이보다 이만큼 위
BODY_FWD_M = 0.05
SAMPLES = 25
STEP_DEG = 15.0              # 경유점 간격 (관절 최대 변화량 기준)

# 들어 올린 중간 자세 후보 (어깨·팔꿈치만 정하고 나머지는 보간): 팔을 위로 세워 손끝을 높이 둠
MID_CANDIDATES = [{"shoulder_lift": 0.0, "elbow_flex": 60.0}, {"shoulder_lift": -30.0, "elbow_flex": 80.0},
                  {"shoulder_lift": 20.0, "elbow_flex": 30.0}]


def load_park():
    if not PARK_FILE.is_file():
        return None
    q = json.loads(PARK_FILE.read_text(encoding="utf-8"))["q_deg"]
    return {j: float(q[j]) for j in ARM}


def park_target(arm, park):
    """측정 범위 끝에서 PARK_MARGIN_DEG 안쪽으로 당긴 park 목표."""
    out = {}
    for j in ARM:
        lo, hi = arm.range_deg(j)
        out[j] = min(max(park[j], lo + PARK_MARGIN_DEG), hi - PARK_MARGIN_DEG) if j != "wrist_roll" else park[j]
    return out


def near_park(q, park):
    return park is not None and all(abs(q[j] - park[j]) <= NEAR_PARK_DEG for j in ARM if j != "wrist_roll")


def _clear(kin, q):
    p, _ = kin.fk([q[j] for j in ARM])
    fwd, _ = M.to_robot(p[0], p[1])
    if fwd > BODY_FWD_M:
        return p[2] >= M.surface_z(p[0], p[1]) + BOX_CLEAR_M
    return p[2] >= -0.0024 + BODY_CLEAR_M


def _seg_ok(kin, a, b):
    return all(_clear(kin, {j: a[j] + (b[j] - a[j]) * t for j in ARM}) for t in np.linspace(0, 1, SAMPLES))


def _waypoints(a, b):
    n = max(1, int(np.ceil(max(abs(b[j] - a[j]) for j in ARM) / STEP_DEG)))
    return [{j: a[j] + (b[j] - a[j]) * k / n for j in ARM} for k in range(1, n + 1)]


def _seg_ok_ends(kin, a, b):
    """구간 검사. 끝점 자체가 낮으면(접힌 자세) 그 끝 20%는 검사에서 뺌 — 원래 낮은 자세에서 빠져나가거나 들어가는 부분."""
    t0 = 0.2 if not _clear(kin, a) else 0.0
    t1 = 0.8 if not _clear(kin, b) else 1.0
    return all(_clear(kin, {j: a[j] + (b[j] - a[j]) * t for j in ARM}) for t in np.linspace(t0, t1, SAMPLES))


def plan(kin, q_from, q_to):
    """관절 공간 경로(경유점 목록). 직접 가면 부딪힐 것 같으면 들어 올린 중간 자세를 거침. 안 되면 ArmStop."""
    q_from = {j: float(q_from[j]) for j in ARM}
    q_to = {j: float(q_to[j]) for j in ARM}
    if _seg_ok_ends(kin, q_from, q_to):
        return _waypoints(q_from, q_to)
    for mid in MID_CANDIDATES:
        m = {j: (q_from[j] + q_to[j]) / 2 for j in ARM}
        m.update(mid)
        if _clear(kin, m) and _seg_ok_ends(kin, q_from, m) and _seg_ok_ends(kin, m, q_to):
            return _waypoints(q_from, m) + _waypoints(m, q_to)
    raise ArmStop("접힌 자세 ↔ 보는 자세 사이에 부딪히지 않는 경로를 못 찾음 — 팔을 손으로 옮겨 시작")


def go_joint(arm, wps, last_tol=None):
    res = None
    for i, q in enumerate(wps):
        last = i == len(wps) - 1
        res = arm.move_deg(q, correct=last, result_tol=last_tol if last else TRAVEL_TOL,
                           pass_tol=None if last else PASS_TOL)
        if not res.status.startswith("도착"):
            raise ArmStop(res.status)
    return res
