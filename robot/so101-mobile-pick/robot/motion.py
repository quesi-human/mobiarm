"""경로 계획 + 실행 도우미 (robot/motion.py)

- ik_roll    : 위에서-집기 IK, roll을 '기본(pan에 따라 정해지는 방향) + roll_off'로 지정
- plan_line  : 직선 경로를 STEP 간격 경유점으로 나눠 IK (roll도 선형 보간)
- go         : 경유점 이동 — 중간은 보정 없이, 마지막만 도착 보정
"""
import math

import numpy as np

from robot.arm import ArmStop

STEP_M = 0.02


def ik_roll(kin, p, roll_off_deg, limits, q_ref):
    # 가지(팔꿈치) 선택만 직전 자세를 따르고, roll은 요청값에 가까운 쪽(±180° 대칭 중)을 고르게 함
    if q_ref is not None:
        q_ref = {k: v for k, v in q_ref.items() if k != "wrist_roll"}
    r0 = kin.ik_topdown(*p, limits=limits, q_ref=q_ref)
    if not r0.ok or abs(roll_off_deg) < 1e-6:
        return r0
    R = kin.fk(list(r0.q.values()))[1]
    psi0 = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    return kin.ik_topdown(*p, yaw_deg=psi0 + roll_off_deg, limits=limits, q_ref=q_ref, allow_flip=False)


def plan_line(kin, limits, p_from, p_to, q_ref, roll_from=0.0, roll_to=None):
    roll_to = roll_from if roll_to is None else roll_to
    p_from, p_to = np.asarray(p_from, float), np.asarray(p_to, float)
    n = max(1, int(math.ceil(np.linalg.norm(p_to - p_from) / STEP_M)))
    wps = []
    for k in range(1, n + 1):
        a = k / n
        p = p_from + (p_to - p_from) * a
        r = ik_roll(kin, p, roll_from + (roll_to - roll_from) * a, limits, q_ref)
        if not r.ok:
            raise ArmStop(f"IK 실패 @ {np.round(p * 100, 1)} cm: {r.problems}")
        q_ref = r.q
        wps.append(r.q)
    return wps, q_ref


def plan_arc(kin, limits, p_from, p_to, q_ref, center=(0.0388, 0.0)):
    """팬 축 기준 원호 경로 (반경·각도·높이를 고르게 보간). 11/17: 직선은 반대편으로 갈 때 베이스 바로 앞을 지나 IK 실패.
    roll은 기본(≈0) — 수집처럼 그리퍼 방향이 상관없을 때."""
    a, b = np.asarray(p_from, float), np.asarray(p_to, float)
    c = np.asarray(center, float)
    ra, rb = np.hypot(*(a[:2] - c)), np.hypot(*(b[:2] - c))
    ta, tb = math.atan2(a[1] - c[1], a[0] - c[0]), math.atan2(b[1] - c[1], b[0] - c[0])
    arc = abs(tb - ta) * max(ra, rb) + abs(rb - ra) + abs(b[2] - a[2])
    n = max(1, int(math.ceil(arc / STEP_M)))
    wps = []
    for k in range(1, n + 1):
        f = k / n
        rr, th = ra + (rb - ra) * f, ta + (tb - ta) * f
        p = np.array([c[0] + rr * math.cos(th), c[1] + rr * math.sin(th), a[2] + (b[2] - a[2]) * f])
        r = kin.ik_topdown(*p, limits=limits, q_ref=q_ref)
        if not r.ok:
            raise ArmStop(f"IK 실패 @ {np.round(p * 100, 1)} cm: {r.problems}")
        q_ref = r.q
        wps.append(r.q)
    return wps, q_ref


PASS_TOL = 40        # 중간 경유점은 이만큼 가까워지면 멈추지 않고 다음으로 (부드럽고 빠르게)
TRAVEL_TOL = 40      # 경유 이동(이동 높이·주차·복귀)의 허용 오차, 카운트 ≈3.5° — 정밀할 필요 없음 (11/16: 15카운트로 멈춤)


def go(arm, wps, precise=True):
    """precise=False: 경유 이동 — 마지막 도착도 TRAVEL_TOL까지 허용."""
    res = None
    for i, q in enumerate(wps):
        last = i == len(wps) - 1
        res = arm.move_deg(q, correct=last, result_tol=None if precise else TRAVEL_TOL,
                           pass_tol=None if last else PASS_TOL)
        if not res.status.startswith("도착"):
            raise ArmStop(res.status)
    return res
