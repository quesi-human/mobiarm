"""블록 집기용 공통 계산: 잡는 방향 고르기, 경로 (v1 robot/pick.py 그대로, 좌표 상수만 mount.py)
11/18: steps/s5_pick_test.py에서 옮김 — s6_pick_place.py가 테스트 스크립트에 의존하지 않도록.
"""
import math

import numpy as np

from robot import mount as W
from robot.arm import ArmStop

PARK_XY = (0.10, -0.14)
PARK_ABOVE_TABLE_M = 0.10
PICK_CLEAR_M = 0.012          # 내려갈 때 쥘 점을 블록 중심에서 고정 손가락 쪽으로 이만큼 비켜 내려감 = 고정 손가락과 블록 면 사이 여유
                              # (11/24: 쥐는 점(mount.GRASP_POINT_DEFAULT)을 실측으로 고친 뒤 12mm. 고치기 전 22mm ≈ 지금 8mm)
PICK_RAISE_M = 0.010          # 11/24 v2: 블록 중심 높이까지 내려가면 손가락 끝이 상자에 닿아 블록이 밀림 → 1cm 위에서 쥐고 놓음
OPEN_PICK_PCT = 48            # 그리퍼 1%≈1.3°. 48%≈62° (11/24: 여유를 늘려서 35→42→48) (블록 25mm는 12%에서 잡힘). 90%는 과했음
OPEN_RELEASE_PCT = 60
ROLL_MAX_DEG = 50.0          # 11/17: 손목을 + 방향으로 크게 돌리면 저항(케이블 추정)으로 과부하 보호에 걸려 기어감
ROLL_CHUNK_DEG = 20.0        # 시작 시 손목을 0°로 되돌릴 때 한 번에 돌리는 각도
START_MARGIN_DEG = 1.0       # 11/17: 10°는 접힌 휴식 자세(팔꿈치 ~91°/97°)도 거부해서 1°로. 범위 밖만 거부


def pick_yaw(kin, p, yaw_block, limits, q_ref):
    """큐브는 90° 대칭 → 블록 면에 맞는 4방향 중 손목 회전이 가장 작은 해."""
    best = None
    for k in range(4):
        r = kin.ik_topdown(*p, yaw_deg=yaw_block + 90 * k, limits=limits, q_ref=q_ref, allow_flip=False)
        if r.ok and abs(r.q["wrist_roll"]) <= ROLL_MAX_DEG and (best is None or abs(r.q["wrist_roll"]) < abs(best[1].q["wrist_roll"])):
            best = (yaw_block + 90 * k, r)
    return best


def line(kin, limits, a, b, q_ref, yaw):
    """yaw 고정 직선 경로."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(1, int(math.ceil(np.linalg.norm(b - a) / 0.02)))
    wps = []
    for k in range(1, n + 1):
        p = a + (b - a) * k / n
        r = kin.ik_topdown(*p, yaw_deg=yaw, limits=limits, q_ref=q_ref, allow_flip=False)
        if not r.ok:
            raise ArmStop(f"IK 실패 @ {np.round(p * 100, 1)} cm: {r.problems}")
        q_ref = r.q
        wps.append(r.q)
    return wps, q_ref


def travel(kin, limits, a, b, q_ref, q_end):
    """이동 구간: 위치는 직선, 손목 회전은 처음→끝으로 고르게 나눠 돌림, 마지막 점은 정확한 목표 자세.
    11/17: yaw를 처음부터 고정하면 대기 자세(팬 66°)에서 첫 경유점에 손목을 ~100° 한 번에 돌려야 해서 시간 초과."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    # 11/17: 직선은 대기 자세(오른쪽)→왼쪽 블록처럼 반대편으로 갈 때 베이스 바로 앞(반경 <10cm)을 지나 IK 실패
    # → 팬 축 기준 원호(반경·각도·높이를 각각 고르게 보간)로 이동
    c = W.PAN_AXIS_XY
    ra, rb = np.hypot(*(a[:2] - c)), np.hypot(*(b[:2] - c))
    ta, tb = math.atan2(a[1] - c[1], a[0] - c[0]), math.atan2(b[1] - c[1], b[0] - c[0])
    arc = abs(tb - ta) * max(ra, rb) + abs(rb - ra) + abs(b[2] - a[2])
    n = max(1, int(math.ceil(arc / 0.02)))
    r0, r1 = q_ref["wrist_roll"], q_end["wrist_roll"]
    wps = []
    for k in range(1, n):
        f = k / n
        rr, th = ra + (rb - ra) * f, ta + (tb - ta) * f
        p = np.array([c[0] + rr * math.cos(th), c[1] + rr * math.sin(th), a[2] + (b[2] - a[2]) * f])
        r = kin.ik_topdown(*p, limits=limits, q_ref=q_ref)
        if not r.ok:
            raise ArmStop(f"IK 실패 @ {np.round(p * 100, 1)} cm: {r.problems}")
        q = dict(r.q)
        q["wrist_roll"] = r0 + (r1 - r0) * k / n
        q_ref = q
        wps.append(q)
    wps.append(dict(q_end))
    return wps, dict(q_end)
