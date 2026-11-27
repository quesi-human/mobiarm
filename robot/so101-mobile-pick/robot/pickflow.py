"""집기 공통 단계 (m4 테스트와 팔 서버가 같이 씀). 11/25 m4_pick_test.py에서 옮김."""
import math

import numpy as np

from robot import mount as M
from robot.arm import ARM, ArmStop
from robot.motion import go
from robot.pick import PICK_RAISE_M, ROLL_CHUNK_DEG, line, pick_yaw, travel

DOWN_TOL_COUNTS = 20      # 11/24: 멀리(앞 26cm) 뻗으면 어깨가 무게로 14카운트(≈1.2°) 처져 '오차 남음' STOP → 내려가 집기·놓기만 허용 넓힘


def go_down(arm, wps):
    """내려가는 경로: 중간은 그냥 지나가고, 마지막만 보정하되 어깨 처짐 20카운트까지 허용."""
    if len(wps) > 1:
        go(arm, wps[:-1], precise=False)
    res = arm.move_deg(wps[-1], result_tol=DOWN_TOL_COUNTS)
    if not res.status.startswith("도착"):
        raise ArmStop(res.status)
    return res


def unwind_roll(arm, target):
    roll = arm.read_deg()["wrist_roll"]
    d = target - roll
    if abs(d) > ROLL_CHUNK_DEG:
        n = int(math.ceil(abs(d) / ROLL_CHUNK_DEG))
        for k in range(1, n + 1):
            res = arm.move_deg({"wrist_roll": roll + d * k / n}, correct=(k == n), result_tol=40)
            if res.status.startswith("정지"):
                raise ArmStop("손목 되돌리기 " + res.status)


def plan_pick(kin_g, kin_pick, limits, look_q, xy, yaw_block, raise_m=PICK_RAISE_M):
    """블록 위치·방향 → (yaw, 블록 위 자세, 경로들) 또는 이유 문자열. look_q = 지금(출발) 관절값."""
    x, y = xy
    ok, r = M.reach_ok(x, y)
    if not ok:
        f, l = M.to_robot(x, y)
        return f"집을 수 없는 위치 (터틀봇 기준 앞 {f*100:.1f}, 왼 {l*100:+.1f} cm, 팬 축에서 {r*100:.1f} cm)"
    z_c = M.block_center_z(x, y) + raise_m      # 블록 중심보다 1cm 위에서 쥠 (손가락 끝이 상자에 안 닿게)
    sel = pick_yaw(kin_g, (x, y, z_c), yaw_block if yaw_block is not None else 0.0, limits, look_q)
    if sel is None:
        return "이 위치·방향으로 집는 자세가 없음 (손목 회전 범위)"
    yaw = sel[0]
    above = np.array([x, y, M.surface_z(x, y) + M.TRAVEL_ABOVE_SURFACE_M])
    r_above = kin_g.ik_topdown(*above, yaw_deg=yaw, limits=limits, q_ref=sel[1].q, allow_flip=False)
    if not r_above.ok:
        return f"블록 위 자세 IK 실패: {r_above.problems}"
    p_look, _ = kin_g.fk([look_q[j] for j in ARM])
    try:
        to_above, q_ref = travel(kin_g, limits, p_look, above, look_q, r_above.q)
        down, q_ref = line(kin_pick, limits, above, (x, y, z_c), q_ref, yaw)
        up, _ = line(kin_g, limits, (x, y, z_c), above, q_ref, yaw)
    except ArmStop as e:
        return str(e)
    return {"yaw": yaw, "z_c": z_c, "above": above, "q_above": r_above.q,
            "to_above": to_above, "down": down, "up": up}


