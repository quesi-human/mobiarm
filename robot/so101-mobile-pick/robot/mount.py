"""팔이 터틀봇 위에 달린 상태의 좌표 상수 (robot/mount.py) — v1 workcell.py를 대신함.

모든 좌표는 '팔 베이스(base_link)' 기준 (m). 터틀봇이 어디로 가든 이 값은 그대로다.
  surface_z_m    물건을 올려 둔 작업면(11/24: 16cm 상자 윗면) 높이. 팔 베이스(바닥에서 21cm)보다 아래라 음수.
                 steps/m1_floor_check.py가 손끝으로 짚어 재서 calibration/mount.json에 저장
  bumper_x_m     팔 회전축(팬 축)에서 터틀봇 몸체 앞 끝까지 거리(터틀봇 정면 방향으로). 이보다 2cm 앞부터 집음
  터틀봇 정면    이 팔은 팬 0°가 터틀봇 정면이 아님(11/24: 정면 = 팬 약 -29.5°, 팔 좌표로 +29.7° 방향).
                 보는 자세(m2)를 터틀봇 정면을 향해 저장하므로, 그 자세의 손끝 방향을 '터틀봇 정면'으로 씀.
  BLOCK_M        블록 한 변
  GRASP_POINT    블록을 쥐었을 때 블록 중심의 그리퍼(tool) 좌표 — v1 값
"""
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MOUNT_FILE = ROOT / "calibration/mount.json"
LOOK_FILE = ROOT / "calibration/look_pose.json"

BLOCK_M = 0.025
GRASP_POINT_DEFAULT = np.array([-0.0235, 0.0, -0.0025])
# ↑ 쥐었을 때 블록 중심 (tool 좌표, m). v1은 -9.5mm였는데, 11/24 m4에서 블록을 그대로 두고 집기→같은 자리 놓기→다시 찍기를
#   반복해 보니 놓인 블록 중심이 이 점보다 tool -x 쪽으로 15.0/13.7mm 떨어져 있었음(손목 카메라 기준) → -9.5-14 = -23.5mm.
#   이 값이 틀리면 ① 집을 때 고정 손가락이 블록에 붙고 ② 놓을 때마다 블록이 1cm 넘게 밀려남.
PAN_AXIS_XY = np.array([0.0388, 0.0])                      # URDF: 팬 회전축 위치
REACH_MAX_M = 0.26        # 위에서 수직으로 집을 수 있는 최대 거리 (팬 축 기준. 작업면이 5cm 아래(16cm 상자)일 때 계산값 27cm)
TRAVEL_ABOVE_SURFACE_M = 0.08


def _mount():
    if MOUNT_FILE.is_file():
        return json.loads(MOUNT_FILE.read_text(encoding="utf-8"))
    return {}


def surface_z(x=None, y=None):
    """물건이 놓인 작업면 높이 (팔 기준, m).

    x, y를 주면 m1이 손끝으로 짚어 맞춘 평면(z = a + b·x + c·y)에서 그 자리 높이를 돌려준다.
    11/24 측정: 손목을 수직으로 해도 앞으로 갈수록 낮게 읽힘(약 6°). 원인이 팔/터틀봇의 실제 기울기든
    어깨 관절 각도 차이든, '팔 계산으로 본 면'을 그대로 쓰면 손끝은 실제 면에 정확히 닿는다.
    x, y를 안 주면 팬 축 바로 아래 높이(예전 값).
    """
    m = _mount()
    coef = m.get("floor_plane_coef")
    if x is not None and y is not None and coef is not None:
        a, b, c = coef
        return float(a + b * x + c * y)
    z = m.get("surface_z_m", m.get("floor_z_m"))
    if z is None:
        raise RuntimeError("calibration/mount.json 없음 — 먼저 python steps/m1_floor_check.py")
    return float(z)


floor_z = surface_z   # 예전 이름


def _look():
    if LOOK_FILE.is_file():
        return json.loads(LOOK_FILE.read_text(encoding="utf-8"))
    return {}


def forward_heading_deg():
    """터틀봇 정면이 팔 좌표에서 몇 도 방향인지 (x축 0°, y쪽 +). 보는 자세 손끝 방향, 없으면 0."""
    lk = _look()
    if "forward_heading_deg" in lk:            # 직접 넣은 값이 있으면 우선
        return float(lk["forward_heading_deg"])
    if "tip_m" in lk:
        t = np.asarray(lk["tip_m"], float)
        return float(np.degrees(np.arctan2(t[1] - PAN_AXIS_XY[1], t[0] - PAN_AXIS_XY[0])))
    return 0.0


def forward_unit():
    h = np.radians(forward_heading_deg())
    return np.array([np.cos(h), np.sin(h)])


def to_robot(x, y):
    """팔 좌표 (x, y) → 팬 축 기준 (터틀봇 앞으로, 왼쪽으로) m."""
    d = np.array([x, y], float) - PAN_AXIS_XY
    u = forward_unit()
    return float(d @ u), float(u[0] * d[1] - u[1] * d[0])


def from_robot(fwd, left):
    """팬 축 기준 (터틀봇 앞으로, 왼쪽으로) m → 팔 좌표 (x, y)."""
    u = forward_unit()
    v = np.array([-u[1], u[0]])
    return PAN_AXIS_XY + fwd * u + left * v


def bumper_x():
    return float(_mount().get("bumper_x_m", 0.10))


def block_center_z(x=None, y=None):
    return surface_z(x, y) + BLOCK_M / 2


def grasp_point():
    return GRASP_POINT_DEFAULT


def reach_ok(x, y):
    """팔이 위에서 집을 수 있고(최대 거리), 터틀봇 몸체보다 앞(터틀봇 정면 방향 기준)인지."""
    r = float(np.hypot(x - PAN_AXIS_XY[0], y - PAN_AXIS_XY[1]))
    fwd, _ = to_robot(x, y)
    return (fwd >= bumper_x() + 0.02 and r <= REACH_MAX_M), r


def pixel_to_plane(px, H, k1, center, fscale, k2=0.0):
    """픽셀 → (방사 왜곡 보정) → 평면 변환 H → 팔 기준 작업면 XY (m). v1과 같은 식. px: (N,2)"""
    px = np.atleast_2d(np.asarray(px, float))
    d = px - center
    r2 = (d ** 2).sum(1, keepdims=True) / fscale ** 2
    pu = center + d * (1 + k1 * r2 + k2 * r2 ** 2)
    hom = np.c_[pu, np.ones(len(pu))] @ np.asarray(H).T
    return hom[:, :2] / hom[:, 2:3]
