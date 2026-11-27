"""손목 카메라: '보는 자세'로 가기 + 그 자세에서 찍은 사진으로 블록 위치·방향(팔 좌표) 구하기.

보정(calibration/wrist_camera.json, m3)은 보는 자세(look_q_deg)에서 찍은 사진에만 맞음.
블록 위치 = 블록 윤곽 중심 픽셀 → 보정식 → 블록 윗면 XY (m3에서 손끝으로 짚은 '윗면 중앙'과 같은 기준).
다시 보기(relook) = 블록이 화면 가장자리에 보이면 팬만 돌려(팔 전체가 팬 축으로 강체 회전 → 보정식을 같은 각도로
  돌리면 그대로 맞음) 블록을 보정이 촘촘한 화면 가운데(REF_PX)로 데려와 다시 찍음. 11/24: 화면 아래쪽(터틀봇 기준 왼쪽)
  블록을 자주 놓침 — 보정 점이 적고 그리퍼에 가까운 곳.
블록 방향 = 윤곽 점들을 같은 보정식으로 팔 좌표에 옮긴 뒤 최소 사각형의 모서리 방향 (0~90°, 정육면체라 90° 대칭).
  손목 카메라가 비스듬해서 윤곽에 블록 옆면이 조금 섞임 → 방향은 ±10° 정도 오차가 날 수 있음.
"""
import json
from pathlib import Path

import numpy as np

from robot import mount as M
from robot.arm import ARM, ArmStop
from robot.motion import go, plan_arc, plan_line

ROOT = Path(__file__).resolve().parents[1]
CALIB_FILE = ROOT / "calibration/wrist_camera.json"
LOOK_TOL_COUNTS = 20      # 보는 자세 도착 허용 (어깨가 무게로 0.6~0.9° 처지는 건 매번 같은 방향이라 괜찮음)
LOOK_Q_WARN_DEG = 1.5     # 보정 때 보는 자세와 지금 보는 자세가 이보다 다르면 보정이 안 맞음
REF_PX = (295.0, 190.0)   # 보정 점들이 모인 가운데 (m3: 가로 149~443, 세로 70~338)
RELOOK_MIN_DEG = 2.0      # 이보다 작게 돌아가면 다시 보기 생략
BLOCK_SIZE_MM = (15.0, 50.0)   # 윗면 크기(보정식으로 바꾼 mm)가 이 범위 밖이면 블록 아님
# 11/25 파이: 블록이 없는 곳에서 7×6mm짜리 노란 점을 블록으로 보고 집으러 감 → 크기로 거름
#   (블록 25mm, 옆면이 섞여 보통 26~35mm로 보임)


def load_look_q():
    if not M.LOOK_FILE.is_file():
        raise ArmStop("calibration/look_pose.json 없음 — 먼저 python steps/m2_teach_look_pose.py")
    look = json.loads(M.LOOK_FILE.read_text(encoding="utf-8"))
    return {j: float(look["q_deg"][j]) for j in ARM}


def load_calib():
    if not CALIB_FILE.is_file():
        raise ArmStop("calibration/wrist_camera.json 없음 — 먼저 m3_collect_camera.py → m3_solve_camera.py --save")
    cal = json.loads(CALIB_FILE.read_text(encoding="utf-8"))
    look_q = load_look_q()
    diff = max(abs(look_q[j] - float(cal["look_q_deg"][j])) for j in ARM)
    if diff > LOOK_Q_WARN_DEG:
        raise ArmStop(f"보는 자세(look_pose.json)가 카메라 보정 때와 {diff:.1f}° 다름 — m3 다시")
    return cal


def go_look(arm, kin, limits, look_q):
    """지금 자세 → (상자 위로 충분히 들어) → 원호로 → 보는 자세(관절값 그대로)."""
    q = arm.read_deg()
    p, _ = kin.fk([q[j] for j in ARM])
    p_look, _ = kin.fk([look_q[j] for j in ARM])
    above = np.array([p[0], p[1], max(p[2], M.surface_z(*p[:2]) + M.TRAVEL_ABOVE_SURFACE_M)])
    q_ref = q
    if above[2] - p[2] > 0.005:
        wps, q_ref = plan_line(kin, limits, p, above, q)
        go(arm, wps, precise=False)
    if np.linalg.norm(above - p_look) > 0.01:
        wps, _ = plan_arc(kin, limits, above, p_look, q_ref)
        go(arm, wps, precise=False)
    res = arm.move_deg(look_q, result_tol=LOOK_TOL_COUNTS)
    if not res.status.startswith("도착"):
        raise ArmStop(f"보는 자세로: {res.status}")


def to_plane(cal, px):
    return M.pixel_to_plane(px, cal["H"], cal["k1"], np.asarray(cal["center_px"], float), cal["fscale_px"],
                            cal.get("k2", 0.0))


def locate(cam, cal, color, save_image=None):
    """보는 자세에서 블록 하나 찾기 → ({'xy_m', 'yaw_deg', 'pixel'}, diag) 또는 (None, diag)."""
    import cv2

    def size_mm(contour):
        pts = to_plane(cal, contour.reshape(-1, 2).astype(float)) * 1000
        (_, (w, h), ang) = cv2.minAreaRect(pts.astype(np.float32))
        return float(w), float(h), float(ang)

    def is_block(d):
        w, h, _ = size_mm(d["contour"])
        return BLOCK_SIZE_MM[0] <= min(w, h) and max(w, h) <= BLOCK_SIZE_MM[1]

    m, diag = cam.measure(color, n_frames=10, save_image=save_image, keep_fn=is_block)
    if m is None:
        return None, diag
    xy = to_plane(cal, [m["pixel"]])[0]
    yaw = None
    if m.get("contour") is not None:
        w, h, ang = size_mm(m["contour"])
        yaw = float(ang % 90.0)
        diag["top_size_mm"] = [round(float(w), 1), round(float(h), 1)]
    diag["spread_px"] = diag.get("spread_px")
    return {"xy_m": xy.tolist(), "yaw_deg": yaw, "pixel": m["pixel"]}, diag


def rotate_about_pan(xy, phi_deg):
    c = M.PAN_AXIS_XY
    t = np.radians(phi_deg)
    d = np.asarray(xy, float) - c
    return c + np.array([d[0] * np.cos(t) - d[1] * np.sin(t), d[0] * np.sin(t) + d[1] * np.cos(t)])


def relook(arm, cam, cal, look_q, b, color, save_image=None):
    """팬만 돌려 블록을 화면 가운데로 → 다시 찍기. 반환 (블록, 그때 관절값, 돌린 각도°) — 못 찾으면 블록 None.
    팬 +1° = 팔 좌표에서 시계 방향 1°(방위각 -1°), 팬 축 기준 반경은 그대로 (FK로 확인)."""
    c = M.PAN_AXIS_XY
    ref = to_plane(cal, [REF_PX])[0]
    th_b = np.degrees(np.arctan2(b["xy_m"][1] - c[1], b["xy_m"][0] - c[0]))
    th_r = np.degrees(np.arctan2(ref[1] - c[1], ref[0] - c[0]))
    phi = float(th_b - th_r)
    q_now = arm.read_deg()
    phi_now = -(q_now["shoulder_pan"] - look_q["shoulder_pan"])
    if abs(phi - phi_now) < RELOOK_MIN_DEG:          # 이미 거의 가운데 (찾기에서 돌린 자세 포함)
        return b, q_now, 0.0
    q_view = dict(look_q)
    q_view["shoulder_pan"] = look_q["shoulder_pan"] - phi
    prob = arm.check_targets_deg(q_view)
    if prob:
        return b, q_now, 0.0                            # 팬 범위 끝이면 처음 사진 그대로
    res = arm.move_deg(q_view, result_tol=LOOK_TOL_COUNTS)
    if not res.status.startswith("도착"):
        raise ArmStop(f"다시 보기 자세로: {res.status}")
    q_act = arm.read_deg()
    phi_act = -(q_act["shoulder_pan"] - look_q["shoulder_pan"])   # 실제로 돈 각도로 계산
    b2, diag = locate(cam, cal, color, save_image=save_image)
    if b2 is None:
        return None, q_act, phi_act
    b2["xy_m"] = rotate_about_pan(b2["xy_m"], phi_act).tolist()
    if b2["yaw_deg"] is not None:
        b2["yaw_deg"] = float((b2["yaw_deg"] + phi_act) % 90.0)
    b2["diag"] = diag
    return b2, q_act, phi_act


SCAN_STEP_DEG = 12.0      # 한 화면이 팬 방향으로 약 20° (집게에 가린 아래쪽 빼고) → 12°씩 겹치게
SCAN_SPAN_DEG = 60.0      # 터틀봇 정면 기준 좌우 이만큼까지 둘러봄


def scan_steps(span=SCAN_SPAN_DEG, step=SCAN_STEP_DEG):
    """0 → 왼12 → 오른12 → 왼24 → 오른24 … (방위각, + = 터틀봇 왼쪽).
    11/25: 왼쪽 끝까지 다 본 뒤 오른쪽으로 가니 '오른쪽은 안 도는 것'처럼 보이고 오래 걸림 → 양쪽 번갈아 넓혀 감."""
    n = int(span // step)
    out = [0.0]
    for k in range(1, n + 1):
        out += [step * k, -step * k]
    return out


SEARCH_PAN_DEG = tuple(scan_steps())


def search(arm, cam, cal, look_q, color, save_image=None, steps=SEARCH_PAN_DEG):
    """보는 자세에서 못 찾으면 팬만 돌려 가며 둘러보기 (기본: 정면 → 왼쪽 60°까지 → 오른쪽 60°까지, 12°씩).
    팬만 돌리면 보는 높이·거리는 그대로라 같은 보정을 돌려서 씀 → 팬 축에서 약 16~27cm 떨어진 띠 전체를 볼 수 있음. 반환 (블록(팔 좌표), 관절값, 돈 각도°, diag) / 못 찾으면 블록 None.
    11/24: 놓은 블록이 화면 아래(집게 뒤)에 있으면 보는 자세에서 못 찾음 → 왼쪽으로 8° 돌리면 화면 위로 올라옴."""
    diag = {}
    for phi in steps:
        q_view = dict(look_q)
        q_view["shoulder_pan"] = look_q["shoulder_pan"] - phi
        if arm.check_targets_deg(q_view):
            continue
        res = arm.move_deg(q_view, result_tol=LOOK_TOL_COUNTS)
        if not res.status.startswith("도착"):
            raise ArmStop(f"찾기 회전: {res.status}")
        q_act = arm.read_deg()
        phi_act = -(q_act["shoulder_pan"] - look_q["shoulder_pan"])
        img = None
        if save_image is not None:
            img = save_image if phi == 0 else Path(save_image).with_name(f"{Path(save_image).stem}_pan{phi:+.0f}.png")
        b, diag = locate(cam, cal, color, save_image=img)
        if b is not None:
            b["xy_m"] = rotate_about_pan(b["xy_m"], phi_act).tolist()
            if b["yaw_deg"] is not None:
                b["yaw_deg"] = float((b["yaw_deg"] + phi_act) % 90.0)
            return b, q_act, phi_act, diag
    return None, arm.read_deg(), 0.0, diag
