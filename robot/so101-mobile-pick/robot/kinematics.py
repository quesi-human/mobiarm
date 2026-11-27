"""SO-101 기구학 (robot/kinematics.py) — URDF만으로 계산, placo/수치 최적화 IK 사용 안 함.

- fk(q_deg)           : 손끝 접촉점 위치와 그리퍼 자세 (LeRobot RobotKinematics와 같은 각도 규약, 도 단위)
- ik_topdown(x,y,z)   : 손가락이 수직 아래를 향하는 자세의 관절각을 '공식'으로 계산
                        (pan = atan2, 어깨·팔꿈치 = 2링크 코사인 법칙, 손목 = 수직 유지, roll = 방향)

좌표계: base_link (원점 = 베이스 바닥 2.4 mm 위, +X 앞, +Y 왼쪽, +Z 위), 단위 m
손끝 접촉점: 11/13 실측으로 찾은 점 (gripper_frame_link 기준 [-6.2, 0.6, 5.2] mm)
"""
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

URDF_DEFAULT = Path(__file__).resolve().parent / "urdf_model/so101/so101_new_calib.urdf"
ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
_CHAIN = ARM + ["gripper_frame_joint"]
CONTACT_IN_TOOL_M = np.array([-0.0062, 0.0006, 0.0052])


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def _ang(v):
    return math.atan2(v[1], v[0])


def _rot2(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


@dataclass
class IKResult:
    q: dict                 # 관절각(도)
    contact_err_mm: float   # FK로 다시 계산한 손끝 위치 오차
    axis_err_deg: float     # 손가락 방향과 수직 아래의 각도 차이
    branch: str
    problems: list

    @property
    def ok(self):
        return not self.problems


class SO101Kinematics:
    def __init__(self, urdf=URDF_DEFAULT, contact=CONTACT_IN_TOOL_M):
        root = ET.parse(str(urdf)).getroot()
        joints = {j.get("name"): j for j in root.findall("joint")}
        self._origins, self._axes = [], []
        for n in _CHAIN:
            o = joints[n].find("origin")
            T = np.eye(4)
            T[:3, :3] = Rotation.from_euler("xyz", [float(v) for v in o.get("rpy").split()]).as_matrix()
            T[:3, 3] = [float(v) for v in o.get("xyz").split()]
            self._origins.append(T)
            ax = joints[n].find("axis")
            self._axes.append(None if ax is None else np.array([float(v) for v in ax.get("xyz").split()]))
        self.contact = np.asarray(contact, dtype=float)
        self._derive_geometry()

    # ---------------------------------------------------------------- FK
    def _frames(self, q_deg):
        """각 관절의 (축 위치, 축 방향) 목록과 tool 변환."""
        T = np.eye(4)
        pts = []
        for i, T0 in enumerate(self._origins):
            T = T @ T0
            if i < 5:
                pts.append((T[:3, 3].copy(), T[:3, :3] @ self._axes[i]))
                R = np.eye(4)
                R[:3, :3] = Rotation.from_rotvec(self._axes[i] * math.radians(q_deg[i])).as_matrix()
                T = T @ R
        return pts, T

    def tool_pose(self, q_deg):
        return self._frames(q_deg)[1]

    def fk(self, q_deg):
        """손끝 접촉점 위치(m)와 tool 회전행렬."""
        T = self.tool_pose(q_deg)
        return T[:3, :3] @ self.contact + T[:3, 3], T[:3, :3]

    # ---------------------------------------------------------------- 기하 추출 (0 자세 기준)
    def _derive_geometry(self):
        pts, T = self._frames(np.zeros(5))
        (p1, a1), (p2, a2), (p3, a3), (p4, a4), (p5, a5) = pts
        n = a2 / np.linalg.norm(a2)
        if not (np.allclose(a3, a2, atol=1e-4) and np.allclose(a4, a2, atol=1e-4)):
            raise RuntimeError("lift/elbow/wrist_flex 축이 평행하지 않음 — 평면 IK 가정 불성립")
        if abs(a1[2]) < 0.999 or abs(n[2]) > 1e-4:
            raise RuntimeError("pan 축이 수직이 아니거나 굽힘 축이 수평이 아님")
        self._pan_xy = p1[:2]
        self._pan_sign = 1.0 if a1[2] > 0 else -1.0          # +q → 위에서 볼 때 반시계(+1) / 시계(-1)
        zhat = np.array([0.0, 0.0, 1.0])
        uhat = np.cross(n, zhat)                              # 팔이 뻗는 수평 방향
        if uhat[0] < 0:
            uhat = -uhat
        self._uhat, self._nhat = uhat, n

        def plane(p):
            d = p - p1
            return np.array([d @ uhat, p[2]])

        self._A2, self._A3, self._A4 = plane(p2), plane(p3), plane(p4)
        self._G = plane(p5)                                   # roll 축 위의 점 (wrist_roll 관절)
        self._h = (p5 - p1) @ n                               # roll 축의 옆 방향 오프셋
        # 굽힘 관절 +q가 평면(u,z)에서 반시계(+1)인지 시계(-1)인지: 반시계 회전축 = u×z
        self._bend_sign = 1.0 if np.cross(uhat, zhat) @ n > 0 else -1.0
        # 0 자세에서 roll 축(=손가락 방향, tool z와 평행) 방향
        tool_z = T[:3, 2]
        if abs(tool_z @ n) > 1e-3:
            raise RuntimeError("손가락 방향이 굽힘 평면 밖에 있음")
        self._t0 = np.array([tool_z @ uhat, tool_z[2]])
        # 접촉점의 G 기준 위치 (tool 좌표계): roll 축 위의 G에서 접촉점까지
        G0 = p5
        contact0 = T[:3, :3] @ self.contact + T[:3, 3]
        self._c_tool = T[:3, :3].T @ (contact0 - G0)
        # roll +q일 때 위에서 본 tool x 회전 방향은 수치로 결정
        self._roll_sign = None

    # ---------------------------------------------------------------- IK
    @staticmethod
    def topdown_rotation(psi_rad):
        """손가락(tool z)이 수직 아래, tool x가 수평 방향 psi인 회전행렬."""
        z = np.array([0.0, 0.0, -1.0])
        x = np.array([math.cos(psi_rad), math.sin(psi_rad), 0.0])
        y = np.cross(z, x)
        return np.column_stack([x, y, z])

    def _solve_arm(self, G, elbow_up):
        """roll 축 점 G(world)를 지나며 손가락이 수직 아래인 q1..q4 (도)."""
        v = G[:2] - self._pan_xy
        r2 = v @ v - self._h ** 2
        if r2 <= 0:
            return None
        u = math.sqrt(r2)
        # pan: 0 자세에서 (u, h) 방향이 v 방향과 일치하도록
        ang0 = _ang(u * self._uhat[:2] + self._h * self._nhat[:2])
        q1 = math.degrees(_ang(v) - ang0) * self._pan_sign
        # 평면 2R + 손목
        phi3 = _ang([0.0, -1.0]) - _ang(self._t0)            # 손가락을 아래로 돌리는 누적 회전
        c = self._G - self._A4
        A4 = np.array([u, G[2]]) - _rot2(phi3) @ c
        a, b = self._A3 - self._A2, self._A4 - self._A3
        d = A4 - self._A2
        la, lb, ld = np.linalg.norm(a), np.linalg.norm(b), np.linalg.norm(d)
        cg = (ld ** 2 - la ** 2 - lb ** 2) / (2 * la * lb)
        if abs(cg) > 1:
            return None
        g = math.acos(cg) * (1 if elbow_up else -1)
        delta = g - (_ang(b) - _ang(a))                       # phi2 - phi1
        w = a + _rot2(delta) @ b
        phi1 = _ang(d) - _ang(w)
        phi2 = phi1 + delta
        s = self._bend_sign
        q2, q3, q4 = (math.degrees(x) * s for x in (phi1, phi2 - phi1, phi3 - phi2))
        return [_wrap(q1), _wrap(q2), _wrap(q3), _wrap(q4)]

    def _roll_for(self, q14, psi):
        """tool x 방향이 psi가 되도록 하는 roll (도)."""
        if self._roll_sign is None:
            R0 = self.tool_pose(q14 + [0.0])[:3, :3]
            R1 = self.tool_pose(q14 + [10.0])[:3, :3]
            self._roll_sign = 1.0 if _wrap(math.degrees(_ang(R1[:2, 0]) - _ang(R0[:2, 0]))) > 0 else -1.0
        psi0 = _ang(self.tool_pose(q14 + [0.0])[:2, 0])
        return _wrap(math.degrees(psi - psi0) * self._roll_sign)

    def ik_topdown(self, x, y, z, yaw_deg=None, limits=None, margin_deg=5.0, q_ref=None, allow_flip=True):
        """손끝 접촉점을 (x,y,z)에, 손가락은 수직 아래로.

        yaw_deg : tool x의 수평 방향(도). None이면 roll≈0이 되는 방향.
        limits  : {관절: (lo, hi)} 도 단위. 주면 margin_deg 안쪽인지 검사.
        q_ref   : 두 팔꿈치 해 중 이 자세에 가까운 쪽 우선 (없으면 elbow_up 우선).
        allow_flip : yaw를 줬을 때 180° 뒤집은 방향(그리퍼 대칭)도 후보로 허용할지.
        """
        target = np.array([x, y, z], dtype=float)
        cands = []
        for elbow_up in (True, False):
            if yaw_deg is None:
                psis = [None]
            elif allow_flip:
                psis = [math.radians(yaw_deg), math.radians(yaw_deg) + math.pi]
            else:
                psis = [math.radians(yaw_deg)]
            for psi in psis:
                q = None
                p = psi if psi is not None else math.atan2(y, x)
                for _ in range(6 if psi is None else 1):       # yaw 미지정: roll=0이 되도록 고정점 반복
                    G = target - self.topdown_rotation(p) @ self._c_tool
                    q14 = self._solve_arm(G, elbow_up)
                    if q14 is None:
                        break
                    if psi is None:
                        p = _ang(self.tool_pose(q14 + [0.0])[:2, 0])
                if q14 is None:
                    continue
                q5 = self._roll_for(q14, p) if psi is not None else 0.0
                q = q14 + [q5]
                cands.append((q, "elbow_up" if elbow_up else "elbow_down"))
        if not cands:
            return IKResult({}, float("inf"), float("inf"), "-", ["도달 불가 (팔 길이 밖)"])

        results = []
        for q, br in cands:
            pos, R = self.fk(q)
            err = float(np.linalg.norm(pos - target) * 1000)
            axis = math.degrees(math.acos(max(-1.0, min(1.0, -R[2, 2]))))
            problems = []
            if limits:
                for name, v in zip(ARM, q):
                    if name in limits and name != "wrist_roll":
                        lo, hi = limits[name]
                        if v < lo + margin_deg or v > hi - margin_deg:
                            problems.append(f"{name}={v:.1f}° 범위 [{lo:.1f},{hi:.1f}] 끝 {margin_deg}° 안")
            if err > 0.05 or axis > 0.05:
                problems.append(f"검증 실패: 위치 {err:.3f} mm, 방향 {axis:.3f}°")
            results.append(IKResult(dict(zip(ARM, q)), err, axis, br, problems))

        def score(r):
            dist = 0.0 if q_ref is None else sum(abs(_wrap(r.q[m] - q_ref[m])) for m in ARM if m in q_ref)
            return (not r.ok, dist, r.branch != "elbow_up", abs(r.q["wrist_roll"]))
        return sorted(results, key=score)[0]
