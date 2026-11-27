"""SO-101 Follower 공통 제어 모듈 (robot/arm.py)

원칙
- robot.connect()를 쓰지 않는다: configure()가 모터 레지스터를 덮어쓰기 때문. 버스만 직접 연다.
- 각도 환산은 calibration/motor_matched/my_follower_arm.json 기준 (내 측정 범위 + 모터 영점).
  연결할 때 모터 Homing_Offset이 이 파일과 다르면 중단한다.
- 모터의 EEPROM 설정(P/I/D, 영점, 범위)은 절대 쓰지 않는다. 공유 팔이기 때문.
  RAM 설정(Goal_Velocity, Acceleration)만 바꾸고 close() 때 원래 값으로 되돌린다.
- 모든 이동은: 속도 제한 → 감시(안 움직임/부하/시간) → 멈추면 남은 오차만큼 목표를 밀어 보정.

사용 예:
    from robot.arm import Arm
    with Arm() as arm:
        arm.torque_on()                    # 현재 자세 그대로 유지하며 토크 ON
        res = arm.move_deg({"wrist_flex": 30.0})
        print(res.summary())
"""
import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ALL = ARM + ["gripper"]
DEFAULT_CALIB = Path("calibration/motor_matched/my_follower_arm.json")

MAX_RES = 4095
DEG_PER_COUNT = 360 / MAX_RES

# 이동 설정
SPEED = 400            # step/s (≈26°/s). 0은 '최고속'이므로 절대 쓰지 않는다
MIN_SPEED = 40         # 여러 관절 동시 이동 시 짧게 움직이는 관절의 최소 속도
ACCELERATION = 20      # LeRobot 기본 254보다 완만
TARGET_MARGIN_DEG = 5.0  # 목표가 측정 범위 끝에서 이만큼 안쪽이어야 함

# 통신: Feetech 버스는 가끔 응답 패킷 하나가 유실됨(11/13 ID4 'no status packet'). 같은 값을 다시 보내도 안전
RETRY = 3

# 감시/정지 기준
# 11/17: 600/40으로 올린 뒤부터 관절이 ~80/s로 기어가는 현상 → 순간 부하가 과부하 기준(80%)을 넘어
# 모터 보호(출력 20%로 제한)가 걸린 것으로 추정. 오전(300/20)엔 없었음 → 400/20 + 보호 상태 비트 감시
CHUNK_COUNTS = 250             # 큰 이동을 나누는 크기 (≈22°)
PASS_TOL_CHUNK = 40            # 조각 사이는 이만큼 가까워지면 바로 다음 조각 (안 멈춤)
STATUS_PROTECT = 0x20 | 0x08   # Status: 0x20 과부하, 0x08 전류 초과
START_TIMEOUT_S = 0.6  # 이 안에 2카운트 이상 움직이지 않으면 정지
START_MIN_COUNTS = 40  # 이보다 짧게(≈3.5°) 움직이는 관절은 시작 감시 제외 (중력 데드밴드 ~25카운트)
LOAD_LIMIT = 600       # Present_Load (0.1% 단위) 60%
STALL_S = 1.0          # 목표에서 먼 관절이 이 시간 동안 진행이 없으면 '막힘'으로 정지 (고정 시간 초과 대신)
STALL_MIN_ERR = 10     # 남은 오차가 이 카운트보다 클 때만 막힘 판정 (중력 데드밴드 ~25는 Moving=0이라 해당 안 됨)
DEADBAND_MAX = 150     # 진행이 멈춘 관절의 남은 거리가 이 이하이고 부하가 낮으면 데드밴드로 보고 보정 단계로 넘김 (≈13°)
LOAD_BLOCK = 300       # 진행 없이 이 부하(30%)를 넘으면 무언가를 밀고 있는 것 → 막힘 (평소 이동은 ≤18%, 11/13 로그)
SETTLE_S = 0.15        # Moving=0 유지 시간 (11/17: 0.3 → 0.15, 매 이동마다 기다리던 시간 줄임)

# 도착 보정 (11/13 측정: 중력 거스르는 방향에서 elbow 24~26, lift 16~17카운트 덜 감)
CORR_TOL = 3           # 이 카운트 이하면 보정 안 함 (≈0.26°)
CORR_MAX_ADJ = 100     # 원래 목표 대비 최대 추가 명령량 (≈8.8°) — 먼 거리에서 어깨 처짐이 더 큼
CORR_ITERS = 3
CORR_SPEED = 200       # 보정 이동 속도 (11/17: 80 → 200, 보정 1회에 1초 넘게 걸렸음)
DEADBAND_ERR = 60      # 진행이 멈춘 관절의 남은 오차가 이 이하면 DEADBAND_S만 기다리고 중력 데드밴드로 판정
DEADBAND_S = 0.25      # (11/17: 데드밴드 관절 하나 때문에 매 이동마다 STALL_S=1초씩 기다려 한 사이클 50초)
CORR_RESULT_TOL = 8    # 보정 후에도 이보다 크면 '오차 남음' (≈0.7°)

# 그리퍼 (open_pct: 0=완전히 닫힘 ~ 100=완전히 열림, 측정 범위 기준)
GRIPPER_CONFIG = Path("calibration/gripper.json")   # {"open_is_high": true/false} — 처음 한 번 확인해서 저장
GRIP_SPEED = 600
GRIP_TORQUE_LIMIT = 500     # RAM Torque_Limit (0.1% 단위) — 블록을 쥘 때 최대 50%
GRIP_SQUEEZE = 15           # 블록에 닿아 멈춘 위치에서 더 조이는 양(카운트). 계속 최대 힘으로 누르지 않게
GRIP_EMPTY_MARGIN = 40      # 닫힘 끝에서 이만큼 안에서 멈추면 '아무것도 안 잡힘'


class ArmStop(RuntimeError):
    """안전 정지. 팔은 정지한 위치에서 유지 중."""


@dataclass
class MoveResult:
    status: str                                   # "도착" 또는 정지 사유
    target: dict = field(default_factory=dict)     # 원래 목표 (raw)
    final: dict = field(default_factory=dict)      # 최종 위치 (raw)
    err_before: dict = field(default_factory=dict)  # 보정 전 오차 (raw)
    iters: int = 0
    max_load: int = 0
    seconds: float = 0.0
    log: list = field(default_factory=list)

    @property
    def err(self):
        return {m: self.target[m] - self.final[m] for m in self.target}

    def summary(self):
        parts = [f"{m}: {self.err_before.get(m, 0):+d}→{self.err[m]:+d}" for m in self.target]
        return (f"{self.status} | 오차(보정 전→후, 카운트) " + ", ".join(parts) +
                f" | 보정 {self.iters}회, 최대부하 {self.max_load}, {self.seconds:.2f}s")


class Arm:
    def __init__(self, port=None, robot_id="my_follower_arm", calibration=DEFAULT_CALIB):
        if port is None:
            from robot.devices import ARM_PORT as port   # 장치 고유 이름(by-id), 없으면 /dev/ttyACM0
        self.port, self.robot_id = port, robot_id
        self.calib_path = Path(calibration)
        if self.calib_path.stem != robot_id:
            raise ValueError(f"캘리브레이션 파일 이름은 {robot_id}.json 이어야 함: {self.calib_path}")
        if not self.calib_path.is_file():
            raise FileNotFoundError(f"{self.calib_path} 없음 (프로젝트 루트에서 실행?)")
        self.cal = json.loads(self.calib_path.read_text())
        self.bus = None
        self.robot = None
        self.last_log = []
        self._orig_ram = {}
        self.torque_was = None
        self.gripper_torque_was = 0
        self.open_is_high = None

    # ---------------------------------------------------------------- 연결
    def connect(self):
        # 11/25: lerobot 대신 robot/feetech_bus.py (라즈베리파이에 lerobot·PyTorch를 깔지 않으려고).
        #        같은 통신 방식이고, RAM 레지스터 5개 말고는 쓸 수 없게 막혀 있음
        from robot.feetech_bus import FeetechBus
        self.bus = FeetechBus(self.port).connect()
        motor_cal = self.bus.read_calibration()
        bad = [m for m in ALL if int(motor_cal[m].homing_offset) != int(self.cal[m]["homing_offset"])]
        if bad:
            self.bus.disconnect(disable_torque=False)
            raise ArmStop(f"모터 Homing_Offset이 {self.calib_path}와 다름: {bad} — 누군가 재캘리브레이션함")
        self.torque_was = self._read("Torque_Enable")
        for m in ALL:
            regs = ("Goal_Velocity", "Acceleration") + (("Torque_Limit",) if m == "gripper" else ())
            self._orig_ram[m] = {r: int(self.bus.read(r, m, normalize=False, num_retry=RETRY)) for r in regs}
        self.gripper_torque_was = int(self.bus.read("Torque_Enable", "gripper", normalize=False, num_retry=RETRY))
        self.open_is_high = None
        if GRIPPER_CONFIG.is_file():
            self.open_is_high = bool(json.loads(GRIPPER_CONFIG.read_text())["open_is_high"])
        return self

    def close(self, disable_torque=None):
        """RAM 설정 복구 후 연결 종료. disable_torque=None이면 연결 전 토크 상태로 되돌림."""
        if self.bus is None:
            return
        # 11/17: 그리퍼 과부하 오류로 복구 쓰기가 실패하면 토크가 켜진 채 끝났음 → 하나씩, 실패해도 계속
        for m, regs in self._orig_ram.items():
            for r, v in regs.items():
                try:
                    self.bus.write(r, m, v, normalize=False, num_retry=RETRY)
                except Exception as e:
                    print(f"[close] {m}.{r} 복구 실패: {e}")
        if disable_torque is None:
            disable_torque = not any((self.torque_was or {}).values()) and not self.gripper_torque_was
        if disable_torque:
            for m in list(self.bus.motors):
                try:
                    self.bus.write("Torque_Enable", m, 0, normalize=False, num_retry=RETRY)
                except Exception as e:
                    print(f"[close] {m} 토크 OFF 실패: {e} — 전원을 잠깐 뺐다 꽂으세요")
        try:
            self.bus.disconnect(disable_torque=False)
        except Exception as e:
            print(f"[close] 연결 종료 오류: {e}")
        self.bus = None
        self.robot = None

    def __enter__(self):
        return self.connect()

    def __exit__(self, exc_type, exc, tb):
        if exc_type is not None and self.bus is not None:
            try:
                self.hold()            # 예외 시 그 자리에서 정지
            except Exception:
                pass
        # 토크 끄기는 호출 쪽에서 팔을 받친 뒤 close(disable_torque=True)로 명시적으로 하는 것이 원칙
        self.close(disable_torque=False if exc_type is not None else None)

    # ---------------------------------------------------------------- 읽기/환산
    def _read(self, reg, motors=ARM):
        return {k: int(v) for k, v in self.bus.sync_read(reg, motors, normalize=False, num_retry=RETRY).items()}

    def _mid(self, m):
        return (self.cal[m]["range_min"] + self.cal[m]["range_max"]) / 2

    def raw_to_deg(self, m, raw):
        return (raw - self._mid(m)) * DEG_PER_COUNT

    def deg_to_raw(self, m, deg):
        return int(round(deg / DEG_PER_COUNT + self._mid(m)))

    def range_deg(self, m):
        half = (self.cal[m]["range_max"] - self.cal[m]["range_min"]) / 2 * DEG_PER_COUNT
        return -half, half

    def read_raw(self):
        return self._read("Present_Position")

    def read_deg(self):
        return {m: self.raw_to_deg(m, v) for m, v in self.read_raw().items()}

    # ---------------------------------------------------------------- 토크/유지
    def hold(self):
        """현재 위치를 목표로 써서 그 자리에 정지."""
        pos = self.read_raw()
        for m, v in pos.items():
            self.bus.write("Goal_Position", m, v, normalize=False, num_retry=RETRY)
        return pos

    def torque_on(self):
        """목표=현재로 맞춘 뒤 토크 ON → 튀지 않음."""
        pos = self.hold()
        self.bus.enable_torque(ARM, num_retry=RETRY)
        return pos

    def torque_off(self):
        self.bus.disable_torque(ARM, num_retry=RETRY)

    # ---------------------------------------------------------------- 이동
    def check_targets_deg(self, targets_deg):
        problems = []
        for m, d in targets_deg.items():
            if m not in ARM:
                problems.append(f"{m}: 알 수 없는 관절")
                continue
            if m == "wrist_roll":
                continue
            lo, hi = self.range_deg(m)
            if d < lo + TARGET_MARGIN_DEG or d > hi - TARGET_MARGIN_DEG:
                problems.append(f"{m}={d:.1f}°: 범위 [{lo:.1f}, {hi:.1f}] 끝 {TARGET_MARGIN_DEG}° 안")
        return problems

    def move_deg(self, targets_deg, correct=True, result_tol=None, pass_tol=None):
        problems = self.check_targets_deg(targets_deg)
        if problems:
            raise ArmStop("목표 거부: " + "; ".join(problems))
        return self.move_raw({m: self.deg_to_raw(m, d) for m, d in targets_deg.items()}, correct=correct,
                             result_tol=result_tol, pass_tol=pass_tol)

    def _set_speeds(self, start, goal):
        """여러 관절이 동시에 도착하도록 거리 비례 속도."""
        dist = {m: abs(goal[m] - start[m]) for m in goal}
        far = max(dist.values()) or 1
        for m in goal:
            v = max(MIN_SPEED, int(SPEED * dist[m] / far))
            self.bus.write("Goal_Velocity", m, v, normalize=False, num_retry=RETRY)
            self.bus.write("Acceleration", m, ACCELERATION, normalize=False, num_retry=RETRY)
        return far / SPEED

    def _watch(self, cmd, res, phase, expect_start, timeout, pass_tol=None):
        """cmd(raw)를 보낸 뒤 모든 관절이 멈출 때까지 감시."""
        motors = list(cmd)
        start = self._read("Present_Position", motors)
        for m, v in cmd.items():
            self.bus.write("Goal_Position", m, v, normalize=False, num_retry=RETRY)
        written = self._read("Goal_Position", motors)
        if written != cmd:
            self.hold()
            raise ArmStop(f"Goal_Position 기록 불일치: 보냄 {cmd}, 읽음 {written}")
        t0 = time.monotonic()
        # START_MIN_COUNTS 미만으로 움직이는 관절은 중력 데드밴드 때문에 아예 안 움직일 수 있어 '시작 감시'에서 제외
        started = {m: (not expect_start) or abs(cmd[m] - start[m]) < START_MIN_COUNTS for m in motors}
        still_since = None
        prog_pos, prog_t = dict(start), {m: 0.0 for m in motors}   # 관절별 마지막으로 '진행'한 위치·시각
        while True:
            t = time.monotonic() - t0
            pos = self._read("Present_Position", motors)
            moving = self._read("Moving", motors)
            load = {m: abs(v) for m, v in self._read("Present_Load", motors).items()}
            status = self._read("Status", motors)
            prot = {m: hex(v) for m, v in status.items() if v & STATUS_PROTECT}
            if prot:
                self.hold()
                raise ArmStop(f"모터 보호 기능 작동 {prot} (0x20 과부하, 0x08 전류) — 힘을 너무 오래/세게 썼음. "
                              "토크 OFF 후 1~2분 쉬었다가 다시")
            res.max_load = max(res.max_load, *load.values())
            for m in motors:
                res.log.append([phase, round(t, 3), m, pos[m], cmd[m], res.target[m] - pos[m], moving[m], load[m]])
                if not started[m] and abs(pos[m] - start[m]) >= 2:
                    started[m] = True
            # 이동시킨 관절이 '하나도' 움직이지 않을 때만 정지 (통신/토크 문제).
            # 한 관절만 늦는 건 중력 데드밴드 — 먼 거리에서 어깨를 들 때 흔함(11/16) → 도착 보정이 마저 밀어줌
            needed = [m for m in motors if abs(cmd[m] - start[m]) > DEADBAND_MAX]
            if needed and t > START_TIMEOUT_S * 2 and not any(abs(pos[m] - start[m]) >= 2 for m in needed):
                self.hold()
                raise ArmStop(f"어떤 관절도 움직이기 시작하지 않음: {needed} (토크/전원/통신 확인)")
            over = [m for m in motors if load[m] > LOAD_LIMIT]
            if over:
                self.hold()
                raise ArmStop(f"부하 한계 초과: {over}")
            for m in motors:
                if abs(pos[m] - prog_pos[m]) >= 3:
                    prog_pos[m], prog_t[m] = pos[m], t
            # 막힘: 아직 목표에서 먼데(움직이려 하는데) STALL_S 동안 3카운트도 못 움직임
            # 한 번도 진행하지 못했고 남은 거리가 작은 관절 = 중력 데드밴드 (막힘 아님, 보정이 밀어줌)
            # 진행이 멈춘 관절: 목표에서 멀거나(>DEADBAND_MAX) 세게 밀고 있으면(부하>LOAD_BLOCK) '막힘',
            # 가깝고 부하도 낮으면 중력 데드밴드 → 멈춘 것으로 보고 도착 보정이 마저 밀어줌
            no_prog = {m for m in motors if abs(cmd[m] - pos[m]) > STALL_MIN_ERR
                       and t - prog_t[m] > (DEADBAND_S if abs(cmd[m] - pos[m]) <= DEADBAND_ERR and load[m] <= LOAD_BLOCK
                                            else STALL_S if prog_t[m] > 0 else START_TIMEOUT_S * 2)}
            blocked = [m for m in no_prog if moving[m]
                       and (abs(cmd[m] - pos[m]) > DEADBAND_MAX or load[m] > LOAD_BLOCK)]
            deadband = no_prog - set(blocked)
            if blocked or t > timeout:
                self.hold()
                info = {m: {"남은오차(카운트)": cmd[m] - pos[m], "움직임": moving[m], "부하": load[m]}
                        for m in motors if abs(cmd[m] - pos[m]) > CORR_TOL or moving[m]}
                why = f"막힘 {blocked} ({STALL_S}s 동안 진행 없음)" if blocked else f"시간 상한 {timeout:.1f}s 초과"
                raise ArmStop(f"{why} — 목표에 못 간 관절: {info} "
                              "(책상/블록/케이블/카메라 스탠드에 걸렸는지 확인)")
            # 통과 경유점: 멈출 때까지 기다리지 않고 목표 근처에 오면 바로 다음 목표로 (11/17: 경유점마다 정지해 느림)
            if pass_tol is not None and all(abs(cmd[m] - pos[m]) <= pass_tol for m in motors):
                return pos, t
            if not any(moving[m] for m in motors if m not in deadband) and \
                    t > max(0.05, START_TIMEOUT_S if not all(started.values()) else 0):
                still_since = still_since if still_since is not None else t
                if t - still_since >= SETTLE_S:
                    return pos, t
            else:
                still_since = None
            time.sleep(0.01)

    def move_raw(self, goal, correct=True, result_tol=None, pass_tol=None):
        """큰 관절 이동은 CHUNK_COUNTS 이하 조각으로 나눠 멈추지 않고 이어 감.
        11/17: 한 번에 600카운트 이상 보내면 관절이 ~80/s로 기어가는 현상이 반복됨(원인 미확정, 펌웨어 추정).
        짧은 이동(오전 2cm 경유점, s5d 15°)은 항상 정상 → 짧게 나눠 보냄."""
        start = self._read("Present_Position", list(goal))
        far = max(abs(goal[m] - start[m]) for m in goal)
        n = int(math.ceil(far / CHUNK_COUNTS))
        self._trace_id = getattr(self, "_trace_id", 0) + 1
        t0, logs = time.monotonic(), []
        for k in range(1, n):
            sub = {m: int(round(start[m] + (goal[m] - start[m]) * k / n)) for m in goal}
            r = self._move_raw_once(sub, correct=False, pass_tol=PASS_TOL_CHUNK)
            logs += r.log
            if not r.status.startswith("도착"):
                r.target = dict(goal)
                return r
        r = self._move_raw_once(goal, correct=correct, result_tol=result_tol, pass_tol=pass_tol)
        r.log = logs + r.log
        r.seconds = time.monotonic() - t0
        self.last_log = r.log
        return r

    def _move_raw_once(self, goal, correct=True, result_tol=None, pass_tol=None):
        """goal: {관절: raw 목표}. 도착 후 남은 오차를 목표 밀기로 보정."""
        res = MoveResult(status="", target=dict(goal))
        t_all = time.monotonic()
        try:
            start = self._read("Present_Position", list(goal))
            expected = self._set_speeds(start, goal)
            cmd = dict(goal)
            pos, _ = self._watch(cmd, res, "move", True, expected * 3 + 5.0, pass_tol=pass_tol)
            res.err_before = {m: goal[m] - pos[m] for m in goal}
            if correct:
                for i in range(CORR_ITERS):
                    err = {m: goal[m] - pos[m] for m in goal}
                    need = {m: e for m, e in err.items() if abs(e) > CORR_TOL}
                    if not need:
                        break
                    for m, e in need.items():
                        adj = max(-CORR_MAX_ADJ, min(CORR_MAX_ADJ, cmd[m] + e - goal[m]))
                        cmd[m] = goal[m] + adj
                    for m in goal:
                        self.bus.write("Goal_Velocity", m, CORR_SPEED, normalize=False, num_retry=RETRY)
                    pos, _ = self._watch(cmd, res, f"corr{i+1}", False, 5.0)
                    res.iters = i + 1
            res.final = pos
            # 보정하지 않는 이동(중간 경유점)은 남은 오차를 판정하지 않음 — 중력 처짐 수십 카운트가 정상
            tol = CORR_RESULT_TOL if result_tol is None else result_tol
            left = {m: goal[m] - pos[m] for m in goal if abs(goal[m] - pos[m]) > tol} if correct else {}
            res.status = "도착" if not left else f"오차 남음: {left}"
        except ArmStop as e:
            res.final = self._read("Present_Position", list(goal))
            res.status = f"정지: {e}"
        res.seconds = time.monotonic() - t_all
        self.last_log = res.log
        tr = getattr(self, "trace", None)
        if tr is not None:   # 11/17: 실제 로봇에서 시간이 어디서 가는지 기록 (s6 --trace)
            t_abs = t_all - getattr(self, "trace_t0", t_all)
            for row in res.log:
                tr.append([getattr(self, "trace_label", ""), getattr(self, "_trace_id", 0), round(t_abs, 3)] + row)
        return res

    # ---------------------------------------------------------------- 그리퍼
    def set_gripper_direction(self, open_is_high, save=True):
        self.open_is_high = bool(open_is_high)
        if save:
            GRIPPER_CONFIG.parent.mkdir(parents=True, exist_ok=True)
            GRIPPER_CONFIG.write_text(json.dumps({"open_is_high": self.open_is_high}))

    def _gripper_raw(self, open_pct):
        if self.open_is_high is None:
            raise ArmStop("그리퍼 열림 방향 미확인 — set_gripper_direction() 먼저")
        lo, hi = self.cal["gripper"]["range_min"], self.cal["gripper"]["range_max"]
        p = max(0.0, min(100.0, open_pct)) / 100
        return int(round(lo + (p if self.open_is_high else 1 - p) * (hi - lo)))

    def gripper_open_pct(self):
        lo, hi = self.cal["gripper"]["range_min"], self.cal["gripper"]["range_max"]
        raw = int(self.bus.read("Present_Position", "gripper", normalize=False, num_retry=RETRY))
        p = (raw - lo) / (hi - lo) * 100
        return p if self.open_is_high else 100 - p

    def gripper_torque_on(self):
        """목표=현재로 맞춘 뒤 토크 ON, 최대 힘 50%로 제한(RAM)."""
        pos = int(self.bus.read("Present_Position", "gripper", normalize=False, num_retry=RETRY))
        self.bus.write("Goal_Position", "gripper", pos, normalize=False, num_retry=RETRY)
        lim = min(GRIP_TORQUE_LIMIT, self._orig_ram["gripper"]["Torque_Limit"] or GRIP_TORQUE_LIMIT)
        self.bus.write("Torque_Limit", "gripper", lim, normalize=False, num_retry=RETRY)
        self.bus.write("Goal_Velocity", "gripper", GRIP_SPEED, normalize=False, num_retry=RETRY)
        self.bus.enable_torque("gripper", num_retry=RETRY)

    def gripper_move_raw(self, goal, timeout=2.5):
        """목표로 보내고 위치가 0.25초 동안 변하지 않으면 종료. 막혀서 멈춰도 오류 아님(블록을 쥔 경우)."""
        self.bus.write("Goal_Velocity", "gripper", GRIP_SPEED, normalize=False, num_retry=RETRY)
        self.bus.write("Goal_Position", "gripper", int(goal), normalize=False, num_retry=RETRY)
        t0 = time.monotonic()
        last, still = None, None
        while True:
            t = time.monotonic() - t0
            pos = int(self.bus.read("Present_Position", "gripper", normalize=False, num_retry=RETRY))
            if last is not None and abs(pos - last) <= 1:
                still = still if still is not None else t
                if t - still >= 0.25:
                    break
            else:
                still = None
            last = pos
            if t > timeout:
                break
            time.sleep(0.02)
        load = abs(int(self.bus.read("Present_Load", "gripper", normalize=False, num_retry=RETRY)))
        return pos, load

    def gripper_open(self, open_pct=60):
        return self.gripper_move_raw(self._gripper_raw(open_pct))

    def grip(self):
        """완전히 닫는 방향으로 보내 블록에 닿아 멈추면, 그 위치에서 GRIP_SQUEEZE만 더 조여 유지.
        반환: (잡았는지, 멈춘 위치의 열림%)"""
        closed = self._gripper_raw(0)
        pos, load = self.gripper_move_raw(closed)
        if abs(pos - closed) <= GRIP_EMPTY_MARGIN:
            return False, self.gripper_open_pct()
        step = GRIP_SQUEEZE if closed > pos else -GRIP_SQUEEZE
        self.bus.write("Goal_Position", "gripper", pos + step, normalize=False, num_retry=RETRY)
        time.sleep(0.3)
        return True, self.gripper_open_pct()
