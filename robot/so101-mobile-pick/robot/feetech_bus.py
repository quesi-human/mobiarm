"""lerobot 없이 SO-101 팔 모터(Feetech STS3215)와 통신하는 최소 버스.

필요한 패키지: feetech-servo-sdk(import 이름 scservo_sdk), pyserial — 합쳐서 수백 KB.
lerobot의 FeetechMotorsBus 중 arm.py가 쓰는 부분만 같은 방식으로 옮김
(같은 통신 속도 1 Mbps, 같은 패킷 타임아웃 보정, 같은 부호 처리(sign-magnitude)).

안전장치 (다른 PC에서 맞춘 모터 설정을 바꾸면 안 됨):
  - 쓰기는 RAM 레지스터 5개만 허용: Torque_Enable, Acceleration, Goal_Position, Goal_Velocity, Torque_Limit
    → EEPROM(영점·범위·PID·ID·통신 속도 등)은 이 모듈로는 쓸 수 없음 (시도하면 바로 오류)
  - lerobot은 토크 ON/OFF 때 Lock 레지스터도 1/0으로 썼지만, 여기선 쓰지 않음 (EEPROM 잠금 상태를 건드리지 않음)
"""
from dataclasses import dataclass

import scservo_sdk as scs

BAUDRATE = 1_000_000
TIMEOUT_MS = 1000
MODEL_STS3215 = 777

# 이름: (주소, 바이트 수) — lerobot feetech/tables.py STS_SMS_SERIES_CONTROL_TABLE과 같은 값
REG = {
    "Model_Number": (3, 2),
    "Homing_Offset": (31, 2),
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),
    "Goal_Position": (42, 2),
    "Goal_Velocity": (46, 2),
    "Torque_Limit": (48, 2),
    "Lock": (55, 1),
    "Present_Position": (56, 2),
    "Present_Velocity": (58, 2),
    "Present_Load": (60, 2),
    "Present_Voltage": (62, 1),
    "Present_Temperature": (63, 1),
    "Status": (65, 1),
    "Moving": (66, 1),
    "Present_Current": (69, 2),
}
# 부호 비트 (sign-magnitude) — lerobot STS_SMS_SERIES_ENCODINGS_TABLE과 같은 값
SIGN_BIT = {"Present_Load": 10, "Homing_Offset": 11, "Goal_Position": 15, "Goal_Velocity": 15,
            "Present_Position": 15, "Present_Velocity": 15}
WRITABLE = {"Torque_Enable", "Acceleration", "Goal_Position", "Goal_Velocity", "Torque_Limit"}

SO101_MOTORS = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3,
                "wrist_flex": 4, "wrist_roll": 5, "gripper": 6}


def _decode(name, v):
    b = SIGN_BIT.get(name)
    if b is not None and v & (1 << b):
        return -(v & ((1 << b) - 1))
    return v


def _encode(name, v):
    b = SIGN_BIT.get(name)
    if b is not None and v < 0:
        return (-v) | (1 << b)
    return v


def _bytes(v, n):
    return [v & 0xFF] if n == 1 else [scs.SCS_LOBYTE(v), scs.SCS_HIBYTE(v)]


def _patched_set_packet_timeout(self, packet_length):  # noqa: N802
    # lerobot과 같은 보정 (PyPI판 SDK의 패킷 타임아웃 버그)
    self.packet_start_time = self.getCurrentTime()
    self.packet_timeout = (self.tx_time_per_byte * packet_length) + (self.tx_time_per_byte * 3.0) + 50


@dataclass
class MotorCalib:
    homing_offset: int


class FeetechBus:
    def __init__(self, port, motors=None):
        self.port = port
        self.motors = dict(motors or SO101_MOTORS)          # 이름 → ID
        self.ph = scs.PortHandler(port)
        self.ph.setPacketTimeout = _patched_set_packet_timeout.__get__(self.ph, scs.PortHandler)
        self.pk = scs.PacketHandler(0)
        self.sync_reader = scs.GroupSyncRead(self.ph, self.pk, 0, 0)
        self.connected = False

    # ------------------------------------------------------------ 연결
    def connect(self):
        if not self.ph.openPort():
            raise ConnectionError(f"포트를 열 수 없음: {self.port}")
        self.ph.setBaudRate(BAUDRATE)
        self.ph.setPacketTimeoutMillis(TIMEOUT_MS)
        self.connected = True
        missing, wrong = [], []
        for name, i in self.motors.items():
            model, comm, _ = self.pk.ping(self.ph, i)
            if comm != scs.COMM_SUCCESS:
                missing.append(f"{name}(ID{i})")
            elif model != MODEL_STS3215:
                wrong.append(f"{name}(ID{i}) 모델 {model}")
        if missing or wrong:
            self.ph.closePort()
            self.connected = False
            msg = []
            if missing:
                msg.append("응답 없는 모터: " + ", ".join(missing)
                           + " — 12V 전원, 보드→첫 모터 케이블, 보드 점퍼(B) 확인. python steps/bus_scan.py")
            if wrong:
                msg.append("모델이 다른 모터: " + ", ".join(wrong))
            raise ConnectionError("\n".join(msg))
        return self

    def disconnect(self, disable_torque=False):
        if not self.connected:
            return
        if disable_torque:
            self.disable_torque(list(self.motors), num_retry=5)
        self.ph.closePort()
        self.connected = False

    # ------------------------------------------------------------ 읽기/쓰기
    def _ids(self, motors):
        if motors is None:
            motors = list(self.motors)
        if isinstance(motors, str):
            motors = [motors]
        return [(m, self.motors[m]) for m in motors]

    def read(self, reg, motor, normalize=False, num_retry=0):
        addr, n = REG[reg]
        fn = self.pk.read1ByteTxRx if n == 1 else self.pk.read2ByteTxRx
        mid = self.motors[motor]
        for _ in range(1 + num_retry):
            v, comm, err = fn(self.ph, mid, addr)
            if comm == scs.COMM_SUCCESS:
                break
        if comm != scs.COMM_SUCCESS:
            raise ConnectionError(f"'{reg}' 읽기 실패 ({motor}, ID{mid}) {self.pk.getTxRxResult(comm)}")
        if err:
            raise RuntimeError(f"'{reg}' 읽기 중 모터 오류 ({motor}, ID{mid}) {self.pk.getRxPacketError(err)}")
        return _decode(reg, v)

    def sync_read(self, reg, motors=None, normalize=False, num_retry=0):
        addr, n = REG[reg]
        ids = self._ids(motors)
        self.sync_reader.clearParam()
        self.sync_reader.start_address = addr
        self.sync_reader.data_length = n
        for _, i in ids:
            self.sync_reader.addParam(i)
        for _ in range(1 + num_retry):
            comm = self.sync_reader.txRxPacket()
            if comm == scs.COMM_SUCCESS:
                break
        if comm != scs.COMM_SUCCESS:
            raise ConnectionError(f"'{reg}' 동시 읽기 실패 {[m for m, _ in ids]} {self.pk.getTxRxResult(comm)}")
        return {m: _decode(reg, self.sync_reader.getData(i, addr, n)) for m, i in ids}

    def write(self, reg, motor, value, normalize=False, num_retry=0):
        if reg not in WRITABLE:
            raise PermissionError(f"'{reg}' 쓰기 금지 — 이 버스는 RAM 레지스터 {sorted(WRITABLE)}만 씀")
        addr, n = REG[reg]
        mid = self.motors[motor]
        data = _bytes(_encode(reg, int(value)), n)
        for _ in range(1 + num_retry):
            comm, err = self.pk.writeTxRx(self.ph, mid, addr, n, data)
            if comm == scs.COMM_SUCCESS:
                break
        if comm != scs.COMM_SUCCESS:
            raise ConnectionError(f"'{reg}'={value} 쓰기 실패 ({motor}, ID{mid}) {self.pk.getTxRxResult(comm)}")
        if err:
            raise RuntimeError(f"'{reg}'={value} 쓰기 중 모터 오류 ({motor}, ID{mid}) {self.pk.getRxPacketError(err)}")

    # ------------------------------------------------------------ 토크·캘리브레이션
    def enable_torque(self, motors=None, num_retry=0):
        for m, _ in self._ids(motors):
            self.write("Torque_Enable", m, 1, num_retry=num_retry)

    def disable_torque(self, motors=None, num_retry=0):
        for m, _ in self._ids(motors):
            self.write("Torque_Enable", m, 0, num_retry=num_retry)

    def read_calibration(self):
        """모터에 기록된 영점만 읽음 (확인용, 쓰지 않음)."""
        return {m: MotorCalib(int(self.read("Homing_Offset", m, num_retry=3))) for m in self.motors}
