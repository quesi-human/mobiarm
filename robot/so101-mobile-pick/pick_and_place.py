#!/usr/bin/env python3
"""SO-ARM worker for soarm_mission robot_bridge (teammate package) — 줄 단위 JSON, stdin 명령 → stdout 결과. 11/26

robot_bridge가 이 파일을 한 번 실행해 두고(start 때) place → shutdown까지 같은 프로세스를 씀:
  ros2 run soarm_mission robot_bridge --worker ~/so101-mobile-pick/pick_and_place.py \
      --worker-python ~/arm-venv/bin/python ...

입력 (stdin, 한 줄에 JSON 하나):
  {"type": "command", "mission_id", "request_id", "boot_id", "action": start|pick|place|shutdown,
   "station_id", "color": red|blue|green|yellow, "timeout_sec", "max_attempts"}
  {"type": "mission_state", "mission_id", "phase", "flag"}      (bridge가 1초마다 전달)
출력 (stdout, 한 줄에 JSON 하나 — 로그는 전부 stderr):
  {"mission_id", "request_id", "action", "flag": busy|done|failed, "holding": bool,
   "travel_ready": bool(pick), "mock": false, "error"?, "arm_result"?}

동작:
  start     장치·보정 확인 → 접힌 자세에서 펴서 보는 자세                     → done, holding=false
  pick      color 블록을 좌우 60° 둘러보며 찾아 집기(max_attempts번까지)      → done, holding=true, travel_ready=true
            → 운반 자세로 들고 정지. 실패(not_found/too_far/missed)면 접힌 자세로 → failed (error에 이유, arm_result에 위치)
  place     내려놓는 자세에서 그리퍼 열기 → 접힌 자세                          → done, holding=false
  shutdown  (빈손·접힌 자세에서) 토크 OFF → done 후 종료. 블록을 들고 있으면 failed
안전:
  - 각 명령은 timeout_sec - 3초 안에 끝냄. 넘으면 그 자리에 멈추고 failed("제한 시간 초과").
  - mission_state가 flag=uncertain 또는 phase=halt_and_hold → 진행 중인 동작을 다음 단계 사이에서 멈추고 유지.
  - 명령 처리 중 mission_state가 STATE_TIMEOUT_S 넘게 끊기면 같은 방식으로 멈춤. (손으로 시험할 땐 --no-watchdog)
  - stdin이 끊겨도(EOF) 블록을 들고 있으면 토크를 끄지 않고 그대로 유지하며 프로세스도 끝내지 않음(작업자 복구 대기).
    빈손이면 접힌 자세로 가서 토크 OFF 후 종료.
"""
import json
import queue
import sys
import threading
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
OUT = sys.stdout
sys.stdout = sys.stderr                 # 우리 모듈의 print는 전부 stderr로 → stdout은 JSON만

STATE_TIMEOUT_S = 5.0
TIMEOUT_MARGIN_S = 3.0
COLORS = ("red", "blue", "green", "yellow")


def log(*a):
    print("[worker]", *a, file=sys.stderr, flush=True)


def emit(msg):
    OUT.write(json.dumps(msg, ensure_ascii=False) + "\n")
    OUT.flush()


class Inbox:
    def __init__(self):
        self.commands = queue.Queue()
        self.last_state = None
        self.last_state_t = None
        self.eof = threading.Event()
        self.on_halt = lambda reason: None
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                m = json.loads(line)
            except json.JSONDecodeError:
                log(f"JSON 아님, 무시: {line[:120]}")
                continue
            if m.get("type") == "mission_state":
                self.last_state, self.last_state_t = m, time.monotonic()
                if m.get("flag") == "uncertain" or m.get("phase") == "halt_and_hold":
                    self.on_halt(f"mission_state {m.get('phase')}/{m.get('flag')}")
            elif m.get("type") == "command":
                self.commands.put(m)
            else:
                log(f"모르는 type, 무시: {m.get('type')}")
        self.eof.set()
        self.commands.put(None)


def main():
    inbox = Inbox()
    tasks = None
    busy = threading.Event()

    def halt(reason):
        if busy.is_set() and tasks is not None:
            log(f"정지 요청: {reason}")
            tasks.request_stop()

    inbox.on_halt = halt

    use_watchdog = "--no-watchdog" not in sys.argv[1:]     # 손으로 시험할 때만 끔

    def watchdog():                      # 명령 처리 중 상태 알림이 끊기면 멈춤
        while use_watchdog:
            time.sleep(0.5)
            if busy.is_set() and inbox.last_state_t is not None and \
                    time.monotonic() - inbox.last_state_t > STATE_TIMEOUT_S:
                halt(f"mission_state {STATE_TIMEOUT_S:.0f}s 끊김")
                inbox.last_state_t = time.monotonic()     # 한 번만 알림
    threading.Thread(target=watchdog, daemon=True).start()

    log("시작 — 명령 대기")
    while True:
        m = inbox.commands.get()
        if m is None:
            break
        action = m.get("action")
        base = {"mission_id": m.get("mission_id"), "request_id": m.get("request_id"), "action": action, "mock": False}

        def reply(flag, **kw):
            holding = bool(tasks.holding) if tasks is not None else False
            emit({**base, "flag": flag, "holding": holding, **kw})

        reply("busy")
        busy.set()
        t0 = time.monotonic()
        try:
            timeout = float(m.get("timeout_sec") or 60.0)
            deadline = t0 + max(5.0, timeout - TIMEOUT_MARGIN_S)
            if action == "start":
                if tasks is None:
                    from robot import devices as DEV
                    from robot.tasks import ArmTasks
                    tasks = ArmTasks(DEV.ARM_PORT, DEV.WRIST_CAM)
                tasks.deadline = deadline
                tasks.ready()
                reply("done", travel_ready=False)
            elif tasks is None:
                reply("failed", error="start가 먼저 필요")
            elif action == "pick":
                color = m.get("color")
                if color not in COLORS:
                    raise ValueError(f"지원하지 않는 색: {color}")
                attempts = max(1, min(3, int(m.get("max_attempts") or 1)))
                tasks.deadline = deadline
                res = None
                for k in range(attempts):
                    if k:
                        log(f"다시 집기 {k + 1}/{attempts} (이전: {res['result']})")
                    res = tasks.pick(color)
                    if res["result"] in ("picked", "not_found", "too_far"):
                        break                     # not_found·too_far는 팔이 다시 해도 같음 → 터틀봇이 움직여야 함
                if res["result"] == "picked":
                    reply("done", travel_ready=True, arm_result=res)
                else:
                    tasks.deadline = None         # 실패 정리(접기)는 시간 제한 없이
                    try:
                        tasks.park()
                    except Exception as e:
                        log(f"접기 실패: {e}")
                    reply("failed", travel_ready=False, error=f"pick {res['result']}", arm_result=res)
            elif action == "place":
                tasks.deadline = deadline
                res = tasks.place()
                reply("done", travel_ready=False, arm_result=res)
            elif action == "shutdown":
                if tasks.holding:
                    reply("failed", error="블록을 들고 있음 — place 먼저")
                else:
                    if tasks.pose not in ("park",):
                        tasks.deadline = deadline
                        tasks.park()
                    tasks.arm.close(disable_torque=True)   # 접힌 자세에서 토크 OFF (몇 도 내려앉을 수 있음)
                    if tasks.cam is not None:
                        tasks.cam.stop()
                    reply("done", travel_ready=False)
                    busy.clear()
                    log("shutdown 완료 — 종료")
                    return
            else:
                reply("failed", error=f"모르는 action: {action}")
        except Exception as e:
            if not type(e).__name__ in ("ArmStop", "Stopped"):
                traceback.print_exc(file=sys.stderr)
            reply("failed", error=f"{type(e).__name__}: {e}", seconds=round(time.monotonic() - t0, 1))
        finally:
            busy.clear()
            if tasks is not None:
                tasks.deadline = None
        log(f"{action} 끝 ({time.monotonic() - t0:.1f}s)")

    # stdin EOF
    if tasks is None:
        return
    if tasks.holding:
        log("stdin 끊김 — 블록을 들고 있어 토크를 켠 채 그대로 유지합니다 (작업자 복구 대기, Ctrl+C·kill로 종료)")
        tasks.halt()
        while True:
            time.sleep(1.0)
    log("stdin 끊김 — 빈손이라 접힌 자세로 가서 토크 OFF 후 종료")
    tasks.shutdown()


if __name__ == "__main__":
    main()
