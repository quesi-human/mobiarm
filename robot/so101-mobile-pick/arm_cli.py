"""팔 명령 실행기 — 터틀봇 작업 제어기가 필요할 때 이 파일을 실행(또는 import)해서 팔을 움직임. 11/25

명령줄 (작업 제어기에서 subprocess로):
  ~/arm-venv/bin/python ~/so101-mobile-pick/arm_cli.py pick --color yellow
  ~/arm-venv/bin/python ~/so101-mobile-pick/arm_cli.py place
  ~/arm-venv/bin/python ~/so101-mobile-pick/arm_cli.py park
  ~/arm-venv/bin/python ~/so101-mobile-pick/arm_cli.py status

  - 동작이 끝나면 **표준출력에 JSON 한 줄**을 내고 끝남. 진행 로그는 표준에러로.
      {"ok": true, "cmd": "pick", "result": "picked", "fwd_m": 0.21, ...}
      {"ok": false, "cmd": "pick", "result": "not_found", ...}
      {"ok": false, "cmd": "place", "result": "error", "error": "..."}
  - 종료 코드: 0 = 성공(picked/placed/parked), 1 = 실패 결과(not_found/too_far/missed), 2 = 오류
  - pick 결과 picked: 블록을 든 채 운반 자세로 **토크를 켠 채 프로그램만 끝남** → 터틀봇 이동 → 나중에 place 실행.
    들고 있는지·자세는 state/arm_state.json에 남겨 다음 실행이 이어받음.
  - 동시에 두 개를 실행하면 두 번째는 바로 오류(종료 코드 2, "busy").

파이썬에서 직접 (작업 제어기가 같은 venv 파이썬일 때):
  from arm_cli import run
  r = run("pick", color="yellow")     # 위와 같은 dict

pick 결과:
  picked     집어서 운반 자세로 들고 있음 → 목적지로 이동 후 place
  not_found  좌우 60°를 봐도 없음 → 터틀봇을 돌리거나 옮긴 뒤 다시 pick
  too_far    보이지만 안 닿음 → fwd_m/left_m(팔 회전축 기준, 앞·왼쪽 +, m)만큼 다가간 뒤 다시 pick
  missed     못 쥠 (스스로 --retries번 더 해 본 뒤) → 다시 pick
"""
import argparse
import contextlib
import fcntl
import os
import json
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
STATE_FILE = ROOT / "state/arm_state.json"
LOCK_FILE = ROOT / "state/arm.lock"
EXIT = {"picked": 0, "placed": 0, "parked": 0, "not_found": 1, "too_far": 1, "missed": 1}


def _load_state():
    try:
        return json.loads(STATE_FILE.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def _save_state(tasks):
    STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
    tmp = STATE_FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps({"holding": tasks.holding, "pose": tasks.pose, "time": time.time()}), encoding="utf-8")
    tmp.replace(STATE_FILE)


def run(cmd, color="yellow", retries=2, arm_port=None, cam=None):
    """팔 명령 하나 실행 → 결과 dict. 진행 로그는 표준에러로."""
    from robot import devices as DEV
    from robot.arm import ArmStop

    LOCK_FILE.parent.mkdir(parents=True, exist_ok=True)
    lock = open(LOCK_FILE, "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return {"ok": False, "cmd": cmd, "result": "error", "error": "busy — 다른 팔 명령이 실행 중"}

    if cmd == "status":
        st = _load_state()
        return {"ok": True, "cmd": cmd, "result": "status", **st}

    from robot.tasks import ArmTasks
    tasks = None
    out = {"cmd": cmd}
    try:
        with contextlib.redirect_stdout(sys.stderr):
            tasks = ArmTasks(arm_port or DEV.ARM_PORT, cam or DEV.WRIST_CAM)
            st = _load_state()
            torque_on = tasks._torque_is_on()
            if torque_on:                               # 이전 실행이 블록을 든 채 끝남 → 이어받기
                tasks.holding = bool(st.get("holding"))
                tasks.pose = st.get("pose", "unknown")
            t0 = time.monotonic()
            if cmd == "pick":
                res = tasks.pick(color)
                for k in range(retries):
                    if res["result"] != "missed":
                        break
                    print(f"[다시 집기 {k+1}/{retries}] 못 쥠 → 다시 둘러보고 집기")
                    res = tasks.pick(color)
            elif cmd == "place":
                res = tasks.place()
            elif cmd == "park":
                res = tasks.park()
            else:
                raise ValueError(f"모르는 명령: {cmd}")
            out.update(res)
            out["seconds"] = round(time.monotonic() - t0, 1)
            out["ok"] = EXIT.get(res["result"], 2) == 0
    except ArmStop as e:
        out.update({"ok": False, "result": "error", "error": str(e)})
    except Exception as e:
        traceback.print_exc(file=sys.stderr)
        out.update({"ok": False, "result": "error", "error": f"{type(e).__name__}: {e}"})
    finally:
        if tasks is not None:
            with contextlib.redirect_stdout(sys.stderr):
                try:
                    out["holding"] = tasks.holding
                    out["pose"] = tasks.pose
                    _save_state(tasks)
                    if tasks.holding:
                        tasks.detach()                  # 블록을 든 채 → 토크 켠 채로 끝 (떨어뜨리지 않게)
                    elif out.get("result") == "error":
                        tasks.detach()                  # 오류 → 그 자리 유지 (사람이 확인)
                    else:
                        tasks.shutdown()                # 빈손 → 접힌 자세로 가서 토크 OFF
                        out["pose"] = "park"
                        tasks.pose = "park"
                        _save_state(tasks)
                except Exception as e:
                    print(f"[종료 처리 오류] {e}")
        lock.close()
    return out


def main():
    ap = argparse.ArgumentParser(description="팔 명령 실행 (끝나면 표준출력에 JSON 한 줄)")
    ap.add_argument("cmd", choices=["pick", "place", "park", "status"])
    ap.add_argument("--color", default="yellow")
    ap.add_argument("--retries", type=int, default=2)
    ap.add_argument("--arm-port")
    ap.add_argument("--cam")
    a = ap.parse_args()
    out = run(a.cmd, a.color, a.retries, a.arm_port, a.cam)
    print(json.dumps(out, ensure_ascii=False), flush=True)
    code = EXIT.get(out.get("result"), 0 if out.get("ok") else 2) if a.cmd != "status" else 0
    sys.stderr.flush()
    os._exit(code)        # 카메라 드라이버에 걸린 스레드가 남아 있어도 바로 끝나게


if __name__ == "__main__":
    main()
