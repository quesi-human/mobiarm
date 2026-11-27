"""팔 HTTP 서버 (라즈베리파이에서 실행). 노트북의 작업 제어기가 JSON으로 명령을 보냄. 파이썬 기본 라이브러리만 씀.

  ~/arm-venv/bin/python arm_server.py [--port 8000]

명령 (POST, 본문 JSON — 바로 '받았음'으로 답하고 팔은 뒤에서 일함):
  /pick   {"color": "yellow", "id": 7}   둘러보고 집어서 운반 자세로
  /place  {"id": 8}                       내려놓는 자세에서 놓고 접힌 자세로
  /park   {"id": 9}                       접힌 자세로
  /stop   {}                              다음 단계 사이에서 멈춤 (그 자리 유지)
상태 (GET):
  /status → {"id", "cmd", "state": idle|running|done|failed, "stage", "result": {...}, "error",
             "holding", "pose", "gripper_pct", "time"}
  일하는 중에 다른 명령을 보내면 409 {"error": "busy"}.

pick 결과(result.result):
  picked     집어서 운반 자세. 이제 목적지로 이동하면 됨
  not_found  좌우 60°를 둘러봤지만 없음 → 터틀봇이 돌거나 옮긴 뒤 다시 pick
  too_far    보이지만 팔이 안 닿음. fwd_m/left_m(팔 회전축 기준, 터틀봇 앞/왼쪽 +), dist_m, bearing_deg 를 줌
             (mount.json에 pan_axis_in_base_m을 넣으면 base_fwd_m/base_left_m도 — 터틀봇 중심 기준)
  missed     내려갔지만 못 쥠 → 다시 pick

같은 와이파이 안에서만 쓰기 (인증 없음).
"""
import argparse
import json
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from robot import devices as DEV
from robot.arm import ArmStop
from robot.tasks import ArmTasks, Stopped

STATE = {"id": None, "cmd": None, "state": "idle", "stage": None, "result": None, "error": None}
LOCK = threading.Lock()
TASKS = None


def set_state(**kw):
    with LOCK:
        STATE.update(kw)


def status():
    with LOCK:
        s = dict(STATE)
    s.update({"holding": TASKS.holding, "pose": TASKS.pose, "time": time.time()})
    if s["state"] != "running":
        s["gripper_pct"] = TASKS.gripper_pct()
    return s


def run_job(cmd, job_id, fn):
    t0 = time.monotonic()
    print(f"[{job_id}] {cmd} 시작")
    try:
        res = fn()
        set_state(state="done", result=res, stage=None, seconds=round(time.monotonic() - t0, 1))
        print(f"[{job_id}] {cmd} 끝: {res}")
    except Stopped as e:
        set_state(state="failed", error=str(e), stage=None)
        print(f"[{job_id}] {cmd} 중지: {e}")
    except ArmStop as e:
        set_state(state="failed", error=str(e), stage=None)
        print(f"[{job_id}] {cmd} 실패: {e}")
    except Exception as e:
        traceback.print_exc()
        set_state(state="failed", error=f"{type(e).__name__}: {e}", stage=None)


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):          # /status 반복 요청으로 화면이 넘치지 않게
        if "/status" not in (args[0] if args else ""):
            super().log_message(fmt, *args)

    def do_GET(self):
        if self.path.startswith("/status"):
            return self._send(200, status())
        self._send(404, {"error": "없는 주소", "paths": ["/status", "/pick", "/place", "/park", "/stop"]})

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        try:
            req = json.loads(self.rfile.read(n) or b"{}")
        except json.JSONDecodeError:
            return self._send(400, {"error": "JSON 형식이 아님"})
        path = self.path.rstrip("/")
        if path == "/stop":
            TASKS.request_stop()
            return self._send(200, {"accepted": True})
        jobs = {"/pick": lambda: TASKS.pick(req.get("color", "yellow")),
                "/place": TASKS.place,
                "/park": TASKS.park}
        if path not in jobs:
            return self._send(404, {"error": "없는 주소"})
        with LOCK:
            if STATE["state"] == "running":
                return self._send(409, {"error": "busy", "id": STATE["id"], "cmd": STATE["cmd"]})
            job_id = req.get("id", int(time.time() * 1000))
            STATE.update(id=job_id, cmd=path[1:], state="running", stage="시작", result=None, error=None)
        threading.Thread(target=run_job, args=(path[1:], job_id, jobs[path]), daemon=True).start()
        self._send(202, {"accepted": True, "id": job_id})


def main():
    global TASKS
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--arm-port", default=DEV.ARM_PORT)
    ap.add_argument("--cam", default=DEV.WRIST_CAM)
    args = ap.parse_args()

    TASKS = ArmTasks(args.arm_port, args.cam)
    TASKS.stage_cb = lambda s: set_state(stage=s)
    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"[팔 서버] http://{args.host}:{args.port}  (pose={TASKS.pose}, Ctrl+C = 접고 끝)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[종료] 접힌 자세로 가서 끝냅니다")
    finally:
        srv.server_close()
        if STATE["state"] == "running":          # 일하던 중이면 먼저 멈추고 기다림 (같은 버스를 동시에 쓰지 않게)
            TASKS.request_stop()
            for _ in range(150):
                if STATE["state"] != "running":
                    break
                time.sleep(0.1)
        TASKS.shutdown()


if __name__ == "__main__":
    main()
