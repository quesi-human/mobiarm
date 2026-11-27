"""노트북용 팔 클라이언트 (파이썬 기본 라이브러리만). 작업 제어기에서 import 하거나 명령줄로 시험.

  python tools/arm_client.py --host 192.168.0.176 status
  python tools/arm_client.py --host 192.168.0.176 pick yellow
  python tools/arm_client.py --host 192.168.0.176 place
  python tools/arm_client.py --host 192.168.0.176 park
  python tools/arm_client.py --host 192.168.0.176 stop

작업 제어기에서:
  from arm_client import ArmClient
  arm = ArmClient("192.168.0.176")
  r = arm.pick("yellow")          # 끝날 때까지 기다림 → {"state": "done", "result": {"result": "picked", ...}}
  if r["state"] == "done" and r["result"]["result"] == "too_far": ... r["result"]["fwd_m"], ["left_m"]
"""
import argparse
import json
import sys
import time
import urllib.error
import urllib.request


class ArmError(RuntimeError):
    pass


class ArmClient:
    def __init__(self, host, port=8000, timeout_s=5.0):
        self.base = f"http://{host}:{port}"
        self.timeout_s = timeout_s
        self._next_id = int(time.time())

    def _req(self, method, path, body=None):
        data = json.dumps(body or {}).encode() if method == "POST" else None
        req = urllib.request.Request(self.base + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            return {"http_error": e.code, **json.loads(e.read() or b"{}")}
        except urllib.error.URLError as e:
            raise ArmError(f"팔 서버에 연결 안 됨 ({self.base}): {e.reason}")

    def status(self):
        return self._req("GET", "/status")

    def stop(self):
        return self._req("POST", "/stop")

    def run(self, cmd, wait_s=120.0, on_stage=None, **params):
        """명령 보내고 끝날 때까지 기다림. 반환: 마지막 status."""
        self._next_id += 1
        job = self._next_id
        r = self._req("POST", f"/{cmd}", {"id": job, **params})
        if not r.get("accepted"):
            raise ArmError(f"{cmd} 거절: {r}")
        t0, last_stage = time.monotonic(), None
        while time.monotonic() - t0 < wait_s:
            s = self.status()
            if s.get("id") == job and s.get("state") in ("done", "failed"):
                return s
            if on_stage and s.get("stage") != last_stage:
                last_stage = s.get("stage")
                on_stage(last_stage)
            time.sleep(0.5)
        self.stop()
        raise ArmError(f"{cmd} 시간 초과 ({wait_s:.0f}s) — stop 보냄")

    def pick(self, color="yellow", **kw):
        return self.run("pick", color=color, **kw)

    def place(self, **kw):
        return self.run("place", **kw)

    def park(self, **kw):
        return self.run("park", **kw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True, help="라즈베리파이 IP")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("cmd", choices=["status", "pick", "place", "park", "stop"])
    ap.add_argument("color", nargs="?", default="yellow")
    a = ap.parse_args()
    c = ArmClient(a.host, a.port)
    if a.cmd == "status":
        out = c.status()
    elif a.cmd == "stop":
        out = c.stop()
    else:
        params = {"color": a.color} if a.cmd == "pick" else {}
        out = c.run(a.cmd, on_stage=lambda s: print(f"  … {s}"), **params)
    print(json.dumps(out, ensure_ascii=False, indent=2))
    sys.exit(0 if out.get("state") != "failed" else 1)


if __name__ == "__main__":
    main()
