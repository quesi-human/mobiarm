"""팔 에이전트 (라즈베리파이에서 실행) — 팀원의 미션 상태 서버(노트북)를 보고 집기·내려놓기를 함.

  ~/arm-venv/bin/python arm_agent.py --status-url http://192.168.110.113:8080/api/status

상태 서버 (팀원 server.py): GET/POST /api/status, 본문 {"mission_id", "phase", "flag": busy|done|failed|uncertain}.
마지막 상태 하나만 보관하고 3초(ttl) 안에 다시 POST가 없으면 stale(503).

주고받기 약속 (phase 이름은 옵션으로 바꿀 수 있음 — 팀원과 맞출 것):
  작업 제어기 → 팔                       팔 → 작업 제어기 (팔이 일하는 동안은 팔이 1초마다 POST)
  phase=ws_ready,   flag=done     →     arm_picking/busy … → arm_picked/done
                                                            arm_pick_not_found/failed   (좌우 60° 봐도 없음)
                                                            arm_pick_too_far/failed     (보이지만 안 닿음)
                                                            arm_pick_missed/failed      (못 쥠)
                                                            arm_error/failed            (팔 오류·중지)
  phase=dest_ready, flag=done     →     arm_placing/busy … → arm_placed/done
  phase=arm_park,   flag=busy     →     arm_parking/busy … → arm_parked/done

규칙:
  - 팔은 '트리거 상태'를 볼 때 한 번만 움직임. 끝 상태(arm_*)를 쓴 뒤 작업 제어기가 같은 트리거를 다시 쓰면 다시 함
    (예: arm_pick_missed → 작업 제어기가 ws_ready/done 다시 → 다시 집기). 11/25
  - 못 쥐면(missed) 팔이 알아서 --retries번(기본 2) 더 둘러보고 집어 봄. 그래도 안 되면 arm_pick_missed.
  - 에이전트를 켜기 전에 써 둔 오래된(stale) 상태로는 움직이지 않음 (켜자마자 갑자기 움직이지 않게).
  - 팔이 일하는 동안 작업 제어기는 POST를 멈춰야 함 (안 그러면 팔의 busy를 덮어씀). 단계 넘기기 = 상태 쓰기 권한 넘기기.
  - 팔은 끝 상태(arm_picked 등)를 작업 제어기가 다음 상태를 쓸 때까지(최대 --final-hold-s) 1초마다 다시 POST → stale 방지.
  - too_far 때 방향·거리는 상태에 넣지 않고(서버 형식 유지) 로그에 남김. 필요하면 --extra 로 상태에 함께 보냄.
"""
import argparse
import json
import threading
import time
import traceback
import urllib.error
import urllib.request

from robot import devices as DEV
from robot.arm import ArmStop
from robot.tasks import ArmTasks

PICK_RESULT_PHASE = {"picked": ("arm_picked", "done"), "not_found": ("arm_pick_not_found", "failed"),
                     "too_far": ("arm_pick_too_far", "failed"), "missed": ("arm_pick_missed", "failed")}


class StatusLink:
    def __init__(self, url, timeout_s=2.0):
        self.url = url
        self.timeout_s = timeout_s

    def get(self):
        """반환 (mission_id, phase, flag, fresh) — 상태 없음/오래됨이면 phase=None."""
        try:
            with urllib.request.urlopen(self.url, timeout=self.timeout_s) as r:
                body = json.loads(r.read() or b"{}")
        except urllib.error.HTTPError as e:          # 503 = 아직 상태 없음(waiting) 또는 오래됨(stale)
            try:
                body = json.loads(e.read() or b"{}")
            except json.JSONDecodeError:
                body = {}
        except (urllib.error.URLError, OSError, json.JSONDecodeError):
            return None, None, None, False
        st = body.get("state") or {}
        if not isinstance(st, dict):
            st = {}
        return st.get("mission_id"), st.get("phase"), st.get("flag"), bool(body.get("fresh"))

    def post(self, mission_id, phase, flag, extra=None):
        data = {"mission_id": mission_id, "phase": phase, "flag": flag, **(extra or {})}
        req = urllib.request.Request(self.url, data=json.dumps(data).encode("utf-8"),
                                     headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as r:
                r.read()
            return True
        except (urllib.error.URLError, OSError) as e:
            print(f"[상태 서버] POST 실패: {e}")
            return False


class Reporter:
    """팔이 일하는 동안 busy를 1초마다, 끝나면 끝 상태를 작업 제어기가 넘겨받을 때까지 1초마다 POST."""

    def __init__(self, link, period_s, final_hold_s, extra_ok):
        self.link, self.period_s, self.final_hold_s, self.extra_ok = link, period_s, final_hold_s, extra_ok
        self.cur = None            # (mission_id, phase, flag, extra)
        self.final_until = 0.0
        self.lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    def busy(self, mission_id, phase):
        with self.lock:
            self.cur, self.final_until = (mission_id, phase, "busy", None), float("inf")
        self.link.post(mission_id, phase, "busy")

    def final(self, mission_id, phase, flag, extra=None):
        extra = extra if self.extra_ok else None
        with self.lock:
            self.cur, self.final_until = (mission_id, phase, flag, extra), time.monotonic() + self.final_hold_s
        self.link.post(mission_id, phase, flag, extra)

    def release(self):
        with self.lock:
            self.cur = None

    def _loop(self):
        while True:
            time.sleep(self.period_s)
            with self.lock:
                cur, until = self.cur, self.final_until
            if cur is None:
                continue
            if time.monotonic() > until:
                print(f"[상태] {cur[1]}를 {self.final_hold_s:.0f}s 보냈지만 작업 제어기가 넘겨받지 않음 — 보내기 중단")
                self.release()
                continue
            mid, ph, fl, ex = cur
            # 끝 상태를 보내는 중 작업 제어기가 다른 상태를 썼으면 넘겨받은 것 → 그만 보냄
            if fl != "busy":
                m2, p2, _, _ = self.link.get()
                if p2 is not None and (m2, p2) != (mid, ph):
                    self.release()
                    continue
            self.link.post(mid, ph, fl, ex)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--status-url", required=True, help="예: http://192.168.110.113:8080/api/status")
    ap.add_argument("--color", default="yellow")
    ap.add_argument("--pick-on", default="ws_ready", help="이 phase + flag=done 이면 집기")
    ap.add_argument("--place-on", default="dest_ready", help="이 phase + flag=done 이면 내려놓기")
    ap.add_argument("--park-on", default="arm_park", help="이 phase 이면 접기")
    ap.add_argument("--poll-s", type=float, default=0.5)
    ap.add_argument("--period-s", type=float, default=1.0, help="busy·끝 상태 다시 보내는 간격 (서버 ttl 3초보다 짧게)")
    ap.add_argument("--final-hold-s", type=float, default=30.0)
    ap.add_argument("--retries", type=int, default=2, help="못 쥐었을 때(missed) 팔이 스스로 다시 집어 보는 횟수")
    ap.add_argument("--extra", action="store_true", help="결과 상세(too_far 방향·거리 등)를 상태에 함께 POST")
    ap.add_argument("--arm-port", default=DEV.ARM_PORT)
    ap.add_argument("--cam", default=DEV.WRIST_CAM)
    args = ap.parse_args()

    link = StatusLink(args.status_url)
    tasks = ArmTasks(args.arm_port, args.cam)
    rep = Reporter(link, args.period_s, args.final_hold_s, args.extra)
    handled = None            # 마지막으로 처리한 (mission_id, phase) — 다른 상태를 거쳐 다시 오면 다시 처리
    print(f"[팔 에이전트] {args.status_url} 감시 중 — 집기: {args.pick_on}/done, 내려놓기: {args.place_on}/done, "
          f"접기: {args.park_on}  (Ctrl+C = 접고 끝)")
    last_seen = None
    try:
        while True:
            mid, phase, flag, fresh = link.get()
            if (mid, phase, flag) != last_seen:
                print(f"[상태] mission={mid} phase={phase} flag={flag} fresh={fresh}")
                last_seen = (mid, phase, flag)
            key = (mid, phase)
            if handled is not None and phase is not None and key != handled:
                handled = None            # 다른 상태(팔의 끝 상태 포함)를 봄 → 같은 트리거가 다시 오면 다시 처리
            job = None
            if fresh and key != handled:
                if phase == args.pick_on and flag == "done":
                    job = ("pick", "arm_picking")
                elif phase == args.place_on and flag == "done":
                    job = ("place", "arm_placing")
                elif phase == args.park_on:
                    job = ("park", "arm_parking")
            if job is None:
                time.sleep(args.poll_s)
                continue
            handled = key
            cmd, busy_phase = job
            print(f"[{mid}] {cmd} 시작 ({phase})")
            rep.busy(mid, busy_phase)
            tasks.stage_cb = lambda s, m=mid, p=busy_phase: print(f"  … {s}")
            try:
                if cmd == "pick":
                    res = tasks.pick(args.color)
                    for k in range(args.retries):
                        if res["result"] != "missed":
                            break
                        print(f"  [다시 집기 {k+1}/{args.retries}] 못 쥠 (그리퍼 {res.get('gripper_pct')}%) → 다시 둘러보고 집기")
                        res = tasks.pick(args.color)
                    ph, fl = PICK_RESULT_PHASE[res["result"]]
                elif cmd == "place":
                    res = tasks.place()
                    ph, fl = "arm_placed", "done"
                else:
                    res = tasks.park()
                    ph, fl = "arm_parked", "done"
                print(f"[{mid}] {cmd} 끝 → {ph}: {res}")
                rep.final(mid, ph, fl, {"arm_result": res})
            except ArmStop as e:
                print(f"[{mid}] {cmd} 실패: {e}")
                rep.final(mid, "arm_error", "failed", {"arm_error": str(e)})
            except Exception as e:
                traceback.print_exc()
                rep.final(mid, "arm_error", "failed", {"arm_error": f"{type(e).__name__}: {e}"})
    except KeyboardInterrupt:
        print("\n[종료] 접힌 자세로 가서 끝냅니다")
    finally:
        rep.release()
        tasks.shutdown()


if __name__ == "__main__":
    main()
