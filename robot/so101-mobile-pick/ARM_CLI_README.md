# SO-ARM 팔 명령 실행기 사용 안내 (터틀봇 작업 제어기용)

터틀봇 작업 제어기(라즈베리파이)가 팔이 필요할 때 **`arm_cli.py`를 실행**하면, 팔이 한 동작을 하고
**결과 JSON 한 줄**을 출력한 뒤 끝납니다. 서버나 상태 감시는 없습니다.

```
노트북 ── SSH ──▶ 라즈베리파이
                  ├─ 작업 제어기 (자연어 해석, Nav2 이동, 순서 관리)
                  │     └─ subprocess: arm_cli.py pick / place / park  →  JSON 결과
                  └─ 터틀봇 드라이버
```

---

## 1. 명령

```bash
~/arm-venv/bin/python ~/so101-mobile-pick/arm_cli.py pick --color yellow   # 찾아 집어서 운반 자세로 들기
~/arm-venv/bin/python ~/so101-mobile-pick/arm_cli.py place                 # 내려놓는 자세에서 놓고 접기
~/arm-venv/bin/python ~/so101-mobile-pick/arm_cli.py park                  # 접힌 자세로 (토크 OFF)
~/arm-venv/bin/python ~/so101-mobile-pick/arm_cli.py status                # 들고 있는지·자세 (팔 안 움직임)
```

- **표준출력**: 마지막에 JSON 한 줄만 나옴 → 이것만 파싱하면 됨
- **표준에러**: 진행 로그 (`[준비] 팔 연결 중`, `… 둘러보기` 등)
- 반드시 **venv 파이썬(`~/arm-venv/bin/python`)**으로 실행 (팔·카메라 패키지가 거기 있음). ROS 파이썬과 섞이지 않음.

### 종료 코드와 결과

| 명령 | `result` | 종료 코드 | 뜻 | 작업 제어기가 할 일 |
|---|---|---|---|---|
| pick | `picked` | 0 | 집어서 **운반 자세로 든 채** 끝남 (토크 켜 둠) | 목적지로 이동 → `place` |
| pick | `not_found` | 1 | 좌우 60° 둘러봐도 없음 | 터틀봇 회전·이동 후 다시 `pick` |
| pick | `too_far` | 1 | 보이지만 안 닿음 | `fwd_m`/`left_m`만큼 다가간 뒤 다시 `pick` |
| pick | `missed` | 1 | 못 쥠 (스스로 2번 더 해 본 뒤) | 다시 `pick` |
| place | `placed` | 0 | 놓고 접힌 자세 (토크 OFF) | 다음 단계 |
| park | `parked` | 0 | 접힌 자세 (토크 OFF) | |
| 모두 | `error` | 2 | 오류 (`error`에 이유). `busy`면 다른 팔 명령 실행 중 | 사람 확인 |

### 결과 예시

```json
{"cmd": "pick", "result": "picked", "ok": true, "gripper_pct": 12.0,
 "fwd_m": 0.21, "left_m": 0.03, "dist_m": 0.212, "bearing_deg": 8.1, "arm_xy_m": [0.2065, 0.13],
 "seconds": 18.4, "holding": true, "pose": "carry"}

{"cmd": "pick", "result": "too_far", "ok": false, "reason": "집을 수 없는 위치 (...)",
 "fwd_m": 0.31, "left_m": -0.04, "dist_m": 0.313, "bearing_deg": -7.3, "holding": false, "pose": "park"}

{"cmd": "place", "result": "error", "ok": false, "error": "들고 있는 블록이 없음"}
```

| 필드 | 뜻 |
|---|---|
| `fwd_m`, `left_m` | 블록 위치. **팔 회전축 기준**, 터틀봇 앞(+)/왼쪽(+), 미터 |
| `dist_m`, `bearing_deg` | 같은 위치의 거리·방향 (왼쪽이 +) |
| `base_fwd_m`, `base_left_m` | 터틀봇 **중심** 기준. 팔 쪽 `calibration/mount.json`에 `"pan_axis_in_base_m": [앞, 왼]` 넣으면 나옴 (아직 미측정) |
| `holding`, `pose` | 끝났을 때 블록을 들고 있는지, 팔 자세 (`park` / `carry` / `look` …) |

**팔이 집을 수 있는 범위**: 팔 회전축에서 약 **17~25cm**, 터틀봇 정면 기준 좌우 **±60°**, 작업 상자 위.
`too_far`면 블록이 팔 회전축에서 약 21cm 거리에 오도록 다가가면 됩니다.

---

## 2. 작업 제어기에서 부르는 예 (Python)

```python
import json, subprocess

ARM = ["/home/turtlebot6/arm-venv/bin/python", "/home/turtlebot6/so101-mobile-pick/arm_cli.py"]

def arm(cmd, *args, timeout=180):
    p = subprocess.run(ARM + [cmd, *args], capture_output=True, text=True, timeout=timeout)
    last = p.stdout.strip().splitlines()[-1] if p.stdout.strip() else "{}"
    return json.loads(last)            # p.stderr 에 진행 로그

# 흐름
r = arm("pick", "--color", "yellow")
for _ in range(4):
    if r["result"] == "picked":
        break
    if r["result"] == "not_found":
        rotate_robot(90)               # 작업 제어기 쪽 함수 (Nav2 Spin 등)
    elif r["result"] == "too_far":
        approach(r["fwd_m"], r["left_m"])
    elif r["result"] == "error":
        raise RuntimeError(r["error"])
    r = arm("pick", "--color", "yellow")

navigate_to_destination()              # 팔은 블록을 든 채 운반 자세로 대기
r = arm("place")
```

- `pick` 한 번에 보통 15~40초 (둘러보기 범위에 따라). `timeout`은 넉넉히.
- 터틀봇이 **움직이는 동안에는 팔 명령을 실행하지 않습니다.** 정지 확인 후 실행.

---

## 3. 동작 방식 (참고)

- 각 실행은 팔에 연결 → 한 동작 → 연결 끊기. 실행 사이에 **들고 있는지·자세는 `state/arm_state.json`**에 저장해 다음 실행이 이어받습니다.
- `pick` 성공 시 블록을 든 채 **토크를 켠 채** 끝납니다 (떨어뜨리지 않게). 실패·`place`·`park` 후에는 접힌 자세로 가서 토크를 끕니다.
- 오류(`error`)면 그 자리에 멈춘 채(토크 유지) 끝납니다 → 사람이 확인.
- 동시에 두 개를 실행하면 두 번째는 바로 `busy` 오류로 끝납니다 (팔 버스를 둘이 같이 쓰지 않게).
- 팔이 접힌 자세가 아니고 토크도 꺼져 있으면 시작하지 않습니다 (손으로 받칠 사람이 없으니). 팔을 접힌 자세에 두고 다시.

---

## 4. 처음 한 번 준비 (팔 쪽, 완료됨)

| 파일 | 만드는 명령 | 내용 |
|---|---|---|
| `calibration/park_pose.json` | `steps/m5_teach_park.py` | 접힌 자세 |
| `calibration/carry_pose.json` | `steps/m6_teach_pose.py carry` | 운반 자세 (라이다를 가리지 않게) |
| `calibration/place_pose.json` | `steps/m6_teach_pose.py place` | 내려놓는 자세 (그리퍼가 통 입구 위) |
| `calibration/look_pose.json`, `wrist_camera.json`, `mount.json` | m1~m3 | 보는 자세, 손목 카메라 보정, 작업면 높이 |

---

## 5. 주의

- **운반 중**: 팔은 블록을 든 채 토크를 켜 두고 있습니다. 급정거·급회전하면 블록이 떨어질 수 있습니다.
- **라이다**: 운반 자세가 라이다를 가리지 않게 잡았지만, Nav2 코스트맵에 팔이 장애물로 보이면 알려 주세요.
- **내려놓기 위치**: 고정 자세(`place_pose`)에서 그리퍼를 열어 떨어뜨립니다. 터틀봇이 목적지에서 **매번 같은 방향·위치**로 서야 통에 들어갑니다.

## 6. 문제 해결

| 증상 | 확인 |
|---|---|
| `error: busy` | 다른 팔 명령이 실행 중. 남은 프로세스: `ps aux \| grep arm_` |
| 시작이 안 됨 / 멈춤 | 표준에러의 마지막 `[준비]` 줄이 멈춘 곳. `sudo fuser -v /dev/video* /dev/ttyACM*` |
| 모터 응답 없음 | `~/arm-venv/bin/python steps/bus_scan.py` → 1000000 bps에 ID1~6. 안 나오면 12V 전원·케이블 |
| 블록을 자주 못 찾음/못 쥠 | 파이의 `steps/logs/task_*_pick*.png` 사진 (블록 주변에 같은 색 물건이 없어야 함) |
| 상태가 꼬임 (`들고 있는 블록이 없음` 등) | 팔을 손으로 받치고 `park` 실행, 또는 `state/arm_state.json` 삭제 후 팔을 접힌 자세에 두고 다시 |
