# SO-ARM 팔 에이전트 연동 안내 (미션 상태 서버용)

라즈베리파이(176)의 SO-ARM이 노트북의 **미션 상태 서버**(`server.py`, `GET/POST /api/status`)를 보고
스스로 **집기 → 운반 자세로 들기 → 내려놓기 → 접기**를 하는 방식입니다.
팔과 작업 제어기는 서로 직접 통신하지 않고, 상태 서버의 `phase` / `flag`만 주고받습니다.

```
노트북 (192.168.110.113)                     라즈베리파이 (192.168.110.176)
 ├─ SLAM/AMCL, Nav2                           ├─ 터틀봇 드라이버
 ├─ 작업 제어기 ── POST/GET ──┐               └─ arm_agent.py (팔)
 └─ 상태 서버 server.py :8080 ◀┴── 0.5초마다 GET, 일할 때 1초마다 POST ──┘
```

---

## 1. 약속 (작업 제어기가 지켜야 할 것)

### 1-1. 팔을 움직이게 하는 상태

| 작업 제어기가 POST | 팔이 하는 일 |
|---|---|
| `{"phase": "ws_ready", "flag": "done"}` | 좌우 60° 둘러보며 블록을 찾고 → 집고 → **운반 자세로 든 채 정지** |
| `{"phase": "dest_ready", "flag": "done"}` | 내려놓는 자세로 가서 그리퍼를 열어 놓고 → 접힌 자세 |
| `{"phase": "arm_park", "flag": "busy"}` | 접힌 자세로 (필요할 때만) |

- `mission_id`는 자유롭게 정하면 됩니다. 팔은 받은 `mission_id`를 그대로 붙여서 결과를 씁니다.
- 목적지 도착 이름 `dest_ready`는 임시로 정한 것입니다. 바꾸려면 팔 쪽 실행 옵션 `--place-on <이름>`만 바꾸면 됩니다.

### 1-2. 팔이 쓰는 상태

| 팔이 POST | 뜻 | 작업 제어기가 할 일 |
|---|---|---|
| `arm_picking` / busy | 집는 중 (1초마다 다시 씀) | 기다림 |
| `arm_picked` / done | **블록을 들고 운반 자세로 정지** | 목적지로 이동 → 도착하면 `dest_ready/done` |
| `arm_pick_not_found` / failed | 좌우 60° 둘러봐도 없음 | 터틀봇을 돌리거나 옮긴 뒤 다시 `ws_ready/done` |
| `arm_pick_too_far` / failed | 보이지만 팔이 안 닿음 | `arm_result`의 방향·거리만큼 다가간 뒤 다시 `ws_ready/done` |
| `arm_pick_missed` / failed | 못 쥠 (팔이 이미 3번 시도함) | 다시 `ws_ready/done` 하면 또 시도 |
| `arm_placing` / busy | 내려놓는 중 | 기다림 |
| `arm_placed` / done | 내려놓고 접힘 완료 | 다음 단계로 (예: `returning_home`) |
| `arm_parking` → `arm_parked` | 접는 중 → 접힘 | |
| `arm_error` / failed | 팔 오류·정지 (`arm_error`에 이유) | 사람 확인 필요 |

### 1-3. 상태 쓰기 차례 (가장 중요)

상태 서버에는 **상태가 하나뿐**이라 두 쪽이 동시에 쓰면 서로 덮어씁니다. 그래서 차례를 넘깁니다.

1. 작업 제어기가 `ws_ready/done`(또는 `dest_ready/done`)을 씁니다.
   - 팔이 받았다는 표시(`arm_picking` / `arm_placing`)가 보일 때까지 **1초마다 같은 상태를 다시 쓰는 것을 권장**합니다.
     한 번만 쓰면, 팔 에이전트가 늦게 켜지거나 재시작 중일 때 3초 뒤 stale이 되어 놓칠 수 있습니다.
     같은 트리거를 여러 번 받아도 팔은 **한 번만** 움직입니다.
2. `arm_picking`이 보이면 작업 제어기는 **heartbeat POST를 멈춥니다.** 이제 팔이 1초마다 씁니다.
3. 팔이 끝 상태(`arm_picked`, `arm_pick_*`, `arm_placed`, `arm_error`)를 씁니다.
   - 팔은 작업 제어기가 다음 상태를 쓸 때까지 **최대 30초** 동안 끝 상태를 1초마다 다시 씁니다(stale 방지).
4. 작업 제어기가 끝 상태를 읽고 다음 상태(예: `navigating_to_dest/busy`)를 쓰면서 **다시 차례를 가져갑니다.**

### 1-4. 다시 시키기

- 팔이 끝 상태를 쓴 뒤 작업 제어기가 **같은 트리거(`ws_ready/done`)를 다시 쓰면** 팔은 새 요청으로 보고 다시 합니다.
  `mission_id`를 바꿀 필요는 없습니다.
- 못 쥐었을 때(missed)는 팔이 결과를 쓰기 전에 **스스로 2번 더** 시도합니다(`--retries`로 변경).
- 팔 에이전트를 켜기 **전에** 써 둔 오래된(stale) 상태로는 움직이지 않습니다 (켜자마자 갑자기 움직이지 않게).

---

## 2. 결과 상세 (`arm_result`)

팔은 `--extra` 옵션으로 실행되어 상태에 `arm_result`(또는 `arm_error`)를 함께 씁니다. 상태 서버는 추가 필드를 그대로 저장합니다.

```json
{"mission_id": "test-4", "phase": "arm_pick_too_far", "flag": "failed",
 "arm_result": {"result": "too_far", "reason": "집을 수 없는 위치 (...)",
                "fwd_m": 0.31, "left_m": -0.04, "dist_m": 0.313, "bearing_deg": -7.3,
                "arm_xy_m": [0.33, 0.12]}}
```

| 필드 | 뜻 |
|---|---|
| `fwd_m`, `left_m` | 블록 위치. **팔 회전축 기준**, 터틀봇 앞(+)/왼쪽(+) 방향, 미터 |
| `dist_m`, `bearing_deg` | 같은 위치를 거리·방향으로 (왼쪽이 +) |
| `base_fwd_m`, `base_left_m`, `base_bearing_deg` | 터틀봇 **중심** 기준 값. 팔 쪽 `calibration/mount.json`에 `"pan_axis_in_base_m": [앞, 왼]`을 넣으면 함께 나옴 (아직 미측정) |
| `gripper_pct` | 그리퍼 열림 %. 블록(25mm)을 쥐면 약 10~14% |

**팔이 집을 수 있는 범위**: 팔 회전축에서 약 **17~25cm**, 터틀봇 정면 기준 좌우 **±60°**, 작업 상자 위.
`too_far`면 블록이 팔 회전축에서 약 21cm 떨어진 곳에 오도록 다가가면 됩니다.

---

## 3. 손으로 시험하기 (curl)

팔 에이전트를 **먼저** 켜 둔 뒤 보냅니다.

```bash
# 집기
curl -X POST http://192.168.110.113:8080/api/status -H 'Content-Type: application/json' \
  -d '{"mission_id":"test-1","phase":"ws_ready","flag":"done"}'

# 상태 보기 (arm_picking → arm_picked)
curl http://192.168.110.113:8080/api/status

# 내려놓기 (집은 뒤에)
curl -X POST http://192.168.110.113:8080/api/status -H 'Content-Type: application/json' \
  -d '{"mission_id":"test-1","phase":"dest_ready","flag":"done"}'

# 접기만
curl -X POST http://192.168.110.113:8080/api/status -H 'Content-Type: application/json' \
  -d '{"mission_id":"test-1","phase":"arm_park","flag":"busy"}'
```

---

## 4. 팔 쪽 실행 (라즈베리파이)

```bash
cd ~/so101-mobile-pick
~/arm-venv/bin/python arm_agent.py --status-url http://192.168.110.113:8080/api/status --extra
```

정상이면 이렇게 뜹니다.
```
[준비] 팔 연결 중 (...)
[준비] 손목 카메라 여는 중 (...)
[준비] 완료
[팔 에이전트] http://192.168.110.113:8080/api/status 감시 중 — ...
[상태] mission=... phase=... flag=... fresh=...
```

- **ROS를 실행하는 터미널과 따로** 실행합니다 (venv의 파이썬 패키지가 ROS와 섞이지 않게).
- 시작할 때 팔은 **접힌 자세**에 둡니다. 토크가 꺼진 상태에서는 접힌 자세에서만 스스로 폅니다.
- 끌 때는 Ctrl+C → 접힌 자세로 가서 토크를 끕니다. 블록을 들고 있으면 떨어지지 않게 토크를 켠 채 둡니다.
- ROS·turtlebot3 환경은 건드리지 않습니다. 팔 USB와 손목 카메라는 장치 고유 이름(by-id)으로 찾아서 OpenCR(ttyACM)·파이 카메라(video0)와 헷갈리지 않습니다.

옵션

| 옵션 | 기본 | 뜻 |
|---|---|---|
| `--pick-on` | `ws_ready` | 이 phase + done이면 집기 |
| `--place-on` | `dest_ready` | 이 phase + done이면 내려놓기 |
| `--park-on` | `arm_park` | 이 phase면 접기 |
| `--color` | `yellow` | 집을 블록 색 |
| `--retries` | `2` | 못 쥐었을 때 스스로 다시 시도하는 횟수 |
| `--extra` | 꺼짐 | 결과 상세(`arm_result`)를 상태에 함께 씀 |
| `--final-hold-s` | `30` | 끝 상태를 작업 제어기가 넘겨받을 때까지 다시 쓰는 최대 시간 |

---

## 5. 처음 한 번 준비 (팔 쪽, 이미 완료)

| 파일 | 만드는 명령 | 내용 |
|---|---|---|
| `calibration/park_pose.json` | `steps/m5_teach_park.py` | 접힌 자세 |
| `calibration/carry_pose.json` | `steps/m6_teach_pose.py carry` | 운반 자세 (라이다를 가리지 않게) |
| `calibration/place_pose.json` | `steps/m6_teach_pose.py place` | 내려놓는 자세 (그리퍼가 통 입구 위) |
| `calibration/look_pose.json`, `wrist_camera.json`, `mount.json` | m1~m3 | 보는 자세, 손목 카메라 보정, 작업면 높이 |

작업 상자 위치·높이가 바뀌면 `steps/m1_floor_check.py`로 높이만 다시 재면 됩니다.

---

## 6. 주의

- **운반 중**: 팔은 블록을 든 채 토크를 켜 두고 있습니다. 급정거·급회전하면 블록이 떨어질 수 있습니다.
- **라이다**: 운반 자세는 라이다를 가리지 않게 잡았지만, Nav2 코스트맵에 팔이 장애물로 찍히면 알려 주세요.
- **내려놓기 위치**: 지금은 `place_pose`(고정 자세)에서 그리퍼를 열어 떨어뜨립니다. 터틀봇이 목적지에서 **매번 같은 방향·위치**로 서야 통에 들어갑니다.
- **보안**: 인증이 없으니 같은 와이파이 안에서만 씁니다.

## 7. 문제 해결

| 증상 | 확인 |
|---|---|
| `감시 중`이 안 뜸 | 마지막으로 찍힌 `[준비]` 줄이 멈춘 곳. 남은 프로세스 확인: `ps aux \| grep arm_agent`, `sudo fuser -v /dev/video* /dev/ttyACM*` |
| 트리거를 보내도 안 움직임 | `[상태] ... fresh=False`면 오래된 상태 → 에이전트를 켠 **뒤에** 다시 보내기 |
| 모터 응답 없음 | `~/arm-venv/bin/python steps/bus_scan.py` → 1000000 bps에 ID1~6이 나와야 함. 안 나오면 12V 전원·케이블 |
| `arm_error` | 상태의 `arm_error` 문구와 파이 화면 로그 확인 |
| 블록을 자주 못 찾음/못 쥠 | 파이의 `steps/logs/task_*_pick*.png` 사진 확인 (블록 주변에 같은 색 물건이 없어야 함) |
