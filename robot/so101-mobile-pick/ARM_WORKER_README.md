# SO-ARM worker (`pick_and_place.py`) — soarm_mission 연동 안내

`soarm_mission` README의 **"pick_and_place 입출력 계약"**을 구현한 실제 팔 worker입니다.
robot_bridge가 `start` 때 이 파일을 한 번 실행하고, `place` → `shutdown`까지 같은 프로세스를 씁니다.

## 실행 (라파, 기존 README 그대로)

```bash
source /opt/ros/humble/setup.bash
source /home/turtlebot6/soarm_mission_ws/install/setup.bash
export ROS_DOMAIN_ID=36

ros2 run soarm_mission robot_bridge \
  --worker /home/turtlebot6/so101-mobile-pick/pick_and_place.py \
  --worker-python /home/turtlebot6/arm-venv/bin/python \
  --status-url http://192.168.110.113:8080/api/status \
  --state-db /home/turtlebot6/soarm_mission_ws/robot_mission.sqlite3 \
  --worker-timeout 120
```

**`--worker-timeout`(라파)과 노트북의 `--worker-timeout`/`manipulation.timeout_sec`를 120초 이상으로** 올려 주세요.
pick은 좌우 60° 둘러보기 + 집기 + 운반 자세까지 보통 20~40초, 재시도 포함 최대 90초쯤 걸립니다.
bridge는 두 값 중 작은 값을 쓰고(`min(--worker-timeout, timeout_sec)`), 기본 60초면 부족할 수 있습니다.
worker는 받은 `timeout_sec - 3초` 안에 끝내고, 넘으면 그 자리에 멈춘 뒤 `failed`("제한 시간 초과")를 보냅니다.

## action별 동작

| action | 팔 동작 | 응답 |
|---|---|---|
| `start` | 장치·보정 확인 → 접힌 자세에서 펴서 보는 자세 | `done`, `holding: false` |
| `pick` | `color` 블록을 좌우 60° 둘러보며 찾아 집기 → **운반 자세로 들고 정지** | `done`, `holding: true`, `travel_ready: true`, `arm_result` |
| `pick` 실패 | 접힌 자세로 돌아감 | `failed`, `error: "pick not_found / too_far / missed"`, `arm_result` |
| `place` | 내려놓는 자세(home에서 쓸 고정 자세)에서 그리퍼 열기 → 접힌 자세 | `done`, `holding: false` |
| `shutdown` | 접힌 자세에서 토크 OFF → 응답 후 프로세스 종료 (3초 안) | `done`, `holding: false` |

- 모든 명령에 먼저 `flag: busy`를 한 번 보냅니다. `mock`은 항상 `false`.
- `max_attempts`: 못 쥔 경우(`missed`)에만 다시 시도합니다. `not_found`·`too_far`는 팔이 다시 해도 같으므로 바로 실패를 보냅니다 (터틀봇이 움직여야 함).
- 색: `red`, `blue`, `green`, `yellow` (bridge 허용 목록과 같음). 실제 손목 카메라로 확인한 색은 **yellow, red**.

### `arm_result` (pick)

```json
{"result": "too_far", "reason": "...", "fwd_m": 0.31, "left_m": -0.04, "dist_m": 0.313,
 "bearing_deg": -7.3, "arm_xy_m": [0.33, 0.12]}
```
`fwd_m`/`left_m`: 블록 위치, **팔 회전축 기준** 터틀봇 앞(+)/왼쪽(+), m. 팔이 집을 수 있는 범위는 팔 회전축에서 약 17~25cm, 정면 기준 좌우 ±60°.
지금 bridge는 failed 응답의 `error`만 쓰지만, `too_far`면 이 값으로 다가가 다시 시도하는 정책을 나중에 넣을 수 있습니다.

## 안전 동작 (계약 항목)

| 상황 | worker 동작 |
|---|---|
| `mission_state`의 `flag: uncertain` 또는 `phase: halt_and_hold` | 진행 중인 동작을 다음 단계 사이(수백 ms~2초 안)에서 멈추고 그 자세 유지. 그리퍼 쥔 힘은 그대로 → 해당 명령은 `failed` |
| 명령 처리 중 `mission_state`가 5초 넘게 끊김 | 위와 같이 멈춤 |
| stdin EOF, 블록을 들고 있음 | **토크를 끄지 않고** 그 자세 유지, 프로세스도 끝내지 않음 (작업자 복구 대기) |
| stdin EOF, 빈손 | 접힌 자세로 가서 토크 OFF 후 종료 |
| 명령 사이 (운반 중 등) | 팔은 움직이지 않고 토크만 유지 |

- stdout에는 JSON만 나옵니다 (팔 코드의 모든 로그는 stderr).
- 한 동작(관절 이동 한 번) 도중에는 끊지 못하고 그 동작이 끝나는 즉시 멈춥니다. 비상시엔 팔 전원(12V)을 끊는 물리 수단이 필요합니다.

## 시험

bridge의 `WorkerSession`으로 가상 모터 환경에서 `start → pick → place → shutdown`이 통과하는 것을 확인했습니다 (shutdown 후 강제 회수 없음).
실제 장치에서는 아래 순서로 먼저 확인하세요.

```bash
# worker만 손으로 (bridge 없이) — 한 줄씩 붙여 넣기. 손으로는 heartbeat를 못 넣으니 --no-watchdog
~/arm-venv/bin/python ~/so101-mobile-pick/pick_and_place.py --no-watchdog
{"type":"mission_state","mission_id":"t1","phase":"ws_ready","flag":"done"}
{"type":"command","mission_id":"t1","request_id":"r1","boot_id":"b","action":"start","station_id":"table_a","color":"red","timeout_sec":120,"max_attempts":2}
{"type":"command","mission_id":"t1","request_id":"r2","boot_id":"b","action":"pick","station_id":"table_a","color":"red","timeout_sec":120,"max_attempts":2}
{"type":"command","mission_id":"t1","request_id":"r3","boot_id":"b","action":"place","station_id":"table_a","color":"red","timeout_sec":120,"max_attempts":1}
{"type":"command","mission_id":"t1","request_id":"r4","boot_id":"b","action":"shutdown","station_id":"table_a","color":"red","timeout_sec":10,"max_attempts":1}
```
(`--no-watchdog`은 손으로 시험할 때만. bridge가 실행할 때는 붙이지 않습니다.)

## 준비 (팔 쪽, 완료)

접힌 자세 `calibration/park_pose.json`, 운반 자세 `carry_pose.json`(라이다 안 가리게), 내려놓는 자세 `place_pose.json`(home에서 통 입구 위), 보는 자세·손목 카메라 보정·작업면 높이.
**place는 고정 자세에서 떨어뜨리므로 home 목표에서 터틀봇이 매번 같은 위치·방향으로 서야 합니다** (Nav2 home 목표의 도착 허용 오차를 작게, 또는 정렬 단계 사용).
