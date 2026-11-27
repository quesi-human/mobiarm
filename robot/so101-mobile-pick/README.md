# so101-mobile-pick (v2)

SO-ARM101을 **TurtleBot3 Waffle Pi** 위에 올려서, 지도상 "그 즈음"으로 이동한 뒤 블록을 **스스로 찾아 집고**, 정해진 상자(쓰레기통)에 넣는 프로젝트입니다.
v1 [so101-voice-pick](../so-arm)(고정 책상 + 위 카메라)에서 팔 제어와 IK는 그대로 가져왔습니다.

> 상태: **구조 잡기 + 1·2단계 스크립트**까지 작성. 실제 로봇으로는 아직 검증 전입니다.

---

## 설계

### 역할 나누기

| 담당 | 하는 일 | 좌표 |
|---|---|---|
| 터틀봇 (ROS2 Nav2) | 지도에 저장한 지점("쓰레기 구역 A", "쓰레기통")까지 이동 | 지도 |
| SO-ARM + 손목 카메라 | 도착 후 블록을 찾고 다가가 집기 / 상자에 넣기 | **팔 베이스** |
| 전체 순서 | A로 이동 → 찾기 → 다가가기 → 집기 → 쓰레기통으로 이동 → 넣기 | 지점 이름만 |

- 팔은 지도를 몰라도 됩니다. 블록 위치는 매번 **팔 기준**으로 손목 카메라로 잽니다.
- 터틀봇이 정확히 서지 않아도 됩니다. 블록이 팔 닿는 범위에만 들어오면 됩니다.
- 실행: 라즈베리파이(터틀봇 + 팔 + 카메라), 노트북은 SSH 접속.

### 정밀하게 집는 방법: 손목 카메라로 "보고 → 옮기고 → 다시 보고"

```
① 보는 자세     팔을 늘 같은 관절 각도로 → 손목 카메라로 바닥을 봄
                 이 자세에서는 카메라 위치가 고정 → v1 위 카메라 보정 방식이 그대로 통함
                 → 블록의 팔 기준 바닥 좌표·방향 (수 mm~1cm)
② 블록 위로     IK로 블록 위 8cm, travel()로 이동 (v1과 같음 — 움직이기 전에 목표가 이미 정해져 있음)
③ 다시 보고 고치기  블록이 화면의 "잡히는 자리"에서 벗어난 만큼 line()으로 몇 mm, 손목 회전 몇 도만 수정 (1~2번)
                 → 보정·IK 오차가 여기서 사라짐
④ 집기          수직으로 내려가 집기 (v1과 같음)
⑤ 확인          들어 올린 뒤 그리퍼 사이에 블록이 있는지 손목 카메라로 확인
```

- 사진은 늘 **터틀봇이 멈춘 상태**에서만 찍습니다. 전제는 바닥이 평평하다는 것뿐입니다(멈춰 있으면 팔과 바닥 사이 높이가 늘 같음).
- **보는 자세는 그리퍼 수직(카메라가 바로 아래를 봄)**으로 정합니다.
  - `travel()`은 경유점을 전부 그리퍼 수직 자세로 계산하므로, 보는 자세가 기울어져 있으면 출발할 때 자세가 크게 바뀝니다.
  - 바로 아래를 보면 블록 방향도 재기 쉽습니다.
  - m2가 출력하는 "그리퍼 아래 방향"이 1에 가까운 자세를 고릅니다.
- **보정은 팬 0°에서 한 번만** 합니다. 베이스 회전(shoulder_pan)은 바닥에 수직인 축을 도는 회전이라, 팬 θ°에서 찍은 사진은 팬 0° 보정 결과를 팬 축 기준으로 θ°만큼 돌리면 그대로 맞습니다.

### 찾기와 다가가기

손목 카메라만 쓰므로 블록이 안 보이면 찾으러 움직여야 합니다. 비용이 적은 순서로 시도합니다.

```
Nav2로 "그 즈음" 도착 (팔은 접은 채, 무게중심 낮게)
→ 멈춤 → ① 팔로 훑기 ─ 보이고 닿음 ──→ 정밀하게 집기
                     ├ 보이는데 멂 ──→ ③ 다가가기 → ①부터 다시
                     └ 안 보임 ─────→ ② 제자리 회전 → ①부터 다시 (한 바퀴 돌아도 없으면 실패 보고)
```

| 단계 | 하는 일 | 움직이는 것 |
|---|---|---|
| ① 팔로 훑기 | 보는 자세에서 팬만 -40° → -20° → 0° → 20° → 40°로 바꾸며 찍기. 보정 1번으로 모든 각도 계산 | 팔만 |
| ② 제자리 회전 | 터틀봇을 60°쯤 돌리고 ① 반복 (Nav2 Spin 또는 `cmd_vel`) | 터틀봇 |
| ③ 다가가기 | 닿는 범위(몸체 앞 약 15cm) 밖에 보이면 방향·거리를 구해 "돌고 몇 cm 전진" (Nav2 DriveOnHeading 또는 `cmd_vel`). 도착하면 ①부터 다시 재기 | 터틀봇 |

- 멀리 볼 때는 카메라를 앞으로 조금 기울인 "멀리 보는 자세"를 따로 둘 수 있습니다. 보정 범위 밖이라 거리가 부정확해서 다가갈 방향을 정하는 데만 씁니다.
- 그래도 찾는 게 너무 느리면 터틀봇 앞쪽을 넓게 보는 고정 카메라를 하나 추가합니다(선택).

### IK 방식 → 나중에 ACT

1. 먼저 블록으로 IK 방식 전체 흐름을 완성합니다.
2. 집을 물건이 다양해지면(캔, 종이 등) ④ 집기만 ACT로 바꿉니다. 이때 노트북 GPU에서 추론합니다.

---

## v1에서 가져온 것

| 파일 | v1 → v2 |
|---|---|
| `robot/arm.py` | 그대로 (모터 통신, 안전 감시, 큰 이동 나누기, 도착 보정, 그리퍼) |
| `robot/kinematics.py`, `robot/urdf_model/` | 그대로 (팔 베이스 기준 FK/IK) |
| `robot/motion.py` | 그대로 (직선·원호 경로) |
| `robot/pick.py` | 그대로, 좌표 상수만 `mount.py`에서 가져옴. 대기 위치(`PARK_XY`) 등은 터틀봇 위에서 다시 정해야 함 |
| `vision/camera.py` | v1 `camera.py`에서 고정 카메라 전용(밀림 보정, RealSense)을 뺌 |
| `robot/mount.py` | v1 `workcell.py` 대신. 책상 높이 → 바닥 높이, 몸체 앞 끝 거리 |
| `steps/s0_make_local_calibration.py`, `calibration/motor_matched/`, `calibration/gripper.json` | 그대로 (같은 팔) |

---

## 설치 (노트북·라즈베리파이 공통)

팔 통신은 lerobot 대신 `robot/feetech_bus.py`를 씀 (11/25). lerobot·PyTorch가 필요 없음.

```bash
python3 -m venv ~/arm-venv
~/arm-venv/bin/pip install -r requirements.txt
sudo usermod -aG dialout $USER      # 한 번만, 다시 로그인
```

- 팔 포트·손목 카메라는 `robot/devices.py`의 장치 고유 이름(by-id)을 기본으로 씀 → 터틀봇 OpenCR(ttyACM0)과 안 헷갈림
- `calibration/` 폴더를 그대로 복사. **s0는 다시 돌리지 않음** (s0만 lerobot을 씀 — 노트북에서 한 번 만든 파일이면 충분)
- 모터 응답 확인: `python steps/bus_scan.py` (읽기 전용)

## 팔 에이전트 — 팀원 상태 서버 연동 (11/25, 실제로 쓸 것)

노트북의 미션 상태 서버(팀원 server.py, `GET/POST /api/status`)를 파이에서 0.5초마다 읽다가 집기·내려놓기를 함.
(위 `arm_server.py`는 팔만 따로 시험할 때 씀)

```bash
~/arm-venv/bin/python arm_agent.py --status-url http://192.168.110.113:8080/api/status
```

| 작업 제어기가 쓰는 상태 | 팔이 하는 일 | 팔이 쓰는 상태 (일하는 동안 1초마다) |
|---|---|---|
| `ws_ready` / done | 둘러보고 집어 운반 자세 | `arm_picking`/busy → `arm_picked`/done, `arm_pick_not_found`·`arm_pick_too_far`·`arm_pick_missed`/failed |
| `dest_ready` / done | 내려놓고 접기 | `arm_placing`/busy → `arm_placed`/done |
| `arm_park` | 접기 | `arm_parking`/busy → `arm_parked`/done |
| (오류·중지) | 그 자리 정지 | `arm_error`/failed |

약속: 트리거 상태를 쓴 뒤 작업 제어기는 POST를 멈춤(상태 쓰기 권한을 팔에 넘김). 팔이 끝 상태를 쓰면 작업 제어기가 다음 상태를 써서 넘겨받음.
phase 이름은 `--pick-on`, `--place-on`, `--park-on`으로 바꿀 수 있음. `--extra`면 결과 상세(too_far 방향·거리)도 상태에 넣음.

## 팔 서버 (HTTP, 11/25)

노트북(SLAM·Nav2·작업 제어기) ↔ 파이(터틀봇 드라이버 + 팔). 팔은 ROS와 상관없이 HTTP로 명령을 받음.

```bash
# 파이 (venv, ROS 터미널과 따로)
~/arm-venv/bin/python arm_server.py            # 8000번 포트, Ctrl+C = 접고 끝
# 노트북 (기본 라이브러리만)
python tools/arm_client.py --host <파이IP> pick yellow   # picked / not_found / too_far / missed
python tools/arm_client.py --host <파이IP> place
python tools/arm_client.py --host <파이IP> park
python tools/arm_client.py --host <파이IP> status
```

흐름: 접힌 자세(park) →pick→ 보는 자세에서 둘러보고 집기 → 운반 자세(carry)로 들고 정지 → 터틀봇 이동 →place→ 내려놓는 자세에서 놓기 → 접힌 자세.
`too_far`면 `fwd_m`/`left_m`(팔 회전축 기준)을 줌. `calibration/mount.json`에 `"pan_axis_in_base_m": [앞, 왼]`(터틀봇 중심→팔 회전축, m)을 넣으면 터틀봇 중심 기준 값도 줌.

## 단계

| 단계 | 스크립트 | 내용 | 상태 |
|---|---|---|---|
| 0 | `steps/s0_make_local_calibration.py` | 모터 기준값 확인 (v1과 같음) | ✅ |
| 1 | `steps/m1_floor_check.py` | 팔을 터틀봇에 단 뒤, 손끝으로 바닥 4~6곳을 짚어 바닥 높이·기울기 재기 → 집을 수 있는 범위 | ✅ 작성 |
| 2 | `steps/m2_teach_look_pose.py` | 손목 카메라로 바닥을 내려다보는 "보는 자세"(그리퍼 수직)를 손으로 정하기 | ✅ 작성 |
| 3 | `m3_collect_camera.py --color yellow` → `m3_solve_camera.py <로그> --save` | 보는 자세(터틀봇 정면)에서 손목 카메라 픽셀 → 블록 윗면 좌표 보정 (v1 s4b + s4_solve 방식). 다른 팬 각도는 회전 계산으로 | `calibration/wrist_camera.json` |
| 4 | `m4_pick_test.py --color yellow` | 보는 자세에서 블록 찾기 → 집기 → 같은 자리에 내려놓기 → 다시 찍어 카메라 오차 확인 (터틀봇 정지). 공통 코드는 `robot/wristcam.py` | `steps/logs/m4_*.jsonl` |
| 5a | `m5_teach_park.py` | 접힌 자세(park) 한 번 저장 → m4·미션이 접힌 자세에서 스스로 펴고, 끝나면 접음 (`robot/park.py`) | `calibration/park_pose.json` |
| 5b | `m6_teach_pose.py carry` / `m6_teach_pose.py place` | 운반 자세(라이다 안 가리게)·내려놓는 자세 저장 | `calibration/carry_pose.json`, `place_pose.json` |
| 5 | (예정) | 상자에 넣는 자세 가르치기 (v1 s6_teach_drop 방식) | |
| 6 | (예정) | 찾기·다가가기: ① 팔로 훑기 → ② 제자리 회전 → ③ 다가가기 (Nav2 Spin·DriveOnHeading 또는 cmd_vel) | |
| 7 | (예정) | Nav2 지점 이동 + 전체 순서 (ROS2 노드) | |

```bash
python steps/s0_make_local_calibration.py
python steps/m1_floor_check.py
python steps/m2_teach_look_pose.py
```

SSH에서는 화면 창이 뜨지 않으니, 사진은 `steps/logs/`에 저장된 파일로 확인합니다.

---

## 하드웨어에서 확인할 것

- **넘어짐**: 팔을 앞으로 뻗고 블록을 들 때 앞으로 쏠리지 않는지
- **전원**: 팔 모터 전원을 터틀봇 배터리와 같이 쓸지 따로 쓸지 (전압이 떨어지면 모터 보호가 걸릴 수 있음)
- **USB**: 라즈베리파이에 팔, 손목 카메라, OpenCR이 같이 꽂힘
