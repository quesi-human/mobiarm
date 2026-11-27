execute_robot.py — 파이 Nav2 + 집기·귀환 FSM

노트북 파일(같은 폴더에 유지):
  execute_robot.py       SSH 기동·CLI·노트북 RViz·종료 관리
  robot_mission.py       파이에 자동 배포되는 Nav2 기동·집기/귀환 FSM
  onboard_navigation.launch.py  파이용 Nav2 실행 구성(서버 연결 대기 시간 20초)
  nav2_mobile_pick.yaml   시험용 축소 원형 충돌 모델을 적용한 전체 Nav2 설정
  mobile_pick.rviz       map 고정·원본 스캔·로봇 중심의 진단 화면
  waypoints.yaml        출발/귀환 및 집기 지점 좌표
  clear_map.yaml        지도 설정
  clear_map_updata.pgm   지도 이미지

기본 설정:
  SSH: turtlebot6@192.168.110.176 (비밀번호 저장 안 함)
  ROS_DOMAIN_ID: 36 / TURTLEBOT3_MODEL: waffle_pi
  파이 프로젝트: /home/turtlebot6/so101-mobile-pick
  팔 Python: /home/turtlebot6/arm-venv/bin/python
  팔 제어: 기존 arm_cli.py 재사용 (원본 변경 없음)
  source ~/.bashrc는 노트북/파이 실행 환경에 모두 적용한다.

실행:
  cd ~/Desktop
  python3 execute_robot.py

자동 흐름:
  1. 노트북 파일 확인 및 SSH 인증
  2. 파이에 맵/전체 Nav2 설정/실행 코드 자동 배포
  3. 파이 bringup 실행 → /scan·/odom·TF 수신 확인
  4. 노트북 RViz를 먼저 열어 초기 위치 지정 가능하게 함
  5. 파이 Nav2 실행 → 고정 시작점으로 자동 초기화 → 준비 확인
  6. 출발·귀환 지점 / 집기 목적지 / 색상 입력
  7. map TF 및 베이스 정지 확인 → 팔 접기
  8. 집기 목적지로 이동(x,y,yaw) → 도착 자세·정지 확인
  9. arm_cli pick → picked/holding/carry 결과 확인
  10. 출발 지점으로 복귀
  11. 저장된 place 자세로 내려놓기 → 팔 접기 → 자신이 시작한 프로세스 정리

출발·귀환 지점:
  current: 작업 시작 시 map -> base_link를 측정해 귀환 위치·방향 저장(기본)
  homepoint 등: waypoints.yaml의 해당 좌표로 귀환
  기본은 homepoint의 x/y/yaw로 AMCL을 자동 초기화한다. 매 실행 시 2D Pose Estimate는 필요 없다.
  초기화·Nav2 준비 확인의 /get_state 요청은 3초 안에 응답이 없으면 취소 후 재요청한다.
  최근 6초 안에 확인한 active 상태만 인정하며, 상태·재시도 횟수를 5초마다 출력한다.
  사용자가 맵을 처음 만들기 시작했던 실제 위치·방향에서 항상 시작한다고 확인하여
  homepoint를 (0,0,0)으로 설정했다. 반드시 같은 실제 위치·방향에 놓고 시작한다.
  map의 (0,0)은 임의의 현재 시작점을 뜻하지 않으므로 다른 자리에 놓으면 이 설정이 맞지 않는다.
  다른 고정 시작점은 --set-home X Y YAW로 map 좌표(m,m,rad)를 한 번 저장한다.
  --set-home은 로컬 YAML만 저장하고 SSH 접속·로봇 기동은 하지 않는다.
  수동 초기화가 필요한 경우에만 --manual-initial-pose를 지정한다.
  YAML 좌표·yaw는 실제 로봇을 이동·회전시키지 않는다.
  지점의 x/y/yaw가 null이면 실행 중 측정값을 추가로 입력받는다.
  추가 입력값은 이번 작업 요청에만 저장하며 원본 waypoint 파일을 덮어쓰지 않는다.

현재 waypoints.yaml은 파이의 기존 waypoint.yaml에서 복사했다.
  workbench: x=-3.4, y=-0.47, yaw=-1.558
  homepoint: x=0, y=0, yaw=0 (원래 맵 생성 시작 위치·방향에서 출발하는 조건)

직접 인자 지정:
  python3 execute_robot.py --start current --destination workbench --color red
  current는 위치 추정 후 현재 map 좌표·방향을 귀환점으로 사용한다.
  --target은 --destination의 별칭이다.
  색상: red, yellow, blue, green, orange, purple 및 빨강/노랑/파랑/초록/주황/보라

기존 프로세스:
  이전 노트북 Nav2는 종료해야 한다. 파이 Nav2와 함께 켜지 않도록 중복을 검사한다.
  실물 bringup만 유지: python3 execute_robot.py --reuse-bringup
  파이 bringup/Nav2 모두 유지: --reuse-bringup --reuse-nav2
  --reuse-nav2는 파이 내부 Nav2 프로세스가 실제 존재하는지도 확인한다.
  재사용한 기존 프로세스는 종료하지 않는다.

작업 완료 후:
  기본은 귀환 후 저장된 place 자세로 내려놓고 팔을 접는다.
  블록을 잡은 carry 자세로 끝내려면 --no-place-on-return을 추가한다.
  내려놓기는 파이의 기존 calibration/place_pose.json을 사용한다.

초기 기동만 시험:
  python3 execute_robot.py --setup-only
  이 모드는 집기/이동 요청을 보내지 않고 Ctrl+C까지 유지한다.

하드웨어를 기동하지 않는 확인:
  python3 execute_robot.py --dry-run
  python3 execute_robot.py --check
  --dry-run: SSH 접속 없음.
  --check: 파이에 파일 배포 및 ROS/팔 가상환경 import 확인. 모터 연결·이동 없음.

추가 옵션:
  --no-rviz
  --params-file /path/to/full_nav2_params.yaml
  --rviz-config /path/to/config.rviz
  --local-node 'ros2 run 패키지 실행파일' (반복 지정 가능)
  --timeout 120 / --nav-timeout 180 / --arm-timeout 180 (초)

Nav2 파라미터:
  Waffle Pi는 같은 폴더의 nav2_mobile_pick.yaml을 기본으로 사용한다.
  사용자 요청으로 base_link 기준 robot_radius 0.18m의 축소 원형 모델을 사용한다.
  footprint는 []로 비워 원형 설정이 적용되며 footprint_padding은 0.005m이다.
  이 시험용 모델은 실제 차체 후면·부착물 외곽을 충분히 포함하지 않을 수 있다.
  전역/지역 코스트맵 모두 같은 원형 모델을 사용한다.
  DWB는 원형 모델용 BaseObstacle로 충돌 비용을 평가한다.
  전역/지역 inflation_radius 0.30m, cost_scaling_factor 8.0으로 설정했다.
  이전 사각형 설정은 nav2_mobile_pick_rectangle_backup.yaml에 보관했다.
  진전 검사: 10초 내 50cm 대신 20초 내 10cm 이동을 요구한다(회전은 진전으로 계산하지 않음).
  Pi 4 계산량 조정: 제어 5Hz, DWB 샘플 12x24, planner 기대 주기 1Hz.
  /scan은 obstacle_layer로 처리하며 같은 입력을 받던 voxel_layer는 제거했다.
  센서 장애물 감지와 충돌 판정은 활성화된 상태다.
  비용 감소 계수가 커지면 장애물에서 멀어질수록 비용이 더 빠르게 줄어든다.
  원본 파일은 변경하지 않는다. 복사본의 goal checker 허용 오차는
  최대 0.10m 및 0.10rad로 제한한다. 도착 후 실제 자세 오차가
  0.15m 또는 0.15rad를 넘으면 집기를 시작하지 않는다.
  ros2 param set으로 임시 변경한 inflation 등은 자동 보존되지 않는다.
  보존하려면 조정한 전체 Nav2 YAML을 --params-file로 지정한다.
  global_costmap_debug.yaml은 전체 설정 파일이 아니다.

정지/실패:
  Ctrl+C 또는 실패 시 이동 취소·제로 속도 전송 후 소유 프로세스를 정리한다.
  팔 명령 취소는 기존 arm_cli의 ArmStop 오류 경로를 사용해 유지·분리한다.
  실패 후 자동 재접근·귀환·그리퍼 열기는 수행하지 않는다.
  노트북 연결 종료/heartbeat 15초 중단 시 파이 supervisor도 소유 프로세스를 정리한다.
  로그는 노트북 /tmp/execute-robot-logs-* 및
  파이 ~/so101-mobile-pick/mission_control/runs/<실행ID>/logs에 남는다.
  navigation_telemetry 이벤트에는 /cmd_vel_nav·/cmd_vel 명령과 /odom 실제 속도·지도 좌표가 1초마다 기록된다.
  odom yaw, map->odom/odom->base_link/base_link->base_scan 변환과 스캔 시각의 TF 성공·실패도 기록한다.
  실패 시 파이 logs/navigation_failed.json에 마지막 30개 샘플과 전역/지역 코스트맵을 저장한다.
  같은 파일에 마지막 30개 /scan_filtered 및 해당 시각의 map/odom 변환도 저장한다.
  스캔 헤더 시각·마지막 광선 시각·현재 시각의 차이를 남기며 센서 시각이나 프레임을 임의로 바꾸지 않는다.
  RViz는 기본으로 map, 필터된 LaserScan(/scan_filtered), RobotModel을 표시한다. 코스트맵/TF 그룹은 화면에서만 꺼 둔다.
  /plan은 초록색 선으로 표시해 계획 경로의 출발 방향과 우회 경로를 확인한다.
  스냅샷에는 /plan, /local_plan, /transformed_global_plan 중 수신한 경로를 저장한다.
  Ctrl+C 중단 시에도 navigation_interrupted.json에 마지막 스캔·TF·코스트맵·경로를 저장한다.
  이 표시 변경은 Nav2의 장애물 처리와 충돌 검사를 끄지 않는다.
  이 기록으로 회전/제로 명령만 생성되는지, 전진 명령이 실제 이동으로 이어지는지 구분한다.

오프라인 테스트:
  python3 ~/Desktop/test_robot_mission.py -v
  이전 실행기는 execute_robot_before_onboard.py로 보관했다.

자기 몸체 스캔 필터(3층 구조 기둥 대응):
  execute_robot.py가 scan_self_filter.py를 파이에 함께 업로드하고 bringup 후 기동한다.
  원본 /scan -> scan_self_filter -> /scan_filtered -> AMCL/양쪽 코스트맵/RViz.
  base_link 기준 x=-0.21~+0.09m, y=-0.16~+0.16m 몸체 내부의 반사점만 NaN으로 바꾼다.
  이 범위는 앞서 확인한 Waffle Pi 몸체 범위이며, 주행용 원형 footprint와 별개다.
  범위 밖의 가까운 장애물은 유지한다. 가려진 광선 뒤를 빈 공간이라고 가정하지 않는다.
  스캔 헤더, 프레임, 시각, 각도, 측정 순서, intensities는 유지한다.
  장착 TF가 없으면 스캔을 내보내지 않고 경고한다. 필터 프로세스는 종료 관리에 포함된다.
  기존 Nav2는 종료하고 --reuse-nav2 없이 다시 시작해야 필터 토픽이 적용된다.
  평소 execute_robot.py 명령을 사용하면 된다. Nav2만 별도 기동할 때는 이 필터도 필요하다.
  scan-filter.log에 masked N/M chassis returns가 5초마다 남는다.
  필터 범위를 변경하려면 scan_self_filter.py의 bounds ROS 파라미터 기본값을 수정한다.
  이미 지도 이미지에 저장된 장애물은 필터가 지우지 않는다.
  저장 데이터 재생 및 오프라인 테스트만 수행했으며 실물 주행 검증은 하지 않았다.

U자 우회 경로 추종 변경:
  기본 FollowPath는 DWB 대신 RegulatedPurePursuitController를 사용한다.
  목표 선속도 0.12m/s, 경로 lookahead 0.25m, 방향 정렬 회전속도 0.5rad/s.
  충돌 검사, 장애물 근접 감속, 곡률 감속을 켠다. footprint 크기는 추가로 줄이지 않았다.
  nav2_mobile_pick_dwb_backup.yaml에 변경 직전 DWB 설정을 보관했다.
  필터 적용 후 최신 실행에서는 기존 궤적 전부 충돌 오류 대신 이동 진전 부족만 반복됐다.
  컨트롤러 교체는 이 경로에서 DWB 회전 선택을 개선하기 위한 변경이며 실물 결과는 미검증이다.
  /lookahead_arc와 /received_global_plan도 진단 스냅샷에 저장한다.

작업대 최종 접근/이탈:
  웨이포인트 이동 및 기존 도착 검사 → APPROACHING_PICK(10cm 전진) → 정지 확인
  → PICKING(집기 성공·holding/carry 확인) → RETREATING_FROM_PICK(10cm 후진)
  → RETURNING → 귀환 후 내려놓기. 전진/후진 속도는 0.04m/s.
  Nav2 drive_on_heading/backup 액션을 사용하고 behavior_server에 플러그인을 등록했다.
  전후진은 현재 로봇 정면 기준이며 map의 x/y 방향으로 고정하지 않는다.
  액션 거절/충돌/시간 초과/오도메트리 중단은 작업을 중단한다.
  완료 후 오도메트리로 종방향 거리(±3cm), 옆밀림(3cm), 회전(0.12rad)을 검사한다.
  집기 실패 또는 carry 자세 미확인 시 후진과 귀환을 시작하지 않는다.
  --pick-approach-distance 0.10으로 거리 지정(기본), 0이면 전후진 단계 생략.
  이미 발생한 도착 오차 0.233m는 Nav2 성공 응답 후 FSM의 0.15m 검사를 넘은 상태다.
  작업대 도착 위치 검사는 현재 실험에서 생략한다. Nav2 성공·정지·yaw 검사는 유지한다.
  웨이포인트 파일은 사용자가 수정한 값을 그대로 유지한다.
  오프라인 테스트만 수행했으며 이 변경으로 실물 로봇을 기동하거나 이동시키지 않았다.

집기 전후 전후진용 실제 차체 footprint:
  workbench 도착·정지 후 로컬 footprint를 직사각형으로 전환한다.
  nav2_mobile_pick.yaml > mission_control > ros__parameters > pick_footprint에서 수정한다.
  base_link 기준 앞 +0.09m, 뒤 -0.21m, 좌우 ±0.16m, 기존 padding 추가 적용.
  전역 코스트맵과 이동 중 원형 모델은 그대로이며, 충돌 검사를 계속 수행한다.
  로컬 /footprint 입력 토픽으로 전환하고 /published_footprint 및 갱신된 코스트맵을 확인한다.
  확인 실패 시 전진하지 않는다. 후진 완료 후 관측해 저장했던 원래 이동 모델을 복원한다.
  복원 확인 실패 시 귀환을 시작하지 않는다. 작업 중단 시에도 정지 후 복원을 시도한다.
  실제 전진을 막던 원형 모델보다 앞쪽은 짧고 뒤쪽은 길게 표현하여 실제 차체 형상을 반영한다.
  실물 주행 시험은 수행하지 않았다.

접힘 자세 사전 검사:
  arm_cli의 park는 토크 OFF일 때 자세를 확인하지 않고 parked를 반환할 수 있다.
  올인원 실행기의 팔 브리지에서 실제 관절값을 P.near_park로 검사하도록 보완했다.
  불일치면 베이스 이동 전에 중단한다. 토크를 강제로 켜거나 팔 보정 파일을 변경하지 않는다.
  팔을 저장된 접힘 자세로 놓거나, 실제로 사용하는 정상 접힘 자세를
  cd ~/so101-mobile-pick && ~/arm-venv/bin/python steps/m5_teach_park.py 로 다시 저장한다.
  로봇의 기존 arm_cli.py/robot/tasks.py는 변경하지 않았다.

팔 단독 pick+place:
  파이 프로젝트 루트에 pick_place.py를 추가했다. 성공한 기존 arm_cli.run을 그대로 호출한다.
  cd ~/so101-mobile-pick
  ~/arm-venv/bin/python pick_place.py --color red
  이미 들고 있다면 ~/arm-venv/bin/python pick_place.py --place-only
  pick의 picked/ok/holding/carry를 확인한 뒤 place의 placed/holding=false/park를 확인한다.
  팔 단독 스크립트는 베이스 이동 없이 저장된 place_pose로 내려놓는다.
  execute_robot.py의 전체 미션은 기존처럼 집기 → 후진 → 귀환 → place 순서다.
  파일만 배포했고 실제 pick/place는 실행하지 않았다.

작업대 위치 오차 검사 임시 생략:
  nav2_mobile_pick.yaml > mission_control > ros__parameters > check_pick_position: false
  작업대 NAVIGATING_TO_PICK의 추가 map 위치 0.15m 검사만 생략한다.
  Nav2 액션 성공, 신선한 map TF, 베이스 정지, yaw 0.15rad 검사는 계속 필요하다.
  RETURNING의 위치 0.15m 및 yaw 검사는 유지한다.
  원래 검사를 복구하려면 check_pick_position: true로 바꾸고 실행기를 재시작한다.
  short_move_done에 measured_distance_m를 기록하며, 집기 성공 후 그 실제 전진 거리만큼 후진한다.
  후진은 오도메트리 거리 기반이며 바퀴 미끄러짐 등을 포함한 완전한 map 위치 복귀를 보장하지 않는다.
  전후진의 충돌 검사와 실패 중단은 계속 적용한다.
