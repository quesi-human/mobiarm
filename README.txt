soARM + TurtleBot — 자동 분할 설치 프로젝트
Ubuntu 22.04 / ROS 2 Humble / TurtleBot3 Waffle Pi + LDS-03

1. 노트북과 파이를 한 번에 설치

  cd /home/ssafy/Desktop/pjt_soarm_turtlebot
  bash setup.sh --host turtlebot6@192.168.110.176

  순서: 노트북 ROS/RViz 확인·필요 패키지 설치 → 노트북 .venv
  → SSH 접속 → 파이 전용 파일만 업로드 → 파이 ROS/Nav2/드라이버 확인
  → 별도 파이 프로젝트·arm-venv 설치 → 동작 없는 import 검사 → 접속 경로 저장.
  기존 ROS 환경이 이미 준비되어 있다면 시스템 패키지를 건드리지 않는다.
  필요한 경우에만 sudo가 요청된다. SSH/sudo 비밀번호는 직접 입력하며 저장하지 않는다.
  설치 과정은 bringup/Nav2/팔/카메라를 실행하거나 기기 포트를 열지 않는다.

  이미 ROS가 완전히 설치된 장비에서는 시스템 설치를 생략할 수 있다:
  bash setup.sh --host turtlebot6@192.168.110.176 --skip-system

2. 설치 후 전체 미션 실행

  python3 execute_robot.py --start homepoint --destination workbench --color yellow

  installation/deployment.json에 기록된 새 파이 경로를 자동 사용한다:
    ~/pjt_soarm_turtlebot/so101-mobile-pick
    ~/pjt_soarm_turtlebot/arm-venv/bin/python
    ~/pjt_soarm_turtlebot/setup_env.sh
  노트북은 이 폴더의 .venv를 자동 사용한다.
  기존 파이 ~/so101-mobile-pick, ~/arm-venv와 Desktop 원본 파일은 수정하지 않는다.

3. 역할별 개별 설치

  노트북만:
    bash laptop/setup.sh

  파이에서 직접 설치(프로젝트 폴더를 파이에 복사한 경우):
    bash robot/setup.sh --prefix "$HOME/pjt_soarm_turtlebot"

  다른 설치 경로/도메인:
    bash setup.sh --host USER@IP --remote-prefix /home/USER/pjt_soarm_turtlebot --domain-id 36

  기존 ROS 워크스페이스가 없으면 묶어 둔 TurtleBot3/LDS-03 소스를 빌드한다.
  기존 워크스페이스 대신 새로 빌드하려면 --build-drivers를 추가한다.
  설치기는 ~/.bashrc를 변경하지 않는다. 실행기는 ~/.bashrc 후 전용 setup_env.sh를 읽는다.
  새로 dialout/video 그룹에 추가된 사용자는 로그아웃/로그인 후 설치를 다시 실행해야 한다.
  새 하드웨어에는 해당 로봇에 맞는 모터·카메라·자세 보정이 필요하다.

4. requirements와 시스템 패키지

  laptop/requirements.txt           노트북 Python: PyYAML
  laptop/ros_packages.txt           노트북 apt: ROS/RViz/TurtleBot3 관련 패키지
  robot/requirements.txt            파이 Python: numpy/scipy/OpenCV/servo SDK/pyserial/PyYAML
  robot/ros_packages.txt            파이 apt: ROS/Nav2/빌드 도구/USB 카메라 도구

  ROS는 pip requirements로 설치하지 않고 시스템 패키지로 설치한다.
  설치 후 실제 Python 버전은 .venv-requirements.lock.txt와 파이
  ~/pjt_soarm_turtlebot/arm-venv-requirements.lock.txt에 기록한다.
  과거 환경 전체 freeze는 environment/arm_venv_freeze.txt이며 설치 입력으로 사용하지 않는다.

5. 실행 없는 검증

  bash setup.sh --dry-run
  python3 execute_robot.py --dry-run --start homepoint --destination workbench --color yellow
  python3 controller/test_robot_mission.py -q
  python3 install/test_setup.py -q

  python3 execute_robot.py --check --no-rviz
  위 --check는 파일 배포·의존성 검사만 수행하며 ROS 노드/모터/카메라는 기동하지 않는다.

6. 수정할 설정과 지도

  controller/waypoints.yaml               작업대·홈 좌표(x,y,yaw)
  controller/nav2_mobile_pick.yaml        Nav2·접근 footprint·집기 위치 검사
  controller/clear_map.yaml               지도 메타데이터
  controller/clear_map_updata.pgm         지도 이미지
  controller/mobile_pick.rviz             RViz 표시

  gimp /home/ssafy/Desktop/pjt_soarm_turtlebot/controller/clear_map_updata.pgm
  수정 후 같은 크기의 PGM으로 내보내고 실행기를 재시작한다.

7. 미션 흐름

  파이 bringup → 자기 몸체 필터 → 파이 Nav2 → 노트북 RViz
  → YAML 초기 좌표 설정 → 팔 실제 접힘 검사 → 작업대 이동
  → 접근 footprint 확인 → 10cm 전진 → pick·carry 확인
  → 측정된 전진 거리만큼 후진 → 이동 footprint 복원 → 귀환 → place
  → IDLE 대기 → 다음 명령.

  실험 설정 check_pick_position: false는 작업대 위치 추가 검사만 생략한다.
  Nav2 성공·정지·yaw·충돌 검사와 귀환 위치 검사는 유지한다.
  homepoint 부근에 로봇을 놓으면 초기 추정 후 라이다로 보정하고 홈으로 실제 이동한다.
  이는 홈 근처의 작은 수동 배치 오차 보정용이며, 임의 장소에서의 전역 위치 인식은 아니다.

8. 새 파이 설치 경로에서 팔 단독 실행

  cd ~/pjt_soarm_turtlebot/so101-mobile-pick
  ../arm-venv/bin/python pick_place.py --color red
  이미 들고 있다면 ../arm-venv/bin/python pick_place.py --place-only

  전체 미션과 팔 단독 명령을 동시에 실행하지 않는다.

9. 폴더 구성

  setup.sh / install/          자동 설치·배포·검증 코드
  execute_robot.py            설치 정보와 전용 Python을 자동 선택하는 실행 진입점
  laptop/                     노트북 requirements와 설치 스크립트
  controller/                 통합 실행기·FSM·필터·지도·설정·테스트
  robot/so101-mobile-pick/     파이 소암 소스·보정값 전체 보관본
  robot/ros_ws/               TurtleBot3/LDS-03 드라이버 소스 스냅샷
  robot/requirements.txt      파이 팔 전용 Python 의존성
  installation/              현재 노트북/파이 설치 결과와 자동 접속 설정
  environment/               원래 장비 환경 기록
  docs/                      파일 안내·검증 기록

  파이에 배포할 때 state, 과거 runs/logs, Git, 캐시는 제외한다.
  재설치 시 새 설치 경로의 기존 calibration/state는 덮어쓰지 않는다.
  변경된 코드는 새 설치 경로 backups/<시각>에 먼저 보관한다.

10. 반복 작업 세션(2026-11-27)

  기본값은 연속 명령 모드이다. 첫 작업은 CLI의 --start/--destination/--color를 사용한다.
  완료하면 같은 bringup/Nav2/RViz/SSH 세션을 유지하고 다음 작업 옵션을 한 줄로 받는다.
  예: --start homepoint --destination workbench --color red
  생략한 옵션은 최초 실행 옵션을 사용한다(최초 기본값: current/workbench/red).
  실행 중 지점·색상·yaw를 개별 질문하지 않으며, 미등록 좌표는 실행 전에 오류로 안내한다.
  q, quit, exit, 종료 또는 Ctrl+C로 세션을 종료하고 소유 프로세스를 정리한다.
  한 번만 실행하려면 --once를 추가한다. --setup-only는 기존 기동 유지 전용 모드이다.

  매 작업 시작 시 최신 map TF·팔 접힘·정지 확인 후 바로 작업대로 이동한다.
  별도 정지 라이다 보정, AMCL 불확실성 검사, homepoint 선행 이동·정렬은 수행하지 않는다.
  최초 YAML 초기 좌표 전송과 Nav2의 기본 AMCL 위치 추정은 유지한다.
  start=current는 작업 시작 당시 추정 위치를 귀환점으로 저장한다.
  --start homepoint 등 이름을 지정하면 기존처럼 해당 지점이 귀환점이다.
  대기 중 YAML을 수정했다면 적용을 위해 실행기를 재시작한다.

  로그: 파이 실행 루트 logs/mission-0001/, mission-0002/ ...
  노트북에는 mission-request-0001.json, mission-request-0002.json ...을 저장한다.
  홈 전용 controller/home_alignment.xml, 일반 이동 controller/navigation.xml을 함께 배포한다.
  --once 외에는 성공 후 스택을 자동 종료하지 않는다. 오류/연결 상실 시에는 정리한다.
  이번 변경은 오프라인으로 검증했으며 실물 기동·이동은 수행하지 않았다.

Git 구성 수정(2026-11-27):
  robot/so101-mobile-pick는 별도 Git 참조가 아닌 일반 소스 폴더로 포함한다.
  소스 코드와 calibration은 상위 저장소가 직접 추적한다.
  state, __pycache__, steps/logs, mission_control/runs는 Git에서 제외한다.
  이전 내부 Git 이력은 Desktop의 pjt_soarm_turtlebot_git_backup_20261127_104225/soarm.git에 보관했다.

11. 집기 후 자동 후진 복구

  기본 Nav2 후진이 실패하면 취소·정지 확인 후 오도메트리로 진행 거리를 검사한다.
  목표 거리 미달 시 5cm씩 최대 3회 추가 후진한다(속도 0.03m/s).
  추가 후진에만 Nav2 코스트맵 충돌 검사를 우회한다. 자동 전진/주행은 기존 검사를 유지한다.
  오도메트리 갱신 중단, 2초간 진전 없음, 방향·옆밀림 오차 시 정지·작업 보류한다.
  최종 실패 시 아래 실패 정리 절차를 수행한 뒤 세션을 종료한다.

12. 동작 실패 정리

  이동·집기·내려놓기 등의 최종 실패 시 베이스와 기존 팔 프로세스를 정지한다.
  arm lock 획득 → 저장된 park 자세 복귀·실제 관절 확인 → 그리퍼 45% 개방
  → 블록 보유 상태 초기화 → 팔·그리퍼 전체 토크 OFF 확인 → park 상태 저장
  → 프로세스 및 bringup/Nav2/RViz/SSH 종료 순서로 정리한다.
  블록 보유 여부와 무관하게 park 자세에서 해제한다. 정상 완료 시 연속 명령 대기는 유지한다.
  복구 이동/통신/토크 해제 확인 실패는 arm_recovery_failed로 기록하고 종료한다.
  park 확인 실패 시 펼쳐진 팔의 강제 토크 OFF를 수행하지 않는다.
  Ctrl+C·제어 연결 강제 종료는 정지 경로이며 새 복구 이동을 시작하지 않는다.
