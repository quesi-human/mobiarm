soARM + TurtleBot 통합 프로젝트
정리 기준: 2026-11-26, Asia/Seoul

기존 Desktop 파일과 파이 프로젝트를 수정하거나 이동하지 않고 복사한 프로젝트입니다.

1. 노트북에서 전체 미션 실행

  cd /home/ssafy/Desktop/pjt_soarm_turtlebot
  python3 execute_robot.py --start homepoint --destination workbench --color yellow

  루트 execute_robot.py는 controller/execute_robot.py를 실행하는 진입점입니다.
  설정과 지도는 새 폴더의 controller/ 내부 파일을 사용합니다.
  파이에서는 기존 /home/turtlebot6/so101-mobile-pick 및 ~/arm-venv를 사용합니다.
  실행 때마다 최신 controller 코드를 파이 mission_control/runs/<실행ID>에 업로드합니다.
  robot/ 폴더는 파이 소스 전체의 보관본이며, 이번 정리에서는 파이에 새 파일을 배포하지 않았습니다.

2. 실행 없는 확인

  python3 execute_robot.py --dry-run --start homepoint --destination workbench --color yellow
  python3 controller/test_robot_mission.py -q

  --dry-run과 위 테스트는 SSH 접속, ROS 노드 기동, 모터 연결을 하지 않습니다.

3. 수정할 파일

  controller/waypoints.yaml                 작업대·홈의 x, y, yaw
  controller/nav2_mobile_pick.yaml          Nav2 및 집기 접근 설정
  controller/clear_map.yaml                 지도 메타데이터
  controller/clear_map_updata.pgm           실제 지도 이미지
  controller/mobile_pick.rviz               RViz 표시

  지도 수정:
  gimp /home/ssafy/Desktop/pjt_soarm_turtlebot/controller/clear_map_updata.pgm

  PGM으로 내보내고 기존 이미지 크기를 유지하세요. 수정 후 실행기를 재시작합니다.
  원본 Desktop 파일을 수정해도 이 정리본에 자동 동기화되지 않으며, 반대도 마찬가지입니다.

4. 현재 미션 흐름

  SSH → 파이 bringup → 자기 몸체 스캔 필터 → 파이 Nav2 → 노트북 RViz
  → 홈 자동 초기화 → 실제 팔 접힘 확인 → 작업대 이동
  → 작업대용 직사각형 footprint 전환·확인 → 정면 10cm 접근
  → arm_cli.py pick → 운반 자세 확인 → 측정된 전진 거리만큼 후진
  → 이동용 footprint 복원 → 귀환 → arm_cli.py place → 소유 프로세스 정리

  현재 작업대 위치 오차 추가 검사는 생략합니다(check_pick_position: false).
  Nav2 성공·yaw·베이스 정지·충돌 검사는 유지합니다. 귀환 위치 검사는 유지합니다.
  homepoint 자동 초기화는 실제 로봇을 맵 생성 시작 위치·방향에 둘 때 사용하는 설정입니다.
  팔 접근 거리: --pick-approach-distance 0.10 (0이면 전후진 생략)
  귀환 후 내려놓기가 기본입니다. --no-place-on-return은 운반 자세를 유지합니다.

5. 팔 단독 실행: 터틀봇 터미널

  cd ~/so101-mobile-pick
  ~/arm-venv/bin/python pick_place.py --color red

  이미 블록을 들고 있다면:
  ~/arm-venv/bin/python pick_place.py --place-only

  팔 단독 스크립트는 베이스를 이동하지 않고 저장된 place_pose로 내려놓습니다.
  단독 팔 명령과 올인원 명령을 동시에 실행하지 마세요.

6. 폴더 구성

  execute_robot.py                 새 폴더의 실행 진입점
  controller/                      통합 실행 코드·현재 지도·설정·오프라인 테스트
  robot/so101-mobile-pick/          파이 프로젝트 전체 복사본(보정값·steps·로그·Git 포함)
  archive/desktop_history/          과거 설정, 보조 웨이포인트 코드, 원본 안내, GIMP XCF
  archive/local_runs/               /tmp/execute-robot-logs-* 복사본
  archive/diagnostics/              수집한 TF·코스트맵·카메라 진단 자료
  archive/robot_project_snapshot.tar.gz   파이 프로젝트 원본 압축 스냅샷
  environment/                     Python/ROS 환경 기록 및 의존성
  docs/                            파일 역할·복사 출처·검증 기록

  ROS 시스템 설치, turtlebot3_ws, arm-venv 자체는 외부 실행 환경이며 이 폴더에 포함되지 않습니다.
  파이 프로젝트 requirements.txt와 environment/arm_venv_freeze.txt를 함께 참고하세요.
  비밀번호·SSH 개인키는 추가로 저장하지 않았습니다.

7. 검증 범위

  새 controller에서 오프라인 테스트 55개 통과.
  루트 진입점 --dry-run 정상, 복사한 지도 참조 경로 정상.
  Desktop 원본 파일 21개와 복사본의 SHA-256 일치 확인.
  이번 정리 과정에서 로봇/팔을 기동하거나 움직이지 않았습니다.
