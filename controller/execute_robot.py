#!/usr/bin/env python3
"""Laptop CLI: robot-side bringup, Nav2 and pick/return FSM; laptop-side RViz."""
import argparse
import json
import os
from pathlib import Path
import select
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid

from robot_mission import PREFIX, COLORS, last_json, validate_point, validate_request, stop_group


class SessionExit(Exception):
    pass


def shell_command(argv, domain, model, setup_file=None):
    setup_file = setup_file or os.environ.get('PJT_SOARM_LOCAL_ENV')
    # -i passes Ubuntu .bashrc's interactive guard; --norc avoids double sourcing.
    return ['bash', '--norc', '-ic', 'source "$HOME/.bashrc"\n'
            + (f'source {shlex.quote(str(setup_file))} || exit $?\n' if setup_file else '') +
            f'export ROS_DOMAIN_ID={shlex.quote(str(domain))}\n'
            f'export TURTLEBOT3_MODEL={shlex.quote(model)}\n'
            'exec ' + shlex.join([str(x) for x in argv])]


class Processes:
    def __init__(self, directory):
        self.directory, self.entries = directory, []

    def start(self, name, command, stdin=None):
        path = self.directory / (name + '.log')
        log = path.open('wb')
        try:
            process = subprocess.Popen(command, stdin=stdin, stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True)
        except BaseException:
            log.close()
            raise
        self.entries.append((name, process, log, path))
        print(f'[{name}] 로그: {path}', flush=True)
        return process, path

    def check(self):
        for name, process, _, path in self.entries:
            if process.poll() is not None:
                raise RuntimeError(f'{name} 종료({process.returncode}): '
                                   + path.read_text(errors='replace')[-3000:])

    def close(self):
        for _, process, log, _ in reversed(self.entries):
            stop_group(process)
            if process.stdin:
                try:
                    process.stdin.close()
                except (BrokenPipeError, OSError):
                    pass
            log.close()


class SSH:
    def __init__(self, host):
        self.host = host
        self.directory = tempfile.TemporaryDirectory(prefix='robot-ssh-')
        self.base = ['ssh', '-S', str(Path(self.directory.name) / 'control'),
            '-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=5',
            '-o', 'ServerAliveCountMax=3']

    def connect(self):
        print(f'SSH 연결: {self.host} (암호는 SSH 프롬프트에 입력)', flush=True)
        subprocess.run(self.base + ['-M', '-o', 'ControlPersist=60', '-fnN', self.host], check=True)

    def command(self, argv):
        return self.base + ['-o', 'BatchMode=yes', '-T', self.host, shlex.join(argv)]

    def run(self, argv, data=None, timeout=45):
        result = subprocess.run(self.command(argv), input=data, capture_output=True, timeout=timeout)
        if result.returncode:
            raise RuntimeError('원격 명령 실패:\n' + result.stderr.decode(errors='replace')[-2000:]
                               + result.stdout.decode(errors='replace')[-2000:])
        return result.stdout.decode(errors='replace')

    def upload(self, destination, data):
        code = ('import pathlib,sys; p=pathlib.Path(sys.argv[1]); '
                'p.parent.mkdir(parents=True,exist_ok=True); t=p.with_suffix(p.suffix+".tmp"); '
                't.write_bytes(sys.stdin.buffer.read()); t.replace(p)')
        self.run(['/usr/bin/python3', '-c', code, str(destination)], data=data)

    def close(self):
        try:
            subprocess.run(self.base + ['-O', 'exit', self.host], timeout=5,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except (OSError, subprocess.TimeoutExpired):
            pass
        self.directory.cleanup()


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parent
    p.add_argument('--host', default='turtlebot6@192.168.110.176')
    p.add_argument('--map', type=Path, default=root / 'clear_map.yaml')
    p.add_argument('--waypoints', type=Path, default=root / 'waypoints.yaml')
    p.add_argument('--set-home', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                   help='고정 시작점 map 좌표(m,m,rad)를 저장하고 종료(SSH/로봇 실행 없음)')
    p.add_argument('--params-file', type=Path, help='전체 Nav2 설정(기본: nav2_mobile_pick.yaml)')
    p.add_argument('--rviz-config', type=Path)
    p.add_argument('--remote-project', help='파이 프로젝트 절대경로; 기본 ~/so101-mobile-pick')
    p.add_argument('--remote-arm-python', help='파이의 팔 가상환경 Python 절대경로')
    p.add_argument('--remote-env', help='파이 전용 ROS 환경 setup_env.sh')
    p.add_argument('--domain-id', type=int, default=36)
    p.add_argument('--model', default='waffle_pi')
    p.add_argument('--start', help='출발·귀환 지점 이름 또는 current')
    p.add_argument('--initial-waypoint', default='homepoint', help='자동 초기화에 사용할 고정 시작 지점(기본 homepoint)')
    initial = p.add_mutually_exclusive_group()
    initial.add_argument('--manual-initial-pose', dest='manual_initial_pose', action='store_true',
                         help='이번 실행만 RViz에서 실제 초기 위치 지정')
    initial.add_argument('--auto-initial-pose', dest='manual_initial_pose', action='store_false',
                         help='저장된 고정 시작점으로 자동 초기화(기본)')
    p.set_defaults(manual_initial_pose=False)
    p.add_argument('--destination', '--target', dest='destination', help='집기 지점 이름')
    p.add_argument('--color', help='블록 색상 (한글/영문)')
    p.add_argument('--place-on-return', action=argparse.BooleanOptionalAction, default=True,
                   help='귀환 후 내려놓기(기본); --no-place-on-return이면 운반 자세 유지')
    p.add_argument('--reuse-bringup', action='store_true')
    p.add_argument('--reuse-nav2', action='store_true', help='파이 내부 Nav2만 재사용')
    p.add_argument('--no-rviz', action='store_true')
    p.add_argument('--setup-only', action='store_true', help='기동만 하고 작업 입력 없이 대기')
    p.add_argument('--once', action='store_true', help='작업 1회 후 종료(기본: 다음 명령 대기)')
    p.add_argument('--timeout', type=float, default=120)
    p.add_argument('--nav-timeout', type=float, default=180)
    p.add_argument('--arm-timeout', type=float, default=180)
    p.add_argument('--pick-approach-distance', type=float, default=.10,
                   help='집기 전 정면 전진/집기 후 후진 거리(m, 기본 0.10; 0이면 생략)')
    p.add_argument('--local-node', action='append', default=[], metavar='COMMAND')
    p.add_argument('--dry-run', action='store_true', help='접속·기동 없이 계획 표시')
    p.add_argument('--check', action='store_true', help='파일 배포·의존성 확인만, 실제 기동 없음')
    a = p.parse_args()
    if a.host.startswith('-') or not 0 <= a.domain_id <= 232:
        p.error('SSH host 또는 ROS_DOMAIN_ID 오류')
    if min(a.timeout, a.nav_timeout, a.arm_timeout) < 5:
        p.error('timeout은 5초 이상이어야 합니다.')
    if not 0 <= a.pick_approach_distance <= .30:
        p.error('pick-approach-distance는 0~0.30m여야 합니다.')
    return a


def local_metadata(args):
    import yaml
    path = args.map.expanduser().resolve(strict=True)
    mapping = yaml.safe_load(path.read_text())
    image = Path(mapping['image']).expanduser()
    image = (image if image.is_absolute() else path.parent / image).resolve(strict=True)
    points = yaml.safe_load(args.waypoints.expanduser().read_text())
    if not isinstance(points, dict) or not points:
        raise ValueError('waypoints.yaml에 지점을 등록하세요.')
    if not args.manual_initial_pose:
        try:
            validate_point(points.get(args.initial_waypoint))
        except ValueError as exc:
            raise ValueError(f'{args.initial_waypoint} 고정 시작 좌표가 미등록입니다. '
                             '실제 map 좌표를 --set-home X Y YAW로 한 번 저장하세요.') from exc
    code = ('import json,sys; from pathlib import Path; '
            'from ament_index_python.packages import get_package_share_directory as get; '
            'p=Path(get("turtlebot3_navigation2")); '
            'print(json.dumps({"params":str(p/"param"/"humble"/(sys.argv[1]+".yaml")),'
            '"rviz":str(p/"rviz"/"tb3_navigation2.rviz")}))')
    result = subprocess.run(shell_command(['/usr/bin/python3', '-c', code, args.model],
        args.domain_id, args.model), capture_output=True, text=True, timeout=20)
    if result.returncode:
        raise RuntimeError('노트북 ROS 환경 오류: ' + result.stderr[-2000:])
    metadata = last_json(result.stdout)
    profile = Path(__file__).with_name('nav2_mobile_pick.yaml')
    params = (args.params_file.expanduser() if args.params_file else
              profile if args.model == 'waffle_pi' and profile.is_file() else Path(metadata['params']))
    local_rviz = Path(__file__).with_name('mobile_pick.rviz')
    rviz = (args.rviz_config.expanduser() if args.rviz_config else
            local_rviz if local_rviz.is_file() else Path(metadata['rviz']))
    if not args.no_rviz and not rviz.is_file():
        raise ValueError(f'RViz 설정 없음: {rviz}')
    parameters = yaml.safe_load(params.read_text())
    if not isinstance(parameters, dict) or 'controller_server' not in parameters or 'amcl' not in parameters:
        raise ValueError('코스트맵 덤프가 아닌 전체 Nav2 params-file이 필요합니다.')
    # Adjust a per-run copy, preserving the user's original parameter file.
    parameters['amcl']['ros__parameters']['scan_topic'] = '/scan_filtered'
    for name in ('local_costmap', 'global_costmap'):
        params = parameters.get(name, {}).get(name, {}).get('ros__parameters', {})
        for layer in params.get('plugins', []):
            settings = params.get(layer, {})
            for source in str(settings.get('observation_sources', '')).split():
                observation = settings.get(source, {})
                if observation.get('data_type', 'LaserScan') == 'LaserScan':
                    observation['topic'] = '/scan_filtered'
    controller = parameters['controller_server']['ros__parameters']
    controller['home_goal_checker'] = {'plugin':'nav2_controller::SimpleGoalChecker',
        'stateful':True, 'xy_goal_tolerance':.03, 'yaw_goal_tolerance':.04}
    if 'home_goal_checker' not in controller.setdefault('goal_checker_plugins', ['general_goal_checker']):
        controller['goal_checker_plugins'].append('home_goal_checker')
    for name in controller.get('goal_checker_plugins', ['general_goal_checker']):
        checker = controller.get(name)
        if isinstance(checker, dict):
            for key in ('xy_goal_tolerance', 'yaw_goal_tolerance'):
                checker[key] = min(float(checker.get(key, .1)), .1)
    return mapping, image, parameters, rviz, points


def deploy(args, ssh, metadata, session):
    import yaml
    mapping, image, parameters, rviz, points = metadata
    home = ssh.run(['/usr/bin/python3', '-c', 'from pathlib import Path; print(Path.home())']).strip()
    project = Path(args.remote_project) if args.remote_project else Path(home) / 'so101-mobile-pick'
    if not project.is_absolute():
        raise ValueError('--remote-project는 파이의 절대경로여야 합니다.')
    directory = project / 'mission_control' / 'runs' / session
    worker = directory / 'robot_mission.py'
    ssh.upload(worker, Path(__file__).with_name('robot_mission.py').read_bytes())
    ssh.upload(directory / 'scan_self_filter.py',
               Path(__file__).with_name('scan_self_filter.py').read_bytes())
    ssh.upload(directory / 'onboard_navigation.launch.py',
               Path(__file__).with_name('onboard_navigation.launch.py').read_bytes())
    ssh.upload(directory / 'home_alignment.xml', Path(__file__).with_name('home_alignment.xml').read_bytes())
    ssh.upload(directory / 'navigation.xml', Path(__file__).with_name('navigation.xml').read_bytes())
    ssh.upload(directory / 'map.pgm', image.read_bytes())
    ssh.upload(directory / 'map.yaml', yaml.safe_dump(dict(mapping, image='map.pgm')).encode())
    ssh.upload(directory / 'nav2_params.yaml', yaml.safe_dump(parameters).encode())
    ssh.upload(directory / 'waypoints.yaml', yaml.safe_dump(points, allow_unicode=True).encode())
    config = {'map': str(directory / 'map.yaml'),
        'params_file': str(directory / 'nav2_params.yaml'), 'logs': str(directory / 'logs'),
        'arm_python': str(getattr(args, 'remote_arm_python', None) or Path(home) / 'arm-venv/bin/python'),
        'arm_cli': str(project / 'arm_cli.py'),
        'timeout': args.timeout, 'nav_timeout': args.nav_timeout, 'arm_timeout': args.arm_timeout,
        'reuse_bringup': args.reuse_bringup, 'reuse_nav2': args.reuse_nav2,
        'place_on_return': args.place_on_return,
        'pick_approach_distance': getattr(args, 'pick_approach_distance', .10),
        'check_pick_position': parameters.get('mission_control', {}).get('ros__parameters', {})
                                .get('check_pick_position', False),
        'scan_filter': str(directory / 'scan_self_filter.py'),
        'initial_pose': None if args.manual_initial_pose else validate_point(points[args.initial_waypoint]),
        'home_pose': validate_point(points.get('homepoint')),
        'home_behavior_tree': str(directory/'home_alignment.xml'),
        'normal_behavior_tree': str(directory/'navigation.xml'),
        'once': getattr(args, 'once', False),
        'navigation_launch': str(directory / 'onboard_navigation.launch.py')}
    config_path = directory / 'config.json'
    ssh.upload(config_path, json.dumps(config).encode())
    return worker, config_path


class Events:
    def __init__(self, path):
        self.path, self.offset, self.partial = path, 0, ''

    def read(self):
        with self.path.open(errors='replace') as stream:
            stream.seek(self.offset)
            chunk = stream.read()
            self.offset = stream.tell()
        lines = (self.partial + chunk).split('\n')
        self.partial = lines.pop()
        out = []
        for line in lines:
            if not line.startswith(PREFIX):
                continue
            item = json.loads(line[len(PREFIX):])
            out.append(item)
            print('[로봇] ' + json.dumps(item, ensure_ascii=False), flush=True)
            if item['event'] == 'error':
                raise RuntimeError(item['message'])
        return out

    def wait(self, wanted, manager, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if any(item['event'] == wanted for item in self.read()):
                return
            manager.check()
            time.sleep(.2)
        raise RuntimeError(f'{wanted} 대기 시간 초과')


def terminal_input(prompt, check):
    print(prompt, end='', flush=True)
    while True:
        check()
        readable, _, _ = select.select([sys.stdin], [], [], .2)
        if readable:
            line = sys.stdin.readline()
            if not line:
                raise EOFError('입력 종료')
            value=line.strip()
            if value.lower() in ('q','quit','exit','종료'):
                raise SessionExit()
            return value


def collect_request(args, points, ask):
    aliases = {'빨강': 'red', '빨간': 'red', '빨간색': 'red', '노랑': 'yellow',
        '노란': 'yellow', '노란색': 'yellow', '파랑': 'blue', '파란색': 'blue',
        '초록': 'green', '초록색': 'green', '주황': 'orange', '주황색': 'orange',
        '보라': 'purple', '보라색': 'purple'}
    print('등록 지점: ' + ', '.join(points) + ' / current = 현재 위치를 귀환점으로 기록')
    print('색상: red(빨강), yellow(노랑), blue(파랑), green(초록), orange(주황), purple(보라)')
    start = args.start or ask('출발·귀환 지점 [current]: ') or 'current'
    destination = args.destination or ask('집기 목적지 [workbench]: ') or 'workbench'
    color = args.color or ask('집을 블록 색상 [red]: ') or 'red'
    color = aliases.get(color.strip().lower(), color.strip().lower())
    if color not in COLORS:
        raise ValueError('색상: ' + ', '.join(sorted(COLORS)))

    def point(name):
        if name not in points or not isinstance(points[name], dict):
            raise ValueError('등록되지 않은 지점: ' + name)
        data = dict(points[name])
        for key in ('x', 'y', 'yaw'):
            if data.get(key) is None:
                unit = 'rad' if key == 'yaw' else 'm'
                data[key] = float(ask(f'{name}.{key} 미등록 — 측정값({unit}): '))
        return validate_point(data)

    request = {'start_name': start, 'destination_name': destination,
        'start': 'current' if start == 'current' else point(start),
        'destination': point(destination), 'color': color}
    validate_request(request)
    return request


def save_home(path, coordinates):
    import yaml
    path = path.expanduser()
    point = validate_point(dict(zip(('frame_id', 'x', 'y', 'yaw'), ['map', *coordinates])))
    points = yaml.safe_load(path.read_text()) if path.exists() else {}
    if not isinstance(points, dict):
        raise ValueError('waypoints.yaml은 지점 이름을 키로 하는 YAML이어야 합니다.')
    points['homepoint'] = point
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(yaml.safe_dump(points, sort_keys=False, allow_unicode=True))
    temporary.replace(path)
    return point


def main():
    args = parse_args()
    if args.set_home is not None:
        point = save_home(args.waypoints, args.set_home)
        print('고정 홈 저장 완료: ' + json.dumps(point, ensure_ascii=False))
        print('이 명령은 로봇에 접속하거나 이동시키지 않습니다.')
        return 0
    if args.dry_run:
        print(f'파이 {args.host}: source ~/.bashrc → bringup → Nav2(headless) → 집기·귀환 FSM')
        print('노트북: source ~/.bashrc → RViz → 출발·귀환 / 집기 목적지 / 색상 입력')
        print('초기 위치: ' + ('RViz 수동 지정' if args.manual_initial_pose else args.initial_waypoint + ' 자동 지정'))
        print(f'집기 접근: 도착 확인 → 정면 {args.pick_approach_distance*100:g}cm 전진 → '
              f'집기·운반 자세 확인 → {args.pick_approach_distance*100:g}cm 후진 → 귀환')
        print('작업 시작 전 홈 정렬. 완료 후 다음 명령 대기(--once는 종료). 귀환 후 ' +
              ('내려놓고 팔을 접습니다.' if args.place_on_return else '블록을 운반 자세로 유지합니다.'))
        if args.start or args.destination or args.color:
            import yaml
            request = collect_request(args, yaml.safe_load(args.waypoints.expanduser().read_text()), input)
            print(json.dumps(request, ensure_ascii=False, indent=2))
        return 0
    metadata = local_metadata(args)
    session = time.strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:8]
    logs = Path(tempfile.mkdtemp(prefix='execute-robot-logs-'))
    manager, ssh = Processes(logs), SSH(args.host)
    stop_heartbeat = threading.Event()
    heartbeat, remote = None, None
    send_lock = threading.Lock()

    def send(message):
        with send_lock:
            remote.stdin.write((json.dumps(message, ensure_ascii=False) + '\n').encode())
            remote.stdin.flush()

    try:
        ssh.connect()
        worker, config_path = deploy(args, ssh, metadata, session)
        command = lambda mode: shell_command(['/usr/bin/python3', '-u', str(worker),
            '--mode', mode, '--config', str(config_path)], args.domain_id, args.model,
            getattr(args, 'remote_env', None))
        print(ssh.run(command('check'), timeout=65), flush=True)
        if args.check:
            print('배포·의존성 확인 완료. bringup/Nav2/팔은 기동하지 않았습니다.')
            return 0
        remote, remote_log = manager.start('remote-stack', ssh.command(command('stack')),
                                           stdin=subprocess.PIPE)

        def keepalive():
            while not stop_heartbeat.is_set():
                try:
                    send({'type': 'heartbeat'})
                except (OSError, ValueError):
                    return
                stop_heartbeat.wait(2)

        heartbeat = threading.Thread(target=keepalive, daemon=True)
        heartbeat.start()
        events = Events(remote_log)
        # RViz must start BEFORE readiness: some Nav2 activations wait for
        # map -> base_link, which needs the user's initial pose from RViz.
        if not args.no_rviz:
            manager.start('rviz', shell_command(['ros2', 'run', 'rviz2', 'rviz2',
                '-d', str(metadata[3]), '-f', 'map'], args.domain_id, args.model),
                stdin=subprocess.DEVNULL)
        for index, command_text in enumerate(args.local_node):
            argv = shlex.split(command_text)
            if not argv:
                raise ValueError('--local-node 명령이 비어 있습니다.')
            manager.start(f'local-{index+1}', shell_command(argv, args.domain_id, args.model),
                          stdin=subprocess.DEVNULL)
        if args.manual_initial_pose:
            print('RViz 2D Pose Estimate로 실제 초기 위치·방향을 맞추세요.', flush=True)
        else:
            print(f'{args.initial_waypoint} 부근에서 초기 추정 → 라이다 위치 갱신 → homepoint로 실제 이동·정렬합니다.', flush=True)
        events.wait('ready', manager, args.timeout * 3 + 90)
        if args.setup_only:
            print('기동 유지 모드: 종료 Ctrl+C', flush=True)
            while True:
                events.read()
                manager.check()
                time.sleep(.2)

        def check():
            events.read()
            manager.check()

        count=0
        initial_args=True
        while True:
            request_args = args if initial_args else argparse.Namespace(**dict(vars(args),start=None,destination=None,color=None))
            initial_args=False
            try:
                request = collect_request(request_args, metadata[4], lambda prompt: terminal_input(prompt, check))
            except ValueError as exc:
                print(f'입력 오류: {exc}. 다시 입력하세요.',flush=True)
                continue
            count+=1
            (logs / f'mission-request-{count:04d}.json').write_text(json.dumps(request, ensure_ascii=False, indent=2))
            print(f"실행: 홈 정렬 → {request['destination_name']}에서 "
                  f"{request['color']} 집기 → {request['start_name']} 귀환", flush=True)
            send({'type': 'mission', 'request': request})
            total = (args.timeout + args.nav_timeout * 3 +
                     args.arm_timeout * (4 if args.place_on_return else 3) + 120)
            events.wait('mission_done', manager, total)
            if args.once:
                print('작업 완료. --once 설정으로 종료합니다.',flush=True)
                remote.wait(timeout=35)
                return 0
            print('작업 완료. 스택을 유지하고 다음 명령을 기다립니다. 종료: q 또는 Ctrl+C',flush=True)
    except SessionExit:
        print('명령 대기를 종료하고 소유 프로세스를 정리합니다.',flush=True)
        return 0
    except (KeyboardInterrupt, EOFError):
        print('\n작업 취소. 이동·작업 및 소유 프로세스를 정리합니다.', flush=True)
        return 130
    finally:
        if remote and remote.poll() is None:
            try:
                send({'type': 'stop'})
                remote.wait(timeout=35)
            except (OSError, ValueError, subprocess.TimeoutExpired):
                pass
        stop_heartbeat.set()
        if heartbeat:
            heartbeat.join(timeout=3)
        manager.close()
        ssh.close()
        print(f'로그: {logs}', flush=True)


if __name__ == '__main__':
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        raise SystemExit(main())
    except Exception as exc:
        print('오류: ' + str(exc), file=sys.stderr)
        raise SystemExit(1)
