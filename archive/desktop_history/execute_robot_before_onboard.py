#!/usr/bin/env python3
"""Start remote TurtleBot bringup and local Nav2; no movement without --target.

python3 execute_robot.py --dry-run
python3 execute_robot.py --check
python3 execute_robot.py
python3 execute_robot.py --reuse-bringup
python3 execute_robot.py --target workbench
"""

import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time


# This supervisor runs on the robot. EOF or a missed heartbeat stops only its
# own process group, including launch descendants, even if the laptop disappears.
REMOTE_SUPERVISOR = r'''
import os, select, signal, subprocess, sys, time
child = subprocess.Popen(
    ["ros2", "launch", "turtlebot3_bringup", "robot.launch.py",
     "use_sim_time:=false"], stdin=subprocess.DEVNULL, start_new_session=True)
stopping = False
def stop(signum, frame):
    global stopping
    stopping = True
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
last = time.monotonic()
try:
    while child.poll() is None and not stopping:
        readable, _, _ = select.select([sys.stdin], [], [], 1)
        if readable:
            data = os.read(sys.stdin.fileno(), 4096)
            if not data:
                break
            last = time.monotonic()
        if time.monotonic() - last > 15:
            print("Heartbeat lost: stopping owned bringup", flush=True)
            break
finally:
    for sig, grace in ((signal.SIGINT, 8), (signal.SIGTERM, 4),
                       (signal.SIGKILL, 1)):
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline:
            child.poll()
            try:
                os.killpg(child.pid, 0)
            except ProcessLookupError:
                break
            time.sleep(0.1)
        else:
            continue
        break
    child.wait()
sys.exit(0 if stopping else child.returncode or 0)
'''


def shell_command(argv, domain, model):
    # Ubuntu's .bashrc returns immediately in noninteractive shells. --norc
    # avoids implicitly sourcing it twice; -i makes its interactive guard pass.
    script = (
        'source "$HOME/.bashrc"\n'
        f'export ROS_DOMAIN_ID={shlex.quote(str(domain))}\n'
        f'export TURTLEBOT3_MODEL={shlex.quote(model)}\n'
        'exec ' + shlex.join([str(x) for x in argv])
    )
    return ['bash', '--norc', '-ic', script]


def probe(args):
    """ROS checks run in a sourced system-Python subprocess, not an arm venv."""
    import yaml
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    if args.probe == 'check':
        from ament_index_python.packages import get_package_share_directory
        for name in ('turtlebot3_navigation2', 'nav2_bringup', 'rviz2'):
            get_package_share_directory(name)
        path = args.map.expanduser().resolve(strict=True)
        config = yaml.safe_load(path.read_text())
        image = Path(config['image']).expanduser()
        if not image.is_absolute():
            image = path.parent / image
        if not image.is_file():
            raise RuntimeError(f'지도 이미지 없음: {image}')
        if args.params_file and not args.params_file.expanduser().is_file():
            raise RuntimeError(f'Nav2 파라미터 파일 없음: {args.params_file}')
        if args.target:
            get_package_share_directory('nav2_simple_commander')
            if not Path(__file__).with_name('go_waypoint.py').is_file():
                raise RuntimeError('같은 폴더에 go_waypoint.py가 필요합니다.')
            waypoint_file = args.waypoints.expanduser().resolve(strict=True)
            if waypoint_file != Path(__file__).resolve().with_name('waypoints.yaml'):
                raise RuntimeError('--target은 같은 폴더의 waypoints.yaml을 사용하세요.')
            points = yaml.safe_load(waypoint_file.read_text())
            import math
            point = points.get(args.target) if isinstance(points, dict) else None
            if not isinstance(point, dict) or point.get('frame_id') != 'map':
                raise RuntimeError('목적지가 없거나 frame_id가 map이 아님')
            for key in ('x', 'y', 'yaw'):
                value = point.get(key)
                if type(value) not in (float, int) or not math.isfinite(value):
                    raise RuntimeError(f'{args.target}.{key}에 실제 숫자 필요')
        print(json.dumps({'map': str(path), 'image': str(image),
                          'ROS_DOMAIN_ID': os.getenv('ROS_DOMAIN_ID'),
                          'model': os.getenv('TURTLEBOT3_MODEL')}, ensure_ascii=False))
        return 0

    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from tf2_ros import Buffer, TransformListener, TransformException
    from lifecycle_msgs.srv import GetState

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('execute_robot_probe_' + str(os.getpid()))
    seen = set()
    subscriptions = []
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    for topic, kind in (('/scan', LaserScan), ('/odom', Odometry)):
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, t=topic: seen.add(t), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(
        OccupancyGrid, '/map', lambda msg: seen.add('/map'),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    names = ('map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator')
    clients = {name: node.create_client(GetState, f'/{name}/get_state') for name in names}
    pending, active = {}, set()
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.probe == 'graph':
                if time.monotonic() - start >= 3:
                    print(json.dumps(sorted(set(node.get_node_names()))))
                    return 0
                continue
            if args.probe == 'bringup':
                if not {'/scan', '/odom'} <= seen:
                    continue
                try:
                    buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
                except TransformException:
                    continue
                print('Bringup 준비: /scan, /odom, odom -> base_link 수신')
                return 0
            for name, client in clients.items():
                if name in active:
                    continue
                future = pending.get(name)
                if future is not None:
                    if future.done():
                        result = future.result()
                        if result and result.current_state.id == 3:
                            active.add(name)
                        del pending[name]
                elif client.service_is_ready():
                    pending[name] = client.call_async(GetState.Request())
            if active == set(names) and '/map' in seen:
                print('Nav2 준비: 지도 수신 및 주요 lifecycle 노드 active')
                print('RViz에서 2D Pose Estimate로 초기 위치를 지정하세요.')
                return 0
        raise RuntimeError(f'{args.probe} 준비 시간 초과: topics={sorted(seen)}, '
                           f'active={sorted(active)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


class Processes:
    def __init__(self, directory):
        self.directory = directory
        self.entries = []

    def start(self, name, command, persistent=True, stdin=None):
        log_path = self.directory / (name + '.log')
        log = log_path.open('wb')
        try:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       stdin=stdin, start_new_session=True)
        except BaseException:
            log.close()
            raise
        self.entries.append((name, process, log, log_path, persistent))
        print(f'[{name}] PID={process.pid}, 로그={log_path}', flush=True)
        return process

    def check(self):
        for name, process, _, path, persistent in self.entries:
            if persistent and process.poll() is not None:
                raise RuntimeError(f'{name} 종료(code={process.returncode})\n'
                                   + path.read_text(errors='replace')[-4000:])

    def run(self, name, command, timeout):
        process = self.start(name, command, persistent=False, stdin=subprocess.DEVNULL)
        deadline = time.monotonic() + timeout
        while process.poll() is None:
            self.check()
            if time.monotonic() > deadline:
                raise RuntimeError(f'{name} 시간 초과: {timeout}초')
            time.sleep(0.2)
        path = self.directory / (name + '.log')
        output = path.read_text(errors='replace')
        if process.returncode:
            raise RuntimeError(f'{name} 실패(code={process.returncode})\n{output[-4000:]}')
        return output

    def close(self):
        # Stop local Nav2 before remote bringup. Never pkill unrelated ROS nodes.
        for _, process, log, _, _ in reversed(self.entries):
            for sig, grace in ((signal.SIGINT, 8), (signal.SIGTERM, 3),
                               (signal.SIGKILL, 1)):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                deadline = time.monotonic() + grace
                while time.monotonic() < deadline:
                    process.poll()
                    try:
                        os.killpg(process.pid, 0)
                    except ProcessLookupError:
                        break
                    time.sleep(0.1)
                else:
                    continue
                break
            if process.stdin:
                process.stdin.close()
            process.wait()
            log.close()


def parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    desktop = Path(__file__).resolve().parent
    p.add_argument('--host', default='turtlebot6@192.168.110.176')
    p.add_argument('--map', type=Path, default=desktop / 'clear_map.yaml')
    p.add_argument('--domain-id', type=int, default=36)
    p.add_argument('--model', default='waffle_pi')
    p.add_argument('--params-file', type=Path)
    p.add_argument('--timeout', type=float, default=90)
    p.add_argument('--reuse-bringup', action='store_true', help='기존 실물 bringup 사용')
    p.add_argument('--reuse-nav2', action='store_true', help='기존 Nav2 사용')
    p.add_argument('--local-node', action='append', default=[], metavar='COMMAND',
                   help='추가 노트북 노드 명령; 여러 번 지정 가능')
    p.add_argument('--target', help='준비 완료 후 이동할 waypoint (생략 시 이동 안 함)')
    p.add_argument('--waypoints', type=Path, default=desktop / 'waypoints.yaml')
    p.add_argument('--dry-run', action='store_true', help='실행 명령만 출력')
    p.add_argument('--check', action='store_true', help='노트북 파일·ROS 의존성만 확인')
    p.add_argument('--probe', choices=['check', 'graph', 'bringup', 'nav2'],
                   help=argparse.SUPPRESS)
    return p


def main():
    args = parser().parse_args()
    if args.probe:
        return probe(args)
    if args.host.startswith('-') or not 0 <= args.domain_id <= 232 or args.timeout < 5:
        raise ValueError('host, domain-id(0..232), timeout(5초 이상)을 확인하세요.')
    script = Path(__file__).resolve()
    local = lambda argv: shell_command(argv, args.domain_id, args.model)
    remote = shell_command(['python3', '-u', '-c', REMOTE_SUPERVISOR],
                           args.domain_id, args.model)
    navigation = ['ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
                  f'map:={args.map.expanduser().resolve()}', 'use_sim_time:=false']
    if args.params_file:
        navigation.append(f'params_file:={args.params_file.expanduser().resolve()}')
    extras = [shlex.split(command) for command in args.local_node]
    if any(not command for command in extras):
        raise ValueError('--local-node 명령이 비어 있습니다.')
    if args.dry_run:
        print('SSH:', args.host)
        print('양쪽 환경: source ~/.bashrc (interactive bash), '
              f'ROS_DOMAIN_ID={args.domain_id}, TURTLEBOT3_MODEL={args.model}')
        print('원격:', '기존 bringup 사용' if args.reuse_bringup else
              'ros2 launch turtlebot3_bringup robot.launch.py use_sim_time:=false')
        print('노트북:', '기존 Nav2 사용' if args.reuse_nav2 else shlex.join(navigation))
        for command in extras:
            print('추가 노드:', shlex.join(command))
        print('목적지:', args.target or '없음 (자동 이동 안 함)')
        return 0

    log_dir = Path(tempfile.mkdtemp(prefix='execute-robot-logs-'))
    manager = Processes(log_dir)
    heartbeat_stop = threading.Event()
    heartbeat_thread = None
    connection = None
    socket_dir = None

    def helper(mode):
        command = ['/usr/bin/python3', str(script), '--probe', mode,
                   '--map', str(args.map.expanduser().resolve()),
                   '--timeout', str(args.timeout)]
        if args.params_file:
            command += ['--params-file', str(args.params_file.expanduser())]
        if args.target:
            command += ['--target', args.target, '--waypoints', str(args.waypoints.expanduser())]
        return local(command)

    try:
        print(manager.run('local-check', helper('check'), 20), flush=True)
        if args.check:
            print('노트북 사전 확인 완료. SSH/bringup/Nav2는 실행하지 않았습니다.')
            return 0
        graph_output = manager.run('graph-check', helper('graph'), 20)
        names = json.loads(graph_output.strip().splitlines()[-1])
        bringup_nodes = {'turtlebot3_node', 'robot_state_publisher', 'lidar_node',
                         'diff_drive_controller'}
        nav_nodes = {'map_server', 'amcl', 'controller_server', 'planner_server', 'bt_navigator'}
        if bringup_nodes.intersection(names) and not args.reuse_bringup:
            raise RuntimeError('실행 중인 bringup 관련 노드가 있습니다. 기존 launch를 '
                               '종료하거나 --reuse-bringup을 지정하세요.')
        if nav_nodes.intersection(names) and not args.reuse_nav2:
            raise RuntimeError('실행 중인 Nav2가 있습니다. 기존 launch를 종료하거나 '
                               '--reuse-nav2를 지정하세요.')

        if not args.reuse_bringup:
            socket_dir = tempfile.TemporaryDirectory(prefix='robot-ssh-')
            socket = str(Path(socket_dir.name) / 'control')
            connection = ['ssh', '-S', socket, '-o', 'ConnectTimeout=10',
                          '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=3']
            print(f'SSH 인증: {args.host} (비밀번호는 SSH 프롬프트에 입력)', flush=True)
            # One foreground authentication, then reuse its private control socket.
            subprocess.run(connection + ['-M', '-o', 'ControlPersist=60', '-fnN',
                                          args.host], check=True)
            process = manager.start('remote-bringup', connection + [
                '-o', 'BatchMode=yes', '-T', args.host, shlex.join(remote)],
                stdin=subprocess.PIPE)

            def heartbeat():
                while not heartbeat_stop.is_set():
                    try:
                        process.stdin.write(b'heartbeat\n')
                        process.stdin.flush()
                    except (BrokenPipeError, OSError, ValueError):
                        return
                    heartbeat_stop.wait(2)

            heartbeat_thread = threading.Thread(target=heartbeat, daemon=True)
            heartbeat_thread.start()

        print(manager.run('bringup-check', helper('bringup'), args.timeout + 10), flush=True)
        if not args.reuse_nav2:
            manager.start('nav2', local(navigation), stdin=subprocess.DEVNULL)
        for index, command in enumerate(extras):
            manager.start(f'local-node-{index + 1}', local(command), stdin=subprocess.DEVNULL)
        print(manager.run('nav2-check', helper('nav2'), args.timeout + 10), flush=True)

        if args.target:
            waypoint_script = script.with_name('go_waypoint.py')
            if not waypoint_script.is_file():
                raise RuntimeError(f'이동 스크립트 없음: {waypoint_script}')
            # go_waypoint.py resolves waypoints.yaml beside itself. Refuse to
            # silently use a different file than the one preflight validated.
            if args.waypoints.expanduser().resolve() != waypoint_script.with_name('waypoints.yaml'):
                raise RuntimeError('--target은 같은 폴더의 waypoints.yaml을 사용하세요.')
            print('초기 위치 지정 후 이동합니다. RViz의 2D Pose Estimate를 사용하세요.')
            output = manager.run('waypoint', local([
                '/usr/bin/python3', str(waypoint_script), args.target]), args.timeout + 180)
            print(output)
        print(f'시스템 실행 중. 종료: Ctrl+C / 로그: {log_dir}', flush=True)
        while True:
            manager.check()
            time.sleep(0.5)
    except KeyboardInterrupt:
        print('\n종료 요청: 이 프로그램이 시작한 노드를 정리합니다.', flush=True)
        return 130
    finally:
        # Ending the heartbeat makes the remote supervisor stop even if the
        # SSH client cannot deliver an interactive terminal signal.
        heartbeat_stop.set()
        if heartbeat_thread:
            heartbeat_thread.join(timeout=3)
        manager.close()
        if connection:
            try:
                subprocess.run(connection + ['-O', 'exit', args.host], timeout=5,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except (subprocess.TimeoutExpired, OSError):
                pass
        if socket_dir:
            socket_dir.cleanup()


if __name__ == '__main__':
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f'오류: {exc}', file=sys.stderr)
        raise SystemExit(1)
