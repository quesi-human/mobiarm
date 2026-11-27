#!/usr/bin/env python3
"""On-robot Nav2 stack supervisor and one-shot pick/return FSM.

Uploaded by execute_robot.py. ROS uses system Python; arm_cli uses arm-venv.
"""
import argparse
from collections import deque
import fcntl
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time

PREFIX = '@@ROBOT_EVENT@@'
COLORS = {'red', 'orange', 'yellow', 'green', 'blue', 'purple'}


class LocalizationNotReady(RuntimeError):
    """Recoverable localization failure; keep the stack available."""


def event(kind, **fields):
    print(PREFIX + json.dumps({'event': kind, **fields}, ensure_ascii=False), flush=True)


def last_json(output):
    for line in reversed(output.splitlines()):
        try:
            value = json.loads(line)
            if isinstance(value, dict):
                return value
        except ValueError:
            pass
    raise RuntimeError('명령의 JSON 결과가 없습니다: ' + output[-500:])


def validate_point(point):
    if not isinstance(point, dict) or point.get('frame_id') != 'map':
        raise ValueError('waypoint는 frame_id: map이어야 합니다.')
    for key in ('x', 'y', 'yaw'):
        value = point.get(key)
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f'waypoint {key}에 유한한 숫자가 필요합니다.')
    return {key: point[key] for key in ('frame_id', 'x', 'y', 'yaw')}


def validate_request(request):
    if request.get('navigate_only') is True:
        validate_point(request['destination'])
        return request
    if request.get('color') not in COLORS:
        raise ValueError('지원하지 않는 블록 색상입니다.')
    validate_point(request['destination'])
    if request.get('start') != 'current':
        validate_point(request['start'])
    return request


def initial_pose_fields(point):
    point = validate_point(point)
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = .15 ** 2
    covariance[35] = math.radians(15) ** 2
    return math.sin(point['yaw'] / 2), math.cos(point['yaw'] / 2), covariance


def laser_record(msg, now_seconds):
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
    duration = max(0, len(msg.ranges)-1) * msg.time_increment
    return {'frame_id': msg.header.frame_id, 'stamp': stamp,
            'stamp_age_s': now_seconds-stamp, 'last_ray_stamp': stamp+duration,
            'last_ray_age_s': now_seconds-stamp-duration, 'scan_time': msg.scan_time,
            'time_increment': msg.time_increment, 'angle_min': msg.angle_min,
            'angle_max': msg.angle_max, 'angle_increment': msg.angle_increment,
            'range_min': msg.range_min, 'range_max': msg.range_max,
            'ranges': [float(r) if math.isfinite(r) else None for r in msg.ranges]}


def path_record(msg):
    return {'frame_id': msg.header.frame_id,
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec/1e9,
            'poses': [{'x': p.pose.position.x, 'y': p.pose.position.y,
                       'yaw': math.atan2(2*(p.pose.orientation.w*p.pose.orientation.z+
                           p.pose.orientation.x*p.pose.orientation.y),
                           1-2*(p.pose.orientation.y**2+p.pose.orientation.z**2))}
                      for p in msg.poses]}


class LifecyclePoller:
    """Retry lost GetState responses; accept only recent active observations."""
    def __init__(self, clients, request_factory, request_timeout=3.0, freshness=6.0):
        self.clients, self.request_factory = clients, request_factory
        self.request_timeout, self.freshness = request_timeout, freshness
        self.pending, self.observed, self.next_request, self.retries = {}, {}, {}, {}

    def discard(self, name, future):
        self.clients[name].remove_pending_request(future)
        future.cancel()

    def tick(self, now):
        for name, client in self.clients.items():
            pending = self.pending.get(name)
            if pending is not None:
                future, deadline = pending
                if future.done():
                    try:
                        response = future.result()
                        if response is None:
                            self.observed.pop(name, None)
                        else:
                            state = response.current_state
                            self.observed[name] = (state.id, getattr(state, 'label', str(state.id)), now)
                    except Exception:
                        self.observed.pop(name, None)
                    del self.pending[name]
                    self.next_request[name] = now + 1.0
                elif now >= deadline:
                    self.discard(name, future)
                    del self.pending[name]
                    self.observed.pop(name, None)
                    self.retries[name] = self.retries.get(name, 0) + 1
                    self.next_request[name] = now + .2
            if (name not in self.pending and now >= self.next_request.get(name, 0)
                    and client.service_is_ready()):
                try:
                    future = client.call_async(self.request_factory())
                    self.pending[name] = (future, now + self.request_timeout)
                except Exception:
                    self.observed.pop(name, None)
                    self.next_request[name] = now + 1.0

    def active(self, now):
        return {name for name, (state, label, received) in self.observed.items()
                if state == 3 and now-received <= self.freshness}

    def summary(self, now):
        return {name: {'state': self.observed[name][1] if name in self.observed else 'unknown',
                       'fresh': name in self.observed and now-self.observed[name][2] <= self.freshness,
                       'pending': name in self.pending, 'retries': self.retries.get(name, 0)}
                for name in self.clients}

    def close(self):
        for name, (future, deadline) in self.pending.items():
            self.discard(name, future)
        self.pending.clear()


def initialize_pose(point, timeout):
    """Seed AMCL from an explicitly configured physical starting pose; no drive."""
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid
    from lifecycle_msgs.srv import GetState
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from tf2_ros import Buffer, TransformListener, TransformException
    z, w, covariance = initial_pose_fields(point)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('mission_initial_pose_' + str(os.getpid()))
    publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    map_seen = []
    subscription = node.create_subscription(OccupancyGrid, '/map', lambda msg: map_seen.append(True),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client = node.create_client(GetState, '/amcl/get_state')
    poller = LifecyclePoller({'amcl': client}, GetState.Request)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    first_sent, last_sent = None, 0
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            poller.tick(now)
            if not ('amcl' in poller.active(now) and map_seen and publisher.get_subscription_count()):
                continue
            if first_sent is None:
                message = PoseWithCovarianceStamped()
                message.header.frame_id = 'map'
                message.header.stamp = node.get_clock().now().to_msg()
                message.pose.pose.position.x, message.pose.pose.position.y = float(point['x']), float(point['y'])
                message.pose.pose.orientation.z, message.pose.pose.orientation.w = z, w
                message.pose.covariance = covariance
                publisher.publish(message)
                if first_sent is None:
                    first_sent = now
                    event('initial_pose_sent', pose=point)
                last_sent = now
            if first_sent is None or now - first_sent < 1:
                continue
            try:
                tf = buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            except TransformException:
                continue
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9
            age = node.get_clock().now().nanoseconds / 1e9 - stamp
            t, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            distance = math.hypot(t.x - point['x'], t.y - point['y'])
            angle = abs(math.atan2(math.sin(yaw-point['yaw']), math.cos(yaw-point['yaw'])))
            if -2 < age < 2 and distance < .3 and angle < .35:
                event('initial_pose_ready', pose=point)
                return
        raise RuntimeError('자동 초기 위치 설정 시간 초과: 지도·AMCL·TF 확인')
    finally:
        poller.close()
        node.destroy_node()
        rclpy.shutdown()


def stop_group(process, graces=(8, 3, 1)):
    for sig, grace in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), graces):
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
            time.sleep(.1)
        else:
            continue
        break
    process.wait()


class Control:
    def __init__(self):
        self.last = time.monotonic()
        self.lost = threading.Event()
        self.messages = queue.Queue()
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        for line in sys.stdin:
            try:
                message = json.loads(line)
                if message.get('type') == 'heartbeat':
                    self.last = time.monotonic()
                elif message.get('type') == 'stop':
                    self.lost.set()
                    return
                elif message.get('type') == 'mission':
                    self.messages.put(message['request'])
            except (ValueError, KeyError, AttributeError):
                self.lost.set()
                return
        self.lost.set()

    def check(self):
        if self.lost.is_set() or time.monotonic() - self.last > 15:
            raise RuntimeError('노트북 제어 연결/heartbeat 종료')


class Processes:
    def __init__(self, directory, control=None):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.entries = []
        self.control = control

    def start(self, name, argv, persistent=True):
        path = self.directory / (name + '.log')
        log = path.open('wb')
        try:
            process = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
        except BaseException:
            log.close()
            raise
        self.entries.append((name, process, log, path, persistent))
        return process, path

    def check(self):
        if self.control:
            self.control.check()
        for name, process, _, path, persistent in self.entries:
            if persistent and process.poll() is not None:
                raise RuntimeError(f'{name} 종료({process.returncode}): '
                                   + path.read_text(errors='replace')[-3000:])

    def run(self, name, argv, timeout):
        process, path = self.start(name, argv, persistent=False)
        deadline = time.monotonic() + timeout
        offset, partial = 0, ''

        def forward():
            nonlocal offset, partial
            with path.open(errors='replace') as stream:
                stream.seek(offset)
                chunk = stream.read()
                offset = stream.tell()
            lines = (partial + chunk).split('\n')
            partial = lines.pop()
            for line in lines:
                if line.startswith(PREFIX):
                    print(line, flush=True)

        while process.poll() is None:
            forward()
            self.check()
            if time.monotonic() > deadline:
                raise RuntimeError(f'{name} 시간 초과')
            time.sleep(.1)
        forward()
        output = path.read_text(errors='replace')
        for entry in list(self.entries):
            if entry[1] is process:
                entry[2].close()
                self.entries.remove(entry)
        if process.returncode == 75:
            raise LocalizationNotReady('작업을 보류했습니다. 로그: '+str(path))
        if process.returncode:
            raise RuntimeError(f'{name} 실패({process.returncode}): {output[-3000:]}')
        return output

    def close(self):
        for _, process, log, _, _ in reversed(self.entries):
            stop_group(process)
            log.close()


def dependencies(config):
    import yaml
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from nav2_msgs.action import DriveOnHeading, BackUp
    for name in ('nav2_bringup', 'nav2_simple_commander', 'turtlebot3_bringup', 'nav2_behaviors'):
        get_package_share_directory(name)
    nav_parameters = yaml.safe_load(Path(config['params_file']).read_text())
    controller = nav_parameters['controller_server']['ros__parameters']
    for name in controller.get('controller_plugins', []):
        if controller.get(name, {}).get('plugin', '').startswith('nav2_regulated_pure_pursuit_controller::'):
            get_package_share_directory('nav2_regulated_pure_pursuit_controller')
    for key in ('map', 'params_file', 'arm_python', 'arm_cli', 'navigation_launch'):
        if not Path(config[key]).is_file():
            raise RuntimeError(f'{key} 파일 없음: {config[key]}')
    for key in ('home_behavior_tree','normal_behavior_tree'):
        if config.get(key) and not Path(config[key]).is_file():
            raise RuntimeError(f'{key} 파일 없음: {config[key]}')
    if config.get('scan_filter') and not Path(config['scan_filter']).is_file():
        raise RuntimeError('scan_self_filter.py 파일 없음')
    path = Path(config['map'])
    image = Path(yaml.safe_load(path.read_text())['image'])
    if not image.is_absolute():
        image = path.parent / image
    if not image.is_file():
        raise RuntimeError(f'지도 이미지 없음: {image}')
    root = Path(config['arm_cli']).parent
    for name in ('park_pose.json', 'carry_pose.json', 'look_pose.json', 'wrist_camera.json'):
        if not (root / 'calibration' / name).is_file():
            raise RuntimeError(f'팔 보정/자세 파일 없음: {name}')
    if config.get('place_on_return') and not (root / 'calibration/place_pose.json').is_file():
        raise RuntimeError('place_pose.json이 필요합니다.')
    completed = subprocess.run([config['arm_python'], '-c',
        'import scipy, numpy; import robot.tasks; from vision.camera import COLOR_RANGES; '
        'print(sorted(COLOR_RANGES))'], cwd=root, capture_output=True, text=True, timeout=40)
    if completed.returncode:
        raise RuntimeError('팔 가상환경 의존성: ' + completed.stderr[-2000:])
    event('checked', model=os.getenv('TURTLEBOT3_MODEL'),
          domain=os.getenv('ROS_DOMAIN_ID'), arm_colors=completed.stdout.strip())


def ros_probe(kind, timeout):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry, OccupancyGrid
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from tf2_ros import Buffer, TransformListener, TransformException
    from lifecycle_msgs.srv import GetState
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('mission_probe_' + str(os.getpid()))
    seen, active, subscriptions = set(), set(), []
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    for topic, msgtype in (('/scan', LaserScan), ('/scan_filtered', LaserScan), ('/odom', Odometry)):
        subscriptions.append(node.create_subscription(msgtype, topic,
            lambda msg, t=topic: seen.add(t), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(OccupancyGrid, '/map',
        lambda msg: seen.add('/map'), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    names = ('map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator', 'behavior_server')
    clients = {name: node.create_client(GetState, '/' + name + '/get_state') for name in names}
    poller = LifecyclePoller(clients, GetState.Request)
    start = time.monotonic()
    next_report = start + 5
    try:
        while time.monotonic() - start < timeout:
            rclpy.spin_once(node, timeout_sec=.1)
            if kind == 'graph':
                if time.monotonic() - start > 3:
                    print(json.dumps(sorted(set(node.get_node_names()))), flush=True)
                    return
                continue
            if kind == 'bringup':
                if not {'/scan', '/odom'} <= seen:
                    continue
                try:
                    buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
                except TransformException:
                    continue
                event('bringup_ready')
                return
            now = time.monotonic()
            poller.tick(now)
            active = poller.active(now)
            if active == set(names) and {'/map', '/scan_filtered'} <= seen:
                return
            if now >= next_report:
                event('readiness_status', states=poller.summary(now))
                next_report = now + 5
        raise RuntimeError(f'{kind} 준비 시간 초과: topics={sorted(seen)}, active={sorted(active)}, '
                           f'states={poller.summary(time.monotonic())}')
    finally:
        poller.close()
        node.destroy_node()
        rclpy.shutdown()


def run_stack(config, config_path):
    lock = (Path(config_path).parent.parent / 'stack.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    control = Control()
    processes = Processes(config['logs'], control)
    script = str(Path(__file__).resolve())
    helper = lambda kind: ['/usr/bin/python3', '-u', script, '--mode', 'probe',
                          '--kind', kind, '--timeout', str(config['timeout']), '--config', str(config_path)]
    try:
        dependencies(config)
        if config.get('scan_filter') and config['reuse_nav2']:
            raise RuntimeError('자기 몸체 필터 적용을 위해 기존 Nav2를 종료하고 --reuse-nav2 없이 실행하세요.')
        names = json.loads(processes.run('graph', helper('graph'), 15).strip().splitlines()[-1])
        if config.get('scan_filter') and 'scan_self_filter' in names:
            raise RuntimeError('기존 scan_self_filter를 종료하세요. 필터 중복 기동은 허용하지 않습니다.')
        bringup = {'turtlebot3_node', 'robot_state_publisher', 'lidar_node', 'diff_drive_controller'}
        nav2 = {'map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator'}
        if bringup.intersection(names) and not config['reuse_bringup']:
            raise RuntimeError('기존 bringup을 종료하거나 --reuse-bringup 사용')
        if nav2.intersection(names) and not config['reuse_nav2']:
            raise RuntimeError('기존 Nav2(노트북 포함)를 종료하세요. 파이 Nav2만 재사용하려면 --reuse-nav2 사용')
        if config['reuse_nav2']:
            # ROS discovery cannot tell which host owns a node. Verify a local
            # Nav2 process instead of silently reusing an old laptop Nav2 stack.
            command = subprocess.check_output(['ps', '-eo', 'args'], text=True)
            if not any(marker in command for marker in
                       ('nav2_container', 'nav2_map_server/map_server', 'nav2_amcl/amcl')):
                raise RuntimeError('--reuse-nav2: 파이 내부 Nav2 프로세스를 찾지 못했습니다.')
        if not config['reuse_bringup']:
            processes.start('bringup', ['ros2', 'launch', 'turtlebot3_bringup',
                                      'robot.launch.py', 'use_sim_time:=false'])
        processes.run('bringup-ready', helper('bringup'), config['timeout'] + 10)
        if config.get('scan_filter'):
            processes.start('scan-filter', ['/usr/bin/python3', '-u', config['scan_filter']])
        if not config['reuse_nav2']:
            processes.start('nav2', ['ros2', 'launch', config['navigation_launch'],
                'map:=' + config['map'], 'params_file:=' + config['params_file'],
                'use_sim_time:=False', 'autostart:=True', 'use_composition:=True'])
        if config.get('initial_pose') is not None:
            processes.run('initial-pose', helper('initial'), config['timeout'] + 10)
        processes.run('nav2-ready', helper('nav2'), config['timeout'] + 10)
        event('ready', logs=config['logs'])
        sequence=0
        event('state',state='IDLE')
        while True:
            processes.check()
            try:
                request = control.messages.get(timeout=.2)
            except queue.Empty:
                continue
            validate_request(request)
            sequence+=1
            request_file = Path(config_path).with_name(f'mission-{sequence:04d}.json')
            request_file.write_text(json.dumps(request, ensure_ascii=False))
            mission_config=Path(config_path).with_name(f'mission-config-{sequence:04d}.json')
            mission_config.write_text(json.dumps(dict(config,logs=str(Path(config['logs'])/f'mission-{sequence:04d}'))))
            outcome='completed'
            try:
                processes.run(f'mission-{sequence:04d}', ['/usr/bin/python3', '-u', script, '--mode', 'mission',
                    '--config', str(mission_config), '--request', str(request_file)],
                    config['nav_timeout'] * 3 + config['arm_timeout'] *
                    (4 if config.get('place_on_return') else 3) + config['timeout'] + 90)
            except LocalizationNotReady as exc:
                outcome='deferred'
                event('mission_paused',message=str(exc))
            event('mission_done',mission_id=sequence,outcome=outcome)
            if config.get('once',False):
                return
            event('state',state='IDLE')
    except BaseException as exc:
        event('error', message=str(exc) or type(exc).__name__)
        raise
    finally:
        processes.close()
        lock.close()
        event('stack_stopped')


def verify_short_motion(origin, actual, requested):
    dx, dy = actual['x']-origin['x'], actual['y']-origin['y']
    c, s = math.cos(origin['yaw']), math.sin(origin['yaw'])
    along, lateral = c*dx+s*dy, -s*dx+c*dy
    yaw_error = abs(math.atan2(math.sin(actual['yaw']-origin['yaw']),
                               math.cos(actual['yaw']-origin['yaw'])))
    if (not all(math.isfinite(v) for v in (along, lateral, yaw_error)) or
            abs(along-requested) > .03 or abs(lateral) > .03 or yaw_error > .12):
        raise RuntimeError(f'정밀 접근 거리/방향 오차 초과: 목표 {requested:.3f}m, '
                           f'진행 {along:.3f}m, 옆밀림 {lateral:.3f}m, '
                           f'회전 {math.degrees(yaw_error):.1f}도')
    return along


def check_arrival(actual, goal, check_position=True, position_limit=.15, yaw_limit=.15):
    distance = math.hypot(actual['x']-goal['x'], actual['y']-goal['y'])
    delta = actual['yaw']-goal['yaw']
    angle = abs(math.atan2(math.sin(delta), math.cos(delta)))
    if not math.isfinite(distance) or not math.isfinite(angle):
        raise RuntimeError('도착 자세에 유효하지 않은 값이 있습니다.')
    if (check_position and distance > position_limit) or angle > yaw_limit:
        raise RuntimeError(f'도착 자세 오차 초과: {distance:.3f}m, {math.degrees(angle):.1f}도')
    return distance, angle


def pad_polygon(points, padding):
    # Same per-coordinate padding convention as Nav2 padFootprint.
    return [[x + (padding if x > 0 else -padding if x < 0 else 0),
             y + (padding if y > 0 else -padding if y < 0 else 0)] for x,y in points]


def polygons_match(actual, expected, tolerance=.008):
    return len(actual) == len(expected) and all(
        any(math.hypot(x-a,y-b) < tolerance for a,b in actual) for x,y in expected)


def install_park_guard(tasks_class, near_park, stop_error):
    """Reject a torque-off false 'parked' result without enabling any motors."""
    original = tasks_class.park
    def checked_park(tasks):
        if not tasks._torque_is_on():
            if tasks.park_q is None or not near_park(tasks.arm.read_deg(), tasks.park_q):
                raise stop_error('이동 전 실제 접힘 자세 확인 실패: 토크 OFF 상태에서 저장된 '
                                 'park 자세와 다릅니다. 팔을 접힌 자세로 놓거나 '
                                 'steps/m5_teach_park.py로 정상 접힘 자세를 저장하세요.')
        return original(tasks)
    tasks_class.park = checked_park


def reset_arm_tasks(tasks, near_park):
    """Return to recorded park, release the object, then reset all torque."""
    if tasks.park_q is None:
        raise RuntimeError('실패 복구용 park 자세가 없습니다.')
    if tasks._torque_is_on():
        tasks.park()
    if not near_park(tasks.arm.read_deg(), tasks.park_q):
        raise RuntimeError('실패 복구 park 자세 확인 실패: 강제 토크 OFF를 중단합니다.')
    tasks.arm.gripper_torque_on()
    tasks.arm.gripper_open(45)
    time.sleep(.5)
    tasks.holding=False
    motors=list(tasks.arm.bus.motors)
    tasks.arm.bus.disable_torque(motors,num_retry=2)
    if any(tasks.arm._read('Torque_Enable',motors).values()):
        raise RuntimeError('실패 복구 토크 OFF 확인 실패')
    tasks.pose='park'
    return {'ok':True,'cmd':'recover','result':'initialized','holding':False,
            'pose':'park','torque_off':True}


def run_arm_reset():
    import contextlib
    import arm_cli
    from robot import devices as DEV, park as P
    from robot.tasks import ArmTasks
    arm_cli.LOCK_FILE.parent.mkdir(parents=True,exist_ok=True)
    with arm_cli.LOCK_FILE.open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        tasks=None
        try:
            with contextlib.redirect_stdout(sys.stderr):
                tasks=ArmTasks(DEV.ARM_PORT,DEV.WRIST_CAM)
                saved=arm_cli._load_state()
                tasks.holding=bool(saved.get('holding'))
                tasks.pose=saved.get('pose','unknown')
                result=reset_arm_tasks(tasks,P.near_park)
                arm_cli._save_state(tasks)
            return result
        finally:
            if tasks is not None:
                # Torque was explicitly verified OFF on success. On a failed
                # park, detach rather than dropping an extended arm.
                with contextlib.redirect_stdout(sys.stderr):
                    tasks.detach()


class Mission:
    def __init__(self, config):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from nav2_simple_commander.robot_navigator import BasicNavigator
        from nav_msgs.msg import Odometry
        from nav_msgs.msg import OccupancyGrid
        from nav_msgs.msg import Path as NavPath
        from sensor_msgs.msg import LaserScan
        from geometry_msgs.msg import Twist
        from geometry_msgs.msg import Polygon, PolygonStamped
        from rclpy.action import ActionClient
        from nav2_msgs.action import DriveOnHeading
        from rclpy.qos import qos_profile_sensor_data
        from tf2_ros import Buffer, TransformListener
        self.config = config
        self.rclpy = rclpy
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.nav = BasicNavigator()
        self.drive_client = ActionClient(self.nav, DriveOnHeading, 'drive_on_heading')
        self.pending_short_goal = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.nav)
        self.footprint_observation = None
        self.original_local_footprint = None
        self.footprint_publisher = self.nav.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.footprint_subscription = self.nav.create_subscription(PolygonStamped,
            '/local_costmap/published_footprint', self.on_footprint, qos_profile_sensor_data)
        self.odom = None
        self.subscription = self.nav.create_subscription(Odometry, '/odom', self.on_odom,
                                                         qos_profile_sensor_data)
        self.velocity = self.nav.create_publisher(Twist, '/cmd_vel', 10)
        self.commands, self.costmaps, self.plans = {}, {}, {}
        self.current_goal = None
        self.motion_history = deque(maxlen=30)
        self.scan, self.scan_history = None, deque(maxlen=30)
        self.telemetry_subscriptions = []
        self.telemetry_subscriptions.append(self.nav.create_subscription(
            LaserScan, '/scan_filtered', self.on_scan, qos_profile_sensor_data))
        for topic in ('/plan', '/local_plan', '/transformed_global_plan',
                      '/lookahead_arc', '/received_global_plan'):
            self.telemetry_subscriptions.append(self.nav.create_subscription(
                NavPath, topic, lambda msg, t=topic: self.on_plan(t, msg), qos_profile_sensor_data))
        for topic in ('/cmd_vel_nav', '/cmd_vel'):
            self.telemetry_subscriptions.append(self.nav.create_subscription(
                Twist, topic, lambda msg, t=topic: self.on_command(t, msg), qos_profile_sensor_data))
        for topic in ('/local_costmap/costmap', '/global_costmap/costmap'):
            self.telemetry_subscriptions.append(self.nav.create_subscription(
                OccupancyGrid, topic, lambda msg, t=topic: self.on_costmap(t, msg), qos_profile_sensor_data))
        self.moving = False
        self.arm_process = None
        self.log_root = Path(config['logs'])
        self.log_root.mkdir(parents=True,exist_ok=True)

    def on_odom(self, msg):
        self.odom = (msg, time.monotonic())

    def on_footprint(self, msg):
        self.footprint_observation = (msg, time.monotonic())

    def footprint_in_base(self):
        if self.footprint_observation is None:
            return None
        msg, received = self.footprint_observation
        if time.monotonic()-received > 1:
            return None
        tf = self.transform_record('base_link', msg.header.frame_id)
        if not tf['ok']:
            return None
        c, s = math.cos(tf['yaw']), math.sin(tf['yaw'])
        x, y = tf['translation'][:2]
        return [[x+c*p.x-s*p.y, y+s*p.x+c*p.y] for p in msg.polygon.points]

    def set_pick_footprint(self, enabled):
        """Use physical body geometry for straight docking, then restore travel shape."""
        import yaml
        from geometry_msgs.msg import Polygon, Point32
        self.stationary()
        params = yaml.safe_load(Path(self.config['params_file']).read_text())
        padding = float(params['local_costmap']['local_costmap']['ros__parameters']
                        .get('footprint_padding', .01))
        if enabled:
            deadline = time.monotonic()+5
            original = None
            while original is None and time.monotonic() < deadline:
                self.rclpy.spin_once(self.nav, timeout_sec=.1)
                original = self.footprint_in_base()
            if original is None or len(original) < 3:
                raise RuntimeError('현재 로컬 footprint 확인 실패')
            # Save the observed footprint, including custom user travel geometry.
            self.original_local_footprint = pad_polygon(original, -padding)
            points = json.loads(params.get('mission_control', {}).get('ros__parameters', {})
                .get('pick_footprint', '[[0.09,0.16],[0.09,-0.16],[-0.21,-0.16],[-0.21,0.16]]'))
        else:
            if self.original_local_footprint is None:
                return
            points = self.original_local_footprint
        if len(points) < 3 or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError('집기 footprint 좌표 오류')
        polygon = Polygon()
        polygon.points = [Point32(x=float(x), y=float(y), z=0.) for x,y in points]
        expected = pad_polygon(points, padding)
        started = time.monotonic()
        deadline, next_publish = started+5, 0
        # Wait for footprint acknowledgement AND a later costmap update. This
        # avoids starting the Nav2 action with the previous cached circle.
        while time.monotonic() < deadline:
            if time.monotonic() >= next_publish:
                self.footprint_publisher.publish(polygon)
                next_publish = time.monotonic()+.2
            self.rclpy.spin_once(self.nav, timeout_sec=.1)
            actual = self.footprint_in_base()
            costmap = self.costmaps.get('/local_costmap/costmap')
            if (actual is not None and polygons_match(actual, expected) and costmap and
                    costmap[1] > started+.5 and time.monotonic()-costmap[1] < 1):
                event('footprint_changed', mode='pick' if enabled else 'travel', points=points)
                if not enabled:
                    self.original_local_footprint = None
                return
        raise RuntimeError('집기 footprint 적용/복원 확인 시간 초과')

    def on_command(self, topic, msg):
        self.commands[topic] = (msg, time.monotonic())

    def on_costmap(self, topic, msg):
        self.costmaps[topic] = (msg, time.monotonic())

    def on_scan(self, msg):
        self.scan = msg

    def on_plan(self, topic, msg):
        self.plans[topic] = path_record(msg)

    def transform_record(self, target, source, stamp=None):
        from tf2_ros import TransformException
        when = self.rclpy.time.Time() if stamp is None else self.rclpy.time.Time.from_msg(stamp)
        try:
            value = self.buffer.lookup_transform(target, source, when)
        except TransformException as exc:
            return {'ok': False, 'error': str(exc)}
        t, q = value.transform.translation, value.transform.rotation
        return {'ok': True, 'stamp': value.header.stamp.sec + value.header.stamp.nanosec/1e9,
                'translation': [t.x,t.y,t.z], 'rotation_xyzw': [q.x,q.y,q.z,q.w],
                'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}

    def current_scan_record(self):
        if self.scan is None:
            return None
        record = laser_record(self.scan, self.nav.get_clock().now().nanoseconds/1e9)
        for target in ('map', 'odom'):
            record['tf_to_' + target] = self.transform_record(
                target, self.scan.header.frame_id, self.scan.header.stamp)
        return record

    def navigation_sample(self):
        now = time.monotonic()
        sample = {'pose': self.pose(), 'commands': {}, 'odometry': None}
        for topic, (msg, received) in self.commands.items():
            sample['commands'][topic] = {'vx': msg.linear.x, 'wz': msg.angular.z,
                                         'age_s': round(now-received, 3)}
        if self.odom:
            msg, received = self.odom
            p, v = msg.pose.pose.position, msg.twist.twist
            sample['odometry'] = {'x': p.x, 'y': p.y, 'vx': v.linear.x,
                                  'wz': v.angular.z, 'age_s': round(now-received, 3)}
            q = getattr(msg.pose.pose, 'orientation', None)
            if q is not None:
                sample['odometry']['yaw'] = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if hasattr(self, 'buffer'):
            sample['tf_chain'] = {target+'->'+source: self.transform_record(target,source)
                for target,source in (('map','odom'),('odom','base_link'),('base_link','base_scan'))}
            scan = self.current_scan_record()
            sample['scan'] = None if scan is None else {k:v for k,v in scan.items() if k!='ranges'}
        return sample

    def navigation_snapshot(self, phase, goal):
        sample = self.navigation_sample()
        maps = {}
        for topic, (msg, received) in self.costmaps.items():
            p, q = msg.info.origin.position, msg.info.origin.orientation
            maps[topic] = {'frame_id': msg.header.frame_id,
                'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                'age_s': time.monotonic()-received, 'width': msg.info.width,
                'height': msg.info.height, 'resolution': msg.info.resolution,
                'origin': {'x': p.x, 'y': p.y, 'z': p.z,
                           'orientation': [q.x, q.y, q.z, q.w]}, 'data': list(msg.data)}
        path = self.log_root / ('navigation_' + phase + '.json')
        self.log_root.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({'goal': goal, 'latest': sample,
            'history': list(self.motion_history), 'costmaps': maps,
            'latest_scan': self.current_scan_record() if hasattr(self, 'scan') else None,
            'laser_scans': list(getattr(self, 'scan_history', [])),
            'plans': getattr(self, 'plans', {})}, ensure_ascii=False))
        event('navigation_snapshot', phase=phase, path=str(path))

    def pose(self):
        from tf2_ros import TransformException
        try:
            tf = self.buffer.lookup_transform('map', 'base_link', self.rclpy.time.Time())
        except TransformException:
            return None
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9
        age = self.nav.get_clock().now().nanoseconds / 1e9 - stamp
        if not -2 < age < 2:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return {'frame_id': 'map', 'x': t.x, 'y': t.y,
                'yaw': math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))}

    def localize(self):
        message = ('자동 초기화 후 map TF를 확인합니다.' if self.config.get('initial_pose') else
                   'RViz 2D Pose Estimate로 초기 위치를 지정하세요.')
        event('state', state='LOCALIZING', message=message)
        deadline = time.monotonic() + self.config['timeout']
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.nav, timeout_sec=.1)
            pose = self.pose()
            if pose:
                return pose
        raise RuntimeError('map -> base_link 없음/오래됨. 초기 위치·AMCL·TF·시간 확인')

    def stop_base(self):
        from geometry_msgs.msg import Twist
        for _ in range(5):
            self.velocity.publish(Twist())
            self.rclpy.spin_once(self.nav, timeout_sec=.05)

    def stationary(self):
        self.stop_base()
        deadline, stable = time.monotonic() + 10, None
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.nav, timeout_sec=.1)
            now = time.monotonic()
            valid = False
            if self.odom:
                msg, received = self.odom
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                age = self.nav.get_clock().now().nanoseconds / 1e9 - stamp
                v = msg.twist.twist
                valid = (now - received < 1 and -.5 < age < 1.5 and
                         math.hypot(v.linear.x, v.linear.y) < .01 and abs(v.angular.z) < .03)
            stable = (stable if stable is not None else now) if valid else None
            if stable is not None and now - stable >= 1:
                return
        raise RuntimeError('베이스 정지 확인 실패: /odom 속도/시각 확인')

    def navigate(self, point, state):
        from geometry_msgs.msg import PoseStamped
        from nav2_simple_commander.robot_navigator import TaskResult
        event('state', state=state, goal=point)
        self.current_goal = point
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.nav.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = float(point['x']), float(point['y'])
        goal.pose.orientation.z = math.sin(point['yaw'] / 2)
        goal.pose.orientation.w = math.cos(point['yaw'] / 2)
        self.motion_history.clear()
        self.scan_history.clear()
        self.navigation_snapshot('start', point)
        self.moving = True
        tree=self.config.get('normal_behavior_tree')
        accepted = self.nav.goToPose(goal,behavior_tree=tree) if tree else self.nav.goToPose(goal)
        if accepted is False:
            raise RuntimeError('Nav2가 이동 목표를 거절했습니다.')
        deadline = time.monotonic() + self.config['nav_timeout']
        next_sample = 0
        while not self.nav.isTaskComplete():
            if time.monotonic() >= next_sample:
                sample = self.navigation_sample()
                self.motion_history.append(sample)
                scan = self.current_scan_record()
                if scan is not None:
                    self.scan_history.append(scan)
                event('navigation_telemetry', **sample)
                next_sample = time.monotonic() + 1
            if time.monotonic() > deadline:
                self.navigation_snapshot('timeout', point)
                raise RuntimeError('Nav2 이동 시간 초과')
        self.moving = False
        result = self.nav.getResult()
        if result != TaskResult.SUCCEEDED:
            self.navigation_snapshot('failed', point)
            raise RuntimeError('Nav2 이동 실패: ' + result.name)
        self.stationary()
        actual = self.pose()
        if actual is None:
            raise RuntimeError('도착 후 map TF 없음')
        check_position = (state != 'NAVIGATING_TO_PICK' or
                          self.config.get('check_pick_position', False))
        distance, angle = check_arrival(actual, point, check_position,
            position_limit=.15,
            yaw_limit=.15)
        event('arrived', distance_error_m=distance, yaw_error_rad=angle,
              position_check=check_position)
        return actual

    def odom_position(self):
        if self.odom is None or time.monotonic()-self.odom[1] > 1:
            raise RuntimeError('정밀 접근 중 오도메트리 수신 중단')
        msg = self.odom[0]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        age = self.nav.get_clock().now().nanoseconds/1e9-stamp
        if not -.5 < age < 1.5:
            raise RuntimeError('정밀 접근 중 오래된 오도메트리')
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        return {'x': p.x, 'y': p.y,
                'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}

    def short_move(self, distance, state):
        """Nav2 collision-checked relative motion in the current body direction."""
        from nav2_msgs.action import DriveOnHeading, BackUp
        from nav2_simple_commander.robot_navigator import TaskResult
        if not math.isfinite(distance) or not 0 < abs(distance) <= .30:
            raise ValueError('정밀 접근 거리는 0 초과 0.30m 이하')
        self.stationary()
        origin = self.odom_position()
        forward = distance > 0
        client = self.drive_client if forward else self.nav.backup_client
        name = 'drive_on_heading' if forward else 'backup'
        event('state', state=state, distance_m=distance, speed_mps=.04, action=name)
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f'Nav2 {name} 액션 없음: behavior_server 설정 확인')
        goal = DriveOnHeading.Goal() if forward else BackUp.Goal()
        goal.target.x = abs(distance)
        goal.speed = .04
        goal.time_allowance.sec = 15
        self.moving = True
        self.pending_short_goal = client.send_goal_async(goal)
        # Also cancel a late accepted goal if the response wait is interrupted.
        canceled = [False]
        def cancel_late(future):
            handle = future.result()
            if canceled[0] and handle is not None and handle.accepted:
                handle.cancel_goal_async()
        self.pending_short_goal.add_done_callback(cancel_late)
        try:
            deadline = time.monotonic()+5
            while not self.pending_short_goal.done():
                self.rclpy.spin_once(self.nav, timeout_sec=.1)
                if time.monotonic() > deadline:
                    raise RuntimeError(f'Nav2 {name} 목표 응답 시간 초과')
            self.nav.goal_handle = self.pending_short_goal.result()
            if self.nav.goal_handle is None or not self.nav.goal_handle.accepted:
                raise RuntimeError(f'Nav2 {name} 목표 거절')
            self.nav.result_future = self.nav.goal_handle.get_result_async()
            deadline = time.monotonic()+17
            while not self.nav.isTaskComplete():
                self.odom_position()
                if time.monotonic() > deadline:
                    raise RuntimeError(f'Nav2 {name} 시간 초과')
            if self.nav.getResult() != TaskResult.SUCCEEDED:
                raise RuntimeError(f'Nav2 {name} 실패: 충돌/TF/시간 제한 확인')
            self.moving = False
        except BaseException:
            canceled[0] = True
            if self.pending_short_goal.done():
                cancel_late(self.pending_short_goal)
            raise
        finally:
            if self.pending_short_goal.done():
                self.pending_short_goal = None
        self.stationary()
        actual = self.odom_position()
        measured = verify_short_motion(origin, actual, distance)
        if abs(measured) > .30:
            raise RuntimeError('측정된 정밀 이동 거리가 0.30m를 초과했습니다.')
        event('short_move_done', state=state, distance_m=distance,
              measured_distance_m=measured, odometry=actual)
        return measured

    def collision_bypassed_reverse_step(self, distance):
        """Bounded recovery only: odometry-controlled reverse, no costmap veto."""
        from geometry_msgs.msg import Twist
        if not math.isfinite(distance) or not 0 < distance <= .05:
            raise ValueError('자동 추가 후진은 1회 최대 5cm')
        self.stationary()
        origin=self.odom_position()
        start=time.monotonic()
        last_progress=start
        best=0.
        try:
            while True:
                self.rclpy.spin_once(self.nav,timeout_sec=.05)
                actual=self.odom_position()
                dx,dy=actual['x']-origin['x'],actual['y']-origin['y']
                c,s=math.cos(origin['yaw']),math.sin(origin['yaw'])
                progress=-(c*dx+s*dy)
                lateral=-s*dx+c*dy
                angle=abs(math.atan2(math.sin(actual['yaw']-origin['yaw']),
                                     math.cos(actual['yaw']-origin['yaw'])))
                if not all(math.isfinite(v) for v in (progress,lateral,angle)):
                    raise RuntimeError('자동 후진 오도메트리 값 오류')
                if abs(lateral)>.03 or angle>.12 or progress<-.01:
                    raise RuntimeError('자동 후진 중 방향/옆밀림 오차 초과')
                if progress>=distance-.005:
                    break
                now=time.monotonic()
                if progress>best+.002:
                    best=progress
                    last_progress=now
                if now-last_progress>2 or now-start>distance/.03+4:
                    raise RuntimeError('자동 후진 진전 없음: 추가 밀기를 중단합니다.')
                command=Twist()
                command.linear.x=-.03
                self.velocity.publish(command)
        finally:
            self.stop_base()
        self.stationary()

    def retreat_with_retry(self, distance):
        if not math.isfinite(distance) or not 0 < distance <= .30:
            raise ValueError('후진 거리 설정 오류')
        self.stationary()
        origin=self.odom_position()
        try:
            return self.short_move(-distance,'RETREATING_FROM_PICK')
        except RuntimeError as exc:
            event('retreat_retry',message=str(exc),extra_step_m=.05,
                  max_extra_steps=3,collision_check=False)
        # Ensure Nav2 has stopped commanding the base before publishing directly.
        if self.pending_short_goal is not None:
            raise LocalizationNotReady('후진 목표 취소 응답 대기: 작업을 보류합니다.')
        if self.moving:
            self.nav.cancelTask()
            deadline=time.monotonic()+3
            while not self.nav.isTaskComplete():
                self.rclpy.spin_once(self.nav,timeout_sec=.1)
                if time.monotonic()>deadline:
                    raise LocalizationNotReady('Nav2 후진 취소 확인 실패')
            self.moving=False
        self.stationary()
        budget=min(.30,distance+.15)
        for attempt in range(4):
            actual=self.odom_position()
            dx,dy=actual['x']-origin['x'],actual['y']-origin['y']
            c,s=math.cos(origin['yaw']),math.sin(origin['yaw'])
            backed=-(c*dx+s*dy)
            lateral=-s*dx+c*dy
            angle=abs(math.atan2(math.sin(actual['yaw']-origin['yaw']),
                                 math.cos(actual['yaw']-origin['yaw'])))
            if (not all(math.isfinite(v) for v in (backed,lateral,angle)) or
                    abs(lateral)>.03 or angle>.12 or backed<-.01 or backed>budget+.01):
                raise LocalizationNotReady('후진 복구 위치/방향 검사 실패')
            if backed>=distance-.015:
                event('retreat_recovered',measured_distance_m=backed,extra_steps=attempt)
                return -backed
            if attempt==3:
                break
            step=min(.05,budget-max(0.,backed))
            event('state',state='RETREATING_RECOVERY',attempt=attempt+1,
                  distance_m=step,speed_mps=.03,collision_check=False)
            try:
                self.collision_bypassed_reverse_step(step)
            except RuntimeError as exc:
                raise LocalizationNotReady('자동 후진 중단: '+str(exc)) from exc
        raise LocalizationNotReady('자동 추가 후진 3회 후에도 거리 미달: 작업을 보류합니다.')

    def arm(self, command, color=None):
        # Convert cancellation into ArmStop INSIDE arm_cli.run. Its error path
        # holds torque and detaches, unlike an unhandled KeyboardInterrupt that
        # could enter the normal shutdown/park path. Do not modify arm_cli.py.
        bridge = ('import signal\nfrom robot.arm import ArmStop\n'
                  'def stop(signum, frame):\n    raise ArmStop("작업 제어기 취소 요청")\n'
                  'signal.signal(signal.SIGINT, stop)\n'
                  'signal.signal(signal.SIGTERM, stop)\n'
                  'from robot.tasks import ArmTasks\nfrom robot import park as P\n'
                  'import sys\nsys.path.insert(0, ' + repr(str(Path(__file__).resolve().parent)) + ')\n'
                  'from robot_mission import install_park_guard\n'
                  'install_park_guard(ArmTasks, P.near_park, ArmStop)\n'
                  'import arm_cli,sys,json\n'
                  'if sys.argv[1] == "recover":\n'
                  '    from robot_mission import run_arm_reset\n'
                  '    print(json.dumps(run_arm_reset()))\n'
                  'else:\n    arm_cli.main()\n')
        argv = [self.config['arm_python'], '-u', '-c', bridge, command]
        if color:
            argv += ['--color', color]
        path = self.log_root / ('arm-' + command + '.log')
        with tempfile_output() as output, path.open('wb') as error:
            self.arm_process = subprocess.Popen(argv, cwd=Path(self.config['arm_cli']).parent,
                stdin=subprocess.DEVNULL, stdout=output, stderr=error, start_new_session=True)
            deadline = time.monotonic() + self.config['arm_timeout']
            while self.arm_process.poll() is None:
                self.rclpy.spin_once(self.nav, timeout_sec=.1)
                # Interrupt any arm command if fresh odometry reports base motion.
                if self.odom and time.monotonic() - self.odom[1] < 1:
                    v = self.odom[0].twist.twist
                    if math.hypot(v.linear.x, v.linear.y) > .02 or abs(v.angular.z) > .05:
                        raise RuntimeError('팔 작업 중 베이스 움직임 감지')
                elif self.odom is None or time.monotonic() - self.odom[1] > 2:
                    raise RuntimeError('팔 작업 중 오도메트리 수신 중단')
                if time.monotonic() > deadline:
                    raise RuntimeError('팔 명령 시간 초과: ' + command)
            code = self.arm_process.returncode
            self.arm_process = None
            output.seek(0)
            result = last_json(output.read().decode(errors='replace'))
        event('arm_result', command=command, result=result)
        if code or result.get('ok') is not True:
            raise RuntimeError(f'팔 {command} 실패: {result}')
        if command == 'pick' and (result.get('result') != 'picked' or
                result.get('holding') is not True or result.get('pose') != 'carry'):
            raise RuntimeError('집기 성공·운반 자세 확인 실패')
        expected = {'place': 'placed', 'park': 'parked', 'status': 'status','recover':'initialized'}
        if command in expected and result.get('result') != expected[command]:
            raise RuntimeError('예상과 다른 팔 결과: ' + str(result))
        if command == 'place' and (result.get('holding') is not False or result.get('pose') != 'park'):
            raise RuntimeError('내려놓기·팔 접기 결과 확인 실패: ' + str(result))
        if command == 'recover' and (result.get('holding') is not False or
                result.get('pose') != 'park' or result.get('torque_off') is not True):
            raise RuntimeError('팔 실패 복구·초기화 확인 실패: '+str(result))
        return result

    def recover_after_failure(self):
        event('state',state='RECOVERING_ARM')
        if self.arm_process is not None:
            stop_group(self.arm_process,graces=(3,1,1))
            self.arm_process=None
        if self.moving:
            self.nav.cancelTask()
            deadline=time.monotonic()+3
            while not self.nav.isTaskComplete():
                self.rclpy.spin_once(self.nav,timeout_sec=.1)
                if time.monotonic()>deadline:
                    raise RuntimeError('실패 복구 전 Nav2 정지 확인 실패')
            self.moving=False
        self.stationary()
        result=self.arm('recover')
        event('arm_initialized',result=result)

    def run(self, request):
        validate_request(request)
        approach = float(self.config.get('pick_approach_distance', .10))
        if not math.isfinite(approach) or not 0 <= approach <= .30:
            raise ValueError('정밀 접근 거리 설정 오류')
        try:
            current = self.localize()
        except RuntimeError as exc:
            raise LocalizationNotReady('위치 추정 대기: '+str(exc)) from exc
        event('state', state='PREPARING')
        self.stationary()
        status = self.arm('status')
        if status.get('holding'):
            raise RuntimeError('팔이 이미 블록을 보유 중입니다. 기존 작업을 먼저 확인하세요.')
        self.arm('park')
        if request.get('navigate_only'):
            self.navigate(request['destination'],'NAVIGATING_ONLY')
            event('state',state='DONE',mode='navigate_only')
            return
        start = current if request['start'] == 'current' else request['start']
        self.navigate(request['destination'], 'NAVIGATING_TO_PICK')
        if approach:
            self.set_pick_footprint(True)
            advanced = self.short_move(approach, 'APPROACHING_PICK')
        event('state', state='PICKING', color=request['color'])
        self.arm('pick', request['color'])
        if approach:
            self.retreat_with_retry(advanced)
            self.set_pick_footprint(False)
        self.navigate(start, 'RETURNING')
        if self.config.get('place_on_return'):
            event('state', state='PLACING')
            self.arm('place')
        event('state', state='DONE', holding=not self.config.get('place_on_return', False))

    def close(self):
        try:
            # The arm has its own process group. Stop it FIRST, within the
            # supervisor's grace period, so forced worker termination cannot
            # leave an arm command running independently.
            if self.arm_process:
                stop_group(self.arm_process, graces=(3, 1, 1))
        finally:
            try:
                if self.moving:
                    self.nav.cancelTask()
                self.stop_base()
                if getattr(self, 'original_local_footprint', None) is not None:
                    try:
                        self.set_pick_footprint(False)
                    except Exception as exc:
                        event('footprint_restore_error', message=str(exc))
            finally:
                if getattr(self, 'pending_short_goal', None) is not None:
                    # Process a late goal acknowledgement so its cancellation
                    # callback can run before the action client is destroyed.
                    end = time.monotonic()+2
                    while not self.pending_short_goal.done() and time.monotonic() < end:
                        self.rclpy.spin_once(self.nav, timeout_sec=.1)
                if hasattr(self, 'drive_client'):
                    self.drive_client.destroy()
                self.nav.destroy_node()
                self.rclpy.shutdown()


def tempfile_output():
    import tempfile
    return tempfile.TemporaryFile()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['stack', 'check', 'probe', 'mission'], required=True)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--request', type=Path)
    parser.add_argument('--kind', choices=['graph', 'bringup', 'nav2', 'initial'])
    parser.add_argument('--timeout', type=float, default=120)
    args = parser.parse_args()
    if args.mode == 'probe':
        if args.kind == 'initial':
            initialize_pose(json.loads(args.config.read_text())['initial_pose'], args.timeout)
        else:
            ros_probe(args.kind, args.timeout)
        return
    config = json.loads(args.config.read_text())
    if args.mode == 'check':
        dependencies(config)
    elif args.mode == 'stack':
        run_stack(config, args.config)
    else:
        # Take the project-wide arm lock only inside arm_cli; never share its bus.
        mission = Mission(config)
        try:
            mission.run(json.loads(args.request.read_text()))
        except Exception as exc:
            event('state', state='ERROR', message=str(exc) or type(exc).__name__)
            if mission.current_goal is not None:
                try:
                    mission.navigation_snapshot('error',mission.current_goal)
                except Exception as snapshot_error:
                    event('snapshot_error',message=str(snapshot_error))
            try:
                mission.recover_after_failure()
            except Exception as recovery_error:
                event('arm_recovery_failed',message=str(recovery_error))
            # Fail only after the arm recovery attempt has completed. The
            # supervisor then tears down bringup/Nav2 and the laptop exits.
            raise RuntimeError(str(exc)) from exc
        finally:
            mission.close()


if __name__ == '__main__':
    stop_requested = False
    def interrupted(signum, frame):
        global stop_requested
        if stop_requested:
            return
        stop_requested = True
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        event('error', message=str(exc))
        raise SystemExit(1)
