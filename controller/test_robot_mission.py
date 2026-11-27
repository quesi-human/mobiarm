"""Offline tests. No SSH connection, motor access or ROS launches."""
import argparse
from collections import deque
from concurrent.futures import Future
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import execute_robot as cli
import robot_mission as worker
import scan_self_filter as self_filter
import math
import pick_place

POINT = {'frame_id': 'map', 'x': -3.4, 'y': -.47, 'yaw': -1.558}
HOME = {'frame_id': 'map', 'x': 0., 'y': 0., 'yaw': 0.}


class SelfFilterTests(unittest.TestCase):
    def test_chassis_reflections_removed_external_wall_preserved(self):
        angles = [56, 117, 238, 298]
        for angle in angles:
            values, count = self_filter.mask_ranges([.15, .8], math.radians(angle), 0,
                .1, 10., (-.064, 0., .122), (0.,0.,0.,1.), [-.21,.09,-.16,.16])
            self.assertTrue(math.isnan(values[0]), angle)
            self.assertEqual(values[1], .8)
            self.assertEqual(count, 1)

    def test_nearby_external_object_and_invalid_measurements_preserved(self):
        original = [.18, float('inf'), float('nan'), .05]
        values, count = self_filter.mask_ranges(original, 0, 0, .1, 10.,
            (-.064,0.,.122), (0.,0.,0.,1.), [-.21,.09,-.16,.16])
        self.assertEqual(count, 0)  # .18 m beam is outside the front of the body.
        self.assertEqual(values[0], .18)
        self.assertTrue(math.isinf(values[1]))
        self.assertTrue(math.isnan(values[2]))
        self.assertEqual(original[0], .18)

    def test_mounting_rotation_is_applied(self):
        values, count = self_filter.mask_ranges([.15], 0, 0, .1, 10., (0.,0.,0.),
            (0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)), [-.02,.02,.14,.16])
        self.assertEqual(count, 1)
        self.assertTrue(math.isnan(values[0]))

    def test_all_navigation_scan_consumers_use_filtered_topic(self):
        import yaml
        profile = yaml.safe_load(Path(__file__).with_name('nav2_mobile_pick.yaml').read_text())
        self.assertEqual(profile['amcl']['ros__parameters']['scan_topic'], '/scan_filtered')
        for name in ('local_costmap','global_costmap'):
            params = profile[name][name]['ros__parameters']
            self.assertEqual(params['obstacle_layer']['scan']['topic'], '/scan_filtered')


class FakeMission(worker.Mission):
    def __init__(self, failure=None, place=False):
        self.config = {'place_on_return': place}
        self.trace, self.failure = [], failure

    def localize(self):
        self.trace.append('localize')
        return dict(HOME)

    def stationary(self):
        self.trace.append('stopped')

    def arm(self, command, color=None):
        self.trace.append((command, color))
        if self.failure == command:
            raise RuntimeError(command + ' failed')
        return {'holding': False}

    def navigate(self, point, state):
        self.trace.append((state, point))
        if self.failure == state:
            raise RuntimeError(state + ' failed')

    def short_move(self, distance, state):
        self.trace.append((state, distance))
        if self.failure == state:
            raise RuntimeError(state + ' failed')
        return distance

    def set_pick_footprint(self, enabled):
        self.trace.append(('pick_footprint', enabled))
        if self.failure == 'footprint':
            raise RuntimeError('footprint acknowledgement failed')


class MissionTests(unittest.TestCase):
    def test_pick_position_guard_can_be_skipped_but_yaw_and_return_stay_checked(self):
        goal = dict(x=0.,y=0.,yaw=0.)
        actual = dict(x=.158,y=0.,yaw=math.radians(3.6))
        distance, angle = worker.check_arrival(actual,goal,check_position=False)
        self.assertAlmostEqual(distance,.158)
        with self.assertRaises(RuntimeError):
            worker.check_arrival(actual,goal)
        with self.assertRaises(RuntimeError):
            worker.check_arrival(dict(actual,yaw=.2),goal,check_position=False)
        with self.assertRaises(RuntimeError):
            worker.check_arrival(dict(actual,x=float('nan')),goal,check_position=False)

    def test_retreat_uses_measured_advance_instead_of_requested_distance(self):
        mission = FakeMission()
        def move(distance,state):
            mission.trace.append((state,distance))
            return .113 if distance>0 else distance
        mission.short_move = move
        with contextlib.redirect_stdout(io.StringIO()):
            mission.run(self.request())
        self.assertIn(('APPROACHING_PICK', .1),mission.trace)
        self.assertIn(('RETREATING_FROM_PICK', -.113),mission.trace)

    def test_arm_only_pick_and_place_validate_real_result_contract(self):
        calls = []
        def run(cmd, **kwargs):
            calls.append((cmd,kwargs))
            return {'status':{'ok':True,'holding':False},
                'pick':{'ok':True,'result':'picked','holding':True,'pose':'carry'},
                'place':{'ok':True,'result':'placed','holding':False,'pose':'park'}}[cmd]
        self.assertEqual(pick_place.sequence(run, 'red')['result'], 'placed')
        self.assertEqual(calls,[('status',{}),('pick',{'color':'red'}),('place',{})])

    def test_arm_only_failed_pick_never_places(self):
        calls = []
        def run(cmd, **kwargs):
            calls.append(cmd)
            return {'ok':True,'holding':False} if cmd=='status' else {'ok':False,'result':'missed'}
        with self.assertRaises(RuntimeError):
            pick_place.sequence(run)
        self.assertNotIn('place',calls)

    def test_arm_only_held_block_requires_place_only(self):
        calls = []
        def run(cmd, **kwargs):
            calls.append(cmd)
            return ({'ok':True,'holding':True} if cmd=='status' else
                    {'ok':True,'result':'placed','holding':False,'pose':'park'})
        with self.assertRaises(RuntimeError):
            pick_place.sequence(run)
        self.assertEqual(calls,['status'])
        calls.clear()
        pick_place.sequence(run,place_only=True)
        self.assertEqual(calls,['status','place'])

    def test_park_guard_rejects_false_success_and_preserves_valid_park(self):
        calls = []
        class Tasks:
            park_q = {'joint':0.}
            arm = SimpleNamespace(read_deg=lambda: {'joint':20.})
            torque = False
            def _torque_is_on(self):
                return self.torque
            def park(self):
                calls.append('park')
                return {'result':'parked'}
        worker.install_park_guard(Tasks, lambda actual,expected: actual==expected, RuntimeError)
        t = Tasks()
        with self.assertRaisesRegex(RuntimeError, '접힘 자세'):
            t.park()
        self.assertEqual(calls, [])
        t.arm = SimpleNamespace(read_deg=lambda: {'joint':0.})
        self.assertEqual(t.park()['result'], 'parked')
        t.arm = SimpleNamespace(read_deg=lambda: {'joint':20.})
        t.torque = True
        self.assertEqual(t.park()['result'], 'parked')

    def test_path_record_preserves_frame_and_heading(self):
        import math
        pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1.,y=2.),
            orientation=SimpleNamespace(x=0.,y=0.,z=math.sin(.5),w=math.cos(.5))))
        msg=SimpleNamespace(header=SimpleNamespace(frame_id='map',
            stamp=SimpleNamespace(sec=10,nanosec=500000000)),poses=[pose])
        record=worker.path_record(msg)
        self.assertEqual(record['frame_id'],'map')
        self.assertEqual(record['stamp'],10.5)
        self.assertAlmostEqual(record['poses'][0]['yaw'],1.)

    def test_laser_record_exposes_future_last_ray_and_preserves_sensor_frame(self):
        scan = SimpleNamespace(header=SimpleNamespace(frame_id='base_scan',
            stamp=SimpleNamespace(sec=10,nanosec=0)), ranges=[1.,float('inf'),2.],
            time_increment=.05,scan_time=.1,angle_min=0.,angle_max=2.,angle_increment=1.,
            range_min=.1,range_max=10.)
        record = worker.laser_record(scan,10.02)
        self.assertEqual(record['frame_id'],'base_scan')
        self.assertEqual(record['ranges'],[1.,None,2.])
        self.assertAlmostEqual(record['last_ray_age_s'],-.08)
        self.assertEqual(scan.header.stamp.sec,10)

    def test_diagnostic_rviz_keeps_map_fixed_and_scan_live(self):
        import yaml
        cfg=yaml.safe_load(Path(__file__).with_name('mobile_pick.rviz').read_text())
        manager=cfg['Visualization Manager']
        self.assertEqual(manager['Global Options']['Fixed Frame'],'map')
        scan=next(v for v in manager['Displays'] if v['Class']=='rviz_default_plugins/LaserScan')
        self.assertTrue(scan['Use Fixed Frame'])
        self.assertEqual(scan['Decay Time'],0)
        self.assertEqual(scan['Topic']['Value'],'/scan_filtered')
        for item in manager['Displays']:
            if item.get('Class')=='rviz_common/Group':
                self.assertFalse(item['Enabled'])

    def test_lifecycle_lost_response_is_retried(self):
        lost, recovered = Future(), Future()
        recovered.set_result(SimpleNamespace(current_state=SimpleNamespace(id=3, label='active')))
        client = self.lifecycle_client([lost, recovered])
        poller = worker.LifecyclePoller({'amcl': client}, object)
        poller.tick(0.)
        poller.tick(3.1)
        self.assertTrue(lost.cancelled())
        self.assertEqual(client.removed, [lost])
        poller.tick(3.4)
        poller.tick(3.5)
        self.assertEqual(poller.active(3.5), {'amcl'})
        self.assertEqual(poller.retries['amcl'], 1)
        self.assertEqual(client.calls, 2)

    @staticmethod
    def lifecycle_client(responses):
        class Client:
            def __init__(self):
                self.responses, self.removed, self.calls = list(responses), [], 0
            def service_is_ready(self):
                return True
            def call_async(self, request):
                self.calls += 1
                return self.responses.pop(0) if self.responses else Future()
            def remove_pending_request(self, future):
                self.removed.append(future)
        return Client()

    def test_inactive_lifecycle_node_is_polled_until_active(self):
        inactive, active = Future(), Future()
        inactive.set_result(SimpleNamespace(current_state=SimpleNamespace(id=2, label='inactive')))
        active.set_result(SimpleNamespace(current_state=SimpleNamespace(id=3, label='active')))
        poller = worker.LifecyclePoller({'planner': self.lifecycle_client([inactive, active])}, object)
        poller.tick(0.)
        poller.tick(.1)
        self.assertEqual(poller.active(.1), set())
        poller.tick(1.2)
        poller.tick(1.3)
        self.assertEqual(poller.active(1.3), {'planner'})

    def test_old_active_observation_does_not_remain_ready_forever(self):
        active = Future()
        active.set_result(SimpleNamespace(current_state=SimpleNamespace(id=3, label='active')))
        poller = worker.LifecyclePoller({'amcl': self.lifecycle_client([active])}, object)
        poller.tick(0.)
        poller.tick(.1)
        self.assertEqual(poller.active(.1), {'amcl'})
        self.assertEqual(poller.active(7.), set())
        poller.close()

    def test_partial_lifecycle_replies_do_not_bypass_readiness(self):
        active = Future()
        active.set_result(SimpleNamespace(current_state=SimpleNamespace(id=3, label='active')))
        poller = worker.LifecyclePoller({'controller': self.lifecycle_client([active]),
                                       'planner': self.lifecycle_client([Future()])}, object)
        poller.tick(0.)
        poller.tick(.1)
        self.assertEqual(poller.active(.1), {'controller'})
        self.assertNotEqual(poller.active(.1), set(poller.clients))
        poller.close()

    def test_navigation_telemetry_separates_command_and_measured_velocity(self):
        mission = worker.Mission.__new__(worker.Mission)
        mission.pose = lambda: dict(HOME)
        command = SimpleNamespace(linear=SimpleNamespace(x=.12), angular=SimpleNamespace(z=.2))
        measured = SimpleNamespace(linear=SimpleNamespace(x=0.), angular=SimpleNamespace(z=.2))
        msg = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0., y=0.))),
                              twist=SimpleNamespace(twist=measured))
        mission.commands = {'/cmd_vel': (command, time.monotonic())}
        mission.odom = (msg, time.monotonic())
        sample = mission.navigation_sample()
        self.assertEqual(sample['commands']['/cmd_vel']['vx'], .12)
        self.assertEqual(sample['odometry']['vx'], 0.)
        self.assertGreaterEqual(sample['commands']['/cmd_vel']['age_s'], 0)

    def test_failure_snapshot_preserves_local_costmap(self):
        mission = worker.Mission.__new__(worker.Mission)
        mission.navigation_sample = lambda: {'pose': HOME}
        mission.motion_history = deque([{'odometry': {'vx': 0.}}])
        grid = SimpleNamespace(header=SimpleNamespace(frame_id='odom', stamp=SimpleNamespace(sec=10, nanosec=0)),
            info=SimpleNamespace(width=2, height=1, resolution=.05,
                origin=SimpleNamespace(position=SimpleNamespace(x=1., y=2., z=0.),
                                       orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))), data=[0,100])
        mission.costmaps = {'/local_costmap/costmap': (grid, time.monotonic())}
        with tempfile.TemporaryDirectory() as directory:
            mission.log_root = Path(directory)
            with contextlib.redirect_stdout(io.StringIO()):
                mission.navigation_snapshot('failed', POINT)
            saved = json.loads((Path(directory)/'navigation_failed.json').read_text())
        self.assertEqual(saved['costmaps']['/local_costmap/costmap']['data'], [0,100])
        self.assertEqual(saved['costmaps']['/local_costmap/costmap']['origin']['x'], 1.)

    def test_rectangle_backup_preserves_previous_body_envelope(self):
        import yaml
        profile = yaml.safe_load(Path(__file__).with_name('nav2_mobile_pick_rectangle_backup.yaml').read_text())
        polygons = []
        for name in ('global_costmap', 'local_costmap'):
            params = profile[name][name]['ros__parameters']
            self.assertNotIn('robot_radius', params)
            polygon = json.loads(params['footprint'])
            polygons.append(polygon)
            # Include the installed model's chassis and wheels, before padding.
            self.assertLessEqual(min(x for x, y in polygon), -.197)
            self.assertGreaterEqual(max(x for x, y in polygon), .081)
            self.assertLessEqual(min(y for x, y in polygon), -.153)
            self.assertGreaterEqual(max(y for x, y in polygon), .153)
            self.assertGreater(params['footprint_padding'], 0)
        self.assertEqual(*polygons)
        critics = profile['controller_server']['ros__parameters']['FollowPath']['critics']
        self.assertIn('ObstacleFootprint', critics)
        self.assertNotIn('BaseObstacle', critics)

    def test_circular_profile_matches_global_local_and_inflates_beyond_radius(self):
        import yaml
        profile = yaml.safe_load(Path(__file__).with_name('nav2_mobile_pick.yaml').read_text())
        models = []
        for name in ('global_costmap', 'local_costmap'):
            params = profile[name][name]['ros__parameters']
            self.assertEqual(json.loads(params['footprint']), [])
            self.assertGreater(params['robot_radius'], 0)
            self.assertGreater(params['inflation_layer']['inflation_radius'],
                               params['robot_radius'] + params['footprint_padding'])
            self.assertIn('obstacle_layer', params['plugins'])
            self.assertTrue(params['obstacle_layer']['scan']['marking'])
            models.append((params['robot_radius'], params['footprint_padding'], params['inflation_layer']))
        self.assertEqual(*models)
        follower = profile['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(follower['plugin'],
            'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')
        self.assertTrue(follower['use_collision_detection'])
        self.assertTrue(follower['use_rotate_to_heading'])
        self.assertLessEqual(follower['regulated_linear_scaling_min_speed'], follower['desired_linear_vel'])
        self.assertLessEqual(follower['cost_scaling_dist'], models[0][2]['inflation_radius'])
        self.assertEqual(follower['inflation_cost_scaling_factor'], models[0][2]['cost_scaling_factor'])

    def test_default_profile_is_selected_without_ros_or_robot(self):
        with patch.object(sys, 'argv', ['execute_robot.py', '--no-rviz', '--manual-initial-pose']):
            args = cli.parse_args()
        response = SimpleNamespace(returncode=0, stderr='', stdout=json.dumps({
            'params': '/unused/software-default.yaml', 'rviz': '/unused/rviz'}))
        with patch.object(cli.subprocess, 'run', return_value=response):
            metadata = cli.local_metadata(args)
        params = metadata[2]['global_costmap']['global_costmap']['ros__parameters']
        self.assertEqual(json.loads(params['footprint']), [])
        self.assertIn('robot_radius', params)

    def test_initial_home_heading_zero(self):
        z, w, covariance = worker.initial_pose_fields(HOME)
        self.assertEqual((z, w), (0., 1.))
        self.assertGreater(covariance[35], 0)
        with patch.object(sys, 'argv', ['execute_robot.py']):
            args = cli.parse_args()
        self.assertEqual(args.initial_waypoint, 'homepoint')
        self.assertFalse(args.manual_initial_pose)

    def test_automatic_initial_pose_rejects_unregistered_home(self):
        with patch.object(sys, 'argv', ['execute_robot.py', '--auto-initial-pose']):
            args = cli.parse_args()
        self.assertFalse(args.manual_initial_pose)
        with tempfile.TemporaryDirectory() as directory:
            file = Path(directory)/'waypoints.yaml'
            file.write_text('homepoint:\n  frame_id: map\n  x: null\n  y: null\n  yaw: null\n')
            args.waypoints = file
            with patch.object(cli.subprocess, 'run') as run, self.assertRaises(ValueError):
                cli.local_metadata(args)
            run.assert_not_called()

    def test_set_home_saves_pose_without_starting_ssh(self):
        import yaml
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'waypoints.yaml'
            path.write_text(yaml.safe_dump({'workbench': POINT}))
            args = ['execute_robot.py', '--waypoints', str(path), '--set-home', '0', '0', '0']
            with patch.object(sys, 'argv', args), patch.object(cli, 'SSH') as ssh, \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(cli.main(), 0)
            ssh.assert_not_called()
            saved = yaml.safe_load(path.read_text())
            self.assertEqual(saved['homepoint'], HOME)
            self.assertEqual(saved['workbench'], POINT)

    def test_set_home_rejects_invalid_coordinates_without_changing_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'waypoints.yaml'
            original = 'workbench: {}\n'
            path.write_text(original)
            with self.assertRaises(ValueError):
                cli.save_home(path, [float('nan'), 0., 0.])
            self.assertEqual(path.read_text(), original)

    def test_cli_places_by_default_with_explicit_hold_override(self):
        with patch.object(sys, 'argv', ['execute_robot.py']):
            self.assertTrue(cli.parse_args().place_on_return)
        with patch.object(sys, 'argv', ['execute_robot.py', '--no-place-on-return']):
            self.assertFalse(cli.parse_args().place_on_return)

    def test_closed_remote_stdin_does_not_mask_cleanup(self):
        class BrokenInput:
            def close(self):
                raise BrokenPipeError('remote exited')
        with tempfile.TemporaryDirectory() as directory:
            manager = cli.Processes(Path(directory))
            log = io.BytesIO()
            manager.entries.append(('remote', SimpleNamespace(stdin=BrokenInput()), log, Path('log')))
            with patch.object(cli, 'stop_group'):
                manager.close()
            self.assertTrue(log.closed)

    def request(self, start='current'):
        return {'start': start, 'destination': dict(POINT), 'color': 'yellow'}

    def test_pick_then_return_without_unrequested_place(self):
        mission = FakeMission()
        with contextlib.redirect_stdout(io.StringIO()):
            mission.run(self.request())
        self.assertEqual(mission.trace, ['localize', 'stopped', ('status', None),
            ('park', None), ('NAVIGATING_TO_PICK', POINT), ('pick_footprint', True),
            ('APPROACHING_PICK', .1), ('pick', 'yellow'), ('RETREATING_FROM_PICK', -.1),
            ('pick_footprint', False), ('RETURNING', HOME)])

    def test_unacknowledged_footprint_never_approaches_or_picks(self):
        mission = FakeMission(failure='footprint')
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
            mission.run(self.request())
        self.assertNotIn(('APPROACHING_PICK', .1), mission.trace)
        self.assertNotIn(('pick', 'yellow'), mission.trace)

    def test_footprint_round_trip_preserves_padding_and_custom_shape(self):
        polygon = [[.09,.16],[.09,-.16],[-.21,-.16],[-.21,.16]]
        padded = worker.pad_polygon(polygon, .005)
        restored = worker.pad_polygon(padded, -.005)
        self.assertTrue(worker.polygons_match(restored, polygon, 1e-8))
        self.assertFalse(worker.polygons_match(polygon, [[.18,0.],[0.,.18],[-.18,0.],[0.,-.18]]))
        self.assertTrue(worker.polygons_match(list(reversed(polygon)), polygon))

    def test_pick_footprint_waits_for_ack_then_restores_observed_travel_shape(self):
        import yaml
        class Polygon:
            pass
        clock = [10.]
        m = object.__new__(worker.Mission)
        m.nav = object()
        original = worker.pad_polygon([[.18,0.],[0.,.18],[-.18,0.],[0.,-.18]], .005)
        observed = [original]
        m.original_local_footprint = None
        m.stationary = lambda: None
        m.footprint_in_base = lambda: observed[0]
        m.costmaps = {}
        def spin(*args, **kwargs):
            clock[0] += .1
            m.costmaps['/local_costmap/costmap'] = (None,clock[0])
        m.rclpy = SimpleNamespace(spin_once=spin)
        def publish(msg):
            observed[0] = worker.pad_polygon([[p.x,p.y] for p in msg.points], .005)
        m.footprint_publisher = SimpleNamespace(publish=publish)
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'params.yaml'
            path.write_text(yaml.safe_dump({'local_costmap': {'local_costmap':
                {'ros__parameters': {'footprint_padding':.005}}}}))
            m.config = {'params_file':str(path)}
            modules = {'geometry_msgs.msg': SimpleNamespace(Polygon=Polygon,
                Point32=lambda **kwargs: SimpleNamespace(**kwargs))}
            with patch.dict(sys.modules,modules), patch.object(worker.time,'monotonic',lambda:clock[0]), \
                    contextlib.redirect_stdout(io.StringIO()):
                m.set_pick_footprint(True)
                self.assertEqual(len(observed[0]),4)
                self.assertAlmostEqual(max(p[0] for p in observed[0]), .095)
                self.assertIsNotNone(m.original_local_footprint)
                m.set_pick_footprint(False)
                self.assertTrue(worker.polygons_match(observed[0],original,1e-8))
                self.assertIsNone(m.original_local_footprint)
                m.footprint_publisher.publish = lambda msg: None
                with self.assertRaisesRegex(RuntimeError, 'footprint'):
                    m.set_pick_footprint(True)

    def test_failed_approach_never_picks(self):
        mission = FakeMission(failure='APPROACHING_PICK')
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
            mission.run(self.request())
        self.assertNotIn(('pick', 'yellow'), mission.trace)

    def test_failed_retreat_never_starts_return(self):
        mission = FakeMission(failure='RETREATING_FROM_PICK')
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
            mission.run(self.request())
        self.assertNotIn(('RETURNING', HOME), mission.trace)

    def test_pick_failure_does_not_retreat_with_unconfirmed_carry(self):
        mission = FakeMission(failure='pick')
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
            mission.run(self.request())
        self.assertNotIn(('RETREATING_FROM_PICK', -.1), mission.trace)

    def test_short_move_can_be_disabled(self):
        mission = FakeMission()
        mission.config['pick_approach_distance'] = 0
        with contextlib.redirect_stdout(io.StringIO()):
            mission.run(self.request())
        self.assertNotIn(('APPROACHING_PICK', .1), mission.trace)
        self.assertNotIn(('RETREATING_FROM_PICK', -.1), mission.trace)

    def test_short_motion_validation_uses_current_heading(self):
        origin = dict(x=2., y=3., yaw=-math.pi/2)
        worker.verify_short_motion(origin, dict(origin,y=2.9), .1)
        worker.verify_short_motion(origin, dict(origin,y=3.1), -.1)
        for actual in (dict(origin,y=3.1), dict(origin,x=2.1,y=2.9),
                       dict(origin,y=2.9,yaw=0.)):
            with self.assertRaises(RuntimeError):
                worker.verify_short_motion(origin, actual, .1)

    def test_short_move_uses_nav2_actions_and_rejects_unsuccessful_result(self):
        from enum import Enum
        class Result(Enum):
            SUCCEEDED = 1
            FAILED = 2
        class Goal:
            def __init__(self):
                self.target = SimpleNamespace(x=0.)
                self.time_allowance = SimpleNamespace(sec=0)
        def future(value):
            f = Future(); f.set_result(value); return f
        for distance in (.1, -.1):
            for success in (True, False):
                sent, canceled = [], []
                handle = SimpleNamespace(accepted=True,
                    get_result_async=lambda: future(None),
                    cancel_goal_async=lambda: canceled.append(True))
                client = SimpleNamespace(wait_for_server=lambda **kw: True,
                    send_goal_async=lambda goal: (sent.append(goal), future(handle))[1])
                m = object.__new__(worker.Mission)
                m.drive_client = client if distance > 0 else None
                m.nav = SimpleNamespace(backup_client=client if distance < 0 else None,
                    isTaskComplete=lambda: True,
                    getResult=lambda: Result.SUCCEEDED if success else Result.FAILED)
                m.stationary = lambda: None
                positions = iter([dict(x=0.,y=0.,yaw=0.), dict(x=distance,y=0.,yaw=0.)])
                m.odom_position = lambda: next(positions)
                modules = {'nav2_msgs.action': SimpleNamespace(
                    DriveOnHeading=SimpleNamespace(Goal=Goal),BackUp=SimpleNamespace(Goal=Goal)),
                    'nav2_simple_commander.robot_navigator': SimpleNamespace(TaskResult=Result)}
                with patch.dict(sys.modules, modules), contextlib.redirect_stdout(io.StringIO()):
                    if success:
                        m.short_move(distance, 'TEST')
                        self.assertFalse(m.moving)
                    else:
                        with self.assertRaises(RuntimeError):
                            m.short_move(distance, 'TEST')
                        self.assertTrue(canceled)
                self.assertEqual(sent[0].target.x, .1)
                self.assertEqual(sent[0].speed, .04)

    def test_named_return_point(self):
        home = dict(HOME, yaw=.7)
        mission = FakeMission()
        with contextlib.redirect_stdout(io.StringIO()):
            mission.run(self.request(home))
        self.assertEqual(mission.trace[-1], ('RETURNING', home))

    def test_pick_failure_does_not_start_return(self):
        mission = FakeMission(failure='pick')
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
            mission.run(self.request())
        self.assertFalse(any(isinstance(x, tuple) and x[0] == 'RETURNING' for x in mission.trace))

    def test_navigation_failure_does_not_start_pick(self):
        mission = FakeMission(failure='NAVIGATING_TO_PICK')
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
            mission.run(self.request())
        self.assertNotIn(('pick', 'yellow'), mission.trace)

    def test_place_only_when_requested(self):
        mission = FakeMission(place=True)
        with contextlib.redirect_stdout(io.StringIO()):
            mission.run(self.request())
        self.assertEqual(mission.trace[-1], ('place', None))

    def test_return_failure_does_not_place(self):
        mission = FakeMission(failure='RETURNING', place=True)
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
            mission.run(self.request())
        self.assertNotIn(('place', None), mission.trace)

    def test_bad_color_rejected_before_movement(self):
        mission = FakeMission()
        request = dict(self.request(), color='black')
        with self.assertRaises(ValueError):
            mission.run(request)
        self.assertEqual(mission.trace, [])

    def test_invalid_points(self):
        for point in (dict(POINT, yaw=None), dict(POINT, x=float('nan')),
                      dict(POINT, frame_id='odom'), dict(POINT, y=True)):
            with self.subTest(point=point), self.assertRaises(ValueError):
                worker.validate_point(point)

    def test_cli_korean_color(self):
        args = argparse.Namespace(start='current', destination='workbench', color='빨강')
        with contextlib.redirect_stdout(io.StringIO()):
            result = cli.collect_request(args, {'workbench': POINT}, lambda _: self.fail('unexpected prompt'))
        self.assertEqual(result['color'], 'red')

    def test_cli_prompts_for_missing_yaw_without_guessing(self):
        args = argparse.Namespace(start='homepoint', destination='workbench', color='blue')
        points = {'workbench': dict(POINT), 'homepoint': dict(HOME, yaw=None)}
        prompts = []
        def ask(prompt):
            prompts.append(prompt)
            return '-1.2'
        with contextlib.redirect_stdout(io.StringIO()):
            result = cli.collect_request(args, points, ask)
        self.assertEqual(result['start']['yaw'], -1.2)
        self.assertIsNone(points['homepoint']['yaw'])
        self.assertEqual(len(prompts), 1)

    def test_unknown_destination(self):
        args = argparse.Namespace(start='current', destination='unknown', color='red')
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(ValueError):
            cli.collect_request(args, {'workbench': POINT}, input)

    def test_shell_source_and_argument_escaping(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / '.bashrc').write_text('case $- in *i*) ;; *) return;; esac\nexport ROBOT_TEST=sourced\n')
            argument = 'space; $(touch SHOULD_NOT_EXIST)'
            code = 'import os,sys,json; print(json.dumps([os.environ["ROBOT_TEST"],sys.argv[1]]))'
            result = subprocess.run(cli.shell_command([sys.executable, '-c', code, argument], 36, 'waffle_pi'),
                env=dict(os.environ, HOME=directory), cwd=directory, capture_output=True, text=True, check=True)
            self.assertEqual(json.loads(result.stdout), ['sourced', argument])
            self.assertFalse((root / 'SHOULD_NOT_EXIST').exists())

    def test_remote_upload_uses_relative_image_reference(self):
        import yaml
        class FakeSSH:
            def __init__(self):
                self.files = {}
            def run(self, argv):
                return '/home/turtlebot6\n'
            def upload(self, path, data):
                self.files[str(path)] = data
        args = argparse.Namespace(remote_project=None, timeout=120, nav_timeout=180,
            arm_timeout=180, reuse_bringup=False, reuse_nav2=False, place_on_return=False,
            manual_initial_pose=True)
        ssh = FakeSSH()
        with tempfile.TemporaryDirectory() as directory:
            image = Path(directory) / 'test.pgm'
            image.write_bytes(b'P5\n1 1\n255\n\xff')
            metadata = ({'image': '/old/machine/image.pgm', 'resolution': .05}, image,
                        {'amcl': {}}, Path('rviz'), {'workbench': POINT})
            worker_path, config_path = cli.deploy(args, ssh, metadata, 'test')
        map_path = str(config_path.with_name('map.yaml'))
        self.assertEqual(yaml.safe_load(ssh.files[map_path])['image'], 'map.pgm')
        self.assertIn('mission_control/runs/test', str(worker_path))

    def test_partial_event_lines_are_not_lost(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'events.log'
            line = worker.PREFIX + json.dumps({'event': 'ready'})
            path.write_text(line[:10])
            events = cli.Events(path)
            self.assertEqual(events.read(), [])
            with path.open('a') as stream:
                stream.write(line[10:] + '\n')
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(events.read(), [{'event': 'ready'}])

    def test_remote_process_timeout_is_cleaned_up(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = worker.Processes(directory)
            try:
                with self.assertRaisesRegex(RuntimeError, '시간 초과'):
                    manager.run('sleep', [sys.executable, '-c', 'import time; time.sleep(30)'], .15)
                process = manager.entries[0][1]
            finally:
                manager.close()
            self.assertIsNotNone(process.poll())

    def test_remote_nav2_command_is_headless(self):
        class Control:
            def __init__(self):
                self.messages = __import__('queue').Queue()
                self.messages.put({'start': 'current', 'destination': POINT, 'color': 'red'})
            def check(self):
                pass
        commands = []
        class Manager:
            def __init__(self, *args):
                pass
            def run(self, name, argv, timeout):
                return '[]' if name == 'graph' else ''
            def start(self, name, argv):
                commands.append((name, argv))
            def check(self):
                pass
            def close(self):
                commands.append(('closed', []))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session/config.json'
            path.parent.mkdir()
            config = {'logs': directory, 'timeout': 5, 'nav_timeout': 5, 'arm_timeout': 5,
                'reuse_bringup': False, 'reuse_nav2': False, 'map': '/map.yaml', 'params_file': '/params.yaml',
                'navigation_launch': '/onboard_navigation.launch.py'}
            with patch.object(worker, 'Control', Control), patch.object(worker, 'Processes', Manager), \
                 patch.object(worker, 'dependencies'), contextlib.redirect_stdout(io.StringIO()):
                worker.run_stack(config, path)
        nav = next(argv for name, argv in commands if name == 'nav2')
        self.assertEqual(nav[:3], ['ros2', 'launch', '/onboard_navigation.launch.py'])
        self.assertIn('use_composition:=True', nav)
        self.assertNotIn('rviz2', ' '.join(nav))
        self.assertEqual(commands[-1][0], 'closed')

    def test_arm_bridge_uses_project_cwd_and_validates_carry(self):
        for carrying in (True, False):
            with self.subTest(carrying=carrying), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                (root / 'robot').mkdir()
                (root / 'robot/__init__.py').write_text('')
                (root / 'robot/arm.py').write_text('class ArmStop(RuntimeError): pass\n')
                (root / 'robot/tasks.py').write_text('class ArmTasks:\n    def park(self): pass\n')
                (root / 'robot/park.py').write_text('def near_park(actual,expected): return True\n')
                result = {'ok': True, 'result': 'picked', 'holding': carrying, 'pose': 'carry'}
                (root / 'arm_cli.py').write_text(
                    'import json,sys\ndef main():\n'
                    '    assert sys.argv[1:] == ["pick", "--color", "red"]\n'
                    '    print(' + repr(json.dumps(result)) + ')\n')
                mission = worker.Mission.__new__(worker.Mission)
                mission.config = {'arm_python': sys.executable, 'arm_cli': str(root/'arm_cli.py'),
                                  'arm_timeout': 5}
                mission.log_root = root
                mission.nav = object()
                mission.arm_process = None
                v = SimpleNamespace(linear=SimpleNamespace(x=0, y=0), angular=SimpleNamespace(z=0))
                mission.odom = (SimpleNamespace(twist=SimpleNamespace(twist=v)), time.monotonic())
                mission.rclpy = SimpleNamespace(spin_once=lambda *a, **kw: time.sleep(.01))
                with contextlib.redirect_stdout(io.StringIO()):
                    if carrying:
                        self.assertTrue(mission.arm('pick', 'red')['holding'])
                    else:
                        with self.assertRaisesRegex(RuntimeError, '운반 자세'):
                            mission.arm('pick', 'red')

    def test_laptop_end_to_end_protocol_and_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fake = root / 'remote.py'
            rviz = root / 'rviz.py'
            marker = root / 'rviz-started'
            rviz.write_text('from pathlib import Path\nimport time\n'
                           'Path(' + repr(str(marker)) + ').touch()\ntime.sleep(30)\n')
            fake.write_text('import json,sys,time\nfrom pathlib import Path\n'
                'prefix=' + repr(worker.PREFIX) + '\n'
                'deadline=time.monotonic()+3\n'
                'while not Path(' + repr(str(marker)) + ').exists():\n'
                '    if time.monotonic()>deadline: raise RuntimeError("RViz must start before ready")\n'
                '    time.sleep(.02)\n'
                'print(prefix+json.dumps({"event":"ready"}),flush=True)\n'
                'for line in sys.stdin:\n'
                '    msg=json.loads(line)\n'
                '    if msg["type"]=="mission":\n'
                '        assert msg["request"]["color"]=="red"\n'
                '        print(prefix+json.dumps({"event":"mission_done"}),flush=True)\n'
                '        break\n')
            closed = []
            class FakeSSH:
                def __init__(self, host):
                    pass
                def connect(self):
                    pass
                def run(self, *a, **kw):
                    return 'checked'
                def command(self, *args):
                    return [sys.executable, '-u', str(fake)]
                def close(self):
                    closed.append(True)
            metadata = ({}, Path('image'), {}, Path('rviz'), {'workbench': POINT})
            argv = ['execute_robot.py', '--start', 'current',
                    '--destination', 'workbench', '--color', 'red']
            def fake_shell(argv, domain, model, setup_file=None):
                if argv[:4] == ['ros2', 'run', 'rviz2', 'rviz2']:
                    return [sys.executable, str(rviz)]
                return argv
            with patch.object(sys, 'argv', argv), patch.object(cli, 'SSH', FakeSSH), \
                 patch.object(cli, 'local_metadata', return_value=metadata), \
                 patch.object(cli, 'deploy', return_value=(Path('worker'), Path('config'))), \
                 patch.object(cli, 'shell_command', side_effect=fake_shell), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(cli.main(), 0)
            self.assertEqual(closed, [True])


if __name__ == '__main__':
    unittest.main()
