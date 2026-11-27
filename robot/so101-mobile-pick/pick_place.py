"""Arm-only pick -> calibrated place, using the existing arm_cli unchanged.

Run on the robot with ~/arm-venv/bin/python pick_place.py --color red.
This script does not drive the mobile base or start ROS/Nav2.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import sys


def sequence(run, color='red', place_only=False, report=lambda result: None):
    status = run('status')
    if status.get('ok') is not True:
        raise RuntimeError(f'팔 상태 확인 실패: {status}')
    if not place_only:
        if status.get('holding'):
            raise RuntimeError('이미 블록을 들고 있습니다. --place-only로 내려놓으세요.')
        result = run('pick', color=color)
        report(result)
        if not (result.get('ok') is True and result.get('result') == 'picked' and
                result.get('holding') is True and result.get('pose') == 'carry'):
            raise RuntimeError(f'집기/운반 자세 확인 실패. 내려놓기를 시작하지 않습니다: {result}')
    elif status.get('holding') is not True:
        raise RuntimeError('저장된 상태에 보유 블록이 없습니다. 실제 팔과 state/arm_state.json을 확인하세요.')
    result = run('place')
    report(result)
    if not (result.get('ok') is True and result.get('result') == 'placed' and
            result.get('holding') is False and result.get('pose') == 'park'):
        raise RuntimeError(f'내려놓기/접힘 자세 확인 실패: {result}')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--color', choices=['red','yellow','green','blue','orange','purple'], default='red')
    parser.add_argument('--place-only', action='store_true', help='이미 들고 있는 블록만 내려놓기')
    default = Path(__file__).resolve().parent
    if not (default/'arm_cli.py').is_file():
        default = Path.home()/'so101-mobile-pick'
    parser.add_argument('--project', type=Path, default=default)
    args = parser.parse_args()
    project = args.project.expanduser().resolve()
    if not (project/'arm_cli.py').is_file():
        parser.error(f'arm_cli.py 없음: {project}')
    os.chdir(project)
    sys.path.insert(0, str(project))
    from robot.arm import ArmStop
    import arm_cli
    def stop(signum, frame):
        raise ArmStop('사용자가 pick/place를 중단했습니다.')
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    def report(result):
        print(json.dumps(result, ensure_ascii=False), flush=True)
    try:
        sequence(arm_cli.run, args.color, args.place_only, report)
        report({'cmd':'pick_place', 'ok':True, 'result':'done', 'holding':False, 'pose':'park'})
        return 0
    except Exception as exc:
        report({'cmd':'pick_place', 'ok':False, 'result':'error', 'error':str(exc)})
        return 1


if __name__ == '__main__':
    sys.exit(main())
