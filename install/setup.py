#!/usr/bin/env python3
"""Split laptop/Pi installer. Never starts ROS nodes or opens robot devices."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
ROS = Path('/opt/ros/humble/setup.bash')
PACKAGES = {
    'laptop': ['rviz2', 'turtlebot3_navigation2', 'nav2_rviz_plugins'],
    'robot': ['turtlebot3_bringup', 'coin_d4_driver', 'nav2_bringup',
              'nav2_simple_commander', 'nav2_regulated_pure_pursuit_controller', 'nav2_behaviors'],
}


def run(argv, **kwargs):
    print('+ '+shlex.join(map(str,argv)), flush=True)
    return subprocess.run(list(map(str,argv)), check=True, **kwargs)


def atomic_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
    temp.replace(path)


def extract(archive, destination):
    destination.mkdir(parents=True, exist_ok=True)
    with tarfile.open(archive) as tf:
        for member in tf.getmembers():
            if (member.issym() or member.islnk() or member.isdev() or
                    not (destination/member.name).resolve().is_relative_to(destination.resolve())):
                raise RuntimeError('Unsafe payload entry: '+member.name)
        tf.extractall(destination)


def overlay_for(prefix=None):
    candidates = ([prefix/'ros_ws/install/setup.bash'] if prefix else [])
    candidates += [Path.home()/'turtlebot3_ws/install/setup.bash']
    return next((p for p in candidates if p.is_file()), None)


def ros_shell(argv, overlay=None):
    # Do not depend on another active Python virtualenv or modify .bashrc.
    source = f'source {shlex.quote(str(ROS))}\n'
    if overlay:
        source += f'source {shlex.quote(str(overlay))}\n'
    return ['bash','-c', source+'exec '+shlex.join(map(str,argv))]


def ros_ready(role, overlay=None):
    if not ROS.is_file():
        return False
    code = ('import rclpy,yaml; from ament_index_python.packages import get_package_share_directory as g; '
            f'[g(p) for p in {PACKAGES[role]!r}]; print("ROS packages OK")')
    result = subprocess.run(ros_shell(['/usr/bin/python3','-c',code],overlay),
                            capture_output=True,text=True)
    if result.returncode:
        print(result.stderr[-1500:],flush=True)
    return result.returncode == 0


def ensure_system(role, skip_system, overlay):
    os_release = dict(line.split('=',1) for line in Path('/etc/os-release').read_text().splitlines() if '=' in line)
    if os_release.get('ID','').strip('"') != 'ubuntu' or os_release.get('VERSION_ID','').strip('"') != '22.04':
        raise RuntimeError('This installer targets Ubuntu 22.04 / ROS 2 Humble (amd64 or arm64).')
    if ros_ready(role,overlay) and subprocess.run(
            ['/usr/bin/python3','-c','import venv,ensurepip'],capture_output=True).returncode == 0:
        print('Existing ROS/venv prerequisites are available; no system changes.',flush=True)
        return
    if skip_system:
        if not ROS.is_file():
            raise RuntimeError('--skip-system requested, but ROS Humble is missing.')
        return
    if not ROS.is_file():
        # Official ros-apt-source release for Jammy; no arbitrary shell downloads.
        run(['sudo','apt-get','update'])
        run(['sudo','apt-get','install','-y','ca-certificates','software-properties-common'])
        run(['sudo','add-apt-repository','-y','universe'])
        with urllib.request.urlopen('https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest',timeout=30) as response:
            release=json.load(response)
        assets=[a for a in release['assets'] if a['name'].startswith('ros2-apt-source_') and a['name'].endswith('.jammy_all.deb')]
        if len(assets)!=1:
            raise RuntimeError('Cannot find official ros2-apt-source Jammy asset.')
        with tempfile.TemporaryDirectory(prefix='ros-apt-') as folder:
            package=Path(folder)/assets[0]['name']
            urllib.request.urlretrieve(assets[0]['browser_download_url'],package)
            run(['sudo','dpkg','-i',package])
    packages=[line.strip() for line in (ROOT/role/'ros_packages.txt').read_text().splitlines()
              if line.strip() and not line.lstrip().startswith('#')]
    missing=[]
    for package in packages:
        found=subprocess.run(['dpkg-query','-W','-f=${Status}',package],capture_output=True,text=True)
        if found.stdout.strip()!='install ok installed':
            missing.append(package)
    if missing:
        run(['sudo','apt-get','update'])
        run(['sudo','apt-get','install','-y','--no-install-recommends',*missing])


def create_venv(path, requirements):
    if not (path/'bin/python').is_file():
        run(['/usr/bin/python3','-m','venv','--system-site-packages',path])
    run([path/'bin/python','-m','pip','install','--prefer-binary',
         '--only-binary=numpy,scipy,opencv-python-headless','-r',requirements])
    freeze=subprocess.check_output([path/'bin/python','-m','pip','freeze','--local'],text=True)
    (path.parent/(path.name+'-requirements.lock.txt')).write_text(freeze)


def write_env(path, overlay, domain, model):
    text='# Generated by pjt_soarm_turtlebot/install/setup.py\n'
    text+=f'source {shlex.quote(str(ROS))}\n'
    if overlay:
        text+=f'source {shlex.quote(str(overlay))}\n'
    text+=f'export ROS_DOMAIN_ID={domain}\nexport TURTLEBOT3_MODEL={shlex.quote(model)}\nexport LDS_MODEL=LDS-03\n'
    path.write_text(text)


def copy_project(source, target, backup):
    """Update code in the NEW prefix; never overwrite existing calibration/state."""
    for path in source.rglob('*'):
        relative=path.relative_to(source)
        if any(part in {'.git','__pycache__','state','logs','mission_control','.venv','_archive'} for part in relative.parts):
            continue
        if not path.is_file() or path.is_symlink():
            continue
        destination=target/relative
        if destination.exists():
            if relative.parts[0]=='calibration' or destination.read_bytes()==path.read_bytes():
                continue
            saved=backup/relative
            saved.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(destination,saved)
        destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(path,destination)


def install_laptop(args):
    overlay=overlay_for()
    ensure_system('laptop',args.skip_system,overlay)
    if not ros_ready('laptop',overlay):
        raise RuntimeError('Laptop ROS/RViz/TurtleBot3 packages are not ready.')
    create_venv(ROOT/'.venv',ROOT/'laptop/requirements.txt')
    write_env(ROOT/'laptop/setup_env.sh',overlay,args.domain_id,args.model)
    atomic_json(ROOT/'installation/laptop.json',{'role':'laptop','root':str(ROOT),
        'venv':str(ROOT/'.venv'),'ros_overlay':str(overlay) if overlay else None,'ready':True})
    print('Laptop environment ready. No ROS nodes started.',flush=True)


def install_robot(args):
    prefix=args.prefix.expanduser().resolve()
    # Keep the known-working original project and venv out of installer targets.
    if prefix in {Path.home(),Path('/'),Path.home()/'so101-mobile-pick',Path.home()/'arm-venv'}:
        raise RuntimeError('Choose a separate prefix such as ~/pjt_soarm_turtlebot.')
    prefix.mkdir(parents=True,exist_ok=True)
    overlay=overlay_for(prefix)
    ensure_system('robot',args.skip_system,overlay)
    if args.build_drivers or not ros_ready('robot',overlay):
        archive=ROOT/'robot/ros_ws/source_snapshot.tar.gz'
        if not archive.is_file():
            raise RuntimeError('Bundled TurtleBot3/LDS-03 ROS sources are missing.')
        extract(archive,prefix/'ros_ws')
        run(ros_shell(['colcon','build','--symlink-install','--executor','sequential',
            '--packages-up-to','turtlebot3_bringup','turtlebot3_navigation2','coin_d4_driver'],None),cwd=prefix/'ros_ws')
        overlay=prefix/'ros_ws/install/setup.bash'
    if not ros_ready('robot',overlay):
        raise RuntimeError('Robot ROS/driver/Nav2 packages are not ready.')
    stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    copy_project(ROOT/'robot/so101-mobile-pick',prefix/'so101-mobile-pick',prefix/'backups'/stamp)
    create_venv(prefix/'arm-venv',ROOT/'robot/requirements.txt')
    write_env(prefix/'setup_env.sh',overlay,args.domain_id,args.model)
    code=('import numpy,scipy,cv2,serial,scservo_sdk,yaml; import robot.tasks; '
          'print("Arm imports OK; no devices opened")')
    run([prefix/'arm-venv/bin/python','-c',code],cwd=prefix/'so101-mobile-pick')
    groups=set(subprocess.check_output(['id','-nG'],text=True).split())
    if not {'dialout','video'} <= groups:
        if args.skip_system:
            raise RuntimeError('Missing dialout/video groups. Rerun without --skip-system and log in again.')
        run(['sudo','usermod','-aG','dialout,video',os.environ['USER']])
        raise RuntimeError('dialout/video groups added. Log out/in and rerun setup to finish.')
    report={'role':'robot','ready':True,'remote_project':str(prefix/'so101-mobile-pick'),
            'remote_arm_python':str(prefix/'arm-venv/bin/python'), 'remote_env':str(prefix/'setup_env.sh'),
            'domain_id':args.domain_id,'model':args.model,'ros_overlay':str(overlay)}
    atomic_json(prefix/'installation.json',report)
    print('Robot environment ready. No bringup/Nav2/arm commands executed.',flush=True)
    return report


def make_payload(destination):
    allowed=[ROOT/'install/setup.py',ROOT/'robot/requirements.txt',ROOT/'robot/ros_packages.txt',
             ROOT/'robot/ros_ws/source_snapshot.tar.gz']
    excluded={'.git','__pycache__','state','logs','mission_control','.venv','_archive'}
    with tarfile.open(destination,'w:gz') as tf:
        for path in allowed:
            tf.add(path,arcname=str(path.relative_to(ROOT)))
        for path in (ROOT/'robot/so101-mobile-pick').rglob('*'):
            if path.is_file() and not path.is_symlink() and not (excluded & set(path.relative_to(ROOT/'robot/so101-mobile-pick').parts)):
                tf.add(path,arcname=str(path.relative_to(ROOT)),recursive=False)


def install_all(args):
    install_laptop(args)
    sys.path.insert(0,str(ROOT/'controller'))
    from execute_robot import SSH
    ssh=SSH(args.host)
    try:
        ssh.connect()
        home=ssh.run(['/usr/bin/python3','-c','from pathlib import Path; print(Path.home())']).strip()
        prefix=str(args.remote_prefix or Path(home)/'pjt_soarm_turtlebot')
        stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        stage=str(Path(prefix)/'packages'/stamp)
        with tempfile.TemporaryDirectory(prefix='pjt-payload-') as folder:
            payload=Path(folder)/'robot_payload.tar.gz'
            make_payload(payload)
            data=payload.read_bytes()
            digest=hashlib.sha256(data).hexdigest()
            remote_tar=str(Path(prefix)/'packages'/('payload_'+stamp+'.tar.gz'))
            ssh.upload(remote_tar,data)
            # Archive is generated from regular files only; verify before extraction.
            unpack=('import pathlib,hashlib,tarfile,sys; p=pathlib.Path(sys.argv[1]); '
                    'assert hashlib.sha256(p.read_bytes()).hexdigest()==sys.argv[3]; '
                    'd=pathlib.Path(sys.argv[2]); d.mkdir(parents=True,exist_ok=True); '
                    't=tarfile.open(p); '
                    'assert all(not(m.issym() or m.islnk() or m.isdev()) and '
                    '(d/m.name).resolve().is_relative_to(d.resolve()) for m in t.getmembers()); '
                    't.extractall(d)')
            ssh.run(['/usr/bin/python3','-c',unpack,remote_tar,stage,digest],timeout=120)
        command=['/usr/bin/python3',stage+'/install/setup.py','robot','--prefix',prefix,
                 '--domain-id',str(args.domain_id),'--model',args.model]
        if args.skip_system:
            command+=['--skip-system']
        if args.build_drivers:
            command+=['--build-drivers']
        # TTY supports sudo's own prompt; no password is saved or echoed by us.
        run(ssh.base+['-tt',args.host,shlex.join(command)])
        report=json.loads(ssh.run(['/usr/bin/python3','-c',
            'from pathlib import Path; import sys; print(Path(sys.argv[1]).read_text())',prefix+'/installation.json']))
        report['host']=args.host
        atomic_json(ROOT/'installation/deployment.json',report)
        print('Split installation complete. execute_robot.py now uses the new Pi project.',flush=True)
    finally:
        ssh.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role',choices=['laptop','robot','all'])
    parser.add_argument('--host',default='turtlebot6@192.168.110.176')
    parser.add_argument('--prefix',type=Path,default=Path.home()/'pjt_soarm_turtlebot')
    parser.add_argument('--remote-prefix',type=Path)
    parser.add_argument('--domain-id',type=int,default=36)
    parser.add_argument('--model',default='waffle_pi')
    parser.add_argument('--skip-system',action='store_true',help='Use already-installed ROS/system packages')
    parser.add_argument('--build-drivers',action='store_true',help='Build bundled drivers instead of reusing an installed overlay')
    parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args()
    if not 0<=args.domain_id<=232 or args.host.startswith('-'):
        parser.error('Invalid domain or SSH host')
    if args.remote_prefix and not args.remote_prefix.is_absolute():
        parser.error('--remote-prefix must be an absolute path on the Pi')
    if args.dry_run:
        print(json.dumps({'role':args.role,'root':str(ROOT),'host':args.host,
              'robot_prefix':str(args.prefix) if args.role=='robot' else str(args.remote_prefix or '~/pjt_soarm_turtlebot'),
              'requirements':{'laptop':'laptop/requirements.txt','robot':'robot/requirements.txt'},
              'starts_robot':False},indent=2))
        return 0
    if os.geteuid()==0:
        parser.error('Run as your regular user; installer requests sudo only for system packages.')
    try:
        {'laptop':install_laptop,'robot':install_robot,'all':install_all}[args.role](args)
        return 0
    except (Exception,KeyboardInterrupt) as exc:
        print('Installation incomplete: '+(str(exc) or type(exc).__name__),file=sys.stderr)
        return 1


if __name__=='__main__':
    sys.exit(main())
