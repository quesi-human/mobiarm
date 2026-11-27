"""Per-run camera ownership checks and faster UVC startup. No devices on import."""
import json
import os
from pathlib import Path
import re
import signal
import sys
import time


def process_info(pid, proc_root=Path('/proc')):
    root=Path(proc_root)/str(pid)
    try:
        fields=(root/'stat').read_text().rsplit(')',1)[1].split()
        return {'pid':int(pid),'uid':root.stat().st_uid,'ppid':int(fields[1]),
                'state':fields[0],'start':fields[19],
                'cwd':str((root/'cwd').resolve(strict=True)),
                'cmd':(root/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')}
    except (OSError,ValueError,IndexError):
        return None


def camera_owners(device, proc_root=Path('/proc')):
    target=Path(device).resolve(strict=True)
    owners=[]
    for root in Path(proc_root).glob('[0-9]*'):
        try:
            for fd in (root/'fd').iterdir():
                try:
                    if fd.resolve(strict=True)==target:
                        info=process_info(root.name,proc_root)
                        if info:owners.append(info)
                        break
                except OSError:
                    continue
        except OSError:
            continue
    return owners


def is_stale_owner(info, project, uid=None):
    # Never kill another active operator's pick or another application's camera.
    return (info['pid']!=os.getpid() and info['uid']==(os.getuid() if uid is None else uid)
            and info['ppid']==1 and info['state']!='Z'
            and Path(info['cwd']).resolve()==Path(project).resolve()
            and ('arm_cli.py' in info['cmd'] or 'install_park_guard' in info['cmd']))


def stop_stale_owner(info):
    for sig,grace in ((signal.SIGTERM,.7),(signal.SIGKILL,1.)):
        current=process_info(info['pid'])
        if current is None or current['start']!=info['start'] or current['state']=='Z':
            return
        try:
            os.kill(info['pid'],sig)
        except ProcessLookupError:
            return
        deadline=time.monotonic()+grace
        while time.monotonic()<deadline:
            current=process_info(info['pid'])
            if current is None or current['start']!=info['start'] or current['state']=='Z':
                return
            time.sleep(.05)
    raise RuntimeError(f"카메라 잔류 프로세스 {info['pid']} 종료 불가(커널 USB 응답 확인 필요)")


def prepare_camera(device, project):
    try:
        owners=camera_owners(device)
    except OSError as exc:
        raise RuntimeError(f'카메라 장치 없음: {device}') from exc
    print('[카메라 점유 확인] '+json.dumps(owners,ensure_ascii=False),file=sys.stderr,flush=True)
    blockers=[p for p in owners if not is_stale_owner(p,project)]
    if blockers:
        raise RuntimeError('카메라를 다른 실행이 사용 중입니다. 활성 프로세스는 자동 종료하지 않습니다: '
                           +json.dumps(blockers,ensure_ascii=False))
    for owner in owners:
        print(f"[카메라 잔류 정리] PID {owner['pid']}",file=sys.stderr,flush=True)
        stop_stale_owner(owner)
    remaining=camera_owners(device)
    if remaining:
        raise RuntimeError('카메라 점유 해제 확인 실패: '+json.dumps(remaining,ensure_ascii=False))


def fast_start(self):
    import cv2
    cv2.setNumThreads(1)
    src=str(self.index)
    real=os.path.realpath(src) if src.startswith('/') else src
    match=re.fullmatch(r'(?:/dev/video)?(\d+)',real)
    if not match:
        raise RuntimeError(f'카메라 경로 오류: {src} -> {real}')
    device=f'/dev/video{match.group(1)}'
    started=time.monotonic()
    def stage(name):
        print(f'[카메라 단계] {name}: {time.monotonic()-started:.3f}s',file=sys.stderr,flush=True)
    stage('open 시작')
    self.cap=cv2.VideoCapture(device,cv2.CAP_V4L2)
    stage('open 완료')
    try:
        if not self.cap.isOpened():
            raise RuntimeError(f'카메라 열기 실패: {device}')
        width,height,_=self.size
        settings=((cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*self.fourcc)),
                  (cv2.CAP_PROP_FRAME_WIDTH,width),(cv2.CAP_PROP_FRAME_HEIGHT,height))
        for prop,value in settings:
            # Avoid redundant format/size writes: each can trigger UVC probing.
            if self.cap.get(prop)!=value:
                stage(f'설정 {prop} 시작')
                if not self.cap.set(prop,value):
                    raise RuntimeError(f'카메라 영상 설정 실패: {prop}={value}')
        stage('설정 완료')
        for prop,value in settings[1:]:
            if abs(self.cap.get(prop)-value)>.5:
                raise RuntimeError('카메라 해상도 불일치: 보정 해상도를 유지할 수 없습니다.')
        got=0
        warmup=time.monotonic()
        # Keep five actual frames for exposure settling, rather than thirty.
        while time.monotonic()-warmup<3:
            ok,frame=self.cap.read()
            if ok and frame is not None:
                got+=1
                if got==1:stage('첫 프레임')
                if got>=5 and time.monotonic()-warmup>=.15:break
        if got==0:
            raise RuntimeError(f'카메라 영상 수신 없음: {device}')
        stage(f'준비 완료 ({got}프레임)')
        return self
    except BaseException:
        self.cap.release()
        self.cap=None
        raise


def install_fast_capture(camera_class):
    camera_class._start=fast_start
