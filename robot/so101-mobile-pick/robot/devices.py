"""팔 USB 포트·손목 카메라 기본 경로.

번호(/dev/ttyACM0, /dev/video0)는 꽂는 순서에 따라 바뀜 — 특히 터틀봇 라즈베리파이에선 OpenCR 보드가
ttyACM0을, 파이 카메라가 video0을 먼저 차지할 수 있음. 그래서 장치 고유 이름(/dev/.../by-id/...)을 기본으로 쓰고,
그 이름이 없으면 카메라는 이름(Innomaker)으로 /sys에서 찾고, 그래도 없으면 예전 번호로 돌아감. 환경변수 SO101_PORT / SO101_CAM 으로 바꿀 수 있음.
11/25 노트북에서 확인한 이름.
"""
import os
from pathlib import Path

ARM_PORT_BY_ID = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14030716-if00"
WRIST_CAM_BY_ID = "/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0"


def _pick(env, by_id, fallback):
    if os.environ.get(env):
        return os.environ[env]
    return by_id if Path(by_id).exists() else fallback


def _find_cam_by_name(word="Innomaker"):
    """/dev/v4l/by-id가 없는 시스템(11/25 터틀봇 라즈베리파이)용: 장치 이름에 word가 들어간 카메라의 영상 장치(index 0)."""
    for d in sorted(Path("/sys/class/video4linux").glob("video*"), key=lambda p: int(p.name[5:] or 0)):
        try:
            name = (d / "name").read_text().strip()
            index = int((d / "index").read_text().strip()) if (d / "index").exists() else 0
        except (OSError, ValueError):
            continue
        if word.lower() in name.lower() and index == 0:
            return f"/dev/{d.name}"
    return None


ARM_PORT = _pick("SO101_PORT", ARM_PORT_BY_ID, "/dev/ttyACM0")
WRIST_CAM = _pick("SO101_CAM", WRIST_CAM_BY_ID, _find_cam_by_name() or "/dev/video0")
