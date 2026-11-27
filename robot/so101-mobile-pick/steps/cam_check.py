"""카메라 점검: /dev/video* 를 하나씩 열어 사진을 찍고 밝기를 출력 (팔은 안 씀).

python steps/cam_check.py
→ steps/logs/cam_check_video<N>.png, 밝기(평균 0~255)가 10 미만이면 거의 검은 화면
"""
import glob
import sys
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
LOGS = ROOT / "steps/logs"
LOGS.mkdir(parents=True, exist_ok=True)


def by_id_names():
    out = {}
    for p in glob.glob("/dev/v4l/by-id/*"):
        out[str(Path(p).resolve())] = Path(p).name
    return out


def main():
    names = by_id_names()
    devs = sorted(glob.glob("/dev/video*"), key=lambda s: int(s.replace("/dev/video", "") or 0))
    if not devs:
        print("/dev/video* 없음 — 카메라 USB 연결 확인")
        return
    for dev in devs:
        label = names.get(dev, "")
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            print(f"{dev:12s} 열기 실패 {label}")
            continue
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        t0, img, n = time.time(), None, 0
        while time.time() - t0 < 3.0:          # 3초 동안 읽으며 자동 노출이 자리 잡게
            ok, f = cap.read()
            if ok:
                img, n = f, n + 1
        auto = cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
        expo = cap.get(cv2.CAP_PROP_EXPOSURE)
        cap.release()
        if img is None:
            print(f"{dev:12s} 영상 없음 (메타데이터 장치일 가능성) {label}")
            continue
        path = LOGS / f"cam_check_{Path(dev).name}.png"
        cv2.imwrite(str(path), img)
        m = float(img.mean())
        note = "  ← 거의 검은 화면" if m < 10 else ""
        print(f"{dev:12s} {img.shape[1]}x{img.shape[0]} 프레임 {n}개, 밝기 {m:5.1f}, "
              f"자동노출 {auto:g}, 노출 {expo:g} → {path.name}{note} {label}")


if __name__ == "__main__":
    sys.exit(main())
