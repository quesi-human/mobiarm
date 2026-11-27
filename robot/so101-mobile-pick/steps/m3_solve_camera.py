"""3단계-b: 손목 카메라 픽셀 → 팔 좌표 변환 계산 (v1 s4_solve_camera의 손끝 기준·homography 부분).

  python steps/m3_solve_camera.py steps/logs/m3_<시각>.jsonl [--save] [--exclude "3,7"]

픽셀 (u,v) → 렌즈 왜곡 1항 보정 → 평면 변환 H → 팔 XY (블록 윗면 평면, '보는 자세'에서 찍은 사진 기준)
모델 두 가지를 다 풀고 검증 오차가 작은 쪽을 씀:
  affine     6개 값 (원근 무시) — 점이 적거나 좁은 곳에 몰렸을 때 안정적 (11/24: 11점이 4×4cm에 몰려 homography가 30mm 튐)
  homography 8개 값 (+ 점 12개 이상이면 렌즈 왜곡 1개) — 화면 전체에 고르게 모였을 때 더 정확
로봇 쪽 기준값: 손끝을 블록 윗면 중앙 바로 위에 두었을 때 FK한 손끝 XY.
검증: 한 점씩 빼고 나머지로 계산한 변환이 빠진 점을 얼마나 맞히는지 (leave-one-out, 수평 오차).
저장: calibration/wrist_camera.json (보는 자세 관절값도 함께 — 이 자세로 찍은 사진에만 유효)
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot import mount as M  # noqa: E402

PASS_LOO_MEAN_MM = 4.0
PASS_LOO_MAX_MM = 8.0
MIN_POINTS = 8
MIN_POINTS_K1 = 12
CENTER, FSCALE = np.array([320.0, 240.0]), 640.0


def dlt(src, dst):
    A = []
    for (u, v), (x, y) in zip(src, dst):
        A.append([-u, -v, -1, 0, 0, 0, u * x, v * x, x])
        A.append([0, 0, 0, -u, -v, -1, u * y, v * y, y])
    h = np.linalg.svd(np.array(A))[2][-1]
    return (h / h[-1]).reshape(3, 3)


def fit_affine(px, xy):
    A = np.c_[px, np.ones(len(px))]
    X, *_ = np.linalg.lstsq(A, xy, rcond=None)
    H = np.vstack([X.T, [0.0, 0.0, 1.0]])
    return {"H": H, "k1": 0.0}, (A @ X - xy) * 1000


def fit_model(kind, px, xy, n_k):
    return fit_affine(px, xy) if kind == "affine" else fit_h(px, xy, n_k)


def loo_errors(kind, px, xy, n_k):
    out = []
    for i in range(len(px)):
        keep = [k for k in range(len(px)) if k != i]
        m, _ = fit_model(kind, px[keep], xy[keep], n_k)
        out.append(np.linalg.norm(M.pixel_to_plane(px[i:i + 1], m["H"], m["k1"], CENTER, FSCALE)[0] - xy[i]) * 1000)
    return np.array(out)


def fit_h(px, xy, n_k):
    H0 = dlt(px, xy)

    def unpack(x):
        return np.append(x[:8], 1.0).reshape(3, 3), (x[8] if n_k else 0.0)

    def resid(x):
        H, k1 = unpack(x)
        return (M.pixel_to_plane(px, H, k1, CENTER, FSCALE) - xy).ravel() * 1000

    sol = least_squares(resid, np.r_[H0.ravel()[:8], np.zeros(n_k)], x_scale="jac")
    H, k1 = unpack(sol.x)
    return {"H": H, "k1": float(k1)}, sol.fun.reshape(-1, 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--save", action="store_true", help="합격 시 calibration/wrist_camera.json 저장")
    ap.add_argument("--exclude", default="", help='빼고 풀 점 번호, 예: "3,7"')
    args = ap.parse_args()
    excl = {int(v) for v in args.exclude.replace(",", " ").split()}

    recs = [json.loads(l) for l in Path(args.log).read_text(encoding="utf-8").splitlines() if l.strip()]
    recs = [r for r in recs if r.get("camera") and r["index"] not in excl]
    if excl:
        print(f"[제외] 점 {sorted(excl)}")
    if len(recs) < MIN_POINTS:
        sys.exit(f"카메라 측정이 있는 점이 {len(recs)}개 — 최소 {MIN_POINTS}개 필요")
    px = np.array([r["camera"]["pixel"] for r in recs])
    xy = np.array([r["tip_xy_m"] for r in recs])
    n_k = 1 if len(recs) >= MIN_POINTS_K1 else 0
    print(f"점 {len(recs)}개, 렌즈 왜곡 보정 항 {n_k}개 (점 {MIN_POINTS_K1}개 이상이면 1개)")

    cands = {}
    for kind in ("affine", "homography"):
        cands[kind] = loo_errors(kind, px, xy, n_k)
        print(f"  {kind:10s} 검증 오차 평균 {cands[kind].mean():.1f} mm, 최대 {cands[kind].max():.1f} mm")
    kind = min(cands, key=lambda k: (cands[k].mean() + cands[k].max() / 4))
    print(f"→ {kind} 사용")
    model, res = fit_model(kind, px, xy, n_k)
    err = np.linalg.norm(res, axis=1)
    if kind == "homography":
        print(f"렌즈 왜곡 계수 k1 = {model['k1']:+.3f}")
    print(f"{'#':>3} {'터틀봇 기준(cm)':>16} {'픽셀':>14} | {'dX':>6} {'dY':>6} | {'|e|':>5}  (mm)")
    for r, e in zip(recs, res):
        f, l = r["robot_fl_m"]
        print(f"{r['index']:>3}   (앞 {f*100:4.1f}, 왼 {l*100:5.1f})  ({r['camera']['pixel'][0]:5.0f},{r['camera']['pixel'][1]:4.0f}) "
              f"| {e[0]:6.1f} {e[1]:6.1f} | {np.linalg.norm(e):5.1f}")
    print(f"적합 오차(수평): 평균 {err.mean():.1f} mm, 최대 {err.max():.1f} mm")

    loo = cands[kind]
    span = np.ptp(px, axis=0)
    print(f"\n화면에서 점이 퍼진 범위: 가로 {span[0]:.0f}/640 px, 세로 {span[1]:.0f}/480 px"
          + ("  ← 좁음: 비어 있는 곳에 점을 더 모으면 정확해짐 (--resume)" if span[0] < 320 or span[1] < 240 else ""))
    print("\n=== 검증 (한 점씩 빼고 예측, 수평 오차 mm) ===")
    print("  " + "  ".join(f"{r['index']}:{v:.1f}" for r, v in zip(recs, loo)))
    print(f"  평균 {loo.mean():.1f} mm, 최대 {loo.max():.1f} mm  "
          f"(합격 기준: 평균 ≤ {PASS_LOO_MEAN_MM}, 최대 ≤ {PASS_LOO_MAX_MM})")
    passed = loo.mean() <= PASS_LOO_MEAN_MM and loo.max() <= PASS_LOO_MAX_MM
    print("  → 합격" if passed else "  → 불합격: 오차가 큰 점의 사진(steps/logs/m3_*_p<번호>.png) 확인, --exclude로 빼고 다시")

    if args.save:
        if not passed:
            print("[저장 안 함] 불합격 결과는 저장하지 않습니다.")
            return
        look = json.loads(M.LOOK_FILE.read_text(encoding="utf-8"))
        out = {"model": kind, "H": model["H"].tolist(), "k1": model["k1"], "k2": 0.0, "center_px": CENTER.tolist(), "fscale_px": FSCALE,
               "look_q_deg": look["q_deg"], "forward_heading_deg": M.forward_heading_deg(),
               "plane": "블록 윗면 (작업면 + BLOCK_M)", "block_m": M.BLOCK_M, "color": recs[0].get("color"),
               "fit_mean_mm": float(err.mean()), "loo_mean_mm": float(loo.mean()), "loo_max_mm": float(loo.max()),
               "n_points": len(recs), "source_log": str(args.log), "created": time.strftime("%Y-%m-%d %H:%M:%S"),
               "note": "보는 자세(look_q_deg)에서 찍은 손목 카메라 픽셀 → 팔 XY. 보는 자세·카메라·상자 높이가 바뀌면 다시 수집."}
        path = ROOT / "calibration/wrist_camera.json"
        path.write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"[저장] {path.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
