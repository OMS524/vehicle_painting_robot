#!/usr/bin/env python3
"""새(4cm) planner CSV에 사다리꼴 시간 프로파일(t, path_speed, path_acceleration)을
복원해 trajectory_nlp.py 가 읽는 구 스키마로 저장한다.
목표 TCP 속도 0.42 m/s (2026-09-22 요구, 이전 0.1). planner 의 비분사 리드인/아웃이 0.08 m 라
가속 구간 V^2/(2A) 이 그 안에 들어가야 분사 구간 전체가 0.42 m/s 등속이 된다 → A >= 1.1025, 1.2 로 둔다
(구 planner 값 0.5 면 분사 시작 시 0.28 m/s). 정지 출발/정지 도착.
사용: add_time_profile.py <src_dir> <dst_dir> [V] [A]
"""
import csv, math, sys
from pathlib import Path

V, A = 0.42, 1.2

def trapezoid(S):
    ramp = V * V / (2 * A)
    if S >= 2 * ramp:
        t_ramp = V / A
        t_total = 2 * t_ramp + (S - 2 * ramp) / V
        def at(s):
            if s <= ramp:
                t = math.sqrt(2 * s / A); return t, A * t, A
            if s >= S - ramp:
                sr = S - s
                tr = math.sqrt(2 * sr / A) if sr > 0 else 0.0
                return t_total - tr, A * tr, -A
            return t_ramp + (s - ramp) / V, V, 0.0
    else:  # 삼각 프로파일
        v_peak = math.sqrt(A * S)
        t_half = v_peak / A
        t_total = 2 * t_half
        def at(s):
            if s <= S / 2:
                t = math.sqrt(2 * s / A); return t, A * t, A
            sr = S - s
            tr = math.sqrt(2 * sr / A) if sr > 0 else 0.0
            return t_total - tr, A * tr, -A
    return at, t_total

def convert(src: Path, dst: Path):
    rows = list(csv.DictReader(src.open(newline="", encoding="utf-8-sig")))
    if len(rows) < 2:
        return None
    s0 = float(rows[0]["s"])
    S = float(rows[-1]["s"]) - s0
    at, t_total = trapezoid(S)
    out_fields = ["row_index","point_index","frame_id","t","s",
                  "position_x","position_y","position_z",
                  "orientation_x","orientation_y","orientation_z","orientation_w",
                  "path_speed","path_acceleration","paint"]
    dst.parent.mkdir(parents=True, exist_ok=True)
    with dst.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=out_fields)
        w.writeheader()
        prev_t = -1.0
        for r in rows:
            s = float(r["s"]) - s0
            t, v, a = at(s)
            if t <= prev_t:              # 수치 안전장치 (동일 t 방지)
                t = prev_t + 1e-6
            prev_t = t
            w.writerow({
                "row_index": r["row_index"], "point_index": r["point_index"],
                "frame_id": r["frame_id"], "t": f"{t:.9f}", "s": f"{s:.9f}",
                "position_x": r["position_x"], "position_y": r["position_y"],
                "position_z": r["position_z"],
                "orientation_x": r["orientation_x"], "orientation_y": r["orientation_y"],
                "orientation_z": r["orientation_z"], "orientation_w": r["orientation_w"],
                "path_speed": f"{v:.9f}", "path_acceleration": f"{a:.9f}",
                "paint": r["paint"],
            })
    return len(rows), S, t_total

if __name__ == "__main__":
    src_dir, dst_dir = Path(sys.argv[1]), Path(sys.argv[2])
    if len(sys.argv) > 3:
        V = float(sys.argv[3])
    if len(sys.argv) > 4:
        A = float(sys.argv[4])
    total_pts = total_T = 0.0
    files = sorted(src_dir.glob("trajectory_[0-9]*.csv"))
    for f in files:
        res = convert(f, dst_dir / f.name)
        if res:
            n, S, T = res
            total_pts += n; total_T += T
            print(f"{f.name}: {n} pts, S={S:.3f} m, T={T:.2f} s")
    print(f"TOTAL rows={len(files)} pts={int(total_pts)} exec_time={total_T:.1f} s")
