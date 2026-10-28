"""planner의 trajectory CSV와 IK joint CSV를 읽는 헬퍼."""

import csv
import glob
import os

import numpy as np

# painting_trajectory_planner.PAINTING_TRAJECTORY_CSV_HEADER와 맞춘 컬럼 이름
REQUIRED_TRAJECTORY_COLUMNS = (
    "position_x", "position_y", "position_z",
    "orientation_x", "orientation_y", "orientation_z", "orientation_w",
)


class TrajectoryRow:
    """trajectory_XXX.csv 한 개(= row 한 줄)에 해당하는 TCP 경로."""

    def __init__(self, row_index, positions, orientations, times, paint, source):
        self.row_index = int(row_index)
        self.positions = positions          # (N, 3) float64
        self.orientations = orientations    # (N, 4) float64, xyzw
        self.times = times                  # (N,) float64
        self.paint = paint                  # (N,) bool
        self.source = source

    def __len__(self):
        return len(self.positions)


def read_trajectory_csv(path):
    """trajectory CSV 하나를 TrajectoryRow로 읽는다."""
    with open(path, "r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        fieldnames = reader.fieldnames or []
        missing = [name for name in REQUIRED_TRAJECTORY_COLUMNS if name not in fieldnames]
        if missing:
            raise ValueError(f"{path} 에 컬럼이 없다: {', '.join(missing)}")
        rows = list(reader)

    if not rows:
        raise ValueError(f"{path} 에 데이터 행이 없다")

    positions = np.array(
        [[float(r["position_x"]), float(r["position_y"]), float(r["position_z"])] for r in rows]
    )
    orientations = np.array(
        [[float(r["orientation_x"]), float(r["orientation_y"]),
          float(r["orientation_z"]), float(r["orientation_w"])] for r in rows]
    )
    times = np.array([float(r.get("t", 0.0) or 0.0) for r in rows])
    paint = np.array([bool(int(float(r.get("paint", 0) or 0))) for r in rows])
    row_index = int(float(rows[0].get("row_index", 0) or 0))
    return TrajectoryRow(row_index, positions, orientations, times, paint, str(path))


def find_trajectory_csv_files(directory, pattern="trajectory_*.csv"):
    """디렉터리에서 trajectory_XXX.csv 목록을 정렬해 돌려준다.

    전체를 합친 painting_trajectory.csv는 row별 파일과 중복이므로 제외한다.
    """
    files = sorted(glob.glob(os.path.join(str(directory), pattern)))
    return [f for f in files if os.path.basename(f) != "painting_trajectory.csv"]


def read_trajectory_dir(directory, max_rows=0):
    """디렉터리의 trajectory_XXX.csv를 모두 읽는다."""
    files = find_trajectory_csv_files(directory)
    if max_rows:
        files = files[:max_rows]
    return [read_trajectory_csv(path) for path in files]


def read_joint_csv(path):
    """joint CSV를 읽어 (times, joint_names, positions)를 돌려준다.

    두 가지 형식을 모두 받는다.

    1) trajectory_ik_pinocchio.py 형식: t, s, q1..qN, qd.., qdd.. (전부 deg)
       6축 회전 관절 전제라 여기서 라디안으로 바꾼다. 관절 이름은 joint_1.. 로 가정.
    2) solve_viz_ik 형식: t, s, <관절이름들> (SI: 회전 rad, 직동 m)
       갠트리 직동축이 섞이면 deg/m 를 한 단위로 못 쓰므로 이 형식을 쓴다.

    positions 는 항상 SI(rad 또는 m)라 JointState에 그대로 넣으면 된다.
    """
    with open(path, "r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        fieldnames = reader.fieldnames or []
        rows = list(reader)

    if not rows:
        raise ValueError(f"{path} 에 데이터 행이 없다")

    times = np.array([float(r["t"]) for r in rows])

    legacy_names = sorted(
        (name for name in fieldnames if name.startswith("q") and name[1:].isdigit()),
        key=lambda name: int(name[1:]),
    )
    if legacy_names:
        q_deg = np.array([[float(r[name]) for name in legacy_names] for r in rows])
        joint_names = [f"joint_{index}" for index in range(1, len(legacy_names) + 1)]
        return times, joint_names, np.radians(q_deg)

    # trajectory_ik_pinocchio.py 의 8축 출력은 관절 컬럼 뒤에 qd_/qdd_ 미분 컬럼이
    # 따라붙는다. 이걸 관절로 세면 관절 수가 3배가 되므로 접두어로 걸러낸다.
    joint_names = [
        name
        for name in fieldnames
        if name not in ("t", "s") and not name.startswith(("qd_", "qdd_"))
    ]
    if not joint_names:
        raise ValueError(f"{path} 에 관절 컬럼이 없다")
    positions = np.array([[float(r[name]) for name in joint_names] for r in rows])
    return times, joint_names, positions
