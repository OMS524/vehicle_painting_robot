#!/usr/bin/env python3

import atexit
import csv
import signal
import sys
import time
from pathlib import Path

from joint_target_test import DoosanRobot, IP, MODE, PORT, RT_IP, RT_PORT, TIMEOUT_SEC


ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = ROOT.parents[2]
DEFAULT_TRAJECTORY_CSV = [
    PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_000.csv",
    PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_001.csv",
    PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_002.csv",
    PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_003.csv",
    PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_004.csv",
    PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_005.csv",
    PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_006.csv",
    PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_007.csv",
    PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_008.csv",
    PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_009.csv",
]
DEFAULT_URDF = (
    PROJECT_ROOT
    / "robot_controller/doosan_a0912_controller/urdf/a0912.white.urdf"
)

TCP_FRAME = "link_6"
COMMAND_TIME_SEC = 0.01
DONE_MARGIN_SEC = 2.0

POSITION_GAIN = 8.0
ORIENTATION_GAIN = 4.0
DAMPING = 0.03
MAX_JOINT_STEP_DEG = 20.0
MAX_JOINT_VELOCITY_DEG_S = 70.0
MAX_JOINT_ACCELERATION_DEG_S2 = 500.0


def trajectory_duration_sec(csv_path: Path) -> float:
    with csv_path.open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if "t" not in (reader.fieldnames or []):
            raise ValueError(f"{csv_path} does not contain a t column")

        last_t = None
        first_row_index = None
        for row in reader:
            text = str(row.get("t", "")).strip()
            row_index_text = str(row.get("row_index", "")).strip()
            row_index = int(float(row_index_text)) if row_index_text else None
            if first_row_index is None:
                first_row_index = row_index
            elif row_index is not None and row_index != first_row_index:
                raise ValueError(
                    f"{csv_path} contains multiple row_index values. "
                    "Use one trajectory_XXX.csv file for this test."
                )

            if text:
                last_t = float(text)

    if last_t is None:
        raise ValueError(f"{csv_path} does not contain trajectory rows")
    return max(last_t, 0.0)


def selected_inputs() -> tuple[list[Path], Path]:
    csv_paths = []
    urdf_path = DEFAULT_URDF.resolve()
    args = sys.argv[1:]
    index = 0
    while index < len(args):
        arg = args[index]
        if arg == "--urdf":
            index += 1
            if index >= len(args):
                raise ValueError("--urdf requires a path")
            urdf_path = Path(args[index]).expanduser().resolve()
        else:
            path = Path(arg).expanduser().resolve()
            if path.suffix.lower() == ".urdf":
                urdf_path = path
            else:
                csv_paths.append(path)
        index += 1

    if not csv_paths:
        csv_paths = [path.resolve() for path in DEFAULT_TRAJECTORY_CSV]

    return csv_paths, urdf_path


def main() -> int:
    csv_paths, urdf_path = selected_inputs()
    for csv_path in csv_paths:
        if not csv_path.is_file():
            raise FileNotFoundError(csv_path)
    if not urdf_path.is_file():
        raise FileNotFoundError(urdf_path)

    durations = [trajectory_duration_sec(csv_path) for csv_path in csv_paths]
    robot = DoosanRobot()

    def cleanup() -> None:
        robot.shutdown()

    def handle_signal(signum, _frame) -> None:
        print(f"\n[python] signal {signum} received")
        cleanup()
        raise SystemExit(128 + signum)

    atexit.register(cleanup)
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, handle_signal)

    print("[python] initialize robot")
    robot.initialize(
        ip=IP,
        port=PORT,
        mode=MODE,
        rt_ip=RT_IP,
        rt_port=RT_PORT,
        enable_rt=True,
        timeout_sec=TIMEOUT_SEC,
    )

    print(f"[python] URDF: {urdf_path}")
    for index, (csv_path, duration) in enumerate(zip(csv_paths, durations), start=1):
        print(f"[python] trajectory CSV #{index}/{len(csv_paths)}: {csv_path}")
        print(f"[python] trajectory duration: {duration:.3f}s")
        robot.task_trajectory_csv_velocity_control(
            csv_path,
            urdf_path,
            tcp_frame=TCP_FRAME,
            command_time_sec=COMMAND_TIME_SEC,
            position_gain=POSITION_GAIN,
            orientation_gain=ORIENTATION_GAIN,
            damping=DAMPING,
            max_joint_velocity_deg_s=MAX_JOINT_VELOCITY_DEG_S,
            max_joint_acceleration_deg_s2=MAX_JOINT_ACCELERATION_DEG_S2,
            max_joint_step_deg=MAX_JOINT_STEP_DEG,
        )
        robot.wait_until_motion_done(duration + DONE_MARGIN_SEC)
        print(f"[python] trajectory velocity control #{index} done")

    print("[python] robot is running. Press Ctrl+C to servo off and exit.")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
