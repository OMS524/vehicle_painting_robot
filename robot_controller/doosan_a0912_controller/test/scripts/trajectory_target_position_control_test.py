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
DEFAULT_TRAJECTORY_CSV = (
    PROJECT_ROOT
    / "painting_trajectory_planner/test/csv/car_door_small/trajectory_000.csv"
)

COMMAND_TIME_SEC = 0.01
DONE_MARGIN_SEC = 2.0

MAX_JOINT_STEP_DEG = 20.0
MAX_JOINT_VELOCITY_DEG_S = 250.0
MAX_JOINT_ACCELERATION_DEG_S2 = 1500.0


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


def selected_csv_path() -> Path:
    if len(sys.argv) > 1:
        return Path(sys.argv[1]).expanduser().resolve()
    return DEFAULT_TRAJECTORY_CSV.resolve()


def main() -> int:
    csv_path = selected_csv_path()
    if not csv_path.is_file():
        raise FileNotFoundError(csv_path)

    duration = trajectory_duration_sec(csv_path)
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

    print(f"[python] trajectory CSV: {csv_path}")
    print(f"[python] trajectory duration: {duration:.3f}s")
    robot.task_trajectory_csv_position_control(
        csv_path,
        command_time_sec=COMMAND_TIME_SEC,
        max_joint_step_deg=MAX_JOINT_STEP_DEG,
        max_joint_velocity_deg_s=MAX_JOINT_VELOCITY_DEG_S,
        max_joint_acceleration_deg_s2=MAX_JOINT_ACCELERATION_DEG_S2,
    )
    robot.wait_until_motion_done(duration + DONE_MARGIN_SEC)
    print("[python] trajectory done")

    print("[python] robot is running. Press Ctrl+C to servo off and exit.")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
