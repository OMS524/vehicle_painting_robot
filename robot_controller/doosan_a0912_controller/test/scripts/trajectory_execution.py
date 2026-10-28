"""Existing six-axis execution backend extracted from trajectory_nlp.py; import only."""
import argparse
import atexit
import csv
from pathlib import Path
import signal
import time

DONE_MARGIN_SEC = 2.0

def trajectory_duration_sec(csv_path: Path) -> float:
    with csv_path.open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        last_t = None
        for row in reader:
            value = str(row.get("t", "")).strip()
            if value:
                last_t = float(value)
    if last_t is None:
        raise ValueError(f"{csv_path} does not contain trajectory rows")
    return max(last_t, 0.0)


def run_robot(outputs: list[Path], args: argparse.Namespace) -> None:
    from joint_target_test import DoosanRobot, IP, MODE, PORT, RT_IP, RT_PORT, TIMEOUT_SEC

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

    for index, csv_path in enumerate(outputs, start=1):
        duration = trajectory_duration_sec(csv_path)
        print(f"[python] optimized joint CSV #{index}/{len(outputs)}: {csv_path}")
        print(f"[python] trajectory duration: {duration:.3f}s")
        robot.joint_trajectory_csv_velocity_control(
            csv_path,
            command_time_sec=args.command_time_sec,
            position_gain=args.joint_position_gain,
            max_joint_step_deg=args.max_joint_step_deg,
            max_joint_velocity_deg_s=args.max_joint_velocity_deg_s,
            max_joint_acceleration_deg_s2=args.max_joint_acceleration_deg_s2,
            preposition_max_joint_velocity_deg_s=args.preposition_max_joint_velocity_deg_s,
            preposition_max_joint_acceleration_deg_s2=args.preposition_max_joint_acceleration_deg_s2,
        )
        robot.wait_until_motion_done(duration + DONE_MARGIN_SEC)
        print(f"[python] optimized joint trajectory #{index} done")

    print("[python] robot is running. Press Ctrl+C to servo off and exit.")
    while True:
        time.sleep(1.0)

