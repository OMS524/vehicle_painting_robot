#!/usr/bin/env python3

import atexit
import signal
import time

from joint_target_test import (
    DoosanRobot,
    IP,
    MODE,
    PORT,
    RT_IP,
    RT_PORT,
    TIMEOUT_SEC,
    VELOCITY_ACCELERATION,
    VELOCITY_COMMAND_TIME_SEC,
    VELOCITY_DONE_MARGIN_SEC,
    VELOCITY_GAIN,
    VELOCITY_MAX_SPEED,
    VELOCITY_TIMEOUT_SEC,
    VELOCITY_TOLERANCE_DEG,
)


HOME_POSITION = [90.0, 0.0, 90.0, -90.0, 0.0, 0.0]


def main() -> int:
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

    print(f"[python] homing target: {HOME_POSITION}")
    robot.velocity_control_to_position(
        HOME_POSITION,
        max_velocity=VELOCITY_MAX_SPEED,
        acceleration=VELOCITY_ACCELERATION,
        gain=VELOCITY_GAIN,
        tolerance_deg=VELOCITY_TOLERANCE_DEG,
        timeout_sec=VELOCITY_TIMEOUT_SEC,
        command_time_sec=VELOCITY_COMMAND_TIME_SEC,
    )
    robot.wait_until_motion_done(VELOCITY_TIMEOUT_SEC + VELOCITY_DONE_MARGIN_SEC)
    print("[python] homing done")

    print("[python] robot is running. Press Ctrl+C to servo off and exit.")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
