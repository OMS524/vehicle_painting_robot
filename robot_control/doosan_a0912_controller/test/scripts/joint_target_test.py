#!/usr/bin/env python3

import atexit
import ctypes
import signal
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LIB_PATH = ROOT / "build" / "libdoosan_controller_c_api.so"
Float6 = ctypes.c_float * 6

IP = "192.168.137.100"
PORT = 12345
MODE = "real"
RT_IP = IP
RT_PORT = 12347
TIMEOUT_SEC = 15

CONTROL_MODE = "velocity"  # "position" or "velocity"

TARGET_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
POSITION_MOVE_TIME_SEC = 8.0
POSITION_COMMAND_TIME_SEC = 0.01
POSITION_DONE_MARGIN_SEC = 2.0
POSITION_SEQUENCE = [
    (TARGET_POSITION, POSITION_MOVE_TIME_SEC),
    ([90.0, 0.0, 90.0, -90.0, 0.0, 0.0], 8.0),
    ([132.0, 19.0, 70.0, -87.0, 45.0, 0.0], 8.0),
    ([45.0, 32.0, 55.0, -87.0, -49.0, 0.0], 8.0),
    ([90.0, 0.0, 90.0, -90.0, 0.0, 0.0], 8.0),
]

VELOCITY_MAX_SPEED = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0]
VELOCITY_ACCELERATION = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
VELOCITY_GAIN = 1.0
VELOCITY_TOLERANCE_DEG = 0.5
VELOCITY_TIMEOUT_SEC = 20.0
VELOCITY_COMMAND_TIME_SEC = 0.01
VELOCITY_DONE_MARGIN_SEC = 2.0
MOTION_DWELL_SEC = 1.0
VELOCITY_SEQUENCE = [
    (TARGET_POSITION, VELOCITY_TIMEOUT_SEC),
    ([90.0, 0.0, 90.0, -90.0, 0.0, 0.0], VELOCITY_TIMEOUT_SEC),
    ([132.0, 19.0, 70.0, -87.0, 45.0, 0.0], VELOCITY_TIMEOUT_SEC),
    ([45.0, 32.0, 55.0, -87.0, -49.0, 0.0], VELOCITY_TIMEOUT_SEC),
    ([90.0, 0.0, 90.0, -90.0, 0.0, 0.0], VELOCITY_TIMEOUT_SEC),
]


class DoosanRobot:
    def __init__(self, library_path: str | Path | None = None) -> None:
        library_path = Path(library_path) if library_path is not None else LIB_PATH
        if not library_path.exists():
            raise RuntimeError(
                f"{library_path} not found. Build first with: cmake -S . -B build && cmake --build build"
            )

        self._lib = ctypes.CDLL(str(library_path))
        self._configure_signatures()
        self._handle = self._lib.doosan_controller_create()
        if not self._handle:
            raise RuntimeError("failed to create DoosanController")
        self._closed = False

    def _configure_signatures(self) -> None:
        self._lib.doosan_controller_create.restype = ctypes.c_void_p

        self._lib.doosan_controller_destroy.argtypes = [ctypes.c_void_p]
        self._lib.doosan_controller_destroy.restype = None

        self._lib.doosan_controller_initialize.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_uint,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_uint,
            ctypes.c_int,
            ctypes.c_int,
        ]
        self._lib.doosan_controller_initialize.restype = ctypes.c_int

        self._lib.doosan_controller_position_control.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_float,
        ]
        self._lib.doosan_controller_position_control.restype = ctypes.c_int

        self._lib.doosan_controller_position_control_timed.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_float,
            ctypes.c_float,
        ]
        self._lib.doosan_controller_position_control_timed.restype = ctypes.c_int

        self._lib.doosan_controller_velocity_control_to_position.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
        ]
        self._lib.doosan_controller_velocity_control_to_position.restype = ctypes.c_int

        self._lib.doosan_controller_task_trajectory_csv_position_control.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
        ]
        self._lib.doosan_controller_task_trajectory_csv_position_control.restype = ctypes.c_int

        self._lib.doosan_controller_joint_trajectory_csv_position_control.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
        ]
        self._lib.doosan_controller_joint_trajectory_csv_position_control.restype = ctypes.c_int

        self._lib.doosan_controller_joint_trajectory_csv_velocity_control.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
        ]
        self._lib.doosan_controller_joint_trajectory_csv_velocity_control.restype = ctypes.c_int

        self._lib.doosan_controller_task_trajectory_csv_velocity_control.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
        ]
        self._lib.doosan_controller_task_trajectory_csv_velocity_control.restype = ctypes.c_int

        self._lib.doosan_controller_hold.argtypes = [ctypes.c_void_p]
        self._lib.doosan_controller_hold.restype = None

        self._lib.doosan_controller_is_realtime_control_running.argtypes = [ctypes.c_void_p]
        self._lib.doosan_controller_is_realtime_control_running.restype = ctypes.c_int

        self._lib.doosan_controller_servo_off.argtypes = [ctypes.c_void_p]
        self._lib.doosan_controller_servo_off.restype = ctypes.c_int

        self._lib.doosan_controller_shutdown.argtypes = [ctypes.c_void_p]
        self._lib.doosan_controller_shutdown.restype = None

    def initialize(
        self,
        ip: str,
        port: int,
        mode: str,
        rt_ip: str,
        rt_port: int,
        enable_rt: bool,
        timeout_sec: int,
    ) -> None:
        ok = self._lib.doosan_controller_initialize(
            self._handle,
            ip.encode(),
            port,
            mode.encode(),
            rt_ip.encode(),
            rt_port,
            1 if enable_rt else 0,
            timeout_sec,
        )
        if not ok:
            raise RuntimeError("robot initialize failed")

    def position_control(
        self,
        position: list[float],
        velocity: list[float] | None = None,
        acceleration: list[float] | None = None,
        move_time_sec: float = 8.0,
        command_time_sec: float = 0.015,
    ) -> None:
        if len(position) != 6:
            raise ValueError("position must have 6 elements")

        velocity = velocity if velocity is not None else [0.0] * 6
        acceleration = acceleration if acceleration is not None else [0.0] * 6
        if len(velocity) != 6 or len(acceleration) != 6:
            raise ValueError("velocity and acceleration must have 6 elements")

        ok = self._lib.doosan_controller_position_control_timed(
            self._handle,
            Float6(*position),
            Float6(*velocity),
            Float6(*acceleration),
            ctypes.c_float(move_time_sec),
            ctypes.c_float(command_time_sec),
        )
        if not ok:
            raise RuntimeError("position control command failed")

    def velocity_control_to_position(
        self,
        target_position: list[float],
        max_velocity: list[float] | None = None,
        acceleration: list[float] | None = None,
        gain: float = 1.0,
        tolerance_deg: float = 0.5,
        timeout_sec: float = 20.0,
        command_time_sec: float = 0.015,
    ) -> None:
        if len(target_position) != 6:
            raise ValueError("target_position must have 6 elements")

        max_velocity = max_velocity if max_velocity is not None else VELOCITY_MAX_SPEED
        acceleration = acceleration if acceleration is not None else VELOCITY_ACCELERATION
        if len(max_velocity) != 6 or len(acceleration) != 6:
            raise ValueError("max_velocity and acceleration must have 6 elements")

        ok = self._lib.doosan_controller_velocity_control_to_position(
            self._handle,
            Float6(*target_position),
            Float6(*max_velocity),
            Float6(*acceleration),
            ctypes.c_float(gain),
            ctypes.c_float(tolerance_deg),
            ctypes.c_float(timeout_sec),
            ctypes.c_float(command_time_sec),
        )
        if not ok:
            raise RuntimeError("velocity control command failed")

    def task_trajectory_csv_position_control(
        self,
        csv_path: str | Path,
        command_time_sec: float = 0.001,
        max_joint_step_deg: float = 20.0,
        max_joint_velocity_deg_s: float = 250.0,
        max_joint_acceleration_deg_s2: float = 1500.0,
    ) -> None:
        path = Path(csv_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(path)

        ok = self._lib.doosan_controller_task_trajectory_csv_position_control(
            self._handle,
            str(path).encode(),
            ctypes.c_float(command_time_sec),
            ctypes.c_float(max_joint_step_deg),
            ctypes.c_float(max_joint_velocity_deg_s),
            ctypes.c_float(max_joint_acceleration_deg_s2),
        )
        if not ok:
            raise RuntimeError("task trajectory CSV position control failed")

    def joint_trajectory_csv_position_control(
        self,
        csv_path: str | Path,
        command_time_sec: float = 0.001,
        max_joint_step_deg: float = 20.0,
        max_joint_velocity_deg_s: float = 250.0,
        max_joint_acceleration_deg_s2: float = 1500.0,
    ) -> None:
        path = Path(csv_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(path)

        ok = self._lib.doosan_controller_joint_trajectory_csv_position_control(
            self._handle,
            str(path).encode(),
            ctypes.c_float(command_time_sec),
            ctypes.c_float(max_joint_step_deg),
            ctypes.c_float(max_joint_velocity_deg_s),
            ctypes.c_float(max_joint_acceleration_deg_s2),
        )
        if not ok:
            raise RuntimeError("joint trajectory CSV position control failed")

    def joint_trajectory_csv_velocity_control(
        self,
        csv_path: str | Path,
        command_time_sec: float = 0.001,
        position_gain: float = 4.0,
        max_joint_step_deg: float = 20.0,
        max_joint_velocity_deg_s: float = 70.0,
        max_joint_acceleration_deg_s2: float = 500.0,
        preposition_max_joint_velocity_deg_s: float = 20.0,
        preposition_max_joint_acceleration_deg_s2: float = 150.0,
    ) -> None:
        path = Path(csv_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(path)

        ok = self._lib.doosan_controller_joint_trajectory_csv_velocity_control(
            self._handle,
            str(path).encode(),
            ctypes.c_float(command_time_sec),
            ctypes.c_float(position_gain),
            ctypes.c_float(max_joint_step_deg),
            ctypes.c_float(max_joint_velocity_deg_s),
            ctypes.c_float(max_joint_acceleration_deg_s2),
            ctypes.c_float(preposition_max_joint_velocity_deg_s),
            ctypes.c_float(preposition_max_joint_acceleration_deg_s2),
        )
        if not ok:
            raise RuntimeError("joint trajectory CSV velocity control failed")

    def task_trajectory_csv_velocity_control(
        self,
        csv_path: str | Path,
        urdf_path: str | Path,
        tcp_frame: str = "link_6",
        command_time_sec: float = 0.001,
        position_gain: float = 8.0,
        orientation_gain: float = 4.0,
        damping: float = 0.03,
        max_joint_velocity_deg_s: float = 70.0,
        max_joint_acceleration_deg_s2: float = 500.0,
        max_joint_step_deg: float = 20.0,
    ) -> None:
        csv = Path(csv_path).expanduser().resolve()
        urdf = Path(urdf_path).expanduser().resolve()
        if not csv.is_file():
            raise FileNotFoundError(csv)
        if not urdf.is_file():
            raise FileNotFoundError(urdf)

        ok = self._lib.doosan_controller_task_trajectory_csv_velocity_control(
            self._handle,
            str(csv).encode(),
            str(urdf).encode(),
            tcp_frame.encode(),
            ctypes.c_float(command_time_sec),
            ctypes.c_float(position_gain),
            ctypes.c_float(orientation_gain),
            ctypes.c_float(damping),
            ctypes.c_float(max_joint_velocity_deg_s),
            ctypes.c_float(max_joint_acceleration_deg_s2),
            ctypes.c_float(max_joint_step_deg),
        )
        if not ok:
            raise RuntimeError("task trajectory CSV velocity control failed")

    def read_actual_joint_state(self) -> dict:
        """Actual controller measurements; no connection or motion is started here."""
        if self._closed or not self._handle:
            raise RuntimeError("robot is closed")
        try:
            read = self._lib.doosan_controller_read_actual_joint_state
        except AttributeError as exc:
            raise RuntimeError("Rebuild the control library to enable scan telemetry") from exc
        read.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
        read.restype = ctypes.c_int
        position, velocity = Float6(), Float6()
        started = time.time_ns()
        if not read(self._handle, position, velocity):
            raise RuntimeError("actual joint position/velocity measurement unavailable")
        return {
            "joint_deg": list(position), "joint_velocity_deg_s": list(velocity),
            "host_query_start_ns": started, "host_query_end_ns": time.time_ns(),
            "motion_running": self.is_motion_running(),
        }

    def hold(self) -> None:
        self._lib.doosan_controller_hold(self._handle)

    def is_motion_running(self) -> bool:
        return bool(self._lib.doosan_controller_is_realtime_control_running(self._handle))

    def wait_until_motion_done(self, timeout_sec: float, poll_sec: float = 0.05) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if not self.is_motion_running():
                return
            time.sleep(poll_sec)
        raise TimeoutError("position control did not finish before timeout")

    def shutdown(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._handle:
            print("[python] shutting down robot: servo off + close connection")
            self._lib.doosan_controller_shutdown(self._handle)
            self._lib.doosan_controller_destroy(self._handle)
            self._handle = None


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

    if CONTROL_MODE == "position":
        for index, (target_position, move_time_sec) in enumerate(POSITION_SEQUENCE, start=1):
            print(f"[python] position control #{index} target: {target_position}")
            robot.position_control(
                target_position,
                move_time_sec=move_time_sec,
                command_time_sec=POSITION_COMMAND_TIME_SEC,
            )
            robot.wait_until_motion_done(move_time_sec + POSITION_DONE_MARGIN_SEC)
            print(f"[python] position control #{index} done")
            time.sleep(MOTION_DWELL_SEC)
    elif CONTROL_MODE == "velocity":
        for index, (target_position, timeout_sec) in enumerate(VELOCITY_SEQUENCE, start=1):
            print(f"[python] velocity control #{index} target: {target_position}")
            robot.velocity_control_to_position(
                target_position,
                max_velocity=VELOCITY_MAX_SPEED,
                acceleration=VELOCITY_ACCELERATION,
                gain=VELOCITY_GAIN,
                tolerance_deg=VELOCITY_TOLERANCE_DEG,
                timeout_sec=timeout_sec,
                command_time_sec=VELOCITY_COMMAND_TIME_SEC,
            )
            robot.wait_until_motion_done(timeout_sec + VELOCITY_DONE_MARGIN_SEC)
            print(f"[python] velocity control #{index} done")
            time.sleep(MOTION_DWELL_SEC)
    else:
        raise ValueError(f"unknown CONTROL_MODE: {CONTROL_MODE}")

    print("[python] robot is running. Press Ctrl+C to servo off and exit.")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
