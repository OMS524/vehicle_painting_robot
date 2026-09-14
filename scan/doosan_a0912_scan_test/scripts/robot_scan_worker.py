#!/usr/bin/env python3
"""제어 환경 전용 JSON-lines worker. scan SDK/Open3D를 import하지 않는다.

직접 실행 대신 three_view_scan.py를 사용한다. --check는 .so 로딩만 검사한다.
"""
import argparse
import ctypes
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import traceback


def load_wrapper(path):
    spec = importlib.util.spec_from_file_location("scan_doosan_wrapper", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check_library(path):
    library = ctypes.CDLL(str(Path(path).resolve()))
    for name in ("create", "initialize", "read_actual_joint_state",
                 "velocity_control_to_position", "shutdown"):
        getattr(library, "doosan_controller_" + name)
    return {"python": sys.executable, "library": str(path), "hardware_connected": False}


def stationary(state, settings):
    for name in ("joint_deg", "joint_velocity_deg_s"):
        values = state[name]
        if not isinstance(values, list) or len(values) != 6 or not all(
                isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                for v in values):
            raise RuntimeError(f"유효하지 않은 실제 관절 상태: {name}")
    return (not state["motion_running"]
            and all(abs(v) <= settings["stationary_velocity_deg_s"]
                    for v in state["joint_velocity_deg_s"]))


def wait_settled(robot, target, settings):
    """실측 자세의 정지 유지로 촬영 허용. 목표각 차이는 진단 정보로만 기록한다."""
    deadline = time.monotonic() + settings["settle_timeout_sec"]
    stable_since, anchor, state = None, None, None
    while time.monotonic() < deadline:
        state = robot.read_actual_joint_state()
        if stationary(state, settings):
            if anchor is None or max(abs(a - b) for a, b in zip(state["joint_deg"], anchor)) > settings["capture_drift_deg"]:
                stable_since, anchor = time.monotonic(), list(state["joint_deg"])
            if time.monotonic() - stable_since >= settings["settle_sec"]:
                error = [actual - goal for actual, goal in zip(state["joint_deg"], target)]
                return {**state, "settle_criterion": "stationary_actual_joints",
                        "target_joint_deg": list(target), "target_error_deg": error,
                        "max_target_error_deg": max(abs(value) for value in error)}
        else:
            stable_since, anchor = None, None
        time.sleep(0.05)
    raise TimeoutError("실제 관절 속도/자세의 정지 유지 확인 실패. 다음 촬영/이동을 중단합니다. "
                       f"마지막 실제 상태: {json.dumps(state, ensure_ascii=False, allow_nan=False)}")


def move_and_settle(robot, target, settings):
    if not isinstance(target, list) or len(target) != 6 or not all(
            isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in target):
        raise ValueError("joint_deg에는 유한한 J1~J6 값이 필요합니다.")
    state = robot.read_actual_joint_state()
    if not stationary(state, settings):
        raise RuntimeError("이동 시작 전 로봇이 정지 상태가 아닙니다.")
    delta = max(abs(a - b) for a, b in zip(target, state["joint_deg"]))
    if delta > settings["max_move_delta_deg"]:
        raise ValueError(f"이동량 {delta:.3f} deg > max_move_delta_deg. 안전한 시작 자세/목표를 확인하세요.")
    robot.velocity_control_to_position(
        target, max_velocity=[settings["max_velocity_deg_s"]] * 6,
        acceleration=[settings["acceleration_deg_s2"]] * 6, gain=1.0,
        tolerance_deg=settings["target_tolerance_deg"],
        timeout_sec=settings["move_timeout_sec"], command_time_sec=settings["command_time_sec"])
    robot.wait_until_motion_done(settings["move_timeout_sec"] + 5)
    return wait_settled(robot, target, settings)


def serve(wrapper, library, protocol):
    robot, settings = None, None

    def interrupted(signum, _frame):
        raise KeyboardInterrupt(f"control worker signal {signum}")

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupted)
    try:
        for line in sys.stdin:
            request = {}
            try:
                request = json.loads(line)
                op = request["op"]
                if op == "initialize" and robot is None:
                    settings = request["settings"]
                    robot = wrapper.DoosanRobot(library_path=library)
                    robot.initialize(settings["ip"], settings["port"], "real", settings["ip"],
                                     settings["rt_port"], False, settings["connect_timeout_sec"])
                    result = robot.read_actual_joint_state()
                elif op == "close":
                    if robot is not None:
                        robot.shutdown()
                        robot = None
                    protocol.write(json.dumps({"id": request["id"], "result": {"closed": True}}) + "\n")
                    protocol.flush()
                    return
                elif robot is None:
                    raise RuntimeError("initialize가 먼저 필요합니다.")
                elif op == "snapshot":
                    result = robot.read_actual_joint_state()
                elif op == "move":
                    result = move_and_settle(robot, request["joint_deg"], settings)
                else:
                    raise ValueError(f"지원하지 않는 요청: {op}")
                protocol.write(json.dumps({"id": request["id"], "result": result}, allow_nan=False) + "\n")
                protocol.flush()
            except Exception:
                error = traceback.format_exc()
                print(error, file=sys.stderr, flush=True)
                # 오류를 응답하기 전에 기존 제어기의 종료/servo-off 경로를 호출한다.
                if robot is not None:
                    try:
                        robot.shutdown()
                    except Exception:
                        error += "\nShutdown failure:\n" + traceback.format_exc()
                    robot = None
                protocol.write(json.dumps({"id": request.get("id"), "error": error}) + "\n")
                protocol.flush()
                return
    finally:
        if robot is not None:
            robot.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wrapper", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--allow-motion", action="store_true")
    args = parser.parse_args()
    # C++의 std::cout까지 stderr로 분리하여 JSON 프로토콜 오염 방지.
    sys.stdout.flush()
    with os.fdopen(os.dup(sys.stdout.fileno()), "w", buffering=1) as protocol:
        os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
        checked = check_library(args.library)
        wrapper = load_wrapper(args.wrapper)
        if args.check:
            protocol.write(json.dumps(checked) + "\n")
        else:
            serve(wrapper, args.library, protocol)


if __name__ == "__main__":
    main()
