#!/usr/bin/env python3
# 실행:
#   conda activate vehicle_painting_robot_scan
#   python -s /home/oms/vehicle_painting_robot/scan/doosan_a0912_scan_test/scripts/live_rgbd_cartesian.py --preview-only
# 하드웨어 없는 검사: --preview-only 대신 --check
# 로봇 이동 포함: --preview-only 대신 --execute (작업 공간 확인 후 MOVE 승인, 좌표 입력).
"""연속 RGB + Depth 미리보기 + 터미널 Cartesian 반복 이동 (두산 SDK IK).

conda activate vehicle_painting_robot_scan
python live_rgbd_cartesian.py --check         # 하드웨어 연결 없음 (기본 동작)
python live_rgbd_cartesian.py --preview-only  # 카메라만 연결
python live_rgbd_cartesian.py --execute       # RGB/Depth를 보며 MOVE 입력 후 로봇 활성화

터미널: MOVE / x,y,z,roll,pitch,yaw / help / q
절대 base_link 좌표, mm/deg, 고정축 RPY = Rz(yaw) @ Ry(pitch) @ Rx(roll).
기본 목표 프레임은 YAML의 cartesian.target_frame(link_6: 플랜지 원점/축).
입력한 숫자 6개 + Enter가 이동 명령이다. 이동 중 입력은 예약하지 않고 버린다.
이동은 SDK IK 목표 관절각까지의 기존 관절 이동이며 MoveL/충돌 회피가 아니다.
"""
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from datetime import datetime
import argparse
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time
import traceback
from zoneinfo import ZoneInfo

import numpy as np

from cartesian_scan import DoosanSDKIK, RPY_CONVENTION, vector6
from multi_view_scan import (DEFAULT_CONFIG, DepthCamera, RobotClient, ScanModel,
                             IKNoSolutionError, color_to_rgb_frame, rgb_image,
                             load_config, save_json, worker_command, worker_environment)

STREAM_STALE_SEC = 2.0
FIRST_FRAME_TIMEOUT_SEC = 10.0
WINDOW = "Live RGB + Depth | Q/ESC: exit | type coordinates in terminal"


class StopRequested(Exception):
    """미완료 제어 요청이 있을 때 context manager의 중단/종료 경로로 진입."""


def parse_pose(text):
    text = text.strip()
    if len(text) > 256:
        raise ValueError("입력이 너무 깁니다. 숫자 6개만 입력하세요.")
    if text.startswith("[") and text.endswith("]"):
        text = text[1:-1].strip()
    # 쉼표를 썼다면 빈 필드도 오류로 취급: 1,,2,... 를 조용히 허용하지 않는다.
    tokens = text.split(",") if "," in text else text.split()
    try:
        values = [float(token.strip()) for token in tokens]
    except ValueError as exc:
        raise ValueError("숫자 6개를 쉼표 또는 공백으로 구분하세요: x,y,z,roll,pitch,yaw") from exc
    return vector6(values, "XYZ/RPY (mm/deg)").tolist()


def command_is_current(stamp, ready_since, pending):
    """작업 완료 전에 입력한 좌표는 완료 후 읽게 되어도 실행하지 않는다."""
    return pending is None and stamp >= ready_since


class TerminalInput:
    def __init__(self, stream=None):
        self.stream = sys.stdin if stream is None else stream
        self.lines = queue.Queue(maxsize=32)
        self.stop = threading.Event()

    def start(self):
        threading.Thread(target=self._read, daemon=True, name="terminal-input").start()

    def _read(self):
        try:
            for line in self.stream:
                text = line.strip()
                if text.lower() in ("q", "quit", "exit"):
                    break
                try:
                    self.lines.put_nowait((time.monotonic(), text))
                except queue.Full:
                    print("입력이 너무 많아 버렸습니다. 한 번에 한 자세만 입력하세요.", flush=True)
        finally:
            self.stop.set()  # q 또는 EOF/터미널 입력 종료


class LatestFrames:
    """SDK 콜백은 스트림별 최신 프레임만 보관. RGB/Depth를 각각 감시한다."""
    def __init__(self):
        self.lock = threading.Lock()
        self.frames = {name: (None, 0.0, 0) for name in ("RGB", "Depth")}
        self.identities = {name: None for name in self.frames}
        self.error = None

    def receive(self, frames):
        try:
            if frames is None:
                return
            with self.lock:
                for name, frame in (("RGB", frames.get_color_frame()), ("Depth", frames.get_depth_frame())):
                    if frame is None:
                        continue
                    identity = (frame.get_index(), frame.get_timestamp_us())
                    if identity != self.identities[name]:
                        self.frames[name] = (frame, time.monotonic(), self.frames[name][2] + 1)
                        self.identities[name] = identity
        except Exception:
            with self.lock:
                self.error = traceback.format_exc()

    def snapshot(self):
        with self.lock:
            if self.error:
                raise RuntimeError("RGB/Depth 콜백 오류:\n" + self.error)
            return dict(self.frames)


def decode_depth(frame, sdk):
    if frame.get_format() != sdk.OBFormat.Y16:
        raise ValueError("Y16 Depth가 아닙니다.")
    raw = np.frombuffer(frame.get_data(), dtype=np.uint16).reshape(frame.get_height(), frame.get_width())
    scale = float(frame.get_depth_scale())
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("잘못된 Depth scale")
    return raw.astype(np.float32) * scale


def depth_image(depth_mm, minimum, maximum, cv):
    gray = np.clip((depth_mm - minimum) / (maximum - minimum) * 255, 0, 255).astype(np.uint8)
    image = cv.applyColorMap(gray, cv.COLORMAP_JET)
    image[depth_mm <= 0] = 0
    return image


def decode_color(frame, sdk, cv):
    return cv.cvtColor(rgb_image(sdk, color_to_rgb_frame(sdk, frame)), cv.COLOR_RGB2BGR)


def preview_image(images, labels, status, footer, fresh, cv):
    """해상도가 다른 원본 RGB/Depth를 왜곡 없이 나란히 표시. 픽셀 정렬은 하지 않는다."""
    width, height, header = 640, 480, 30
    canvas = np.zeros((height + header + 55, width * 2, 3), dtype=np.uint8)
    for index, name in enumerate(("RGB", "Depth")):
        image = images[name]
        h, w = image.shape[:2]
        scale = min(width / w, height / h)
        resized = cv.resize(image, (max(1, round(w * scale)), max(1, round(h * scale))),
                            interpolation=cv.INTER_AREA)
        rh, rw = resized.shape[:2]
        x, y = index * width + (width - rw) // 2, header + (height - rh) // 2
        canvas[y:y + rh, x:x + rw] = resized
        cv.putText(canvas, labels[name], (index * width + 10, 21), cv.FONT_HERSHEY_SIMPLEX,
                   0.48, (0, 255, 255), 1, cv.LINE_AA)
    for index, label in enumerate((status, footer)):
        cv.putText(canvas, label, (10, height + header + 21 + 24 * index), cv.FONT_HERSHEY_SIMPLEX,
                   0.48, (0, 255, 255) if fresh else (0, 0, 255), 1, cv.LINE_AA)
    return canvas


def check_environment(cfg, model, with_robot=True):
    import cv2
    import pyorbbecsdk as sdk

    # import/심볼/툴 TF만 검사. Context/Pipeline/로봇 인스턴스를 만들지 않는다.
    if not hasattr(sdk.Pipeline, "start") or not hasattr(cv2, "imshow"):
        raise RuntimeError("카메라 SDK/OpenCV GUI API가 없습니다.")
    if with_robot:
        for key in ("control_python", "control_wrapper", "control_library"):
            if not Path(cfg[key]).is_file():
                raise FileNotFoundError(f"{key}: {cfg[key]}")
        DoosanSDKIK(model.xml, cfg["cartesian"])
        checked = subprocess.run(worker_command(cfg, "--check"),
            input=json.dumps({"model_xml": model.xml, "cartesian": cfg["cartesian"]}),
            env=worker_environment(cfg["control_python"]), capture_output=True, text=True, timeout=30)
        if checked.returncode:
            raise RuntimeError("제어 환경/SDK IK 검사 실패:\n" + checked.stdout + checked.stderr)
        if json.loads(checked.stdout).get("cartesian_ik_backend") != "doosan_sdk":
            raise RuntimeError("두산 SDK IK worker가 아닙니다.")
    print("환경 검사 통과. 로봇·카메라 연결 없음. 실제 IK 해/충돌 안전을 검사한 것은 아닙니다.")


def print_help(cfg, preview_only):
    print(f"\n기준: base_link -> {cfg['cartesian']['target_frame']} | 위치 mm, 회전 deg | {RPY_CONVENTION}")
    print("목표는 절대 자세입니다. 두산 펜던트의 Euler ZYZ 값을 그대로 RPY에 넣지 마세요.")
    print("RGB(왼쪽) / Depth(오른쪽) 창은 관찰용입니다. 좌표는 이 터미널에 입력하세요.")
    print("원본 영상을 각각 표시하며 RGB↔Depth 픽셀 정렬/시간 동기화는 하지 않습니다.")
    print("입력: x,y,z,roll,pitch,yaw (또는 공백 구분) + Enter / help / q")
    if preview_only:
        print("--preview-only: 로봇에는 연결하지 않습니다. 좌표 입력은 실행되지 않습니다.", flush=True)
    else:
        print("현장 안전·장착·속도·전체 이동 구간을 확인한 뒤 MOVE를 입력하면 제어권/서보를 활성화합니다.")
        print("그 후 숫자 6개 + Enter마다 이동합니다. 충돌 회피/직선 이동 기능은 없습니다.")
        print("IK 해가 없으면 이동하지 않고 입력 대기로 돌아갑니다. 다른 좌표를 입력하세요.")
        print("이동/초기화 중 입력은 예약하지 않고 버립니다. 완료 안내를 본 후 다시 입력하세요.", flush=True)


def run_live(cfg, model, preview_only=False):
    import cv2 as cv

    if not sys.stdin.isatty():
        raise RuntimeError("입력 파일/파이프 실행은 금지합니다. 실제 터미널에서 실행하세요.")
    if sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        raise RuntimeError("RGB/Depth 창을 띄울 GUI 세션이 없습니다.")
    if not preview_only and cfg["mounting_tf_confirmed"] is not True:
        raise ValueError("실제 장착 TF를 확인한 후 mounting_tf_confirmed: true로 설정하세요.")
    output = Path(cfg["output_dir"]) / "live_depth_cartesian" / datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True, exist_ok=False)
    save_json(output / "settings.json", {"config": cfg, "mode": "preview_only" if preview_only else "interactive",
                                         "views_executed": False, "capture_color": True,
                                         "stream_stale_sec": STREAM_STALE_SEC})
    print(f"입력/제어 로그: {output} (RGB/Depth/점군 파일은 저장하지 않음)", flush=True)
    terminal = TerminalInput()
    try:
        with ExitStack() as stack:
            events = stack.enter_context((output / "events.jsonl").open("w", encoding="utf-8", buffering=1))

            def log(event, **data):
                events.write(json.dumps({"at": datetime.now(ZoneInfo("Asia/Seoul")).isoformat(),
                                         "event": event, **data}, ensure_ascii=False, allow_nan=False) + "\n")

            pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="robot-request")
            stack.callback(pool.shutdown, wait=True, cancel_futures=True)
            stack.callback(cv.destroyAllWindows)
            cv.namedWindow(WINDOW, cv.WINDOW_NORMAL)
            cv.resizeWindow(WINDOW, 1280, 565)
            camera_settings = {**cfg["camera"], "capture_color": True}
            camera = DepthCamera(camera_settings)
            # 정렬 없는 미리보기: 한 스트림이 없어도 다른 스트림의 수신/중단 상태를 볼 수 있게 한다.
            camera.pipeline.disable_frame_sync()
            camera.config.set_frame_aggregate_output_mode(camera.sdk.OBFrameAggregateOutputMode.ANY_SITUATION)
            latest = LatestFrames()
            # start 중 실패해도 stop을 시도하며, 이동 중 종료 시 robot cleanup이 먼저 실행된다.
            stack.callback(camera.pipeline.stop)
            camera.pipeline.start(camera.config, latest.receive)
            started = time.monotonic()
            save_json(output / "camera.json", {"device": camera.device_info,
                "width": camera.profile.get_width(), "height": camera.profile.get_height(),
                "fps": camera.profile.get_fps(), "format": "Y16", "temporal_fusion": False,
                "color": {"width": camera.color_profile.get_width(), "height": camera.color_profile.get_height(),
                          "fps": camera.color_profile.get_fps(), "format": str(camera.color_profile.get_format())},
                "pixel_aligned": False, "frame_sync": False})
            print_help(cfg, preview_only)
            terminal.start()
            robot, pending, operation = None, None, None
            ready_since = time.monotonic()
            status = "PREVIEW ONLY" if preview_only else "Type MOVE in terminal to enable robot"
            last_count = {name: -1 for name in ("RGB", "Depth")}
            images = {name: np.zeros((480, 640, 3), dtype=np.uint8) for name in last_count}
            last_fps_count, fps = dict.fromkeys(last_count, 0), dict.fromkeys(last_count, 0.0)
            last_fps_time, center = started, 0.0
            minimum, maximum = cfg["camera"]["min_depth_mm"], cfg["camera"]["max_depth_mm"]
            while True:
                frames = latest.snapshot()
                now = time.monotonic()
                ages = {name: now - (stamp if frame is not None else started)
                        for name, (frame, stamp, _) in frames.items()}
                waiting = [name for name, (frame, _, _) in frames.items()
                           if frame is None or ages[name] >= STREAM_STALE_SEC]
                fresh = not waiting
                for name, (frame, _, count) in frames.items():
                    if count == last_count[name] or frame is None:
                        continue
                    if name == "RGB":
                        images[name] = decode_color(frame, camera.sdk, cv)
                    else:
                        depth_mm = decode_depth(frame, camera.sdk)
                        images[name] = depth_image(depth_mm, minimum, maximum, cv)
                        h, w = depth_mm.shape
                        center = float(depth_mm[h // 2, w // 2])
                        cv.drawMarker(images[name], (w // 2, h // 2), (255, 255, 255), cv.MARKER_CROSS, 18, 1)
                    last_count[name] = count
                if now - last_fps_time >= 1:
                    for name, (_, _, count) in frames.items():
                        fps[name] = (count - last_fps_count[name]) / (now - last_fps_time)
                        last_fps_count[name] = count
                    last_fps_time = now
                labels = {name: f"{name} | {fps[name]:.1f} FPS | age {ages[name]:.2f}s | original / unaligned"
                          for name in frames}
                footer = (f"Depth center {center:.0f} mm (0=invalid) | range {minimum:g}..{maximum:g} mm"
                          if fresh else "WAITING / STALE: " + ", ".join(waiting) + " - motion disabled")
                image = preview_image(images, labels, status, footer, fresh, cv)
                cv.imshow(WINDOW, image)
                key = cv.waitKey(10) & 0xFF
                if terminal.stop.is_set() or key in (27, ord("q")) or cv.getWindowProperty(WINDOW, cv.WND_PROP_VISIBLE) < 1:
                    log("exit_requested", pending=operation)
                    if pending is not None:
                        raise StopRequested("미완료 요청 중 종료: 제어 프로세스에 종료/servo-off를 요청합니다.")
                    return output
                for name, (frame, _, _) in frames.items():
                    timeout = FIRST_FRAME_TIMEOUT_SEC if frame is None else STREAM_STALE_SEC
                    if ages[name] >= timeout:
                        raise RuntimeError(f"새 {name} 수신 중단 ({ages[name]:.2f}초). 관찰 없이 이동하지 않도록 종료합니다.")
                if robot is not None and robot.process.poll() is not None:
                    raise RuntimeError("제어 프로세스가 종료되었습니다. robot_worker.log와 실제 로봇 정지를 확인하세요.")
                if pending is not None and pending.done():
                    try:
                        result = pending.result()
                    except IKNoSolutionError as error:
                        if operation != "move":
                            raise
                        log("move_rejected", reason="ik_no_solution", error=str(error), sdk_status=error.sdk_status)
                        print(f"\nIK 실패: {error} 이동하지 않았습니다.", flush=True)
                        status = "IK FAILED - READY for another pose"
                    else:
                        log(operation + "_complete", result=result)
                        print(f"\n{operation} 완료 / 실제 관절각: {result['joint_deg']}", flush=True)
                        if operation == "move":
                            print(f"목표 관절각과 최대 차이: {result['max_target_error_deg']:.4f} deg (기록용)", flush=True)
                        status = "READY - enter next pose in terminal"
                    print("다음 x,y,z,roll,pitch,yaw 입력 대기 (q: 종료)", flush=True)
                    pending, operation = None, None
                    ready_since = time.monotonic()
                for _ in range(32):
                    try:
                        entered_at, text = terminal.lines.get_nowait()
                    except queue.Empty:
                        break
                    if not text:
                        continue
                    if text.lower() in ("help", "h", "?"):
                        print_help(cfg, preview_only)
                        continue
                    if preview_only:
                        print("카메라 전용 모드입니다. 로봇 이동에는 --execute를 사용하세요.", flush=True)
                        continue
                    if not fresh or not command_is_current(entered_at, ready_since, pending):
                        print("대기/이동 중 입력은 버렸습니다. READY 상태에서 다시 입력하세요.", flush=True)
                        log("input_discarded", text=text)
                        continue
                    if text.upper() == "MOVE":
                        if robot is not None:
                            print("이미 활성화되어 있습니다. 목표 숫자 6개를 입력하세요.", flush=True)
                            continue
                        robot = stack.enter_context(RobotClient(cfg, output))
                        operation, status = "initialize", "INITIALIZING ROBOT - inputs discarded"
                        log("initialize_requested")
                        pending = pool.submit(robot.call, "initialize", settings=cfg["robot"],
                            model_xml=model.xml, cartesian=cfg["cartesian"],
                            timeout=cfg["robot"]["connect_timeout_sec"] + 15)
                        continue
                    if robot is None:
                        print("먼저 현장 안전을 확인한 뒤 MOVE를 입력하세요. 아직 로봇에 연결하지 않았습니다.", flush=True)
                        continue
                    try:
                        pose = parse_pose(text)
                    except ValueError as error:
                        print(f"입력 오류: {error}", flush=True)
                        continue
                    log("move_requested", pose_mm_deg=pose, target_frame=cfg["cartesian"]["target_frame"])
                    print(f"이동 요청: {pose} (base_link 기준 mm/deg)", flush=True)
                    operation, status = "move", "MOVING / SETTLING - inputs discarded"
                    pending = pool.submit(robot.move_cartesian, pose, recover_ik_failure=True)
    except StopRequested as error:
        print(str(error), flush=True)
        return output
    except BaseException:
        (output / "error.log").write_text(traceback.format_exc(), encoding="utf-8")
        raise
    finally:
        terminal.stop.set()
        print("세션 종료. 종료 명령은 물리적 정지를 보장하지 않으므로 현장에서 정지 여부를 확인하세요.", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="환경 검사만 (기본값), 장치 연결 없음")
    mode.add_argument("--execute", action="store_true", help="연속 RGB/Depth + MOVE 승인 후 반복 이동")
    mode.add_argument("--preview-only", action="store_true", help="RGB/Depth만 보기, 로봇 연결 없음")
    args = parser.parse_args()
    cfg = load_config(args.config)
    model = ScanModel(cfg["xacro"])
    check_environment(cfg, model, with_robot=not args.preview_only)
    if args.execute or args.preview_only:
        run_live(cfg, model, args.preview_only)
    return 0


if __name__ == "__main__":
    def interrupted(signum, _frame):
        raise KeyboardInterrupt(f"종료 신호 {signum}")

    signal.signal(signal.SIGTERM, interrupted)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, interrupted)
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("사용자 중단. 실제 로봇 정지를 확인하세요.", file=sys.stderr)
        raise SystemExit(130)
    except Exception:
        traceback.print_exc()
        raise SystemExit(1)
