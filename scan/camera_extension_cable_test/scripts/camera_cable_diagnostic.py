#!/usr/bin/env python3
"""Femto Bolt USB 연장선 비교: 독립 RGB/Depth 수신 통계, 선택적 미리보기.

로봇 제어, 정합, 영상 전체 저장, SDK/장치 설정 파일 변경 없음.
센서 콜백에서는 메타데이터만 큐에 넣고 영상 변환/파일 저장은 하지 않는다.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, dataclass
from datetime import datetime
from importlib import metadata
import json
import math
import os
from pathlib import Path
import platform
import queue
import re
import subprocess
import sys
import threading
import time
import traceback
from zoneinfo import ZoneInfo

# ======================== 기본 설정 ========================
CABLE_LABEL = "unnamed"  # direct / cable_a / cable_b 등으로 구분
DURATION_SEC = 600.0  # 준비 시간 제외한 실제 측정 시간
WARMUP_SEC = 5.0
OUTPUT_ROOT = Path(__file__).resolve().parents[1] / "log"
COLOR_WIDTH, COLOR_HEIGHT, COLOR_FORMAT = 1920, 1080, "MJPG"
DEPTH_WIDTH, DEPTH_HEIGHT = 640, 576
FPS = 30
TIMEOUT_SEC = 2.0  # 새 고유 프레임이 이 시간 동안 없으면 무수신 사건 1회
PREVIEW_FPS = 5.0  # 화면 갱신만 제한. 수신 계측은 모든 프레임 대상.
# ===========================================================


def now_iso():
    return datetime.now(ZoneInfo("Asia/Seoul")).isoformat()


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


@dataclass(frozen=True)
class Settings:
    label: str = CABLE_LABEL
    note: str = ""  # 케이블 제품명/길이/전원 방식 등 자유 기록
    seconds: float = DURATION_SEC
    warmup: float = WARMUP_SEC
    timeout: float = TIMEOUT_SEC
    serial: str = ""
    color_width: int = COLOR_WIDTH
    color_height: int = COLOR_HEIGHT
    color_format: str = COLOR_FORMAT
    depth_width: int = DEPTH_WIDTH
    depth_height: int = DEPTH_HEIGHT
    fps: int = FPS
    visualize: bool = False
    preview_fps: float = PREVIEW_FPS


def validate(s):
    if not re.fullmatch(r"[\w-]{1,64}", s.label):
        raise ValueError("label은 1~64자 문자/숫자/밑줄/하이픈만 가능합니다.")
    for value in (s.seconds, s.timeout, s.preview_fps):
        if not math.isfinite(value) or value <= 0:
            raise ValueError("seconds, timeout, preview-fps는 유한한 양수여야 합니다.")
    if not math.isfinite(s.warmup) or s.warmup < 0:
        raise ValueError("warmup은 0 이상의 유한한 값이어야 합니다.")
    for value in (s.color_width, s.color_height, s.depth_width, s.depth_height, s.fps):
        if type(value) is not int or value <= 0:
            raise ValueError("해상도와 FPS는 양의 정수여야 합니다. 자동 프로파일 선택은 하지 않습니다.")
    if s.color_format not in ("MJPG", "RGB", "YUYV", "NV12"):
        raise ValueError("지원 컬러 포맷: MJPG / RGB / YUYV / NV12")


class StreamStats:
    """측정 구간 내 수신 메타데이터만 분석. 예상 누락은 케이블 패킷 손실 확정값이 아님."""
    def __init__(self, fps, start, timeout):
        self.fps, self.start, self.timeout = fps, start, timeout
        self.counts = Counter()
        self.last = None
        self.stalled = False
        self.max_silence = self.max_device_gap_ms = self.max_arrival_gap_ms = 0.0
        self.sum_arrival_gap_ms = 0.0

    def silence(self, now):
        last = self.last["host_s"] if self.last is not None else self.start
        gap = max(0.0, now - last)
        self.max_silence = max(self.max_silence, gap)
        if gap >= self.timeout and not self.stalled:
            self.stalled = True
            self.counts["timeout_episodes"] += 1
            return True
        return False

    def add(self, frame):
        flags = []
        self.counts["received"] += 1
        if frame["bytes"] <= 0:
            self.counts["empty_payloads"] += 1
            flags.append("empty_payload")
        previous = self.last
        if previous is not None and (frame["index"], frame["device_us"]) == (previous["index"], previous["device_us"]):
            self.counts["duplicates"] += 1
            return [*flags, "duplicate"]
        if self.silence(frame["host_s"]):
            flags.append("timeout")
        self.stalled = False
        self.counts["unique"] += 1
        if previous is not None:
            index_delta = frame["index"] - previous["index"]
            device_delta = frame["device_us"] - previous["device_us"]
            arrival_ms = (frame["host_s"] - previous["host_s"]) * 1000
            frame.update(index_delta=index_delta, device_gap_ms=device_delta / 1000, arrival_gap_ms=arrival_ms)
            # 시각/번호가 돌아가면 새 구간으로 처리. 거대한 누락 수로 해석하지 않음.
            if index_delta < 0 or device_delta < 0:
                self.counts["resets_or_reorders"] += 1
                flags.append("reset_or_reorder")
            else:
                if index_delta > 1:
                    self.counts["index_gap_events"] += 1
                    self.counts["suspected_missing_by_index"] += index_delta - 1
                    flags.append("index_gap")
                if index_delta == 0:
                    self.counts["repeated_index"] += 1
                    flags.append("repeated_index")
                if device_delta == 0:
                    self.counts["repeated_device_timestamp"] += 1
                    flags.append("repeated_device_timestamp")
                self.max_device_gap_ms = max(self.max_device_gap_ms, device_delta / 1000)
                if device_delta > 1.5 * 1_000_000 / self.fps:
                    self.counts["device_gap_events"] += 1
                    flags.append("device_gap")
            if arrival_ms >= 0:
                self.counts["arrival_intervals"] += 1
                self.sum_arrival_gap_ms += arrival_ms
                self.max_arrival_gap_ms = max(self.max_arrival_gap_ms, arrival_ms)
                if arrival_ms > max(100.0, 2.5 * 1000 / self.fps):
                    self.counts["host_gap_events"] += 1
                    flags.append("host_gap")
            else:
                self.counts["host_reorders"] += 1
                flags.append("host_reorder")
        self.last = dict(frame)
        return flags

    def summary(self, end):
        self.silence(end)
        elapsed = max(0.0, end - self.start)
        keys = ("received", "unique", "duplicates", "index_gap_events", "suspected_missing_by_index",
                "resets_or_reorders", "repeated_index", "repeated_device_timestamp", "device_gap_events",
                "host_gap_events", "host_reorders", "timeout_episodes", "empty_payloads")
        result = {key: self.counts[key] for key in keys}
        result.update(measured_seconds=elapsed, configured_fps=self.fps,
                      received_fps=self.counts["unique"] / elapsed if elapsed else None,
                      max_no_unique_frame_ms=self.max_silence * 1000,
                      max_device_gap_ms=self.max_device_gap_ms, max_arrival_gap_ms=self.max_arrival_gap_ms,
                      mean_arrival_gap_ms=(self.sum_arrival_gap_ms / self.counts["arrival_intervals"]
                                           if self.counts["arrival_intervals"] else None))
        return result


class Channel:
    def __init__(self, preview=False, capacity=20000):
        self.queue = queue.Queue(maxsize=capacity)
        self.preview = preview
        self.latest = {}
        self.lock = threading.Lock()
        self.overflow = 0
        self.accepting = True

    def put(self, event):
        if not self.accepting:
            return
        try:
            self.queue.put_nowait(event)
        except queue.Full:
            with self.lock:
                self.overflow += 1

    def callback(self, name):
        def receive(frame):
            stamp = time.monotonic()  # 장치 시계와 빼서 절대 지연으로 해석하지 않음
            if not self.accepting:
                return
            try:
                video = frame.as_video_frame()
                self.put({"type": "frame", "stream": name, "host_s": stamp,
                          "index": int(frame.get_index()), "device_us": int(frame.get_timestamp_us()),
                          "bytes": int(frame.get_data_size()), "width": video.get_width(),
                          "height": video.get_height(), "format": str(frame.get_format()).split(".")[-1]})
                if self.preview:
                    with self.lock:
                        self.latest[name] = (stamp, frame)  # 스트림마다 최신 프레임 1개만 유지
            except Exception:
                self.put({"type": "callback_error", "stream": name, "host_s": stamp,
                          "message": traceback.format_exc()})
        return receive


def profile_info(profile):
    p = profile.as_video_stream_profile()
    return {"width": p.get_width(), "height": p.get_height(), "fps": p.get_fps(),
            "format": str(p.get_format()).split(".")[-1]}


def select_device(sdk, context, serial):
    devices = context.query_devices()
    if serial:
        device = devices.get_device_by_serial_number(serial)
    elif devices.get_count() == 1:
        device = devices.get_device_by_index(0)
    else:
        raise RuntimeError(f"카메라 {devices.get_count()}대 발견. 한 대만 연결하거나 --serial을 지정하세요.")
    if "femto bolt" not in device.get_device_info().get_name().lower():
        raise ValueError("이 진단의 기본 설정은 Femto Bolt용입니다. 선택된 카메라 모델을 확인하세요.")
    return device


def get_profiles(sdk, device, s):
    requests = {"color": (sdk.OBSensorType.COLOR_SENSOR, s.color_width, s.color_height, s.color_format),
                "depth": (sdk.OBSensorType.DEPTH_SENSOR, s.depth_width, s.depth_height, "Y16")}
    selected = {}
    for name, (sensor_type, width, height, fmt) in requests.items():
        sensor = device.get_sensor(sensor_type)
        profiles = sensor.get_stream_profile_list()
        try:
            profile = profiles.get_video_stream_profile(width, height, getattr(sdk.OBFormat, fmt), s.fps)
        except Exception as error:
            raise RuntimeError(f"{name}: {width}x{height} {fmt} {s.fps} FPS 미지원. "
                               "--list-profiles로 확인하세요. 낮은 설정으로 자동 변경하지 않습니다.") from error
        expected = {"width": width, "height": height, "fps": s.fps, "format": fmt}
        if profile_info(profile) != expected:
            raise RuntimeError(f"요청한 {name} 프로파일과 SDK 반환값이 다릅니다: {profile_info(profile)}")
        selected[name] = (sensor, profile)
    return selected


class Preview:
    """관찰용 5 FPS 화면. 이 화면의 FPS는 수신 FPS가 아니다."""
    def __init__(self, timeout):
        if sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            raise RuntimeError("GUI 화면이 없습니다. --visualize 없이 실행하세요.")
        import cv2
        import numpy as np
        self.cv, self.np = cv2, np
        self.timeout = timeout
        self.last = {}
        self.images = {}
        self.frames_checked = Counter()
        self.decode_errors = Counter()
        self.window = "Femto Bolt cable diagnostic | Q/ESC: stop"
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window, 1280, 480)

    def update(self, channel, current):
        cv, np = self.cv, self.np
        with channel.lock:
            latest = dict(channel.latest)
        errors = []
        for name, (stamp, frame) in latest.items():
            if self.last.get(name) == stamp:
                continue
            self.last[name] = stamp
            self.frames_checked[name] += 1
            try:
                v = frame.as_video_frame()
                width, height = v.get_width(), v.get_height()
                raw = np.frombuffer(frame.get_data(), dtype=np.uint8)
                fmt = str(frame.get_format()).split(".")[-1]
                if name == "depth":
                    depth = raw.view(np.uint16).reshape(height, width)
                    depth_mm = depth.astype(np.float32) * frame.as_depth_frame().get_depth_scale()
                    gray = np.clip(depth_mm / 3000 * 255, 0, 255).astype(np.uint8)
                    image = cv.applyColorMap(gray, cv.COLORMAP_JET)
                    image[depth == 0] = 0
                elif fmt == "MJPG":
                    image = cv.imdecode(raw, cv.IMREAD_COLOR)
                elif fmt == "RGB":
                    image = cv.cvtColor(raw.reshape(height, width, 3), cv.COLOR_RGB2BGR)
                elif fmt == "YUYV":
                    image = cv.cvtColor(raw.reshape(height, width, 2), cv.COLOR_YUV2BGR_YUY2)
                else:
                    image = cv.cvtColor(raw.reshape(height * 3 // 2, width), cv.COLOR_YUV2BGR_NV12)
                if image is None or image.shape[:2] != (height, width):
                    raise ValueError("영상 디코딩 실패/해상도 불일치")
                self.images[name] = cv.resize(image, (640, 480))
            except Exception as error:
                self.decode_errors[name] += 1
                errors.append({"type": "preview_decode_error", "stream": name, "host_s": current,
                               "message": str(error)})
        panels = []
        for name in ("color", "depth"):
            panel = self.images.get(name, np.zeros((480, 640, 3), dtype=np.uint8)).copy()
            age = current - latest[name][0] if name in latest else None
            label = f"{name} | age {age:.2f}s" if age is not None else f"{name} | NO FRAME"
            cv.putText(panel, label, (12, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            if age is None or age >= self.timeout:
                cv.putText(panel, "STALE / NO DATA", (12, 65), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            panels.append(panel)
        cv.imshow(self.window, np.hstack(panels))
        stop = cv.waitKey(1) & 0xFF in (27, ord("q"))
        stop = stop or cv.getWindowProperty(self.window, cv.WND_PROP_VISIBLE) < 1
        return stop, errors

    def close(self):
        self.cv.destroyAllWindows()


def usb_tree():
    try:
        p = subprocess.run(["lsusb", "-t"], capture_output=True, text=True, timeout=3, check=False)
        return p.stdout + p.stderr
    except (OSError, subprocess.TimeoutExpired) as error:
        return f"USB 토폴로지 조회 불가: {error}\n"


def assessment(report):
    reasons = []
    if report["queue_overflow"]:
        reasons.append("진단 프로그램 로그 큐가 넘쳤습니다. 계측 누락이 있어 케이블 비교에 사용할 수 없습니다.")
    if report["status"] != "completed" or report["queue_overflow"]:
        return "inconclusive", [*reasons, "설정된 전체 구간의 정상 계측 완료가 아닙니다."]
    if re.search(r"usb\s*2", report.get("device", {}).get("connection", ""), re.IGNORECASE):
        reasons.append("SDK가 USB 2 연결로 보고했습니다. USB 3 링크/케이블/포트를 확인하세요.")
    for name, stats in report["streams"].items():
        if not stats["unique"] or stats["received_fps"] < 0.95 * stats["configured_fps"]:
            reasons.append(f"{name}: 평균 수신 FPS가 설정값의 95% 미만입니다.")
        for key in ("duplicates", "suspected_missing_by_index", "resets_or_reorders", "repeated_index",
                    "repeated_device_timestamp", "host_gap_events", "host_reorders", "device_gap_events",
                    "timeout_episodes", "empty_payloads"):
            if stats[key]:
                reasons.append(f"{name}: {key}={stats[key]}")
    for kind, count in report["events"].items():
        if count:
            reasons.append(f"{kind}: {count}회 (준비/측정/종료 시점은 events.jsonl에서 확인)")
    return ("review_required" if reasons else "no_anomalies_observed"), reasons


def run(s, output_root, sdk_module=None):
    validate(s)
    output = Path(output_root).expanduser().resolve() / (datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d_%H%M%S_%f") + "_" + s.label)
    output.mkdir(parents=True, exist_ok=False)
    report = {"schema": "camera_cable_diagnostic_v1", "status": "starting", "started_at": now_iso(),
              "settings": asdict(s), "python": sys.executable, "platform": platform.platform(),
              "streams": {}, "events": {}, "queue_overflow": 0,
              "mode": "independent_sensor_callbacks_no_alignment_no_frame_pairing",
              "notes": ["SDK가 애플리케이션에 전달한 프레임 계측이며 USB 패킷 분석/케이블 합격 인증이 아닙니다.",
                        "번호 건너뜀은 수신 누락 의심값. SDK 버퍼, CPU/GIL 지연 등도 원인일 수 있습니다.",
                        "장치 timestamp와 host monotonic 시계는 다릅니다. 두 값을 빼서 절대 지연으로 보지 마세요.",
                        "미리보기는 일부 프레임만 디코딩합니다. 화면 갱신 수와 수신 수는 다릅니다.",
                        "로봇 동작/전원 제어/재연결/설정 완화/깊이 평균/정합을 하지 않습니다."]}
    write_json(output / "summary.json", report)
    (output / "usb_tree_before.txt").write_text(usb_tree(), encoding="utf-8")
    channel = Channel(s.visualize)
    events = Counter()
    stats = {}
    started_sensors = []
    preview = context = sdk = None
    origin = time.monotonic()
    measure_start = end = stop_at = None
    selected = {}
    last_interval = None
    last_counts = {"color": 0, "depth": 0}
    with (output / "frames.jsonl").open("w", encoding="utf-8") as frames_log, \
         (output / "events.jsonl").open("w", encoding="utf-8") as events_log, \
         (output / "intervals.jsonl").open("w", encoding="utf-8") as intervals_log:

        def record_event(event):
            event = dict(event)
            event["elapsed_s"] = event.pop("host_s", time.monotonic()) - origin
            events[event["type"]] += 1
            events_log.write(json.dumps(event, ensure_ascii=False) + "\n")

        def consume(event, cutoff=None):
            if event["type"] != "frame":
                record_event(event)
                return
            if cutoff is not None and event["host_s"] > cutoff:
                return
            measuring = measure_start is not None and event["host_s"] >= measure_start
            expected = report["profiles"][event["stream"]]
            if any(event[key] != expected[key] for key in ("width", "height", "format")):
                record_event({"type": "frame_profile_mismatch", "stream": event["stream"], "host_s": event["host_s"]})
            bytes_per_pixel = {"Y16": 2, "RGB": 3, "YUYV": 2, "NV12": 1.5}.get(event["format"])
            if bytes_per_pixel is not None and event["bytes"] != event["width"] * event["height"] * bytes_per_pixel:
                record_event({"type": "unexpected_payload_size", "stream": event["stream"],
                              "host_s": event["host_s"], "bytes": event["bytes"]})
            flags = stats[event["stream"]].add(event) if measuring else []
            row = dict(event, elapsed_s=event["host_s"] - origin, phase="measurement" if measuring else "warmup", flags=flags)
            frames_log.write(json.dumps(row, ensure_ascii=False) + "\n")
            if flags:
                record_event({"type": "frame_anomaly", "stream": event["stream"], "host_s": event["host_s"],
                              "index": event["index"], "flags": flags})

        def update_summary(now):
            report["streams"] = {name: stat.summary(now) for name, stat in stats.items()}
            report["events"] = dict(events)
            report["queue_overflow"] = channel.overflow
            write_json(output / "summary.json", report)

        try:
            print(f"케이블: {s.label} / 준비 {s.warmup:g}초 + 측정 {s.seconds:g}초\n저장: {output}", flush=True)
            if sdk_module is None:
                import pyorbbecsdk as sdk
            else:
                sdk = sdk_module
            for dist in ("pyorbbecsdk2", "pyorbbecsdk"):
                try:
                    report["sdk_package"] = {"name": dist, "version": metadata.version(dist)}
                    break
                except metadata.PackageNotFoundError:
                    pass
            sdk_dir = output / "sdk_logs"
            sdk_dir.mkdir()
            sdk.Context.set_logger_to_file(sdk.OBLogLevel.INFO, str(sdk_dir))
            sdk.Context.set_logger_to_console(sdk.OBLogLevel.WARNING)

            def sdk_log(*args):
                channel.put({"type": "sdk_warning_or_error", "host_s": time.monotonic(),
                             "message": " | ".join(str(value) for value in args)})

            sdk.Context.set_logger_to_callback(sdk.OBLogLevel.WARNING, sdk_log)
            context = sdk.Context()
            device = select_device(sdk, context, s.serial)
            info = device.get_device_info()
            report["device"] = {"name": info.get_name(), "serial": info.get_serial_number(),
                                "firmware": info.get_firmware_version(), "connection": info.get_connection_type()}
            target_serial = info.get_serial_number()

            def changed(removed, added):
                try:
                    for action, devices in (("device_removed", removed), ("device_added", added)):
                        for i in range(devices.get_count()):
                            serial = devices.get_device_serial_number_by_index(i)
                            if serial == target_serial:
                                channel.put({"type": action, "host_s": time.monotonic(), "serial": serial})
                except Exception as error:
                    channel.put({"type": "device_event_error", "host_s": time.monotonic(), "message": str(error)})

            context.set_device_changed_callback(changed)
            selected = get_profiles(sdk, device, s)
            report["profiles"] = {name: profile_info(profile) for name, (_, profile) in selected.items()}
            print(f"카메라: {report['device']}\n프로파일: {report['profiles']}", flush=True)
            if s.visualize:
                preview = Preview(s.timeout)
            # 독립 센서 콜백: RGB가 누락돼도 Depth 관측은 계속한다. frameset 동기화로 숨기지 않음.
            for name, (sensor, profile) in selected.items():
                started_sensors.append(sensor)  # start가 부분 성공한 뒤 실패해도 stop 시도
                sensor.start(profile, channel.callback(name))
            origin = time.monotonic()
            measure_start, end = origin + s.warmup, origin + s.warmup + s.seconds
            stats = {name: StreamStats(s.fps, measure_start, s.timeout) for name in selected}
            report.update(status="running", measurement_started_at_after_warmup_seconds=s.warmup)
            next_report, next_preview = origin, origin
            while True:
                now = time.monotonic()
                if now >= end:
                    stop_at = end
                    report["status"] = "completed"
                    break
                try:
                    consume(channel.queue.get(timeout=min(0.05, end - now)), end)
                    # 큐를 한 번에 무제한 비우지 않아 UI/타임아웃 감시가 굶지 않게 한다.
                    for _ in range(255):
                        try:
                            consume(channel.queue.get_nowait(), end)
                        except queue.Empty:
                            break
                except queue.Empty:
                    pass
                now = min(time.monotonic(), end)
                if now >= measure_start:
                    for name, stat in stats.items():
                        if stat.silence(now):
                            record_event({"type": "no_unique_frame_timeout", "stream": name, "host_s": now})
                if now >= next_report:
                    if now >= measure_start:
                        previous = measure_start if last_interval is None else last_interval
                        duration = now - previous
                        rates = {name: (stat.counts["unique"] - last_counts[name]) / duration if duration > 0 else 0
                                 for name, stat in stats.items()}
                        intervals_log.write(json.dumps({"elapsed_s": now - measure_start, "window_s": duration,
                                                       "received_fps": rates, "queue_size": channel.queue.qsize()}) + "\n")
                        last_counts = {name: stat.counts["unique"] for name, stat in stats.items()}
                        last_interval = now
                        print(f"{now - measure_start:7.1f}/{s.seconds:g}s | RGB {rates['color']:5.1f} FPS | "
                              f"Depth {rates['depth']:5.1f} FPS | 누락 의심 "
                              f"RGB {stats['color'].counts['suspected_missing_by_index']} / "
                              f"Depth {stats['depth'].counts['suspected_missing_by_index']}", flush=True)
                    else:
                        print(f"준비 중: {max(0, measure_start - now):.1f}초", flush=True)
                    update_summary(now)
                    for stream in (frames_log, events_log, intervals_log):
                        stream.flush()
                    next_report = now + 1
                if preview is not None and now >= next_preview:
                    should_stop, errors = preview.update(channel, now)
                    for error in errors:
                        record_event(error)
                    next_preview = time.monotonic() + 1 / s.preview_fps
                    if should_stop:
                        stop_at = min(time.monotonic(), end)
                        report["status"] = "interrupted"
                        break
        except KeyboardInterrupt:
            stop_at = time.monotonic() if end is None else min(time.monotonic(), end)
            report["status"] = "interrupted"
            print("\n사용자 중단: 수집한 결과를 저장합니다.", flush=True)
        except Exception:
            stop_at = time.monotonic() if end is None else min(time.monotonic(), end)
            report["status"] = "failed"
            report["error"] = traceback.format_exc()
            (output / "error.log").write_text(report["error"], encoding="utf-8")
            print(report["error"], file=sys.stderr)
        finally:
            channel.accepting = False
            for sensor in reversed(started_sensors):
                try:
                    sensor.stop()
                except Exception as error:
                    record_event({"type": "stop_error", "message": str(error)})
            if sdk is not None:
                try:
                    sdk.Context.set_logger_to_callback(sdk.OBLogLevel.NONE, lambda *args: None)
                except Exception:
                    pass
            while True:
                try:
                    consume(channel.queue.get_nowait(), stop_at)
                except queue.Empty:
                    break
            if preview is not None:
                report["preview"] = {"frames_checked": dict(preview.frames_checked), "decode_errors": dict(preview.decode_errors)}
                try:
                    preview.close()
                except Exception as error:
                    record_event({"type": "preview_close_error", "message": str(error)})
            stop_at = time.monotonic() if stop_at is None else stop_at
            update_summary(stop_at)
            report["finished_at"] = now_iso()
            report["assessment"], report["review_reasons"] = assessment(report)
            write_json(output / "summary.json", report)
    (output / "usb_tree_after.txt").write_text(usb_tree(), encoding="utf-8")
    lines = [f"케이블: {s.label}", f"실행 상태: {report['status']}", f"관측 판정: {report['assessment']}"]
    if "profiles" in report:
        lines.append(f"프로파일: {report['profiles']}")
    for name, stat in report["streams"].items():
        fps_text = f"{stat['received_fps']:.3f}" if stat["received_fps"] is not None else "측정 전 중단"
        lines.append(f"{name}: {fps_text} FPS, 고유 수신 {stat['unique']}, 누락 의심 {stat['suspected_missing_by_index']}, "
                     f"중복 {stat['duplicates']}, 최장 무수신 {stat['max_no_unique_frame_ms']:.1f} ms, "
                     f"타임아웃 사건 {stat['timeout_episodes']}")
    lines.extend(report["review_reasons"])
    lines.append("관측된 이상이 없어도 케이블의 장기/실제 겐트리 운용 안정성을 보장하지 않습니다.")
    (output / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n" + "\n".join(lines) + f"\n저장 완료: {output}", flush=True)
    return output, report


def main():
    parser = argparse.ArgumentParser(description="Femto Bolt RGB/Depth USB 연장선 진단 (로봇 제어 없음)")
    parser.add_argument("--label", default=CABLE_LABEL)
    parser.add_argument("--note", default="", help="케이블 제품명/길이/전원 방식 기록")
    parser.add_argument("--seconds", type=float, default=DURATION_SEC)
    parser.add_argument("--warmup", type=float, default=WARMUP_SEC)
    parser.add_argument("--timeout", type=float, default=TIMEOUT_SEC)
    parser.add_argument("--serial", default="")
    parser.add_argument("--color-width", type=int, default=COLOR_WIDTH)
    parser.add_argument("--color-height", type=int, default=COLOR_HEIGHT)
    parser.add_argument("--color-format", choices=["MJPG", "RGB", "YUYV", "NV12"], default=COLOR_FORMAT)
    parser.add_argument("--depth-width", type=int, default=DEPTH_WIDTH)
    parser.add_argument("--depth-height", type=int, default=DEPTH_HEIGHT)
    parser.add_argument("--fps", type=int, default=FPS)
    parser.add_argument("--visualize", action="store_true", help="관찰용 미리보기 추가 (추가 CPU 부하 발생)")
    parser.add_argument("--preview-fps", type=float, default=PREVIEW_FPS)
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    parser.add_argument("--list-profiles", action="store_true", help="장치 연결 후 지원 프로파일만 출력. 스트림 시작/저장 없음")
    args = vars(parser.parse_args())
    list_profiles, output_root = args.pop("list_profiles"), args.pop("output_root")
    try:
        s = Settings(**args)
        validate(s)
        if list_profiles:
            import pyorbbecsdk as sdk
            context = sdk.Context()
            device = select_device(sdk, context, s.serial)
            for kind in (sdk.OBSensorType.COLOR_SENSOR, sdk.OBSensorType.DEPTH_SENSOR):
                profiles = device.get_sensor(kind).get_stream_profile_list()
                print(kind)
                for i in range(len(profiles)):
                    print(profile_info(profiles[i]))
            return 0
        _, report = run(s, output_root)
        if report["status"] == "interrupted":
            return 130
        if report["status"] == "failed" or report["assessment"] == "inconclusive":
            return 1
        return 0 if report["assessment"] == "no_anomalies_observed" else 2
    except Exception as error:
        print(f"오류: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
