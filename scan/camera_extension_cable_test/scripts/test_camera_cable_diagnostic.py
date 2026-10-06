"""하드웨어 없이 통계와 SDK 콜백/로그 수명주기 검증."""
from contextlib import redirect_stderr, redirect_stdout
from dataclasses import replace
import io
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import camera_cable_diagnostic as diagnostic


def frame(index, stamp, device_us=None):
    return {"index": index, "host_s": stamp, "device_us": round(stamp * 1e6) if device_us is None else device_us,
            "bytes": 100, "type": "frame", "stream": "color", "width": 10, "height": 10, "format": "MJPG"}


class FakeProfile:
    def __init__(self, width, height, fmt, fps):
        self.width, self.height, self.fmt, self.fps = width, height, fmt, fps

    def as_video_stream_profile(self): return self
    def get_width(self): return self.width
    def get_height(self): return self.height
    def get_format(self): return self.fmt
    def get_fps(self): return self.fps


class FakeFrame:
    def __init__(self, profile, index):
        self.profile, self.index = profile, index

    def as_video_frame(self): return self
    def get_index(self): return self.index
    def get_timestamp_us(self): return round(self.index / self.profile.fps * 1e6)
    def get_data_size(self): return self.profile.width * self.profile.height * 2 if self.profile.fmt == "Y16" else 100
    def get_width(self): return self.profile.width
    def get_height(self): return self.profile.height
    def get_format(self): return self.profile.fmt


class FakeSensor:
    def __init__(self, silent=False, fail=False):
        self.silent, self.fail = silent, fail
        self.quit = threading.Event()
        self.thread = None
        self.stopped = False

    def get_stream_profile_list(self): return self

    def get_video_stream_profile(self, width, height, fmt, fps):
        return FakeProfile(width, height, fmt, fps)

    def start(self, profile, callback):
        if self.fail:
            raise RuntimeError("synthetic start error")

        def produce():
            index = 0
            while not self.quit.is_set():
                if not self.silent:
                    callback(FakeFrame(profile, index))
                index += 1
                self.quit.wait(1 / profile.fps)
        self.thread = threading.Thread(target=produce, daemon=True)
        self.thread.start()

    def stop(self):
        self.stopped = True
        self.quit.set()
        if self.thread:
            self.thread.join(timeout=1)


def fake_sdk(silent_depth=False, fail_depth=False, missing=False):
    color, depth = FakeSensor(), FakeSensor(silent_depth, fail_depth)
    info = SimpleNamespace(get_name=lambda: "Orbbec Femto Bolt", get_serial_number=lambda: "TEST",
                           get_firmware_version=lambda: "fake", get_connection_type=lambda: "USB3.0")
    device = SimpleNamespace(get_device_info=lambda: info,
                             get_sensor=lambda kind: color if kind == "color" else depth)
    devices = SimpleNamespace(get_count=lambda: 0 if missing else 1,
                              get_device_by_index=lambda i: device, get_device_by_serial_number=lambda sn: device)

    class Context:
        @staticmethod
        def set_logger_to_file(*args): pass
        @staticmethod
        def set_logger_to_console(*args): pass
        @staticmethod
        def set_logger_to_callback(*args): pass
        def query_devices(self): return devices
        def set_device_changed_callback(self, callback): self.changed = callback

    return SimpleNamespace(Context=Context, OBLogLevel=SimpleNamespace(INFO=1, WARNING=2, NONE=0),
                           OBFormat=SimpleNamespace(MJPG="MJPG", Y16="Y16"),
                           OBSensorType=SimpleNamespace(COLOR_SENSOR="color", DEPTH_SENSOR="depth"),
                           sensors=[color, depth])


class StatisticsTests(unittest.TestCase):
    def test_nominal_frame_timing_and_fps(self):
        stats = diagnostic.StreamStats(30, 0, 2)
        for i in range(300):
            self.assertEqual(stats.add(frame(i, (i + 0.5) / 30)), [])
        summary = stats.summary(10)
        self.assertEqual(summary["unique"], 300)
        self.assertEqual(summary["received_fps"], 30)
        self.assertEqual(summary["suspected_missing_by_index"], 0)
        self.assertAlmostEqual(summary["max_no_unique_frame_ms"], 1000 / 30)
        report = {"status": "completed", "streams": {"color": summary}, "events": {}, "queue_overflow": 0}
        self.assertEqual(diagnostic.assessment(report)[0], "no_anomalies_observed")
        report["device"] = {"connection": "USB2.0"}
        self.assertEqual(diagnostic.assessment(report)[0], "review_required")

    def test_gaps_duplicates_and_clock_reset_not_huge_loss(self):
        stats = diagnostic.StreamStats(30, 0, 2)
        stats.add(frame(100, 0.01, 1000))
        self.assertIn("duplicate", stats.add(frame(100, 0.02, 1000)))
        self.assertIn("index_gap", stats.add(frame(103, 0.11, 101000)))
        self.assertIn("reset_or_reorder", stats.add(frame(0, 0.14, 0)))
        summary = stats.summary(0.15)
        self.assertEqual(summary["suspected_missing_by_index"], 2)
        self.assertEqual(summary["duplicates"], 1)
        self.assertEqual(summary["resets_or_reorders"], 1)
        self.assertEqual(summary["unique"], 3)

    def test_same_index_new_timestamp_and_empty_payload(self):
        stats = diagnostic.StreamStats(30, 0, 2)
        stats.add(frame(0, 0))
        value = frame(0, 0.033)
        value["bytes"] = 0
        flags = stats.add(value)
        self.assertIn("repeated_index", flags)
        self.assertIn("empty_payload", flags)
        self.assertEqual(stats.counts["unique"], 2)

    def test_timeout_episodes_leading_trailing_and_recovery(self):
        stats = diagnostic.StreamStats(30, 10, 2)
        self.assertFalse(stats.silence(11))
        self.assertTrue(stats.silence(12))
        self.assertFalse(stats.silence(13))
        stats.add(frame(1, 13.1))
        self.assertTrue(stats.silence(15.2))
        stats.add(frame(2, 16.0))
        summary = stats.summary(18.1)
        self.assertEqual(summary["timeout_episodes"], 3)
        self.assertAlmostEqual(summary["max_no_unique_frame_ms"], 3100)

    def test_no_frames_never_reports_good(self):
        stats = diagnostic.StreamStats(30, 0, 2)
        report = {"status": "completed", "queue_overflow": 0, "streams": {"depth": stats.summary(10)}, "events": {}}
        self.assertEqual(diagnostic.assessment(report)[0], "review_required")
        report["queue_overflow"] = 1
        self.assertEqual(diagnostic.assessment(report)[0], "inconclusive")
        report.update(queue_overflow=0, status="interrupted")
        self.assertEqual(diagnostic.assessment(report)[0], "inconclusive")

    def test_callback_metadata_queue_overflow_and_disabled(self):
        channel = diagnostic.Channel(capacity=1)
        callback = channel.callback("depth")
        f = FakeFrame(FakeProfile(640, 576, "Y16", 30), 1)
        callback(f)
        callback(f)
        self.assertEqual(channel.overflow, 1)
        self.assertEqual(channel.queue.get()["index"], 1)
        channel.accepting = False
        callback(f)
        self.assertTrue(channel.queue.empty())

    def test_validation_and_no_profile_fallback(self):
        for values in ({"label": "../bad"}, {"seconds": 0}, {"seconds": float("nan")},
                       {"warmup": -1}, {"fps": 0}, {"color_format": "BAD"}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                diagnostic.validate(replace(diagnostic.Settings(), **values))
        sdk = fake_sdk()
        device = diagnostic.select_device(sdk, sdk.Context(), "")
        sdk.sensors[0].get_video_stream_profile = lambda *args: FakeProfile(640, 480, "MJPG", 30)
        with self.assertRaisesRegex(RuntimeError, "반환값"):
            diagnostic.get_profiles(sdk, device, diagnostic.Settings())


class LifecycleTests(unittest.TestCase):
    def run_fake(self, sdk, **changes):
        s = replace(diagnostic.Settings(), seconds=0.24, warmup=0.04, timeout=0.06, fps=50, **changes)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        with patch.object(diagnostic, "usb_tree", return_value="fake USB tree"), \
             redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            return diagnostic.run(s, self.temp.name, sdk_module=sdk)

    def test_two_streams_jsonl_warmup_exclusion_and_cleanup(self):
        sdk = fake_sdk()
        out, report = self.run_fake(sdk)
        self.assertEqual(report["status"], "completed")
        self.assertEqual(report["profiles"]["depth"]["fps"], 50)
        rows = [json.loads(line) for line in (out / "frames.jsonl").read_text().splitlines()]
        for name in ("color", "depth"):
            self.assertGreater(report["streams"][name]["unique"], 0)
            measured = [r for r in rows if r["stream"] == name and r["phase"] == "measurement"]
            self.assertEqual(report["streams"][name]["unique"], len(measured))
            self.assertAlmostEqual(report["streams"][name]["measured_seconds"], 0.24)
        self.assertTrue(any(r["phase"] == "warmup" for r in rows))
        self.assertTrue(all(s.stopped for s in sdk.sensors))
        for filename in ("summary.json", "summary.txt", "events.jsonl", "intervals.jsonl", "usb_tree_before.txt", "usb_tree_after.txt"):
            self.assertTrue((out / filename).is_file())

    def test_one_stream_stall_independent_from_healthy_color(self):
        _, report = self.run_fake(fake_sdk(silent_depth=True))
        self.assertEqual(report["status"], "completed")
        self.assertEqual(report["assessment"], "review_required")
        self.assertGreater(report["streams"]["color"]["unique"], 0)
        self.assertEqual(report["streams"]["depth"]["unique"], 0)
        self.assertEqual(report["streams"]["depth"]["timeout_episodes"], 1)
        self.assertAlmostEqual(report["streams"]["depth"]["max_no_unique_frame_ms"], 240)

    def test_partial_start_failure_stops_both_and_saves_error(self):
        sdk = fake_sdk(fail_depth=True)
        out, report = self.run_fake(sdk)
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["assessment"], "inconclusive")
        self.assertTrue((out / "error.log").is_file())
        self.assertTrue(all(s.stopped for s in sdk.sensors))

    def test_missing_device_is_logged_without_false_success(self):
        out, report = self.run_fake(fake_sdk(missing=True))
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["streams"], {})
        self.assertTrue((out / "summary.txt").is_file())

    def test_keyboard_interrupt_saves_partial_result_and_cleanup(self):
        sdk = fake_sdk()
        original = diagnostic.StreamStats.add
        interrupted = False

        def interrupt_once(stats, event):
            nonlocal interrupted
            if not interrupted:
                interrupted = True
                raise KeyboardInterrupt
            return original(stats, event)

        with patch.object(diagnostic.StreamStats, "add", interrupt_once):
            # warmup 후 첫 프레임을 처리할 때 중단. 스레드 종료와 부분 로그 확인.
            out, report = self.run_fake(sdk)
        self.assertEqual(report["status"], "interrupted")
        self.assertEqual(report["assessment"], "inconclusive")
        self.assertTrue(all(s.stopped for s in sdk.sensors))
        self.assertTrue((out / "summary.json").is_file())


if __name__ == "__main__":
    unittest.main()
