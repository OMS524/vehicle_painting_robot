"""SAM 추론용 자원 샘플링/단계별 시간 측정. 모델이나 카메라 SDK에는 의존하지 않는다."""
from __future__ import annotations

from contextlib import contextmanager
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time


MIB = 1024 ** 2
METRICS = (
    "process_cpu_percent", "process_ram_mib", "gpu_utilization_percent_device",
    "gpu_memory_used_mib_device", "gpu_memory_used_mib_process",
    "torch_allocated_mib", "torch_reserved_mib",
)
CSV_FIELDS = ("timestamp_utc", "elapsed_sec", "phase", "image", *METRICS)


def number_or_none(value):
    """NVIDIA의 N/A 등은 알 수 없는 값이므로 0으로 대체하지 않는다."""
    try:
        value = float(value)
        return value if math.isfinite(value) and value >= 0 else None
    except (TypeError, ValueError):
        return None


class ResourceMonitor:
    def __init__(self, output: Path, interval_sec: float = 0.5):
        if (isinstance(interval_sec, bool) or not isinstance(interval_sec, (int, float))
                or not math.isfinite(interval_sec) or interval_sec < 0.1):
            raise ValueError("RESOURCE_SAMPLE_INTERVAL_SEC은 0.1초 이상의 유한한 숫자여야 합니다.")
        try:
            import psutil
        except ImportError as error:
            raise RuntimeError("자원 로그에는 psutil이 필요합니다: python -m pip install psutil") from error
        self.psutil = psutil
        self.process = psutil.Process(os.getpid())
        self.output, self.interval = Path(output), interval_sec
        self.torch, self.cuda_device, self.gpu_uuid = None, None, None
        self.gpu_info = None
        self.smi = shutil.which("nvidia-smi")
        self.context = ("startup", "")
        self.phases, self.warnings = [], {}
        self.stats = {key: {"samples": 0, "sum": 0.0, "max": None} for key in METRICS}
        self.stop_event = threading.Event()
        self.last_cpu = self.last_sample_time = None

    def warn(self, key, error):
        if key not in self.warnings:
            self.warnings[key] = str(error)
            print(f"자원 로그 경고 [{key}]: {error}", file=sys.stderr, flush=True)

    def __enter__(self):
        self.started_at = datetime.now(timezone.utc).isoformat()
        self.started = time.perf_counter()
        self.stream = (self.output / "resource_usage.csv").open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.stream, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        self.stream.flush()
        self.thread = threading.Thread(target=self._sample_loop, name="sam-resource-monitor", daemon=True)
        self.thread.start()
        return self

    def attach_torch(self, torch, device):
        self.torch = torch
        if device == "cpu":
            return
        self.cuda_device = torch.cuda.current_device()
        props = torch.cuda.get_device_properties(self.cuda_device)
        self.gpu_info = {"cuda_device": self.cuda_device, "name": props.name,
                         "total_memory_mib": props.total_memory / MIB}
        uuid = str(getattr(props, "uuid", ""))
        if uuid:
            # CUDA_VISIBLE_DEVICES가 설정되어도 물리 GPU 0을 임의로 선택하지 않는다.
            self.gpu_uuid = uuid if uuid.startswith(("GPU-", "MIG-")) else "GPU-" + uuid
            self.gpu_info["uuid"] = self.gpu_uuid
        else:
            self.warn("gpu_uuid", "CUDA 장치 UUID가 없어 nvidia-smi 수집을 생략합니다.")
        if not self.smi:
            self.warn("nvidia_smi", "nvidia-smi가 없습니다. PyTorch VRAM만 기록합니다.")

    def _query_smi(self, query):
        try:
            result = subprocess.run(
                [self.smi, f"--id={self.gpu_uuid}", query, "--format=csv,noheader,nounits"],
                capture_output=True, text=True, timeout=2, check=True,
            )
        except subprocess.CalledProcessError as error:
            raise RuntimeError((error.stderr or error.stdout or str(error)).strip()) from error
        return list(csv.reader(result.stdout.splitlines(), skipinitialspace=True))

    def _gpu_metrics(self):
        values = {}
        if self.cuda_device is None:
            return values
        try:
            values["torch_allocated_mib"] = self.torch.cuda.memory_allocated(self.cuda_device) / MIB
            values["torch_reserved_mib"] = self.torch.cuda.memory_reserved(self.cuda_device) / MIB
        except Exception as error:
            self.warn("torch_memory", error)
        if not self.smi or not self.gpu_uuid:
            return values
        try:
            rows = self._query_smi("--query-gpu=uuid,utilization.gpu,memory.used")
            row = next((row for row in rows if len(row) == 3 and row[0].strip() == self.gpu_uuid), None)
            if row is None:
                raise RuntimeError(f"nvidia-smi 결과에 선택한 GPU가 없습니다: {self.gpu_uuid}")
            values["gpu_utilization_percent_device"] = number_or_none(row[1])
            values["gpu_memory_used_mib_device"] = number_or_none(row[2])
        except Exception as error:
            self.warn("gpu_device_query", error)
        try:
            rows = self._query_smi("--query-compute-apps=pid,gpu_uuid,used_gpu_memory")
            used = [number_or_none(row[2]) for row in rows if len(row) == 3
                    and row[0].strip() == str(self.process.pid) and row[1].strip() == self.gpu_uuid]
            # 프로세스가 조회되지 않거나 드라이버가 N/A를 반환하면 빈 값으로 둔다.
            values["gpu_memory_used_mib_process"] = sum(used) if used and all(v is not None for v in used) else None
        except Exception as error:
            self.warn("gpu_process_query", error)
        return values

    def _sample(self):
        now = time.perf_counter()
        phase, image = self.context
        row = {"timestamp_utc": datetime.now(timezone.utc).isoformat(),
               "elapsed_sec": now - self.started, "phase": phase, "image": image}
        try:
            times = self.process.cpu_times()
            cpu = times.user + times.system
            row["process_ram_mib"] = self.process.memory_info().rss / MIB
            if self.last_sample_time is not None and now > self.last_sample_time:
                row["process_cpu_percent"] = max(0.0, (cpu - self.last_cpu) / (now - self.last_sample_time) * 100)
            self.last_cpu, self.last_sample_time = cpu, now
        except Exception as error:
            self.warn("process_query", error)
        row.update(self._gpu_metrics())
        self.writer.writerow(row)
        self.stream.flush()
        for key in METRICS:
            value = row.get(key)
            if value is not None:
                stats = self.stats[key]
                stats["samples"] += 1
                stats["sum"] += value
                stats["max"] = value if stats["max"] is None else max(stats["max"], value)

    def _sample_loop(self):
        while not self.stop_event.is_set():
            started = time.perf_counter()
            try:
                self._sample()
            except Exception as error:
                self.warn("sampler", error)
                return
            # 측정 자체가 느리면 요청 주기보다 길어질 수 있다. 실제 CSV 시각이 기준이다.
            if self.stop_event.wait(max(0, self.interval - (time.perf_counter() - started))):
                return

    def _synchronize(self):
        if self.cuda_device is not None:
            self.torch.cuda.synchronize(self.cuda_device)

    @contextmanager
    def phase(self, name, image=""):
        self._synchronize()
        if self.cuda_device is not None:
            self.torch.cuda.reset_peak_memory_stats(self.cuda_device)
        cpu_start = time.process_time()
        started = time.perf_counter()
        entry = {"phase": name, "image": str(image), "status": "complete"}
        self.context = (name, str(image))
        try:
            yield entry
            # CUDA는 비동기 실행이므로 실제 완료까지 기다린 시간을 측정한다.
            self._synchronize()
        except BaseException as error:
            entry.update(status="interrupted" if isinstance(error, KeyboardInterrupt) else "failed",
                         error=f"{type(error).__name__}: {error}")
            raise
        finally:
            entry["elapsed_sec"] = time.perf_counter() - started
            entry["process_cpu_time_sec"] = time.process_time() - cpu_start
            entry["average_process_cpu_percent"] = entry["process_cpu_time_sec"] / entry["elapsed_sec"] * 100
            if self.cuda_device is not None:
                try:
                    entry["torch_peak_allocated_mib"] = self.torch.cuda.max_memory_allocated(self.cuda_device) / MIB
                    entry["torch_peak_reserved_mib"] = self.torch.cuda.max_memory_reserved(self.cuda_device) / MIB
                except Exception as error:
                    self.warn("torch_peak", error)
            self.phases.append(entry)
            self.context = ("between_phases", "")

    def __exit__(self, exc_type, error, traceback):
        elapsed = time.perf_counter() - self.started
        finished_at = datetime.now(timezone.utc).isoformat()
        self.stop_event.set()
        self.thread.join()  # 각 nvidia-smi 호출에는 2초 제한이 있다.
        self.stream.close()
        status = "complete" if exc_type is None else "interrupted" if issubclass(exc_type, KeyboardInterrupt) else "failed"
        summary = {
            "schema": "sam3_resource_usage_v1", "status": status,
            "error": None if error is None else f"{exc_type.__name__}: {error}",
            "started_at_utc": self.started_at, "finished_at_utc": finished_at,
            "elapsed_sec": elapsed, "pid": self.process.pid, "python": sys.executable,
            "torch_version": getattr(self.torch, "__version__", None),
            "logical_cpu_count": self.psutil.cpu_count(), "gpu": self.gpu_info,
            "sample_interval_sec": self.interval,
            "metrics": {key: {"sample_count": stats["samples"], "max": stats["max"],
                              "mean_of_samples": stats["sum"] / stats["samples"] if stats["samples"] else None}
                        for key, stats in self.stats.items()},
            "phases": self.phases, "warnings": self.warnings,
            "notes": [
                "Memory unit: MiB (1024^2 bytes). CPU 100% = one logical CPU; values can exceed 100%.",
                "process_* covers this Python PID (including monitor thread), not other apps or child processes.",
                "gpu_*_device includes other processes; gpu_*_process is NVIDIA-reported VRAM for this PID.",
                "torch_* is PyTorch allocator memory, not total process VRAM including CUDA context/libraries.",
                "CSV GPU/CPU/RAM values are periodic samples; sampled maxima can miss brief peaks.",
                "Per-phase torch_peak_* uses allocator peak counters; CUDA is synchronized at phase boundaries.",
                "Missing/unavailable metrics are CSV blanks / JSON null, never substituted with zero.",
                "Samples carry the phase at sampling start; a slow GPU query can cross a phase boundary.",
                "Model loading may include checkpoint download. First inference may include warm-up.",
                "Logging/synchronization adds overhead; measured phases do not include logger shutdown.",
            ],
        }
        for key in ("torch_peak_allocated_mib", "torch_peak_reserved_mib"):
            summary[key] = max((p[key] for p in self.phases if key in p), default=None)
        try:
            (self.output / "performance_summary.json").write_text(
                json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        except Exception as write_error:
            self.warn("summary_write", write_error)
        return False
