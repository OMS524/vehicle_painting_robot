#!/usr/bin/env python3
"""A0912 + Femto Bolt SDK 필터 스캔.

python -s multi_view_filter.py --check
python -s multi_view_filter.py --execute
python -s multi_view_filter.py --view /결과/merged.ply

로봇/자세/TF는 scan_config, 필터는 multi_view_filter.yaml에서 읽는다.
6가지 필터를 개별 선택하거나 연결하며 필터 전/후 Depth를 함께 보존한다.
"""
import argparse
import copy
from datetime import datetime
import json
from pathlib import Path
import signal
import sys
import time
import traceback

import numpy as np
import yaml

import multi_view_scan_single_frame as scan

DEFAULT_CONFIG = Path(__file__).with_suffix(".yaml")
# SDK 권장 순서. YAML 나열 순서를 바꾸어도 이 순서로 실행한다.
FILTER_ORDER = ("DecimationFilter", "ThresholdFilter", "NoiseRemovalFilter",
                "SpatialAdvancedFilter", "TemporalFilter", "HoleFillingFilter")
# (기본값, 최소, 최대, 정수 여부). 실제 SDK schema 범위는 장치 초기화 후 다시 확인.
PARAMETERS = {
    "DecimationFilter": {"decimate": (2, 1, 8, True)},
    "ThresholdFilter": {"min": (0, 0, 16000, True), "max": (16000, 0, 16000, True)},
    "NoiseRemovalFilter": {"min_diff": (256, 1, 10000, True), "max_size": (80, 1, 1000, True)},
    "SpatialAdvancedFilter": {"alpha": (0.5, 0.1, 1, False), "disp_diff": (160, 1, 10000, True),
                              "magnitude": (1, 1, 5, True), "radius": (1, 0, 8, True)},
    "TemporalFilter": {"diff_scale": (0.1, 0.1, 1, False), "weight": (0.4, 0.1, 1, False)},
    "HoleFillingFilter": {"hole_filling_mode": (0, 0, 2, True)},
}


def validate_filters(settings):
    if not isinstance(settings, dict) or set(settings) - {"warmup_frames", "filters"}:
        raise ValueError("depth_filters에는 warmup_frames와 filters만 지정하세요.")
    result = copy.deepcopy(settings)
    warmup = result.setdefault("warmup_frames", 30)
    if type(warmup) is not int or warmup < 0:
        raise ValueError("depth_filters.warmup_frames는 0 이상의 정수여야 합니다.")
    filters = result.setdefault("filters", {})
    if not isinstance(filters, dict) or set(filters) - set(FILTER_ORDER):
        raise ValueError("지원하는 필터: " + ", ".join(FILTER_ORDER))
    for name in FILTER_ORDER:
        item = filters.setdefault(name, {"enabled": False})
        if not isinstance(item, dict) or set(item) - {"enabled", *PARAMETERS[name]}:
            raise ValueError(f"{name}: 알 수 없는 설정 키입니다.")
        if type(item.setdefault("enabled", False)) is not bool:
            raise ValueError(f"{name}.enabled는 true/false여야 합니다.")
        for key, (default, low, high, integer) in PARAMETERS[name].items():
            value = item.setdefault(key, default)
            if (type(value) not in (int, float) or not np.isfinite(value)
                    or (integer and type(value) is not int) or not low <= value <= high):
                raise ValueError(f"{name}.{key}: {low}~{high} 범위의 {'정수' if integer else '숫자'}가 필요합니다.")
    threshold = filters["ThresholdFilter"]
    if threshold["min"] >= threshold["max"]:
        raise ValueError("ThresholdFilter.min은 max보다 작아야 합니다.")
    if filters["TemporalFilter"]["enabled"] and warmup < 1:
        raise ValueError("TemporalFilter 사용 시 depth_filters.warmup_frames를 1 이상으로 설정하세요.")
    return result


def load_config(path):
    path = Path(path).expanduser().resolve()
    settings = yaml.safe_load(path.read_text(encoding="utf-8"))
    if (not isinstance(settings, dict) or "scan_config" not in settings
            or set(settings) - {"scan_config", "output_dir", "depth_filters"}):
        raise ValueError("필터 YAML에는 scan_config, output_dir, depth_filters를 지정하세요.")
    source = Path(settings["scan_config"]).expanduser()
    source = (path.parent / source).resolve() if not source.is_absolute() else source.resolve()
    cfg = scan.load_config(source)
    if "output_dir" in settings:
        output = Path(settings["output_dir"]).expanduser()
        cfg["output_dir"] = str((path.parent / output).resolve() if not output.is_absolute() else output.resolve())
    cfg.update(capture_mode="sdk_filter_chain", scan_config=str(source), filter_config=str(path),
               depth_filters=validate_filters(settings.get("depth_filters", {})))
    return cfg


def depth_array(frame):
    return np.frombuffer(frame.get_data(), dtype=np.uint16).reshape(frame.get_height(), frame.get_width()).copy()


class FilterChain:
    """장치 초기화 후 생성. Frame 프로파일/단위를 SDK가 갱신한 상태로 유지한다."""
    def __init__(self, sdk, settings):
        self.sdk = sdk
        self.settings = validate_filters(settings)
        self.filters, self.metadata = [], []
        for name in FILTER_ORDER:
            requested = self.settings["filters"][name]
            if not requested["enabled"]:
                continue
            # Factory 경로는 장치 미초기화 시 OBError를 반환한다. 이 SDK 버전의
            # 일부 직접 생성자는 private filter 활성화 전 native crash를 일으킨다.
            filt = sdk.FilterFactory.create_filter(name)
            schemas = {p.name: p for p in filt.get_config_schema_vec()}
            values, defaults = {}, {}
            for key in PARAMETERS[name]:
                if key not in schemas:
                    raise ValueError(f"설치된 SDK의 {name}에 {key}가 없습니다.")
                value, schema = requested[key], schemas[key]
                if not schema.min <= value <= schema.max:
                    raise ValueError(f"{name}.{key}={value}: SDK 범위 {schema.min}~{schema.max} 밖입니다.")
                filt.set_config_value(key, value)
                actual = filt.get_config_value(key)
                if not np.isclose(actual, value, rtol=1e-6, atol=1e-6):
                    raise ValueError(f"{name}.{key} 설정 불일치: 요청 {value}, SDK {actual}")
                values[key], defaults[key] = actual, schema.default
            filt.enable(True)
            self.filters.append((name, filt, schemas))
            self.metadata.append({"name": name, "parameters": values, "sdk_defaults": defaults})
        self.last_stages = []

    def reset(self):
        for _, filt, _ in self.filters:
            filt.reset()
        self.last_stages = []

    def process(self, depth):
        # SDK 필터의 in-place 처리 가능성에 관계없이 비교용 원본을 보존한다.
        current = self.sdk.Frame.create_frame_from_other_frame(depth, True).as_depth_frame()
        stages = []
        for name, filt, schemas in self.filters:
            auto = {}
            if name == "NoiseRemovalFilter":
                for key, value in (("width", current.get_width()), ("height", current.get_height())):
                    if key in schemas:
                        if not schemas[key].min <= value <= schemas[key].max:
                            raise ValueError(f"NoiseRemovalFilter의 {key} 지원 범위를 벗어났습니다: {value}")
                        filt.set_config_value(key, value)
                        auto[key] = value
            out = filt.process(current)
            if out is None:
                self.last_stages = stages + [{"name": name, "output": None}]
                return None
            current = out.as_depth_frame()
            if current.get_format() != self.sdk.OBFormat.Y16:
                raise ValueError(f"{name} 출력이 Y16 Depth가 아닙니다.")
            profile = current.get_stream_profile().as_video_stream_profile()
            intrinsic = profile.get_intrinsic()
            if (intrinsic.width != current.get_width() or intrinsic.height != current.get_height()
                    or intrinsic.fx <= 0 or intrinsic.fy <= 0):
                raise ValueError(f"{name}: 출력 해상도와 내부 파라미터가 일치하지 않습니다.")
            stages.append({"name": name, "width": current.get_width(), "height": current.get_height(),
                           "valid_pixels": int(np.count_nonzero(depth_array(current))),
                           "automatic_parameters": auto})
        self.last_stages = stages
        return current


class FilteredDepthCamera(scan.DepthCamera):
    def __init__(self, camera_settings, filter_settings):
        super().__init__(camera_settings)  # SDK 장치 초기화/필터 활성화를 먼저 수행.
        self.chain = FilterChain(self.sdk, filter_settings)

    def make_capture(self, depth, color, received_ns):
        sdk = self.sdk
        profile = depth.get_stream_profile().as_video_stream_profile()
        scale = float(depth.get_depth_scale())
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError("잘못된 Depth scale")
        frames = sdk.Frame.create_frame_set()
        frames.push_frame(depth)
        point_filter = sdk.PointCloudFilter()
        point_filter.set_create_point_format(sdk.OBFormat.POINT)
        point_filter.set_coordinate_system(sdk.OBCoordinateSystemType.RIGHT_HAND)
        points = point_filter.process(frames)
        if points is None:
            raise RuntimeError("SDK Depth → point cloud 변환 실패")
        points = points.as_points_frame()
        raw, xyz = depth_array(depth), scan.points_in_meters(points)
        if len(xyz) != raw.size:
            raise ValueError("필터 출력 Depth와 점군 픽셀 개수가 다릅니다.")
        metadata = {"device": self.device_info, "frame": scan.DEPTH_FRAME,
                    "alignment": "native_depth_no_color_alignment", "axes": "x right, y down, z forward",
                    "host_received_ns": received_ns, "device_timestamp_us": depth.get_timestamp_us(),
                    "sdk_system_timestamp_us": depth.get_system_timestamp_us(), "frame_index": depth.get_index(),
                    "width": depth.get_width(), "height": depth.get_height(), "fps": profile.get_fps(),
                    "format": "Y16", "depth_scale_mm_per_unit": scale,
                    "point_scale_mm_per_unit": float(points.get_position_value_scale()),
                    **scan.profile_calibration(profile)}
        result = scan.CapturedView(raw, xyz, metadata)
        if color is not None:
            result.color_rgb, result.color_aligned_rgb = scan.align_color_to_depth(sdk, depth, color)
            color_profile = color.get_stream_profile().as_video_stream_profile()
            ex = profile.get_extrinsic_to(color_profile)
            metadata["alignment"] = "color_to_depth_output_grid"
            metadata["color"] = {"width": color.get_width(), "height": color.get_height(),
                "fps": color_profile.get_fps(), "source_format": str(color.get_format()), "saved_format": "RGB8",
                "frame_index": color.get_index(), "device_timestamp_us": color.get_timestamp_us(),
                "sdk_system_timestamp_us": color.get_system_timestamp_us(),
                "depth_color_time_delta_us": abs(color.get_timestamp_us() - depth.get_timestamp_us()),
                **scan.profile_calibration(color_profile), "alignment_target_distortion": True,
                "depth_to_color": {"rotation": np.asarray(ex.rot).reshape(3, 3).tolist(),
                                   "translation_mm": np.asarray(ex.transform).reshape(3).tolist()},
                "unmapped_color": "black", "selection": "same_pair_as_source_depth"}
        return result

    def capture(self):
        settings, sdk = self.settings, self.sdk
        self.chain.reset()  # 각 자세별 Temporal 이력 초기화.
        self.pipeline.start(self.config)
        try:
            deadline = time.monotonic() + settings["capture_timeout_sec"]
            count, processed_count, seen = 0, 0, set()
            while time.monotonic() < deadline:
                frames = self.pipeline.wait_for_frames(500)
                depth = frames.get_depth_frame() if frames is not None else None
                if depth is None:
                    continue
                identity = (depth.get_index(), depth.get_timestamp_us())
                if identity in seen:
                    continue
                color = frames.get_color_frame() if settings.get("capture_color", False) else None
                if settings.get("capture_color", False) and (color is None or abs(
                        color.get_timestamp_us() - depth.get_timestamp_us()) > settings["color_max_time_delta_ms"] * 1000):
                    continue
                if depth.get_format() != sdk.OBFormat.Y16:
                    raise ValueError("Y16 Depth만 지원합니다.")
                seen.add(identity)
                count += 1
                if count <= settings["warmup_frames"]:
                    continue
                received_ns = time.time_ns()
                filtered = self.chain.process(depth)
                processed_count += 1
                if filtered is None or processed_count <= self.chain.settings["warmup_frames"]:
                    continue
                result = self.make_capture(filtered, color, received_ns)
                result.unfiltered = self.make_capture(depth, color, received_ns)
                result.unfiltered.metadata["capture_mode"] = "sdk_output_before_test_filters"
                result.metadata.update(capture_mode="sdk_filter_chain", captured_frames=1,
                    discarded_warmup_frames=settings["warmup_frames"], filter_input_frames=processed_count,
                    filter_warmup_frames=self.chain.settings["warmup_frames"],
                    filter_chain=self.chain.metadata, filter_stages=self.chain.last_stages,
                    source_depth={"width": depth.get_width(), "height": depth.get_height(),
                                  "frame_index": depth.get_index(), "device_timestamp_us": depth.get_timestamp_us()},
                    temporal_history_reset_per_view=True)
                print(f"SDK 필터: {processed_count}프레임 처리 후 마지막 출력 저장 "
                      f"({depth.get_width()}×{depth.get_height()} → {filtered.get_width()}×{filtered.get_height()})", flush=True)
                return result
            raise TimeoutError(f"SDK 필터 출력 취득 시간 초과: 입력 {processed_count}프레임, 마지막 단계 {self.chain.last_stages}")
        finally:
            self.pipeline.stop()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--view", type=Path, metavar="FILE.ply")
    args = parser.parse_args(argv)
    if args.view:
        scan.show_cloud(args.view.expanduser().resolve())
        return 0
    cfg = load_config(args.config)
    model = scan.ScanModel(cfg["xacro"])
    pose_source = scan.ScanPoseSource(cfg, model)
    scan.preflight(cfg, model, args.execute, pose_source)
    enabled = [n for n in FILTER_ORDER if cfg["depth_filters"]["filters"][n]["enabled"]]
    print("필터 순서:", " → ".join(enabled) or "모두 꺼짐")
    for name in enabled:
        print(f"  {name}: {cfg['depth_filters']['filters'][name]}")
    print(f"필터 워밍업: {cfg['depth_filters']['warmup_frames']}프레임. SDK 실제 지원 범위는 장치 초기화 후 확인합니다.")
    if not args.execute:
        return 0
    for view in cfg["views"]:
        print(f"{view['name']}: {scan.target_description(view, cfg)}")
    print("주의: 실제 로봇의 서보/자동 제어가 활성화됩니다. 이동 구간 충돌 검사는 없습니다.\n"
          "툴 무게·장착·전체 이동 경로·주변 안전·비상정지를 확인하세요. 종료 시 servo-off 합니다.")
    if input("실제 이동/촬영을 시작하려면 SCAN 입력: ").strip() != "SCAN":
        print("취소했습니다.")
        return 0
    output = Path(cfg["output_dir"]) / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True, exist_ok=False)
    print(f"저장 경로: {output}", flush=True)
    scan.save_json(output / "settings.json", cfg)
    try:
        # 모든 필터 생성/파라미터 오류는 RobotClient 시작 전에 처리한다.
        camera = FilteredDepthCamera(cfg["camera"], cfg["depth_filters"])
        scan.save_json(output / "filter_parameters.json", {"requested": cfg["depth_filters"], "applied": camera.chain.metadata})
        with scan.RobotClient(cfg, output) as robot:
            robot.call("initialize", settings=cfg["robot"], **scan.ik_request(cfg, model, pose_source),
                       timeout=cfg["robot"]["connect_timeout_sec"] + 15)
            scan.capture_sequence(cfg, model, robot, camera, output, pose_source)
    except BaseException:
        error = traceback.format_exc()
        (output / "error.log").write_text(error, encoding="utf-8")
        manifest_path = output / "manifest.json"
        manifest = json.loads(manifest_path.read_text()) if manifest_path.is_file() else {"views": []}
        manifest.update(status="failed", error=error, capture_mode="sdk_filter_chain")
        scan.save_json(manifest_path, manifest)
        raise
    print(f"완료: {output / 'merged.ply'}\n보기: python -s {Path(__file__).name} --view {output / 'merged.ply'}")
    return 0


if __name__ == "__main__":
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt("스캔 중단 요청")
    signal.signal(signal.SIGTERM, interrupt)
    try:
        raise SystemExit(main())
    except (Exception, KeyboardInterrupt) as exc:
        print(f"필터 스캔 오류: {exc}", file=sys.stderr)
        raise SystemExit(1)
