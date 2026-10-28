#!/usr/bin/env python3
# 실행: conda activate vehicle_painting_robot_planner
# python path_planning/scripts/path_planning.py --help

import csv
import copy
from dataclasses import MISSING, asdict, dataclass, fields, is_dataclass
import os
import sys
from datetime import datetime
from pathlib import Path

import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
PROJECT_NAME = "path_planning"
SOURCE_DIR = os.path.join(PROJECT_DIR, "src")
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

from painting_path_planner import (
    generate_surface_spline_rows,
    correct_surface_spline_rows,
    generate_offset_rows,
    generate_paint_spline_rows,
    structure_painting_trajectory,
)


@dataclass
class SurfaceExtractionConfig:
    spline_start_side: str
    spline_start_offset: float
    spline_spacing: float
    spline_half_width: float
    slicing_method: str = "axis"
    geodesic_normal_neighbor_count: int = 20
    geodesic_band_half_width: float = 0.0
    guide_extension_min_bin_points: int = 8
    geodesic_min_band_points: int = 8
    curvature_slice_angle_deg: float = 20.0

    @classmethod
    def from_dict(cls, data=None):
        return _dataclass_from_dict(cls, data)

    def to_kwargs(self):
        return asdict(self)


@dataclass
class CorrectionEndpointConfig:
    quantize_step: float
    graph_neighbor_count: int
    component_min_arc_length: float
    component_merge_backtrack_tolerance: float

    @classmethod
    def from_dict(cls, data=None):
        return _dataclass_from_dict(cls, data)


@dataclass
class CorrectionSupportConfig:
    support_point_spacing: float
    tangent_half_width: float
    anchor_max_distance: float
    transition_count: int
    endpoint_transition_count: int

    @classmethod
    def from_dict(cls, data=None):
        return _dataclass_from_dict(cls, data)


@dataclass
class CorrectionOptimizationConfig:
    lambda_data: float
    lambda_stretch: float
    lambda_bend: float
    lambda_end: float
    regularizer: float

    @classmethod
    def from_dict(cls, data=None):
        return _dataclass_from_dict(cls, data)


@dataclass
class CorrectionConfig:
    endpoint: CorrectionEndpointConfig
    support: CorrectionSupportConfig
    optimization: CorrectionOptimizationConfig

    @classmethod
    def from_dict(cls, data=None):
        source = _mapping_from_config(data)
        support_data = dict(source.get("support", {}) or {})
        if "support_point_spacing" not in support_data:
            if source.get("support_point_spacing") is not None:
                support_data["support_point_spacing"] = source["support_point_spacing"]
            else:
                legacy_sampling = dict(source.get("sampling", {}) or {})
                if legacy_sampling.get("point_spacing") is not None:
                    support_data["support_point_spacing"] = legacy_sampling["point_spacing"]
        return cls(
            endpoint=CorrectionEndpointConfig.from_dict(source.get("endpoint", {})),
            support=CorrectionSupportConfig.from_dict(support_data),
            optimization=CorrectionOptimizationConfig.from_dict(
                source.get("optimization", {})
            ),
        )

    def to_kwargs(self):
        endpoint = self.endpoint
        support = self.support
        optimization = self.optimization
        return {
            "row_point_spacing": support.support_point_spacing,
            "endpoint_quantize_step": endpoint.quantize_step,
            "endpoint_graph_neighbor_count": endpoint.graph_neighbor_count,
            "endpoint_component_min_arc_length": endpoint.component_min_arc_length,
            "endpoint_component_merge_backtrack_tolerance": (
                endpoint.component_merge_backtrack_tolerance
            ),
            "support_tangent_half_width": support.tangent_half_width,
            "support_anchor_max_distance": support.anchor_max_distance,
            "transition_count": support.transition_count,
            "endpoint_transition_count": support.endpoint_transition_count,
            "lambda_data": optimization.lambda_data,
            "lambda_stretch": optimization.lambda_stretch,
            "lambda_bend": optimization.lambda_bend,
            "lambda_end": optimization.lambda_end,
            "regularizer": optimization.regularizer,
        }


@dataclass
class OffsetConfig:
    offset_point_spacing: float
    offset_distance: float

    @classmethod
    def from_dict(cls, data=None):
        return _dataclass_from_dict(cls, data)

    def to_kwargs(self):
        return asdict(self)


@dataclass
class SplineConfig:
    spline_point_spacing: float
    final_point_spacing: float
    endpoint_extension_length: float
    raster_zigzag: bool
    paint_check_distance: float
    paint_check_half_width: float
    paint_min_false_length: float
    paint_min_true_length: float

    @classmethod
    def from_dict(cls, data=None):
        source = _mapping_from_config(data)
        if "spline_point_spacing" not in source:
            legacy_point_spacing = source.get("paint_spline_point_spacing")
            if legacy_point_spacing is not None:
                source["spline_point_spacing"] = legacy_point_spacing
        return _dataclass_from_dict(cls, source)

    def to_kwargs(self):
        return {
            "paint_spline_point_spacing": self.spline_point_spacing,
            "final_point_spacing": self.final_point_spacing,
            "endpoint_extension_length": self.endpoint_extension_length,
            "raster_zigzag": self.raster_zigzag,
            "paint_check_distance": self.paint_check_distance,
            "paint_check_half_width": self.paint_check_half_width,
            "paint_min_false_length": self.paint_min_false_length,
            "paint_min_true_length": self.paint_min_true_length,
        }


PaintSplineConfig = SplineConfig


@dataclass
class StructuringConfig:
    frame_id: str = "world"
    save_csv: bool = True
    csv_output_dir: str = ""
    painting_trajectory_csv_name: str = "painting_trajectory.csv"
    trajectory_csv_name_format: str = "trajectory_{row_index:03d}.csv"

    @classmethod
    def from_dict(cls, data=None):
        return _dataclass_from_dict(cls, data)

    def to_kwargs(self):
        return asdict(self)


@dataclass
class PaintingTrajectoryConfig:
    surface_extraction: SurfaceExtractionConfig
    correction: CorrectionConfig
    offset: OffsetConfig
    spline: SplineConfig
    structuring: StructuringConfig

    @classmethod
    def from_yaml(cls, path=None, overrides=None):
        return cls.from_dict(_load_painting_trajectory_config(path), overrides=overrides)

    @classmethod
    def from_dict(cls, data=None, overrides=None):
        source = _mapping_from_config(data)
        if "painting_trajectory" in source:
            source = _mapping_from_config(source["painting_trajectory"])
        if overrides:
            source = _deep_update_dict(source, overrides)
        legacy_spline = _mapping_from_config(source.get("paint_spline", {}))
        spline_data = _mapping_from_config(source.get("spline", {}))
        if legacy_spline:
            spline_data = _deep_update_dict(spline_data, legacy_spline)
        structuring_data = _mapping_from_config(source.get("structuring", {}))
        if "raster_zigzag" not in spline_data and "raster_zigzag" in structuring_data:
            spline_data["raster_zigzag"] = structuring_data["raster_zigzag"]
        return cls(
            surface_extraction=SurfaceExtractionConfig.from_dict(
                source.get("surface_extraction", {})
            ),
            correction=CorrectionConfig.from_dict(source.get("correction", {})),
            offset=OffsetConfig.from_dict(source.get("offset", {})),
            spline=SplineConfig.from_dict(spline_data),
            structuring=StructuringConfig.from_dict(structuring_data),
        )

    def to_dict(self):
        return asdict(self)

    def copy_with_overrides(self, overrides=None):
        return PaintingTrajectoryConfig.from_dict(self.to_dict(), overrides=overrides)

    @property
    def paint_spline(self):
        return self.spline


def _mapping_from_config(config):
    if config is None:
        return {}
    if isinstance(config, PaintingTrajectoryConfig):
        return config.to_dict()
    if is_dataclass(config):
        return asdict(config)
    if isinstance(config, dict):
        return copy.deepcopy(config)
    raise TypeError(f"Unsupported paint path config type: {type(config).__name__}")


def _dataclass_from_dict(cls, data=None):
    source = _mapping_from_config(data)
    class_fields = fields(cls)
    field_names = [field_info.name for field_info in class_fields]
    missing = [
        field_info.name
        for field_info in class_fields
        if (
            field_info.name not in source
            and field_info.default is MISSING
            and field_info.default_factory is MISSING
        )
    ]
    if missing:
        raise ValueError(
            f"Missing {cls.__name__} field(s) in paint path config: "
            + ", ".join(missing)
        )
    names = set(field_names)
    return cls(**{name: source[name] for name in names if name in source})


def _deep_update_dict(base, updates):
    result = _mapping_from_config(base)
    for key, value in _mapping_from_config(updates).items():
        if (
            key in result
            and isinstance(result[key], dict)
            and isinstance(value, dict)
        ):
            result[key] = _deep_update_dict(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def load_painting_trajectory_config(path=None, overrides=None) -> PaintingTrajectoryConfig:
    return PaintingTrajectoryConfig.from_yaml(path=path, overrides=overrides)


_DEFAULT_PAINTING_TRAJECTORY_CONFIG = None


def get_default_painting_trajectory_config() -> PaintingTrajectoryConfig:
    global _DEFAULT_PAINTING_TRAJECTORY_CONFIG
    if _DEFAULT_PAINTING_TRAJECTORY_CONFIG is None:
        _DEFAULT_PAINTING_TRAJECTORY_CONFIG = PaintingTrajectoryConfig.from_yaml()
    return _DEFAULT_PAINTING_TRAJECTORY_CONFIG.copy_with_overrides()


def _resolve_config_path(filename):
    install_prefix = os.path.dirname(PROJECT_DIR)
    candidates = [
        os.path.join(PROJECT_DIR, "config", filename),
        os.path.join(SCRIPT_DIR, filename),
        os.path.join(SCRIPT_DIR, "config", filename),
        os.path.join(install_prefix, "share", PROJECT_NAME, "config", filename),
        os.path.join(os.getcwd(), "config", filename),
        os.path.join(os.getcwd(), filename),
    ]

    seen = set()
    for path in candidates:
        normalized = os.path.abspath(path)
        if normalized in seen:
            continue
        seen.add(normalized)
        if os.path.isfile(normalized):
            return normalized
    if candidates:
        return os.path.abspath(candidates[0])
    return filename


def _load_painting_trajectory_config(path=None):
    config_path = path or _resolve_config_path("painting_path.yaml")
    if not os.path.isfile(config_path):
        raise FileNotFoundError(f"Config file not found: {config_path}")

    with open(config_path, "r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream) or {}

    if not isinstance(loaded, dict) or not isinstance(loaded.get("painting_trajectory"), dict):
        raise ValueError(
            f"Config file must contain top-level 'painting_trajectory' mapping: {config_path}"
        )
    return dict(loaded["painting_trajectory"])


def _resolve_painting_trajectory_config(config=None, config_path=None, overrides=None):
    if config is None and config_path is None:
        return get_default_painting_trajectory_config().copy_with_overrides(overrides)
    if config is None:
        return PaintingTrajectoryConfig.from_yaml(config_path, overrides=overrides)
    if isinstance(config, PaintingTrajectoryConfig):
        return config.copy_with_overrides(overrides)
    return PaintingTrajectoryConfig.from_dict(config, overrides=overrides)


PAINTING_TRAJECTORY_CSV_HEADER = [
    "row_index",
    "point_index",
    "frame_id",
    "s",
    "position_x",
    "position_y",
    "position_z",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
    "paint",
]


def _default_csv_output_dir():
    return os.path.join(PROJECT_DIR, "log", datetime.now().strftime("%Y%m%d_%H%M%S_%f"))


def _resolve_csv_output_dir(csv_output_dir):
    path = "" if csv_output_dir is None else str(csv_output_dir).strip()
    if not path:
        return _default_csv_output_dir()
    return os.path.abspath(os.path.expanduser(path))


def _csv_name(name, default):
    value = str(name or "").strip() or str(default)
    root, ext = os.path.splitext(value)
    if not ext:
        value = f"{value}.csv"
    return value


def _trajectory_csv_name(template, row_index):
    name_template = _csv_name(
        template,
        "trajectory_{row_index:03d}.csv",
    )
    try:
        return name_template.format(
            path_index=int(row_index),
            path_number=int(row_index) + 1,
            row_index=int(row_index),
            row_number=int(row_index) + 1,
            trajectory_index=int(row_index),
            trajectory_number=int(row_index) + 1,
            index=int(row_index),
            number=int(row_index) + 1,
        )
    except (IndexError, KeyError, ValueError):
        return name_template


def _unique_csv_path(path, used_paths):
    candidate = os.path.abspath(path)
    if candidate not in used_paths:
        used_paths.add(candidate)
        return candidate

    root, ext = os.path.splitext(candidate)
    suffix = 1
    while True:
        candidate = f"{root}_{suffix:03d}{ext}"
        if candidate not in used_paths:
            used_paths.add(candidate)
            return candidate
        suffix += 1


def _csv_row(frame_id, row_index, point_index, point):
    position = list(getattr(point, "position", (0.0, 0.0, 0.0)))
    orientation = list(getattr(point, "orientation", (0.0, 0.0, 0.0, 1.0)))
    return [
        int(row_index),
        int(point_index),
        str(frame_id),
        float(getattr(point, "s", 0.0)),
        float(position[0]),
        float(position[1]),
        float(position[2]),
        float(orientation[0]),
        float(orientation[1]),
        float(orientation[2]),
        float(orientation[3]),
        int(bool(getattr(point, "paint", False))),
    ]


def _write_csv(path, rows):
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(PAINTING_TRAJECTORY_CSV_HEADER)
        writer.writerows(rows)


def save_painting_trajectory_csv_files(
    painting_trajectory,
    csv_output_dir="",
    painting_trajectory_csv_name="painting_trajectory.csv",
    trajectory_csv_name_format="trajectory_{row_index:03d}.csv",
):
    output_dir = _resolve_csv_output_dir(csv_output_dir)
    os.makedirs(output_dir, exist_ok=True)

    frame_id = str(getattr(painting_trajectory, "frame_id", ""))
    rows = list(getattr(painting_trajectory, "rows", []))
    all_rows = []
    trajectory_csv_paths = []
    used_paths = set()

    full_path = _unique_csv_path(
        os.path.join(
            output_dir,
            _csv_name(painting_trajectory_csv_name, "painting_trajectory.csv"),
        ),
        used_paths,
    )

    for row_number, trajectory_row in enumerate(rows):
        row_index = int(getattr(trajectory_row, "row_index", row_number))
        points = list(getattr(trajectory_row, "points", []))
        csv_rows = [
            _csv_row(frame_id, row_index, point_index, point)
            for point_index, point in enumerate(points)
        ]
        all_rows.extend(csv_rows)

        trajectory_csv_path = _unique_csv_path(
            os.path.join(
                output_dir,
                _trajectory_csv_name(trajectory_csv_name_format, row_index),
            ),
            used_paths,
        )
        _write_csv(trajectory_csv_path, csv_rows)
        trajectory_csv_paths.append(trajectory_csv_path)

    _write_csv(full_path, all_rows)
    return {
        "painting_trajectory": full_path,
        "trajectories": trajectory_csv_paths,
        "paths": trajectory_csv_paths,
    }


def _structure_and_export_painting_trajectory(
    painted,
    structuring_config,
    structuring_kwargs=None,
):
    explicit_structuring_kwargs = dict(structuring_kwargs or {})
    options = {
        **structuring_config.to_kwargs(),
        **explicit_structuring_kwargs,
    }
    painting_trajectory = structure_painting_trajectory(
        painted,
        frame_id=options.get("frame_id", "world"),
    )

    csv_files = {}
    if bool(options.get("save_csv", False)):
        csv_files = save_painting_trajectory_csv_files(
            painting_trajectory,
            csv_output_dir=options.get("csv_output_dir", ""),
            painting_trajectory_csv_name=options.get(
                "painting_trajectory_csv_name",
                "painting_trajectory.csv",
            ),
            trajectory_csv_name_format=options.get(
                "trajectory_csv_name_format",
                "trajectory_{row_index:03d}.csv",
            ),
        )
    return painting_trajectory, csv_files


def _stage4_kwargs_with_legacy_structuring_raster_zigzag(
    paint_spline_kwargs,
    spline_kwargs,
    structuring_kwargs,
):
    effective_paint_spline_kwargs = dict(paint_spline_kwargs or {})
    effective_spline_kwargs = dict(spline_kwargs or {})
    source = dict(structuring_kwargs or {})
    if (
        "raster_zigzag" in source
        and "raster_zigzag" not in effective_paint_spline_kwargs
        and "raster_zigzag" not in effective_spline_kwargs
    ):
        effective_spline_kwargs["raster_zigzag"] = bool(source["raster_zigzag"])
    return effective_paint_spline_kwargs, effective_spline_kwargs


def _generate_stage4_result(
    point_cloud_xyz,
    config=None,
    config_path=None,
    overrides=None,
    extraction_kwargs=None,
    correction_kwargs=None,
    offset_kwargs=None,
    paint_spline_kwargs=None,
    spline_kwargs=None,
    return_config=False,
):
    paint_config = _resolve_painting_trajectory_config(
        config=config,
        config_path=config_path,
        overrides=overrides,
    )

    extraction_kwargs = {
        **paint_config.surface_extraction.to_kwargs(),
        **dict(extraction_kwargs or {}),
    }
    correction_kwargs = {
        **paint_config.correction.to_kwargs(),
        **dict(correction_kwargs or {}),
    }
    offset_kwargs = {
        **paint_config.offset.to_kwargs(),
        **dict(offset_kwargs or {}),
    }
    paint_spline_kwargs = {
        **paint_config.spline.to_kwargs(),
        **dict(paint_spline_kwargs or {}),
        **dict(spline_kwargs or {}),
    }

    extraction = generate_surface_spline_rows(point_cloud_xyz, **extraction_kwargs)
    corrected = correct_surface_spline_rows(extraction, **correction_kwargs)
    offset = generate_offset_rows(corrected, **offset_kwargs)
    painted = generate_paint_spline_rows(offset, **paint_spline_kwargs)
    if return_config:
        return painted, paint_config
    return painted


def generate_painting_trajectory_debug(
    point_cloud_xyz,
    config=None,
    config_path=None,
    overrides=None,
    extraction_kwargs=None,
    correction_kwargs=None,
    offset_kwargs=None,
    paint_spline_kwargs=None,
    spline_kwargs=None,
    structuring_kwargs=None,
):
    paint_spline_kwargs, spline_kwargs = _stage4_kwargs_with_legacy_structuring_raster_zigzag(
        paint_spline_kwargs,
        spline_kwargs,
        structuring_kwargs,
    )
    painted, paint_config = _generate_stage4_result(
        point_cloud_xyz,
        config=config,
        config_path=config_path,
        overrides=overrides,
        extraction_kwargs=extraction_kwargs,
        correction_kwargs=correction_kwargs,
        offset_kwargs=offset_kwargs,
        paint_spline_kwargs=paint_spline_kwargs,
        spline_kwargs=spline_kwargs,
        return_config=True,
    )
    painting_trajectory, csv_files = _structure_and_export_painting_trajectory(
        painted,
        paint_config.structuring,
        structuring_kwargs=structuring_kwargs,
    )
    debug_result = dict(painted)
    debug_result["painting_trajectory"] = painting_trajectory
    debug_result["painting_trajectory_csv_files"] = csv_files
    return debug_result


def generate_painting_trajectory(
    point_cloud_xyz,
    config=None,
    config_path=None,
    overrides=None,
    extraction_kwargs=None,
    correction_kwargs=None,
    offset_kwargs=None,
    paint_spline_kwargs=None,
    spline_kwargs=None,
    structuring_kwargs=None,
):
    paint_spline_kwargs, spline_kwargs = _stage4_kwargs_with_legacy_structuring_raster_zigzag(
        paint_spline_kwargs,
        spline_kwargs,
        structuring_kwargs,
    )
    painted, paint_config = _generate_stage4_result(
        point_cloud_xyz,
        config=config,
        config_path=config_path,
        overrides=overrides,
        extraction_kwargs=extraction_kwargs,
        correction_kwargs=correction_kwargs,
        offset_kwargs=offset_kwargs,
        paint_spline_kwargs=paint_spline_kwargs,
        spline_kwargs=spline_kwargs,
        return_config=True,
    )
    painting_trajectory, _ = _structure_and_export_painting_trajectory(
        painted,
        paint_config.structuring,
        structuring_kwargs=structuring_kwargs,
    )
    return painting_trajectory


def main() -> int:
    import argparse
    import webbrowser

    parser = argparse.ArgumentParser(description="부품 PLY/XYZ CSV에서 도장 경로를 생성합니다.")
    parser.add_argument("--input", required=True, type=Path, help="입력 부품 PLY 또는 XYZ CSV")
    parser.add_argument("--config", type=Path, default=Path(PROJECT_DIR) / "config/painting_path.yaml")
    parser.add_argument("--paint-dir", required=True, type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument("--scan-dir", required=True, type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument("--output-dir", type=Path, help="생략하면 log/<실행일시>에 저장")
    parser.add_argument("--visualize", action="store_true", help="단계별 디버그 HTML도 저장")
    parser.add_argument("--open", action="store_true", help="디버그 HTML을 저장하고 브라우저로 열기")
    args = parser.parse_args()

    from painting_path_planner.visualization import load_surface_data, write_debug_plotly_html

    input_path = args.input.expanduser().resolve()
    config = load_painting_trajectory_config(args.config.expanduser().resolve())
    points, colors = load_surface_data(input_path)
    output_dir = (args.output_dir or Path(_default_csv_output_dir())).expanduser().resolve()
    # 기존 결과를 조용히 덮어쓰거나 남은 row CSV와 섞지 않습니다.
    if output_dir.exists() and any(output_dir.iterdir()):
        parser.error(f"출력 디렉토리가 비어 있지 않습니다. 새 경로를 지정하세요: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    config = config.copy_with_overrides({"structuring": {"save_csv": True, "csv_output_dir": str(output_dir)}})
    result = generate_painting_trajectory_debug(
        points, config=config,
        extraction_kwargs={"paint_dir": args.paint_dir, "scan_dir": args.scan_dir},
    )
    (output_dir / "painting_path.yaml").write_text(yaml.safe_dump({
        "project": {"input": str(input_path), "paint_dir": args.paint_dir, "scan_dir": args.scan_dir},
        "painting_trajectory": config.to_dict(),
    }, allow_unicode=True, sort_keys=False), encoding="utf-8")
    trajectory = result["painting_trajectory"]
    print(f"완료: {len(trajectory.rows)} rows / {sum(len(row.points) for row in trajectory.rows)} points")
    print(f"저장: {output_dir}")
    if args.visualize or args.open:
        html = write_debug_plotly_html(
            {"name": input_path.stem, "output_dir": output_dir}, points, colors, result,
        )
        print(f"시각화: {html}")
        if args.open:
            webbrowser.open(html.as_uri())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
