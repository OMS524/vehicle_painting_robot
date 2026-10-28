#!/usr/bin/env python3
# 설정 검사: conda activate vehicle_painting_robot_scan
#   python -s scan_planning/scripts/scan_planning.py --check
# 새 취득 + 전체 공정: python -s scan_planning/scripts/scan_planning.py --execute
# 저장 데이터 재처리: python -s scan_planning/scripts/scan_planning.py --capture-dir /촬영/폴더 --visualize
"""상위 제어기: 뷰 취득 -> TF 재구성 -> ICP/PGO 보정 -> DBSCAN 부품 선택."""
from contextlib import redirect_stdout, redirect_stderr
from datetime import datetime, timezone
from dataclasses import asdict, fields
from pathlib import Path
import argparse
import shutil
import signal
import sys
import time
import traceback
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from scan_planner import acquisition, registration, clustering
from scan_planner.acquisition import prepare_capture, acquire


SCAN_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = SCAN_ROOT / "config/scan.yaml"


def read_yaml(path):
    value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"YAML 객체가 필요합니다: {path}")
    return value

def make_settings(cls, value, label):
    if not isinstance(value, dict):
        raise ValueError(f"{label}은 YAML 객체여야 합니다.")
    unknown = set(value) - {f.name for f in fields(cls)}
    if unknown:
        raise ValueError(f"{label}: 알 수 없는 항목 {sorted(unknown)}")
    return cls(**value)

def load_configuration(path=DEFAULT_CONFIG):
    path = Path(path).expanduser().resolve()
    cfg = acquisition.load_config(path)
    files = {"scan": path}
    if not isinstance(cfg.get("configs"), dict):
        raise ValueError("scan.yaml에 configs를 지정하세요.")
    for name in ("sdk_filters", "registration", "segmentation"):
        p = Path(cfg["configs"][name]).expanduser()
        files[name] = (path.parent / p).resolve()
    mode = cfg.get("capture_mode", "median")
    if mode not in ("median", "sdk_filters"):
        raise ValueError("capture_mode는 median / sdk_filters입니다.")
    cfg["capture_mode"] = mode
    root = Path(cfg["output_dir"]).resolve()
    if not root.is_relative_to((SCAN_ROOT / "log").resolve()):
        raise ValueError("새 파이프라인의 output_dir는 scan_planning/log 또는 그 하위여야 합니다.")
    filter_doc = read_yaml(files["sdk_filters"])
    if set(filter_doc) != {"depth_filters"}:
        raise ValueError("SDK 필터 YAML 최상위는 depth_filters만 사용합니다.")
    filters = acquisition.validate_filters(filter_doc["depth_filters"])
    reg = read_yaml(files["registration"])
    if set(reg) - {"enabled", "method", "reference_view", "icp", "graph"}:
        raise ValueError("registration.yaml의 알 수 없는 항목")
    if type(reg.get("enabled")) is not bool or reg.get("method") not in registration.METHOD_DIRS:
        raise ValueError("registration enabled(bool)와 지원하는 method를 지정하세요.")
    if reg.get("reference_view") is not None and not isinstance(reg["reference_view"], str):
        raise ValueError("reference_view는 뷰 이름 또는 null입니다.")
    s = make_settings(registration.Settings, reg.get("icp", {}), "icp")
    g = make_settings(registration.GraphSettings, reg.get("graph", {}), "graph")
    registration.validate_settings(s)
    registration.validate_graph_settings(g)
    seg = read_yaml(files["segmentation"])
    if set(seg) - {"method", "clustering"} or seg.get("method") != "dbscan":
        raise ValueError("현재 segmentation은 dbscan만 지원합니다.")
    cluster = make_settings(clustering.Settings, seg.get("clustering", {}), "clustering")
    clustering.validate_settings(cluster)
    return {"capture": cfg, "filters": filters, "registration": reg,
            "icp": s, "graph": g, "cluster": cluster, "files": files}

def describe(settings):
    cfg = settings["capture"]
    print(f"취득: {cfg['capture_mode']} / {len(cfg['views'])}개 뷰")
    print(f"재구성: 실측 TF -> base_link / {cfg['reconstruction']['voxel_size_mm']} mm voxel")
    reg = settings["registration"]
    print(f"보정: {reg['method']} + PGO" if reg["enabled"] else "보정: 사용 안 함")
    print(f"부품 선택: DBSCAN / eps {settings['cluster'].dbscan_eps_mm} mm")
    print(f"로그 루트: {cfg['output_dir']}")
    missing = [k for k in ("control_python", "control_wrapper", "control_library") if not cfg.get(k)]
    if missing:
        print("실장비 실행 미준비: " + ", ".join(missing) + "를 새 제어 모듈에 맞게 지정하세요.")
    if not cfg.get("mounting_tf_confirmed"):
        print("실장비 실행 미준비: 실제 장착 TF와 이동 경로 확인 후 mounting_tf_confirmed: true 필요.")
    print("설정 검사만으로 장비 연결, IK 성공, 충돌 안전을 확인한 것은 아닙니다.")


def execute_pipeline(settings, output, *, capture_dir=None, geometry_dir=None,
                     stage="all", prepared=None):
    """테스트 가능한 공정 순서. 실패한 단계 이후는 실행하지 않는다."""
    cfg = settings["capture"]
    report = {"schema": "scan_pipeline_v1", "status": "running", "stage": stage,
              "started_at": datetime.now(timezone.utc).isoformat(), "python": sys.executable,
              "steps": [], "output": str(output), "config_files": {},
              "source_capture": str(capture_dir) if capture_dir else None,
              "source_geometry": str(geometry_dir) if geometry_dir else None}
    configs = output / "config"
    configs.mkdir()
    for name, source in settings["files"].items():
        target = configs / source.name
        shutil.copyfile(source, target)
        report["config_files"][name] = {"source": str(source), "snapshot": str(target.relative_to(output)),
                                        "sha256": registration.sha256(target)}
    registration.save_json(output / "resolved_scan_settings.json", cfg)
    registration.save_json(output / "resolved_settings.json", {
        "capture": cfg, "sdk_filters": settings["filters"],
        "registration": settings["registration"], "icp": asdict(settings["icp"]),
        "graph": asdict(settings["graph"]), "clustering": asdict(settings["cluster"]),
    })
    started = time.perf_counter()

    def step(name, fn):
        record = {"name": name, "status": "running"}
        report["steps"].append(record)
        registration.save_json(output / "manifest.json", report)
        tick = time.perf_counter()
        print(f"\n[{name}] 시작", flush=True)
        try:
            result = fn()
        except BaseException:
            record.update(status="failed", elapsed_sec=time.perf_counter() - tick)
            raise
        record.update(status="complete", output=str(result), elapsed_sec=time.perf_counter() - tick)
        registration.save_json(output / "manifest.json", report)
        return result

    try:
        if stage != "segmentation":
            if capture_dir is None:
                if prepared is None:
                    raise ValueError("실장비 취득은 --execute와 사전 검사/Enter 확인이 필요합니다.")
                capture_dir = step("01_capture", lambda: acquire(settings, output / "01_capture", *prepared))
            else:
                report["steps"].append({"name": "01_capture", "status": "reused",
                                        "output": str(capture_dir)})
            report["source_capture"] = str(capture_dir)
            if stage == "capture":
                report.update(status="complete", result=str(capture_dir))
                return report
            geometry_dir = step("02_reconstruction", lambda: registration.reconstruct(
                capture_dir, output / "02_reconstruction", cfg["reconstruction"]["voxel_size_mm"]))
            reg = settings["registration"]
            if reg["enabled"]:
                geometry_dir = step("03_registration", lambda: registration.run(
                    reg["method"], geometry_dir, output / "03_registration",
                    reg.get("reference_view"), settings["icp"], settings["graph"]))
                review = registration.read_json(geometry_dir / "summary.json")
                report["registration_status"] = review["status"]
                report["registration_warnings"] = review["warnings"]
            else:
                report["steps"].append({"name": "03_registration", "status": "disabled"})
            report["geometry"] = str(geometry_dir)
            if stage == "registration":
                report.update(status="complete", result=str(geometry_dir / "merged.ply"))
                return report
        if geometry_dir is None:
            raise ValueError("부품 선택에는 --geometry-dir가 필요합니다.")
        selected = step("04_segmentation", lambda: clustering.run(
            geometry_dir, output / "04_segmentation", settings=settings["cluster"]))
        summary = registration.read_json(selected / "summary.json")
        report["segmentation_status"] = summary["status"]
        report["segmentation_warnings"] = summary["warnings"]
        if summary["status"] == "complete":
            shutil.copyfile(selected / "part.ply", output / "part.ply")
            report.update(status="complete", result=str(output / "part.ply"),
                          selected_points=summary["selected_points"],
                          requires_visual_review=True)
        else:
            report.update(status="needs_selection", result=str(selected / "clusters_colored.ply"))
            print("부품 자동 선택 실패: 군집 결과를 확인하고 segmentation.yaml의 기준점/군집 ID를 지정하세요.")
        return report
    except BaseException:
        report.update(status="failed", error=traceback.format_exc())
        (output / "error.log").write_text(report["error"], encoding="utf-8")
        print(report["error"], file=sys.stderr)
        raise
    finally:
        report.update(finished_at=datetime.now(timezone.utc).isoformat(),
                      elapsed_sec=time.perf_counter() - started)
        registration.save_json(output / "manifest.json", report)


def main(argv=None, default_stage="all"):
    def on_terminate(_signum, _frame):
        raise KeyboardInterrupt("스캔 중단 요청")
    signal.signal(signal.SIGTERM, on_terminate)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--stage", choices=("all", "capture", "registration", "segmentation"),
                        default=default_stage)
    inputs = parser.add_mutually_exclusive_group()
    inputs.add_argument("--execute", action="store_true", help="실제 이동/취득. 별도 Enter 확인 필요")
    inputs.add_argument("--capture-dir", type=Path, help="저장된 원본 촬영 폴더로 재구성부터 실행")
    inputs.add_argument("--geometry-dir", type=Path, help="재구성/PGO 폴더에서 클러스터링만 실행")
    parser.add_argument("--check", action="store_true", help="설정/저장 입력 검사만. 장비 연결·저장 없음")
    parser.add_argument("--visualize", action="store_true", help="완료 후 최종 PLY 시각화")
    args = parser.parse_args(argv)
    try:
        settings = load_configuration(args.config)
        describe(settings)
        if args.geometry_dir and args.stage not in ("all", "segmentation"):
            parser.error("--geometry-dir는 segmentation 단계에서만 사용합니다.")
        stage = "segmentation" if args.geometry_dir else args.stage
        if stage == "segmentation" and not args.geometry_dir:
            parser.error("클러스터링만 실행하려면 --geometry-dir를 지정하세요.")
        if stage == "registration" and not args.capture_dir:
            parser.error("정합만 실행하려면 --capture-dir를 지정하세요.")
        if args.capture_dir and stage == "capture":
            parser.error("capture 단계는 --execute를 사용하세요.")
        prepared = None
        if args.check:
            if args.capture_dir:
                import open3d as o3d
                min_views = 2 if settings["registration"]["enabled"] else 1
                views, _ = registration.load_capture(args.capture_dir, settings["registration"]["method"], o3d,
                                            min_views=min_views)
                reference = settings["registration"].get("reference_view")
                if reference is not None and reference not in [v["name"] for v in views]:
                    raise ValueError(f"기준 뷰가 없습니다: {reference}")
            if args.geometry_dir:
                clustering.run(args.geometry_dir, None, settings=settings["cluster"], check_only=True)
            print("검사 완료. 하드웨어 연결/결과 저장 없음.")
            return 0
        if not any((args.execute, args.capture_dir, args.geometry_dir)):
            print("설정 검사 완료. 촬영: --execute / 저장 데이터: --capture-dir 경로")
            return 0
        if args.execute:
            prepared = prepare_capture(settings)
            from scan_planner.acquisition import target_description
            for view in settings["capture"]["views"]:
                print(view["name"], target_description(view, settings["capture"]))
            print("주의: 실제 로봇 이동/서보 활성화. 이동 구간 충돌 검사는 없습니다.\n"
                  "장착·툴 무게·전체 이동 경로·주변 안전·비상정지를 확인하세요. 종료 시 servo-off.")
            if input("시작하려면 Enter, 취소하려면 다른 문자를 입력하세요: ").strip():
                print("취소했습니다. 장비에 연결하지 않았습니다.")
                return 0
        output = Path(settings["capture"]["output_dir"]) / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        output.mkdir(parents=True, exist_ok=False)
        capture_dir = args.capture_dir.expanduser().resolve() if args.capture_dir else None
        geometry_dir = args.geometry_dir.expanduser().resolve() if args.geometry_dir else None
        with (output / "run.log").open("w", encoding="utf-8") as stream:
            with redirect_stdout(registration.Tee(sys.stdout, stream)), redirect_stderr(registration.Tee(sys.stderr, stream)):
                print(f"저장: {output}", flush=True)
                report = execute_pipeline(settings, output, capture_dir=capture_dir,
                                          geometry_dir=geometry_dir, stage=stage, prepared=prepared)
        print(f"완료 상태: {report['status']} / 결과: {report.get('result', output)}")
        if args.visualize and stage != "capture":
            from scan_planner.acquisition import show_cloud
            show_cloud(Path(report["result"]))
        return 0 if report["status"] == "complete" else 2
    except KeyboardInterrupt:
        print("사용자 중단. 로봇의 실제 정지 상태를 확인하세요.", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"오류: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt("스캔 중단 요청")
    signal.signal(signal.SIGTERM, interrupt)
    raise SystemExit(main())
