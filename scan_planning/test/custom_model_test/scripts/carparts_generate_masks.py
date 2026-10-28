#!/usr/bin/env python3
# 실행 (어느 디렉토리에서나 가능):
#   conda activate vehicle_painting_robot_custom_model
#   python -s /home/oms/vehicle_painting_robot/scan/custom_model_test/scripts/carparts_generate_masks.py
# 모델/이미지를 명령행에서 지정 (생략하면 아래 사용자 파라미터 사용):
#   python -s /home/oms/vehicle_painting_robot/scan/custom_model_test/scripts/carparts_generate_masks.py --model classes12 --images /경로/view_01/color_rgb.png /경로/view_02/color_rgb.png
# 입력 검사만 (모델 로드/추론/결과 저장 없음): 위 명령 끝에 --check
"""차량 부품 모델 4종 중 하나로 RGB 이미지의 세그멘테이션 결과를 저장한다.

카메라/로봇 연결, 학습, 모델 다운로드 없음. 원본 이미지와 가중치는 수정하지 않는다.
각 이미지마다 overlay.png, 원본 해상도 mask.png, instances/*.png, detections.json 저장.
mask.png는 선택 클래스의 모든 검출 영역의 합집합이며, 개별 객체는 instances/에 저장한다.
이 결과 메타데이터는 기존 SAM 전용 extract_masked_depth.py의 입력 형식과는 별개이다.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
MODEL_ROOT = HERE.parent / "Car_Painting"
SCAN_ROOT = HERE.parents[1]
MODELS = {
    "baseline": MODEL_ROOT / "runs/carparts_yolo11n_seg/weights/best.pt",
    "fender_refresh": MODEL_ROOT / "runs/carparts_fender_refresh_20261009/weights/best.pt",
    "combined_augmentation": MODEL_ROOT / "runs/carparts_combined_augmentation_20261011/weights/best.pt",
    "classes12": MODEL_ROOT / "runs/carparts_12classes_20261012/weights/best.pt",
}

# ======================== 사용자가 수정할 파라미터 ========================
# baseline: 기본 24종 / fender_refresh: 휀더 라벨 정제 후 24종
# combined_augmentation: 통합 증강 24종 / classes12: 클래스 통합 12종
MODEL_NAME = "combined_augmentation"
RGB_IMAGE_PATHS: list[str | Path] = [
    SCAN_ROOT / "custom_model_test/images/1_front.png",
    SCAN_ROOT / "custom_model_test/images/1_left.png",
    SCAN_ROOT / "custom_model_test/images/1_right.png",
    SCAN_ROOT / "custom_model_test/images/2_bottom_front.png",
    SCAN_ROOT / "custom_model_test/images/2_bottom_left.png",
    SCAN_ROOT / "custom_model_test/images/2_bottom_right.png",
    SCAN_ROOT / "custom_model_test/images/2_middle_front.png",
    SCAN_ROOT / "custom_model_test/images/2_middle_left.png",
    SCAN_ROOT / "custom_model_test/images/2_middle_right.png",
    SCAN_ROOT / "custom_model_test/images/2_top_front.png",
    SCAN_ROOT / "custom_model_test/images/2_top_left.png",
    SCAN_ROOT / "custom_model_test/images/2_top_right.png",
]
OUTPUT_ROOT = HERE.parent / "log"
DEVICE = "auto"  # auto: CUDA 가능하면 0, 아니면 cpu. 명시적 선택: "0" / "cpu"
CONFIDENCE_THRESHOLD = 0.25
IMAGE_SIZE = 640  # 모델 입력 크기. 저장되는 마스크는 원본 RGB 해상도.
CLASS_NAMES: list[str] | None = None  # None: 전체. 12종 모델 예: ["front_door", "back_door"]
# =========================================================================


def write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                    encoding="utf-8")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=tuple(MODELS), default=MODEL_NAME)
    parser.add_argument("--images", nargs="+", type=Path, default=RGB_IMAGE_PATHS)
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    parser.add_argument("--device", default=DEVICE, help="auto / cpu / 0 등 CUDA 장치 번호")
    parser.add_argument("--conf", type=float, default=CONFIDENCE_THRESHOLD)
    parser.add_argument("--imgsz", type=int, default=IMAGE_SIZE)
    parser.add_argument("--classes", nargs="+", default=CLASS_NAMES, help="추론할 클래스 이름 (생략: 상단 설정)")
    parser.add_argument("--check", action="store_true", help="입력 검사만. 모델 로드/추론/저장 없음")
    return parser.parse_args(argv)


def validate_inputs(args):
    from PIL import Image

    if args.model not in MODELS:
        raise ValueError(f"MODEL_NAME은 {', '.join(MODELS)} 중 하나여야 합니다.")
    checkpoint = MODELS[args.model].resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(f"학습 가중치가 없습니다: {checkpoint}")
    if not math.isfinite(args.conf) or not 0 <= args.conf <= 1:
        raise ValueError("CONFIDENCE_THRESHOLD/--conf는 0~1의 유한한 값이어야 합니다.")
    if type(args.imgsz) is not int or args.imgsz < 1:
        raise ValueError("IMAGE_SIZE/--imgsz는 양의 정수여야 합니다.")
    if args.classes is not None and (not isinstance(args.classes, list) or not args.classes
                                   or any(not isinstance(name, str) or not name.strip() for name in args.classes)):
        raise ValueError("CLASS_NAMES는 None 또는 비어 있지 않은 클래스 이름 리스트여야 합니다.")
    if not isinstance(args.images, (list, tuple)) or not args.images:
        raise ValueError("RGB_IMAGE_PATHS/--images에 입력 이미지 경로를 넣으세요.")
    images = [Path(path).expanduser().resolve() for path in args.images]
    if len(set(images)) != len(images):
        raise ValueError("입력 이미지 경로가 중복되었습니다.")
    for path in images:
        if not path.is_file():
            raise FileNotFoundError(f"입력 이미지가 없습니다: {path}")
        with Image.open(path) as image:
            image.verify()
    return checkpoint, images


def save_result(result, source: Path, output: Path, model_name: str) -> dict:
    import numpy as np
    from PIL import Image

    height, width = result.orig_shape
    count = len(result.boxes) if result.boxes is not None else 0
    if result.masks is None:
        if count:
            raise ValueError("객체는 검출됐지만 마스크가 없습니다. 세그멘테이션 가중치를 확인하세요.")
        masks = np.zeros((0, height, width), dtype=bool)
    else:
        values = result.masks.data.detach().cpu().numpy()
        if values.shape != (count, height, width) or not np.isfinite(values).all():
            raise ValueError(f"마스크 크기/값 오류: {values.shape}, 원본={(height, width)}")
        masks = values > 0.5
    combined = np.any(masks, axis=0)
    detections = []
    output.mkdir(parents=True, exist_ok=False)
    if count:
        (output / "instances").mkdir()
    for index, box in enumerate(result.boxes if result.boxes is not None else []):
        class_id = int(box.cls.item())
        filename = f"instances/mask_{index:03d}.png"
        Image.fromarray(masks[index].astype(np.uint8) * 255).save(output / filename)
        detections.append({
            "index": index, "class_id": class_id, "class_name": result.names[class_id],
            "confidence": float(box.conf.item()), "box_xyxy": box.xyxy[0].tolist(),
            "pixel_count": int(masks[index].sum()), "mask": filename,
        })
    Image.fromarray(combined.astype(np.uint8) * 255).save(output / "mask.png")
    # Ultralytics plot()은 BGR ndarray를 반환한다. PIL 저장 전에 RGB로 변환.
    overlay_bgr = result.plot(labels=True, boxes=True, conf=True, pil=False)
    Image.fromarray(overlay_bgr[..., ::-1]).save(output / "overlay.png")
    report = {
        "schema": "carparts_rgb_mask_v1", "coordinate_space": "native_color_pixels",
        "model": model_name, "source_rgb": str(source), "image_size": [width, height],
        "mask_selection": "union_of_detected_instances", "detection_count": count,
        "status": "ok" if combined.any() else "no_detection",
        "mask_pixels": int(combined.sum()), "detections": detections,
        "inference_speed_ms": result.speed,
    }
    write_json(output / "detections.json", report)
    return {"source_rgb": str(source), "output": str(output),
            "detection_count": count, "status": report["status"],
            "classes": sorted({entry["class_name"] for entry in detections})}


def run(args):
    checkpoint, images = validate_inputs(args)
    print(f"Python: {sys.executable}\n모델: {args.model}\n가중치: {checkpoint}\n입력 RGB: {len(images)}개")
    if args.check:
        print("입력 검사 완료. 모델 로드/추론/결과 저장은 하지 않았습니다.")
        return None

    import torch
    import ultralytics
    from PIL import Image
    from ultralytics import YOLO

    device = ("0" if torch.cuda.is_available() else "cpu") if args.device == "auto" else args.device
    model = YOLO(str(checkpoint))
    if model.task != "segment":
        raise ValueError(f"세그멘테이션 모델이 아닙니다: task={model.task}")
    names = model.names
    selected_ids = None
    if args.classes is not None:
        unknown = sorted(set(args.classes) - set(names.values()))
        if unknown:
            raise ValueError(f"모델에 없는 클래스: {unknown}. 사용 가능: {list(names.values())}")
        selected_ids = [key for key, name in names.items() if name in args.classes]
    output = Path(args.output_root).expanduser().resolve() / args.model / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True, exist_ok=False)
    summary = {"status": "running", "model": args.model, "checkpoint": str(checkpoint),
               "ultralytics_version": ultralytics.__version__, "device": device,
               "confidence_threshold": args.conf, "image_size": args.imgsz,
               "class_filter": args.classes, "model_classes": names, "images": []}
    write_json(output / "summary.json", summary)
    print(f"장치: {device}\n저장: {output}")
    started = time.perf_counter()
    try:
        for index, path in enumerate(images, 1):
            print(f"[{index}/{len(images)}] {path}")
            with Image.open(path) as image:
                # PIL RGB를 전달해 채널 순서를 명확히 하고 원본 픽셀 좌표를 유지한다.
                rgb = image.convert("RGB")
                result = model.predict(source=rgb, imgsz=args.imgsz, conf=args.conf,
                                       device=device, classes=selected_ids, retina_masks=True,
                                       save=False, verbose=False)[0]
            # 모든 뷰의 파일명이 color_rgb.png여도 서로 덮어쓰지 않는다.
            target = output / f"{index:03d}_{path.parent.name}_{path.stem}"
            entry = save_result(result, path, target, args.model)
            summary["images"].append(entry)
            write_json(output / "summary.json", summary)
            print(f"  검출 {entry['detection_count']}개 / {entry['classes']} -> {target / 'overlay.png'}")
            if entry["status"] == "no_detection":
                print("  주의: 검출 영역이 없습니다. mask.png는 비어 있는 마스크입니다.")
        summary["status"] = "complete"
    except Exception as error:
        summary.update(status="failed", error=str(error))
        raise
    finally:
        summary["elapsed_seconds"] = time.perf_counter() - started
        write_json(output / "summary.json", summary)
    print(f"완료: {output}")
    return output


def main(argv=None):
    args = parse_args(argv)
    try:
        run(args)
    except Exception as error:
        print(f"오류: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
