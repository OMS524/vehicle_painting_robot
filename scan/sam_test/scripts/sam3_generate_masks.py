#!/usr/bin/env python3
"""원본 RGB -> SAM 3 마스크. vehicle_painting_robot_sam3 환경에서 실행.

상단 RGB_IMAGE_PATHS/PROMPT를 수정하고 실행한다. 카메라/로봇은 연결하지 않는다.
mask.png 옆 mask.json에는 다음 단계에서 사용할 원본 파일 경로/해시가 저장된다.
모델 가중치는 CHECKPOINT_PATH로 지정하거나, HF 접근 승인/로그인 후 자동 로드한다.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent

# ======================== 사용자가 수정할 설정 ========================
RGB_IMAGE_PATHS: list[str | Path] = [
    # "/.../log/촬영일시/view_01/color_rgb.png",
    # "/.../log/촬영일시/view_02/color_rgb.png",
]
PROMPT = "car door"
OUTPUT_ROOT = HERE.parent / "log" / "sam3_masks"
CHECKPOINT_PATH: str | Path | None = None  # None: Hugging Face 다운로드/캐시 사용
DEVICE = "cuda"
CONFIDENCE_THRESHOLD = 0.5
MASK_SELECTION = "highest_score"  # 단일 부품: highest_score / 모든 후보 합집합: all
# 모든 후보도 candidates/에 별도로 저장하므로 잘못 선택되면 다른 후보를 사용 가능.
# =====================================================================


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def source_record(path: Path, output_dir: Path) -> dict | None:
    if not path.is_file():
        return None
    return {"path": os.path.relpath(path, output_dir), "sha256": sha256(path)}


def validate_images(paths: list[str | Path]) -> list[Path]:
    from PIL import Image

    if not paths:
        raise ValueError("상단 RGB_IMAGE_PATHS에 color_rgb.png 경로를 넣으세요.")
    resolved = [Path(p).expanduser().resolve() for p in paths]
    if len(set(resolved)) != len(resolved):
        raise ValueError("RGB_IMAGE_PATHS에 중복 이미지가 있습니다.")
    for path in resolved:
        if path.name == "color_aligned_depth.png":
            raise ValueError(f"Depth 정렬 RGB 대신 원본 color_rgb.png를 사용하세요: {path}")
        with Image.open(path) as image:
            image.verify()
    return resolved


def validate_settings() -> None:
    if not PROMPT.strip():
        raise ValueError("PROMPT가 비어 있습니다.")
    if not 0 <= CONFIDENCE_THRESHOLD <= 1:
        raise ValueError("CONFIDENCE_THRESHOLD는 0~1이어야 합니다.")
    if MASK_SELECTION not in {"highest_score", "all"}:
        raise ValueError("MASK_SELECTION은 highest_score 또는 all이어야 합니다.")
    if DEVICE not in {"cuda", "cpu"}:
        raise ValueError("DEVICE는 cuda 또는 cpu로 지정하세요.")
    if CHECKPOINT_PATH is not None and not Path(CHECKPOINT_PATH).expanduser().is_file():
        raise FileNotFoundError(f"가중치 파일을 찾을 수 없습니다: {CHECKPOINT_PATH}")


def import_model_api():
    # upstream SAM 3가 pkg_resources를 직접 import한다. 라이브러리 코드는 수정하지 않음.
    if importlib.util.find_spec("pkg_resources") is None:
        raise RuntimeError(
            "SAM 3가 요구하는 pkg_resources가 없습니다. SAM 3 환경에서 실행하세요:\n"
            '  python -m pip install "setuptools>=68,<82"'
        )
    import torch
    from sam3.model_builder import build_sam3_image_model
    from sam3.model.sam3_image_processor import Sam3Processor

    if DEVICE.startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError("PyTorch에서 CUDA GPU를 사용할 수 없습니다. GPU/드라이버 설치를 확인하세요.")
    return torch, build_sam3_image_model, Sam3Processor


def numpy_output(value):
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy()
    import numpy as np
    return np.asarray(value)


def save_result(image_path: Path, image, result: dict, output_dir: Path) -> dict:
    import numpy as np
    from PIL import Image, ImageDraw

    scores = numpy_output(result["scores"]).reshape(-1)
    masks = numpy_output(result["masks"])
    boxes = numpy_output(result["boxes"]).reshape(-1, 4)
    if masks.ndim == 4 and masks.shape[1] == 1:
        masks = masks[:, 0]
    width, height = image.size
    if masks.size == 0:
        masks = np.zeros((0, height, width), dtype=bool)
    if masks.shape != (len(scores), height, width) or len(boxes) != len(scores):
        raise ValueError(f"SAM 3 출력 크기가 RGB와 다릅니다: {masks.shape}, RGB={image.size}")
    if not np.isfinite(scores).all() or not np.isfinite(boxes).all():
        raise ValueError("SAM 3 출력 점수/박스에 비정상 값이 있습니다.")
    masks = masks.astype(bool)
    candidates = [int(i) for i in np.argsort(-scores)
                  if scores[i] >= CONFIDENCE_THRESHOLD and masks[i].any()]
    selected = candidates[:1] if MASK_SELECTION == "highest_score" else candidates
    combined = np.any(masks[selected], axis=0) if selected else np.zeros((height, width), bool)

    output_dir.mkdir(parents=True, exist_ok=False)
    view_dir = image_path.parent
    sources = {key: source_record(path, output_dir) for key, path in {
        "rgb": image_path, "depth": view_dir / "depth_raw.npy",
        "camera": view_dir / "camera.json", "pose": view_dir / "pose.json",
    }.items()}
    metadata = {
        "schema": "sam3_rgb_mask_v1", "coordinate_space": "native_color_pixels",
        "image_size": [width, height], "prompt": PROMPT,
        "confidence_threshold": CONFIDENCE_THRESHOLD, "selection": MASK_SELECTION,
        "selected_indices": selected, "sources": sources,
        "candidates": [{"index": i, "score": float(scores[i]),
                        "box_xyxy": boxes[i].tolist(), "pixel_count": int(masks[i].sum())}
                       for i in candidates],
        "checkpoint": str(Path(CHECKPOINT_PATH).expanduser().resolve()) if CHECKPOINT_PATH else "HF:facebook/sam3",
        "status": "ok" if selected else "no_detection",
    }
    Image.fromarray(combined.astype(np.uint8) * 255).save(output_dir / "mask.png")
    write_json(output_dir / "mask.json", metadata)
    # 원본 크기를 유지한 확인용 오버레이. 추출 입력으로는 overlay가 아닌 mask 사용.
    rgb = np.asarray(image).copy()
    rgb[combined] = (rgb[combined] * 0.55 + np.array([0, 220, 160]) * 0.45).astype(np.uint8)
    overlay = Image.fromarray(rgb)
    draw = ImageDraw.Draw(overlay)
    for i in candidates:
        box = boxes[i].tolist()
        color = "lime" if i in selected else "orange"
        draw.rectangle(box, outline=color, width=3)
        draw.text((max(0, box[0]), max(0, box[1] - 15)), f"#{i} {scores[i]:.3f}", fill=color)
    overlay.save(output_dir / "overlay.png")

    candidate_dir = output_dir / "candidates"
    if candidates:
        candidate_dir.mkdir()
    for i in candidates:
        path = candidate_dir / f"mask_{i:03d}.png"
        Image.fromarray(masks[i].astype(np.uint8) * 255).save(path)
        candidate_metadata = dict(metadata, selection="individual", selected_indices=[i])
        candidate_metadata["sources"] = {
            key: dict(record, path=os.path.relpath((output_dir / record["path"]).resolve(), candidate_dir))
            if record else None for key, record in sources.items()
        }
        write_json(path.with_suffix(".json"), candidate_metadata)
    return {"rgb": str(image_path), "mask": str(output_dir / "mask.png"),
            "candidate_count": len(candidates), "selected_indices": selected,
            "mask_pixels": int(combined.sum()), "status": metadata["status"]}


def run(paths: list[str | Path] | None = None, *, check_only: bool = False) -> Path | None:
    from PIL import Image

    validate_settings()
    images = validate_images(RGB_IMAGE_PATHS if paths is None else paths)
    torch, build_model, processor_class = import_model_api()
    print(f"Python: {sys.executable}\nRGB: {len(images)}개 / prompt: {PROMPT}", flush=True)
    if check_only:
        print("입력/API/GPU 확인 완료. 모델 가중치 로드나 추론은 실행하지 않았습니다.")
        return None
    checkpoint = str(Path(CHECKPOINT_PATH).expanduser().resolve()) if CHECKPOINT_PATH else None
    model = build_model(checkpoint_path=checkpoint, load_from_HF=checkpoint is None, device=DEVICE)
    processor = processor_class(model, device=DEVICE, confidence_threshold=CONFIDENCE_THRESHOLD)
    output = Path(OUTPUT_ROOT).expanduser().resolve() / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True, exist_ok=False)
    manifest = {"status": "running", "python": sys.executable, "prompt": PROMPT, "views": []}
    write_json(output / "manifest.json", manifest)
    try:
        for index, path in enumerate(images, 1):
            print(f"[{index}/{len(images)}] {path}", flush=True)
            with Image.open(path) as loaded:
                image = loaded.convert("RGB")
            with torch.inference_mode():
                state = processor.set_image(image)
                result = processor.set_text_prompt(state=state, prompt=PROMPT)
            entry = save_result(path, image, result, output / f"{index:02d}_{path.parent.name}")
            manifest["views"].append(entry)
            write_json(output / "manifest.json", manifest)
            print(f"  후보 {entry['candidate_count']}개, 선택 {entry['selected_indices']}: {entry['mask']}", flush=True)
            if entry["candidate_count"] != 1:
                print("  주의: 후보가 0개 또는 여러 개입니다. overlay.png와 candidates/를 확인하세요.", flush=True)
            del state, result
        manifest["status"] = "complete"
    except Exception as error:
        manifest.update(status="failed", error=str(error))
        raise
    finally:
        write_json(output / "manifest.json", manifest)
    print(f"완료: {output}\n두 번째 코드의 MASK_PATHS에 각 mask.png 경로를 넣으세요.")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="모델/가중치 로드 없이 입력/API/GPU 확인")
    args = parser.parse_args()
    try:
        run(check_only=args.check)
    except Exception as error:
        print(f"오류: {error}", file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
