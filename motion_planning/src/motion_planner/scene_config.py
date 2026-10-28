"""config/scene.yaml 로더.

launch(시각화)와 viz IK 솔버가 같은 부품 배치를 쓰도록 설정을 한 곳에서 읽는다.
둘이 어긋나면 RViz의 부품 위치와 로봇 동작이 서로 다른 곳을 가리키게 된다.
"""

import os

import numpy as np
import yaml


DEFAULTS = {
    "part_file": "",
    "trajectory_dir": "",
    "part_xyz": [0.0, 0.0, 0.0],
    "part_rpy": [0.0, 0.0, 0.0],
    "max_part_points": 120000,
    "max_rows": 0,
}


def load_scene_config(path):
    """scene.yaml을 읽어 dict로 돌려준다."""
    if not os.path.isfile(path):
        raise FileNotFoundError(f"scene 설정 파일이 없다: {path}")
    with open(path, "r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream) or {}
    scene = loaded.get("scene", loaded)
    if not isinstance(scene, dict):
        raise ValueError(f"scene 매핑이 없다: {path}")

    config = dict(DEFAULTS)
    config.update({key: value for key, value in scene.items() if value is not None})
    # 파일 경로: $REPO / $NLP4CM / ~ 치환, 상대경로는 yaml 파일 위치 기준 (yaml 에 /home/<user> 절대경로를 적지 않는다)
    from motion_planner.configuration import expand_project_path
    base = os.path.dirname(os.path.abspath(path))
    for key in ("part_file", "trajectory_dir"):
        value = config.get(key)
        if value:
            expanded = expand_project_path(value)
            config[key] = expanded if os.path.isabs(expanded) else os.path.normpath(os.path.join(base, expanded))
    return config


def rotation_from_rpy(rpy):
    """고정축 roll-pitch-yaw(라디안)를 3x3 회전행렬로 바꾼다."""
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def part_transform(config):
    """scene 설정에서 world←part 변환 (R, t)를 만든다."""
    rotation = rotation_from_rpy(config["part_rpy"])
    translation = np.asarray(config["part_xyz"], dtype=float).reshape(3)
    return rotation, translation
