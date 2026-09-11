#!/usr/bin/env python3
"""Register the supplied bracket/camera STLs without changing robot models.

Internal registration units: mm. Matrix output units: m.
T_A_from_B maps column-vector coordinates in B into A.
This is CAD-mesh registration, NOT a physical hand-eye calibration.
"""
import os
os.environ.setdefault("OMP_NUM_THREADS", "2")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "2")
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
import open3d as o3d
import trimesh
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = ROOT / "description" / "meshes"
OUTPUT = Path(__file__).resolve().parent / "mesh_alignment"
# Connected components sorted by descending surface area. Matched using bounds,
# area, and shape. Full-mesh distances are also measured without exclusions.
BRACKET_PAIRS = [(0, 0), (1, 2)]
CAMERA_PAIRS = [(0, 1), (1, 3), (2, 4), (4, 6), (8, 7)]


def parts(mesh):
    return sorted(mesh.split(only_watertight=False), key=lambda item: item.area, reverse=True)


def scene(mesh):
    shape = o3d.t.geometry.TriangleMesh(
        o3d.core.Tensor(np.asarray(mesh.vertices), dtype=o3d.core.Dtype.Float32),
        o3d.core.Tensor(np.asarray(mesh.faces), dtype=o3d.core.Dtype.Int64))
    result = o3d.t.geometry.RaycastingScene(nthreads=2)
    result.add_triangles(shape)
    return result


def sample(mesh, count, seed):
    return trimesh.sample.sample_surface(mesh, count, seed=seed)[0]


def batches(source_parts, target_parts, pairs, count, seed):
    return [
        (sample(source_parts[i], count, seed + k), scene(target_parts[j]))
        for k, (i, j) in enumerate(pairs)
    ]


def correspond(data, rotation, translation):
    pp, qq, nn = [], [], []
    for points, target in data:
        p = points @ rotation.T + translation
        hit = target.compute_closest_points(
            o3d.core.Tensor(p.astype(np.float32)), nthreads=2)
        pp.append(p)
        qq.append(hit["points"].numpy().astype(float))
        nn.append(hit["primitive_normals"].numpy().astype(float))
    return np.vstack(pp), np.vstack(qq), np.vstack(nn)


def distances(data, rotation, translation):
    p, q, _ = correspond(data, rotation, translation)
    return np.linalg.norm(p - q, axis=1)


def stats(d):
    return {
        "samples": len(d),
        "rmse_mm": float(np.sqrt(np.mean(d * d))),
        "median_mm": float(np.median(d)),
        "p95_mm": float(np.quantile(d, .95)),
        "max_mm": float(d.max()),
        "fraction_below_0_1_mm": float(np.mean(d < .1)),
        "fraction_below_0_5_mm": float(np.mean(d < .5)),
    }


def cube_rotations():
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((-1, 1), repeat=3):
            rotation = np.eye(3)[list(perm)] * np.asarray(signs)[:, None]
            if np.linalg.det(rotation) > .5:
                yield rotation


def matrix(rotation, translation):
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = rotation, translation
    return out


def metric_matrix(transform_mm):
    result = transform_mm.copy()
    result[:3, 3] *= .001
    return result.tolist()


def describe(transform):
    return {
        "matrix_translation_unit_m": metric_matrix(transform),
        "xyz_mm": transform[:3, 3].tolist(),
        "rpy_rad_extrinsic_xyz": Rotation.from_matrix(transform[:3, :3]).as_euler("xyz").tolist(),
        "rpy_deg_extrinsic_xyz": Rotation.from_matrix(transform[:3, :3]).as_euler("xyz", degrees=True).tolist(),
    }


def refine(data, rotation, translation):
    rotation, translation = rotation.copy(), translation.copy()
    # Point-to-triangle correspondences, robust point-to-plane least squares.
    # Rotation updates are about the moving frame origin, avoiding large lever arms.
    for iteration in range(40):
        p, q, normal = correspond(data, rotation, translation)
        distance = np.linalg.norm(p - q, axis=1)
        keep = distance < .5
        if keep.sum() < 100:
            raise RuntimeError("Insufficient nearby correspondences")
        p, q, normal = p[keep], q[keep], normal[keep]
        residual = np.einsum("ij,ij->i", normal, q - p)
        design = np.column_stack((np.cross(p - translation, normal), normal))
        weight = np.sqrt(np.minimum(1.0, .02 / np.maximum(np.abs(residual), 1e-12)))
        update, _, rank, _ = np.linalg.lstsq(
            design * weight[:, None], residual * weight, rcond=None)
        if rank != 6:
            raise RuntimeError("Degenerate registration: all six rigid DOFs are not constrained")
        rotation = Rotation.from_rotvec(update[:3]).as_matrix() @ rotation
        translation += update[3:]
        if np.linalg.norm(update[:3]) < 1e-9 and np.linalg.norm(update[3:]) < 1e-7:
            break
    return rotation, translation, iteration + 1


def fit(label, source, source_parts, target, target_parts, pairs):
    coarse = batches(source_parts, target_parts, pairs, 1000, 100)
    candidates = []
    for rotation in cube_rotations():
        translation = (
            target_parts[pairs[0][1]].bounds.mean(axis=0)
            - rotation @ source_parts[pairs[0][0]].bounds.mean(axis=0))
        error = stats(distances(coarse, rotation, translation))
        candidates.append((error["rmse_mm"], rotation, translation, error))
    candidates.sort(key=lambda item: item[0])
    _, initial_r, initial_t, _ = candidates[0]
    fitting = batches(source_parts, target_parts, pairs, 5000, 2000)
    r, t, iterations = refine(fitting, initial_r, initial_t)
    transform = matrix(r, t)
    # Readable nominal candidate: nearest tested axis rotation and 0.01 mm translation.
    rounded = matrix(initial_r, np.round(t, 2))
    validation = batches(source_parts, target_parts, pairs, 10000, 50000)
    reverse = batches(target_parts, source_parts, [(j, i) for i, j in pairs], 10000, 70000)
    reverse_transform = np.linalg.inv(transform)
    selected_source = trimesh.util.concatenate([source_parts[i] for i, _ in pairs])
    selected_target = trimesh.util.concatenate([target_parts[j] for _, j in pairs])
    weighted_forward = [(sample(selected_source, 60000, 80000), scene(selected_target))]
    weighted_reverse = [(sample(selected_target, 60000, 90000), scene(selected_source))]
    full = [(sample(source, 60000, 100000), scene(target))]
    report = {
        "component_pairs_sorted_by_area": pairs,
        "selected_source_area_fraction": float(selected_source.area / source.area),
        "iterations": iterations,
        "T_assembly_from_source_fitted": describe(transform),
        "T_assembly_from_source_rounded": describe(rounded),
        "independent_equal_component_samples_forward": stats(distances(validation, r, t)),
        "independent_equal_component_samples_reverse": stats(distances(
            reverse, reverse_transform[:3, :3], reverse_transform[:3, 3])),
        "independent_area_weighted_forward": stats(distances(weighted_forward, r, t)),
        "independent_area_weighted_reverse": stats(distances(
            weighted_reverse, reverse_transform[:3, :3], reverse_transform[:3, 3])),
        "rounded_transform_validation": stats(distances(
            validation, rounded[:3, :3], rounded[:3, 3])),
        "whole_source_to_whole_assembly": stats(distances(full, r, t)),
        "coarse_top_four": [
            {"transform": describe(matrix(cr, ct)), "error": error}
            for _, cr, ct, error in candidates[:4]
        ],
        "per_component_forward": [],
    }
    for k, (i, j) in enumerate(pairs):
        data = [(sample(source_parts[i], 20000, 120000 + k), scene(target_parts[j]))]
        report["per_component_forward"].append({
            "source": i, "target": j, "error": stats(distances(data, r, t)),
            "source_bounds_mm": source_parts[i].bounds.tolist(),
            "target_bounds_mm": target_parts[j].bounds.tolist(),
        })
    print(label, json.dumps({
        "fitted": describe(transform),
        "rounded": describe(rounded),
        "area_weighted_forward": report["independent_area_weighted_forward"],
        "area_weighted_reverse": report["independent_area_weighted_reverse"],
        "whole_source": report["whole_source_to_whole_assembly"],
    }), flush=True)
    return transform, rounded, report


def preview(assembly, bracket, camera, bracket_t, camera_t):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    reference = sample(assembly, 65000, 800)
    b = sample(bracket, 24000, 801)
    c = sample(camera, 36000, 802)
    b = b @ bracket_t[:3, :3].T + bracket_t[:3, 3]
    c = c @ camera_t[:3, :3].T + camera_t[:3, 3]
    fig, axes = plt.subplots(2, 3, figsize=(15, 9), facecolor="#202329")
    for col, (i, j, title) in enumerate([(0, 1, "XY"), (0, 2, "XZ"), (1, 2, "YZ")]):
        lo = np.minimum(reference.min(axis=0), np.vstack((b, c)).min(axis=0)) - 5
        hi = np.maximum(reference.max(axis=0), np.vstack((b, c)).max(axis=0)) + 5
        for row in (0, 1):
            ax = axes[row, col]
            ax.set_facecolor("#202329")
            ax.scatter(reference[:, i], reference[:, j], s=.18, color="#b9bec6", alpha=.3, rasterized=True)
            if row == 1:
                ax.scatter(b[:, i], b[:, j], s=.2, color="#55aaff", alpha=.45, rasterized=True)
                ax.scatter(c[:, i], c[:, j], s=.2, color="#ffad55", alpha=.45, rasterized=True)
            ax.set_aspect("equal")
            ax.set_xlim(lo[i], hi[i]); ax.set_ylim(lo[j], hi[j])
            ax.set_xlabel("XYZ"[i] + " (mm)", color="white")
            ax.set_ylabel("XYZ"[j] + " (mm)", color="white")
            ax.tick_params(colors="#cccccc", labelsize=8)
            ax.set_title(("Assembly reference: " if row == 0 else "Aligned overlay: ") + title, color="white")
            ax.grid(alpha=.15)
            for spine in ax.spines.values():
                spine.set_color("#555555")
    fig.suptitle("Grey: supplied assembly | Blue: bracket | Orange: official camera\n"
                 "STL alignment only; not physical calibration", color="white", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, .94))
    fig.savefig(OUTPUT / "alignment_preview.png", dpi=150)
    plt.close(fig)


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    paths = {name: MESH_DIR / filename for name, filename in {
        "bracket": "bracket.stl", "assembly": "bracket_and_camera.stl",
        "camera": "femto_bolt.stl"}.items()}
    meshes = {name: trimesh.load(path, force="mesh", process=True) for name, path in paths.items()}
    # User exports inferred to be mm; official ROS mesh is in m.
    meshes["camera"].apply_scale(1000.0)
    components = {name: parts(mesh) for name, mesh in meshes.items()}
    b_t, b_round, b_report = fit("bracket", meshes["bracket"], components["bracket"],
                               meshes["assembly"], components["assembly"], BRACKET_PAIRS)
    c_t, c_round, c_report = fit("camera", meshes["camera"], components["camera"],
                               meshes["assembly"], components["assembly"], CAMERA_PAIRS)
    relative = np.linalg.inv(b_t) @ c_t
    rounded_relative = np.linalg.inv(b_round) @ c_round
    result = {
        "definition": "T_A_from_B maps points from frame B to frame A; column vectors",
        "matrix_translation_unit": "m",
        "input_scale_to_mm": {"bracket": 1.0, "assembly": 1.0, "camera": 1000.0},
        "assumptions": [
            "User bracket and assembly STL coordinate units are mm, inferred from dimensions.",
            "Official femto_bolt.stl coordinates are the camera base/mount frame, as its Xacro visual origin is identity.",
            "Component correspondence follows matching shell/cover/lens shape and dimensions.",
            "STLs differ in tessellation and some modeled details; residuals are not calibration uncertainty."
        ],
        "inputs": {
            name: {"file": str(path.relative_to(ROOT)), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                   "components": len(components[name]), "faces": len(meshes[name].faces),
                   "bounds_after_scale_mm": meshes[name].bounds.tolist()}
            for name, path in paths.items()
        },
        "bracket_registration": b_report,
        "camera_registration": c_report,
        "T_bracket_from_camera_mount_fitted": describe(relative),
        "T_bracket_from_camera_mount_rounded": describe(rounded_relative),
        "not_applied_to_urdf": True,
    }
    (OUTPUT / "alignment_result.json").write_text(json.dumps(result, indent=2) + "\n")
    preview(meshes["assembly"], meshes["bracket"], meshes["camera"], b_t, c_t)
    print("RELATIVE", json.dumps(describe(relative)), flush=True)
    print("ROUNDED_RELATIVE", json.dumps(describe(rounded_relative)), flush=True)
    print("OUTPUT", OUTPUT, flush=True)


if __name__ == "__main__":
    main()
