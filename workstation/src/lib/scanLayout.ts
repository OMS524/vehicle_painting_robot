import type { Vector3Tuple } from "../types";

export interface ScanPlacement {
  id: string;
  name: string;
  xyz: Vector3Tuple; // part center in world_base, metres
  rpy: Vector3Tuple; // part orientation, Rz Ry Rx, radians
  direction: Vector3Tuple; // camera -> part, world_base unit vector
}
export interface ScanLayout { version: 1; frame: "world_base"; selectedId: string; placements: ScanPlacement[] }
export const SCAN_LAYOUT_KEY = "painting-workstation-scan-layout-v1";
export const USER_PLACEMENT_ID = "user-defined";

// A separate reusable manual slot: never overwrite one of the six presets.
export function selectUserPlacement(layout: ScanLayout): ScanLayout {
  if (layout.selectedId === USER_PLACEMENT_ID) return layout;
  if (layout.placements.some((p) => p.id === USER_PLACEMENT_ID)) {
    return { ...layout, selectedId: USER_PLACEMENT_ID };
  }
  if (layout.placements.length >= 100) return layout;
  const source = layout.placements.find((p) => p.id === layout.selectedId)!;
  return { ...layout, selectedId: USER_PLACEMENT_ID, placements: [...layout.placements, {
    id: USER_PLACEMENT_ID, name: "사용자 설정",
    xyz: [...source.xyz], rpy: [...source.rpy], direction: [...source.direction],
  }] };
}

export function defaultScanLayout(): ScanLayout {
  const placements: ScanPlacement[] = [];
  for (const side of [1, -1]) {
    [-2, 0, 2].forEach((x, index) => placements.push({
      id: `${side > 0 ? "left" : "right"}-${index + 1}`,
      name: `${side > 0 ? "왼쪽 (+Y)" : "오른쪽 (−Y)"} ${index + 1}`,
      xyz: [x, side * 1.2, 1.2],
      rpy: [Math.PI / 2, 0, side > 0 ? 0 : Math.PI],
      direction: [0, side, 0],
    }));
  }
  return { version: 1, frame: "world_base", selectedId: placements[0].id, placements };
}

export function validScanLayout(value: unknown): value is ScanLayout {
  if (!value || typeof value !== "object") return false;
  const layout = value as ScanLayout;
  const vector = (v: unknown): v is Vector3Tuple => Array.isArray(v) && v.length === 3
    && v.every((n) => typeof n === "number" && Number.isFinite(n));
  return layout.version === 1 && layout.frame === "world_base" && Array.isArray(layout.placements)
    && layout.placements.length > 0 && layout.placements.length <= 100
    && layout.placements.every((p) => p && typeof p.id === "string" && typeof p.name === "string"
      && vector(p.xyz) && vector(p.rpy) && vector(p.direction) && Math.hypot(...p.direction) > 1e-9)
    && new Set(layout.placements.map((p) => p.id)).size === layout.placements.length
    && layout.placements.some((p) => p.id === layout.selectedId);
}

export function loadScanLayout(): ScanLayout {
  try {
    const value: unknown = JSON.parse(localStorage.getItem(SCAN_LAYOUT_KEY) ?? "null");
    if (validScanLayout(value)) return value;
  } catch { /* Fall back to the labelled example presets, never to robot motion. */ }
  return defaultScanLayout();
}
