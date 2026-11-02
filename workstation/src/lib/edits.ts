import type { EditableControlPoint } from "../types";

// Selecting a point, committing an unchanged input, and q -> -q are not edits.
export function sameControlPose(a: EditableControlPoint, b: EditableControlPoint): boolean {
  if (a.position.some((value, i) => Math.abs(value - b.position[i]) > 1e-9)) return false;
  const an = Math.hypot(...a.orientation), bn = Math.hypot(...b.orientation);
  if (!an || !bn) return false;
  const dot = a.orientation.reduce((sum, value, i) => sum + value * b.orientation[i], 0) / (an * bn);
  return Math.abs(dot) >= 1 - 1e-12;
}
