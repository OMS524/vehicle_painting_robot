import { invoke } from "@tauri-apps/api/core";
import type {
  BackendInfo,
  CompletionSummary,
  ControlPointDataset,
  EditableControlPoint,
  GenerationSummary,
  ScanInfo,
} from "../types";

export function isTauriRuntime(): boolean {
  return "__TAURI_INTERNALS__" in window;
}

function requireTauri(): void {
  if (!isTauriRuntime()) {
    throw new Error("경로 생성과 저장은 Tauri 애플리케이션에서 실행해야 합니다.");
  }
}

export async function getBackendInfo(): Promise<BackendInfo> {
  requireTauri();
  return invoke<BackendInfo>("get_backend_info");
}

export async function readDefaultParameters(): Promise<string> {
  requireTauri();
  return invoke<string>("read_default_parameters");
}

export async function stageScanFile(file: File): Promise<ScanInfo> {
  requireTauri();
  const bytes = new Uint8Array(await file.arrayBuffer());
  return invoke<ScanInfo>("stage_scan_file", bytes, {
    headers: {
      "x-file-name": encodeURIComponent(file.name),
    },
  });
}

export async function generateTrajectory(
  parameters: Record<string, unknown>,
  paintDir: [number, number, number],
  scanDir: [number, number, number],
): Promise<GenerationSummary> {
  requireTauri();
  return invoke<GenerationSummary>("generate_trajectory", {
    request: { parameters, paintDir, scanDir },
  });
}

export async function readGeneratedTrajectory(): Promise<ArrayBuffer> {
  requireTauri();
  return invoke<ArrayBuffer>("read_generated_trajectory");
}

export async function readControlPoints(): Promise<ControlPointDataset> {
  requireTauri();
  return invoke<ControlPointDataset>("read_control_points");
}

export async function regenerateTrajectory(
  controlPoints: EditableControlPoint[],
  deletedRowIndices: number[] = [],
  deletedPointIds: string[] = [],
): Promise<GenerationSummary> {
  requireTauri();
  return invoke<GenerationSummary>("regenerate_trajectory", {
    request: { controlPoints, deletedRowIndices, deletedPointIds },
  });
}

export async function completeTrajectory(): Promise<CompletionSummary> {
  requireTauri();
  return invoke<CompletionSummary>("complete_trajectory");
}
