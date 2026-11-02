import { useEffect, useRef, useState } from "react";
import type { ReactNode } from "react";

type ParameterKind = "number" | "integer" | "boolean" | "select" | "text";

interface ParameterDefinition {
  path: readonly string[];
  label: string;
  kind: ParameterKind;
  unit?: "mm" | "°";
  scale?: number;
  options?: readonly string[];
  min?: number;
}

interface ParameterSection {
  title: string;
  parameters: readonly ParameterDefinition[];
}

const mm = 1000;
const optionLabels: Record<string, string> = {
  axis: "축 기준", surface_adaptive: "표면 대응",
  top: "위쪽", bottom: "아래쪽", left: "왼쪽", right: "오른쪽",
};
const mainParameters = new Set([
  "slicing_method", "spline_start_side", "spline_start_offset", "spline_spacing", "curvature_slice_angle_deg",
  "offset_distance", "final_point_spacing", "endpoint_extension_length", "raster_zigzag", "paint_min_false_length", "paint_min_true_length",
]);

const sections: readonly ParameterSection[] = [
  {
    title: "표면 및 슬라이싱",
    parameters: [
      { path: ["painting_trajectory", "surface_extraction", "slicing_method"], label: "슬라이싱 방식", kind: "select", options: ["axis", "surface_adaptive"] },
      { path: ["painting_trajectory", "surface_extraction", "spline_start_side"], label: "슬라이싱 시작 방향", kind: "select", options: ["top", "bottom", "left", "right"] },
      { path: ["painting_trajectory", "surface_extraction", "spline_start_offset"], label: "시작 오프셋", kind: "number", unit: "mm", scale: mm, min: 0 },
      { path: ["painting_trajectory", "surface_extraction", "spline_spacing"], label: "슬라이싱 간격", kind: "number", unit: "mm", scale: mm, min: 0 },
      { path: ["painting_trajectory", "surface_extraction", "spline_half_width"], label: "슬라이스 반폭", kind: "number", unit: "mm", scale: mm, min: 0 },
      { path: ["painting_trajectory", "surface_extraction", "geodesic_normal_neighbor_count"], label: "법선 이웃 개수", kind: "integer", min: 1 },
      { path: ["painting_trajectory", "surface_extraction", "geodesic_band_half_width"], label: "가이드 밴드 반폭", kind: "number", unit: "mm", scale: mm, min: 0 },
      { path: ["painting_trajectory", "surface_extraction", "guide_extension_min_bin_points"], label: "가이드 셀 최소 점", kind: "integer", min: 1 },
      { path: ["painting_trajectory", "surface_extraction", "geodesic_min_band_points"], label: "밴드 최소 점", kind: "integer", min: 1 },
      { path: ["painting_trajectory", "surface_extraction", "curvature_slice_angle_deg"], label: "곡률 보정 기준 각도", kind: "number", unit: "°", min: 0 },
    ],
  },
  {
    title: "경로 보정 · 끝점",
    parameters: [
      { path: ["painting_trajectory", "correction", "endpoint", "quantize_step"], label: "양자화 간격", kind: "number", unit: "mm", scale: mm, min: 0 },
      { path: ["painting_trajectory", "correction", "endpoint", "graph_neighbor_count"], label: "그래프 이웃 개수", kind: "integer", min: 1 },
      { path: ["painting_trajectory", "correction", "endpoint", "component_min_arc_length"], label: "컴포넌트 최소 길이", kind: "number", unit: "mm", scale: mm, min: 0 },
      { path: ["painting_trajectory", "correction", "endpoint", "component_merge_backtrack_tolerance"], label: "역방향 병합 허용값", kind: "number", unit: "mm", scale: mm, min: 0 },
    ],
  },
  {
    title: "경로 보정 · 지지점",
    parameters: [
      { path: ["painting_trajectory", "correction", "support", "support_point_spacing"], label: "지지점 간격", kind: "number", unit: "mm", scale: mm, min: 0 },
      { path: ["painting_trajectory", "correction", "support", "tangent_half_width"], label: "접선 탐색 반폭", kind: "number", unit: "mm", scale: mm, min: 0 },
      { path: ["painting_trajectory", "correction", "support", "anchor_max_distance"], label: "앵커 최대 거리", kind: "number", unit: "mm", scale: mm, min: 0 },
      { path: ["painting_trajectory", "correction", "support", "transition_count"], label: "전이 샘플 개수", kind: "integer", min: 0 },
      { path: ["painting_trajectory", "correction", "support", "endpoint_transition_count"], label: "끝점 전이 샘플 개수", kind: "integer", min: 0 },
    ],
  },
  {
    title: "경로 보정 · 최적화",
    parameters: [
      { path: ["painting_trajectory", "correction", "optimization", "lambda_end"], label: "끝점 가중치", kind: "number", min: 0 },
      { path: ["painting_trajectory", "correction", "optimization", "lambda_data"], label: "데이터 가중치", kind: "number", min: 0 },
      { path: ["painting_trajectory", "correction", "optimization", "lambda_stretch"], label: "간격 유지 가중치", kind: "number", min: 0 },
      { path: ["painting_trajectory", "correction", "optimization", "lambda_bend"], label: "곡률 가중치", kind: "number", min: 0 },
      { path: ["painting_trajectory", "correction", "optimization", "regularizer"], label: "정규화 계수", kind: "number", min: 0 },
    ],
  },
  {
    title: "오프셋",
    parameters: [
      { path: ["painting_trajectory", "offset", "offset_point_spacing"], label: "오프셋 제어점 간격", kind: "number", unit: "mm", scale: mm, min: 0 },
      { path: ["painting_trajectory", "offset", "offset_distance"], label: "표면 오프셋 거리", kind: "number", unit: "mm", scale: mm, min: 0 },
    ],
  },
  {
    title: "스플라인 및 분사",
    parameters: [
      { path: ["painting_trajectory", "spline", "spline_point_spacing"], label: "내부 스플라인 해상도", kind: "number", unit: "mm", scale: mm, min: 0 },
      { path: ["painting_trajectory", "spline", "final_point_spacing"], label: "최종 포인트 간격", kind: "number", unit: "mm", scale: mm, min: 0 },
      { path: ["painting_trajectory", "spline", "endpoint_extension_length"], label: "끝점 연장 길이", kind: "number", unit: "mm", scale: mm, min: 0 },
      { path: ["painting_trajectory", "spline", "raster_zigzag"], label: "지그재그 경로", kind: "boolean" },
      { path: ["painting_trajectory", "spline", "paint_check_distance"], label: "분사 검사 거리", kind: "number", unit: "mm", scale: mm, min: 0 },
      { path: ["painting_trajectory", "spline", "paint_check_half_width"], label: "분사 검사 반폭", kind: "number", unit: "mm", scale: mm, min: 0 },
      { path: ["painting_trajectory", "spline", "paint_min_false_length"], label: "최소 미분사 구간", kind: "number", unit: "mm", scale: mm, min: 0 },
      { path: ["painting_trajectory", "spline", "paint_min_true_length"], label: "최소 분사 구간", kind: "number", unit: "mm", scale: mm, min: 0 },
    ],
  },
];

function valueAt(source: Record<string, unknown>, path: readonly string[]): unknown {
  let current: unknown = source;
  for (const key of path) {
    if (!current || typeof current !== "object" || Array.isArray(current)) {
      return undefined;
    }
    current = (current as Record<string, unknown>)[key];
  }
  return current;
}

function cloneWithValue(
  source: Record<string, unknown>,
  path: readonly string[],
  value: unknown,
): Record<string, unknown> {
  const clone = structuredClone(source);
  let current = clone;
  for (let index = 0; index < path.length - 1; index += 1) {
    const key = path[index];
    const child = current[key];
    if (!child || typeof child !== "object" || Array.isArray(child)) {
      current[key] = {};
    }
    current = current[key] as Record<string, unknown>;
  }
  current[path[path.length - 1]] = value;
  return clone;
}

function NumericInput({
  value,
  disabled,
  integer,
  min,
  onCommit,
}: {
  value: number;
  disabled: boolean;
  integer: boolean;
  min?: number;
  onCommit: (value: number) => void;
}) {
  const inputRef = useRef<HTMLInputElement>(null);
  const [source, setSource] = useState(String(value));
  useEffect(() => {
    if (document.activeElement !== inputRef.current) {
      setSource(String(value));
    }
  }, [value]);
  const commit = () => {
    const parsed = Number(source);
    if (!Number.isFinite(parsed)) {
      setSource(String(value));
      return;
    }
    const normalized = integer ? Math.round(parsed) : parsed;
    if (normalized !== value) onCommit(normalized);
    setSource(String(normalized));
  };
  return (
    <input
      ref={inputRef}
      type="number"
      value={source}
      min={min}
      step={integer ? 1 : "any"}
      disabled={disabled}
      onChange={(event) => setSource(event.target.value)}
      onBlur={commit}
      onKeyDown={(event) => {
        if (event.key === "Enter") event.currentTarget.blur();
        if (event.key === "Escape") {
          setSource(String(value));
          event.currentTarget.blur();
        }
      }}
    />
  );
}

function Field({
  definition,
  parameters,
  disabled,
  onChange,
}: {
  definition: ParameterDefinition;
  parameters: Record<string, unknown>;
  disabled: boolean;
  onChange: (parameters: Record<string, unknown>) => void;
}) {
  const rawValue = valueAt(parameters, definition.path);
  const update = (value: unknown) => onChange(
    cloneWithValue(parameters, definition.path, value),
  );

  let control: ReactNode;
  if (definition.kind === "boolean") {
    control = (
      <label className="parameter-boolean">
        <input
          type="checkbox"
          checked={Boolean(rawValue)}
          disabled={disabled}
          onChange={(event) => update(event.target.checked)}
        />
        <span>{rawValue ? "사용" : "사용 안 함"}</span>
      </label>
    );
  } else if (definition.kind === "select") {
    control = (
      <select
        value={String(rawValue ?? "")}
        disabled={disabled}
        onChange={(event) => update(event.target.value)}
      >
        {definition.options?.map((option) => <option key={option} value={option}>{optionLabels[option] ?? option}</option>)}
      </select>
    );
  } else if (definition.kind === "number" || definition.kind === "integer") {
    const scale = definition.scale ?? 1;
    const numericValue = typeof rawValue === "number" ? rawValue * scale : 0;
    control = (
      <div className="parameter-number">
        <NumericInput
          value={Number.isFinite(numericValue) ? numericValue : 0}
          min={definition.min}
          integer={definition.kind === "integer"}
          disabled={disabled}
          onCommit={(next) => update(definition.kind === "integer" ? next : next / scale)}
        />
        {definition.unit && <em>{definition.unit}</em>}
      </div>
    );
  } else {
    control = (
      <input
        type="text"
        value={String(rawValue ?? "")}
        disabled={disabled}
        onChange={(event) => update(event.target.value)}
      />
    );
  }

  return (
    <label className="editable-parameter">
      <span>{definition.label}</span>
      {control}
    </label>
  );
}

export function ParameterEditor({
  parameters,
  disabled,
  onChange,
}: {
  parameters: Record<string, unknown>;
  disabled: boolean;
  onChange: (parameters: Record<string, unknown>) => void;
}) {
  const renderSection = (section: ParameterSection, advanced = false) => {
    const definitions = section.parameters.filter((definition) =>
      mainParameters.has(definition.path[definition.path.length - 1]) !== advanced);
    if (!definitions.length) return null;
    return <details key={`${section.title}-${advanced}`} open={!advanced}>
      <summary>{section.title}</summary>
      <div className="parameter-section-fields">
        {definitions.map((definition) => <Field key={definition.path.join(".")}
          definition={definition} parameters={parameters} disabled={disabled} onChange={onChange} />)}
      </div>
    </details>;
  };
  return (
    <div className="parameter-editor">
      {sections.map((section) => renderSection(section))}
      <details className="advanced-parameters"><summary>고급 파라미터</summary>
        <div className="details-content">{sections.map((section) => renderSection(section, true))}</div>
      </details>
    </div>
  );
}
