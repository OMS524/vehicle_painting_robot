import { useEffect, useMemo, useRef, useState } from "react";
import type { ReactNode } from "react";
import * as THREE from "three";
import { ParameterEditor } from "./components/ParameterEditor";
import { TrajectoryViewer } from "./components/TrajectoryViewer";
import {
  completeTrajectory,
  generateTrajectory,
  getBackendInfo,
  isTauriRuntime,
  readControlPoints,
  readDefaultParameters,
  readGeneratedTrajectory,
  regenerateTrajectory,
  stageScanFile,
} from "./lib/api";
import { parseParameterDocument, parseTrajectoryCsv } from "./lib/loaders";
import type {
  BackendInfo,
  CompletionSummary,
  EditableControlPoint,
  LayerVisibility,
  ScanInfo,
  TransformGizmoMode,
  TrajectoryDataset,
  Vector3Tuple,
  VisualizationSettings,
} from "./types";

type ProcessStep = 1 | 2 | 3;
type BusyAction = "scan" | "generate" | "regenerate" | "complete" | null;

interface StandaloneProject {
  info: {
    name: string;
    surfaceFile: string;
    trajectoryFile: string;
    surfaceBytes: number;
    trajectoryBytes: number;
  };
  surfaceBase64: string;
  trajectoryBase64: string;
  parametersBase64: string;
  sourceLabel: string;
}

declare global {
  interface Window {
    __PAINTING_TRAJECTORY_STANDALONE__?: StandaloneProject;
  }
}

const initialLayers: LayerVisibility = {
  surface: true,
  trajectory: true,
  sprayOff: true,
  sprayDirections: true,
  controlPoints: true,
  axes: true,
};

const initialVisualization: VisualizationSettings = {
  surfaceStyle: "original",
  background: "gray",
  rimLight: true,
  lightIntensity: 1,
};

const processSteps: readonly { id: ProcessStep; title: string; caption: string }[] = [
  { id: 1, title: "스캔", caption: "Scan & Import" },
  { id: 2, title: "도장 경로 생성", caption: "Generate" },
  { id: 3, title: "도장 경로 수정", caption: "Edit & Complete" },
];

function Foldout({ title, children }: { title: string; children: ReactNode }) {
  const [open, setOpen] = useState(true);
  return (
    <section className={`foldout ${open ? "open" : ""}`}>
      <button
        className="foldout-heading"
        type="button"
        aria-expanded={open}
        onClick={() => setOpen((current) => !current)}
      >
        <i />
        <span>{title}</span>
      </button>
      {open && <div className="foldout-content">{children}</div>}
    </section>
  );
}

function Toggle({
  checked,
  label,
  color,
  onChange,
}: {
  checked: boolean;
  label: string;
  color?: string;
  onChange: (checked: boolean) => void;
}) {
  return (
    <label className="layer-toggle">
      <input
        type="checkbox"
        checked={checked}
        onChange={(event) => onChange(event.target.checked)}
      />
      <span className="toggle-track"><span /></span>
      {color && <i style={{ backgroundColor: color }} />}
      <span>{label}</span>
    </label>
  );
}

function formatBytes(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
}

function errorText(reason: unknown): string {
  return reason instanceof Error ? reason.message : String(reason);
}

function decodeBase64(base64: string): ArrayBuffer {
  const binary = atob(base64);
  const bytes = new Uint8Array(binary.length);
  for (let index = 0; index < binary.length; index += 1) {
    bytes[index] = binary.charCodeAt(index);
  }
  return bytes.buffer;
}

function projectToPlane(position: Vector3Tuple, point: EditableControlPoint): Vector3Tuple {
  const value = new THREE.Vector3(...position);
  const origin = new THREE.Vector3(...point.planeOrigin);
  const normal = new THREE.Vector3(...point.planeNormal).normalize();
  value.addScaledVector(normal, -value.clone().sub(origin).dot(normal));
  return value.toArray() as Vector3Tuple;
}

function eulerDegrees(point: EditableControlPoint): Vector3Tuple {
  const quaternion = new THREE.Quaternion(...point.orientation).normalize();
  const euler = new THREE.Euler().setFromQuaternion(quaternion, "XYZ");
  return [
    THREE.MathUtils.radToDeg(euler.x),
    THREE.MathUtils.radToDeg(euler.y),
    THREE.MathUtils.radToDeg(euler.z),
  ];
}

function quaternionFromEulerDegrees(values: Vector3Tuple): [number, number, number, number] {
  const euler = new THREE.Euler(
    THREE.MathUtils.degToRad(values[0]),
    THREE.MathUtils.degToRad(values[1]),
    THREE.MathUtils.degToRad(values[2]),
    "XYZ",
  );
  return new THREE.Quaternion().setFromEuler(euler).normalize().toArray();
}

function CommitNumberInput({
  value,
  step = "any",
  disabled,
  onCommit,
}: {
  value: number;
  step?: number | "any";
  disabled: boolean;
  onCommit: (value: number) => void;
}) {
  const inputRef = useRef<HTMLInputElement>(null);
  const [source, setSource] = useState(String(value));
  useEffect(() => {
    if (document.activeElement !== inputRef.current) setSource(String(value));
  }, [value]);
  const commit = () => {
    const parsed = Number(source);
    if (!Number.isFinite(parsed)) {
      setSource(String(value));
      return;
    }
    onCommit(parsed);
    setSource(String(parsed));
  };
  return (
    <input
      ref={inputRef}
      type="number"
      step={step}
      value={source}
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

function VectorEditor({
  label,
  value,
  disabled,
  onChange,
}: {
  label: string;
  value: Vector3Tuple;
  disabled: boolean;
  onChange: (value: Vector3Tuple) => void;
}) {
  return (
    <div className="vector-editor">
      <strong>{label}</strong>
      <div>
        {(["X", "Y", "Z"] as const).map((axis, index) => (
          <label key={axis}>
            <span>{axis}</span>
            <CommitNumberInput
              value={value[index]}
              disabled={disabled}
              onCommit={(next) => {
                const vector = [...value] as Vector3Tuple;
                vector[index] = next;
                onChange(vector);
              }}
            />
          </label>
        ))}
      </div>
    </div>
  );
}

export default function App() {
  const fileInputRef = useRef<HTMLInputElement>(null);
  const [step, setStep] = useState<ProcessStep>(1);
  const [layers, setLayers] = useState(initialLayers);
  const [visualization, setVisualization] = useState(initialVisualization);
  const [parameters, setParameters] = useState<Record<string, unknown> | null>(null);
  const [paintDir, setPaintDir] = useState<Vector3Tuple>([0, 0, -1]);
  const [scanDir, setScanDir] = useState<Vector3Tuple>([0, 1, 0]);
  const [backendInfo, setBackendInfo] = useState<BackendInfo | null>(null);
  const [scanInfo, setScanInfo] = useState<ScanInfo | null>(null);
  const [scanBuffer, setScanBuffer] = useState<ArrayBuffer | null>(null);
  const [trajectory, setTrajectory] = useState<TrajectoryDataset | null>(null);
  const [controlPoints, setControlPoints] = useState<EditableControlPoint[]>([]);
  const [selectedControlPointId, setSelectedControlPointId] = useState<string | null>(null);
  const [transformGizmoMode, setTransformGizmoMode] = useState<TransformGizmoMode>("translate");
  const [deletedRowIndices, setDeletedRowIndices] = useState<number[]>([]);
  const [deletedPointIds, setDeletedPointIds] = useState<string[]>([]);
  const [busy, setBusy] = useState<BusyAction>(null);
  const [dirty, setDirty] = useState(false);
  const [generationStale, setGenerationStale] = useState(false);
  const [status, setStatus] = useState("백엔드를 확인하는 중입니다.");
  const [error, setError] = useState("");
  const [completion, setCompletion] = useState<CompletionSummary | null>(null);
  const [standalone, setStandalone] = useState(false);

  useEffect(() => {
    const embedded = window.__PAINTING_TRAJECTORY_STANDALONE__;
    if (embedded) {
      try {
        const trajectoryBuffer = decodeBase64(embedded.trajectoryBase64);
        setScanInfo({
          fileName: embedded.info.surfaceFile,
          byteCount: embedded.info.surfaceBytes,
          sessionDirectory: embedded.sourceLabel,
        });
        setScanBuffer(decodeBase64(embedded.surfaceBase64));
        setTrajectory(parseTrajectoryCsv(trajectoryBuffer));
        setParameters(parseParameterDocument(
          new TextDecoder("utf-8").decode(decodeBase64(embedded.parametersBase64)),
        ));
        setStandalone(true);
        setStep(3);
        setStatus(`${embedded.info.name} · 단일 HTML 읽기 전용 검토 모드`);
      } catch (reason: unknown) {
        setError(errorText(reason));
        setStatus("단일 HTML 데이터를 읽지 못했습니다.");
      }
      return;
    }
    if (!isTauriRuntime()) {
      setStatus("브라우저 미리보기 · 생성과 저장은 Tauri 앱에서 사용할 수 있습니다.");
      setError("이 공정 화면은 Python 백엔드를 호출하므로 Tauri 애플리케이션으로 실행해야 합니다.");
      return;
    }
    Promise.all([readDefaultParameters(), getBackendInfo()])
      .then(([source, info]) => {
        setParameters(parseParameterDocument(source));
        setBackendInfo(info);
        setStatus("스캔 데이터를 불러와 공정을 시작하세요.");
      })
      .catch((reason: unknown) => {
        setError(errorText(reason));
        setStatus("백엔드 초기화에 실패했습니다.");
      });
  }, []);

  const selectedControlPoint = useMemo(
    () => controlPoints.find((point) => point.id === selectedControlPointId) ?? null,
    [controlPoints, selectedControlPointId],
  );
  const selectedEuler = useMemo(
    () => selectedControlPoint ? eulerDegrees(selectedControlPoint) : null,
    [selectedControlPoint],
  );

  const setLayer = (key: keyof LayerVisibility, value: boolean) => {
    setLayers((current) => ({ ...current, [key]: value }));
  };
  const setVisualizationValue = <Key extends keyof VisualizationSettings>(
    key: Key,
    value: VisualizationSettings[Key],
  ) => setVisualization((current) => ({ ...current, [key]: value }));

  const canOpenStep = (target: ProcessStep): boolean => (
    standalone
      ? target === 3
      : target === 1
        || (target === 2 && Boolean(scanInfo))
        || (target === 3 && Boolean(trajectory) && !generationStale)
  );

  const invalidateGeneratedTrajectory = () => {
    if (!trajectory) return;
    setGenerationStale(true);
    setCompletion(null);
    setStatus("경로 생성 파라미터가 변경되었습니다. 경로를 다시 생성하세요.");
  };

  const updateParameters = (next: Record<string, unknown>) => {
    setParameters(next);
    invalidateGeneratedTrajectory();
  };

  const updatePaintDirection = (next: Vector3Tuple) => {
    setPaintDir(next);
    invalidateGeneratedTrajectory();
  };

  const updateScanDirection = (next: Vector3Tuple) => {
    setScanDir(next);
    invalidateGeneratedTrajectory();
  };

  const loadScan = async (file: File) => {
    setBusy("scan");
    setError("");
    setCompletion(null);
    setStatus(`${file.name} 스캔 데이터를 불러오는 중입니다.`);
    try {
      const buffer = await file.arrayBuffer();
      const info = await stageScanFile(file);
      setScanBuffer(buffer);
      setScanInfo(info);
      setTrajectory(null);
      setControlPoints([]);
      setSelectedControlPointId(null);
      setDeletedRowIndices([]);
      setDeletedPointIds([]);
      setDirty(false);
      setGenerationStale(false);
      setStep(1);
      setStatus(`${info.fileName} · ${formatBytes(info.byteCount)} 스캔 데이터를 불러왔습니다.`);
    } catch (reason: unknown) {
      setError(errorText(reason));
      setStatus("스캔 데이터 불러오기에 실패했습니다.");
    } finally {
      setBusy(null);
      if (fileInputRef.current) fileInputRef.current.value = "";
    }
  };

  const loadGeneratedData = async () => {
    const [trajectoryBuffer, controls] = await Promise.all([
      readGeneratedTrajectory(),
      readControlPoints(),
    ]);
    setTrajectory(parseTrajectoryCsv(trajectoryBuffer));
    setControlPoints(controls.points);
    setSelectedControlPointId(null);
    setDeletedRowIndices([]);
    setDeletedPointIds([]);
  };

  const generate = async () => {
    if (!scanInfo || !parameters) return;
    setBusy("generate");
    setError("");
    setCompletion(null);
    setStatus("기존 경로 생성 알고리즘을 실행하고 있습니다.");
    try {
      const summary = await generateTrajectory(parameters, paintDir, scanDir);
      await loadGeneratedData();
      setDirty(false);
      setGenerationStale(false);
      setStep(2);
      setStatus(
        `경로 생성 완료 · ${summary.rowCount} rows · ${summary.pointCount.toLocaleString()} points`,
      );
    } catch (reason: unknown) {
      setError(errorText(reason));
      setStatus("도장 경로 생성에 실패했습니다.");
    } finally {
      setBusy(null);
    }
  };

  const updateControlPoint = (
    id: string,
    updater: (point: EditableControlPoint) => EditableControlPoint,
  ) => {
    setControlPoints((current) => current.map((point) => (
      point.id === id ? updater(point) : point
    )));
    setDirty(true);
    setCompletion(null);
  };

  const moveControlPoint = (id: string, position: Vector3Tuple) => {
    updateControlPoint(id, (point) => ({
      ...point,
      position: projectToPlane(position, point),
    }));
  };

  const rotateControlPoint = (
    id: string,
    orientation: [number, number, number, number],
  ) => {
    updateControlPoint(id, (point) => ({ ...point, orientation }));
  };

  const setSelectedPositionAxis = (axis: number, valueMm: number) => {
    if (!selectedControlPoint) return;
    const position = [...selectedControlPoint.position] as Vector3Tuple;
    position[axis] = valueMm / 1000;
    moveControlPoint(selectedControlPoint.id, position);
  };

  const setSelectedRotationAxis = (axis: number, valueDegrees: number) => {
    if (!selectedControlPoint || !selectedEuler) return;
    const rotation = [...selectedEuler] as Vector3Tuple;
    rotation[axis] = valueDegrees;
    updateControlPoint(selectedControlPoint.id, (point) => ({
      ...point,
      orientation: quaternionFromEulerDegrees(rotation),
    }));
  };

  const selectedRowPointCount = selectedControlPoint
    ? controlPoints.filter((point) => point.rowIndex === selectedControlPoint.rowIndex).length
    : 0;

  const deleteSelectedPoint = () => {
    if (!selectedControlPoint || !trajectory || busy || selectedRowPointCount <= 2) return;
    const { id, rowIndex, pointIndex } = selectedControlPoint;
    if (!window.confirm(`Row ${rowIndex}의 Point ${pointIndex} 하나를 삭제할까요?\n재생성 시 남은 점들을 연결해 경로를 다시 만듭니다.`)) return;
    setControlPoints((current) => current.filter((point) => point.id !== id));
    setDeletedPointIds((current) => current.includes(id) ? current : [...current, id]);
    setTrajectory((current) => {
      if (!current) return current;
      const rows = current.rows.map((row) => {
        if (row.rowIndex !== rowIndex) return row;
        const points = row.points.filter((point) => point.pointId !== pointIndex);
        return {
          ...row,
          points,
          length: points.length ? Math.max(0, points[points.length - 1].distance - points[0].distance) : 0,
          paintRatio: points.length ? points.filter((point) => point.paint).length / points.length : 0,
        };
      });
      return {
        rows,
        pointCount: rows.reduce((count, row) => count + row.points.length, 0),
        paintPointCount: rows.reduce((count, row) => count + row.points.filter((point) => point.paint).length, 0),
      };
    });
    setSelectedControlPointId(null);
    setDirty(true);
    setCompletion(null);
    setStatus(`Row ${rowIndex}의 Point ${pointIndex}를 삭제했습니다. 경로 재생성을 눌러 반영하세요.`);
  };

  const deleteSelectedRow = () => {
    if (!selectedControlPoint || !trajectory || trajectory.rows.length <= 1) return;
    const rowIndex = selectedControlPoint.rowIndex;
    if (!window.confirm(`Row ${rowIndex} 경로 전체를 삭제할까요?`)) return;
    setTrajectory((current) => {
      if (!current) return current;
      const rows = current.rows.filter((row) => row.rowIndex !== rowIndex);
      return {
        rows,
        pointCount: rows.reduce((count, row) => count + row.points.length, 0),
        paintPointCount: rows.reduce(
          (count, row) => count + row.points.filter((point) => point.paint).length,
          0,
        ),
      };
    });
    setControlPoints((current) => current.filter((point) => point.rowIndex !== rowIndex));
    setDeletedRowIndices((current) => (
      current.includes(rowIndex)
        ? current
        : [...current, rowIndex].sort((left, right) => left - right)
    ));
    setSelectedControlPointId(null);
    setDirty(true);
    setCompletion(null);
    setStatus(`Row ${rowIndex} 경로를 삭제했습니다. 경로 재생성을 눌러 반영하세요.`);
  };

  const regenerate = async () => {
    if (!trajectory || !controlPoints.length) return;
    setBusy("regenerate");
    setError("");
    setStatus("수정한 최종 경로 코어 포인트로 경로를 재생성하고 있습니다.");
    try {
      const selectedId = selectedControlPointId;
      const summary = await regenerateTrajectory(controlPoints, deletedRowIndices, deletedPointIds);
      const [trajectoryBuffer, controls] = await Promise.all([
        readGeneratedTrajectory(),
        readControlPoints(),
      ]);
      setTrajectory(parseTrajectoryCsv(trajectoryBuffer));
      setControlPoints(controls.points);
      setDeletedRowIndices([]);
      setDeletedPointIds([]);
      setSelectedControlPointId(
        selectedId && controls.points.some((point) => point.id === selectedId)
          ? selectedId
          : null,
      );
      setDirty(false);
      setStatus(
        `경로 재생성 완료 · ${summary.rowCount} rows · ${summary.pointCount.toLocaleString()} points`,
      );
    } catch (reason: unknown) {
      setError(errorText(reason));
      setStatus("도장 경로 재생성에 실패했습니다.");
    } finally {
      setBusy(null);
    }
  };

  const complete = async () => {
    if (!trajectory || dirty) return;
    setBusy("complete");
    setError("");
    setStatus("완료 경로와 작업 정보를 저장하고 있습니다.");
    try {
      const result = await completeTrajectory();
      setCompletion(result);
      setStatus(`공정 경로 저장 완료 · ${result.outputDirectory}`);
    } catch (reason: unknown) {
      setError(errorText(reason));
      setStatus("완료 경로 저장에 실패했습니다.");
    } finally {
      setBusy(null);
    }
  };

  const renderRightPanel = () => {
    if (step === 1) {
      return (
        <div className="process-panel-content">
          <div className="process-panel-heading">
            <span>PROCESS 01</span>
            <h2>스캔 데이터 불러오기</h2>
            <p>차량 부품의 PLY 또는 XYZ CSV 스캔 데이터를 선택합니다.</p>
          </div>
          <button
            className="primary-action"
            type="button"
            disabled={Boolean(busy) || !isTauriRuntime()}
            onClick={() => fileInputRef.current?.click()}
          >
            스캔 데이터 불러오기
          </button>
          <input
            ref={fileInputRef}
            hidden
            type="file"
            accept=".ply,.csv"
            onChange={(event) => {
              const file = event.target.files?.[0];
              if (file) void loadScan(file);
            }}
          />
          <div className="stage-information">
            <div><span>파일</span><strong>{scanInfo?.fileName ?? "—"}</strong></div>
            <div><span>크기</span><strong>{scanInfo ? formatBytes(scanInfo.byteCount) : "—"}</strong></div>
            <div><span>형식</span><strong>{scanInfo?.fileName.split(".").pop()?.toUpperCase() ?? "—"}</strong></div>
          </div>
          <button
            className="secondary-action next-action"
            type="button"
            disabled={!scanInfo || Boolean(busy)}
            onClick={() => setStep(2)}
          >
            다음: 도장 경로 생성
          </button>
        </div>
      );
    }

    if (step === 2) {
      return (
        <div className="process-panel-content generation-panel">
          <div className="process-panel-heading sticky-heading">
            <span>PROCESS 02</span>
            <h2>도장 경로 생성</h2>
            <p>기본 YAML 값을 확인하고 필요한 항목을 수정합니다.</p>
          </div>
          <div className="direction-section">
            <VectorEditor label="도장 방향 벡터" value={paintDir} disabled={Boolean(busy)} onChange={updatePaintDirection} />
            <VectorEditor label="스캔 방향 벡터" value={scanDir} disabled={Boolean(busy)} onChange={updateScanDirection} />
          </div>
          {parameters ? (
            <ParameterEditor parameters={parameters} disabled={Boolean(busy)} onChange={updateParameters} />
          ) : (
            <p className="empty-message">기본 파라미터를 불러오는 중입니다.</p>
          )}
          <div className="process-actions sticky-actions">
            <button
              className="primary-action"
              type="button"
              disabled={!scanInfo || !parameters || Boolean(busy)}
              onClick={() => void generate()}
            >
              도장 경로 생성
            </button>
            <button
              className="secondary-action"
              type="button"
              disabled={!trajectory || generationStale || Boolean(busy)}
              onClick={() => setStep(3)}
            >
              다음: 도장 경로 수정
            </button>
          </div>
        </div>
      );
    }

    if (standalone) {
      return (
        <div className="process-panel-content edit-panel">
          <div className="process-panel-heading">
            <span>STANDALONE REVIEW</span>
            <h2>도장 경로 검토</h2>
            <p>이 HTML에는 생성 시점의 스캔, 경로와 파라미터가 내장되어 있습니다.</p>
          </div>
          <div className="stage-information">
            <div><span>스캔</span><strong>{scanInfo?.fileName ?? "—"}</strong></div>
            <div><span>Rows</span><strong>{trajectory?.rows.length ?? 0}</strong></div>
            <div><span>Points</span><strong>{trajectory?.pointCount.toLocaleString() ?? "0"}</strong></div>
          </div>
          <p className="empty-message">
            단일 HTML은 읽기 전용입니다. 경로 생성·수정·완료 저장은 Painting Process Workstation 앱에서 수행합니다.
          </p>
        </div>
      );
    }

    return (
      <div className="process-panel-content edit-panel">
        <div className="process-panel-heading">
          <span>PROCESS 03</span>
          <h2>도장 경로 수정</h2>
          <p>2단계에서 생성된 분홍색 경로 포인트를 선택하고 위치와 회전을 조정합니다.</p>
        </div>
        {selectedControlPoint && selectedEuler ? (
          <>
            <div className="selected-point-heading" data-preserve-point-selection>
              <strong>Row {selectedControlPoint.rowIndex}</strong>
              <span>Point {selectedControlPoint.pointIndex}</span>
            </div>
            <div className="transform-gizmo-mode" data-preserve-point-selection>
              <button
                type="button"
                className={transformGizmoMode === "translate" ? "active" : ""}
                disabled={Boolean(busy)}
                onClick={() => setTransformGizmoMode("translate")}
              >
                이동
              </button>
              <button
                type="button"
                className={transformGizmoMode === "rotate" ? "active" : ""}
                disabled={Boolean(busy)}
                onClick={() => setTransformGizmoMode("rotate")}
              >
                회전
              </button>
              <p>
                {transformGizmoMode === "translate"
                  ? "Scene의 빨강·초록 화살표를 드래그하여 슬라이싱 평면의 두 축으로 이동합니다."
                  : "Scene의 빨강·초록·파랑 링을 드래그하여 세 축으로 회전합니다."}
              </p>
            </div>
            <div className="transform-section" data-preserve-point-selection>
              <h3>위치 <em>mm</em></h3>
              <div className="transform-grid">
                {(["X", "Y", "Z"] as const).map((axis, index) => (
                  <label key={`position-${axis}`}>
                    <span>{axis}</span>
                    <CommitNumberInput
                      step={0.1}
                      value={Number((selectedControlPoint.position[index] * 1000).toFixed(3))}
                      disabled={Boolean(busy)}
                      onCommit={(value) => setSelectedPositionAxis(index, value)}
                    />
                  </label>
                ))}
              </div>
              <p>입력값은 선택한 row의 슬라이싱 평면 위로 자동 투영됩니다.</p>
            </div>
            <div className="transform-section" data-preserve-point-selection>
              <h3>회전 <em>deg · XYZ</em></h3>
              <div className="transform-grid">
                {(["X", "Y", "Z"] as const).map((axis, index) => (
                  <label key={`rotation-${axis}`}>
                    <span>{axis}</span>
                    <CommitNumberInput
                      step={0.1}
                      value={Number(selectedEuler[index].toFixed(3))}
                      disabled={Boolean(busy)}
                      onCommit={(value) => setSelectedRotationAxis(index, value)}
                    />
                  </label>
                ))}
              </div>
            </div>
            <div className="path-row-actions" data-preserve-point-selection>
              <button
                className="delete-row-action"
                type="button"
                disabled={Boolean(busy) || selectedRowPointCount <= 2}
                onClick={deleteSelectedPoint}
              >
                선택 포인트 삭제
              </button>
              <p>
                {selectedRowPointCount <= 2
                  ? "경로에는 최소 2개의 편집 포인트가 필요합니다. 전체 삭제는 아래 버튼을 사용하세요."
                  : "선택한 포인트 하나만 삭제합니다. 재생성 시 남은 점을 연결하고 설정된 간격으로 다시 샘플링합니다."}
              </p>
              <button
                className="delete-row-action"
                type="button"
                disabled={Boolean(busy) || (trajectory?.rows.length ?? 0) <= 1}
                onClick={deleteSelectedRow}
              >
                선택 경로 삭제
              </button>
              <p>선택 포인트가 포함된 Row 전체를 삭제합니다.</p>
            </div>
          </>
        ) : (
          <p className="empty-message">뷰어에서 수정할 제어점을 선택하세요.</p>
        )}
        <div className={`edit-state ${dirty ? "dirty" : "clean"}`}>
          <i />
          <span>{dirty ? "재생성되지 않은 수정 사항이 있습니다." : "경로가 최신 제어점과 일치합니다."}</span>
        </div>
        <div className="process-actions edit-actions">
          <button
            className="secondary-action"
            type="button"
            disabled={!trajectory || !controlPoints.length || Boolean(busy)}
            onClick={() => void regenerate()}
          >
            경로 재생성
          </button>
          <button
            className="primary-action complete-action"
            type="button"
            disabled={!trajectory || dirty || Boolean(busy)}
            onClick={() => void complete()}
          >
            완료 및 저장
          </button>
        </div>
        {completion && (
          <div className="completion-card">
            <strong>저장 완료</strong>
            <span>{completion.completedAt}</span>
            <code>{completion.outputDirectory}</code>
          </div>
        )}
      </div>
    );
  };

  return (
    <main
      className="app-shell workstation-shell"
      onPointerDownCapture={(event) => {
        const target = event.target;
        if (
          selectedControlPointId
          && target instanceof Element
          && !target.closest(".viewer-panel")
          && !target.closest("[data-preserve-point-selection]")
        ) {
          setSelectedControlPointId(null);
        }
      }}
    >
      <header className="process-header">
        <div className="product-mark">
          <span>PW</span>
          <div><strong>Painting Process Workstation</strong><small>Robot Paint Path Workflow</small></div>
        </div>
        <nav className="process-stepper" aria-label="도장 공정 단계">
          {processSteps.map((item) => {
            const enabled = canOpenStep(item.id);
            const completed = (item.id === 1 && Boolean(scanInfo))
              || (item.id === 2 && Boolean(trajectory) && !generationStale)
              || (item.id === 3 && Boolean(completion));
            return (
              <button
                key={item.id}
                type="button"
                className={`${step === item.id ? "active" : ""} ${completed ? "completed" : ""}`}
                disabled={!enabled || Boolean(busy)}
                onClick={() => enabled && setStep(item.id)}
              >
                <i>{completed ? "✓" : item.id}</i>
                <span><strong>{item.title}</strong><small>{item.caption}</small></span>
              </button>
            );
          })}
        </nav>
      </header>

      <section className="workspace">
        <aside className="left-panel panel">
          <div className="panel-titlebar"><span className="panel-title-icon">▦</span><strong>Scene</strong></div>
          <div className="left-panel-content">
            <div className="workspace-object">
              <span className="object-icon">P</span>
              <span><strong>{scanInfo?.fileName ?? "No scan data"}</strong><small>Painting process job</small></span>
            </div>
            <Foldout title="Scene Layers">
              <div className="layer-list">
                <Toggle checked={layers.surface} label="Part surface" onChange={(value) => setLayer("surface", value)} />
                <Toggle checked={layers.trajectory} label="Spray ON path" color="#20e66a" onChange={(value) => setLayer("trajectory", value)} />
                <Toggle checked={layers.sprayOff} label="Spray OFF path" color="#ffc857" onChange={(value) => setLayer("sprayOff", value)} />
                <Toggle checked={layers.sprayDirections} label="Spray directions" color="#4cc9f0" onChange={(value) => setLayer("sprayDirections", value)} />
                <Toggle checked={layers.controlPoints} label="Path points" color="#ff4fa3" onChange={(value) => setLayer("controlPoints", value)} />
                <Toggle checked={layers.axes} label="Coordinate axes" onChange={(value) => setLayer("axes", value)} />
              </div>
            </Foldout>
            <Foldout title="Visualization">
              <div className="inspector-property">
                <span>Surface</span>
                <div className="segmented-control">
                  <button className={visualization.surfaceStyle === "original" ? "active" : ""} type="button" onClick={() => setVisualizationValue("surfaceStyle", "original")}>Original</button>
                  <button className={visualization.surfaceStyle === "inspection" ? "active" : ""} type="button" onClick={() => setVisualizationValue("surfaceStyle", "inspection")}>Inspection</button>
                </div>
              </div>
              <div className="inspector-property">
                <span>Background</span>
                <div className="segmented-control">
                  <button className={visualization.background === "gray" ? "active" : ""} type="button" onClick={() => setVisualizationValue("background", "gray")}>Gray</button>
                  <button className={visualization.background === "black" ? "active" : ""} type="button" onClick={() => setVisualizationValue("background", "black")}>Black</button>
                </div>
              </div>
              <label className="unity-toggle"><span>Rim Light</span><input type="checkbox" checked={visualization.rimLight} onChange={(event) => setVisualizationValue("rimLight", event.target.checked)} /><i /></label>
              <label className="light-control"><span><strong>Scene Light</strong><output>{visualization.lightIntensity.toFixed(1)}×</output></span><input type="range" min="0.5" max="1.8" step="0.1" value={visualization.lightIntensity} onChange={(event) => setVisualizationValue("lightIntensity", Number(event.target.value))} /></label>
            </Foldout>
            <Foldout title="Backend">
              <div className="source-row"><span>Planner</span><strong title={backendInfo?.plannerRoot}>{backendInfo?.plannerRoot ?? "—"}</strong></div>
              <div className="source-row"><span>Python</span><strong>{backendInfo?.pythonCommand ?? "—"}</strong></div>
              <div className="source-row"><span>Log</span><strong title={backendInfo?.logRoot}>{backendInfo?.logRoot ?? "—"}</strong></div>
            </Foldout>
          </div>
        </aside>

        <section className="viewer-panel">
          <div className="scene-titlebar"><span className="scene-tab active">Scene</span><span className="scene-project">{scanInfo?.fileName ?? "SCAN NOT LOADED"}</span></div>
          <div className="scene-body">
            <TrajectoryViewer
              surfaceBuffer={scanBuffer}
              surfaceFileName={scanInfo?.fileName ?? ""}
              trajectory={trajectory}
              controlPoints={controlPoints}
              selectedControlPointId={selectedControlPointId}
              editingEnabled={step === 3 && !busy && !standalone}
              transformGizmoMode={transformGizmoMode}
              layers={layers}
              visualization={visualization}
              onSelectControlPoint={setSelectedControlPointId}
              onMoveControlPoint={moveControlPoint}
              onRotateControlPoint={rotateControlPoint}
            />
            {!scanBuffer && (
              <div className="empty-scene"><span>01</span><strong>스캔 데이터를 불러오세요</strong><p>오른쪽 공정 패널에서 PLY 또는 CSV 파일을 선택합니다.</p></div>
            )}
            {busy && (
              <div className="loading-card"><span className="loader" /><strong>Processing</strong><p>{status}</p></div>
            )}
          </div>
        </section>

        <aside className="right-panel panel workstation-inspector">
          <div className="panel-titlebar inspector-titlebar"><span className="panel-title-icon">☷</span><strong>Process Inspector</strong><span className="panel-menu">⋮</span></div>
          <div className="process-scroll">{renderRightPanel()}</div>
        </aside>
      </section>

      <footer className="statusbar">
        <span className={error ? "status-dot error" : "status-dot"} />
        <span>{status}</span>
        <span className="status-version">Workstation v0.2.0 · {dirty ? "MODIFIED" : "READY"}</span>
      </footer>

      {error && (
        <div className="error-log-backdrop" role="presentation">
          <section
            className="error-log-dialog"
            role="alertdialog"
            aria-modal="true"
            aria-labelledby="error-log-title"
          >
            <header>
              <div>
                <span>BACKEND ERROR</span>
                <strong id="error-log-title">오류 로그</strong>
              </div>
              <button type="button" aria-label="오류 로그 닫기" onClick={() => setError("")}>×</button>
            </header>
            <pre>{error}</pre>
            <div className="error-log-actions">
              <p>백엔드에서 전달된 오류 내용을 그대로 표시합니다.</p>
              <button type="button" onClick={() => setError("")}>닫기</button>
            </div>
          </section>
        </div>
      )}
    </main>
  );
}
