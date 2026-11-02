import { useEffect, useRef, useState } from "react";
import type { ReactNode } from "react";
import { invoke } from "@tauri-apps/api/core";
import { MotionPreview } from "./MotionPreview";
import { getMotionModel } from "../lib/motion";
import type { MotionModel } from "../lib/motion";
import { defaultScanLayout, loadScanLayout, selectUserPlacement, SCAN_LAYOUT_KEY, USER_PLACEMENT_ID } from "../lib/scanLayout";
import type { ScanPlacement } from "../lib/scanLayout";
import type { LayerVisibility, TrajectoryDataset, Vector3Tuple, VisualizationSettings } from "../types";

const EMPTY_PATH: TrajectoryDataset = { rows: [], pointCount: 0, paintPointCount: 0 };
const ZERO: Vector3Tuple = [0, 0, 0];
interface Props {
  active: boolean; cellView: boolean; busy: boolean;
  onCellView: (value: boolean) => void;
  layers: LayerVisibility; visualization: VisualizationSettings;
  importPanel: ReactNode; sceneSettings: ReactNode; onError: (value: string) => void;
  onNext: () => void;
}

function NumberField({ value, onCommit, disabled }: { value: number; onCommit: (v: number) => void; disabled: boolean }) {
  const [draft, setDraft] = useState(String(value));
  const changed = useRef(false);
  useEffect(() => { setDraft(String(Number(value.toFixed(4)))); changed.current = false; }, [value]);
  return <input type="number" step="any" value={draft} disabled={disabled}
    onChange={(event) => { changed.current = true; setDraft(event.target.value); }}
    onBlur={() => {
      if (!changed.current) return;
      changed.current = false;
      const next = Number(draft);
      if (draft.trim() && Number.isFinite(next)) onCommit(next);
      setDraft(String(Number(value.toFixed(4))));
    }} onKeyDown={(event) => { if (event.key === "Enter") event.currentTarget.blur(); }} />;
}

export function ScanStage({ active, cellView, busy, onCellView, layers, visualization, importPanel, sceneSettings, onNext, onError }: Props) {
  const [layout, setLayout] = useState(loadScanLayout);
  const [model, setModel] = useState<MotionModel | null>(null);
  const [ready, setReady] = useState(false);
  const [modelError, setModelError] = useState("");
  const [reload, setReload] = useState(0);
  const [saved, setSaved] = useState("");
  const [saving, setSaving] = useState(false);
  const errorRef = useRef(onError); errorRef.current = onError;
  useEffect(() => {
    if (!active || model) return;
    let cancelled = false;
    setModelError("");
    void getMotionModel().then((data) => { if (!cancelled) setModel(data); })
      .catch((error: unknown) => { if (!cancelled) { setModelError(String(error)); errorRef.current(String(error)); } });
    return () => { cancelled = true; };
  }, [active, reload]);
  useEffect(() => {
    try { localStorage.setItem(SCAN_LAYOUT_KEY, JSON.stringify(layout)); }
    catch (error) { errorRef.current(`배치 설정 자동 저장 실패: ${String(error)}`); }
    setSaved("");
  }, [layout]);
  const selected = layout.placements.find((p) => p.id === layout.selectedId)!;
  const userSelected = selected.id === USER_PLACEMENT_ID;
  const hasUserPlacement = layout.placements.some((p) => p.id === USER_PLACEMENT_ID);
  // Pair left/right presets for display only; stored placement order is unchanged.
  const presetIds = ["left-1", "right-1", "left-2", "right-2", "left-3", "right-3"];
  const displayedPlacements = [
    ...presetIds.flatMap((id) => layout.placements.filter((p) => p.id === id)),
    ...layout.placements.filter((p) => !presetIds.includes(p.id) && p.id !== USER_PLACEMENT_ID),
  ];
  const update = (patch: Partial<ScanPlacement>) => setLayout((previous) => ({ ...previous,
    placements: previous.placements.map((p) => p.id === previous.selectedId ? { ...p, ...patch } : p) }));
  const select = (id: string) => { if (!busy) setLayout((previous) => ({ ...previous, selectedId: id })); };
  const vector = (label: string, key: "xyz" | "rpy" | "direction", factor: number, names: string[]) => (
    <div className="transform-section"><h3>{label}</h3><div className="transform-grid">
      {selected[key].map((value, i) => <label key={`${selected.id}-${key}-${i}`}><span>{names[i]}</span>
        <NumberField value={value * factor} disabled={busy} onCommit={(nextValue) => {
          const next = [...selected[key]] as Vector3Tuple; next[i] = nextValue / factor;
          if (key === "direction" && Math.hypot(...next) < 1e-9) { onError("스캔 방향 벡터는 0일 수 없습니다."); return; }
          update({ [key]: key === "direction" ? next.map((v) => v / Math.hypot(...next)) as Vector3Tuple : next });
        }} /></label>)}
    </div></div>
  );
  const save = async () => {
    setSaving(true);
    try { setSaved(await invoke<string>("save_scan_layout", { layout })); }
    catch (error) { onError(String(error)); }
    finally { setSaving(false); }
  };
  return <>
    <section className="viewer-panel" style={active && cellView ? undefined : { display: "none" }}>
      <div className="scene-titlebar"><span className="scene-tab active">스캔 배치</span><span className="scene-project">{model?.name ?? "모델"} · world_base</span></div>
      <div className="scene-body">
        {model && <MotionPreview model={model} trajectory={EMPTY_PATH} xyz={ZERO} rpy={ZERO}
          scanLayout={layout} onSelectPlacement={select} layers={layers} visualization={visualization}
          onReady={setReady} onError={(message) => { setModelError(message); onError(message); }} />}
        {!ready && <div className="empty-scene"><span>01</span><strong>{modelError ? "모델 불러오기 실패" : "갠트리·로봇 모델 불러오기"}</strong>
          <p>{modelError || "로컬 URDF 미리보기 · 장비 연결 없음"}</p>
          {modelError && <button className="secondary-action" style={{ pointerEvents: "auto" }} onClick={() => { setModel(null); setReload((v) => v + 1); }}>다시 불러오기</button>}
        </div>}
      </div>
    </section>
    <aside className="panel workstation-inspector" style={active ? undefined : { display: "none" }}>
      <div className="process-scroll"><div className="process-panel-content scan-layout-panel">
        <div className="process-panel-heading"><h2>스캔 준비</h2>
          <p>{cellView ? "프리셋을 선택하거나 사용자 설정으로 직접 지정하세요." : "저장된 스캔을 불러와 경로 생성에 사용합니다."}</p></div>
        <div className="segmented-control mode-tabs scan-view-tabs" aria-label="스캔 화면 선택">
          <button className={cellView ? "active" : ""} onClick={() => onCellView(true)}>부품 배치</button>
          <button className={!cellView ? "active" : ""} onClick={() => onCellView(false)}>스캔 파일</button>
        </div>
        {cellView && <>
          <div className="section-heading"><h3>프리셋 위치 <em>({displayedPlacements.length})</em></h3><span>왼쪽 +Y / 오른쪽 −Y</span></div>
          <div className="scan-presets">
            {displayedPlacements.map((placement) => <button key={placement.id} className={placement.id === layout.selectedId ? "active" : ""}
              title={placement.name} aria-pressed={placement.id === layout.selectedId}
              disabled={busy} onClick={() => select(placement.id)}>{placement.name.replace(" (+Y)", "").replace(" (−Y)", "")}</button>)}
          </div>
          <button type="button" className={`custom-placement-button ${userSelected ? "active" : ""}`}
            aria-pressed={userSelected} disabled={busy || (!hasUserPlacement && layout.placements.length >= 100)}
            title={hasUserPlacement ? "입력한 사용자 위치와 방향을 다시 선택" : "현재 위치와 방향을 복사하여 별도로 직접 설정"}
            onClick={() => setLayout(selectUserPlacement)}>
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.4" strokeLinecap="round" aria-hidden="true"><path d="M3 6h4m4 0h10M3 12h10m4 0h4M3 18h4m4 0h10" /><circle cx="9" cy="6" r="2" /><circle cx="15" cy="12" r="2" /><circle cx="9" cy="18" r="2" /></svg>
            사용자 설정
          </button>
          {userSelected && <p className="custom-placement-help">아래 위치·회전·스캔 방향을 직접 입력하세요. 프리셋과 별도로 보관되며, 처음에는 선택했던 배치 값을 가져옵니다.</p>}
          <div className="compact-actions">
            <button className="text-action" title="현재 배치를 복사하여 추가" disabled={busy || layout.placements.length >= 100} onClick={() => {
              const id = `custom-${crypto.randomUUID()}`;
              setLayout((previous) => ({ ...previous, selectedId: id,
                placements: [...previous.placements, { ...selected, id, name: `사용자 위치 ${previous.placements.length - 5}` }] }));
            }}>+ 배치 추가</button>
            <button className="text-action danger-text" disabled={busy || layout.placements.length <= 1} onClick={() => {
              if (!window.confirm(`${selected.name} 배치를 삭제할까요?`)) return;
              const placements = layout.placements.filter((p) => p.id !== selected.id);
              setLayout({ ...layout, selectedId: placements[0].id, placements });
            }}>선택 삭제</button>
          </div>
          {vector("부품 중심 위치 · mm", "xyz", 1000, ["X", "Y", "Z"])}
          {vector("부품 회전 · deg", "rpy", 180 / Math.PI, ["R", "P", "Y"])}
          {vector("스캔 방향 · 카메라 → 부품", "direction", 1, ["X", "Y", "Z"])}
          <details className="settings-details"><summary>배치 관리 및 좌표 안내</summary><div className="details-content">
            <label className="scan-name">배치 이름<input value={selected.name} disabled={busy} maxLength={80} onChange={(e) => update({ name: e.target.value })} /></label>
            <button className="secondary-action" disabled={busy} onClick={() => update({ direction: selected.direction.map((v) => -v) as Vector3Tuple })}>스캔 방향 반대로</button>
            <button className="secondary-action" disabled={busy} onClick={() => {
              if (window.confirm("사용자 배치를 지우고 기본 6개 프리셋으로 되돌릴까요?")) setLayout(defaultScanLayout());
            }}>기본 프리셋 복원</button>
            <p>world_base 기준, 회전은 Rz·Ry·Rx 순서입니다. 프리셋 1 → 3은 −X → +X 순서이며 방향 벡터는 정규화됩니다.</p>
            <p>사각형은 1200 × 900 mm, 카메라는 800 mm 이격된 예시입니다. 실제 촬영 자세가 아니며 IK·충돌·도달성은 검증하지 않습니다.</p>
            <p>불러온 스캔이나 경로 좌표에 자동 적용하지 않습니다. 설정은 앱에 자동 보관되며 파일 저장 시 scan_planning/log에 기록됩니다.</p>
          </div></details>
          {saved && <details className="completion-card"><summary>배치 저장 완료 · 위치 확인</summary><code>{saved}</code></details>}
        </>}
        {!cellView && importPanel}
        {sceneSettings}
        <div className="sticky-actions process-actions">
          {cellView && <button className="secondary-action" disabled={busy || saving} onClick={() => void save()}>배치 설정 파일 저장</button>}
          <button className="primary-action" disabled={busy} onClick={onNext}>다음: 도장 경로 생성</button>
        </div>
      </div></div>
    </aside>
  </>;
}
