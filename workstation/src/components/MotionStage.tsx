import { useEffect, useMemo, useRef, useState } from "react";
import type { ReactNode } from "react";
import { MotionPreview } from "./MotionPreview";
import { getMotionModel, getMotionStatus, openMotionRviz, startMotionJob, stopMotionProcess } from "../lib/motion";
import type { MotionModel, MotionRequest, MotionStatus } from "../lib/motion";
import type { CompletionSummary, LayerVisibility, TrajectoryDataset, Vector3Tuple, VisualizationSettings } from "../types";

interface Props {
  active: boolean;
  completion: CompletionSummary;
  trajectory: TrajectoryDataset;
  layers: LayerVisibility;
  visualization: VisualizationSettings;
  sceneSettings: ReactNode;
  onBusy: (busy: boolean) => void;
  onError: (message: string) => void;
}

const labels = { running: "실행 중", succeeded: "완료", failed: "실패", cancelled: "중단" };

export function MotionStage({ active, completion, trajectory, layers, visualization, sceneSettings, onBusy, onError }: Props) {
  const [model, setModel] = useState<MotionModel | null>(null);
  const [modelError, setModelError] = useState("");
  const [reload, setReload] = useState(0);
  const [ready, setReady] = useState(false);
  const [xyzMm, setXyzMm] = useState<Vector3Tuple>([0, 0, 0]);
  const [degrees, setDegrees] = useState<Vector3Tuple>([0, 0, 0]);
  const [velocity, setVelocity] = useState(420);
  const [acceleration, setAcceleration] = useState(1200);
  const [status, setStatus] = useState<MotionStatus>({ planning: null, rviz: null });
  const [launching, setLaunching] = useState(false);
  const [monitor, setMonitor] = useState(false);
  const [lastRequest, setLastRequest] = useState("");
  const reported = useRef(new Set<string>());
  const callbacks = useRef({ onBusy, onError });
  callbacks.current = { onBusy, onError };
  // Match planner CLI significant-digit precision. Zero means no placement
  // transform: the CSV is NOT centered on a guessed part origin automatically.
  const request = useMemo<MotionRequest>(() => ({
    modelKey: model?.modelKey ?? "",
    sourceDirectory: completion.outputDirectory,
    xyz: xyzMm.map((v) => Number((v / 1000).toPrecision(6))) as Vector3Tuple,
    rpy: degrees.map((v) => Number((v * Math.PI / 180).toPrecision(7))) as Vector3Tuple,
    velocity: velocity / 1000,
    acceleration: acceleration / 1000,
  }), [completion.outputDirectory, xyzMm, degrees, velocity, acceleration, model?.modelKey]);
  const fingerprint = JSON.stringify(request);
  const planning = launching || status.planning?.status === "running";
  const currentResult = status.planning?.status === "succeeded" && lastRequest === fingerprint;

  useEffect(() => {
    if (!active || model) return;
    setModelError("");
    let disposed = false;
    void getMotionModel().then((result) => {
      if (disposed) return;
      setModel(result);
      setVelocity(result.velocity * 1000);
      setAcceleration(result.acceleration * 1000);
    }).catch((reason: unknown) => {
      if (!disposed) { setModelError(String(reason)); callbacks.current.onError(String(reason)); }
    });
    return () => { disposed = true; };
  }, [active, reload]);

  useEffect(() => {
    callbacks.current.onBusy(planning);
  }, [planning]);

  useEffect(() => {
    if (!monitor) return;
    let disposed = false;
    let timer: ReturnType<typeof setTimeout>;
    const poll = async () => {
      try {
        const next = await getMotionStatus();
        if (disposed) return;
        setStatus(next);
        for (const job of [next.planning, next.rviz]) {
          if (job?.status === "failed" && !reported.current.has(job.directory)) {
            reported.current.add(job.directory);
            callbacks.current.onError(job.log || `외부 프로세스 실패: ${job.directory}`);
          }
        }
        if (next.planning?.status !== "running" && next.rviz?.status !== "running") {
          setMonitor(false);
          return;
        }
      } catch (reason: unknown) {
        if (!disposed) callbacks.current.onError(String(reason));
        // Keep the busy guard until an actual terminal status is known.
      }
      if (!disposed) timer = setTimeout(() => void poll(), 1000);
    };
    void poll();
    return () => { disposed = true; clearTimeout(timer); };
  }, [monitor]);

  useEffect(() => () => {
    callbacks.current.onBusy(false);
    void stopMotionProcess("rviz").catch(() => {});
  }, []);

  useEffect(() => {
    if (lastRequest && lastRequest !== fingerprint && status.rviz?.status === "running") {
      void stopMotionProcess("rviz").catch((reason: unknown) => callbacks.current.onError(String(reason)));
    }
  }, [fingerprint, lastRequest, status.rviz?.status]);

  const generate = async () => {
    setLaunching(true);
    setLastRequest("");
    try {
      await startMotionJob(request);
      setStatus({ planning: { status: "running", directory: "", log: "모션 계획을 시작했습니다.", result: null }, rviz: null });
      setLastRequest(fingerprint);
      setMonitor(true);
    } catch (reason: unknown) { onError(String(reason)); }
    finally { setLaunching(false); }
  };

  const openRviz = async () => {
    setLaunching(true);
    try {
      await openMotionRviz();
      setMonitor(true);
    } catch (reason: unknown) { onError(String(reason)); }
    finally { setLaunching(false); }
  };

  const vectorInputs = (title: string, values: Vector3Tuple, names: string[], set: (next: Vector3Tuple) => void) => (
    <div className="transform-section">
      <h3>{title}</h3>
      <div className="transform-grid">
        {values.map((value, index) => <label key={index}>
          <span>{names[index]}</span>
          <input type="number" step={1} value={value} disabled={planning} onChange={(event) => {
            const number = Number(event.target.value);
            if (!Number.isFinite(number)) return;
            const next = [...values] as Vector3Tuple;
            next[index] = number;
            set(next);
          }} />
        </label>)}
      </div>
    </div>
  );

  return <>
    <section className="viewer-panel" style={active ? undefined : { display: "none" }}>
      <div className="scene-titlebar"><span className="scene-tab active">모션 배치</span><span className="scene-project">{model?.name ?? "모델"} · world_base</span></div>
      <div className="scene-body">
        {model && <MotionPreview model={model} trajectory={trajectory} xyz={request.xyz} rpy={request.rpy}
          layers={layers} visualization={visualization} onReady={setReady}
          onError={(message) => { setModelError(message); onError(message); }} />}
        {!ready && <div className="empty-scene"><span>04</span><strong>{modelError ? "모델을 불러오지 못했습니다" : "갠트리·로봇 모델 불러오기"}</strong><p>{modelError || "로컬 URDF와 메시를 읽습니다. 모션 계산은 시작하지 않습니다."}</p>
          {modelError && <button className="secondary-action" style={{ pointerEvents: "auto" }} onClick={() => { setModel(null); setReload((n) => n + 1); }}>다시 불러오기</button>}</div>}
      </div>
    </section>
    <aside className="panel workstation-inspector" style={active ? undefined : { display: "none" }}>
      <div className="process-scroll"><div className="process-panel-content motion-panel">
        <div className="process-panel-heading"><h2>경로 배치 및 계산</h2>
          <p>셀 안의 경로 위치를 확인한 뒤 계산하세요.</p></div>
        <div className="preview-notice"><i />오프라인 계산 · 충돌 검사 없음</div>
        <div className="stage-information">
          <div><span>기준 좌표계</span><strong>world_base · Z 위쪽</strong></div>
          <div><span>갠트리</span><strong>{model?.name ?? "불러오는 중"}</strong></div>
          <div><span>경로</span><strong>{trajectory.rows.length}개 경로 · {trajectory.pointCount.toLocaleString()}개 포인트</strong></div>
        </div>
        {vectorInputs("경로 전체 이동 · mm", xyzMm, ["X", "Y", "Z"], setXyzMm)}
        {vectorInputs("경로 전체 회전 · deg", degrees, ["R", "P", "Y"], setDegrees)}
        <div className="transform-section motion-timing"><h3>모션 시간 프로파일</h3>
          <label>속도 · mm/s<input type="number" min="1" value={velocity} disabled={planning} onChange={(e) => setVelocity(Number(e.target.value))} /></label>
          <label>가속도 · mm/s²<input type="number" min="1" value={acceleration} disabled={planning} onChange={(e) => setAcceleration(Number(e.target.value))} /></label>
        </div>
        <details className="settings-details"><summary>좌표계 및 계산 안내</summary><div className="details-content">
          <p>CSV 원점 기준 회전 후 world_base 방향으로 이동합니다. p′ = Rz·Ry·Rx·p + t. 0이면 원본 좌표 그대로입니다.</p>
          <p>실제 장비를 제어하지 않습니다. 부품 표면/충돌 검사 없이 고정 후퇴 모드(기본 30 mm)로 경로 사이를 연결합니다.</p>
          <button className="secondary-action" disabled={planning} onClick={() => { setXyzMm([0, 0, 0]); setDegrees([0, 0, 0]); }}>배치 초기화</button>
        </div></details>
        {status.planning?.result && <details className="completion-card"><summary>모션 결과 · 저장 위치</summary><code>{status.planning.result.runDirectory}</code></details>}
        <details className="motion-details settings-details"><summary>실행 로그</summary><pre>{[status.planning?.log, status.rviz?.log].filter(Boolean).join("\n") || "아직 실행하지 않았습니다."}</pre></details>
        <details className="motion-details settings-details"><summary>원본 · 모델 · 실행 환경</summary><div className="details-content">
          <p>원본 경로 (변경하지 않음)</p><code>{completion.outputDirectory}</code>
          <p>URDF</p><code>{model?.urdfPath}</code><p>모션 Python</p><code>{model?.python}</code>
          <p>초기 관절값: gantry X/Y (m), joint 1–6 (deg)</p><code>{model?.initialJoint.join(", ")}</code>
          <button className="secondary-action" disabled={planning} onClick={() => {
            setReady(false); setModel(null); setReload((n) => n + 1);
          }}>모델 / 설정 다시 불러오기</button>
        </div></details>
        {sceneSettings}
        <div className="sticky-actions">
          {status.planning && <div className={`edit-state ${currentResult ? "clean" : "dirty"}`}>
            <i /><span>모션 계획: {labels[status.planning.status]}{status.planning.status === "succeeded" && !currentResult ? " · 설정 변경됨" : ""}</span>
          </div>}
          <div className="process-actions">
            {planning ? <button className="secondary-action" onClick={() => void stopMotionProcess("planning").catch((e: unknown) => onError(String(e)))}>계산 중단</button>
              : <button className="primary-action" disabled={!ready || !Number.isFinite(velocity) || !Number.isFinite(acceleration) || velocity <= 0 || acceleration <= 0} onClick={() => void generate()}>모션 생성</button>}
            {status.rviz?.status === "running"
              ? <button className="secondary-action" onClick={() => void stopMotionProcess("rviz").catch((e: unknown) => onError(String(e)))}>RViz 닫기</button>
              : <button className="secondary-action" disabled={!currentResult || planning} onClick={() => void openRviz()}>RViz 열기</button>}
          </div>
        </div>
      </div></div>
    </aside>
  </>;
}
