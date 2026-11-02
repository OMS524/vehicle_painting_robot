import { processStages } from "./ProcessNavigation";
import type { ProcessStep } from "./ProcessNavigation";

interface Props {
  hasWorkspace: boolean;
  step: ProcessStep;
  scanName?: string;
  pathName?: string;
  rowCount: number;
  pointCount: number;
  dirty: boolean;
  stale: boolean;
  savedDirectory?: string;
  onStart: () => void;
}

const descriptions: Record<ProcessStep, string> = {
  1: "부품 위치와 스캔 방향을 정하고, 스캔 데이터를 불러옵니다.",
  2: "파라미터로 경로를 생성하거나 저장된 CSV 경로를 불러옵니다.",
  3: "포인트의 위치와 회전을 검토·수정하고 경로를 저장합니다.",
  4: "갠트리 안의 경로 위치를 확인하고 모션을 계산합니다.",
};

export function HomeScreen({ hasWorkspace, step, scanName, pathName, rowCount, pointCount,
  dirty, stale, savedDirectory, onStart }: Props) {
  const stage = processStages.find((item) => item.id === step)!;
  const state = stale ? "파라미터 변경 · 다시 생성 필요" : dirty ? "포인트 수정 · 재생성 필요"
    : savedDirectory ? "경로 저장 완료" : pointCount ? "경로 검토 가능"
      : scanName ? "스캔 데이터 준비됨" : hasWorkspace ? "부품 배치 준비 중" : "시작 전";

  return <section className="home-screen" aria-labelledby="home-title">
    <div className="home-content">
      <header className="home-heading">
        <div><span className="home-eyebrow">WORKSPACE / HOME</span><h1 id="home-title">작업의 시작, 워크스테이션</h1>
          <p>스캔 데이터부터 도장 경로, 모션 계획까지 한곳에서 준비하세요.</p></div>
        <span className="home-mode"><i />오프라인 준비 모드</span>
      </header>

      <div className="home-main-grid">
        <section className="home-start-card" aria-labelledby="home-start-title">
          <div className="home-start-heading"><span className="home-card-kicker">PAINTING WORKFLOW</span><span className="home-stage-count">04 STEPS</span></div>
          <div className="home-start-copy"><h2 id="home-start-title">{hasWorkspace ? "준비하던 작업을\n이어서 진행하세요." : "차량 도장 공정을\n여기서 시작하세요."}</h2>
            <p>{hasWorkspace ? "현재 설정과 경로를 유지한 채 마지막 작업 화면으로 돌아갑니다." : "부품 배치를 확인하고 필요한 데이터를 준비하는 것부터 시작합니다."}</p></div>
          <div className="home-start-footer">
            <button className="primary-action home-start-button" onClick={onStart}>
              <span>{hasWorkspace ? "작업 이어서" : "공정 시작"}</span>
              <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><path d="M4 12h16m-6-6 6 6-6 6" /></svg>
            </button>
            <p>작업 화면만 엽니다. 장비 이동이나 공정 실행은 시작하지 않습니다.</p>
          </div>
        </section>

        <section className="home-session-card" aria-labelledby="home-session-title">
          <div className="home-card-heading"><h2 id="home-session-title">현재 작업</h2><span>{hasWorkspace ? "이 실행 세션" : "대기"}</span></div>
          <div className={`home-session-state ${dirty || stale ? "needs-attention" : ""}`}><i />{state}</div>
          {hasWorkspace ? <>
            <div className="home-current-stage"><span>{String(step).padStart(2, "0")}</span><div><small>마지막 작업 화면</small><strong>{stage.title}</strong></div></div>
            <dl className="home-session-data">
              <div><dt>스캔 파일</dt><dd title={scanName}>{scanName ?? "아직 불러오지 않음"}</dd></div>
              <div><dt>도장 경로</dt><dd title={pathName}>{pathName ?? (pointCount ? "현재 생성한 경로" : "아직 준비하지 않음")}</dd></div>
              <div><dt>경로 데이터</dt><dd>{pointCount ? `${rowCount}개 경로 · ${pointCount.toLocaleString()}개 포인트` : "—"}</dd></div>
            </dl>
            {savedDirectory && <details className="home-saved-location"><summary>저장 위치 확인</summary><code>{savedDirectory}</code></details>}
          </> : <div className="home-empty-session">
            <svg viewBox="0 0 48 48" fill="none" stroke="currentColor" strokeWidth="1.5" aria-hidden="true"><path d="M8 14h13l4 5h15v20H8zM8 14V9h13l4 5h15v5" /><path d="M18 29h12m-6-6v12" /></svg>
            <strong>아직 시작한 작업이 없습니다.</strong>
            <p>공정을 시작하면 선택한 데이터와<br />작업 상태를 여기서 확인할 수 있습니다.</p>
          </div>}
          <p className="home-session-note">홈 이동 시 작업은 유지됩니다. 앱 종료 전에는 경로를 저장해 주세요.</p>
        </section>
      </div>

      <section className="home-workflow" aria-labelledby="home-workflow-title">
        <div className="home-workflow-heading"><h2 id="home-workflow-title">이렇게 진행합니다</h2><span>필요한 단계로 돌아가 설정을 검토하고 수정할 수 있습니다.</span></div>
        <ol className="home-workflow-list">
          {processStages.map((item) => <li key={item.id}>
            <div className="home-workflow-step"><span>{String(item.id).padStart(2, "0")}</span>
              <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.35" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">{item.icon}</svg></div>
            <h3>{item.title}</h3><p>{descriptions[item.id]}</p>
          </li>)}
        </ol>
      </section>
      <div className="home-footnote"><span>PAINTING WORKSTATION</span><p>현재 버전은 데이터 준비·경로 편집·모션 계획용이며 실제 장비를 제어하지 않습니다.</p></div>
    </div>
  </section>;
}
